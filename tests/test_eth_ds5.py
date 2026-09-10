"""Offline contracts for ETH intake: no network, GPU or dataset required."""
import hashlib
import importlib.util
import math
import io
from pathlib import Path
import tempfile
import unittest
import sys
import json
import contextlib
from unittest import mock

SPEC = importlib.util.spec_from_file_location("eth_ds5", Path(__file__).resolve().parents[1] / "tools/prepare_eth_ds5.py")
ETH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ETH)
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from prepare_eth_ds5_frames import validate_video
import prepare_eth_ds5_frames as FRAMES
from extract_eth_ds5 import video_member
import verify_eth_ds5_video as VERIFY
import prepare_eth_ds5_review as REVIEW
import check_eth_ds5_reprojection as REPROJ
import select_eth_ds5_alignment_frames as SELECT


class Response(io.BytesIO):
    def __init__(self, data, status=200, headers=None):
        super().__init__(data)
        self.status, self.headers = status, headers or {}


class IntakeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.data = b"abcdef"
        self.entry = {"path": "file.txt", "size": 6,
                      "sha": hashlib.sha1(b"blob 6\0abcdef").hexdigest()}

    def test_download_and_verified_skip(self):
        sha = ETH.download(self.root, self.entry, lambda *a, **k: Response(self.data))
        self.assertEqual(sha, hashlib.sha256(self.data).hexdigest())
        self.assertEqual(ETH.download(self.root, self.entry, None), sha)

    def test_corrupt_existing_is_preserved(self):
        (self.root / "file.txt").write_bytes(b"xxxxxx")
        with self.assertRaises(ValueError): ETH.download(self.root, self.entry, None)
        self.assertEqual((self.root / "file.txt").read_bytes(), b"xxxxxx")

    def test_valid_resume(self):
        (self.root / "file.txt.partial").write_bytes(b"abc")
        ETH.download(self.root, self.entry, lambda *a, **k: Response(b"def", 206, {"Content-Range": "bytes 3-5/6"}))
        self.assertEqual((self.root / "file.txt").read_bytes(), self.data)

    def test_ignored_range_preserves_partial(self):
        (self.root / "file.txt.partial").write_bytes(b"abc")
        with self.assertRaises(ValueError):
            ETH.download(self.root, self.entry, lambda *a, **k: Response(self.data))
        self.assertEqual((self.root / "file.txt.partial").read_bytes(), b"abc")

    def test_wrong_range(self):
        with self.assertRaises(ValueError):
            ETH.download(self.root, self.entry, lambda *a, **k: Response(self.data, 206, {"Content-Range": "bytes 1-6/7"}))

    def test_wrong_blob_not_promoted(self):
        with self.assertRaises(ValueError):
            ETH.download(self.root, self.entry, lambda *a, **k: Response(b"xxxxxx"))
        self.assertFalse((self.root / "file.txt").exists())

    def test_incomplete_resumable(self):
        with self.assertRaises(ValueError):
            ETH.download(self.root, self.entry, lambda *a, **k: Response(b"abc"))
        self.assertEqual((self.root / "file.txt.partial").read_bytes(), b"abc")

    def test_traversal(self):
        for p in ("../x", "/x", "x/../../a", "x\\a"):
            with self.assertRaises(ValueError): ETH.safe_path(self.root, p)

    def test_symlink_escape(self):
        (self.root / "escape").symlink_to(self.root.parent)
        with self.assertRaises(ValueError): ETH.safe_path(self.root, "escape/x")

    def test_transport_retry(self):
        with mock.patch.object(ETH, "download", side_effect=[TimeoutError("interrupted"), "sha"]), mock.patch.object(ETH.time, "sleep"):
            self.assertEqual(ETH.fetch_verified(self.root, self.entry), "sha")

    def test_hash_failure_not_retried(self):
        with mock.patch.object(ETH, "download", side_effect=ValueError("Git blob mismatch")) as fetch:
            with self.assertRaises(ValueError): ETH.fetch_verified(self.root, self.entry)
            self.assertEqual(fetch.call_count, 1)


class MetadataTest(unittest.TestCase):
    def setUp(self):
        self.pose = [[t, 3., 4., 0., 0., 0., 0., .01, .01, .01, 0.] for t in (1., 2., 3.)]
        self.frames = [[float(i+1), t] for i, t in enumerate((0., 1., 1.5, 2., 3., 4.))]

    def test_overlap_not_full_video(self):
        r = ETH.audit(self.pose, self.frames, [0., 0., 0.])
        self.assertEqual(r["temporal_overlap_frames"], 4)
        self.assertEqual(r["raw_slant_range_m_not_quality_filtered"], [5., 5.])
        self.assertFalse(r["range_model_fit"])
        self.assertTrue(r["blockers"])

    def test_duplicate_timestamp_rejected(self):
        self.pose[1][0] = 1.
        with self.assertRaises(ValueError): ETH.audit(self.pose, self.frames, [0., 0., 0.])

    def test_invalid_frame_id(self):
        self.frames[0][0] = .5
        with self.assertRaises(ValueError): ETH.audit(self.pose, self.frames, [0., 0., 0.])

    def test_queue_never_fabricates_gt(self):
        queue = ETH.annotation_queue(self.pose, self.frames, 1)
        self.assertEqual(len(queue), 4)
        for r in queue:
            self.assertIsNone(r["box_xyxy"])
            self.assertFalse(r["measurement_eligible"])
            self.assertFalse(r["identity_verified"])
            self.assertEqual(r["opencv_index"], r["frame_id"]-1)

    def test_invalid_stride(self):
        with self.assertRaises(ValueError): ETH.annotation_queue(self.pose, self.frames, 0)

    def test_nonfinite_table(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "table.txt"
            p.write_text("header\n1 nan\n")
            with self.assertRaises(ValueError): ETH.numeric_table(p, 2)


class VideoTest(unittest.TestCase):
    def test_matching_metadata_is_not_calibration_proof(self):
        result = validate_video({"codec_type": "video", "nb_frames": "10", "width": 1920, "height": 1080, "avg_frame_rate": "30000/1001"},
                                10, {"resolution": [1920, 1080], "fps": 29.970030})
        self.assertTrue(result["dimensions_match_calibration_candidate"])
        self.assertFalse(result["calibration_validated"])

    def test_mismatched_count_rejected(self):
        with self.assertRaises(ValueError):
            validate_video({"codec_type": "video", "nb_frames": "9"}, 10, {})

    def test_mismatched_count_marks_alignment_unresolved_when_allowed(self):
        r = validate_video({"codec_type": "video", "nb_frames": "11", "width": 1, "height": 1, "avg_frame_rate": "30/1"},
                           10, {"resolution": [1, 1], "fps": 30.}, allow_count_mismatch=True)
        self.assertEqual(r["frame_index_alignment"], "UNRESOLVED_count_mismatch")
        self.assertFalse(r["calibration_validated"])

    def test_unknown_count_rejected(self):
        with self.assertRaises(ValueError): validate_video({"codec_type": "video"}, 10, {})

    def test_audio_rejected(self):
        with self.assertRaises(ValueError): validate_video({"codec_type": "audio"}, 10, {})

    def test_fps_mismatch(self):
        with self.assertRaises(ValueError):
            validate_video({"codec_type": "video", "nb_frames": "10", "avg_frame_rate": "60/1"}, 10, {"fps": 30.})

    def test_real_decode_pilot_stays_unannotated(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest("OpenCV/numpy unavailable")
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            video = root / "synthetic.avi"
            writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 30., (64, 48))
            self.assertTrue(writer.isOpened())
            for i in range(4): writer.write(np.full((48, 64, 3), i * 50, np.uint8))
            writer.release()
            calibration = root / "calibration.json"
            calibration.write_text(json.dumps({"resolution": [64, 48], "fps": 30.}))
            receipt = root / "intake.json"
            receipt.write_text(json.dumps({"source_commit": ETH.COMMIT, "video_timestamp_rows": 4,
                "files": [{"path": "calibration/sony5100/sony5100.json", "sha256": ETH.digests(calibration)[1]}]}))
            queue = root / "queue.json"
            queue.write_text(json.dumps({"source_commit": ETH.COMMIT, "frames": [
                {"frame_id": 2, "opencv_index": 1, "box_xyxy": None,
                 "measurement_eligible": False, "identity_verified": False}]}))
            stream = {"streams": [{"codec_type": "video", "nb_frames": "4", "width": 64, "height": 48,
                                   "avg_frame_rate": "30/1"}]}

            def run(output, *extra):
                args = ["prepare", "--video", str(video), "--queue", str(queue), "--intake-receipt", str(receipt),
                        "--calibration", str(calibration), "--output", str(root / output), "--ffprobe", "mock"] + list(extra)
                with mock.patch.object(sys, "argv", args), mock.patch.object(
                        FRAMES.subprocess, "check_output", return_value=json.dumps(stream)), contextlib.redirect_stdout(io.StringIO()):
                    FRAMES.main()
                return json.loads((root / output / "receipt.json").read_text())

            result = run("output")
            self.assertFalse(result["calibration_validated"])
            self.assertIsNone(result["frames"][0]["box_xyxy"])
            self.assertEqual(result["container_index_offset"], -1)
            self.assertEqual(result["frames"][0]["container_index"], 1)
            image = cv2.imread(str(root / "output/frame_000002.png"))
            self.assertAlmostEqual(float(image.mean()), 50., delta=3.)

            # The offset is a hypothesis, not a constant: offset 0 must render the NEXT container frame.
            shifted = run("output_offset0", "--index-offset", "0")
            self.assertEqual(shifted["container_index_offset"], 0)
            self.assertEqual(shifted["container_index_rule"], "container_index = frame_id + 0")
            self.assertEqual(shifted["frames"][0]["container_index"], 2)
            self.assertEqual(shifted["frames"][0]["queue_declared_opencv_index"], 1)
            image = cv2.imread(str(root / "output_offset0/frame_000002.png"))
            self.assertAlmostEqual(float(image.mean()), 100., delta=3.)

    def test_index_offset_outside_the_video_is_refused(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest("OpenCV/numpy unavailable")
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            video = root / "synthetic.avi"
            writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 30., (64, 48))
            for i in range(4):
                writer.write(np.full((48, 64, 3), i * 50, np.uint8))
            writer.release()
            calibration = root / "calibration.json"
            calibration.write_text(json.dumps({"resolution": [64, 48], "fps": 30.}))
            receipt = root / "intake.json"
            receipt.write_text(json.dumps({"source_commit": ETH.COMMIT, "video_timestamp_rows": 4,
                "files": [{"path": "calibration/sony5100/sony5100.json", "sha256": ETH.digests(calibration)[1]}]}))
            queue = root / "queue.json"
            queue.write_text(json.dumps({"source_commit": ETH.COMMIT, "frames": [
                {"frame_id": 4, "opencv_index": 3, "box_xyxy": None,
                 "measurement_eligible": False, "identity_verified": False}]}))
            stream = {"streams": [{"codec_type": "video", "nb_frames": "4", "width": 64, "height": 48,
                                   "avg_frame_rate": "30/1"}]}
            args = ["prepare", "--video", str(video), "--queue", str(queue), "--intake-receipt", str(receipt),
                    "--calibration", str(calibration), "--output", str(root / "out"), "--ffprobe", "mock",
                    "--index-offset", "0"]
            with mock.patch.object(sys, "argv", args), mock.patch.object(
                    FRAMES.subprocess, "check_output", return_value=json.dumps(stream)), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(ValueError):
                    FRAMES.main()


class AlignmentSelectionTest(unittest.TestCase):
    def setUp(self):
        self.values = {100: 6.0, 110: 5.5, 400: 4.5, 405: 4.4, 900: 1.0}
        self.ranges = {f: 60.0 for f in self.values}
        self.ranges[110] = 10.0

    def test_prefers_motion_keeps_separation_and_respects_range(self):
        chosen = SELECT.select(self.values, self.ranges, 4.0, 25.0, 30, 10)
        self.assertEqual(chosen, [100, 400])          # 110 too close AND too near, 405 too close, 900 too slow
        self.assertEqual(SELECT.select(self.values, self.ranges, 4.0, 25.0, 1, 10), [100, 400, 405])

    def test_out_of_view_frames_are_excluded(self):
        in_frame = {100: None, 400: [10., 10.], 405: [20., 20.], 110: [1., 1.], 900: [1., 1.]}
        self.assertEqual(SELECT.select(self.values, self.ranges, 4.0, 25.0, 30, 10, in_frame), [400])

    def test_invalid_arguments(self):
        with self.assertRaises(ValueError): SELECT.select(self.values, self.ranges, 4.0, 25.0, 0, 10)
        with self.assertRaises(ValueError): SELECT.select(self.values, self.ranges, 4.0, 25.0, 30, 0)

    def test_level_camera_axis_and_degeneracy(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy unavailable")
        R = SELECT.level_camera(30., 20.)
        axis = R.T @ np.array([0., 0., 1.])
        self.assertAlmostEqual(math.degrees(math.atan2(axis[1], axis[0])), 30., places=6)
        self.assertAlmostEqual(math.degrees(math.asin(axis[2])), 20., places=6)
        self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=9)
        self.assertAlmostEqual(float(R[0][2]), 0.0, places=9)   # roll-free: image x axis stays horizontal
        with self.assertRaises(ValueError): SELECT.level_camera(0., 90.)

    def test_predicted_pixel_rejects_behind_and_out_of_frame(self):
        K = [[1500., 0., 960.], [0., 1500., 540.], [0., 0., 1.]]
        R = SELECT.level_camera(0., 0.)
        self.assertIsNone(SELECT.predicted_pixel(R, [0., 0., 0.], [-50., 0., 0.], K, [0.] * 5, [1920, 1080], 0))
        centre = SELECT.predicted_pixel(R, [0., 0., 0.], [50., 0., 0.], K, [0.] * 5, [1920, 1080], 0)
        self.assertAlmostEqual(centre[0], 960., places=6)
        self.assertIsNone(SELECT.predicted_pixel(R, [0., 0., 0.], [50., 40., 0.], K, [0.] * 5, [1920, 1080], 0))
        self.assertIsNone(SELECT.predicted_pixel(R, [0., 0., 0.], [50., 0., 0.], K, [0.] * 5, [1920, 1080], 1000))


class ExtractionTest(unittest.TestCase):
    def test_single_video(self):
        self.assertEqual(video_member("header\n----------\nPath = cam0.mp4\nSize = 123\n")["Size"], "123")

    def test_traversal_rejected(self):
        with self.assertRaises(ValueError): video_member("----------\nPath = ../cam0.mp4\nSize = 123\n")

    def test_symlink_rejected(self):
        with self.assertRaises(ValueError): video_member("----------\nPath = cam0.mp4\nSize = 123\nSymbolic Link = /etc/passwd\n")

    def test_duplicate_rejected(self):
        with self.assertRaises(ValueError):
            video_member("----------\nPath = cam0.mp4\nSize = 123\n\nPath = sub/cam0.mp4\nSize = 123\n")


class TimestampTest(unittest.TestCase):
    PERIOD = 1001 / 30000

    def test_affine_hypothesis_detected_and_raw_rejected(self):
        pts = [i * self.PERIOD for i in range(2000)]
        published = [1.01 * p + 5. for p in pts]
        r = VERIFY.timestamp_consistency(pts, published, 1.01, 5., self.PERIOD)
        self.assertTrue(r["alignments"]["packet_offset_0"]["synchronised_affine"]["within_half_frame"])
        self.assertFalse(r["alignments"]["packet_offset_0"]["raw_camera_time"]["within_half_frame"])
        self.assertNotIn("packet_offset_1", r["alignments"])

    def test_extra_container_frame_reports_both_alignments(self):
        pts = [i * self.PERIOD for i in range(11)]
        published = [1.0 * p + 5. for p in pts[1:]]
        r = VERIFY.timestamp_consistency(pts, published, 1.0, 5., self.PERIOD)
        self.assertEqual(sorted(r["alignments"]), ["packet_offset_0", "packet_offset_1"])
        self.assertAlmostEqual(r["alignments"]["packet_offset_1"]["synchronised_affine"]["offset_first_row_s"], 0.)
        self.assertAlmostEqual(r["alignments"]["packet_offset_0"]["synchronised_affine"]["offset_first_row_frames"], 1.)

    def test_published_fit_identifies_generating_scale(self):
        sync = {0: (1.000004, 10.), 1: (1.00003, 7.)}
        ids = list(range(1, 501))
        published = [1.00003 * self.PERIOD * (i + 2) + 10. for i in ids]
        fit = VERIFY.published_fit(ids, published, self.PERIOD, sync)
        self.assertEqual(fit["closest_camera_scale"], 1)
        self.assertAlmostEqual(fit["intercept_minus_cam0_shift_frames"], 2., places=3)
        self.assertLess(fit["max_abs_residual_s"], 1e-9)

    def test_provenance_detects_split_scale_and_shift_rows(self):
        """The real file's slope is cam1's scale while its origin only fits cam0's shift."""
        sync = {0: (1.000004517, 10.48625553), 1: (1.000031087, 7.72945441), 2: (0.997549813, 9.80204540)}
        slope, intercept = 0.033367703956, 10.551275280
        r = VERIFY.stamp_provenance(slope, intercept, sync, self.PERIOD)
        self.assertEqual(r["scale_matches_camera"], 1)
        self.assertEqual(r["origin_matches_camera"], 0)
        self.assertFalse(r["single_row_explains_file"])
        cam0 = r["cameras"]["0"]
        self.assertEqual(cam0["nearest_integer_origin"], 2)
        self.assertLess(cam0["origin_distance_to_integer"], 0.06)
        self.assertLess(abs(cam0["shift_revision_needed_s"]), 0.002)

    def test_provenance_accepts_a_self_consistent_table(self):
        sync = {0: (1.000004517, 10.48625553), 1: (1.000031087, 7.72945441)}
        slope = 1.000004517 * self.PERIOD
        intercept = 10.48625553 + 2 * slope
        r = VERIFY.stamp_provenance(slope, intercept, sync, self.PERIOD)
        self.assertTrue(r["single_row_explains_file"])
        self.assertEqual(r["scale_matches_camera"], 0)
        self.assertEqual(r["cameras"]["0"]["nearest_integer_origin"], 2)

    def test_alignment_candidates_stay_unresolved(self):
        edits = [{"handler": "soun", "edits": [{"media_time": 0, "media_time_frames": 0.}]},
                 {"handler": "vide", "edits": [{"media_time": 1001, "media_time_frames": 1.0}]}]
        r = VERIFY.index_alignment_candidates(edits, 20970, 20969)
        self.assertEqual(r["missing_rows"], 1)
        self.assertFalse(r["resolved"])
        supported = [c for c in r["candidates"] if c["supported_by_edit_list"]]
        self.assertEqual([c["container_index_of_frame_id"] for c in supported], ["frame_id"])
        self.assertEqual(len(r["candidates"]), 3)

    def test_alignment_candidate_unsupported_without_a_video_edit(self):
        r = VERIFY.index_alignment_candidates([{"handler": "vide", "edits": []}], 20970, 20969)
        self.assertIsNone(r["video_edit_list_frames"])
        self.assertFalse(any(c["supported_by_edit_list"] for c in r["candidates"]))

    def test_edit_frames_use_the_track_timescale_not_an_assumed_rate(self):
        """A 25 fps track at a 12800 timescale: 512 media units is one frame, not 512/1001."""
        tracks = [{"handler": "vide", "timescale": 12800, "sample_duration": 512,
                   "edits": [{"media_time": 512, "media_time_frames": 512 / 512}]}]
        self.assertEqual(VERIFY.video_edit_frames(tracks), 1.0)

    def test_video_edit_frames_ignores_other_tracks(self):
        tracks = [{"handler": "soun", "edits": [{"media_time": 0, "media_time_frames": 0.}]},
                  {"handler": "vide", "edits": [{"media_time": 1001, "media_time_frames": 1.0}]}]
        self.assertEqual(VERIFY.video_edit_frames(tracks), 1.0)
        self.assertIsNone(VERIFY.video_edit_frames([{"handler": "soun", "edits": [{"media_time": 5, "media_time_frames": 5.}]}]))

    def test_edit_lists_parsed_per_track_from_the_real_file(self):
        video = Path("/home/fair/workspaces/aerial_gym_ws/datasets/eth_ds5_cam0_extracted/cam0.mp4")
        if not video.is_file():
            self.skipTest("extracted cam0.mp4 unavailable")
        tracks = VERIFY.edit_list_offsets(video)
        self.assertEqual([t["handler"] for t in tracks], ["vide", "soun", "meta"])
        self.assertEqual(VERIFY.video_edit_frames(tracks), 1.0)
        self.assertEqual([t["edits"][0]["media_time"] for t in tracks if t["handler"] != "vide"], [0, 0])

    def test_calibration_candidate_never_validated_by_metadata(self):
        r = VERIFY.calibration_candidate_check({"width": 1920, "height": 1080, "avg_frame_rate": "30000/1001"},
                                               {"encoder": "AVC Coding", "com.apple.quicktime.model": "X"},
                                               {"resolution": [1920, 1080], "fps": 29.97003})
        self.assertTrue(r["resolution_matches"] and r["fps_matches"])
        self.assertFalse(r["calibration_validated"])
        self.assertIn("com.apple.quicktime.model", r["camera_model_tags"])


class ReviewTest(unittest.TestCase):
    def setUp(self):
        self.pose = [[t, t, 2 * t, 3., 0., 0., 170. + 20. * t, .01, .01, .01, 1.] for t in (0., 1., 2., 5.)]

    def test_interpolation_and_yaw_wrap(self):
        gt = REVIEW.interpolate_pose(self.pose, 0.5, max_gap_s=1.5)
        self.assertAlmostEqual(gt["xyz_m"][1], 1.)
        self.assertAlmostEqual(gt["speed_mps"], (1 + 4) ** .5)
        # 170 -> 190 wraps through the branch cut; the result is the same angle, normalised into range
        yaw = gt["rpy_deg_uninterpreted"][2]
        self.assertAlmostEqual(REVIEW.wrap_deg(yaw - 180.), 0.)
        self.assertTrue(-180. <= yaw <= 180.)

    def test_interpolated_angles_stay_in_range(self):
        wrapping = [[0., 0, 0, 0, 350., -170., 359.9, 0, 0, 0, 0],
                    [0.1, 0, 0, 0, -350., 170., 0.1, 0, 0, 0, 0]]
        rpy = REVIEW.interpolate_pose(wrapping, 0.05, max_gap_s=1)["rpy_deg_uninterpreted"]
        for angle in rpy:
            self.assertTrue(-180. <= angle <= 180., angle)
        self.assertAlmostEqual(REVIEW.wrap_deg(rpy[2] - 0.), 0.)

    def test_no_extrapolation_or_long_gap(self):
        self.assertIsNone(REVIEW.interpolate_pose(self.pose, -0.1, max_gap_s=1.5))
        self.assertIsNone(REVIEW.interpolate_pose(self.pose, 5.1, max_gap_s=1.5))
        self.assertIsNone(REVIEW.interpolate_pose(self.pose, 0.5))  # default 0.5 s gap limit refuses 1 s brackets
        self.assertIsNone(REVIEW.interpolate_pose(self.pose, 3., max_gap_s=1.5))
        self.assertIsNotNone(REVIEW.interpolate_pose(self.pose, 3., max_gap_s=5.))

    def test_bearing(self):
        g = REVIEW.bearing_from_camera([0., 0., 0.], [0., 3., 4.])
        self.assertAlmostEqual(g["slant_range_m"], 5.)
        self.assertAlmostEqual(g["azimuth_deg_from_east_ccw"], 90.)
        self.assertAlmostEqual(g["elevation_deg"], 53.130102354)

    def test_motion_candidates_find_only_moving_blob(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy unavailable")
        prev, cur, nxt = [np.full((60, 80, 3), 100, np.uint8) for _ in range(3)]
        for img in (prev, cur, nxt):
            img[5:15, 5:15] = 0  # static dark square must not be a candidate
        cur[30:34, 40:44] = 255
        boxes, moving = REVIEW.motion_candidates(prev, cur, nxt)
        self.assertEqual(len(boxes), 1)
        x1, y1, x2, y2 = boxes[0]["xyxy"]
        self.assertTrue(x1 <= 40 and y1 <= 30 and x2 >= 44 and y2 >= 34)
        panel = REVIEW.render_panel(cur, boxes, ["a", "b"])
        self.assertGreater(panel.shape[0], cur.shape[0])


class DistortionTest(unittest.TestCase):
    def test_fold_back_radius_of_the_real_calibration(self):
        limit = REPROJ.valid_distortion_radius([-0.011232359677, 0.045931232417, 0.000263868094,
                                                -0.001253638454, -0.151703077572])
        self.assertTrue(1.0 < limit < 1.1)          # image corner is at 0.713, comfortably inside
        r = limit * 0.999
        self.assertGreater(self._radial(r, -0.011232359677, 0.045931232417, -0.151703077572),
                           self._radial(r * 0.99, -0.011232359677, 0.045931232417, -0.151703077572))
        beyond = limit * 1.3
        self.assertLess(self._radial(beyond, -0.011232359677, 0.045931232417, -0.151703077572),
                        self._radial(limit, -0.011232359677, 0.045931232417, -0.151703077572))

    @staticmethod
    def _radial(r, k1, k2, k3):
        return r * (1 + k1 * r ** 2 + k2 * r ** 4 + k3 * r ** 6)

    def test_no_distortion_never_folds(self):
        self.assertEqual(REPROJ.valid_distortion_radius([0., 0., 0., 0., 0.]), float("inf"))


class ReprojectionTest(unittest.TestCase):
    def setUp(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest("OpenCV/numpy unavailable")
        self.np, self.cv2 = np, cv2
        self.K = [[1500., 0., 960.], [0., 1500., 540.], [0., 0., 1.]]
        self.dist = [-0.01, 0.04, 0.0002, -0.001, -0.15]
        self.camera = [14.84, 6.939, 1.494]
        R0, _ = cv2.Rodrigues(np.array([0.3, -1.2, 0.1]))
        self.R = R0
        rng = np.random.default_rng(0)
        self.world = [self.camera + R0.T @ np.array([x, y, z]) for x, y, z in
                      rng.uniform([-15., -8., 30.], [15., 8., 100.], size=(12, 3))]
        self.pixels = REPROJ.reproject(self.R, self.camera, self.world, self.K, self.dist)

    def test_recovers_rotation_and_centre(self):
        rays_c = REPROJ.camera_rays(self.pixels, self.K, self.dist)
        rays_w = self.np.asarray(self.world) - self.np.asarray(self.camera)
        rays_w /= self.np.linalg.norm(rays_w, axis=1, keepdims=True)
        R, angular = REPROJ.fit_rotation(rays_c, rays_w)
        self.assertLess(self.np.abs(R - self.R).max(), 1e-6)
        self.assertLess(angular.max(), 1e-4)
        centre = REPROJ.pnp_centre(self.world, self.pixels, self.K, self.dist)
        self.assertLess(self.np.linalg.norm(self.np.asarray(centre) - self.camera), 1e-3)

    def _reviewed(self, pixels):
        return [{"frame_id": i + 1, "opencv_index": i, "project_timestamp_s": 1. + 0.1 * i, "centre": list(p)} for i, p in enumerate(pixels)]

    def _pose(self):
        return [[1. + 0.1 * i] + list(w) + [0., 0., 0., .01, .01, .01, 1.] for i, w in enumerate(self.world)]

    def test_evaluate_consistent_and_wrong_focal_inconsistent(self):
        ok = REPROJ.evaluate(self._reviewed(self.pixels), self._pose(), self.camera, self.K, self.dist, 1 / 30., 0)
        self.assertEqual(ok["status"], "CONSISTENT")
        wrong_K = [[1200., 0., 960.], [0., 1200., 540.], [0., 0., 1.]]
        bad = REPROJ.evaluate(self._reviewed(self.pixels), self._pose(), self.camera, wrong_K, self.dist, 1 / 30., 0)
        self.assertEqual(bad["status"], "INCONSISTENT")
        few = REPROJ.evaluate(self._reviewed(self.pixels)[:3], self._pose(), self.camera, self.K, self.dist, 1 / 30., 0)
        self.assertEqual(few["status"], "INSUFFICIENT_REVIEWED_BOXES")

    def test_points_behind_the_camera_are_refused(self):
        world = list(self.world) + [self.camera - self.R.T @ self.np.array([0., 0., 50.])]
        pixels = list(self.pixels) + [[960., 540.]]
        reviewed = [{"frame_id": i + 1, "opencv_index": i, "project_timestamp_s": 1. + 0.1 * i, "centre": list(p)}
                    for i, p in enumerate(pixels)]
        pose = [[1. + 0.1 * i] + list(w) + [0., 0., 0., .01, .01, .01, 1.] for i, w in enumerate(world)]
        r = REPROJ.evaluate(reviewed, pose, self.camera, self.K, self.dist, 1 / 30., 0)
        self.assertEqual(r["status"], "POINTS_BEHIND_FITTED_CAMERA")
        self.assertGreaterEqual(r["points_behind_camera"], 1)

    def test_points_past_the_fold_back_radius_are_refused(self):
        """A target far outside the field of view still projects into the image once the model folds."""
        limit = REPROJ.valid_distortion_radius(self.dist)
        far = self.camera + self.R.T @ self.np.array([limit * 2.5 * 40., 0., 40.])
        world = list(self.world) + [far]
        reviewed = [{"frame_id": i + 1, "opencv_index": i, "project_timestamp_s": 1. + 0.1 * i,
                     "centre": list(p)} for i, p in enumerate(list(self.pixels) + [[500., 500.]])]
        pose = [[1. + 0.1 * i] + list(w) + [0., 0., 0., .01, .01, .01, 1.] for i, w in enumerate(world)]
        r = REPROJ.evaluate(reviewed, pose, self.camera, self.K, self.dist, 1 / 30., 0)
        self.assertEqual(r["status"], "POINTS_OUTSIDE_VALID_DISTORTION_RADIUS")
        self.assertGreater(r["max_normalized_radius"], r["valid_radius_limit"])

    def test_discrimination_reports_pixels_per_frame(self):
        pose = [[t, 0., 100., 0., 0., 0., 0., .01, .01, .01, 1.] for t in (0.,)]
        pose = [[0.0, 0., 100., 0., 0., 0., 0., .01, .01, .01, 1.],
                [0.1, 1., 100., 0., 0., 0., 0., .01, .01, .01, 1.]]
        d = REPROJ.alignment_discrimination(pose, [0., 0., 0.], [0.02], 1 / 30., 1500.)
        self.assertEqual(d["frames"], 1)
        self.assertAlmostEqual(d["median_px"], 1500. * math.atan(1 / 3. / 100.), delta=0.05)

    def test_discrimination_empty_outside_support(self):
        pose = [[0.0, 0., 100., 0., 0., 0., 0., .01, .01, .01, 1.],
                [0.1, 1., 100., 0., 0., 0., 0., .01, .01, .01, 1.]]
        self.assertEqual(REPROJ.alignment_discrimination(pose, [0., 0., 0.], [5.0], 1 / 30., 1500.)["frames"], 0)

    def test_true_offset_recovered_from_the_winning_shift(self):
        self.assertEqual(REPROJ.true_index_offset(0, 0), 0)     # edit-list candidate
        self.assertEqual(REPROJ.true_index_offset(0, 1), -1)    # reader dropped the last frame
        self.assertEqual(REPROJ.true_index_offset(-1, 0), -1)   # the first pilot's own rendering
        self.assertEqual(REPROJ.true_index_offset(-1, -1), 0)

    def test_review_csv_requires_visible_and_valid_box(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "review.csv"
            path.write_text("frame_id,opencv_index,project_timestamp_s,drone0_visible,box_x1,box_y1,box_x2,box_y2\n"
                            "1,0,1.0,yes,10,10,20,20\n2,1,2.0,no,,,,\n")
            rows = REPROJ.load_review(path)
            self.assertEqual([r["frame_id"] for r in rows], [1])
            self.assertEqual(rows[0]["centre"], [15., 15.])
            path.write_text("frame_id,opencv_index,project_timestamp_s,drone0_visible,box_x1,box_y1,box_x2,box_y2\n1,0,1.0,yes,20,10,10,20\n")
            with self.assertRaises(ValueError): REPROJ.load_review(path)


if __name__ == "__main__":
    unittest.main()
