"""Offline contracts for ETH intake: no network, GPU or dataset required."""
import hashlib
import importlib.util
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
            args = ["prepare", "--video", str(video), "--queue", str(queue), "--intake-receipt", str(receipt),
                    "--calibration", str(calibration), "--output", str(root / "output"), "--ffprobe", "mock"]
            stream = {"streams": [{"codec_type": "video", "nb_frames": "4", "width": 64, "height": 48,
                                   "avg_frame_rate": "30/1"}]}
            with mock.patch.object(sys, "argv", args), mock.patch.object(FRAMES.subprocess, "check_output", return_value=json.dumps(stream)), contextlib.redirect_stdout(io.StringIO()):
                FRAMES.main()
            result = json.loads((root / "output/receipt.json").read_text())
            self.assertFalse(result["calibration_validated"])
            self.assertIsNone(result["frames"][0]["box_xyxy"])
            image = cv2.imread(str(root / "output/frame_000002.png"))
            self.assertAlmostEqual(float(image.mean()), 50., delta=3.)


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
        self.assertAlmostEqual(gt["rpy_deg_uninterpreted"][2], 180.)

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
