"""CPU scalar-contract tests for evaluation-only recovery chronology."""
from dataclasses import FrozenInstanceError, asdict
import json
import math
import random
import struct
import unittest

from motar_eval.recovery_events import (
    CONTRACT, FINAL_DIAGNOSTICS_CONTRACT, NOT_RECORDED, EvaluationGeometry,
    FrameStamp, RecoveryEpisodeEvents, SensorFrame,
)


def sensor(camera=False, lidar=False, active=True, age=0., **kwargs):
    return SensorFrame(camera, lidar, camera or lidar, active, age,
                       fused_sources_contract=FINAL_DIAGNOSTICS_CONTRACT, **kwargs)


def geometry(fov=True, within_range=True, blocked=False, **kwargs):
    kwargs.setdefault("measurement_alignment", "ZERO_LATENCY_RENDER_DELIVERY")
    return EvaluationGeometry(fov, within_range, blocked, "CPU_SYNTHETIC_MESH_CENTRE_V1", **kwargs)


class RecoveryEventsTest(unittest.TestCase):
    def observe(self, monitor, index, s=None, g=None, **kwargs):
        return monitor.observe(FrameStamp(monitor.episode_id, index, index * monitor.dt_s, **kwargs),
                               sensor() if s is None else s, geometry() if g is None else g)

    def acquired(self, episode="episode", **kwargs):
        monitor = RecoveryEpisodeEvents(episode, **kwargs)
        first = self.observe(monitor, 0, sensor(camera=True))
        self.assertFalse(first["eligible_before_frame"])
        return monitor

    def test_never_acquired_hidden_start_and_lidar_do_not_open_recovery(self):
        m = RecoveryEpisodeEvents("episode")
        for i in range(6):
            row = self.observe(m, i, sensor(lidar=True), geometry(blocked=True))
            self.assertFalse(row["eligible_before_frame"])
            self.assertTrue(row["initial_search"])
            self.assertFalse(row["post_acquisition_occlusion_candidate"])
        summary = m.finish(.6, "timeout")
        self.assertFalse(summary["ever_acquired"])
        self.assertEqual(summary["intervals"], [])

    def test_acquisition_requires_camera_fused_and_active_same_frame(self):
        m = RecoveryEpisodeEvents("episode")
        self.observe(m, 0, sensor(camera=True, active=False))
        self.observe(m, 1, sensor(lidar=True, active=True))
        self.assertFalse(m.ever_acquired)
        self.observe(m, 2, sensor(camera=True))
        row = self.observe(m, 3)
        self.assertTrue(row["eligible_before_frame"])
        self.assertIn("camera_measurement_loss", m.open_runs)

    def test_independent_camera_fused_track_geometry_and_reacquisition(self):
        m = self.acquired()
        self.observe(m, 1, sensor(lidar=True), geometry(fov=False, blocked=True))
        self.observe(m, 2, sensor(active=False, age=5.000001, track_age_dtype="torch.float32"),
                     geometry(within_range=False, blocked=True))
        row = self.observe(m, 3, sensor(camera=True), geometry(blocked=True, rendered_target_visible_pixels=8))
        events = row["newly_emitted_events"]
        self.assertEqual({e["event"] for e in events if "reacquired" in e["event"]},
                         {"camera_measurement_reacquired", "fused_measurement_reacquired", "track_reacquired"})
        self.assertTrue(row["center_los_occlusion"])
        self.assertFalse(row["post_acquisition_occlusion_candidate"])
        self.assertEqual(row["full_visual_occlusion"], NOT_RECORDED)
        all_events = m.finish(.4, "capture")["intervals"]
        camera_loss = next(e for e in all_events if e["event"] == "camera_measurement_loss")
        fused_loss = next(e for e in all_events if e["event"] == "fused_measurement_loss")
        self.assertAlmostEqual(camera_loss["duration_s"], .2)
        self.assertAlmostEqual(fused_loss["duration_s"], .1)
        self.assertTrue(any(e["event"] == "track_expiry_transition" for e in all_events))
        self.assertTrue(any(e["event"] == "fov_exit" for e in all_events))
        self.assertTrue(any(e["event"] == "range_exit" for e in all_events))

    def test_mesh_block_does_not_prove_loss_or_full_occlusion(self):
        m = self.acquired()
        row = self.observe(m, 1, sensor(camera=True), geometry(blocked=True, rendered_target_visible_pixels=4))
        self.assertTrue(row["mesh_center_los_blocked"])
        self.assertTrue(row["center_los_occlusion"])
        self.assertFalse(row["camera_measurement_loss"])
        self.assertFalse(row["post_acquisition_occlusion_candidate"])
        with self.assertRaises(ValueError):
            geometry(full_visual_occlusion="TRUE")

    def test_unknown_geometry_censors_and_splits_without_inventing_visibility(self):
        m = self.acquired()
        self.observe(m, 1, g=geometry(blocked=True))
        row = self.observe(m, 2, g=geometry(fov=None, within_range=None, blocked=None))
        ended = [e for e in row["newly_emitted_events"] if e["event"] == "center_los_occlusion"]
        self.assertEqual(len(ended), 1)
        self.assertTrue(ended[0]["censored"])
        self.assertEqual(ended[0]["boundary"], "geometry_unknown")
        self.assertIsNone(row["center_los_occlusion"])
        self.assertIn("camera_measurement_loss", m.open_runs)
        self.observe(m, 3, g=geometry(blocked=True))
        result = m.finish(.4, "timeout")
        self.assertEqual(sum(e["event"] == "center_los_occlusion" for e in result["intervals"]), 2)

    def test_unknown_or_delayed_measurement_alignment_never_proves_primary_occlusion(self):
        m = self.acquired()
        first = self.observe(m, 1, g=geometry(blocked=True))
        self.assertTrue(first["post_acquisition_occlusion_candidate"])
        # Current render geometry is known, but the delivered measurement can be delayed.
        # Its actual older capture time does not establish a matching old geometry probe.
        unknown = EvaluationGeometry(True, True, True, "CPU_CURRENT_RENDER_V1")
        row = self.observe(m, 2, g=unknown, capture_time_s=.1)
        self.assertTrue(row["center_los_occlusion"])
        self.assertIsNone(row["post_acquisition_occlusion_candidate"])
        self.assertEqual(row["measurement_alignment"], NOT_RECORDED)
        self.assertNotIn("post_acquisition_occlusion_candidate", m.open_runs)
        ended = next(e for e in row["newly_emitted_events"]
                     if e["event"] == "post_acquisition_occlusion_candidate")
        self.assertTrue(ended["censored"])
        self.assertEqual(ended["boundary"], "alignment_unknown")
        self.assertIn("center_los_occlusion", m.open_runs)
        with self.assertRaises(ValueError):
            geometry(measurement_alignment="DELAYED_BUT_ASSUMED_MATCHED")
        a, b = self.acquired(), self.acquired()
        aligned = self.observe(a, 1, g=geometry(blocked=True))
        unaligned = self.observe(b, 1, g=geometry(blocked=True, measurement_alignment=NOT_RECORDED))
        self.assertNotEqual(aligned["prefix_hash"], unaligned["prefix_hash"])

    def test_actual_strict_float32_expiry_not_evaluator_elapsed_time(self):
        ages = [None, 4.999999523162842, 5., 5.000000476837158]
        for age in ages:
            with self.subTest(age=age):
                m = self.acquired(dt_s=10.)
                row = self.observe(m, 1, sensor(active=False, age=age, track_age_dtype="torch.float32"))
                event = next(e for e in row["newly_emitted_events"] if e["event"].startswith("track_"))
                expected = "track_expiry_transition" if age is not None and age > 5. else "track_deactivation"
                self.assertEqual(event["event"], expected)
                self.assertEqual(event["actual_track_age_s"], age)
                self.assertEqual(event["track_age_dtype"], "torch.float32")
        # Recorded 0.1 float32 accumulation remains below 5 at the 50th step;
        # its actual scalar, rather than integer frame arithmetic, decides expiry.
        age = 0.
        for _ in range(50):
            age = struct.unpack("f", struct.pack("f", age + .1))[0]
        self.assertLess(age, 5.)
        m = self.acquired()
        row = self.observe(m, 1, sensor(active=False, age=age, track_age_dtype="torch.float32"))
        self.assertEqual(row["newly_emitted_events"][0]["event"], "track_deactivation")
        age = struct.unpack("f", struct.pack("f", age + .1))[0]
        self.assertGreater(age, 5.)
        m = self.acquired()
        row = self.observe(m, 1, sensor(active=False, age=age, track_age_dtype="torch.float32"))
        self.assertEqual(row["newly_emitted_events"][0]["event"], "track_expiry_transition")

    def test_2_5_10_15_second_arithmetic_is_chronology_only(self):
        for duration in (2, 5, 10, 15):
            with self.subTest(duration=duration):
                m = self.acquired()
                for i in range(1, duration * 10 + 1):
                    self.observe(m, i, g=geometry(blocked=True))
                self.observe(m, duration * 10 + 1, sensor(camera=True))
                summary = m.finish((duration * 10 + 2) * .1, "capture")
                loss = next(e for e in summary["intervals"] if e["event"] == "camera_measurement_loss")
                self.assertAlmostEqual(loss["duration_s"], duration)
                self.assertFalse(loss["censored"])
                self.assertTrue(any(e["event"] == "camera_measurement_reacquired" for e in summary["intervals"]))

    def test_terminal_and_shutdown_reset_censor_without_expiry(self):
        for kind, outcome in (("episode_end", "capture"), ("episode_end", "crash"),
                              ("episode_end", "timeout"), ("run_shutdown", None), ("reset", None)):
            with self.subTest(kind=kind, outcome=outcome):
                m = self.acquired()
                self.observe(m, 1)
                summary = m.finish(.2, outcome, terminal_kind=kind)
                self.assertTrue(all(e["censored"] for e in summary["intervals"]))
                self.assertFalse(any("expiry" in e["event"] for e in summary["intervals"]))
                self.assertEqual(summary["outcome"], outcome)
                self.assertEqual(summary["safe_abort"], NOT_RECORDED)
                with self.assertRaises(ValueError):
                    m.finish(.3, outcome, terminal_kind=kind)
                with self.assertRaises(ValueError):
                    self.observe(m, 2)
        m = self.acquired("next")
        self.assertTrue(m.ever_acquired)
        next_episode = RecoveryEpisodeEvents("next_episode")
        self.assertFalse(self.observe(next_episode, 0)["eligible_before_frame"])

    def test_validation_does_not_mutate_monitor_on_failure(self):
        m = self.acquired()
        attempts = [FrameStamp("wrong", 1, .1), FrameStamp("episode", 0, .1),
                    FrameStamp("episode", 2, .1), FrameStamp("episode", 1, .2)]
        for stamp in attempts:
            with self.assertRaises(ValueError):
                m.observe(stamp, sensor(), geometry())
        with self.assertRaises(ValueError):
            self.observe(m, 1, g=geometry(frame_index=0))
        with self.assertRaises(ValueError):
            self.observe(m, 1, g=geometry(state_time_s=.0))
        self.assertEqual(m.frames, 1)
        self.assertEqual(self.observe(m, 1)["frame_index"], 1)
        for bad in (math.inf, math.nan, -1, True, "0"):
            with self.assertRaises(ValueError):
                FrameStamp("episode", 0, bad)
            with self.assertRaises(ValueError):
                SensorFrame(False, False, False, False, bad)
        for bad in (0., True, -1):
            with self.assertRaises(ValueError):
                FrameStamp("episode", bad, 0.)
        with self.assertRaises(ValueError):
            SensorFrame(1, False, True, True, 0.)

    def test_frame_tokens_capture_times_and_delay_are_separate(self):
        m = RecoveryEpisodeEvents("episode")
        self.observe(m, 0, sensor(camera=True), frame_token="ring0", capture_time_s=0.)
        row = self.observe(m, 1, frame_token="ring1", capture_time_s=0.)
        self.assertEqual(row["capture_time_s"], 0.)
        self.assertEqual(row["state_time_s"], .1)
        with self.assertRaises(ValueError):
            self.observe(m, 2, frame_token="ring1", capture_time_s=.1)
        self.observe(m, 2, frame_token="ring2", capture_time_s=.1)
        with self.assertRaises(ValueError):
            self.observe(m, 3, frame_token="ring3", capture_time_s=0.)
        with self.assertRaises(ValueError):
            FrameStamp("episode", 0, 0., capture_time_s=.1)

    def test_known_source_or_and_unknown_source_are_distinct(self):
        with self.assertRaises(ValueError):
            SensorFrame(True, False, False, True, 0., fused_sources_contract=FINAL_DIAGNOSTICS_CONTRACT)
        with self.assertRaises(ValueError):
            SensorFrame(False, None, False, True, 0., fused_sources_contract=FINAL_DIAGNOSTICS_CONTRACT)
        unknown = SensorFrame(False, None, True, True, 0.)
        row = self.observe(RecoveryEpisodeEvents("episode"), 0, unknown)
        self.assertFalse(row["fused_source_identity_verified"])
        self.assertIsNone(row["lidar_valid"])
        self.assertEqual(row["lidar_valid_status"], NOT_RECORDED)
        self.assertEqual(row["camera_loss_cause"], NOT_RECORDED)

    def test_inconsistent_final_measurement_cannot_prove_expiry(self):
        m = self.acquired()
        row = self.observe(m, 1, sensor(camera=True, active=False, age=5.1))
        event = next(e for e in row["newly_emitted_events"] if e["event"].startswith("track_"))
        self.assertEqual(event["event"], "track_deactivation")
        self.assertEqual(event["reason"], "UNKNOWN")

    def test_immutable_copied_records_prefix_suffix_independence_and_no_rng(self):
        s, g, stamp = sensor(camera=True), geometry(), FrameStamp("episode", 0, 0.)
        with self.assertRaises(FrozenInstanceError):
            s.camera_valid = False
        identity = {"source_commit": "abc", "checkpoint_sha256": "def", "seed": 49}
        state = random.getstate()
        a = RecoveryEpisodeEvents("episode", run_identity=identity)
        b = RecoveryEpisodeEvents("episode", run_identity=dict(identity))
        identity["seed"] = 50
        row_a = a.observe(stamp, s, g)
        row_b = b.observe(stamp, s, g)
        self.assertEqual(row_a, row_b)
        prefix = json.dumps(row_a, sort_keys=True)
        self.observe(a, 1)
        self.observe(b, 1, sensor(camera=True))
        result_a = a.finish(.2, "crash")
        result_b = b.finish(.2, "capture")
        self.assertEqual(prefix, json.dumps(row_a, sort_keys=True))
        self.assertNotEqual(result_a["prefix_hash"], result_b["prefix_hash"])
        self.assertEqual(result_a["run_identity"]["seed"], 49)
        self.assertEqual(random.getstate(), state)
        self.assertEqual(asdict(stamp), {"episode_id": "episode", "frame_index": 0,
                                        "state_time_s": 0., "capture_time_s": None, "frame_token": None})
        self.assertEqual(result_a["scene_manifest_hash"], NOT_RECORDED)
        self.assertEqual(result_a["pre_loss_actor_prefix_hash"], NOT_RECORDED)
        self.assertEqual(result_a["contract"], CONTRACT)
        json.dumps(result_a, allow_nan=False)

    def test_finish_validates_outcome_boundary_and_unobserved_empty_episode(self):
        m = self.acquired()
        for args, kwargs in (((-.1, "timeout"), {}), ((.1,), {}),
                             ((.1, "capture"), {"terminal_kind": "run_shutdown"}),
                             ((.1, "safe_abort"), {}), ((.1,), {"terminal_kind": "other"})):
            with self.assertRaises(ValueError):
                m.finish(*args, **kwargs)
        self.assertFalse(m.closed)
        empty = RecoveryEpisodeEvents("empty").finish(0., terminal_kind="run_shutdown")
        self.assertEqual(empty["frames"], 0)
        self.assertEqual(empty["intervals"], [])


if __name__ == "__main__":
    unittest.main()
