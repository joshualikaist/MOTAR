"""R-P1a-v2 compact occlusion telemetry: storage, non-interference and the compact recovery mode (CPU).

CPU stand-ins exercise the recorder lifecycle; they are not GPU rollout evidence.
"""
import ast
import importlib.util
import json
import os
from pathlib import Path
import random
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motar_eval import occlusion_trace as ot  # noqa: E402


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


rp1a = load(ROOT / "tests/test_recovery_observability_contract.py", "_occlusion_rp1a_fixture")
legacy, adapter = rp1a.legacy, rp1a.adapter


def make_recorder(test, directory, task, **flags):
    env = legacy.recorder_env(directory, NAVRL_EVAL_CHECKPOINT=rp1a.audit.POLICY_F_SHA256, **flags)
    with mock.patch.dict(os.environ, env), mock.patch.object(adapter, "_build_recovery_probe", return_value=None):
        rec = adapter.build_kroc_adapter(task)
    test.addCleanup(rec.finalize, "test cleanup")
    rec.on_reset(torch.arange(task.num_envs))
    return rec


def drive(rec, task, steps=3, outcome="capture"):
    for step in range(steps):
        rec.after_perception(rp1a.flags(task, camera=step != 1))
        rec.before_action()
        task.obs_dict["robot_position"][:, 0] += 0.25
        task.obs_dict["robot_linvel"][:, 0] = 2.5
        task.target_position[:, 1] += 0.1
        task.target_vel_w[:, 1] = 1.0
        rec._steps += 1
    n = task.num_envs
    flags = {k: torch.full((n,), outcome == k) for k in ("capture", "crash", "timeout")}
    rec.on_episode_end(torch.ones(n, dtype=torch.bool), flags["capture"], flags["crash"], flags["timeout"])


class StorageTest(unittest.TestCase):
    def test_record_size_budget(self):
        self.assertLessEqual(ot.FRAME_DTYPE.itemsize, 96)
        # A quota-4 cell: about 50,000 env-frames and 1,000 resets of 205 boxes.
        self.assertLess(ot.estimate_bytes(50_000, 1_000, 205), 12_000_000)

    def test_round_trip_dedupe_and_tamper(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = ot.TraceWriter(directory, {"dt_s": 0.1})
            center = np.array([[1.0, 2.0, 1.5]])
            half = np.array([[0.3, 0.3, 1.5]])
            writer.add_reset(0, 0, "h1", center, half)
            writer.add_reset(1, 0, "h1", center, half)
            writer.add_reset(0, 1, "h2", center + 1, half)
            rec = np.zeros(2, dtype=ot.FRAME_DTYPE)
            rec["env"] = [0, 1]
            rec["flags"] = ot.pack_flags([True, False], [True, False], [True, True], [False, True],
                                         [False, False], None, [False, True])
            writer.write_frames(rec)
            summary = writer.close("test")
            self.assertEqual((summary["layouts"], summary["resets"], summary["frames"]), (2, 3, 2))
            data = ot.read_trace(directory)
            self.assertEqual(ot.reset_lookup(data["layouts"]), {(0, 0): 0, (1, 0): 0, (0, 1): 1})
            np.testing.assert_allclose(ot.layout_arrays(data["layouts"], 1)[0], center + 1)
            self.assertEqual(list(ot.flag(data["frames"], ot.CAMERA_VALID)), [True, False])
            self.assertEqual(list(ot.flag(data["frames"], ot.ACQUIRED_BEFORE)), [False, True])
            with self.assertRaises(FileExistsError):
                ot.TraceWriter(directory, {})
            path = Path(directory) / ot.FRAMES_FILE
            path.write_bytes(path.read_bytes()[:-1] + b"\x01")
            with self.assertRaisesRegex(ValueError, "SHA-256"):
                ot.read_trace(directory)


class RecorderIntegrationTest(unittest.TestCase):
    def test_off_on_existing_outputs_task_and_rng_unchanged(self):
        with tempfile.TemporaryDirectory() as base:
            outputs = []
            for enabled in (False, True):
                directory = Path(base) / str(enabled) / "same_cell"
                directory.mkdir(parents=True)
                task = rp1a.task_fixture(n=2)
                task.obs_dict["obstacle_orientation"] = torch.zeros(2, 3, 4)
                task.obs_dict["obstacle_orientation"][..., 3] = 1.0
                start = [t.clone() for t in (*task.obs_dict.values(), task.target_position, task.target_vel_w)]
                rng = random.getstate(), np.random.get_state()[1].copy(), torch.get_rng_state().clone()
                with mock.patch.object(adapter, "datetime") as fixed:
                    fixed.now.return_value.isoformat.return_value = "CPU_FIXED_TIMESTAMP"
                    rec = make_recorder(self, directory, task, NAVRL_KROC2027_OCCLUSION_TRACE=str(int(enabled)))
                    # Snapshot after reset: everything the recorder reads must stay as the driver left it.
                    rec.after_perception(rp1a.flags(task))
                    before = [t.clone() for t in (*task.obs_dict.values(), task.target_position)]
                    rec.before_action()
                    for tensor, snap in zip((*task.obs_dict.values(), task.target_position), before):
                        self.assertTrue(torch.equal(tensor, snap))
                    rec._steps += 1
                    drive(rec, task, steps=2)
                    rec.finalize()
                self.assertEqual(random.getstate(), rng[0])
                np.testing.assert_array_equal(np.random.get_state()[1], rng[1])
                self.assertTrue(torch.equal(torch.get_rng_state(), rng[2]))
                outputs.append([(directory / n).read_bytes() for n in (adapter.EPISODES_FILE, adapter.SUMMARY_FILE)])
                self.assertEqual((directory / ot.FRAMES_FILE).exists(), enabled)
                del start
            self.assertEqual(outputs[0], outputs[1])
            data = ot.read_trace(Path(base) / "True" / "same_cell")
            frames = data["frames"]
            self.assertEqual(len(frames), 2 * 3)  # 2 envs x 3 consumed observations
            self.assertEqual(sorted(set(frames["frame"].tolist())), [0, 1, 2])
            env0 = frames[frames["env"] == 0]
            np.testing.assert_allclose(np.diff(env0["p_pos"][:, 0]), [0.0, 0.25], atol=1e-6)
            self.assertEqual(list(ot.flag(env0, ot.CAMERA_VALID)), [True, True, False])
            self.assertEqual(list(ot.flag(env0, ot.ACQUIRED_BEFORE)), [False, True, True])
            self.assertEqual(len(data["layouts"]["hashes"]), 1)  # both envs share the stand-in layout
            summary = data["summary"]
            self.assertEqual(summary["metadata"]["render_alignment"], "ZERO_LATENCY_RENDER_DELIVERY")
            self.assertEqual(summary["metadata"]["sensor_capture_time"], "NOT_RECORDED")
            self.assertEqual(summary["metadata"]["gpu_trajectory_noninterference"], "NOT_TESTED")

    def test_flags_fail_closed(self):
        with mock.patch.dict(os.environ, {adapter.RECORDER_VAR: "0", adapter.OCCLUSION_VAR: "1"}):
            with self.assertRaisesRegex(ValueError, "requires"):
                adapter.build_kroc_adapter(object())
        with mock.patch.dict(os.environ, {adapter.OCCLUSION_VAR: "yes please"}):
            with self.assertRaises(ValueError):
                adapter.build_kroc_adapter(object())
        with mock.patch.dict(os.environ, {adapter.RECOVERY_TRACE_VAR: "compact", adapter.RECOVERY_VAR: "0"}):
            with self.assertRaisesRegex(ValueError, "requires"):
                adapter.build_kroc_adapter(object())
        with mock.patch.dict(os.environ, {adapter.RECOVERY_TRACE_VAR: "tiny"}):
            with self.assertRaises(ValueError):
                adapter.build_kroc_adapter(object())

    def test_compact_recovery_keeps_episode_records_only_and_probe_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            task = rp1a.task_fixture()
            rec = make_recorder(self, directory, task, NAVRL_KROC2027_RECOVERY_EVENTS="1",
                                NAVRL_KROC2027_RECOVERY_TRACE="compact", NAVRL_KROC2027_OCCLUSION_TRACE="1")
            drive(rec, task, steps=3)
            rec.finalize()
            rows = rp1a.rows(Path(directory) / adapter.RECOVERY_FILE)
            kinds = [r["record_type"] for r in rows]
            self.assertNotIn("observation", kinds)
            self.assertEqual(kinds[0], "header")
            self.assertEqual(rows[0]["metadata"]["observation_records"], "omitted_compact_episode_records_only")
            closed = [r for r in rows if r["record_type"] == "episode_end"]
            self.assertEqual(len(closed), 1)
            self.assertTrue(closed[0]["summary"]["intervals"])  # the camera loss at step 1 after acquisition
            frames = ot.read_trace(directory)["frames"]
            self.assertTrue(np.all(frames["probe_center_blocked"] == -1))  # probe absent -> unknown
            self.assertTrue(np.all(frames["probe_visible_pixels"] == -1))

    def test_trace_class_never_assigns_task_state(self):
        tree = ast.parse((ROOT / "aerial_gym/task/navrl_task/kroc_eval_adapter.py").read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "_OcclusionTrace")
        for node in ast.walk(cls):
            if isinstance(node, (ast.Assign, ast.AugAssign, ast.AnnAssign)):
                targets = getattr(node, "targets", [getattr(node, "target", None)])
                for target in targets:
                    for child in ast.walk(target):
                        self.assertFalse(isinstance(child, ast.Attribute) and child.attr in ("task", "obs_dict"))
                        self.assertFalse(isinstance(child, ast.Subscript) and isinstance(child.value, ast.Name)
                                         and child.value.id in ("t", "od", "task"))


if __name__ == "__main__":
    unittest.main()
