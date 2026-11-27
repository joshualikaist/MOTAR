"""CPU stand-ins exercise the evaluator lifecycle; these are not GPU rollout evidence."""
import ast
import importlib.util
import json
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


legacy = load(ROOT / "tests/test_kroc2027_adapter_contract.py", "_recovery_legacy_fixture")
adapter = legacy.adapter
audit = load(ROOT / "tools/recovery_observability.py", "_recovery_trace_tool_test")


def task_fixture(n=1):
    task = legacy.make_task(n=n)
    task.perception.tracker.memory_s = 5.0
    task.perception.tracker.age = torch.zeros(n)
    task.perception.detection_latency_s = 0.0
    task.perception._latency_steps = 0
    task.perception.empirical_error = None
    task._target_dynamics = "bounded"
    task._physical_target = False
    task._target_behavior_level = "e2_obstacle_aware"
    task._target_distance_backend = "direct_l2_v1"
    task._target_motion_model = "bounded_planar_drone_v1_rollout_direct_l2_v1"
    task.task_obs = {"observations": torch.zeros(n, 898)}
    return task


def flags(task, camera=True, lidar=False, age=0.0):
    n = task.num_envs
    task.perception.tracker.active[:] = camera or age <= 5.0
    task.perception.tracker.age[:] = age
    return {"camera_visible": torch.full((n,), camera, dtype=torch.bool),
            "lidar_visible": torch.full((n,), lidar, dtype=torch.bool),
            "visible": torch.full((n,), camera or lidar, dtype=torch.bool),
            "track_age": task.perception.tracker.age}


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


class RecoveryIntegrationTests(unittest.TestCase):
    def make_recorder(self, directory, task=None, enabled=True):
        task = task or task_fixture()
        env = legacy.recorder_env(directory, NAVRL_KROC2027_RECOVERY_EVENTS=str(int(enabled)),
                                  NAVRL_EVAL_CHECKPOINT=audit.POLICY_F_SHA256)
        with mock.patch.dict(os.environ, env), mock.patch.object(adapter, "_build_recovery_probe", return_value=None):
            rec = adapter.build_kroc_adapter(task)
        self.addCleanup(rec.finalize, "test cleanup")
        rec.on_reset(torch.arange(task.num_envs))
        return task, rec

    def test_off_import_boundary_and_required_recorder(self):
        with mock.patch.dict(os.environ, {adapter.RECORDER_VAR: "0", adapter.RECOVERY_VAR: "0"}):
            self.assertIsNone(adapter.build_kroc_adapter(object()))
        with mock.patch.dict(os.environ, {adapter.RECORDER_VAR: "0", adapter.RECOVERY_VAR: "1"}):
            with self.assertRaisesRegex(ValueError, "requires"):
                adapter.build_kroc_adapter(object())
        with mock.patch.dict(os.environ, {adapter.RECOVERY_VAR: "typo"}):
            with self.assertRaises(ValueError):
                adapter.build_kroc_adapter(object())

    def test_detached_staging_once_clock_and_final_outcome_join(self):
        with tempfile.TemporaryDirectory() as directory:
            task, rec = self.make_recorder(directory)
            diag = flags(task)
            rec.after_perception(diag)
            rec.after_perception(diag)  # Repeated staging cannot count an extra frame.
            diag["camera_visible"][:] = False
            task.perception.tracker.active[:] = False
            task.perception.tracker.age[:] = 9.0
            rec.before_action()
            with self.assertRaisesRegex(ValueError, "unconsumed"):
                rec.before_action()
            rec._steps += 1  # CPU fixture: completed physical-action boundary, not a rollout.
            rec.after_perception(flags(task, camera=False, age=.1))
            rec.before_action()
            rec._steps += 1
            rec.on_episode_end(torch.tensor([True]), torch.tensor([True]),
                               torch.tensor([False]), torch.tensor([False]))
            rec.finalize()
            trace = rows(Path(directory) / adapter.RECOVERY_FILE)
            observed = [r["observation"] for r in trace if r["record_type"] == "observation"]
            self.assertEqual([r["frame_index"] for r in observed], [0, 1])
            self.assertTrue(observed[0]["camera_valid"])
            self.assertTrue(observed[0]["track_active"])
            self.assertEqual(observed[0]["track_age_s"], 0.0)
            self.assertIsNone(observed[0]["capture_time_s"])
            self.assertIsNone(observed[0]["center_los_blocked"])
            self.assertEqual(trace[-1]["summary"]["terminal_time_s"], .2)
            self.assertEqual(trace[-1]["summary"]["outcome"], "capture")
            result = audit.validate_trace(Path(directory) / adapter.RECOVERY_FILE,
                                          Path(directory) / adapter.EPISODES_FILE)
            self.assertEqual(result["status"], "PASS", result)
            self.assertEqual(result["ledger_episodes_joined"], 1)

    def test_partial_reset_censors_and_clears_eligibility_only_for_reset_env(self):
        with tempfile.TemporaryDirectory() as directory:
            task, rec = self.make_recorder(directory, task_fixture(n=2))
            rec.after_perception(flags(task))
            rec.before_action()
            rec._steps += 1
            rec.after_perception(flags(task, camera=False, age=.1))
            rec.before_action()
            rec._steps += 1
            rec.on_reset(torch.tensor([0]))
            rec.after_perception(flags(task, camera=False, age=.2))
            rec.before_action()
            rec._steps += 1
            rec.finalize("interrupted")
            trace = rows(Path(directory) / adapter.RECOVERY_FILE)
            reset = next(r for r in trace if r["record_type"] == "reset")
            self.assertIsNone(reset["summary"]["outcome"])
            self.assertEqual(reset["terminal_time_s"], .2)
            obs = [r for r in trace if r["record_type"] == "observation"][-2:]
            self.assertEqual([r["observation"]["eligible_before_frame"] for r in obs], [False, True])
            self.assertEqual([r["observation"]["frame_index"] for r in obs], [0, 2])
            self.assertTrue(all(r["summary"]["outcome"] is None for r in trace
                                if r["record_type"] == "run_shutdown"))
            result = audit.validate_trace(Path(directory) / adapter.RECOVERY_FILE)
            self.assertEqual(result["status"], "PASS", result)

    def test_empty_reset_is_skipped_and_shutdown_is_not_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            task, rec = self.make_recorder(directory)
            rec.on_reset(torch.tensor([0]))
            rec.after_perception(flags(task, camera=False, age=5.1))
            rec.before_action()
            rec.finalize("budget stop")
            trace = rows(Path(directory) / adapter.RECOVERY_FILE)
            self.assertEqual(trace[-1]["record_type"], "run_shutdown")
            self.assertFalse(trace[-1]["summary"]["ever_acquired"])
            self.assertIsNone(trace[-1]["summary"]["outcome"])
            self.assertEqual(trace[-1]["terminal_time_s"], 0.0)
            self.assertEqual(json.loads((Path(directory) / adapter.RECOVERY_SUMMARY_FILE).read_text())
                             ["skipped_empty_episodes"], 1)

    def test_read_only_task_tensors_rng_and_legacy_sidecar_identity(self):
        with tempfile.TemporaryDirectory() as base:
            old_bytes = []
            for enabled in (False, True):
                directory = Path(base) / str(enabled) / "same_cell"
                directory.mkdir(parents=True)
                task = task_fixture()
                diag = flags(task)
                tensors = [*task.obs_dict.values(), task.target_position, task.target_vel_w,
                           task.perception.tracker.active, task.perception.tracker.age,
                           task.task_obs["observations"], *diag.values()]
                snapshots = [tensor.clone() for tensor in tensors]
                py_rng, np_rng, torch_rng = random.getstate(), np.random.get_state(), torch.get_rng_state().clone()
                with mock.patch.object(adapter, "datetime") as fixed_time:
                    fixed_time.now.return_value.isoformat.return_value = "CPU_FIXED_TIMESTAMP"
                    _, rec = self.make_recorder(directory, task, enabled)
                    rec.after_perception(diag)
                    rec.before_action()
                    rec._steps += 1
                    rec.on_episode_end(torch.tensor([True]), torch.tensor([False]),
                                       torch.tensor([False]), torch.tensor([True]))
                    rec.finalize()
                for tensor, before in zip(tensors, snapshots):
                    self.assertTrue(torch.equal(tensor, before))
                self.assertEqual(random.getstate(), py_rng)
                self.assertEqual(np.random.get_state()[0], np_rng[0])
                np.testing.assert_array_equal(np.random.get_state()[1], np_rng[1])
                self.assertEqual(np.random.get_state()[2:], np_rng[2:])
                self.assertTrue(torch.equal(torch.get_rng_state(), torch_rng))
                old_bytes.append([(directory / name).read_bytes()
                                  for name in (adapter.EPISODES_FILE, adapter.SUMMARY_FILE)])
                self.assertEqual((directory / adapter.RECOVERY_FILE).exists(), enabled)
            self.assertEqual(old_bytes[0], old_bytes[1])

    def test_existing_evidence_refused_before_old_recorder_open(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / adapter.EPISODES_FILE
            path.write_bytes(b"historical evidence\n")
            with mock.patch.dict(os.environ, legacy.recorder_env(directory, NAVRL_KROC2027_RECOVERY_EVENTS="1")):
                with self.assertRaises(FileExistsError):
                    adapter.build_kroc_adapter(task_fixture())
            self.assertEqual(path.read_bytes(), b"historical evidence\n")

    def test_missing_camera_flag_or_inconsistent_fusion_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            task, rec = self.make_recorder(directory)
            with self.assertRaisesRegex(ValueError, "camera_visible"):
                rec.after_perception({"visible": torch.tensor([True])})
            diag = flags(task)
            diag["visible"][:] = False
            with self.assertRaisesRegex(ValueError, "fused validity"):
                rec.after_perception(diag)
            rec.finalize()

    def test_current_mesh_and_delayed_measurements_are_not_a_primary_occlusion(self):
        class Probe:
            def snapshot(self, token, *, render_token):
                return [{"target_center_in_fov": True, "target_center_in_range": True,
                         "center_los_blocked": True, "geometry_contract": "CPU_FIXTURE_MESH"}]
        with tempfile.TemporaryDirectory() as directory:
            task, rec = self.make_recorder(directory)
            rec._recovery.probe = Probe()
            rec.after_perception(flags(task))
            rec.before_action()
            rec._steps += 1
            task.perception.detection_latency_s = .2
            task.perception._latency_steps = 2
            rec.after_perception(flags(task, camera=False, age=.1))
            rec.before_action()
            rec._steps += 1
            rec.finalize()
            obs = [r["observation"] for r in rows(Path(directory) / adapter.RECOVERY_FILE)
                   if r["record_type"] == "observation"]
            self.assertTrue(obs[1]["center_los_occlusion"])
            self.assertIsNone(obs[1]["post_acquisition_occlusion_candidate"])
            self.assertEqual(obs[1]["measurement_alignment"], "NOT_RECORDED")

    def test_static_scene_claim_requires_source_checked_fixed_bars(self):
        bars = type("bar_asset_params", (), {
            "__module__": "aerial_gym.config.asset_config.env_object_config",
            "num_assets": 70, "fix_base_link": True,
            "min_state_ratio": [0.] * 13, "max_state_ratio": [0.] * 13})
        cfg = type("NavRLBarsEnvCfg", (), {
            "__module__": "aerial_gym.config.env_config.navrl_bars_env",
            "env": SimpleNamespace(use_warp=True),
            "env_config": SimpleNamespace(include_asset_type={"bars": True},
                                          asset_type_to_dict_map={"bars": bars})})
        task = task_fixture()
        self.assertEqual(adapter._recovery_scene_contract(task), "UNKNOWN")
        self.assertIsNone(adapter._build_recovery_probe(task))
        task.sim_env = SimpleNamespace(cfg=cfg)
        self.assertEqual(adapter._recovery_scene_contract(task), "static_scene_mesh")
        bars.fix_base_link = False
        self.assertEqual(adapter._recovery_scene_contract(task), "UNKNOWN")
        bars.fix_base_link = True
        bars.min_state_ratio[7] = .1
        self.assertEqual(adapter._recovery_scene_contract(task), "UNKNOWN")
        bars.min_state_ratio[7] = 0.
        task._physical_target = True
        self.assertEqual(adapter._recovery_scene_contract(task), "UNKNOWN")

    def test_unsupported_motion_cannot_be_mislabeled_as_valid_direct_lineage(self):
        for dynamics, model in (("physical", "physx_ref5in_6dof_motor_wrench_v2_same_substep"),
                                ("bounded", "unapproved_route_model")):
            with self.subTest(dynamics=dynamics), tempfile.TemporaryDirectory() as directory:
                task = task_fixture()
                task._target_dynamics, task._target_motion_model = dynamics, model
                _, rec = self.make_recorder(directory, task)
                rec.after_perception(flags(task))
                rec.before_action()
                rec.finalize()
                path = Path(directory) / adapter.RECOVERY_FILE
                self.assertEqual(rows(path)[0]["metadata"]["motion_lineage"], "NOT_RECORDED")
                self.assertEqual(rows(path)[0]["metadata"]["target_motion_model"], model)
                self.assertEqual(audit.validate_trace(path)["status"], "INVALID")

    def test_cli_replay_detects_tampered_trace_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            task, rec = self.make_recorder(directory)
            rec.after_perception(flags(task))
            rec.before_action()
            rec._steps += 1
            rec.finalize()
            path = Path(directory) / adapter.RECOVERY_FILE
            original = rows(path)
            for field, value in (("state_time_s", .1), ("eligible_before_frame", True), ("prefix_hash", "0" * 64)):
                data = json.loads(json.dumps(original))
                if field == "state_time_s":
                    data[1]["inputs"]["stamp"][field] = value
                else:
                    data[1]["observation"][field] = value
                damaged = Path(directory) / (field + ".jsonl")
                damaged.write_text("\n".join(json.dumps(r) for r in data) + "\n")
                self.assertEqual(audit.validate_trace(damaged)["status"], "INVALID")
            out = Path(directory) / "audit.json"
            env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1")
            command = [sys.executable, "-B", str(ROOT / "tools/recovery_observability.py"),
                       "--trace", str(path), "--device", "cpu", "--out", str(out)]
            run = subprocess.run(command, env=env, text=True, capture_output=True)
            self.assertEqual(run.returncode, 0, run.stderr + run.stdout)
            before = out.read_bytes()
            run = subprocess.run(command, env=env, text=True, capture_output=True)
            self.assertNotEqual(run.returncode, 0)
            self.assertEqual(out.read_bytes(), before)

    def test_no_new_runtime_readback_or_frozen_source_edits(self):
        for relative in ("aerial_gym/task/navrl_task/navrl_task.py",
                         "aerial_gym/task/navrl_task/navrl_perception.py",
                         "aerial_gym/task/navrl_task/navrl_detector.py",
                         "motar_eval/events.py"):
            baseline = subprocess.run(["git", "show", "HEAD:" + relative], cwd=ROOT,
                                      check=True, capture_output=True).stdout
            self.assertEqual((ROOT / relative).read_bytes(), baseline)
        tree = ast.parse((ROOT / "aerial_gym/task/navrl_task/kroc_eval_adapter.py").read_text())
        recovery = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "_RecoveryRecorder")
        for node in ast.walk(recovery):
            if isinstance(node, (ast.Assign, ast.AugAssign, ast.AnnAssign)):
                targets = getattr(node, "targets", [getattr(node, "target", None)])
                self.assertFalse(any(isinstance(child, ast.Attribute) and child.attr == "task"
                                     for target in targets if target is not None
                                     for child in ast.walk(target)))


if __name__ == "__main__":
    unittest.main()
