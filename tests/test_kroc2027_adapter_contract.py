"""KROC2027 adapter contract (CPU only, no Isaac Gym).

The adapter is loaded by file path and driven with a stand-in task. It must:
* do nothing and import nothing from motar_eval when every NAVRL_KROC2027_* variable is unset;
* consume no torch, numpy or Python RNG state and write into no task tensor while recording;
* index episodes exactly as the episode ledger does, with the same outcome rule;
* stop on a motion-contract violation in "stop" mode and only record it in "record" mode;
* draw the Experiment A initial view with the task's own footprint rule (taken from navrl_task.py by
  AST), respawning the pursuer when needed and failing closed when no target is admissible;
and navrl_task.py must call it only behind `if self._kroc_adapter is not None` guards.
"""
import ast
import importlib.util
import json
import math
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
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
ADAPTER = ROOT / "aerial_gym/task/navrl_task/kroc_eval_adapter.py"
TASK = ROOT / "aerial_gym/task/navrl_task/navrl_task.py"
LEDGER = ROOT / "aerial_gym/task/navrl_task/navrl_episode_ledger.py"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


adapter = _load(ADAPTER, "_kroc2027_adapter_under_test")


def _task_function(name):
    """A module-level function of navrl_task.py, compiled alone (navrl_task imports Isaac Gym)."""
    tree = ast.parse(TASK.read_text(encoding="utf-8"))
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = {"torch": torch}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(TASK), "exec"), namespace)
    return namespace[name]


footprint_rule = _task_function("_spawn_footprint_clearance_accepted")


def _yaw_quat(yaw):
    return torch.tensor([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def make_task(n=4, bars=None):
    """A stand-in with exactly the attributes the adapter reads."""
    if bars is None:
        bars = [[30.0, 30.0], [5.0, 35.0], [35.0, 5.0]]
    b = len(bars)
    center = torch.zeros(n, b, 3)
    center[:, :, 0:2] = torch.tensor(bars)
    center[:, :, 2] = 1.5
    half = torch.zeros(n, b, 3)
    half[:, :, 0:2] = 0.3
    half[:, :, 2] = 1.5
    robot = torch.zeros(n, 3)
    robot[:, 0:2] = torch.tensor([[12.0 + 2 * i, 15.0] for i in range(n)])
    robot[:, 2] = 1.0
    quat = torch.stack([_yaw_quat(0.0)] * n)
    target = robot.clone()
    target[:, 0] += 10.0
    task = SimpleNamespace(
        num_envs=n, device=torch.device("cpu"), step_dt=0.1,
        vis_cfg=SimpleNamespace(detector_hfov_deg=87.0, detector_vfov_deg=58.0,
                                detector_max_range=20.0, camera_translation=[0.10, 0.0, 0.03],
                                camera_target_radius=0.15),
        task_config=SimpleNamespace(max_velocity=2.5, flight_altitude=1.0),
        obs_dict={
            "robot_position": robot, "robot_vehicle_orientation": quat.clone(),
            "robot_orientation": quat.clone(), "robot_linvel": torch.zeros(n, 3),
            "obstacle_position": center, "asset_collision_half_extents": half,
            "env_bounds_min": torch.zeros(n, 3), "env_bounds_max": torch.tensor([[40.0, 40.0, 3.0]] * n),
        },
        target_position=target, target_vel_w=torch.zeros(n, 3), _tm_speed=torch.full((n,), 1.0),
        _solid_obstacle_offset=0, _bar_offset=0, n_bars_active=b,
        ep_min_goal_dist=torch.full((n,), 10.0),
        perception=SimpleNamespace(tracker=SimpleNamespace(active=torch.zeros(n, dtype=torch.bool))),
    )
    task._spawn_required_surface_margin_m = lambda: 0.65
    return task


def recorder_env(directory, **extra):
    env = {"NAVRL_KROC2027_RECORDER": "1", "NAVRL_BULK_EVAL_JSON": str(Path(directory) / "result.json"),
           "NAVRL_SEED": "4190", "NAVRL_NUM_BARS": "70", "NAVRL_KROC2027_ARM": "A3",
           "NAVRL_KROC2027_RUN_ID": "KROC27-A-C01"}
    env.update(extra)
    return env


def drive(task, rec, steps, finished_at, speed=1.0):
    """Move targets along +x at ``speed`` and finish env e at the steps listed in finished_at[e]."""
    n = task.num_envs
    for step in range(1, steps + 1):
        rec.before_action()
        task.target_vel_w[:, 0] = speed
        task.target_position[:, 0] += speed * task.step_dt
        rec.after_target_advance()
        actions = torch.zeros(n, 4)
        actions[:, 0] = 0.5
        command = torch.zeros(n, 4)
        command[:, 0] = 1.25
        task.obs_dict["robot_linvel"][:, 0] = 1.2
        task.obs_dict["robot_position"][:, 0] += 0.12
        rec.after_physics(actions, command)
        rec.observe_capture_distance(torch.full((n,), 3.0))
        finished = torch.tensor([step in finished_at.get(e, ()) for e in range(n)])
        successes = finished & (torch.arange(n) % 2 == 0)
        crashes = finished & ~successes
        timeouts = torch.zeros(n, dtype=torch.bool)
        rec.on_episode_end(finished, successes, crashes, timeouts)
        if bool(finished.any()):
            ids = torch.nonzero(finished).flatten()
            task.target_position[ids, 0] = task.obs_dict["robot_position"][ids, 0] + 10.0
            task.target_vel_w[ids] = 0.0
            task.target_vel_w[ids, 0] = speed  # a new episode starts at its cruise velocity
            rec.on_reset(ids)
        rec.after_perception({"visible": torch.ones(n, dtype=torch.bool),
                              "camera_visible": torch.ones(n, dtype=torch.bool)})
    return rec


class OffByDefaultTest(unittest.TestCase):
    def test_no_variable_means_no_adapter_and_no_motar_eval_import(self):
        code = ("import importlib.util, sys\n"
                "spec = importlib.util.spec_from_file_location('a', %r)\n"
                "m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)\n"
                "print(m.initial_view_from_env({}) is None, m.build_kroc_adapter(object()) is None,"
                " 'motar_eval' in sys.modules)\n" % str(ADAPTER))
        env = {k: v for k, v in os.environ.items() if not k.startswith("NAVRL_KROC2027_")}
        env["PYTHONNOUSERSITE"] = "1"
        out = subprocess.run([sys.executable, "-B", "-c", code], env=env, capture_output=True,
                             text=True, cwd=str(ROOT / "tests"))
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertEqual(out.stdout.strip().splitlines()[-1], "True True False")

    def test_off_values(self):
        for value in ("", "0", "off", "OFF", "none"):
            self.assertIsNone(adapter.initial_view_from_env({"NAVRL_KROC2027_INITIAL_VIEW": value}))
        spec = adapter.initial_view_from_env({"NAVRL_KROC2027_INITIAL_VIEW": "Hidden",
                                              "NAVRL_KROC2027_INITIAL_DISTANCE_M": "10"})
        self.assertEqual((spec.view, spec.distance_m), ("hidden", 10.0))

    def test_task_hooks_are_all_guarded(self):
        tree = ast.parse(TASK.read_text(encoding="utf-8"))
        calls, guarded = [], []
        for node in ast.walk(tree):
            if isinstance(node, ast.If):
                test = ast.dump(node.test)
                if "_kroc_adapter" in test and "IsNot" in test:
                    for inner in ast.walk(node):
                        if isinstance(inner, ast.Call) and isinstance(inner.func, ast.Attribute) \
                                and "_kroc_adapter" in ast.dump(inner.func.value):
                            guarded.append(inner.func.attr)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                    and "_kroc_adapter" in ast.dump(node.func.value):
                calls.append(node.func.attr)
        self.assertEqual(sorted(calls), sorted(guarded))
        self.assertEqual(sorted(set(calls)), ["after_perception", "after_physics", "after_target_advance",
                                              "before_action", "finalize", "observe_capture_distance",
                                              "on_episode_end", "on_reset"])


class RecorderContractTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def _recorder(self, task, velocity=0.0, **extra):
        with mock.patch.dict(os.environ, recorder_env(self.tmp.name, **extra)):
            rec = adapter.build_kroc_adapter(task)
        task.target_vel_w[:, 0] = velocity
        rec.on_reset(torch.arange(task.num_envs))
        rec.after_perception({"visible": torch.zeros(task.num_envs, dtype=torch.bool)})
        return rec

    def test_no_rng_and_no_task_writes(self):
        task = make_task()
        torch.manual_seed(1)
        np.random.seed(1)
        random.seed(1)
        before_rng = (torch.get_rng_state().clone(), np.random.get_state()[1].copy(), random.getstate())
        rec = self._recorder(task)
        snapshot = {k: v.clone() for k, v in task.obs_dict.items()}
        for step in range(1, 4):
            rec.before_action()
            rec.after_target_advance()
            rec.after_physics(torch.zeros(task.num_envs, 4), torch.zeros(task.num_envs, 4))
            rec.observe_capture_distance(torch.full((task.num_envs,), 2.0))
            rec.after_perception({"visible": torch.zeros(task.num_envs, dtype=torch.bool)})
        rec.on_episode_end(torch.ones(task.num_envs, dtype=torch.bool),
                           torch.ones(task.num_envs, dtype=torch.bool),
                           torch.zeros(task.num_envs, dtype=torch.bool),
                           torch.zeros(task.num_envs, dtype=torch.bool))
        rec.finalize()
        self.assertTrue(torch.equal(before_rng[0], torch.get_rng_state()))
        self.assertTrue(np.array_equal(before_rng[1], np.random.get_state()[1]))
        self.assertEqual(before_rng[2], random.getstate())
        for key, value in snapshot.items():
            self.assertTrue(torch.equal(value, task.obs_dict[key]), key)

    def test_episode_indices_and_outcomes_match_the_ledger(self):
        ledger_mod = _load(LEDGER, "_kroc2027_ledger_under_test")
        task = make_task()
        rec = self._recorder(task, velocity=1.0)
        finished_at = {0: (3, 7), 1: (5,), 2: (2, 4, 8), 3: ()}
        ledger = ledger_mod.EpisodeLedger(task.num_envs, 2, Path(self.tmp.name) / "episodes.jsonl",
                                          Path(self.tmp.name) / "summary.json",
                                          {"cell_id": "c", "arm": "a", "bars": 70, "seed": 1}, 100)
        original = rec.on_episode_end

        def both(finished, successes, crashes, timeouts):
            original(finished, successes, crashes, timeouts)
            ledger.record(finished, successes, crashes, timeouts, torch.zeros(task.num_envs))

        rec.on_episode_end = both
        drive(task, rec, 9, finished_at)
        rec.finalize()
        ledger._handle.close()
        rows = [json.loads(l) for l in (Path(self.tmp.name) / "episodes.jsonl").read_text().splitlines()]
        records = [json.loads(l) for l in rec.episodes_path.read_text().splitlines()]
        self.assertEqual([(r["env_index"], r["env_episode_index"], r["outcome"]) for r in rows],
                         [(r["env_index"], r["env_episode_index"], r["outcome"]) for r in records])

    def test_records_and_summary(self):
        task = make_task()
        rec = self._recorder(task, velocity=1.0)
        drive(task, rec, 6, {0: (6,), 1: (6,), 2: (6,), 3: (6,)})
        rec.finalize()
        summary = json.loads(rec.summary_path.read_text())
        records = [json.loads(l) for l in rec.episodes_path.read_text().splitlines()]
        self.assertEqual(summary["episodes_recorded"], 4)
        self.assertEqual(summary["motion_gate"]["verdict"], "PASS")
        self.assertEqual(summary["swept_clearance"]["violations"], 0)
        self.assertEqual(summary["import_origins"]["motar_eval"],
                         str((ROOT / "motar_eval/__init__.py").resolve()))
        import hashlib
        self.assertEqual(summary["episodes_sha256"], hashlib.sha256(rec.episodes_path.read_bytes()).hexdigest())
        first = records[0]
        self.assertEqual(first["actions"], 6)
        self.assertEqual(first["first_obs_fused_valid"], 2)
        self.assertAlmostEqual(first["first_acquisition_latency_s"], 0.1)
        self.assertAlmostEqual(first["pursuer_actual_speed_max_mps"], 1.2, places=5)
        self.assertAlmostEqual(first["pursuer_requested_speed_max_mps"], 1.25, places=5)
        self.assertEqual(first["steps_actual_over_2p5"], 0)
        self.assertEqual(first["initial"]["episode_key"], "s4190/b70/e000/k000")
        self.assertTrue(first["initial"]["initial_geo_visible"])
        self.assertAlmostEqual(first["initial"]["initial_range_xy_m"], 10.0, places=5)
        self.assertEqual(first["swept_min_distance_m"], 3.0)

    def test_observation_counted_when_an_action_uses_it(self):
        task = make_task(n=2)
        task._fa_ep_first_fused = torch.tensor([2, -1])
        task._fa_ep_first_camera = torch.tensor([2, -1])
        rec = self._recorder(task)
        # A repeated observation at reset replaces the pending one; it is not counted twice.
        rec.after_perception({"visible": torch.zeros(2, dtype=torch.bool)})
        rec.before_action()
        rec.after_perception({"visible": torch.tensor([True, False])})
        rec.before_action()
        rec.on_episode_end(torch.ones(2, dtype=torch.bool), torch.ones(2, dtype=torch.bool),
                           torch.zeros(2, dtype=torch.bool), torch.zeros(2, dtype=torch.bool))
        rec.finalize()
        records = [json.loads(l) for l in rec.episodes_path.read_text().splitlines()]
        self.assertEqual([r["observations"] for r in records], [2, 2])
        self.assertEqual([r["first_obs_fused_valid"] for r in records], [2, None])
        self.assertEqual([r["task_first_fused_obs"] for r in records], [2, None])
        self.assertEqual([r["task_first_camera_obs"] for r in records], [2, None])

    def test_action_without_observation_fails(self):
        task = make_task(n=1)
        with mock.patch.dict(os.environ, recorder_env(self.tmp.name)):
            rec = adapter.build_kroc_adapter(task)
        rec.on_reset(torch.arange(1))
        with self.assertRaises(RuntimeError):
            rec.before_action()
        rec.finalize()

    def test_stop_mode_raises_and_record_mode_continues(self):
        for mode, raises in (("stop", True), ("record", False)):
            task = make_task()
            rec = self._recorder(task, NAVRL_KROC2027_ON_VIOLATION=mode)
            task.target_position[1, 0] += 0.9  # a 9 m/s jump with zero velocity: G1/G2 violations
            if raises:
                with self.assertRaises(RuntimeError):
                    rec.after_target_advance()
            else:
                rec.after_target_advance()
                rec.finalize()
            summary = json.loads(rec.summary_path.read_text())
            self.assertEqual(summary["motion_gate"]["verdict"], "FAIL")

    def test_swept_clearance_and_wall(self):
        task = make_task(n=1, bars=[[20.0, 15.0]])
        task.target_position[0, 0:2] = torch.tensor([19.0, 15.0])
        rec = self._recorder(task, NAVRL_KROC2027_ON_VIOLATION="record")
        task.target_vel_w[0, 0] = 1.0
        task.target_position[0, 0] = 19.1  # bar face at 19.7: clearance 0.6 m, legal
        rec.after_target_advance()
        self.assertEqual(rec.swept["violations"], 0)
        task.target_position[0, 0:2] = torch.tensor([45.0, 15.0])
        rec.after_target_advance()
        self.assertEqual(rec.swept["wall_violations"], 1)
        rec.finalize()
        self.assertGreater(json.loads(rec.summary_path.read_text())["swept_clearance"]["violations"], 0)


class InitialViewForTaskTest(unittest.TestCase):
    def _sample(self, view, task, seed=7, respawn=None):
        spec = adapter.initial_view_from_env({"NAVRL_KROC2027_INITIAL_VIEW": view})
        if respawn is not None:
            task._randomize_general_drone_spawn = respawn
        torch.manual_seed(seed)
        env_ids = torch.arange(task.num_envs)
        return adapter.sample_initial_view_for_task(
            task, env_ids, task.obs_dict["env_bounds_min"], task.obs_dict["env_bounds_max"], spec,
            footprint_rule)

    def test_distance_altitude_yaw_and_stats(self):
        visible_task, hidden_task = make_task(), make_task()
        goal_v = self._sample("visible", visible_task)
        goal_h = self._sample("hidden", hidden_task)
        self.assertTrue(torch.equal(goal_v, goal_h))
        rng = (goal_v[:, 0:2] - visible_task.obs_dict["robot_position"][:, 0:2]).norm(dim=1)
        self.assertTrue(torch.allclose(rng, torch.full_like(rng, 10.0), atol=1e-4))
        self.assertTrue(bool((goal_v[:, 2] == 1.0).all()))
        qv = visible_task.obs_dict["robot_orientation"]
        qh = hidden_task.obs_dict["robot_orientation"]
        yaw_v = torch.atan2(2 * qv[:, 3] * qv[:, 2], 1 - 2 * qv[:, 2] ** 2)
        yaw_h = torch.atan2(2 * qh[:, 3] * qh[:, 2], 1 - 2 * qh[:, 2] ** 2)
        diff = torch.remainder(yaw_h - yaw_v + math.pi, 2 * math.pi) - math.pi
        self.assertTrue(torch.allclose(diff.abs(), torch.full_like(diff, math.pi), atol=1e-4))
        bearing = torch.atan2(goal_v[:, 1] - visible_task.obs_dict["robot_position"][:, 1],
                              goal_v[:, 0] - visible_task.obs_dict["robot_position"][:, 0])
        self.assertTrue(torch.allclose(torch.remainder(yaw_v - bearing + math.pi, 2 * math.pi) - math.pi,
                                       torch.zeros_like(yaw_v), atol=1e-4))
        self.assertTrue(bool((visible_task._kroc_initial_view_stats["respawns"] == 0).all()))

    def test_respawn_then_fail_closed(self):
        # A ring of bars 5 m around env 0's pursuer blocks every 10 m ray.
        angles = [2 * math.pi * k / 72 for k in range(72)]
        ring = [[12.0 + 5 * math.cos(a), 15.0 + 5 * math.sin(a)] for a in angles]
        task = make_task(n=1, bars=ring)
        moves = []

        def respawn(ids):
            moves.append(ids.tolist())
            task.obs_dict["robot_position"][ids, 0:2] = torch.tensor([30.0, 30.0])

        goal = self._sample("visible", task, respawn=respawn)
        self.assertEqual(moves, [[0]])
        self.assertEqual(int(task._kroc_initial_view_stats["respawns"][0]), 1)
        self.assertAlmostEqual(float((goal[0, 0:2] - torch.tensor([30.0, 30.0])).norm()), 10.0, places=3)
        stuck = make_task(n=1, bars=ring)
        with self.assertRaises(RuntimeError):
            self._sample("visible", stuck, respawn=lambda ids: None)


if __name__ == "__main__":
    unittest.main()
