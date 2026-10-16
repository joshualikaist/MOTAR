"""Execute the actual MT-P1 task call boundary on CPU without importing Isaac Gym."""
import ast
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import torch


ROOT = Path(__file__).resolve().parents[1]
TASK = ROOT / "aerial_gym/task/navrl_task/navrl_task.py"
spec = importlib.util.spec_from_file_location(
    "_mt_p1_task_motion", ROOT / "aerial_gym/task/navrl_task/target_motion.py"
)
motion = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motion)


def call_boundary():
    tree = ast.parse(TASK.read_text())
    advance = next(n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == "_advance_target")
    block = next(n for n in ast.walk(advance) if isinstance(n, ast.If) and any(
        isinstance(s, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "bounded_kwargs"
                                         for t in s.targets) for s in n.body))
    begin = next(i for i, n in enumerate(block.body) if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == "bounded_kwargs" for t in n.targets))
    end = next(i for i, n in enumerate(block.body) if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == "local_rollout_feasible" for t in n.targets))
    fn = ast.parse("def boundary(self, old_xy, desired_velocity, bounded_speed_limit, dt, bars_all, "
                   "planner_lo, planner_hi, bars_half_extents):\n    pass\n").body[0]
    fn.body = block.body[begin:end] + ast.parse("return bounded_result").body
    namespace = {"torch": torch, "math": math,
                 "bounded_drone_target_step": motion.bounded_drone_target_step}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])),
                 str(TASK), "exec"), namespace)
    return namespace["boundary"]


class MotionTaskBoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.boundary = staticmethod(call_boundary())

    def fixture(self, capture, backend):
        # Use production-like strided xyz views and a nonzero bar slot offset.
        n, b = 2, 70
        pos = torch.tensor([[10., 10., 1.], [20., 20., 1.]])
        vel = torch.zeros(n, 3)
        obstacles = torch.full((n, b + 1, 3), 35.)
        task = SimpleNamespace(
            target_vel_w=vel, _tm_speed=torch.ones(n),
            _tm_avoid_sign=torch.tensor([1., -1.]),
            tm=SimpleNamespace(max_accel=4., max_turn_rate_deg=150., avoidance_lookahead_s=1.),
            _target_planner_clearance=lambda: .77,
            _mt_precision_capture=capture, _target_distance_backend=backend,
            _target_motion_model=(motion.DIRECT_L2_TARGET_MOTION_MODEL
                                  if backend == "direct_l2_v1" else motion.BOUNDED_TARGET_MOTION_MODEL),
            _target_behavior_level="e1_cv", num_task_steps=5, num_envs=n,
            _bar_offset=1, n_bars_active=b, _mt_precision_last_call=None,
        )
        args = (pos[:, :2], torch.tensor([[1., 0.], [0., 1.]]), task._tm_speed,
                .1, obstacles[:, 1:, :2], torch.zeros(n, 2), torch.full((n, 2), 40.), None)
        return task, args

    def test_capture_preserves_outputs_rng_and_input_values(self):
        for backend in ("legacy_cdist", "direct_l2_v1"):
            plain, args = self.fixture(False, backend)
            capture, captured_args = self.fixture(True, backend)
            rng = torch.get_rng_state().clone()
            expected = self.boundary(plain, *args)
            actual = self.boundary(capture, *captured_args)
            self.assertTrue(torch.equal(torch.get_rng_state(), rng))
            self.assertIsNone(plain._mt_precision_last_call)
            for a, b in zip(actual[:4], expected):
                self.assertTrue(torch.equal(a, b))
            saved = capture._mt_precision_last_call
            self.assertEqual(saved["kwargs"]["distance_backend"], backend)
            self.assertEqual(saved["metadata"]["env_ids"].tolist(), [0, 1])
            self.assertEqual(saved["metadata"]["obstacle_slots"].tolist(), list(range(1, 71)))
            for key, arg in (("old_xy", captured_args[0]), ("bars_xy", captured_args[4])):
                self.assertEqual(saved["input_layouts"][key]["stride"], list(arg.stride()))
                self.assertEqual(saved["input_layouts"][key]["storage_offset"], arg.storage_offset())
                self.assertTrue(torch.equal(saved["kwargs"][key], arg))
                arg.add_(1.)
                self.assertFalse(torch.equal(saved["kwargs"][key], arg))
            self.assertEqual(saved["kwargs"]["turn_sign"].tolist(), [1., -1.])
            self.assertTrue(torch.equal(saved["result"]["new_xy"], actual[0]))
            actual[0].add_(1.)
            self.assertFalse(torch.equal(saved["result"]["new_xy"], actual[0]))

    def test_default_task_kwargs_match_public_legacy_api(self):
        task, args = self.fixture(False, "legacy_cdist")
        result = self.boundary(task, *args)
        expected = motion.bounded_drone_target_step(
            args[0], task.target_vel_w[:, :2], args[1], args[2], args[3], args[4], args[5],
            args[6], .77, task._tm_avoid_sign, torch.full((2,), 4.),
            torch.full((2,), math.radians(150.)), 1., None)
        for a, b in zip(result, expected):
            self.assertTrue(torch.equal(a, b))


if __name__ == "__main__":
    unittest.main()
