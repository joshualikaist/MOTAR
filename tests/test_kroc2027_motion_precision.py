"""CPU contracts for the opt-in virtual-target direct-distance lineage.

The injected cdist fault is a synthetic counterexample, not a CPU reproduction of TF32 or
evidence that a simulator trajectory is collision-free.
"""

import ast
import importlib.util
import math
import os
from pathlib import Path
import unittest
from unittest import mock

import torch


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "_kroc2027_motion_precision", ROOT / "aerial_gym/task/navrl_task/target_motion.py"
)
MOTION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOTION)
DENSITIES = (0, 40, 70, 80, 115, 120, 160, 205)


def inputs(bars=0):
    """Two rows in the arena; distant bars do not restrict the initial trajectory."""
    return dict(
        old_xy=torch.tensor([[20.0, 20.0], [21.0, 20.0]]),
        current_velocity=torch.zeros(2, 2),
        desired_velocity=torch.tensor([[1.5, 0.0], [0.0, 1.5]]),
        speed_limit=torch.full((2,), 1.5),
        dt=0.1,
        bars_xy=torch.full((2, bars, 2), 35.0),
        lo=torch.full((2, 2), 1.0),
        hi=torch.full((2, 2), 39.0),
        clearance=0.77,
        turn_sign=torch.tensor([1.0, -1.0]),
        max_accel=torch.full((2,), 4.0),
        max_turn_rate=torch.full((2,), math.radians(150.0)),
        lookahead_s=1.0,
        return_certificate=True,
    )


def assert_same(test, first, second):
    for a, b in zip(first[:-1], second[:-1]):
        test.assertTrue(torch.equal(a, b))
    test.assertEqual(first[-1].keys(), second[-1].keys())
    for key in first[-1]:
        a, b = first[-1][key], second[-1][key]
        if isinstance(a, torch.Tensor):
            test.assertTrue(torch.equal(a, b), key)
        else:
            test.assertEqual(a, b, key)


class MotionPrecisionTest(unittest.TestCase):
    def test_new_lineage_requires_bounded_moving_behavior(self):
        for behavior in ("e1_cv", "e2_obstacle_aware"):
            contract = MOTION.resolve_target_distance_contract("direct_l2_v1", "bounded", behavior)
            self.assertEqual(contract["distance_backend"], "direct_l2_v1")
            self.assertEqual(contract["motion_model"], MOTION.DIRECT_L2_TARGET_MOTION_MODEL)
        for dynamics in ("legacy", "bounded", "physical"):
            for behavior in ("historical", "e0_static", "e1_cv", "e2_obstacle_aware", "e3_reactive"):
                if dynamics == "bounded" and behavior in ("e1_cv", "e2_obstacle_aware"):
                    continue
                with self.subTest(dynamics=dynamics, behavior=behavior), self.assertRaises(ValueError):
                    MOTION.resolve_target_distance_contract("direct_l2_v1", dynamics, behavior)
        for dynamics, expected in (("legacy", MOTION.TARGET_MOTION_MODEL),
                                   ("bounded", MOTION.BOUNDED_TARGET_MOTION_MODEL),
                                   ("physical", MOTION.PHYSICAL_TARGET_MOTION_MODEL)):
            contract = MOTION.resolve_target_distance_contract("legacy_cdist", dynamics, "historical")
            self.assertEqual(contract["motion_model"], expected)
        with self.assertRaises(ValueError):
            MOTION.resolve_target_distance_contract("direct_l2_v1", "unknown", "e1_cv")

    def test_backend_validation_is_explicit_and_fail_closed(self):
        for backend in MOTION.TARGET_DISTANCE_BACKENDS:
            self.assertEqual(MOTION.validate_target_distance_backend(backend), backend)
        for invalid in ("", "direct", "DIRECT_L2_V1", None):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                MOTION.bounded_drone_target_step(**inputs(), distance_backend=invalid)

    def test_default_equals_explicit_legacy_with_unchanged_cdist_dispatch(self):
        for density in DENSITIES:
            with self.subTest(density=density):
                kwargs = inputs(density)
                with mock.patch.object(torch, "cdist", wraps=torch.cdist) as call:
                    default = MOTION.bounded_drone_target_step(**kwargs)
                    self.assertEqual(call.call_count, 10 if density else 0)
                legacy = MOTION.bounded_drone_target_step(**kwargs, distance_backend="legacy_cdist")
                assert_same(self, default, legacy)

    def test_direct_screen_matches_independent_double_distance_oracle(self):
        kwargs = inputs(160)
        kwargs["current_velocity"][:] = torch.tensor([1.0, 0.0])
        kwargs["desired_velocity"][:] = torch.tensor([1.5, 0.0])
        kwargs["bars_xy"][:, 0, :] = torch.tensor([21.7, 20.0])

        def oracle(x, y):
            # An independent float64 reference avoids squared-norm cancellation.
            dx = x.double().unsqueeze(-2) - y.double().unsqueeze(-3)
            return torch.sqrt(torch.sum(dx * dx, dim=-1))

        with mock.patch.object(torch, "cdist", side_effect=oracle):
            reference = MOTION.bounded_drone_target_step(**kwargs)
        with mock.patch.object(torch, "cdist", side_effect=AssertionError("cdist called")):
            direct = MOTION.bounded_drone_target_step(**kwargs, distance_backend="direct_l2_v1")
        for a, b in zip(reference[:4], direct[:4]):
            self.assertTrue(torch.allclose(a, b) if a.is_floating_point() else torch.equal(a, b))
        self.assertTrue(torch.equal(reference[-1]["selected_index"], direct[-1]["selected_index"]))
        self.assertTrue(torch.equal(reference[-1]["full_horizon_safe"], direct[-1]["full_horizon_safe"]))

    def test_synthetic_cdist_overestimate_cannot_hide_bar_from_direct_screen(self):
        kwargs = inputs(160)
        kwargs["old_xy"][:] = torch.tensor([30.0, 30.0])
        kwargs["current_velocity"][:] = torch.tensor([1.5, 0.0])
        kwargs["desired_velocity"][:] = torch.tensor([1.5, 0.0])
        kwargs["bars_xy"][:, 0, :] = torch.tensor([30.1, 30.0])

        def overestimated(x, y):
            return torch.full(x.shape[:-1] + (y.shape[-2],), 5.0, dtype=x.dtype)

        with mock.patch.object(torch, "cdist", side_effect=overestimated):
            legacy = MOTION.bounded_drone_target_step(**kwargs)
            direct = MOTION.bounded_drone_target_step(**kwargs, distance_backend="direct_l2_v1")
        self.assertTrue(bool(legacy[3].all()))
        self.assertFalse(bool(direct[3].any()))
        self.assertFalse(bool(direct[-1]["full_horizon_safe"].any()))
        # Direct arithmetic does not introduce teleportation or an instantaneous stop on failure.
        self.assertTrue(bool((direct[1].norm(dim=1) > 0.0).all()))
        self.assertTrue(torch.allclose(direct[0], kwargs["old_xy"] + direct[1] * 0.1))

    def test_direct_motion_bounds_and_finiteness_across_density_shapes(self):
        for density in DENSITIES:
            kwargs = inputs(density)
            previous = kwargs["current_velocity"]
            for step in range(12):
                angle = step * math.pi / 12.0
                kwargs["desired_velocity"][:] = torch.tensor([math.cos(angle), math.sin(angle)]) * 1.5
                result = MOTION.bounded_drone_target_step(**kwargs, distance_backend="direct_l2_v1")
                pos, velocity = result[:2]
                with self.subTest(density=density, step=step):
                    self.assertTrue(bool(torch.isfinite(pos).all()))
                    self.assertTrue(bool((velocity.norm(dim=1) <= 1.5 + 1e-6).all()))
                    self.assertTrue(bool(((velocity - previous).norm(dim=1) / 0.1 <= 4.0 + 1e-5).all()))
                    self.assertTrue(torch.allclose(pos, kwargs["old_xy"] + velocity * 0.1))
                    cross = previous[:, 0] * velocity[:, 1] - previous[:, 1] * velocity[:, 0]
                    turn = torch.atan2(cross.abs(), (previous * velocity).sum(dim=1)) / 0.1
                    judged = previous.norm(dim=1) > MOTION.HEADING_VALID_SPEED_MPS
                    self.assertTrue(bool((turn[judged] <= math.radians(150) + 1e-5).all()))
                kwargs["old_xy"] = pos
                kwargs["current_velocity"] = previous = velocity

    def test_direct_rejects_physical_or_aabb_paths_without_changing_legacy(self):
        kwargs = inputs(1)
        for extra in ({"bars_half_extents_xy": torch.full((2, 1, 2), 0.3)},
                      {"exact_aabb_clearance": True}):
            with self.subTest(extra=tuple(extra)):
                MOTION.bounded_drone_target_step(**kwargs, **extra)
                with self.assertRaisesRegex(ValueError, "virtual target"):
                    MOTION.bounded_drone_target_step(**kwargs, **extra, distance_backend="direct_l2_v1")

    def test_direct_does_not_mutate_inputs_rng_or_global_precision(self):
        kwargs = inputs(160)
        before = {key: value.clone() for key, value in kwargs.items() if isinstance(value, torch.Tensor)}
        rng = torch.random.get_rng_state().clone()
        matmul = torch.get_float32_matmul_precision()
        tf32 = torch.backends.cuda.matmul.allow_tf32
        result = MOTION.bounded_drone_target_step(**kwargs, distance_backend="direct_l2_v1")
        again = MOTION.bounded_drone_target_step(**kwargs, distance_backend="direct_l2_v1")
        assert_same(self, result, again)
        for key, value in before.items():
            self.assertTrue(torch.equal(value, kwargs[key]), key)
        self.assertTrue(torch.equal(rng, torch.random.get_rng_state()))
        self.assertEqual(matmul, torch.get_float32_matmul_precision())
        self.assertEqual(tf32, torch.backends.cuda.matmul.allow_tf32)

    def test_config_default_opt_in_and_unknown_value_without_simulator_import(self):
        # Execute the real class statements alone: importing aerial_gym would import Isaac Gym.
        source = ast.parse((ROOT / "aerial_gym/config/task_config/navrl_task_config.py").read_text())
        task = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == "task_config")
        target = next(n for n in task.body if isinstance(n, ast.ClassDef) and n.name == "target_motion")
        statements = [n for n in target.body if (
            isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "distance_backend" for t in n.targets)
        ) or (isinstance(n, ast.If) and "distance_backend" in ast.dump(n.test))]
        self.assertEqual(len(statements), 2)
        code = compile(ast.Module(body=statements, type_ignores=[]), "distance_backend_config", "exec")
        for raw, want in ((None, "legacy_cdist"), ("direct_l2_v1", "direct_l2_v1")):
            env = {} if raw is None else {"NAVRL_TARGET_DISTANCE_BACKEND": raw}
            with mock.patch.dict(os.environ, env, clear=True):
                scope = {"os": os}
                exec(code, scope)
                self.assertEqual(scope["distance_backend"], want)
        with mock.patch.dict(os.environ, {"NAVRL_TARGET_DISTANCE_BACKEND": "unknown"}, clear=True):
            with self.assertRaises(ValueError):
                exec(code, {"os": os})


if __name__ == "__main__":
    unittest.main()
