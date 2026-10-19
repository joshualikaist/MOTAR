"""U5 synthetic CPU fixtures; no simulator, recorded-data calibration or new policy."""
import ast
from dataclasses import fields, replace
import math
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import candidates, safety  # noqa: E402
from motar_eval.boia.types import BoiaParams, CameraParams, FreeContext, OnlineInputs, Rollout  # noqa: E402

for module in (candidates, safety):
    assert str(Path(module.__file__).resolve()).startswith(str(ROOT) + "/")


def parameters(**kwargs):
    return BoiaParams(tau_on=.1, r_bar_est=.2, **kwargs)


def inputs(n=1, dtype=torch.float64):
    z = torch.zeros(n, dtype=dtype)
    pos, vel = torch.zeros(n, 3, dtype=dtype), torch.zeros(n, 3, dtype=dtype)
    quat = torch.zeros(n, 4, dtype=dtype)
    quat[:, 3] = 1
    mu = torch.zeros(n, 6, dtype=dtype)
    mu[:, 0] = 4
    angles = torch.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72, dtype=dtype)
    camera = CameraParams(math.pi / 2, math.pi / 3, 80, 80, 80, 45, (0., 0., 0.))
    action = torch.zeros(n, 4, dtype=dtype)
    return OnlineInputs(pos, vel, quat, z, torch.zeros(n, 3, dtype=dtype),
                        torch.full((n, 288), 12., dtype=dtype), angles,
                        torch.tensor((20., 10., 0., -10.), dtype=dtype) * math.pi / 180,
                        mu, torch.eye(6, dtype=dtype)[None].expand(n, 6, 6).clone(),
                        torch.ones(n, dtype=torch.bool), z.clone(), torch.ones(n, dtype=torch.bool),
                        action, camera, 12.)


def empty_points(n=1, dtype=torch.float64):
    return torch.empty(n, 0, 2, dtype=dtype), torch.empty(n, 0, dtype=torch.bool)


def free_context(dtype=torch.float64):
    inp = inputs(dtype=dtype)
    return FreeContext(inp.pos, inp.yaw, torch.full((1, 72), 12., dtype=dtype), inp.angles, 12.)


def synthetic_roll(points, velocity=(0., 0., 0.), initial=(0., 0., 0.)):
    pos = torch.tensor(points, dtype=torch.float64).reshape(1, 1, 1, -1, 3)
    vel = torch.tensor(velocity, dtype=torch.float64).reshape(1, 1, 1, 1, 3).expand_as(pos).clone()
    psi = torch.zeros(pos.shape[:-1], dtype=pos.dtype)
    diag = {"initial_pos": torch.tensor(initial, dtype=pos.dtype).reshape(1, 1, 3),
            "initial_vel": vel[:, :, 0, 0].clone()}
    return Rollout(pos, vel, psi, pos[:, :, :, -1], vel[:, :, :, -1], psi[:, :, :, -1],
                   torch.zeros(1, 1, 1, 4, dtype=pos.dtype), diag)


class CandidateTest(unittest.TestCase):
    def test_twenty_families_and_exact_policy_f_preservation(self):
        inp = inputs(2)
        action = torch.tensor([[.1, -.2, .6, .4], [-.2, .3, -.7, -.1]], dtype=torch.float64)
        commands, family, diag = candidates.generate(action, inp, inp.mu, *empty_points(2), parameters())
        self.assertEqual(tuple(commands.shape), (2, 20, 4))
        self.assertTrue(torch.equal(commands[:, 0], action))
        self.assertEqual(family.tolist(), [0, 1] + [2] * 15 + [3, 4, 5])
        self.assertTrue(bool((commands[:, 1:, 2] == 0).all()))
        self.assertTrue(bool((commands.abs() <= 1).all()))
        self.assertEqual(tuple(diag["arc_found"].shape), (2, 2))

    def test_lattice_speeds_bearings_and_frame_rotation_independently(self):
        inp = inputs()
        commands, _, _ = candidates.generate(inp.policy_action, inp, inp.mu, *empty_points(), parameters())
        for j, angle in enumerate((-40, -20, 0, 20, 40)):
            for k, speed in enumerate((.6, 1.2, 2.0)):
                actual = commands[0, 2 + j * 3 + k, :2] * 2.5
                expected = torch.tensor([speed * math.cos(math.radians(angle)),
                                         speed * math.sin(math.radians(angle))], dtype=actual.dtype)
                torch.testing.assert_close(actual, expected, rtol=0, atol=5e-16)
        yaw = torch.tensor([math.pi / 2], dtype=inp.yaw.dtype)
        mu = inp.mu.clone()
        mu[:, :2] = torch.tensor([0., 4.])
        vel = inp.vel.clone()
        vel[:, 0] = 1
        rotated = replace(inp, yaw=yaw, vel=vel, mu=mu)
        commands, _, _ = candidates.generate(inp.policy_action, rotated, mu, *empty_points(), parameters())
        torch.testing.assert_close(commands[0, 8, :2], torch.tensor([.24, 0.], dtype=yaw.dtype), atol=1e-15, rtol=0)
        torch.testing.assert_close(commands[0, 19, :2], torch.tensor([0., -.24], dtype=yaw.dtype), atol=1e-15, rtol=0)

    def test_arc_first_clear_offset_uses_independent_line_distance_fixture(self):
        inp = inputs()
        points = torch.tensor([[[2., 0.]]], dtype=inp.pos.dtype)
        commands, _, diag = candidates.generate(inp.policy_action, inp, inp.mu, points,
                                                torch.ones(1, 1, dtype=torch.bool), parameters())
        # For line (0,delta)->(4,0), distance from (2,0) is 2|delta|/sqrt(16+delta²).
        for smaller in (.25, .5, .75):
            self.assertLess(2 * smaller / math.sqrt(16 + smaller ** 2), .40)
        self.assertGreater(2 / math.sqrt(17), .40)
        torch.testing.assert_close(diag["arc_offsets_m"], torch.tensor([[1., -1.]], dtype=inp.pos.dtype))
        self.assertEqual(diag["arc_found"].tolist(), [[True, True]])
        torch.testing.assert_close(commands[0, 17:19, :2] * 2.5,
                                   torch.tensor([[1., 1.5], [1., -1.5]], dtype=inp.pos.dtype))

    def test_arc_no_clear_path_returns_zero_offsets_with_false_diagnostics(self):
        inp = inputs()
        points = torch.tensor([[[0., .1]]], dtype=inp.pos.dtype)
        commands, _, diag = candidates.generate(inp.policy_action, inp, inp.mu, points,
                                                torch.ones(1, 1, dtype=torch.bool), parameters())
        self.assertEqual(diag["arc_found"].tolist(), [[False, False]])
        self.assertTrue(bool((diag["arc_offsets_m"] == 0).all()))
        torch.testing.assert_close(commands[0, 17:19, :2], torch.tensor([[.4, 0.], [.4, 0.]], dtype=inp.pos.dtype))

    def test_inactive_nan_belief_uses_velocity_then_yaw_without_virtual_goal(self):
        inp = inputs(2)
        vel = inp.vel.clone()
        vel[0, 1] = 2
        yaw = torch.tensor([0., math.pi / 2], dtype=inp.pos.dtype)
        mu = torch.full_like(inp.mu, float("nan"))
        inp = replace(inp, mu=mu, active=torch.zeros(2, dtype=torch.bool), vel=vel, yaw=yaw)
        commands, _, diag = candidates.generate(inp.policy_action, inp, mu, *empty_points(2), parameters())
        self.assertTrue(bool(torch.isfinite(commands).all()))
        self.assertEqual(diag["inactive_belief"].tolist(), [True, True])
        self.assertEqual(diag["heading_from_velocity"].tolist(), [True, False])
        self.assertFalse(bool(diag["arc_found"].any()))
        self.assertTrue(bool((diag["arc_offsets_m"] == 0).all()))
        torch.testing.assert_close(commands[0, 17:19, :2], torch.tensor([[0., .4], [0., .4]], dtype=inp.pos.dtype))
        torch.testing.assert_close(commands[1, 17:19, :2], torch.tensor([[.4, 0.], [.4, 0.]], dtype=inp.pos.dtype), atol=1e-15, rtol=0)
        self.assertTrue(bool((commands[:, 1] == 0).all()))

    def test_coincident_active_belief_is_finite_and_explicitly_abstains(self):
        inp = inputs()
        inp = replace(inp, mu=torch.zeros_like(inp.mu))
        commands, _, diag = candidates.generate(inp.policy_action, inp, inp.mu, *empty_points(), parameters())
        self.assertTrue(bool(torch.isfinite(commands).all()))
        self.assertEqual(diag["direction_valid"].tolist(), [False])
        self.assertFalse(bool(diag["arc_found"].any()))

    def test_invalid_f_is_rejected_instead_of_silently_clipped(self):
        inp = inputs()
        for value in (1.01, float("nan")):
            action = inp.policy_action.clone()
            action[0, 0] = value
            with self.assertRaises(ValueError):
                candidates.generate(action, inp, inp.mu, *empty_points(), parameters())

    def test_inputs_are_not_mutated_and_global_rng_is_unchanged(self):
        inp = inputs(2, torch.float32)
        snapshots = {f.name: getattr(inp, f.name).clone() for f in fields(inp) if isinstance(getattr(inp, f.name), torch.Tensor)}
        points, valid = empty_points(2, torch.float32)
        before = torch.get_rng_state().clone()
        a = candidates.generate(inp.policy_action, inp, inp.mu, points, valid, parameters())
        b = candidates.generate(inp.policy_action, inp, inp.mu, points, valid, parameters())
        self.assertTrue(torch.equal(a[0], b[0]))
        self.assertTrue(torch.equal(before, torch.get_rng_state()))
        for key, value in snapshots.items():
            self.assertTrue(torch.equal(value, getattr(inp, key)), key)


class SafetyTest(unittest.TestCase):
    def test_swept_crossing_rejected_even_when_substep_endpoints_are_clear(self):
        roll = synthetic_roll([[1., 0., 0.]], initial=(-1., 0., 0.))
        points = torch.tensor([[[0., 0.]]], dtype=torch.float64)
        hard, stop, clearance, _ = safety.screen(roll, points, torch.ones(1, 1, dtype=torch.bool), free_context(), parameters())
        self.assertFalse(bool(hard))
        self.assertTrue(bool(stop))
        self.assertEqual(float(clearance), 0.)

    def test_narrow_gap_accepts_low_speed_but_rejects_latency_margin_at_high_speed(self):
        points = torch.tensor([[[-1., .275], [0., .275], [1., .275], [2., .275],
                                [-1., -.275], [0., -.275], [1., -.275], [2., -.275]]], dtype=torch.float64)
        valid = torch.ones(1, 8, dtype=torch.bool)
        results = []
        for speed in (.6, 2.):
            roll = synthetic_roll([[0., 0., 0.], [.5, 0., 0.]], velocity=(speed, 0., 0.), initial=(-.5, 0., 0.))
            hard, stop, clearance, soft = safety.screen(roll, points, valid, free_context(), parameters())
            results.append(bool(hard))
            self.assertTrue(bool(stop))
            self.assertAlmostEqual(float(clearance), .275)
            self.assertAlmostEqual(float(soft), .398 - .275)
        self.assertEqual(results, [True, False])

    def test_stop_two_mps_point_eight_ahead_rejected_and_reaction_added_once(self):
        roll = synthetic_roll([[0., 0., 0.]], velocity=(2., 0., 0.), initial=(-.2, 0., 0.))
        for obstacle_x, expected in ((.8, False), (1.25, True)):
            points = torch.tensor([[[obstacle_x, 0.]]], dtype=torch.float64)
            hard, stop, _, _ = safety.screen(roll, points, torch.ones(1, 1, dtype=torch.bool), free_context(), parameters())
            self.assertTrue(bool(hard))
            self.assertEqual(bool(stop), expected)

    def test_stop_is_exact_segment_not_endpoint_test(self):
        roll = synthetic_roll([[0., 0., 0.]], velocity=(2., 0., 0.), initial=(-.1, 0., 0.))
        points = torch.tensor([[[.6, 0.]]], dtype=torch.float64)
        hard, stop, _, _ = safety.screen(roll, points, torch.ones(1, 1, dtype=torch.bool), free_context(), parameters())
        self.assertTrue(bool(hard))
        self.assertFalse(bool(stop))

    def test_unknown_blocks_even_when_visibility_ablation_is_off(self):
        roll = synthetic_roll([[1., 10., 0.]], initial=(1., -10., 0.))
        ctx = free_context()
        radius = ctx.free_radius.clone()
        radius[0, 35] = .5
        ctx = replace(ctx, free_radius=radius)
        hard, _, _, _ = safety.screen(roll, *empty_points(), ctx, parameters(unknown_blocks=False))
        self.assertFalse(bool(hard))
        roll = synthetic_roll([[11.5, 0., 0.]], velocity=(2., 0., 0.), initial=(11.4, 0., 0.))
        hard, stop, _, _ = safety.screen(roll, *empty_points(), free_context(), parameters())
        self.assertTrue(bool(hard))
        self.assertFalse(bool(stop))

    def test_zero_velocity_empty_scan_and_invalid_points_are_well_defined(self):
        roll = synthetic_roll([[0., 0., 0.]])
        points = torch.tensor([[[float("nan"), float("nan")]]], dtype=torch.float64)
        before = roll.pos.clone()
        hard, stop, clearance, soft = safety.screen(roll, points, torch.zeros(1, 1, dtype=torch.bool), free_context(), parameters())
        self.assertTrue(bool(hard) and bool(stop))
        self.assertTrue(bool(torch.isinf(clearance).all()))
        self.assertEqual(float(soft), 0.)
        self.assertTrue(torch.equal(before, roll.pos))
        with self.assertRaises(AssertionError):
            safety.screen(replace(roll, diagnostics=None), *empty_points(), free_context(), parameters())
        with self.assertRaises(AssertionError):
            safety.screen(replace(roll, diagnostics={"initial_pos": roll.diagnostics["initial_pos"]}),
                          *empty_points(), free_context(), parameters())

    def test_public_shapes_multiple_candidates_horizons_and_cpu_purity(self):
        roll = synthetic_roll([[0., 0., 0.], [.1, 0., 0.]])
        pos = roll.pos.expand(1, 2, 3, 2, 3).clone()
        vel, psi = torch.zeros_like(pos), torch.zeros(pos.shape[:-1], dtype=pos.dtype)
        roll = Rollout(pos, vel, psi, pos[:, :, :, -1], vel[:, :, :, -1], psi[:, :, :, -1],
                       torch.zeros(1, 2, 3, 4, dtype=pos.dtype),
                       {"initial_pos": torch.zeros(1, 2, 3, dtype=pos.dtype),
                        "initial_vel": torch.zeros(1, 2, 3, dtype=pos.dtype)})
        before = torch.get_rng_state().clone()
        out = safety.screen(roll, *empty_points(), free_context(), parameters())
        self.assertEqual([tuple(x.shape) for x in out], [(1, 2), (1, 2), (1, 2, 3), (1, 2)])
        self.assertTrue(bool(out[0].all() and out[1].all()))
        self.assertTrue(torch.equal(before, torch.get_rng_state()))
        self.assertFalse(any(bool(torch.isnan(x).any()) for x in out))

    def test_static_pure_import_and_distance_contract(self):
        for module in (candidates, safety):
            source = Path(module.__file__).read_text()
            tree = ast.parse(source)
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    self.assertTrue(all(x.name in {"math", "torch"} for x in node.names))
                if isinstance(node, ast.ImportFrom):
                    self.assertTrue(node.level == 1 and node.module in {"types", "geometry", "dynamics"})
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    self.assertNotIn(node.func.attr, {"cdist", "matmul", "rand", "randn", "manual_seed"})
            self.assertNotIn("@", source)


if __name__ == "__main__":
    unittest.main()
