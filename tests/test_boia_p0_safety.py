"""P0-C/D independent physical-footprint and braking CPU regressions.

Box tests parse the frozen collision asset and use independent rigid transforms;
the stopping reference is scalar arithmetic, not the implementation's helper.
"""
import itertools
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import dynamics, safety, types  # noqa: E402
from motar_eval.boia.types import BoiaParams, FreeContext, Rollout  # noqa: E402

for module in (dynamics, safety, types):
    assert Path(module.__file__).resolve().parent == ROOT / "motar_eval/boia"

ASSET = ROOT / "resources/robots/quad/quad_navrl_collide.urdf"
SPEEDS = (0., .6, 1.2, 2., 2.5, 3.52)


def half_dimensions():
    collision = ET.parse(ASSET).getroot().findall("./link/collision")
    assert len(collision) == 1
    assert collision[0].find("origin").attrib["xyz"] == "0 0 0"
    return tuple(float(x) / 2 for x in collision[0].find("geometry/box").attrib["size"].split())


def params(**kwargs):
    return BoiaParams(tau_on=.1, r_bar_est=.2, **kwargs)


def total_stop_reference(v):
    """Independent scalar table law including one 0.1 s reaction allowance."""
    if v < 2:
        return .205 * v * v + .1 * v
    if v <= 2.5:
        return .82 + (v - 2) / .5 * .23 + .1 * v
    if v <= 3.52:
        return 1.05 + (v - 2.5) / 1.02 * .52 + .1 * v
    return 1.57 + (v * v - 3.52 ** 2) * .205 + .1 * v


def fixture(speed=0., dtype=torch.float64):
    initial = torch.zeros(1, 1, 3, dtype=dtype)
    pos = initial.reshape(1, 1, 1, 1, 3).clone()
    pos[..., 0] = speed * .01
    vel = torch.zeros_like(pos)
    vel[..., 0] = speed
    yaw = torch.full(pos.shape[:-1], -math.pi / 4, dtype=dtype)
    roll = Rollout(pos, vel, yaw, pos[:, :, :, -1], vel[:, :, :, -1], yaw[:, :, :, -1],
                   torch.zeros(1, 1, 1, 4, dtype=dtype),
                   {"initial_pos": initial, "initial_vel": vel[:, :, 0, 0].clone()})
    angles = torch.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72, dtype=dtype)
    ctx = FreeContext(torch.zeros(1, 3, dtype=dtype), torch.zeros(1, dtype=dtype),
                      torch.full((1, 72), 12., dtype=dtype), angles, 12.)
    return roll, ctx


def evaluate(point, speed=0., config=None, dtype=torch.float64):
    roll, ctx = fixture(speed, dtype)
    points = torch.tensor([[point]], dtype=dtype)
    return safety.screen(roll, points, torch.ones(1, 1, dtype=torch.bool), ctx,
                         params() if config is None else config)


def tilted_rotation():
    """Rotate a box corner into the horizontal x axis, with only 11.42 deg tilt."""
    hx, hy, hz = half_dimensions()
    yaw, pitch = -math.atan2(hy, hx), math.atan2(hz, math.hypot(hx, hy))
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    return ((cp * cy, -cp * sy, sp), (sy, cy, 0.), (-sp * cy, sp * sy, cp))


class PhysicalFootprintTest(unittest.TestCase):
    def test_source_box_and_orientation_independent_default(self):
        hx, hy, hz = half_dimensions()
        self.assertEqual((hx, hy, hz), (.14, .14, .04))
        radius = math.sqrt(hx * hx + hy * hy + hz * hz)
        self.assertGreaterEqual(params().r_hard, radius)
        self.assertAlmostEqual(types.VERIFIED_COLLISION_RADIUS_M, radius, places=15)

    def test_level_yaw_corner_inside_box_is_not_clear_at_rest(self):
        # Inverse yaw transform of world point (.18,0,0): both body XY < .14.
        body = (.18 / math.sqrt(2), .18 / math.sqrt(2), 0.)
        self.assertTrue(all(abs(x) < h for x, h in zip(body, half_dimensions())))
        hard, stop, _, _ = evaluate((.18, 0.))
        self.assertFalse(bool(hard))
        self.assertFalse(bool(stop))

    def test_tilted_corner_collision_all_six_speeds(self):
        # A point at x=.200 is outside the level XY radius but INSIDE a tilted box.
        rotation = tilted_rotation()
        body = tuple(rotation[0][j] * .200 for j in range(3))  # R^T (.200,0,0)
        self.assertTrue(all(abs(x) < h for x, h in zip(body, half_dimensions())))
        self.assertGreater(.200, math.hypot(*half_dimensions()[:2]))
        for speed in SPEEDS:
            with self.subTest(speed=speed):
                hard, _, _, _ = evaluate((.200, 0.), speed)
                self.assertFalse(bool(hard))

    def test_old_xy_radius_is_insufficient_even_when_explicitly_configured(self):
        legacy_xy = params(r_hard=.198, r_body=.198)
        hard, stop, _, _ = evaluate((.200, 0.), config=legacy_xy)
        self.assertFalse(bool(hard))
        self.assertFalse(bool(stop))

    def test_configured_smaller_radii_cannot_shrink_physical_floor(self):
        hard, stop, _, _ = evaluate((.200, 0.), config=params(r_hard=.01, r_body=.01))
        self.assertFalse(bool(hard))
        self.assertFalse(bool(stop))

    def test_physical_floor_plus_existing_delay_at_all_six_speeds(self):
        radius = math.sqrt(sum(x * x for x in half_dimensions()))
        for speed in SPEEDS:
            with self.subTest(speed=speed):
                # Analytic capsule threshold probe, not an actual controller simulation.
                point = (speed * .01, radius + speed * .09 - 1e-5)
                hard, _, _, _ = evaluate(point, speed, params(r_hard=.01, r_body=.01))
                self.assertFalse(bool(hard))

    def test_stop_tail_retains_full_footprint_at_all_six_speeds(self):
        for speed in SPEEDS:
            with self.subTest(speed=speed):
                # After empirical+reaction tail, a .200 m gap still intersects the 3-D bound.
                point = (speed * .01 + total_stop_reference(speed) + .200, 0.)
                _, stop, _, _ = evaluate(point, speed)
                self.assertFalse(bool(stop))

    def test_independent_transformed_vertices_are_bounded_at_arbitrary_attitudes(self):
        radius = math.sqrt(sum(x * x for x in half_dimensions()))
        for roll, pitch, yaw in itertools.product((-.7, 0., .7), repeat=3):
            cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                                     math.sin(pitch), math.cos(yaw), math.sin(yaw))
            # Independent Rz Ry Rx, no production transform or geometry helpers.
            r = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                 (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                 (-sp, cp * sr, cp * cr))
            for signs in itertools.product((-1, 1), repeat=3):
                v = tuple(s * h for s, h in zip(signs, half_dimensions()))
                world = tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))
                self.assertLessEqual(math.hypot(world[0], world[1]), radius + 1e-15)

    def test_larger_declared_radius_and_low_speed_gap_remain_meaningful(self):
        hard, stop, _, _ = evaluate((.35, 0.), config=params(r_hard=.4))
        self.assertFalse(bool(hard) or bool(stop))
        roll, ctx = fixture(.6)
        points = torch.tensor([[[0., .275], [.2, .275], [0., -.275], [.2, -.275]]], dtype=torch.float64)
        hard, stop, _, _ = safety.screen(roll, points, torch.ones(1, 4, dtype=torch.bool), ctx, params())
        self.assertTrue(bool(hard) and bool(stop))

    def test_purity_float32_float64_and_no_random_state_changes(self):
        before = torch.get_rng_state().clone()
        for dtype in (torch.float32, torch.float64):
            a, b = evaluate((.18, 0.), dtype=dtype), evaluate((.18, 0.), dtype=dtype)
            self.assertTrue(all(torch.equal(x, y) for x, y in zip(a, b)))
        self.assertTrue(torch.equal(before, torch.get_rng_state()))

    def test_negative_delay_cannot_undo_the_physical_floor(self):
        with self.assertRaises(ValueError):
            evaluate((.18, 0.), speed=1., config=params(t_det=-1.))

    def test_known_free_centerline_does_not_certify_body_containment(self):
        # Explicit unresolved counterexample: this test records a limitation, not safety GO.
        from dataclasses import replace
        roll, ctx = fixture()
        pos = roll.pos.clone()
        pos[..., 0] = 1.
        roll = replace(roll, pos=pos, step_pos=pos[:, :, :, -1],
                       diagnostics={"initial_pos": pos[:, :, 0, 0].clone(),
                                    "initial_vel": roll.vel[:, :, 0, 0].clone()})
        radii = ctx.free_radius.clone()
        radii[0, 35] = .5  # adjacent sector centred at +5 degrees.
        ctx = replace(ctx, free_radius=radii)
        hard, stop, _, _ = safety.screen(roll, torch.empty(1, 0, 2, dtype=pos.dtype),
                                         torch.empty(1, 0, dtype=torch.bool), ctx, params())
        self.assertTrue(bool(hard) and bool(stop))  # Current centre-only result.
        # Offset (0,.1) is inside the actual level box, but this body point is unknown.
        angle, norm = math.atan2(.1, 1.), math.hypot(1., .1)
        nearest = min(range(72), key=lambda i: abs(math.atan2(math.sin(angle - float(ctx.angles[i])),
                                                             math.cos(angle - float(ctx.angles[i])))))
        self.assertEqual(nearest, 35)
        self.assertGreater(norm, float(radii[0, nearest]))
        self.assertLess(.1, half_dimensions()[1])


class BrakingInterpretationTest(unittest.TestCase):
    def test_low_speed_stop_tail_cannot_contradict_own_zero_command_response(self):
        for speed, obstacle_offset in ((.6, .4), (1.2, .64)):
            with self.subTest(speed=speed):
                hard, stop, _, _ = evaluate((speed * .01 + obstacle_offset, 0.), speed)
                self.assertTrue(bool(hard))
                # Independent exact geometric series and inverse yaw transform show contact.
                distance = speed * .39 * (1 - .975 ** 800)
                body_xy = (obstacle_offset - distance) / math.sqrt(2)
                self.assertLess(abs(body_xy), .14)
                self.assertFalse(bool(stop))

    def test_saturated_braking_tail_is_screened_against_independent_scalar_steps(self):
        speed, tau, accel, delta = 3.52, .4, 2., .01
        v, distance = speed, 0.
        for _ in range(2000):
            v = max(0., v - delta * min(v / tau, accel))
            distance += delta * v
        continuous = tau * speed + max(speed - accel * tau, 0.) ** 2 / (2 * accel)
        self.assertGreaterEqual(continuous, distance)
        self.assertLess(continuous - distance, .04)
        self.assertGreater(distance, 2.5)
        hard, stop, _, _ = evaluate((speed * .01 + 2.5, 0.), speed, params(a_max=accel))
        self.assertTrue(bool(hard))
        self.assertFalse(bool(stop))

    def test_stop_envelope_reaction_is_once_in_each_branch_before_maximum(self):
        radius = math.sqrt(sum(x * x for x in half_dimensions()))
        for speed in SPEEDS:
            empirical = total_stop_reference(speed)
            own_bound = .1 * speed + .4 * speed + max(speed - 9.81 * .4, 0.) ** 2 / (2 * 9.81)
            total = max(empirical, own_bound)
            with self.subTest(speed=speed):
                for sign, expected in ((-1, False), (1, True)):
                    point = (speed * .01 + total + radius + sign * 1e-5, 0.)
                    _, stop, _, _ = evaluate(point, speed)
                    self.assertEqual(bool(stop), expected)

    def test_stopping_surrogate_requires_valid_parameters_and_resolved_substeps(self):
        for kwargs in ({"tau_v": 0.}, {"tau_v": float("nan")}, {"a_max": 0.},
                       {"a_max": float("inf")}, {"dt": .5}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                evaluate((2., 0.), .6, params(**kwargs))

    def test_cap_floor_is_not_a_minimum_commanded_velocity(self):
        speeds = torch.tensor(SPEEDS, dtype=torch.float64)
        commands = torch.stack((speeds, torch.zeros_like(speeds)), -1)
        output, diag = dynamics.apply_riskcap(commands, torch.zeros_like(speeds), dynamics.RiskcapConfig())
        torch.testing.assert_close(output[:, 0], speeds.clamp(max=2.), rtol=0, atol=0)
        self.assertTrue(bool((diag["speed_cap_mps"] == 2.).all()))
        self.assertEqual(output[:3, 0].tolist(), [0., .6, 1.2])

    def test_empirical_helper_reaction_once_and_telemetry_is_a_distinct_model(self):
        speeds = torch.tensor([2., 2.5, 3.52], dtype=torch.float64)
        empirical = torch.tensor([.82, 1.05, 1.57], dtype=torch.float64)
        total, _ = dynamics.braking_distance(speeds, t_react=.1)
        torch.testing.assert_close(total, empirical + .1 * speeds, rtol=0, atol=2e-16)
        cfg = dynamics.RiskcapConfig()
        _, diag = dynamics.apply_riskcap(torch.stack((speeds, speeds * 0), -1), speeds * 0 + 12, cfg)
        risk_reference = .1 * speeds + speeds.square() / (2 * 2.96)
        torch.testing.assert_close(12 - .45 - diag["stopping_margin_requested_m"], risk_reference,
                                   rtol=0, atol=2e-15)
        self.assertGreater(float((risk_reference - total).abs().max()), .5)

    def test_zero_command_is_not_an_instantaneous_stop_and_prior_errors_remain(self):
        speeds = torch.tensor([2., 2.5, 3.52], dtype=torch.float64)
        p, v = torch.zeros(3, 3, dtype=torch.float64), torch.zeros(3, 3, dtype=torch.float64)
        v[:, 0] = speeds
        out = dynamics.rollout(torch.zeros(3, 1, 4, dtype=p.dtype), p, v, torch.zeros(3, dtype=p.dtype),
                               torch.empty(3, 0, 2, dtype=p.dtype), torch.empty(3, 0, dtype=torch.bool),
                               dynamics.RiskcapConfig(), params(), H=80, S=10, dt=.1)
        ratio, steps = 1 - .01 / .4, 800
        independent = speeds * .01 * ratio * (1 - ratio ** steps) / (1 - ratio)
        distance = out.step_pos[:, 0, -1, 0]
        torch.testing.assert_close(distance, independent, rtol=0, atol=3e-14)
        self.assertTrue(bool((out.vel[:, 0, 0, 0, 0] > 0).all()))
        table = torch.tensor([.82, 1.05, 1.57], dtype=p.dtype)
        error = (distance - table) / table
        # 8 s has <2e-9 relative residual; report the exact limiting values separately.
        torch.testing.assert_close(error, torch.tensor([-.0487804878, -.0714285714, -.1256050955],
                                                       dtype=p.dtype), rtol=0, atol=2e-9)
        self.assertTrue(bool((error.abs() <= .25).all()))
        self.assertEqual((error.abs() > .10).tolist(), [False, False, True])


if __name__ == "__main__":
    unittest.main()
