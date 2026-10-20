"""C02 opt-in model oracles; no simulator or physical-safety qualification.

Inputs are specified by independent scalar/trigonometric arithmetic, not a BOIA
rollout/helper used as its own answer. Legacy checks are comparison arms only.
"""
import ast
import cmath
from dataclasses import replace
import hashlib
import math
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import safety  # noqa: E402
from motar_eval.boia.types import (BoiaParams, FreeContext, Rollout, SafetyStatus,
                                    SweepBounds)  # noqa: E402

assert Path(safety.__file__).resolve() == ROOT / "motar_eval/boia/safety.py"
RADIUS = math.sqrt(.14 ** 2 + .14 ** 2 + .04 ** 2)


def params(**kwargs):
    return BoiaParams(tau_on=.1, r_bar_est=.2, **kwargs)


def context(radius=12., yaw=0., n=1):
    angles = torch.tensor([math.pi - j * math.pi / 36 for j in range(72)], dtype=torch.float64)
    pos = torch.zeros(n, 3, dtype=angles.dtype)
    pos[:, 2] = 1.
    radii = torch.full((n, 72), float(radius), dtype=angles.dtype)
    return FreeContext(pos, torch.full((n,), float(yaw), dtype=angles.dtype), radii, angles, 12.)


def empty_points(n=1):
    return torch.empty(n, 0, 2, dtype=torch.float64), torch.empty(n, 0, dtype=torch.bool)


def scalar_roll(v0=(0., 0.), target=(0., 0.), p0=(0., 0.), yaw=0., h=10, sub=10, accel=9.81):
    """Independent semi-implicit scalar recurrence, including complete .1 s intervals."""
    p, v, positions, velocities = list(p0), list(v0), [], []
    for _ in range(h * sub):
        a = [(target[i] - v[i]) / .4 for i in range(2)]
        norm = math.hypot(*a)
        if norm > accel:
            a = [x * accel / norm for x in a]
        v = [v[i] + .1 / sub * a[i] for i in range(2)]
        p = [p[i] + .1 / sub * v[i] for i in range(2)]
        positions.append(p[:] + [1.])
        velocities.append(v[:] + [0.])
    pos = torch.tensor(positions, dtype=torch.float64).reshape(1, 1, h, sub, 3)
    vel = torch.tensor(velocities, dtype=torch.float64).reshape_as(pos)
    psi = torch.full(pos.shape[:-1], float(yaw), dtype=pos.dtype)
    commands = torch.zeros(1, 1, h, 4, dtype=pos.dtype)
    commands[..., 0] = math.cos(yaw) * target[0] + math.sin(yaw) * target[1]
    commands[..., 1] = -math.sin(yaw) * target[0] + math.cos(yaw) * target[1]
    return Rollout(pos, vel, psi, pos[:, :, :, -1], vel[:, :, :, -1], psi[:, :, :, -1], commands,
                   {"initial_pos": torch.tensor([[list(p0) + [1.]]], dtype=pos.dtype),
                    "initial_vel": torch.tensor([[list(v0) + [0.]]], dtype=pos.dtype)})


def repeated(roll, n):
    expand = lambda x: x.expand((n,) + tuple(x.shape[1:])).clone()
    return Rollout(*(expand(x) for x in (roll.pos, roll.vel, roll.psi, roll.step_pos,
                   roll.step_vel, roll.step_psi, roll.governed_cmd)),
                   {key: expand(value) for key, value in roll.diagnostics.items()})


def cast_roll(roll, dtype):
    return Rollout(*(x.to(dtype) for x in (roll.pos, roll.vel, roll.psi, roll.step_pos,
                   roll.step_vel, roll.step_psi, roll.governed_cmd)),
                   {key: value.to(dtype) for key, value in roll.diagnostics.items()})


def run(roll, ctx=None, points=None, config=None, bounds=None):
    pts, valid = empty_points(roll.pos.shape[0]) if points is None else points
    return safety.screen_next_cycle(roll, pts, valid, context(n=roll.pos.shape[0]) if ctx is None else ctx,
                                    params() if config is None else config, bounds=bounds)


class NextCycleOracleTest(unittest.TestCase):
    def test_center_only_false_safe_and_sector_boundary_penetration(self):
        ctx = context()
        radii = ctx.free_radius.clone()
        radii[0, 35] = .5
        ctx = replace(ctx, free_radius=radii)
        roll = scalar_roll(p0=(1., 0.))
        legacy = safety.screen(roll, *empty_points(), ctx, params())
        self.assertTrue(bool(legacy[0]) and bool(legacy[1]))
        # Independent distance to the +2.5-degree radial step < body radius.
        self.assertLess(math.sin(math.radians(2.5)), RADIUS)
        # The physical box point (1,.1) crosses into that unknown sector.
        self.assertGreater(math.atan2(.1, 1.), math.radians(2.5))
        self.assertLess(.1, .14)
        out = run(roll, ctx)
        self.assertEqual(int(out.status), SafetyStatus.UNKNOWN)
        self.assertFalse(bool(out.prefix_ok) or bool(out.backup_ok))

    def test_nonconvex_free_space_corners_are_not_a_containment_oracle(self):
        ctx = context()
        radii = ctx.free_radius.clone()
        radii[0, 35:38] = .65
        ctx = replace(ctx, free_radius=radii)
        for x in (.46, .74):
            for y in (-.14, .14):
                self.assertGreater(abs(math.atan2(y, x)), math.radians(7.5))
                self.assertLess(math.hypot(x, y), 12.)
        self.assertLess(.6, .65)  # Centre and corners free.
        self.assertGreater(.74, .65)  # Front-face midpoint unknown.
        out = run(scalar_roll(p0=(.6, 0.)), ctx)
        self.assertEqual(int(out.status), SafetyStatus.UNKNOWN)
        self.assertFalse(bool(out.prefix_ok))

    def test_narrow_gap_low_high_speed_and_declared_radius_allowance(self):
        points = torch.tensor([[[x, y] for x in (-1., 0., .2, .4, .6, 1., 2., 3.)
                                for y in (-.275, .275)]], dtype=torch.float64)
        obs = (points, torch.ones(points.shape[:2], dtype=torch.bool))
        for speed, expected in ((.6, SafetyStatus.MODEL_SAFE), (2., SafetyStatus.MODEL_UNSAFE)):
            with self.subTest(speed=speed):
                oracle_radius = RADIUS + .09 * speed
                self.assertEqual(oracle_radius < .275, expected == SafetyStatus.MODEL_SAFE)
                out = run(scalar_roll((speed, 0.), (speed, 0.)), points=obs)
                self.assertEqual(int(out.status), expected)
        out = run(scalar_roll((.6, 0.), (.6, 0.)), points=obs,
                  bounds=SweepBounds(position_error_m=.03))
        self.assertGreater(RADIUS + .09 * .6 + .03, .275)
        self.assertEqual(int(out.status), SafetyStatus.MODEL_UNSAFE)

    def test_original_raw_row2_bin25_horizon_pass_next_cycle_fail(self):
        bearing = math.atan2(.2, .97)
        yaw = bearing - math.radians(55)
        ctx = context(yaw=yaw)
        slant = math.hypot(.97, .2)
        # Independent raw four-row projection and neighbour contraction.
        rows = [[12.] * 72 for _ in range(4)]
        rows[2][25] = slant
        elevations = (20., 10., 0., -10.)
        base = [max(0., min(rows[v][j] * math.cos(math.radians(elevations[v]))
                            for v in range(4)) - .1) for j in range(72)]
        contracted = [min(base[j], base[(j - 1) % 72], base[(j + 1) % 72]) for j in range(72)]
        ctx = replace(ctx, free_radius=torch.tensor([contracted], dtype=torch.float64))
        point = (slant * math.cos(yaw + math.radians(55)), slant * math.sin(yaw + math.radians(55)))
        self.assertAlmostEqual(point[0], .97, places=14)
        self.assertAlmostEqual(point[1], .2, places=14)
        obs = (torch.tensor([[point]], dtype=torch.float64), torch.ones(1, 1, dtype=torch.bool))
        roll = scalar_roll((2., 0.), (0., 2.), yaw=yaw)
        hard, stop, _, _ = safety.screen(roll, *obs, ctx, params())
        self.assertTrue(bool(hard) and bool(stop))
        # Independent closed forms for 10 resolved substeps.
        q = .975 ** 10
        self.assertAlmostEqual(float(roll.pos[0, 0, 0, -1, 0]), .78 * (1 - q), places=14)
        out = run(roll, ctx, obs)
        self.assertTrue(bool(out.legacy_hard_ok) and bool(out.legacy_stop_ok))
        self.assertTrue(bool(out.prefix_ok))
        self.assertFalse(bool(out.backup_ok))
        self.assertEqual(int(out.status), SafetyStatus.MODEL_UNSAFE)
        # Actual zero-input discrete limit after .1s is (.78,.2), so this is not
        # merely an oversized table extension: the yawed physical box contains the point.
        dx, dy = .97 - .78, 0.
        body = (math.cos(yaw) * dx + math.sin(yaw) * dy,
                -math.sin(yaw) * dx + math.cos(yaw) * dy)
        self.assertTrue(all(abs(x) < .14 for x in body))

    def test_curved_prefix_uses_current_velocity_for_direction_preserving_backup(self):
        omega, delta, q, steps = .7, .01, .975, 100
        rot, gain = cmath.exp(1j * omega * delta), delta / .4
        pos, vel, yaws, accumulated = [], [], [], 0j
        for n in range(1, steps + 1):
            vn = gain * (rot ** n - q ** n) / (rot - q)
            sr = rot * (1 - rot ** n) / (1 - rot)
            sq = q * (1 - q ** n) / (1 - q)
            pn = delta * gain * (sr - sq) / (rot - q)
            # The analytic sum can lose several ULPs through complex cancellation.
            # Build the supplied U4 trace by independent scalar accumulation, and
            # retain the closed form as a separate numerical oracle, not a looser
            # kinematic tolerance inside the safety verifier.
            accumulated += delta * vn
            self.assertLess(abs(accumulated - pn), 64 * sys.float_info.epsilon * (1 + abs(pn)))
            pos.append([accumulated.real, accumulated.imag, 1.])
            vel.append([vn.real, vn.imag, 0.])
            yaws.append(omega * n * delta)
        roll = scalar_roll()
        p = torch.tensor(pos, dtype=torch.float64).reshape_as(roll.pos)
        v = torch.tensor(vel, dtype=torch.float64).reshape_as(roll.vel)
        psi = torch.tensor(yaws, dtype=torch.float64).reshape_as(roll.psi)
        roll = replace(roll, pos=p, vel=v, psi=psi, step_pos=p[:, :, :, -1],
                       step_vel=v[:, :, :, -1], step_psi=psi[:, :, :, -1])
        out = run(roll)
        self.assertEqual(int(out.status), SafetyStatus.MODEL_SAFE)
        current = v[0, 0, 0, -1]
        self.assertGreater(float(current[1]), 0.)
        trace = out.diagnostics["backup"]
        torch.testing.assert_close(trace["backup_velocities_mps"][0, 0, 2], q * current,
                                   rtol=0, atol=1e-15)
        expected = p[0, 0, 0, -1] + .1 * current + delta * q * current
        torch.testing.assert_close(trace["backup_positions_m"][0, 0, 2], expected, rtol=0, atol=1e-15)
        self.assertTrue(bool(trace["backup_complete_by_envelope"].all()))

    def test_sample_cap_has_a_complete_acceleration_saturated_remainder(self):
        speed, accel, tau = 3.52, 2., .4
        out = run(scalar_roll((speed, 0.), (speed, 0.), yaw=-math.pi / 4), config=params(a_max=accel))
        self.assertEqual(int(out.status), SafetyStatus.MODEL_SAFE)
        expected = .1 * speed + tau * speed + (speed - accel * tau) ** 2 / (2 * accel)
        self.assertAlmostEqual(float(out.backup_distance_m), expected, places=12)
        trace = out.diagnostics["backup"]
        self.assertEqual(trace["backup_sample_cap"], 64)
        self.assertGreater(float(trace["backup_velocities_mps"][0, 0, -1, 0]), 0.)
        self.assertGreaterEqual(float(trace["backup_remaining_cap_m"]),
                                float(trace["backup_remaining_continuous_bound_m"]))

    def test_zero_speed_near_boundary_and_unknown_only_environment(self):
        for x, expected in ((.79, SafetyStatus.MODEL_SAFE), (.81, SafetyStatus.UNKNOWN)):
            with self.subTest(x=x):
                out = run(scalar_roll(p0=(x, 0.)), context(radius=1.))
                self.assertEqual(x + RADIUS <= 1, expected == SafetyStatus.MODEL_SAFE)
                self.assertEqual(int(out.status), expected)
                self.assertEqual(float(out.backup_distance_m), 0.)
        out = run(scalar_roll(), context(radius=0.))
        self.assertEqual(int(out.status), SafetyStatus.UNKNOWN)
        self.assertFalse(bool(out.model_safe))

    def test_missing_bounds_and_supplied_unsupported_bounds_are_explicit(self):
        out = run(scalar_roll())
        self.assertEqual(int(out.status), SafetyStatus.MODEL_SAFE)
        self.assertEqual(len(out.diagnostics["missing_uncertainty_bounds"]), 5)
        self.assertEqual(out.physical_qualification, "PHYSICAL_QUALIFICATION_NOT_AVAILABLE")
        for bounds in (SweepBounds(latency_s=.02), SweepBounds(attitude_error_rad=.01)):
            with self.subTest(bounds=bounds):
                out = run(scalar_roll(), bounds=bounds)
                self.assertEqual(int(out.status), SafetyStatus.UNKNOWN)
                self.assertTrue(out.diagnostics["unsupported_bounds"])

    def test_vertical_fov_blindspot_and_real_rotor_shape_remain_unqualified(self):
        # At x=.4,z=1.03, a thin obstacle lies between 0 and +10deg LiDAR rows,
        # inside the level box's +/- .04 vertical extent. A rearward instance is
        # also outside the forward RGB-D FOV. No represented point means C02 cannot fix it.
        self.assertLess(.03, .04)
        self.assertGreater(math.atan2(.03, .4), 0.)
        self.assertLess(math.atan2(.03, .4), math.radians(10))
        out = run(scalar_roll())
        self.assertEqual(out.physical_qualification, "PHYSICAL_QUALIFICATION_NOT_AVAILABLE")
        # A .01 visual motor radius is not evidence of the actual propeller radius.
        self.assertAlmostEqual(.14 - .13, .01)
        self.assertEqual(len(out.diagnostics["missing_uncertainty_bounds"]), 5)

    def test_per_environment_invalid_isolation_and_batch_independence(self):
        healthy = run(scalar_roll())
        for poison in ("position", "context", "speed_overflow"):
            roll, ctx = repeated(scalar_roll(), 2), context(n=2)
            if poison == "context":
                ctx.free_radius[0, 5] = float("nan")
            elif poison == "speed_overflow":
                roll.vel[0, ..., 0] = 1e200
                roll.step_vel[0, ..., 0] = 1e200
                roll.diagnostics["initial_vel"][0, ..., 0] = 1e200
            else:
                roll.pos[0, 0, 0, 0, 0] = float("nan")
            out = run(roll, ctx)
            self.assertEqual(int(out.status[0]), SafetyStatus.INVALID)
            self.assertEqual(int(out.status[1]), int(healthy.status[0]))
            self.assertEqual(bool(out.prefix_ok[1]), bool(healthy.prefix_ok[0]))
            self.assertEqual(float(out.backup_distance_m[1]), float(healthy.backup_distance_m[0]))

    def test_float32_body_floor_rounddown_cannot_create_a_point_certificate(self):
        roll, ctx = cast_roll(scalar_roll(), torch.float32), context()
        ctx = FreeContext(*(x.to(torch.float32) for x in
                          (ctx.pos, ctx.yaw, ctx.free_radius, ctx.angles)), ctx.far_m)
        nearest = float(torch.tensor(RADIUS, dtype=torch.float32))
        self.assertLess(nearest, RADIUS)  # About 1e-9 m inside the verified body.
        points = torch.tensor([[[nearest, 0.]]], dtype=torch.float32)
        valid = torch.ones(1, 1, dtype=torch.bool)
        legacy = safety.screen(roll, points, valid, ctx, params())
        self.assertTrue(bool(legacy[0]) and bool(legacy[1]))
        out = safety.screen_next_cycle(roll, points, valid, ctx, params())
        self.assertEqual(int(out.status), SafetyStatus.MODEL_UNSAFE)
        self.assertEqual(out.status.device.type, "cpu")
        self.assertEqual(out.prefix_min_clearance_m.dtype, torch.float32)
        self.assertEqual(out.diagnostics["verification_dtype"], "float64")
        self.assertEqual(out.diagnostics["prefix_radius_m"].dtype, torch.float64)
        self.assertGreaterEqual(float(out.diagnostics["prefix_radius_m"].min()), RADIUS)
        self.assertLess(float(out.prefix_min_clearance_m), 0.)

    def test_shapes_and_cpu_only_transport_fail_explicitly(self):
        roll = scalar_roll()
        malformed = replace(roll, step_vel=roll.step_vel[:, :, :1])
        self.assertEqual(int(run(malformed).status), SafetyStatus.INVALID)
        with self.assertRaises(ValueError):
            run(repeated(roll, 0))
        # Meta tensors exercise the device guard without allocating or running a GPU.
        with self.assertRaises(ValueError):
            run(replace(roll, pos=torch.empty_like(roll.pos, device="meta")))

    def test_mismatched_prefix_time_grid_is_invalid_without_poisoning_other_env(self):
        healthy = scalar_roll((.6, 0.), (.6, 0.))
        roll = repeated(healthy, 2)
        doubled = roll.pos.clone()
        doubled[0, ..., :2] *= 2  # Implies .02 s substeps, with the same velocity trace.
        roll = replace(roll, pos=doubled, step_pos=doubled[:, :, :, -1])
        out = run(roll)
        self.assertEqual(out.status[:, 0].tolist(), [SafetyStatus.INVALID, SafetyStatus.MODEL_SAFE])
        self.assertEqual(out.diagnostics["prefix_kinematic_consistent"].tolist(), [False, True])
        self.assertGreater(float(out.diagnostics["prefix_kinematic_max_residual_m"][0]), .005)
        self.assertIn("NO_MEASURED_TIMESTAMP", out.diagnostics["timing_basis"])
        reference = run(healthy)
        self.assertEqual(float(out.backup_distance_m[1]), float(reference.backup_distance_m[0]))
        healthy32, ctx32 = cast_roll(healthy, torch.float32), context()
        ctx32 = FreeContext(*(x.to(torch.float32) for x in
                            (ctx32.pos, ctx32.yaw, ctx32.free_radius, ctx32.angles)), ctx32.far_m)
        checked32 = safety.screen_next_cycle(healthy32, torch.empty(1, 0, 2, dtype=torch.float32),
                                             torch.empty(1, 0, dtype=torch.bool), ctx32, params())
        self.assertEqual(int(checked32.status), SafetyStatus.MODEL_SAFE)
        self.assertTrue(bool(checked32.diagnostics["prefix_kinematic_consistent"].all()))

    def test_legacy_functions_policy_inputs_and_default_path_are_unchanged(self):
        expected = {"screen": "f6155f349e444505a550a59ffd42667e51a76df0d386c298cd3a83db763ad3c2",
                    "_stopping_envelope": "56f667f0a948f046eadf55e29e42b0a942f13e4fd24130d9e187db5c4ae07132"}
        source = Path(safety.__file__).read_text()
        for node in ast.parse(source).body:
            if isinstance(node, ast.FunctionDef) and node.name in expected:
                self.assertEqual(hashlib.sha256(ast.get_source_segment(source, node).encode()).hexdigest(),
                                 expected[node.name])
        roll, ctx = scalar_roll((.6, 0.), (.6, 0.)), context()
        originals = [x.clone() for x in (roll.pos, roll.vel, roll.psi, roll.governed_cmd, ctx.free_radius)]
        before = safety.screen(roll, *empty_points(), ctx, params())
        run(roll, ctx)
        after = safety.screen(roll, *empty_points(), ctx, params())
        self.assertTrue(all(torch.equal(x, y) for x, y in zip(before, after)))
        self.assertTrue(all(torch.equal(x, y) for x, y in zip(originals,
                            (roll.pos, roll.vel, roll.psi, roll.governed_cmd, ctx.free_radius))))
        # Source/input preservation only; this CPU test makes no actor-trajectory claim.

    def test_repeatability_and_no_global_rng_contamination(self):
        roll = scalar_roll((.6, 0.), (.6, 0.))
        state = torch.get_rng_state().clone()
        first, second = run(roll), run(roll)
        self.assertTrue(torch.equal(first.status, second.status))
        self.assertTrue(torch.equal(first.backup_distance_m, second.backup_distance_m))
        self.assertTrue(torch.equal(first.diagnostics["backup"]["backup_positions_m"],
                                    second.diagnostics["backup"]["backup_positions_m"]))
        self.assertTrue(torch.equal(state, torch.get_rng_state()))


if __name__ == "__main__":
    unittest.main()
