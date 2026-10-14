"""KROC2027 evaluation core: time/frame contract, motion gates G1/G2, acquisition events, records.

CPU only. Nothing here imports aerial_gym or Isaac Gym; the bounded target limiter is loaded by file
path, exactly as tools/audits/audit_e2_target_validity.py does.
"""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motar_eval import contracts, events, scene_record  # noqa: E402
from motar_eval.contracts import (  # noqa: E402
    FrameStamp,
    GateTolerances,
    MotionGateMonitor,
    MotionLimits,
    float32_ulp,
)

_MOTION_PATH = ROOT / "aerial_gym/task/navrl_task/target_motion.py"


def _load_target_motion():
    spec = importlib.util.spec_from_file_location("_kroc2027_target_motion", _MOTION_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _waypoint_motion(n=512, steps=300, seed=0, lo=32.0, hi=39.5):
    """Fault-free bounded motion near 40 m, produced by the real float32 limiter.

    Returns float32 positions [steps+1, n, 2] and generator velocities [steps+1, n, 2].
    """
    tm = _load_target_motion()
    gen = torch.Generator().manual_seed(seed)
    f32 = torch.float32
    pos = (lo + (hi - lo) * torch.rand(n, 2, generator=gen)).to(f32)
    vel = torch.zeros(n, 2, dtype=f32)
    goal = (lo + (hi - lo) * torch.rand(n, 2, generator=gen)).to(f32)
    lim = torch.full((n,), 1.5, dtype=f32)
    acc = torch.full((n,), 4.0, dtype=f32)
    turn = torch.full((n,), math.radians(150.0), dtype=f32)
    positions, velocities = [pos.clone()], [vel.clone()]
    for _ in range(steps):
        near = (goal - pos).norm(dim=1) < 0.5
        fresh = (lo + (hi - lo) * torch.rand(n, 2, generator=gen)).to(f32)
        goal = torch.where(near.unsqueeze(1), fresh, goal)
        desired = goal - pos
        desired = desired / desired.norm(dim=1, keepdim=True).clamp(min=1e-6) * 1.5
        vel = tm.limit_planar_velocity(vel, desired.to(f32), lim, 0.1, acc, turn)
        pos = pos + vel * 0.1
        positions.append(pos.clone())
        velocities.append(vel.clone())
    return torch.stack(positions), torch.stack(velocities)


def _run_monitor(positions, velocities, nominal=1.5):
    n = positions.shape[1]
    monitor = MotionGateMonitor(n)
    monitor.reset(torch.arange(n), positions[0], velocities[0])
    nominal_t = torch.full((n,), nominal)
    for t in range(1, positions.shape[0]):
        monitor.step(positions[t], velocities[t], nominal_t)
    return monitor


class Float32UlpTest(unittest.TestCase):
    def test_known_spacings(self):
        self.assertEqual(float32_ulp(40.0), 2.0 ** -18)
        self.assertEqual(float32_ulp(32.0), 2.0 ** -18)
        self.assertEqual(float32_ulp(31.99), 2.0 ** -19)
        self.assertEqual(float32_ulp(1.0), 2.0 ** -23)
        self.assertEqual(float32_ulp(-40.0), 2.0 ** -18)

    def test_matches_numpy_spacing(self):
        import numpy as np
        for value in (0.3, 1.0, 7.5, 16.0, 39.9, 63.9, 64.0, 500.0):
            self.assertEqual(float32_ulp(value), float(np.spacing(np.float32(value))))

    def test_rejects_nonfinite(self):
        with self.assertRaises(ValueError):
            float32_ulp(float("inf"))


class ToleranceDerivationTest(unittest.TestCase):
    def test_pre_registered_values(self):
        tol = GateTolerances()
        dt = 0.1
        u = 2.0 ** -18
        self.assertAlmostEqual(tol.ulp_m(), u)
        self.assertAlmostEqual(tol.g2_speed_mps(dt), 2 * math.sqrt(2) * (u / 2) / dt)
        self.assertAlmostEqual(tol.g2_accel_mps2(dt), 2 * math.sqrt(2) * u / dt ** 2)
        self.assertAlmostEqual(tol.g2_position_m(), 2 * math.sqrt(2) * (u / 2))
        self.assertAlmostEqual(tol.g2_speed_mps(dt), 5.394e-5, delta=1e-8)
        self.assertAlmostEqual(tol.g2_accel_mps2(dt), 1.0789e-3, delta=1e-7)
        self.assertAlmostEqual(tol.g2_position_m(), 5.394e-6, delta=1e-9)
        turn_deg = math.degrees(tol.g2_turn_rad_s(0.20, dt))
        self.assertAlmostEqual(turn_deg, 0.309, delta=1e-3)

    def test_describe_is_complete(self):
        described = GateTolerances().describe(0.1)
        for key in ("ulp_m", "g2_speed_mps", "g2_accel_mps2", "g2_position_m",
                    "g2_turn_deg_s_at_min_speed", "g1_relative"):
            self.assertIn(key, described)


class FrameStampTest(unittest.TestCase):
    def test_time_and_tick(self):
        stamp = FrameStamp(action_index=7, rl_dt_s=0.1, physics_steps_per_action=10,
                           frame="world_env_local")
        self.assertAlmostEqual(stamp.time_s, 0.7)
        self.assertEqual(stamp.physics_tick, 70)

    def test_validation(self):
        with self.assertRaises(ValueError):
            FrameStamp(-1, 0.1, 10, "world_env_local")
        with self.assertRaises(ValueError):
            FrameStamp(0, 0.0, 10, "world_env_local")
        with self.assertRaises(ValueError):
            FrameStamp(0, 0.1, 0, "world_env_local")
        with self.assertRaises(ValueError):
            FrameStamp(0, 0.1, 10, "pixel")

    def test_same_instant_and_series(self):
        a = FrameStamp(3, 0.1, 10, "world_env_local")
        contracts.require_same_instant(a, FrameStamp(3, 0.1, 10, "world_env_local"))
        with self.assertRaises(ValueError):
            contracts.require_same_instant(a, FrameStamp(4, 0.1, 10, "world_env_local"))
        with self.assertRaises(ValueError):
            contracts.require_same_instant(a, FrameStamp(3, 0.1, 10, "vehicle_yaw"))
        series = [FrameStamp(i, 0.1, 10, "world_env_local") for i in range(5)]
        contracts.require_consecutive_series(series)
        with self.assertRaises(ValueError):
            contracts.require_consecutive_series(series[:2] + series[3:])


class MotionGateFaultFreeTest(unittest.TestCase):
    """The real float32 limiter at 32-40 m passes G1 and G2, and the v1 tolerances would not."""

    @classmethod
    def setUpClass(cls):
        cls.positions, cls.velocities = _waypoint_motion()

    def test_gates_pass_on_fault_free_motion(self):
        monitor = _run_monitor(self.positions, self.velocities)
        summary = monitor.summary()
        self.assertEqual(summary["verdict"], "PASS", summary["violations"])
        self.assertTrue(all(v == 0 for v in summary["violations"].values()))
        # The gates really judged something: turning saturates in this motion.
        self.assertGreater(summary["judged"]["g1_turn"], 0)
        self.assertGreater(summary["judged"]["g2_turn"], 0)
        self.assertGreater(summary["judged"]["g2_accel"], 0)
        self.assertLess(summary["max_abs_coordinate_m"], 64.0)

    def test_v1_absolute_tolerances_flag_rounding(self):
        """Why v1 was replaced: 1e-4 on finite differences reports float32 rounding as violations."""
        p = self.positions.double()
        w = (p[1:] - p[:-1]) / 0.1
        accel = (w[1:] - w[:-1]).norm(dim=-1) / 0.1
        speed = w.norm(dim=-1)
        gated = (speed[:-1] > 0.10) & (speed[1:] > 0.10)
        turn = contracts.course_change_rad(w[:-1], w[1:]) / 0.1
        v1_accel = int((accel > 4.0 + 1e-4).sum())
        v1_turn = int(((turn > math.radians(150.0) + math.radians(1e-4)) & gated).sum())
        self.assertGreater(v1_accel + v1_turn, 0)


class MotionGateFaultInjectionTest(unittest.TestCase):
    """Each gate fires on the fault it exists for."""

    def setUp(self):
        self.positions, self.velocities = _waypoint_motion(n=16, steps=60, seed=3)

    def _summary(self, positions, velocities, nominal=1.5):
        return _run_monitor(positions, velocities, nominal).summary()

    def test_teleport(self):
        p = self.positions.clone()
        p[30:, 4, 0] += 0.5  # an instantaneous 0.5 m jump that no velocity explains
        s = self._summary(p, self.velocities)
        self.assertEqual(s["verdict"], "FAIL")
        self.assertGreater(s["violations"]["g2_position"], 0)
        self.assertGreater(s["violations"]["g2_speed"], 0)
        self.assertEqual(s["first_violation"]["g2_position"]["env_id"], 4)

    def test_velocity_rewrite_is_caught_by_g1(self):
        p = self.positions.clone()
        v = self.velocities.clone()
        v[30, 2] = (p[30, 2] - p[29, 2]) / 0.1 + torch.tensor([3.0, 0.0])
        p[30:, 2, 0] += 0.3
        s = self._summary(p, v)
        self.assertGreater(s["violations"]["g1_speed"] + s["violations"]["g1_accel"], 0)

    def test_generator_accel_spike(self):
        v = self.velocities.clone()
        p = self.positions.clone()
        # Rebuild positions so G2 position stays consistent; only the generator state jumps.
        v[20, 1] = v[19, 1] + torch.tensor([0.0, 1.0])
        for t in range(20, p.shape[0]):
            p[t, 1] = p[t - 1, 1] + v[t, 1] * 0.1
        s = self._summary(p, v)
        self.assertGreater(s["violations"]["g1_accel"], 0)

    def test_turn_spike(self):
        n, steps = 1, 12
        p = torch.zeros(steps + 1, n, 2)
        v = torch.zeros(steps + 1, n, 2)
        v[0, 0] = torch.tensor([1.0, 0.0])
        p[0, 0] = torch.tensor([10.0, 10.0])
        for t in range(1, steps + 1):
            heading = 0.0 if t < 6 else math.radians(60.0)  # a 60 deg course change in one step
            v[t, 0] = torch.tensor([math.cos(heading), math.sin(heading)])
            p[t, 0] = p[t - 1, 0] + v[t, 0] * 0.1
        s = self._summary(p, v)
        self.assertGreater(s["violations"]["g1_turn"], 0)
        self.assertGreater(s["violations"]["g2_turn"], 0)

    def test_static_target_must_not_move(self):
        n, steps = 2, 5
        p = torch.full((steps + 1, n, 2), 20.0)
        v = torch.zeros(steps + 1, n, 2)
        self.assertEqual(self._summary(p, v, nominal=0.0)["verdict"], "PASS")
        v[3, 1] = torch.tensor([0.01, 0.0])
        p[3:, 1, 0] += 0.001
        s = self._summary(p, v, nominal=0.0)
        self.assertGreater(s["violations"]["g1_speed"], 0)

    def test_nonfinite(self):
        p = self.positions.clone()
        p[10, 0, 0] = float("nan")
        s = self._summary(p, self.velocities)
        self.assertGreater(s["violations"]["nonfinite"], 0)

    def test_coordinate_bound(self):
        p = self.positions.clone() + 40.0  # 72-80 m: coarser float32 grid than declared
        s = self._summary(p, self.velocities)
        self.assertGreater(s["violations"]["coordinate_bound"], 0)

    def test_reset_is_a_boundary_not_a_step(self):
        n = 2
        monitor = MotionGateMonitor(n)
        p0 = torch.tensor([[10.0, 10.0], [20.0, 20.0]])
        v0 = torch.zeros(n, 2)
        monitor.reset(torch.arange(n), p0, v0)
        monitor.step(p0, v0, torch.full((n,), 1.5))
        far = torch.tensor([[30.0, 5.0], [20.0, 20.0]])
        monitor.reset(torch.tensor([0]), far, v0)  # env 0 starts a new episode elsewhere
        monitor.step(far, v0, torch.full((n,), 1.5))
        self.assertEqual(monitor.summary()["verdict"], "PASS")

    def test_inactive_rows_are_not_judged(self):
        p = self.positions.clone()
        p[30:, 4, 0] += 0.5
        n = p.shape[1]
        monitor = MotionGateMonitor(n)
        monitor.reset(torch.arange(n), p[0], self.velocities[0])
        active = torch.ones(n, dtype=torch.bool)
        active[4] = False
        for t in range(1, p.shape[0]):
            monitor.step(p[t], self.velocities[t], torch.full((n,), 1.5), active=active)
        self.assertEqual(monitor.summary()["verdict"], "PASS")

    def test_heading_band_is_counted_not_judged(self):
        speed = 0.10 + 5e-7  # inside (0.10, 0.10 + 1e-6]
        p = torch.zeros(3, 1, 2)
        v = torch.zeros(3, 1, 2)
        v[0, 0] = torch.tensor([speed, 0.0])
        v[1, 0] = torch.tensor([speed, 0.0])
        v[2, 0] = torch.tensor([0.0, speed])  # 90 deg in one step, unlimited below the threshold
        p[0, 0] = torch.tensor([5.0, 5.0])
        for t in (1, 2):
            p[t, 0] = p[t - 1, 0] + v[t, 0] * 0.1
        monitor = _run_monitor(p, v)
        s = monitor.summary()
        self.assertEqual(s["violations"]["g1_turn"], 0)
        self.assertGreater(s["g1_heading_band_steps"], 0)

    def test_first_violation_carries_context(self):
        p = self.positions.clone()
        p[30:, 4, 0] += 0.5
        n = p.shape[1]
        monitor = MotionGateMonitor(n, context=lambda env: {"episode_key": "s1/b70/e%03d/k000" % env})
        monitor.reset(torch.arange(n), p[0], self.velocities[0])
        for t in range(1, p.shape[0]):
            monitor.step(p[t], self.velocities[t], torch.full((n,), 1.5))
        first = monitor.summary()["first_violation"]["g2_position"]
        self.assertEqual(first["context"]["episode_key"], "s1/b70/e004/k000")
        for key in ("p_prev", "p_next", "v_prev", "v_next", "excess", "monitor_step"):
            self.assertIn(key, first)


class AcquisitionEventTest(unittest.TestCase):
    @staticmethod
    def _feed(monitor, rows):
        """rows: list of per-observation tuples of 5 bool lists (one entry per env)."""
        for row in rows:
            monitor.observe(*[torch.tensor(signal) for signal in row])

    def test_never_acquired_reports_none_not_zero(self):
        m = events.AcquisitionEventMonitor(1)
        f = [False]
        self._feed(m, [(f, f, f, f, f)] * 4)
        record = m.finish([0])[0]
        self.assertIsNone(record["first_obs_fused_valid"])
        self.assertIsNone(record["first_obs_camera_valid"])
        self.assertFalse(record["ever_fused_valid"])
        self.assertEqual(record["observations"], 4)

    def test_first_acquisition_loss_reacquisition_and_censoring(self):
        m = events.AcquisitionEventMonitor(1)
        T, F = [True], [False]
        # obs:   1  2  3(acq) 4  5(lost) 6  7(reacq) 8  9(lost, open at end)
        fused = [F, F, T, T, F, F, T, T, F]
        rows = [(x, x, x, T, x) for x in fused]
        self._feed(m, rows)
        r = m.finish([0])[0]
        self.assertEqual(r["first_obs_fused_valid"], 3)
        self.assertEqual(r["detection_losses"], 2)
        self.assertEqual(r["detection_reacquisitions"], 1)
        self.assertEqual(r["detection_reacq_latency_mean_obs"], 2)
        self.assertEqual(r["detection_reacq_latency_max_obs"], 2)
        self.assertEqual(r["detection_open_loss_censored_obs"], 1)
        self.assertEqual(r["obs_fused_valid"], 4)
        self.assertEqual(r["geo_visible_undetected_obs"], 0)

    def test_track_coasts_through_a_missed_measurement(self):
        m = events.AcquisitionEventMonitor(1)
        T, F = [True], [False]
        rows = [(T, T, T, T, T), (F, F, T, T, T), (F, F, T, T, T), (T, T, T, T, T)]
        self._feed(m, rows)
        r = m.finish([0])[0]
        self.assertEqual(r["detection_losses"], 1)
        self.assertEqual(r["detection_reacquisitions"], 1)
        self.assertEqual(r["track_losses"], 0)
        self.assertEqual(r["geo_visible_undetected_obs"], 2)
        self.assertIsNone(r["detection_open_loss_censored_obs"])

    def test_envs_are_independent_and_finish_clears(self):
        m = events.AcquisitionEventMonitor(2)
        self._feed(m, [([True, False],) * 5, ([False, True],) * 5])
        first = m.finish([1])
        self.assertEqual(first[0]["first_obs_fused_valid"], 2)
        self._feed(m, [([False, False],) * 5])
        again = m.finish([0, 1])
        self.assertEqual(again[0]["first_obs_fused_valid"], 1)
        self.assertEqual(again[0]["observations"], 3)
        self.assertIsNone(again[1]["first_obs_fused_valid"])
        self.assertEqual(again[1]["observations"], 1)

    def test_geometry_signals_are_kept_apart_from_measurement(self):
        m = events.AcquisitionEventMonitor(1)
        T, F = [True], [False]
        self._feed(m, [(F, F, F, T, T), (F, F, F, T, F), (T, T, T, T, T)])
        r = m.finish([0])[0]
        self.assertEqual(r["first_obs_geo_visible"], 1)
        self.assertEqual(r["first_obs_camera_valid"], 3)
        self.assertEqual(r["obs_geo_in_fov"], 3)
        self.assertEqual(r["geo_visible_undetected_obs"], 1)


class SceneRecordTest(unittest.TestCase):
    def test_episode_key(self):
        self.assertEqual(scene_record.episode_key(4201, 115, 7, 3), "s4201/b115/e007/k003")
        with self.assertRaises(ValueError):
            scene_record.episode_key(4201, 115, -1, 0)
        with self.assertRaises(ValueError):
            scene_record.episode_key(4201, 115, 1.5, 0)

    def test_scene_hash_is_order_free_and_quantised(self):
        centers = [[1.0, 2.0, 1.5], [3.0, 4.0, 1.5]]
        halves = [[0.3, 0.2, 1.5], [0.4, 0.4, 1.5]]
        base = scene_record.scene_hash(centers, halves)
        self.assertEqual(base, scene_record.scene_hash(centers[::-1], halves[::-1]))
        nudged = [[1.0 + 1e-5, 2.0, 1.5], [3.0, 4.0, 1.5]]
        self.assertEqual(base, scene_record.scene_hash(nudged, halves))
        moved = [[1.0 + 2e-3, 2.0, 1.5], [3.0, 4.0, 1.5]]
        self.assertNotEqual(base, scene_record.scene_hash(moved, halves))
        with self.assertRaises(ValueError):
            scene_record.scene_hash([[float("nan"), 0, 0]], [[1, 1, 1]])

    def test_initial_state_record(self):
        rec = scene_record.initial_state_record(
            "s1/b70/e000/k000", "abc", [10.0, 10.0, 1.0], 0.5, [16.0, 18.0, 1.0],
            [0.3, 0.0], 0.3, {"view": "visible", "distance_m": 10.0},
        )
        self.assertAlmostEqual(rec["initial_range_xy_m"], 10.0)
        self.assertAlmostEqual(rec["initial_world_bearing_rad"], math.atan2(8.0, 6.0))
        self.assertEqual(rec["stream"], "UNPAIRED_STREAM")
        again = scene_record.initial_state_record(
            "s1/b70/e000/k000", "abc", [10.0, 10.0, 1.0 + 1e-6], 0.5, [16.0, 18.0, 1.0],
            [0.3, 0.0], 0.3, {"view": "visible", "distance_m": 10.0},
        )
        self.assertEqual(rec["initial_state_hash"], again["initial_state_hash"])
        other = scene_record.initial_state_record(
            "s1/b70/e000/k000", "abc", [10.0, 10.0, 1.0], 0.5 + math.pi, [16.0, 18.0, 1.0],
            [0.3, 0.0], 0.3, {"view": "hidden", "distance_m": 10.0},
        )
        self.assertNotEqual(rec["initial_state_hash"], other["initial_state_hash"])


if __name__ == "__main__":
    unittest.main()
