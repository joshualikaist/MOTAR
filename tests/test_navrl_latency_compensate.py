"""Unit tests for latency compensation P0/P1 (WORKLOG 2026-08-05 checklist item 4).

CPU-only: loads navrl_perception by file path (no aerial_gym package import, so no Isaac Gym),
drives the real BatchedConstantVelocityTracker with tau-delayed measurements of a synthetic
constant-velocity target, and asserts the forward-predicted position beats the raw delayed
estimate on both position and bearing error.

Run: PYTHONNOUSERSITE=1 python tests/test_navrl_latency_compensate.py
"""

import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest

import torch

_TASK_DIR = Path(__file__).parents[1] / "aerial_gym/task/navrl_task"
for _package in ("aerial_gym", "aerial_gym.task", "aerial_gym.task.navrl_task"):
    sys.modules.setdefault(_package, types.ModuleType(_package))
_CORRIDOR_NAME = "aerial_gym.task.navrl_task.navrl_corridor"
_CORRIDOR_SPEC = importlib.util.spec_from_file_location(
    _CORRIDOR_NAME, _TASK_DIR / "navrl_corridor.py"
)
_CORRIDOR = importlib.util.module_from_spec(_CORRIDOR_SPEC)
sys.modules[_CORRIDOR_NAME] = _CORRIDOR
_CORRIDOR_SPEC.loader.exec_module(_CORRIDOR)

_SPEC = importlib.util.spec_from_file_location(
    "navrl_perception_standalone", _TASK_DIR / "navrl_perception.py"
)
_PERCEPTION = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_PERCEPTION)
BatchedConstantVelocityTracker = _PERCEPTION.BatchedConstantVelocityTracker

DT = 0.1
TAU = 0.1  # NAVRL_DETECTION_LATENCY_S of the R3 catastrophic arm
STEPS = 60


def _run_delayed_cv_tracking(tau_steps, speed_mps=1.5, heading_rad=0.7):
    """Feed the KF measurements that lag the true CV target by tau_steps; return final errors."""
    tracker = BatchedConstantVelocityTracker(1, "cpu", DT, memory_s=3.0)
    vel = torch.tensor([[math.cos(heading_rad), math.sin(heading_rad), 0.0]]) * speed_mps
    pos0 = torch.tensor([[4.0, -2.0, 1.0]])
    meas_var = torch.full((1, 3), 0.05**2)

    truth_now = None
    for k in range(STEPS):
        t_now = k * DT
        t_meas = max(0.0, t_now - tau_steps * DT)
        truth_now = pos0 + vel * t_now
        measurement = pos0 + vel * t_meas
        tracker.step(measurement, torch.ones(1, dtype=torch.bool), meas_var)

    raw_pos = tracker.state[:, :3]
    # P0: exactly the expression _target_features uses when latency_compensate is on.
    pred_pos = tracker.state[:, :3] + tracker.state[:, 3:] * (tau_steps * DT)

    def bearing_err(est):
        # Bearing from a drone at the origin, as the policy's vehicle frame would see it.
        b_est = math.atan2(float(est[0, 1]), float(est[0, 0]))
        b_true = math.atan2(float(truth_now[0, 1]), float(truth_now[0, 0]))
        return abs(math.atan2(math.sin(b_est - b_true), math.cos(b_est - b_true)))

    return {
        "raw_pos_err": float((raw_pos - truth_now).norm()),
        "pred_pos_err": float((pred_pos - truth_now).norm()),
        "raw_bearing_err": bearing_err(raw_pos),
        "pred_bearing_err": bearing_err(pred_pos),
        "vel_err": float((tracker.state[:, 3:] - vel).norm()),
    }


class LatencyCompensationMath(unittest.TestCase):
    def test_forward_predict_beats_raw_delayed(self):
        """Checklist 4: CV target + tau=0.1 -> predicted bearing error < raw delayed error."""
        r = _run_delayed_cv_tracking(tau_steps=1)
        self.assertLess(r["pred_bearing_err"], r["raw_bearing_err"])
        self.assertLess(r["pred_pos_err"], r["raw_pos_err"])
        # The KF's velocity estimate is unbiased for a CV target even from delayed positions,
        # which is the whole reason output-side extrapolation works.
        self.assertLess(r["vel_err"], 0.15)
        # The raw estimate must actually exhibit the ~v*tau lag being compensated
        # (0.15 m at 1.5 m/s * 0.1 s); the prediction should remove most of it.
        self.assertGreater(r["raw_pos_err"], 0.10)
        self.assertLess(r["pred_pos_err"], 0.5 * r["raw_pos_err"])

    def test_two_step_latency_also_recovered(self):
        r = _run_delayed_cv_tracking(tau_steps=2)
        self.assertLess(r["pred_pos_err"], 0.5 * r["raw_pos_err"])
        self.assertLess(r["pred_bearing_err"], r["raw_bearing_err"])

    def test_zero_latency_predict_is_identity(self):
        """With tau=0 the compensation expression is a no-op by construction."""
        r = _run_delayed_cv_tracking(tau_steps=0)
        self.assertAlmostEqual(r["pred_pos_err"], r["raw_pos_err"], places=6)

    def test_static_target_unaffected(self):
        """A hovering target has ~zero estimated velocity: predict must not invent motion."""
        r = _run_delayed_cv_tracking(tau_steps=1, speed_mps=0.0)
        self.assertLess(abs(r["pred_pos_err"] - r["raw_pos_err"]), 0.02)


class LatencyCompensationPlumbing(unittest.TestCase):
    """Guard the cfg wiring by source inspection (the module needs no simulator to check this)."""

    SOURCE = (_TASK_DIR / "navrl_perception.py").read_text(encoding="utf-8")

    def test_p0_is_output_side_only(self):
        # P0 must live in _target_features (policy-facing output), not inside the tracker.
        start = self.SOURCE.index("def _target_features")
        end = self.SOURCE.index("def _update_histories")
        body = self.SOURCE[start:end]
        self.assertIn("latency_compensate", body)
        self.assertIn("state[:, 3:] * self.detection_latency_s", body)
        tracker_src = self.SOURCE[
            self.SOURCE.index("class BatchedConstantVelocityTracker") :
            self.SOURCE.index("class NavRLPerceptionModule")
        ]
        self.assertNotIn("latency_compensate", tracker_src)

    def test_p1_gate_reads_backup_flag(self):
        start = self.SOURCE.index("def observe")
        body = self.SOURCE[start : start + 4000]
        self.assertIn("latency_lidar_backup", body)
        self.assertIn("lidar_camera_gate", body)

    def test_knobs_read_from_cfg_defaults_off(self):
        self.assertIn('getattr(cfg, "latency_compensate", False)', self.SOURCE)
        self.assertIn('getattr(cfg, "latency_lidar_backup", False)', self.SOURCE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
