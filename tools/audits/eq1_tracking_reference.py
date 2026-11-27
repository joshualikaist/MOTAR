#!/usr/bin/env python3
"""EQ-1C tracking — hand-derived constant-velocity Kalman reference against the production tracker (CPU).

    python -B tools/audits/eq1_tracking_reference.py [--out docs/evidence/.../eq1c_tracking.json]

Validation only. The reference is written from the equations, in float64 NumPy, without importing
production: x' = F x, P' = F P F^T + Q with F = [[I, dt I], [0, I]], Q = diag(0.025 dt^2 I, 0.25 dt I);
a new track starts at the measurement with P0 = diag(R, I); a correction uses H = [I 0] and the
Joseph form. The track expires when its age, the time since the last accepted fused correction,
exceeds memory_s = 5 s (strict >), so with dt = 0.1 s it survives 50 misses and expires on the 51st.

Gates (fixed before execution):
  T1 state/covariance agree with the reference to |d| <= 1e-5 (1 + |ref|) over a scripted sequence;
  T2 expiry after exactly 51 misses (0/49/50 active, 51/60 expired) — the 5 s contract in exact time;
  T3 identity-bound clock: with the camera lost and only a target-compatible bar on the LiDAR, the
     track must expire within 5 s of the last camera (identity-bearing) observation;
  T4 information: a range-only LiDAR measurement must not reduce position variance perpendicular to
     its ray (independent EKF with H = u^T leaves it at the prior plus nothing).
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eq1_common as common  # noqa: E402

DT, MEMORY = 0.1, 5.0
TOL = 1e-5
CAMERA_VAR = np.array([0.05 ** 2, 0.07 ** 2, 0.06 ** 2])
LIDAR_VAR = np.array([0.08 ** 2, 0.15 ** 2, 0.20 ** 2])
CAMERA_RADIUS, LIDAR_RANGE = 0.15, 12.0


# ------------------------------------------------------------------ independent reference
class ReferenceCV:
    def __init__(self, dt=DT, memory=MEMORY):
        self.dt, self.memory = dt, memory
        self.F = np.eye(6)
        self.F[:3, 3:] = dt * np.eye(3)
        self.Q = np.diag([0.025 * dt * dt] * 3 + [0.25 * dt] * 3)
        self.H = np.hstack([np.eye(3), np.zeros((3, 3))])
        self.x, self.P, self.active, self.misses_since_fused = np.zeros(6), np.eye(6), False, 0

    def step(self, z, visible, var):
        if self.active:
            self.x = self.F @ self.x
            self.P = self.F @ self.P @ self.F.T + self.Q
            self.misses_since_fused += 1
        if visible and not self.active:
            self.x = np.concatenate([z, np.zeros(3)])
            self.P = np.zeros((6, 6))
            self.P[:3, :3] = np.diag(var)
            self.P[3:, 3:] = np.eye(3)
            self.active, self.misses_since_fused = True, 0
        elif visible:
            self.correct(z, np.diag(var))
        # Exact-time contract: age = misses * dt (decimal); expire when strictly older than memory.
        if self.active and round(self.misses_since_fused * self.dt, 9) > self.memory:
            self.active = False

    def correct(self, z, r):
        s = self.H @ self.P @ self.H.T + r
        k = self.P @ self.H.T @ np.linalg.inv(s)
        self.x = self.x + k @ (z - self.H @ self.x)
        ikh = np.eye(6) - k @ self.H
        self.P = ikh @ self.P @ ikh.T + k @ r @ k.T
        self.misses_since_fused = 0


def range_only_posterior(p_pos, u, sigma_r):
    """Independent EKF position covariance after one range measurement along unit vector u."""
    h = u.reshape(1, 3)
    s = float(h @ p_pos @ h.T) + sigma_r ** 2
    k = p_pos @ h.T / s
    return (np.eye(3) - k @ h) @ p_pos


# ------------------------------------------------------------------ production
def perception():
    return common.load_production("aerial_gym.task.navrl_task.navrl_perception")


def tracker(p):
    return p.BatchedConstantVelocityTracker(1, torch.device("cpu"), DT, MEMORY)


def case_kalman(p):
    trk, ref = tracker(p), ReferenceCV()
    worst = 0.0
    rows = []
    for t in range(40):
        visible = t < 10 or 20 <= t < 24
        z = np.array([1.0 + 1.0 * t * DT, 2.0 + 0.5 * t * DT, 1.0])
        trk.step(torch.tensor(z[None], dtype=torch.float32), torch.tensor([visible]),
                 torch.tensor(CAMERA_VAR[None], dtype=torch.float32))
        ref.step(z, visible, CAMERA_VAR)
        xs, ps = trk.state[0].double().numpy(), trk.cov[0].double().numpy()
        err = max(float(np.max(np.abs(xs - ref.x) / (1 + np.abs(ref.x)))),
                  float(np.max(np.abs(ps - ref.P) / (1 + np.abs(ref.P)))))
        worst = max(worst, err)
        rows.append({"t": t, "visible": visible, "active": bool(trk.active[0]), "ref_active": ref.active,
                     "age": float(trk.age[0]), "rel_err": err})
    return {"name": "T1 Kalman predict/correct against the hand-derived reference", "worst_rel_err": worst,
            "gate": "<= %g" % TOL, "rows": rows,
            "production": common.verdict(worst <= TOL and all(r["active"] == r["ref_active"] for r in rows),
                                         "float32 production vs float64 reference")}


def case_expiry(p):
    rows = []
    for misses in (0, 49, 50, 51, 60):
        trk = tracker(p)
        z = torch.tensor([[3.0, 0.0, 1.0]])
        var = torch.tensor(CAMERA_VAR[None], dtype=torch.float32)
        trk.step(z, torch.tensor([True]), var)
        for _ in range(misses):
            trk.step(z, torch.tensor([False]), var)
        exact_age = round(misses * DT, 9)
        rows.append({"misses": misses, "active": bool(trk.active[0]), "age_float32": float(trk.age[0]),
                     "age_exact_s": exact_age, "expected_active": not exact_age > MEMORY})
    ok = all(r["active"] == r["expected_active"] for r in rows)
    return {"name": "T2 5 s expiry after camera acquisition (0/49/50/51/60 misses)", "rows": rows,
            "production": common.verdict(ok, "expires on the 51st miss; float32 age accumulation checked")}


class _AssocStandIn:
    def __init__(self, p, trk, range_only=False):
        self.tracker, self.target_radius, self.num_envs, self.device = trk, CAMERA_RADIUS, 1, torch.device("cpu")
        self._lidar_angles = p.lidar_bin_bearings(self.device)
        self.lidar_max_range, self.lidar_assoc_gate_m = LIDAR_RANGE, 0.0
        self.lidar_range_only_update, self.lidar_silent_correct = range_only, False


def _bar_scan(p, s, drone, bar_surface_fn):
    rel = s.tracker.state[0, :3] - drone
    bearing = math.atan2(float(rel[1]), float(rel[0]))
    j = int(torch.argmin(torch.atan2(torch.sin(s._lidar_angles - bearing), torch.cos(s._lidar_angles - bearing)).abs()))
    scan = torch.full((1, p.VBEAMS, p.HBEAMS), LIDAR_RANGE)
    scan[0, :, j] = bar_surface_fn(float(rel.norm()))
    return scan.view(1, -1)


def case_lidar_keepalive(p):
    """Camera lost; a static bar sits where the track predicts the target. Does the track outlive 5 s?"""
    trk = tracker(p)
    var = torch.tensor(CAMERA_VAR[None], dtype=torch.float32)
    trk.step(torch.tensor([[5.0, 0.0, 1.0]]), torch.tensor([True]), var)
    s = _AssocStandIn(p, trk)
    drone = torch.tensor([0.0, 0.0, 1.0])
    accepted = 0
    for t in range(100):
        trk.step(torch.zeros(1, 3), torch.tensor([False]), var)
        scan = _bar_scan(p, s, drone, lambda d: d - CAMERA_RADIUS)
        valid, *_ = p.NavRLPerceptionModule._associate_lidar_target(
            s, scan, drone.view(1, 3), torch.tensor([[0.0, 0.0, 0.0, 1.0]]), torch.zeros(1, dtype=torch.bool))
        accepted += int(valid[0])
    alive = bool(trk.active[0])
    return {"name": "T3 identity-bound clock (LiDAR-compatible bar, no camera, 10 s)", "steps": 100,
            "lidar_accepts": accepted, "track_active_after_10s": alive, "age_after_s": float(trk.age[0]),
            "literal_contract": {"status": "PASS", "note": "age is time since the last accepted fused correction"},
            "production_identity_clock": common.verdict(not alive, "every accepted bar return resets the age; the 5 s "
                                                        "bound is not a bound on time since a verified target view")}


def case_range_information(p):
    rows = []
    for range_only in (False, True):
        for bearing_deg in (0.0, 90.0, 45.0):
            trk = tracker(p)
            b = math.radians(bearing_deg)
            target = torch.tensor([5 * math.cos(b), 5 * math.sin(b), 1.0])
            trk.step(target.view(1, 3), torch.tensor([True]), torch.tensor(CAMERA_VAR[None], dtype=torch.float32))
            trk.cov[0, :3, :3] = torch.eye(3)  # a uniform prior: 1 m^2 in every direction
            prior = trk.cov[0, :3, :3].double().numpy().copy()
            s = _AssocStandIn(p, trk, range_only=range_only)
            drone = torch.tensor([0.0, 0.0, 1.0])
            scan = _bar_scan(p, s, drone, lambda d: d - CAMERA_RADIUS)
            valid, *_ = p.NavRLPerceptionModule._associate_lidar_target(
                s, scan, drone.view(1, 3), torch.tensor([[0.0, 0.0, 0.0, 1.0]]), torch.zeros(1, dtype=torch.bool))
            post = trk.cov[0, :3, :3].double().numpy()
            u = np.array([math.cos(b), math.sin(b), 0.0])
            ref = range_only_posterior(prior, u, 0.08)
            perp = np.array([-math.sin(b), math.cos(b), 0.0])
            z = np.array([0.0, 0.0, 1.0])
            rows.append({"lidar_range_only_update": range_only, "bearing_deg": bearing_deg, "accepted": bool(valid[0]),
                         "range_var_prod": float(u @ post @ u), "range_var_ref": float(u @ ref @ u),
                         "tangential_var_prod": float(perp @ post @ perp), "tangential_var_ref": float(perp @ ref @ perp),
                         "vertical_var_prod": float(z @ post @ z), "vertical_var_ref": float(z @ ref @ z)})
    policy_f = [r for r in rows if not r["lidar_range_only_update"]]
    opt_in = [r for r in rows if r["lidar_range_only_update"]]
    def ok(rs, tol):
        return all(r["accepted"] and r["tangential_var_prod"] >= r["tangential_var_ref"] * (1 - tol)
                   and r["vertical_var_prod"] >= r["vertical_var_ref"] * (1 - tol) for r in rs)
    return {"name": "T4 range-only information versus the independent measurement Jacobian", "rows": rows,
            "production_policy_f": common.verdict(ok(policy_f, 1e-3), "diagonal world-axis R [0.08, 0.15, 0.20]^2 on a "
                                                  "3-D point whose bearing and height are the track's own prediction"),
            "existing_opt_in_flag": common.verdict(ok(opt_in, 1e-3), "NAVRL_LIDAR_RANGE_ONLY_UPDATE=1: anisotropic R "
                                                   "(sigma_perp 50 m); not used by Policy F")}


def run():
    with common.production():
        return _run()


def _run():
    p = perception()
    return {"schema": "motar.eq1c-tracking.v1", "package": "EQ-1C tracker reference (CPU, validation only)",
            "source": common.source_binding(["aerial_gym/task/navrl_task/navrl_perception.py"]),
            "config": {"dt_s": DT, "memory_s": MEMORY, "camera_var_fixture": CAMERA_VAR.tolist(),
                       "lidar_var_production": LIDAR_VAR.tolist()},
            "cases": [case_kalman(p), case_expiry(p), case_lidar_keepalive(p), case_range_information(p)]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    record = run()
    if args.out:
        common.write_record(args.out, record)
    for c in record["cases"]:
        print(c["name"], {k: v["status"] for k, v in c.items() if isinstance(v, dict) and "status" in v})
    return 0


if __name__ == "__main__":
    sys.exit(main())
