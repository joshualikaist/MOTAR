#!/usr/bin/env python3
"""Independent float64 reference for S-L1C LiDAR target association and range-only updates (CPU).

NumPy only; never imports production code. LiDAR scans come from the committed S-L1A first-hit
oracle with the declared yaw-only ray table (azimuth +pi decreasing by 2pi/72, rows +20..-10 deg)
and the 0.20 m target proxy sphere. Track priming is fixture input, never a perception input.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s_l1a_reference as ref  # noqa: E402

CONTRACT = "MOTAR_S_L1C_FLOAT64_REFERENCE_V1"
LIDAR_PROXY_RADIUS_M = 0.20
CAMERA_TARGET_RADIUS_M = 0.15
SHAPE_TOL_M = 0.10
MIN_CONCLUSIVE_M = LIDAR_PROXY_RADIUS_M / math.sin(math.radians(10.0))
SIGMA_RANGE_M = 0.08
SIGMA_UNOBSERVED_M = 50.0
ANGLES = np.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72)
ELEVATIONS = np.radians([20., 10., 0., -10.])


def beams():
    return np.stack(np.broadcast_arrays(np.cos(ELEVATIONS[:, None]) * np.cos(ANGLES),
                                        np.cos(ELEVATIONS[:, None]) * np.sin(ANGLES),
                                        np.sin(ELEVATIONS[:, None]) + 0 * ANGLES), axis=-1)


def lidar_scan(box_centers=(), box_half_extents=(), *, sphere_center=None, far_m=12.):
    """[4,72] ranges from the vehicle origin; no return = far_m."""
    hits = ref.first_hits(np.zeros(3), beams(), box_centers, box_half_extents, sphere_center=sphere_center,
                          sphere_radius=LIDAR_PROXY_RADIUS_M, far_m=far_m)
    return np.minimum(hits["depth_m"], far_m), hits["hit_class"]


def associate(scan, predicted_center, *, gate_m, camera_visible=False, active=True, mode="s_l1c", far_m=12.):
    """Legacy or S-L1C association of one track against one [4,72] scan (vehicle frame)."""
    scan = np.asarray(scan, dtype=np.float64)
    rel = np.asarray(predicted_center, dtype=np.float64)
    d = float(np.linalg.norm(rel))
    bearing = math.atan2(rel[1], rel[0])
    h = int(np.argmin(np.abs(np.arctan2(np.sin(ANGLES - bearing), np.cos(ANGLES - bearing)))))
    if mode == "legacy":
        measured = float(scan[:, h].min())
        predicted_surface = max(d - CAMERA_TARGET_RADIUS_M, 0.)
        ok = (active and not camera_visible and predicted_surface < far_m and measured < far_m * .995
              and abs(measured - predicted_surface) < gate_m)
        return {"valid": bool(ok), "measured_surface": measured, "centre_range": measured + CAMERA_TARGET_RADIUS_M,
                "column": h, "row": int(np.argmin(scan[:, h]))}
    el = math.atan2(rel[2], math.hypot(rel[0], rel[1]))
    half = math.asin(min(1., LIDAR_PROXY_RADIUS_M / max(d, 1e-9)))
    eligible = np.abs(ELEVATIONS - el) < half
    if not eligible.any():
        return {"valid": False, "reason": "no_eligible_row", "column": h}
    rows = np.where(eligible)[0]
    row = int(rows[np.argmin(scan[rows, h])])
    measured = float(scan[row, h])
    horizontal = measured * math.cos(ELEVATIONS[row])
    extended = False
    for adj in (row - 1, row + 1):
        if 0 <= adj < len(ELEVATIONS):
            r = scan[adj, h]
            if 0 < r < far_m * .995 and abs(r * math.cos(ELEVATIONS[adj]) - horizontal) <= SHAPE_TOL_M:
                extended = True
    predicted_surface = max(d - LIDAR_PROXY_RADIUS_M, 0.)
    ok = (active and not camera_visible and d >= MIN_CONCLUSIVE_M and not extended
          and predicted_surface < far_m and measured < far_m * .995 and abs(measured - predicted_surface) < gate_m)
    centre = measured + LIDAR_PROXY_RADIUS_M
    return {"valid": bool(ok), "measured_surface": measured, "centre_range": centre, "column": h, "row": row,
            "extended": extended, "eligible_rows": rows.tolist(), "measurement_point": (centre * rel / d).tolist()}


def range_only_posterior(prior_cov, direction, *, sigma_r=SIGMA_RANGE_M, sigma_u=SIGMA_UNOBSERVED_M):
    """Position covariance after one KF update with R = sr^2 uu^T + su^2 (I - uu^T)."""
    p = np.asarray(prior_cov, dtype=np.float64)
    u = np.asarray(direction, dtype=np.float64)
    u = u / np.linalg.norm(u)
    r = sigma_r ** 2 * np.outer(u, u) + sigma_u ** 2 * (np.eye(3) - np.outer(u, u))
    k = p @ np.linalg.inv(p + r)
    ikh = np.eye(3) - k
    return ikh @ p @ ikh.T + k @ r @ k.T


def orthonormal_basis(u):
    u = np.asarray(u, dtype=np.float64) / np.linalg.norm(u)
    a = np.array([0., 0., 1.]) if abs(u[2]) < .9 else np.array([1., 0., 0.])
    t1 = np.cross(u, a)
    t1 /= np.linalg.norm(t1)
    return u, t1, np.cross(u, t1)


def self_test():
    checks = {}
    bar = ([[5.0 + .3, 0., .5]], [[.3, .3, 1.5]])
    scan, _ = lidar_scan(*bar)
    primed = np.array([5.0 + CAMERA_TARGET_RADIUS_M, 0., 0.])
    checks["legacy_accepts_bar"] = associate(scan, primed, gate_m=.35, mode="legacy")["valid"]
    checks["s_l1c_rejects_bar"] = not associate(scan, primed, gate_m=.35)["valid"]
    tscan, _ = lidar_scan(sphere_center=[5., 0., 0.])
    res = associate(tscan, [5., 0., 0.], gate_m=.35)
    checks["s_l1c_accepts_target"] = res["valid"]
    checks["s_l1c_centre_exact"] = abs(res["centre_range"] - 5.0) < 1e-9
    post = range_only_posterior(np.eye(3), [1., 0., 0.])
    checks["range_only_keeps_tangential"] = bool(post[1, 1] > .99 and post[2, 2] > .99 and post[0, 0] < .01)
    return checks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--self-test", action="store_true")
    parser.parse_args(argv)
    checks = self_test()
    print(json.dumps({"contract": CONTRACT, "checks": checks}, indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
