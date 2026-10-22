"""Module C v0: one-scan, sensor-only planar obstacle representation (CPU, pure).

Free space follows motar_eval.boia.geometry.known_free exactly (numpy port, equality-tested):
per beam the minimum over rows of range*cos(elevation) minus a margin, then the minimum with
both neighbouring beams. LiDAR first hits are OBSERVED_OCCUPIED points. Everything else,
including space behind hits, between rays beyond the conservative radius and above/below the
scan, is UNKNOWN. UNKNOWN is never FREE. A planar model at the held flight altitude; no
multi-scan fusion, no camera-depth fusion and no 3-D free-volume certificate (G4-R5 B03).
"""
from dataclasses import dataclass
import math

import numpy as np

from .geometry import angle_diff, lidar_directions, yaw_rotation

OBSERVED_FREE, OBSERVED_OCCUPIED, UNKNOWN = "OBSERVED_FREE", "OBSERVED_OCCUPIED", "UNKNOWN"
SAFE, BLOCKED, UNCERTIFIED = "SAFE_IN_OBSERVED_FREE", "BLOCKED_BY_OBSERVED_OCCUPANCY", "UNCERTIFIED_UNKNOWN_SPACE"


def known_free_radii(lidar, model, margin_m):
    """numpy port of boia.geometry.known_free for one scan (4, 72) -> (72,) horizontal radii."""
    r = np.where(np.isfinite(lidar), lidar, 0.0).clip(0.0, model.max_range_m)
    horizontal = r * np.cos(model.elevation_rad())[:, None]
    base = (horizontal.min(axis=0) - margin_m).clip(min=0.0)
    return np.minimum(base, np.minimum(np.roll(base, 1), np.roll(base, -1)))


@dataclass(frozen=True)
class LocalMap:
    origin_xy: tuple
    origin_yaw: float
    stamp_s: float
    radii: np.ndarray          # (72,) conservative free radius per beam, origin-centred
    beam_az: np.ndarray        # (72,) vehicle-frame azimuth
    hits_xy: np.ndarray        # (K, 2) world-frame first-hit points (non-far, above min range)
    margin_m: float
    source: str = "LIDAR_SINGLE_SCAN_PLANAR"
    ground_truth_used: bool = False


def build(inp, model, margin_m=0.45):
    """From one validated EvidenceInput: world-frame hits and known-free radii at the scan origin."""
    from .geometry import quat_yaw_xyzw
    yaw = quat_yaw_xyzw(inp.vehicle_quat)
    lidar = np.asarray(inp.lidar, float)
    dirs = lidar_directions(model)
    valid = np.isfinite(lidar) & (lidar < model.max_range_m - 1e-6) & (lidar > model.min_range_m)
    pts_v = dirs[valid] * lidar[valid][:, None]
    pts_w = pts_v[:, :2] @ yaw_rotation(yaw)[:2, :2].T + np.asarray(inp.position[:2], float)
    radii = known_free_radii(lidar, model, margin_m)
    for a in (radii, pts_w):
        a.setflags(write=False)
    az = model.azimuth_rad().copy(); az.setflags(write=False)
    return LocalMap((float(inp.position[0]), float(inp.position[1])), yaw, float(inp.decision_s), radii, az, pts_w, margin_m)


def point_status(m, p_xy, clearance_m):
    """Status of a world point for the vehicle body of radius `clearance_m`."""
    p = np.asarray(p_xy, float)
    if m.hits_xy.size and np.min(np.linalg.norm(m.hits_xy - p, axis=1)) < clearance_m:
        return OBSERVED_OCCUPIED
    rel = p - np.asarray(m.origin_xy)
    rng = float(np.hypot(*rel))
    if rng < 1e-9:
        return OBSERVED_FREE
    bearing = math.atan2(rel[1], rel[0]) - m.origin_yaw
    j = int(np.argmin(np.abs(angle_diff(m.beam_az, bearing))))
    return OBSERVED_FREE if rng + clearance_m <= float(m.radii[j]) else UNKNOWN


def segment_status(m, a_xy, b_xy, clearance_m, step_m=0.05, now_s=None, max_age_s=0.1):
    """SAFE only if every sample is OBSERVED_FREE; BLOCKED if any sample hits observed occupancy;
    otherwise UNCERTIFIED. A stale map never certifies (UNCERTIFIED with reason)."""
    a, b = np.asarray(a_xy, float), np.asarray(b_xy, float)
    n = max(2, int(math.ceil(np.linalg.norm(b - a) / step_m)) + 1)
    samples = a + (b - a) * np.linspace(0.0, 1.0, n)[:, None]
    status = [point_status(m, s, clearance_m) for s in samples]
    unknown = sum(s == UNKNOWN for s in status) / n
    if any(s == OBSERVED_OCCUPIED for s in status):
        return BLOCKED, unknown, ("PATH_INTERSECTS_OBSERVED_OCCUPANCY",)
    if now_s is not None and now_s - m.stamp_s > max_age_s + 1e-9:
        return UNCERTIFIED, unknown, ("MAP_STALE",)
    if unknown > 0:
        return UNCERTIFIED, unknown, ("PATH_ENTERS_UNKNOWN_SPACE",)
    return SAFE, 0.0, ()
