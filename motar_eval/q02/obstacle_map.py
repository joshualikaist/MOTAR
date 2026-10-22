"""Q03 local obstacle representation v0: sensor-only, world-anchored rolling 2-D grid (CPU, numpy).

Structure: a horizontal slice of the world around the held flight altitude (Frozen F holds altitude with
a PI loop; obstacles are vertical bars), 0.2 m cells, 120 x 120 cells (24 m x 24 m), re-anchored by whole
cells when the vehicle drifts from the centre. A world anchor makes ego-motion compensation an exact pose
transform per frame, where an egocentric polar grid would need resampling every decision.

Per cell three timestamps are kept: last observed free, last observed occupied, last cross-sensor
conflict. The state is derived at query time and is never stored:
- OCCUPIED: occupancy observed within `occupied_ttl_s`;
- CONFLICT: a camera/LiDAR disagreement within `conflict_ttl_s`, or a free observation newer than a still-recent occupancy;
- FREE: free observed within `free_ttl_s` and nothing recent against it;
- UNKNOWN: everything else (never observed, stale, outside the grid).

UNKNOWN is never FREE, and CONFLICT and OCCUPIED both block.

Inputs are only validated Q02-A data:
- LiDAR getter ranges with Q02-A sample categories (VALID_HIT carves free to the hit and marks the hit;
  NO_RETURN carves free to 12 m; NEAR_CLIPPED raises a near-obstacle warning and carves nothing);
- the recovered 40 x 24 native obstacle depth (only when `obstacle_depth` is OK). A 10 m no-return carves
  free up to 10 m and never beyond; the target paste is not in the native grid, so the target never
  enters the camera's obstacle evidence.

Only the part of a ray inside the altitude band |z - z_ref| <= band_half_m is used. Ground truth is never
an input. LiDAR returns from the target's proxy sphere cannot be told apart from obstacles and are kept as
occupancy (conservative; not excluded).

Cost: 120 x 120 cells x (3 float64 + bookkeeping) ~ 0.35 MB. Per frame there are about 35k LiDAR and
100k camera samples (rays sampled at half a cell), done with vectorised numpy.
"""
from dataclasses import dataclass
import math

import numpy as np

from .contracts import FROZEN_F_CAMERA, FROZEN_F_LIDAR, FROZEN_F_PROFILE, NEAR_CLIPPED, NO_RETURN, VALID_HIT
from .geometry import lidar_directions, native_camera_rays, quat_yaw_xyzw, yaw_rotation
from .validation import classify_depth_native, classify_lidar

FREE, OCCUPIED, UNKNOWN, CONFLICT = "FREE", "OCCUPIED", "UNKNOWN", "CONFLICT"
SAFE, BLOCKED, UNCERTIFIED = "SAFE_IN_OBSERVED_FREE", "BLOCKED_BY_OBSERVED_OCCUPANCY", "UNCERTIFIED_UNKNOWN_OR_CONFLICT"
_RANK = {FREE: 0, UNKNOWN: 1, CONFLICT: 2, OCCUPIED: 3}
NEVER = -np.inf


@dataclass(frozen=True)
class MapParams:
    """Declared priors, not calibrated against the plant or a sensor noise model."""
    cell_m: float = 0.2
    size_cells: int = 120
    recentre_m: float = 4.0            # re-anchor when the vehicle is this far from the grid centre
    band_half_m: float = 0.5           # altitude slice around z_ref used for both free carving and hits
    altitude_reset_m: float = 0.3      # z change beyond this invalidates the slice (map reset to UNKNOWN)
    free_ttl_s: float = 1.0            # free evidence ages out quickly (the target moves; unseen changes)
    occupied_ttl_s: float = 5.0        # occupancy is kept longer (conservative for safety)
    conflict_ttl_s: float = 2.0
    hit_backoff_m: float = 0.2         # free carving stops this far before a hit (cell quantisation)


@dataclass(frozen=True)
class FrameReceipt:
    accepted: bool
    reasons: tuple
    stamp_s: float
    lidar_rays: int = 0
    camera_rays: int = 0
    free_cells: int = 0
    occupied_cells: int = 0
    conflict_cells: int = 0
    near_clipped_beams: int = 0


class LocalObstacleMap:
    """One episode's map. `integrate` one validated frame per decision; queries take `now_s`."""

    def __init__(self, params=MapParams(), lidar=FROZEN_F_LIDAR, camera=FROZEN_F_CAMERA, profile=FROZEN_F_PROFILE):
        self.p, self.lidar, self.camera, self.profile = params, lidar, camera, profile
        n = params.size_cells
        self.free_s = np.full((n, n), NEVER); self.occ_s = np.full((n, n), NEVER); self.conf_s = np.full((n, n), NEVER)
        self.origin_xy = None          # world xy of cell (0, 0) corner
        self.z_ref = None
        self.last_stamp_s = None
        self.near_warning_s = NEVER
        self.frames = 0
        self.ground_truth_used = False

    # --- bookkeeping -------------------------------------------------------------------------
    def _anchor(self, xy):
        half = self.p.size_cells * self.p.cell_m / 2.0
        if self.origin_xy is None:
            self.origin_xy = np.array([xy[0] - half, xy[1] - half])
            return
        centre = self.origin_xy + half
        if np.max(np.abs(np.asarray(xy) - centre)) <= self.p.recentre_m:
            return
        shift = np.round((np.asarray(xy) - centre) / self.p.cell_m).astype(int)   # whole cells
        for a in (self.free_s, self.occ_s, self.conf_s):
            moved = np.full_like(a, NEVER)
            n = self.p.size_cells
            sx, sy = shift
            src = a[max(sx, 0):n + min(sx, 0), max(sy, 0):n + min(sy, 0)]
            moved[max(-sx, 0):max(-sx, 0) + src.shape[0], max(-sy, 0):max(-sy, 0) + src.shape[1]] = src
            a[...] = moved
        self.origin_xy = self.origin_xy + shift * self.p.cell_m

    def _cells(self, xy):
        ij = np.floor((np.asarray(xy, float) - self.origin_xy) / self.p.cell_m).astype(int)
        inside = (ij >= 0).all(axis=-1) & (ij < self.p.size_cells).all(axis=-1)
        return ij, inside

    def reset(self, reason):
        self.free_s[...] = NEVER; self.occ_s[...] = NEVER; self.conf_s[...] = NEVER
        self.z_ref = None
        return reason

    # --- integration -------------------------------------------------------------------------
    def _ray_samples(self, origin, dirs, lengths):
        """World xy every half cell from the origin along each ray, up to its length or the point where the
        ray leaves the altitude band |z - z_ref| <= band_half_m, whichever comes first."""
        step, band = self.p.cell_m / 2.0, self.p.band_half_m
        dz0, dz = origin[2] - self.z_ref, dirs[:, 2]
        if abs(dz0) > band:
            return np.zeros((0, 2))
        with np.errstate(divide="ignore", invalid="ignore"):
            t_band = np.where(dz > 1e-9, (band - dz0) / dz, np.where(dz < -1e-9, (band + dz0) / -dz, np.inf))
        lengths = np.minimum(np.clip(np.asarray(lengths, float), 0.0, None), t_band)
        n = np.floor(lengths / step).astype(int) + (lengths > 0)
        if n.sum() == 0:
            return np.zeros((0, 2))
        idx = np.repeat(np.arange(len(dirs)), n)
        k = (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)) * step
        return (origin + dirs[idx] * k[:, None])[:, :2]

    def _hits(self, origin, dirs, ranges):
        pts = origin + dirs * ranges[:, None]
        in_band = np.abs(pts[:, 2] - self.z_ref) <= self.p.band_half_m
        return pts[in_band, :2]

    def _cell_set(self, xy):
        if len(xy) == 0:
            return np.zeros((0, 2), int)
        ij, inside = self._cells(xy)
        return np.unique(ij[inside], axis=0)

    def integrate(self, inp, report):
        """Integrate one frame. `report` is the Q02-A ValidationReport of the same EvidenceInput."""
        stamp = float(inp.decision_s)
        reasons = []
        if self.last_stamp_s is not None and stamp <= self.last_stamp_s + 1e-9:
            return FrameReceipt(False, ("FRAME_NOT_NEWER_THAN_MAP",), stamp)
        if report.status("pose") != "OK":
            return FrameReceipt(False, ("POSE_INVALID",), stamp)
        if report.status("calibration") != "OK":
            return FrameReceipt(False, ("CALIBRATION_MISMATCH",), stamp)
        lidar_ok, cam_ok = report.lidar_usable, report.obstacle_depth_usable
        if not lidar_ok and not cam_ok:
            return FrameReceipt(False, ("NO_USABLE_RANGE_SENSOR",) + report.problems(), stamp)
        pos = np.asarray(inp.position, float); yaw = quat_yaw_xyzw(inp.vehicle_quat); rot = yaw_rotation(yaw)
        if self.z_ref is not None and abs(pos[2] - self.z_ref) > self.p.altitude_reset_m:
            reasons.append(self.reset("ALTITUDE_CHANGED_MAP_RESET"))
        if self.z_ref is None:
            self.z_ref = float(pos[2])
        self._anchor(pos[:2])
        free_l = occ_l = free_c = occ_c = np.zeros((0, 2), int)
        n_near = n_l = n_c = 0
        if lidar_ok:
            dirs = lidar_directions(self.lidar).reshape(-1, 3) @ rot.T
            r = np.asarray(inp.lidar, float).reshape(-1)
            cat = classify_lidar(r, self.lidar, self.profile)
            hit, noret, near = cat == VALID_HIT, cat == NO_RETURN, cat == NEAR_CLIPPED
            n_near = int(near.sum()); n_l = int((hit | noret).sum())
            length = np.where(hit, r - self.p.hit_backoff_m, np.where(noret, self.profile.lidar_no_return_m, 0.0))
            free_l = self._cell_set(self._ray_samples(pos, dirs, length))
            occ_l = self._cell_set(self._hits(pos, dirs[hit], r[hit]))
        else:
            reasons.append("LIDAR_NOT_INTEGRATED")
        if cam_ok:
            origin = pos + rot @ np.asarray(self.camera.camera_offset, float)
            dirs = native_camera_rays(self.camera, self.profile.obstacle_native_wh).reshape(-1, 3) @ rot.T
            d = np.asarray(report.depth_native, float).reshape(-1)
            cat = classify_depth_native(d, self.profile)
            hit, noret, near = cat == VALID_HIT, cat == NO_RETURN, cat == NEAR_CLIPPED
            n_near += int(near.sum()); n_c = int((hit | noret).sum())
            # 10 m no-return: free up to the far plane, never beyond (the far end stays UNKNOWN).
            length = np.where(hit, d - self.p.hit_backoff_m, np.where(noret, self.profile.obstacle_far_m - self.p.cell_m, 0.0))
            free_c = self._cell_set(self._ray_samples(origin, dirs, np.nan_to_num(length)))
            occ_c = self._cell_set(self._hits(origin, dirs[hit], d[hit]))
        else:
            reasons.append("CAMERA_OBSTACLE_DEPTH_NOT_INTEGRATED")
        if n_near:
            self.near_warning_s = stamp; reasons.append("NEAR_CLIPPED_RETURNS")
        key = lambda a: set(map(tuple, a))
        occ = key(occ_l) | key(occ_c)
        free = (key(free_l) | key(free_c)) - occ                      # within a frame occupancy wins
        conflict = (key(free_l) & key(occ_c)) | (key(free_c) & key(occ_l))
        for cells, arr in ((free, self.free_s), (occ, self.occ_s), (conflict, self.conf_s)):
            if cells:
                ij = np.array(sorted(cells)); arr[ij[:, 0], ij[:, 1]] = stamp
        self.last_stamp_s, self.frames = stamp, self.frames + 1
        return FrameReceipt(True, tuple(reasons), stamp, n_l, n_c, len(free), len(occ), len(conflict), n_near)

    # --- queries -----------------------------------------------------------------------------
    def _state_cells(self, ij, inside, now_s):
        p = self.p
        out = np.full(len(ij), UNKNOWN, dtype=object)
        if self.origin_xy is None:
            return out
        i, j = np.clip(ij[:, 0], 0, p.size_cells - 1), np.clip(ij[:, 1], 0, p.size_cells - 1)
        fs, os_, cs = self.free_s[i, j], self.occ_s[i, j], self.conf_s[i, j]
        occ_recent = now_s - os_ <= p.occupied_ttl_s + 1e-9
        free_recent = now_s - fs <= p.free_ttl_s + 1e-9
        conf_recent = now_s - cs <= p.conflict_ttl_s + 1e-9
        future = (fs > now_s + 1e-9) | (os_ > now_s + 1e-9)          # query before the evidence: refuse it
        out[free_recent] = FREE
        out[conf_recent | (occ_recent & free_recent & (fs > os_))] = CONFLICT
        out[occ_recent & ~(free_recent & (fs > os_))] = OCCUPIED
        out[~inside | future] = UNKNOWN
        return out

    def state_at(self, xy, now_s):
        ij, inside = self._cells(np.atleast_2d(xy)) if self.origin_xy is not None else (np.zeros((1, 2), int), np.zeros(1, bool))
        return self._state_cells(ij, inside, now_s)[0]

    def _disc_offsets(self, radius):
        r = int(math.ceil(radius / self.p.cell_m))
        g = np.arange(-r, r + 1)
        di, dj = np.meshgrid(g, g, indexing="ij")
        keep = (di ** 2 + dj ** 2) * self.p.cell_m ** 2 <= (radius + self.p.cell_m * 0.71) ** 2
        return np.stack([di[keep], dj[keep]], axis=1)

    def segment_status(self, a_xy, b_xy, radius_m, now_s, step_m=None):
        """SAFE only if every cell within `radius_m` of every sample is FREE; BLOCKED if any is OCCUPIED;
        otherwise UNCERTIFIED (UNKNOWN or CONFLICT). Returns (status, unknown_fraction, reasons)."""
        if self.origin_xy is None:
            return UNCERTIFIED, 1.0, ("MAP_EMPTY",)
        a, b = np.asarray(a_xy, float), np.asarray(b_xy, float)
        step = step_m or self.p.cell_m / 2.0
        n = max(2, int(math.ceil(np.linalg.norm(b - a) / step)) + 1)
        samples = a + (b - a) * np.linspace(0.0, 1.0, n)[:, None]
        ij, inside = self._cells(samples)
        off = self._disc_offsets(radius_m)
        cells = (ij[:, None, :] + off[None, :, :]).reshape(-1, 2)
        ok = np.repeat(inside, len(off)) & (cells >= 0).all(axis=1) & (cells < self.p.size_cells).all(axis=1)
        st = self._state_cells(cells, ok, now_s).reshape(n, len(off))
        worst = np.array([max(row, key=_RANK.get) for row in st])
        unknown = float(np.mean([w in (UNKNOWN, CONFLICT) for w in worst]))
        if (worst == OCCUPIED).any():
            return BLOCKED, unknown, ("PATH_INTERSECTS_OBSERVED_OCCUPANCY",)
        if self.last_stamp_s is None or now_s - self.last_stamp_s > self.p.free_ttl_s + 1e-9:
            return UNCERTIFIED, unknown, ("MAP_STALE",)
        if (worst == CONFLICT).any():
            return UNCERTIFIED, unknown, ("PATH_ENTERS_CONFLICTING_EVIDENCE",)
        if (worst == UNKNOWN).any():
            return UNCERTIFIED, unknown, ("PATH_ENTERS_UNKNOWN_SPACE",)
        return SAFE, 0.0, ()

    def line_of_sight(self, a_xy, b_xy, now_s, radius_m=0.0):
        """CLEAR (all FREE) / BLOCKED (any OCCUPIED) / UNKNOWN for a thin line of sight."""
        status, _, _ = self.segment_status(a_xy, b_xy, radius_m, now_s)
        return "CLEAR" if status == SAFE else ("BLOCKED" if status == BLOCKED else "UNKNOWN")

    def clearance(self, xy, now_s, max_m=3.0):
        """Distance to the nearest non-FREE cell (OCCUPIED, CONFLICT or UNKNOWN), capped at max_m."""
        if self.origin_xy is None:
            return 0.0
        for k in range(0, int(max_m / self.p.cell_m) + 1):
            off = self._disc_offsets(k * self.p.cell_m)
            ij, inside = self._cells(np.atleast_2d(xy))
            cells = ij[0] + off
            ok = inside[0] & (cells >= 0).all(axis=1) & (cells < self.p.size_cells).all(axis=1)
            if (self._state_cells(cells, ok, now_s) != FREE).any():
                return k * self.p.cell_m
        return max_m

    def validity(self, now_s):
        return dict(frames=self.frames, last_stamp_s=self.last_stamp_s,
                    age_s=None if self.last_stamp_s is None else round(now_s - self.last_stamp_s, 9),
                    stale=self.last_stamp_s is None or now_s - self.last_stamp_s > self.p.free_ttl_s + 1e-9,
                    near_obstacle_warning=now_s - self.near_warning_s <= self.p.conflict_ttl_s,
                    z_ref=self.z_ref, ground_truth_used=False, representation="WORLD_ANCHORED_2D_GRID_ALTITUDE_SLICE")
