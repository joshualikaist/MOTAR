"""Q03 reacquisition candidate planner v0 (CPU, numpy). Proposes; never executes.

Candidates are short straight-line viewpoint moves around the current position, each ending facing the target
belief mean (lateral detours left/right of the line of sight, advance, retreat, rotate in place). Each one is
evaluated against the sensor-only LocalObstacleMap (Q03 local map v0) and the tracker belief (Q02-B), with the
score kept separate from hard feasibility.

Feasibility is reported at four distinct levels; this module can only establish the first two:
  L1 OBSERVED_DIRECTION_OPEN   the straight path does not cross observed occupancy (it may cross UNKNOWN);
  L2 KINEMATIC_CHECK           the path lies in observed-free space for the body radius, speed / yaw-rate /
                               velocity-change / travel-time caps hold, and the stopping distance at the cruise
                               speed fits inside the known-free clearance at the end point (CPU approximation);
  L3 DYNAMIC_FEASIBILITY       NOT_VALIDATED: no qualified plant/controller response for these trajectories;
  L4 CLOSED_LOOP_SAFETY        NOT_VALIDATED: no trajectory follower exists (Frozen F has only a velocity
                               controller), so nothing here was ever flown.

The visibility score is the weighted share of deterministic belief sigma points that would be inside the
camera FOV and range, with a CLEAR line of sight in the map (UNKNOWN lines of sight are reported separately,
never counted as visible). It is an uncalibrated ordinal score, not a probability.

Reused (ADAPTED) from motar_eval/boia:
- visibility.fov_mask -> fov_mask (numpy port, equality-tested);
- dynamics.braking_distance -> braking_distance (numpy port, equality-tested);
- the safety.screen idea that stopping must fit inside known-free space.

Not reused:
- boia.candidates (arc offsets of F's own command) and boia.dynamics.rollout (unqualified planar response
  model): the Q03 output is a viewpoint near the target belief, not a perturbation of F's command.
"""
from dataclasses import dataclass
import math

import numpy as np

from . import obstacle_map as om
from .contracts import FROZEN_F_CAMERA

PASS, FAIL, NOT_VALIDATED = "PASS", "FAIL", "NOT_VALIDATED"
NO_FEASIBLE_PLAN = "NO_FEASIBLE_PLAN"
SCORE_SEMANTICS = "SIGMA_POINT_VISIBLE_SHARE_UNCALIBRATED_NOT_A_PROBABILITY"


@dataclass(frozen=True)
class MotionLimits:
    """Declared priors for the CPU kinematic check, not qualified against the plant."""
    speed_cap_mps: float = 1.5            # h_r3 arc cap (below Frozen F's 2.5 m/s action scale)
    yaw_rate_cap: float = 3.0             # rad/s, Frozen F yaw_rate_max
    accel_cap_mps2: float = 2.0           # declared; no qualified acceleration envelope exists
    max_travel_s: float = 3.0
    body_radius_m: float = 0.45           # riskcap half-width (hard collision radius 0.14 m is smaller)
    t_react_s: float = 0.1                # one decision period
    braking_table: tuple = ((2.0, 0.82), (2.5, 1.05), (3.52, 1.57))   # boia.dynamics.braking_distance default
    max_velocity_cancel_s: float = 1.0    # budget to cancel cross-path / opposing velocity before the move


@dataclass(frozen=True)
class PlannerParams:
    lateral_m: tuple = (0.75, 1.5)
    advance_m: tuple = (1.0,)
    retreat_m: tuple = (1.0,)
    dt_s: float = 0.1
    camera_range_m: float = 20.0
    min_score_gain: float = 1e-9


@dataclass(frozen=True)
class Candidate:
    candidate_id: str
    kind: str                      # LEFT / RIGHT / ADVANCE / RETREAT / ROTATE
    position_xy: tuple
    heading_rad: float
    trajectory: tuple              # ((t, x, y, yaw), ...) at dt_s, straight line + rate-limited yaw
    visibility_score: float        # uncalibrated, SCORE_SEMANTICS: share with a CLEAR line of sight in FOV
    visibility_unknown_share: float  # share in FOV whose line of sight crosses UNKNOWN space (possible gain)
    visibility_blocked_share: float  # share in FOV whose line of sight hits observed occupancy
    current_visibility_score: float
    current_blocked_share: float
    known_space_clearance_m: float
    unknown_exposure: float        # share of path samples whose body disc is not all FREE
    travel_time_s: float
    l1_observed_direction_open: str
    l2_kinematic_check: str
    l3_dynamic_feasibility: str = NOT_VALIDATED
    l4_closed_loop_safety: str = NOT_VALIDATED
    reasons: tuple = ()
    score_semantics: str = SCORE_SEMANTICS


def fov_mask(rel_v, hfov, vfov, max_range):
    """numpy port of motar_eval.boia.visibility.fov_mask: pinhole frustum |y| <= x tan(hfov/2), |z| <= x tan(vfov/2),
    x > 0 and radial range <= max_range, for vehicle-camera relative vectors (..., 3)."""
    rel = np.asarray(rel_v, float)
    x, y, z = rel[..., 0], rel[..., 1], rel[..., 2]
    return (np.isfinite(rel).all(-1) & (x > 0) & (np.abs(y) <= x * math.tan(hfov / 2))
            & (np.abs(z) <= x * math.tan(vfov / 2)) & ((rel * rel).sum(-1) <= max_range ** 2))


def braking_distance(speed, table, t_react):
    """numpy port of motar_eval.boia.dynamics.braking_distance (returns distance, extrapolated flag)."""
    v = np.asarray(speed, float)
    first_v, first_d = table[0]
    accel = first_v ** 2 / (2 * first_d)
    comp = v ** 2 / (2 * accel)
    for (va, da), (vb, db) in zip(table[:-1], table[1:]):
        comp = np.where((v >= va) & (v <= vb), da + (v - va) / (vb - va) * (db - da), comp)
    last_v, last_d = table[-1]
    extra = v > last_v
    comp = np.where(extra, last_d + (v ** 2 - last_v ** 2) / (2 * accel), comp)
    return v * t_react + comp, extra


def sigma_points(mean_xyz, cov3, k=2.0):
    """Deterministic belief samples: the mean and +-k sigma along each principal axis (weights 1/3 mean, rest equal)."""
    mean = np.asarray(mean_xyz, float)
    lam, vec = np.linalg.eigh(0.5 * (np.asarray(cov3, float) + np.asarray(cov3, float).T))
    pts = [mean] + [mean + s * k * math.sqrt(max(l, 0.0)) * vec[:, i] for i, l in enumerate(lam) for s in (1.0, -1.0)]
    w = np.array([1.0 / 3.0] + [2.0 / 3.0 / 6.0] * 6)
    return np.array(pts), w


def _view(m, pos_xy, heading, z, pts, w, now_s, camera, params):
    """(clear, unknown, blocked) shares of sigma points in FOV from a camera at pos_xy/heading."""
    cs, sn = math.cos(heading), math.sin(heading)
    off = np.asarray(camera.camera_offset, float)
    cam = np.array([pos_xy[0] + cs * off[0] - sn * off[1], pos_xy[1] + sn * off[0] + cs * off[1], z + off[2]])
    rel = pts - cam
    rel_v = np.stack([cs * rel[:, 0] + sn * rel[:, 1], -sn * rel[:, 0] + cs * rel[:, 1], rel[:, 2]], axis=1)
    infov = fov_mask(rel_v, camera.hfov, camera.vfov, params.camera_range_m)
    vis = unk = blk = 0.0
    for p, wi, inside in zip(pts, w, infov):
        if not inside:
            continue
        d = p[:2] - cam[:2]; r = float(np.linalg.norm(d))
        end = cam[:2] + d * max(r - 0.55, 0.0) / max(r, 1e-9)   # stop short of the target proxy (0.20 + 0.15 + 0.20 guard)
        los = m.line_of_sight(cam[:2], end, now_s)
        vis += wi * (los == "CLEAR"); unk += wi * (los == "UNKNOWN"); blk += wi * (los == "BLOCKED")
    return float(vis), float(unk), float(blk)


def plan(m, position, yaw, velocity, belief, now_s, camera=FROZEN_F_CAMERA, limits=MotionLimits(), params=PlannerParams()):
    """Candidates with separate score and feasibility levels, plus a plan status (None or NO_FEASIBLE_PLAN) and reasons."""
    reasons = []
    if not belief.valid:
        return (), NO_FEASIBLE_PLAN, ("BELIEF_INVALID",) + tuple(belief.reasons)
    v = m.validity(now_s)
    if v["frames"] == 0:
        return (), NO_FEASIBLE_PLAN, ("LOCAL_MAP_EMPTY",)
    if v["stale"]:
        reasons.append("LOCAL_MAP_STALE")             # nothing can pass L2 (no FREE cells), recorded explicitly
    pos = np.asarray(position, float); vel = np.asarray(velocity, float)
    mean = np.asarray(belief.mean[:3], float); cov = np.asarray(belief.cov)[:3, :3]
    pts, w = sigma_points(mean, cov)
    los_dir = mean[:2] - pos[:2]; dist_t = float(np.linalg.norm(los_dir))
    u = los_dir / max(dist_t, 1e-9); left = np.array([-u[1], u[0]])
    face = lambda xy: math.atan2(mean[1] - xy[1], mean[0] - xy[0])
    current_vis, current_unk, current_blk = _view(m, pos[:2], yaw, pos[2], pts, w, now_s, camera, params)
    specs = [("ROTATE", pos[:2])]
    specs += [("LEFT", pos[:2] + d * left) for d in params.lateral_m] + [("RIGHT", pos[:2] - d * left) for d in params.lateral_m]
    specs += [("ADVANCE", pos[:2] + d * u) for d in params.advance_m] + [("RETREAT", pos[:2] - d * u) for d in params.retreat_m]
    out = []
    for i, (kind, xy) in enumerate(specs):
        why = []
        heading = face(xy)
        seg, unknown, seg_why = m.segment_status(pos[:2], xy, limits.body_radius_m, now_s)
        why += list(seg_why)
        dist = float(np.linalg.norm(xy - pos[:2]))
        dyaw = abs(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
        # Trapezoidal speed profile from rest along the path (accel cap, speed cap, stop at the end); velocity that
        # must first be cancelled (cross-path or opposing component of the current velocity) is budgeted separately.
        a_max, v_max = limits.accel_cap_mps2, limits.speed_cap_mps
        d_ramp = v_max ** 2 / a_max
        t_move = (2.0 * math.sqrt(dist / a_max) if dist < d_ramp else dist / v_max + v_max / a_max) if dist > 0 else 0.0
        v_peak = min(v_max, math.sqrt(a_max * dist)) if dist > 0 else 0.0
        u_path = (xy - pos[:2]) / max(dist, 1e-9) if dist > 0 else np.zeros(2)
        v_along = float(vel[:2] @ u_path); v_cross = float(np.linalg.norm(vel[:2] - v_along * u_path))
        t_cancel = (v_cross + max(0.0, -v_along)) / a_max
        travel = max(t_move + t_cancel, dyaw / limits.yaw_rate_cap)
        stop, extra = braking_distance(np.array([v_peak]), limits.braking_table, limits.t_react_s)
        stop = float(stop[0])
        clearance = m.clearance(xy, now_s, max_m=3.0)
        path_clear = min([m.clearance(pos[:2] + (xy - pos[:2]) * f, now_s, max_m=3.0) for f in np.linspace(0, 1, 6)])
        l1 = FAIL if seg == om.BLOCKED else PASS
        kin = []
        if seg != om.SAFE:
            kin.append("PATH_NOT_IN_OBSERVED_FREE")
        if travel > limits.max_travel_s:
            kin.append("TRAVEL_TIME_ABOVE_LIMIT")
        if t_cancel > limits.max_velocity_cancel_s:
            kin.append("EXCESSIVE_VELOCITY_CHANGE")
        # Stopping screen (adapted from boia.safety stop_ok): the overshoot segment from the endpoint along the path, of the
        # braking distance at the peak speed, must lie in observed FREE for the body radius.
        if dist > 0 and m.segment_status(xy, xy + stop * u_path, limits.body_radius_m, now_s)[0] != om.SAFE:
            kin.append("STOPPING_DISTANCE_EXCEEDS_KNOWN_FREE")
        l2 = PASS if (l1 == PASS and not kin) else FAIL
        why += kin
        vis, vis_unk, vis_blk = _view(m, xy, heading, pos[2], pts, w, now_s, camera, params)
        # Gain = more sigma points with a CLEAR line of sight, or more VIEWABLE ones (inside the FOV and not blocked:
        # CLEAR or UNKNOWN). The latter covers FOV recovery and turning a blocked view into an unknown one; both are
        # possible information gains, never certified views.
        if not (vis > current_vis + params.min_score_gain
                or vis + vis_unk > current_vis + current_unk + params.min_score_gain):
            why.append("NO_VISIBILITY_GAIN")
        n = max(1, int(math.ceil(travel / params.dt_s)))
        traj = []
        for k in range(n + 1):
            t = min(k * params.dt_s, travel)
            tm_ = max(0.0, min(t - t_cancel, t_move))
            if dist <= 0:
                s_t = 0.0
            elif dist < d_ramp:
                h_ = t_move / 2.0
                s_t = 0.5 * a_max * tm_ ** 2 if tm_ <= h_ else dist - 0.5 * a_max * (t_move - tm_) ** 2
            else:
                t1 = v_max / a_max
                s_t = (0.5 * a_max * tm_ ** 2 if tm_ <= t1 else
                       0.5 * a_max * t1 ** 2 + v_max * (tm_ - t1) if tm_ <= t_move - t1 else dist - 0.5 * a_max * (t_move - tm_) ** 2)
            frac = min(1.0, s_t / dist) if dist > 0 else 1.0
            yk = yaw + math.copysign(min(dyaw, t * limits.yaw_rate_cap), math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
            p = pos[:2] + (xy - pos[:2]) * frac
            traj.append((round(t, 6), float(p[0]), float(p[1]), float(yk)))
        out.append(Candidate("Q03C%02d_%s" % (i, kind), kind, (float(xy[0]), float(xy[1])), float(heading), tuple(traj),
                             vis, vis_unk, vis_blk, current_vis, current_blk, float(path_clear), float(unknown), float(travel), l1, l2,
                             reasons=tuple(dict.fromkeys(why))))
    useful = [cnd for cnd in out if cnd.l2_kinematic_check == PASS and "NO_VISIBILITY_GAIN" not in cnd.reasons]
    if not useful:
        return tuple(out), NO_FEASIBLE_PLAN, tuple(reasons) + ("NO_CANDIDATE_PASSES_L2_WITH_VISIBILITY_GAIN",)
    return tuple(out), None, tuple(reasons)
