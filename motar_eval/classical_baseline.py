"""B1-R classical sensor-based baselines B-PN and B-PP (docs/plans/prereg_b1r_classical_baseline_v1.md §2).

Velocity-direction proportional navigation (B-PN) and pure pursuit (B-PP) on the online tracker
belief, followed by one shared LiDAR approach filter with target-return exclusion, a
direction-preserving per-axis speed bound and a yaw law that keeps the camera on the belief.

Inputs are only what an onboard system has: own pose and velocity, the online tracker mean and
active flag, the raw LiDAR scan and the sensor-associated target-return mask. No target ground
truth, renderer mask, semantic id, obstacle layout or critic state. Every function is pure,
deterministic and stateless; none draws random numbers or uses a matrix multiply or cdist (TF32
hazard), so clearances are explicit difference norms.
"""
from __future__ import annotations

import math

import torch

ARMS = ("B-PN", "B-PP")
PARAMS = {
    "dt_s": 0.1,
    "max_velocity": 2.5,
    "yaw_rate_max": 3.0,
    "lidar_far_m": 12.0,
    "slow_course_speed_mps": 0.2,
    "pn_turn_rate_max_radps": 3.0,
    "los_rate_eps_m2": 1e-6,
    "yaw_gain_per_s": 2.0,
    "scan_yaw_rate_radps": 0.6,
    "safety_radius_floor_m": 1.5,
    "reaction_s": 0.1,
    "brake_mps2": 2.9608856678,
    "safety_full_m": 0.5,
    "safety_points": 5,
    "belief_exclusion_m": 0.5,
    "use_target_like_mask": True,
}
FAR = 1e4


def quat_rotate_inverse(q, v):
    """Rotate world vectors v [N,3] into the frame of unit quaternions q [N,4] (x, y, z, w)."""
    qv, w = -q[:, :3], q[:, 3:4]
    t = 2.0 * torch.cross(qv, v, dim=1)
    return v + w * t + torch.cross(qv, t, dim=1)


def los_geometry(rel_xy, rel_vel_xy, eps=PARAMS["los_rate_eps_m2"]):
    """LOS angle and rate [N] from relative position and velocity [N,2] (world XY)."""
    lam = torch.atan2(rel_xy[:, 1], rel_xy[:, 0])
    cross = rel_xy[:, 0] * rel_vel_xy[:, 1] - rel_xy[:, 1] * rel_vel_xy[:, 0]
    rate = cross / (rel_xy * rel_xy).sum(dim=1).clamp(min=eps)
    return lam, rate


def course_command(arm, rel_xy, rel_vel_xy, own_vel_xy, *, n_gain, params=PARAMS):
    """Commanded world course [N] and diagnostics for B-PN or B-PP."""
    if arm not in ARMS:
        raise ValueError("unknown arm %r" % (arm,))
    lam, rate = los_geometry(rel_xy, rel_vel_xy, params["los_rate_eps_m2"])
    slow = (own_vel_xy * own_vel_xy).sum(dim=1).sqrt() < params["slow_course_speed_mps"]
    if arm == "B-PP":
        return lam, {"los_rate": rate, "slow": slow, "pn_saturated": torch.zeros_like(slow)}
    turn_max = params["pn_turn_rate_max_radps"]
    turn = float(n_gain) * rate
    saturated = turn.abs() > turn_max
    psi_v = torch.atan2(own_vel_xy[:, 1], own_vel_xy[:, 0])
    psi = psi_v + turn.clamp(-turn_max, turn_max) * params["dt_s"]
    psi = torch.where(slow, lam, psi)
    return psi, {"los_rate": rate, "slow": slow, "pn_saturated": saturated & ~slow}


def lidar_points_xy(lidar_m, lidar_angles, lidar_elevations, far_m):
    """Horizontal points [N, V*H, 2] in the vehicle frame from raw ranges [N, V*H] (h_r3_arms layout)."""
    n = lidar_m.shape[0]
    v, h = lidar_elevations.numel(), lidar_angles.numel()
    r = lidar_m.view(n, v, h)
    valid = (r > 0) & (r < far_m * 0.995)
    horiz = r * torch.cos(lidar_elevations).view(1, v, 1)
    pts = torch.stack([horiz * torch.cos(lidar_angles).view(1, 1, h),
                       horiz * torch.sin(lidar_angles).view(1, 1, h)], dim=-1)
    pts = torch.where(valid.unsqueeze(-1), pts, torch.full_like(pts, FAR))
    return pts.view(n, v * h, 2)


def exclude_target_returns(pts, target_like, belief_rel_xy, belief_active, radius_m):
    """Set returns that belong to the target to FAR: (a) the sensor-associated mask, (b) near the belief.

    ``target_like`` is [N, V, H] bool (or None); ``belief_rel_xy`` is the belief in the vehicle frame.
    """
    n, p = pts.shape[0], pts.shape[1]
    far = torch.full_like(pts, FAR)
    by_mask = (torch.zeros(n, p, dtype=torch.bool, device=pts.device) if target_like is None
               else target_like.reshape(n, p).bool())
    near = ((pts - belief_rel_xy.unsqueeze(1)).norm(dim=-1) < radius_m) & belief_active.view(-1, 1)
    real = pts[..., 0].abs() < FAR * 0.5
    by_mask = by_mask & real
    by_belief = near & real & ~by_mask
    out = torch.where((by_mask | by_belief).unsqueeze(-1), far, pts)
    return out, {"excluded_mask": by_mask.sum(dim=1), "excluded_belief": by_belief.sum(dim=1)}


def safety_radius(v_nom, params=PARAMS):
    """Filter onset r_s: full-removal distance plus the governor-model stopping distance, floored at H-R3's 1.5 m."""
    v = float(v_nom)
    stop = params["reaction_s"] * v + v * v / (2.0 * params["brake_mps2"])
    return max(params["safety_radius_floor_m"], params["safety_full_m"] + stop)


def approach_filter(vel, pts, radius, params=PARAMS):
    """H-R3 shared safety filter (h_r3_arms.py:85-93): remove approach components toward the nearest points."""
    full, k_pts = params["safety_full_m"], int(params["safety_points"])
    rng = pts.norm(dim=-1)
    near_r, near_i = rng.topk(k_pts, dim=1, largest=False)
    active = torch.zeros(vel.shape[0], dtype=torch.bool, device=vel.device)
    for k in range(k_pts):
        r = near_r[:, k:k + 1]
        q = torch.gather(pts, 1, near_i[:, k].view(-1, 1, 1).expand(-1, 1, 2)).squeeze(1) / r.clamp(min=1e-6)
        w = ((radius - r) / (radius - full)).clamp(0.0, 1.0)
        comp = (vel * q).sum(dim=1, keepdim=True).clamp(min=0.0)
        active |= ((w * comp) > 0).squeeze(1)
        vel = vel - w * comp * q
    return vel, active


def bound_velocity(vel, max_velocity):
    """Direction-preserving per-axis bound: normalized XY action in [-1, 1]."""
    scale = (vel.abs().amax(dim=1, keepdim=True) / max_velocity).clamp(min=1.0)
    return (vel / scale) / max_velocity


def baseline_action(arm, own_pos_w, own_vel_w, vehicle_quat, belief_state_w, belief_active, lidar_m,
                    lidar_angles, lidar_elevations, target_like, *, n_gain, v_nom, params=PARAMS):
    """Normalized actions [N,4] = [vx, vy, 0, yaw] (vehicle frame) and per-env diagnostics."""
    active = belief_active.bool()
    rel_w = belief_state_w[:, 0:3] - own_pos_w
    rel_vel_w = belief_state_w[:, 3:6] - own_vel_w
    psi, info = course_command(arm, rel_w[:, 0:2], rel_vel_w[:, 0:2], own_vel_w[:, 0:2],
                               n_gain=n_gain, params=params)
    nominal_w = torch.stack([torch.cos(psi), torch.sin(psi), torch.zeros_like(psi)], dim=1) * float(v_nom)
    vel = quat_rotate_inverse(vehicle_quat, nominal_w)[:, 0:2]
    rel_v = quat_rotate_inverse(vehicle_quat, rel_w)[:, 0:2]
    pts = lidar_points_xy(lidar_m, lidar_angles, lidar_elevations, params["lidar_far_m"])
    mask = target_like if params["use_target_like_mask"] else None
    pts, excluded = exclude_target_returns(pts, mask, rel_v, active, params["belief_exclusion_m"])
    vel, filtered = approach_filter(vel, pts, safety_radius(v_nom, params), params)
    max_v, yaw_max = params["max_velocity"], params["yaw_rate_max"]
    xy = bound_velocity(vel, max_v)
    yaw = (params["yaw_gain_per_s"] * torch.atan2(rel_v[:, 1], rel_v[:, 0])).clamp(-yaw_max, yaw_max) / yaw_max
    scan = torch.full_like(yaw, params["scan_yaw_rate_radps"] / yaw_max)
    actions = torch.zeros(own_pos_w.shape[0], 4, device=own_pos_w.device, dtype=own_pos_w.dtype)
    actions[:, 0:2] = torch.where(active.view(-1, 1), xy, torch.zeros_like(xy))
    actions[:, 3] = torch.where(active, yaw, scan)
    info.update(excluded)
    info.update(no_belief=~active, filter_active=filtered & active,
                pn_saturated=info["pn_saturated"] & active, slow=info["slow"] & active,
                raw_out_of_bounds=(actions.abs() > 1.0 + 1e-6).any(dim=1))
    return actions.clamp(-1.0, 1.0), info


def check_action(actions):
    """Counts of non-finite rows, out-of-bound rows and non-zero z (G2 runtime gate)."""
    finite = torch.isfinite(actions).all(dim=1)
    return {"non_finite": int((~finite).sum()), "out_of_bounds": int((actions.abs() > 1.0 + 1e-6).any(dim=1).sum()),
            "nonzero_z": int((actions[:, 2] != 0).sum())}

