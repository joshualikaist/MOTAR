"""H-R3 pilot scripted arms B1/B2/B3 (preregistration docs/plans/h_r3_pilot_v1.json).

One controller with two switches: ``yaw`` (B2, B3) and ``translate`` (B3). Inputs are only what an
onboard system has: own pose and velocity, the online tracker belief, and the raw LiDAR scan from
the body origin. No target ground truth, occlusion class, renderer mask or LiDAR semantics.
The candidate-viewpoint evaluation runs in every scripted arm so compute is matched; B1/B2 discard
its lateral output. Pure functions of their inputs: calling them changes no simulator state.
"""
from __future__ import annotations

import math

import torch

ARMS = {"B1": (False, False), "B2": (True, False), "B3": (True, True)}  # (yaw, translate)
PARAMS = {
    "pursuit_speed_mps": 1.0,
    "candidate_offsets_m": [-2.0, -1.75, -1.5, -1.25, -1.0, -0.75, -0.5, -0.25, 0.0,
                            0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0],
    "los_clearance_m": 0.40,
    "path_clearance_m": 0.50,
    "belief_exclusion_m": 0.50,
    "lateral_gain_per_s": 1.5,
    "lateral_max_mps": 1.5,
    "yaw_gain_per_s": 2.0,
    "safety_radius_m": 1.5,
    "safety_full_m": 0.5,
    "safety_points": 5,
}


def _quat_rotate_inverse(q, v):
    """Rotate world vectors v [N,3] into the frame of unit quaternions q [N,4] (x, y, z, w)."""
    qv, w = -q[:, :3], q[:, 3:4]
    t = 2.0 * torch.cross(qv, v, dim=1)
    return v + w * t + torch.cross(qv, t, dim=1)


def lidar_points_xy(lidar_m, lidar_angles, lidar_elevations, far_m):
    """Horizontal obstacle points [N, V*H, 2] in the vehicle frame from raw ranges [N, V*H]."""
    n = lidar_m.shape[0]
    v, h = lidar_elevations.numel(), lidar_angles.numel()
    r = lidar_m.view(n, v, h)
    valid = (r > 0) & (r < far_m * 0.995)
    horiz = r * torch.cos(lidar_elevations).view(1, v, 1)
    pts = torch.stack([horiz * torch.cos(lidar_angles).view(1, 1, h), horiz * torch.sin(lidar_angles).view(1, 1, h)], dim=-1)
    pts = torch.where(valid.unsqueeze(-1), pts, torch.full_like(pts, 1e4))
    return pts.view(n, v * h, 2)


def _segment_distance(a, b, p):
    """Distance from points p [N,P,2] to segments a->b ([N,K,2] each) -> [N,K,P]."""
    ab = (b - a).unsqueeze(2)
    ap = p.unsqueeze(1) - a.unsqueeze(2)
    t = ((ap * ab).sum(-1) / (ab * ab).sum(-1).clamp(min=1e-12)).clamp(0.0, 1.0)
    return (ap - t.unsqueeze(-1) * ab).norm(dim=-1)


def scripted_action(arm, own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations, *,
                    max_velocity, yaw_rate_max, lidar_far_m, params=PARAMS):
    """Normalized actions [N,4] for arm B1/B2/B3 and per-env diagnostics."""
    yaw_on, translate_on = ARMS[arm]
    rel = _quat_rotate_inverse(vehicle_quat, belief_pos_w - own_pos_w)[:, :2]
    dist = rel.norm(dim=1, keepdim=True).clamp(min=1e-6)
    u = rel / dist
    nrm = torch.stack([-u[:, 1], u[:, 0]], dim=1)
    pts = lidar_points_xy(lidar_m, lidar_angles, lidar_elevations, lidar_far_m)
    # LiDAR returns near the belief are probably the target itself; they cannot block its sight line.
    near_belief = (pts - rel.unsqueeze(1)).norm(dim=-1) < params["belief_exclusion_m"]
    pts_los = torch.where(near_belief.unsqueeze(-1), torch.full_like(pts, 1e4), pts)
    offsets = torch.tensor(params["candidate_offsets_m"], device=rel.device, dtype=rel.dtype)
    cand = offsets.view(1, -1, 1) * nrm.unsqueeze(1)                       # [N,K,2]
    goal = rel.unsqueeze(1).expand_as(cand)
    los_ok = (_segment_distance(cand, goal, pts_los) >= params["los_clearance_m"]).all(dim=-1)
    path_ok = (_segment_distance(torch.zeros_like(cand), cand, pts) >= params["path_clearance_m"]).all(dim=-1)
    ok = los_ok & path_ok
    score = offsets.abs().view(1, -1) + 1e-3 * (offsets < 0).float().view(1, -1) + 1e6 * (~ok).float()
    best = score.argmin(dim=1)
    any_ok = ok.any(dim=1)
    chosen = torch.where(any_ok, offsets[best], torch.zeros_like(offsets[best]))
    vel = params["pursuit_speed_mps"] * u
    if translate_on:
        lateral = (params["lateral_gain_per_s"] * chosen).clamp(-params["lateral_max_mps"], params["lateral_max_mps"])
        vel = vel + lateral.unsqueeze(1) * nrm
    # Shared LiDAR safety filter: remove approach components toward the nearest points.
    rng = pts.norm(dim=-1)
    near_r, near_i = rng.topk(params["safety_points"], dim=1, largest=False)
    for k in range(params["safety_points"]):
        r = near_r[:, k:k + 1]
        q = torch.gather(pts, 1, near_i[:, k].view(-1, 1, 1).expand(-1, 1, 2)).squeeze(1) / r.clamp(min=1e-6)
        w = ((params["safety_radius_m"] - r) / (params["safety_radius_m"] - params["safety_full_m"])).clamp(0.0, 1.0)
        comp = (vel * q).sum(dim=1, keepdim=True).clamp(min=0.0)
        vel = vel - w * comp * q
    speed = vel.norm(dim=1, keepdim=True)
    vel = vel * (max_velocity / speed.clamp(min=max_velocity))
    actions = torch.zeros(rel.shape[0], 4, device=rel.device, dtype=rel.dtype)
    actions[:, 0:2] = vel / max_velocity
    if yaw_on:
        err = torch.atan2(rel[:, 1], rel[:, 0])
        actions[:, 3] = (params["yaw_gain_per_s"] * err).clamp(-yaw_rate_max, yaw_rate_max) / yaw_rate_max
    return actions.clamp(-1.0, 1.0), {"chosen_offset_m": chosen, "clear_found": any_ok, "belief_dist_m": dist.squeeze(1)}
