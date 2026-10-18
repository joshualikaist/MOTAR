"""Opt-in P02 command controls; the frozen P01 controller remains unchanged.

B4 randomizes only the lateral side and retains B3's visibility-derived magnitude.
B5 matches B3's same-state nominal horizontal command norm along pursuit. Neither
matches realized effort: point safety, the task governor and dynamics may attenuate
different directions differently. Nothing here reverses safety attenuation.
"""
from __future__ import annotations

import hashlib
import math

import torch

from motar_eval import h_r3_arms as v1

ARMS = ("B1", "B2", "B3", "B4", "B5")
PARAMS = v1.PARAMS


def sham_signs(seed, env_ids, *, device=None, dtype=None):
    """Fixed exogenous sides, independent of batch order, sensors and all RNG state."""
    ids = torch.as_tensor(env_ids).detach().cpu().reshape(-1).tolist()
    values = []
    for env_id in ids:
        if int(env_id) != env_id or int(env_id) < 0:
            raise ValueError("env_ids must be nonnegative integer identities")
        key = "HR3-P02|%d|%d" % (int(seed), int(env_id))
        bit = hashlib.sha256(key.encode("ascii")).digest()[0] & 1
        values.append(1.0 if bit else -1.0)
    return torch.tensor(values, device=device, dtype=dtype or torch.float32)


def _filter_stages(nominal, pts, max_velocity, params):
    """P01's exact point-projection order and cap, exposing both stages."""
    vel = nominal.clone()
    rng = pts.norm(dim=-1)
    near_r, near_i = rng.topk(params["safety_points"], dim=1, largest=False)
    for k in range(params["safety_points"]):
        r = near_r[:, k:k + 1]
        q = torch.gather(pts, 1, near_i[:, k].view(-1, 1, 1).expand(-1, 1, 2)).squeeze(1) / r.clamp(min=1e-6)
        w = ((params["safety_radius_m"] - r) / (params["safety_radius_m"] - params["safety_full_m"])).clamp(0.0, 1.0)
        comp = (vel * q).sum(dim=1, keepdim=True).clamp(min=0.0)
        vel = vel - w * comp * q
    post_point = vel
    speed = vel.norm(dim=1, keepdim=True)
    post_cap = vel * (max_velocity / speed.clamp(min=max_velocity))
    return {"nominal_xy": nominal, "post_point_xy": post_point, "post_cap_xy": post_cap,
            "nominal_norm_mps": nominal.norm(dim=1), "post_point_norm_mps": post_point.norm(dim=1),
            "post_cap_norm_mps": post_cap.norm(dim=1)}


def evaluate_actions(own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations, *,
                     sham_sign, max_velocity, yaw_rate_max, lidar_far_m, params=PARAMS):
    """Return actions B1--B5 and same-input stage diagnostics in physical m/s.

    Every call computes the identical set of candidates and all five arms. B0 can
    discard these outputs. The caller owns the fixed intervention state machine
    and the unchanged task governor. The only extra input is a fixed exogenous
    sham assignment; no evaluator information belongs in this interface.
    """
    for name, value in (("max_velocity", max_velocity), ("yaw_rate_max", yaw_rate_max), ("lidar_far_m", lidar_far_m)):
        if not math.isfinite(float(value)) or float(value) <= 0:
            raise ValueError("%s must be finite and positive" % name)
    n = own_pos_w.shape[0]
    if own_pos_w.shape != (n, 3) or belief_pos_w.shape != (n, 3) or vehicle_quat.shape != (n, 4):
        raise ValueError("pose/belief/quaternion batch shapes differ")
    if lidar_m.shape != (n, lidar_angles.numel() * lidar_elevations.numel()):
        raise ValueError("LiDAR shape differs from angles/elevations")
    tensors = (own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations)
    if any(not bool(torch.isfinite(t).all()) for t in tensors):
        raise ValueError("controller inputs must be finite")
    signs = torch.as_tensor(sham_sign, device=own_pos_w.device, dtype=own_pos_w.dtype)
    if signs.shape != (n,) or not bool(((signs == -1) | (signs == 1)).all()):
        raise ValueError("sham_sign must contain one fixed -1/+1 assignment per env")

    # The frozen implementation supplies candidate geometry and its reference action.
    reference, choice = v1.scripted_action(
        "B3", own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations,
        max_velocity=max_velocity, yaw_rate_max=yaw_rate_max, lidar_far_m=lidar_far_m, params=params)
    rel = v1._quat_rotate_inverse(vehicle_quat, belief_pos_w - own_pos_w)[:, :2]
    dist = rel.norm(dim=1, keepdim=True).clamp(min=1e-6)
    u = rel / dist
    normal = torch.stack([-u[:, 1], u[:, 0]], dim=1)
    lateral = (params["lateral_gain_per_s"] * choice["chosen_offset_m"]).clamp(
        -params["lateral_max_mps"], params["lateral_max_mps"])
    pursuit = params["pursuit_speed_mps"] * u
    nominal = {"B1": pursuit, "B2": pursuit,
               "B3": pursuit + lateral.unsqueeze(1) * normal,
               "B4": pursuit + (signs * lateral.abs()).unsqueeze(1) * normal,
               "B5": torch.sqrt(params["pursuit_speed_mps"] ** 2 + lateral.square()).unsqueeze(1) * u}
    pts = v1.lidar_points_xy(lidar_m, lidar_angles, lidar_elevations, lidar_far_m)
    yaw = (params["yaw_gain_per_s"] * torch.atan2(rel[:, 1], rel[:, 0])).clamp(-yaw_rate_max, yaw_rate_max) / yaw_rate_max
    actions, stages = {}, {}
    for arm in ARMS:
        stage = _filter_stages(nominal[arm], pts, max_velocity, params)
        action = torch.zeros(n, 4, device=rel.device, dtype=rel.dtype)
        action[:, :2] = stage["post_cap_xy"] / max_velocity
        if arm != "B1":
            action[:, 3] = yaw
        actions[arm] = action.clamp(-1.0, 1.0)
        stages[arm] = stage
    info = dict(choice, arms=stages, shadow_B2=stages["B2"], shadow_B3=stages["B3"],
                sham_sign=signs, lateral_mps=lateral, shadow_B3_reference_action=reference)
    return actions, info
