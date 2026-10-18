"""Opt-in P03 qualification controls with a common, frozen prefix budget.

Only own pose, permitted tracker belief and raw LiDAR enter these functions.
The caller freezes one schedule before loss and supplies the same event schedule
to B3/B4/B5. Each arm subsequently uses only its own current measurements. Shared
nominal budgets do not imply equal post-safety commands or realized motion.
Frozen Policy F (B0), its governor, and every historical controller are unchanged.
"""
from __future__ import annotations

import hashlib
import math

import torch

from motar_eval import h_r3_arms as v1
from motar_eval import h_r3_p02_arms as v2

ARMS = ("B0", "B1", "B2", "B3", "B4", "B5")
SCRIPTED_ARMS = ARMS[1:]
PARAMS = v1.PARAMS
WINDOW_STEPS = 20
DT_S = 0.1


def _identities(value, count, name):
    values = torch.as_tensor(value).detach().cpu().reshape(-1).tolist()
    if len(values) != count:
        raise ValueError("%s must contain one identity per environment" % name)
    if any(isinstance(x, bool) or not math.isfinite(float(x)) or int(x) != x or x < 0 for x in values):
        raise ValueError("%s must contain nonnegative integer identities" % name)
    return [int(x) for x in values]


def sham_signs(seed, env_ids, episode_ids, event_ids, decision_ids, *, device=None, dtype=None):
    """Exogenous SHA assignment; no reset order, batch state or random generator.

    The hash domain and all five integer identities are recorded by the runner.
    A decision is its index in the registered intervention window, not call count.
    """
    seed = _identities([seed], 1, "seed")[0]
    env_values = torch.as_tensor(env_ids).reshape(-1)
    count = env_values.numel()
    keys = [_identities(values, count, name) for values, name in
            ((env_values, "env_ids"), (episode_ids, "episode_ids"),
             (event_ids, "event_ids"), (decision_ids, "decision_ids"))]
    signs = []
    for identity in zip(*keys):
        key = "HR3-P03|%d|%d|%d|%d|%d" % ((seed,) + identity)
        signs.append(1.0 if hashlib.sha256(key.encode("ascii")).digest()[0] & 1 else -1.0)
    return torch.tensor(signs, device=device, dtype=dtype or torch.float32)


def _validate(own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations,
              max_velocity, yaw_rate_max, lidar_far_m):
    for name, value in (("max_velocity", max_velocity), ("yaw_rate_max", yaw_rate_max),
                        ("lidar_far_m", lidar_far_m)):
        if not math.isfinite(float(value)) or float(value) <= 0:
            raise ValueError("%s must be finite and positive" % name)
    n = own_pos_w.shape[0]
    if own_pos_w.shape != (n, 3) or belief_pos_w.shape != (n, 3) or vehicle_quat.shape != (n, 4):
        raise ValueError("pose/belief/quaternion batch shapes differ")
    if lidar_angles.ndim != 1 or lidar_elevations.ndim != 1 or lidar_m.shape != (
            n, lidar_angles.numel() * lidar_elevations.numel()):
        raise ValueError("LiDAR shape differs from angles/elevations")
    tensors = (own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations)
    if any(not bool(torch.isfinite(t).all()) for t in tensors):
        raise ValueError("controller inputs must be finite")


def _candidates(own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations,
                lidar_far_m, params):
    """Frozen P01 candidate geometry, with explicit independent side scores."""
    rel = v1._quat_rotate_inverse(vehicle_quat, belief_pos_w - own_pos_w)[:, :2]
    raw_distance = rel.norm(dim=1, keepdim=True)
    direction_valid = raw_distance >= 1e-6
    direction = torch.where(direction_valid, rel / raw_distance.clamp(min=1e-6),
                            torch.zeros_like(rel))
    normal = torch.stack([-direction[:, 1], direction[:, 0]], dim=1)
    pts = v1.lidar_points_xy(lidar_m, lidar_angles, lidar_elevations, lidar_far_m)
    near_belief = (pts - rel.unsqueeze(1)).norm(dim=-1) < params["belief_exclusion_m"]
    pts_los = torch.where(near_belief.unsqueeze(-1), torch.full_like(pts, 1e4), pts)
    offsets = torch.tensor(params["candidate_offsets_m"], device=rel.device, dtype=rel.dtype)
    candidates = offsets.view(1, -1, 1) * normal.unsqueeze(1)
    goal = rel.unsqueeze(1).expand_as(candidates)
    los_ok = (v1._segment_distance(candidates, goal, pts_los) >= params["los_clearance_m"]).all(-1)
    path_ok = (v1._segment_distance(torch.zeros_like(candidates), candidates, pts) >= params["path_clearance_m"]).all(-1)
    eligible = los_ok & path_ok
    scores = offsets.abs().view(1, -1) + 1e-3 * (offsets < 0).float().view(1, -1) + 1e6 * (~eligible).float()
    best = scores.argmin(dim=1)
    any_ok = eligible.any(dim=1)
    chosen = torch.where(any_ok, offsets[best], torch.zeros_like(offsets[best]))
    # The budget is frozen separately. Side scoring excludes the centre, so the
    # informed arm can choose a sign even when the current centre is unobstructed.
    minus = scores[:, offsets < 0].min(dim=1).values
    plus = scores[:, offsets > 0].min(dim=1).values
    selected_side = torch.where(plus <= minus, torch.ones_like(plus), -torch.ones_like(plus))
    info = {"chosen_offset_m": chosen, "clear_found": any_ok,
            "belief_dist_m": raw_distance[:, 0], "candidate_offsets_m": offsets,
            "candidate_scores": scores, "candidate_eligible": eligible,
            "negative_side_score": minus, "positive_side_score": plus,
            "negative_side_feasible": eligible[:, offsets < 0].any(dim=1),
            "positive_side_feasible": eligible[:, offsets > 0].any(dim=1),
            "selected_side": selected_side, "direction_valid": direction_valid[:, 0]}
    return rel, direction, normal, pts, info


def freeze_schedule(own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations, *,
                    max_velocity, yaw_rate_max, lidar_far_m, params=PARAMS):
    """Create a detached constant 20 x 0.1s reference from one pre-loss state.

    The largest absolute nonzero prefix candidate that meets the unchanged
    LiDAR LOS/path criteria sets the magnitude, independently of its side/score.
    This gives a nonzero reference when the centre is best but lateral motion is
    feasible. No feasible lateral candidate or no belief direction gives zero;
    these events remain in the denominator. Magnitude is never re-estimated.
    The caller owns event identity and stores the schedule before branching.
    """
    _validate(own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations,
              max_velocity, yaw_rate_max, lidar_far_m)
    if max_velocity < params["pursuit_speed_mps"]:
        raise ValueError("common budget requires max_velocity >= frozen pursuit speed")
    *_, info = _candidates(own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles,
                          lidar_elevations, lidar_far_m, params)
    offsets = info["candidate_offsets_m"]
    eligible_lateral = info["candidate_eligible"] & (offsets[None, :] != 0)
    largest_offset = torch.where(eligible_lateral, offsets.abs()[None, :],
                                 torch.zeros_like(info["candidate_scores"])).max(dim=1).values
    largest_offset = torch.where(info["direction_valid"], largest_offset, torch.zeros_like(largest_offset))
    lateral = (params["lateral_gain_per_s"] * largest_offset).clamp(
        max=params["lateral_max_mps"]).detach().clone()
    budget_cap = math.sqrt(max(0.0, max_velocity ** 2 - params["pursuit_speed_mps"] ** 2))
    lateral = lateral.clamp(max=budget_cap)
    lateral = lateral[:, None].repeat(1, WINDOW_STEPS)
    reference = torch.sqrt(params["pursuit_speed_mps"] ** 2 + lateral.square())
    # Round the prefix budget down if its float32 square root rounded above the
    # physical scalar cap. This reduces a reference; it never undoes attenuation.
    for _ in range(16):
        if not bool((reference.double() > float(max_velocity)).any()):
            break
        over = reference.double() > float(max_velocity)
        lateral = torch.where(over, torch.nextafter(lateral, torch.zeros_like(lateral)), lateral)
        reference = torch.sqrt(params["pursuit_speed_mps"] ** 2 + lateral.square())
    if bool((reference.double() > float(max_velocity)).any()):
        raise ValueError("common budget cannot be represented below its scalar speed cap")
    return {"lateral_abs_mps": lateral, "reference_norm_mps": reference,
            "zero_lateral": lateral[:, 0] == 0,
            "prefix_chosen_offset_m": info["chosen_offset_m"].detach().clone(),
            "prefix_budget_offset_abs_m": largest_offset.detach().clone(),
            "prefix_candidate_scores": info["candidate_scores"].detach().clone(),
            "prefix_clear_found": info["clear_found"].detach().clone(),
            "window_steps": WINDOW_STEPS, "dt_s": DT_S}


def evaluate_actions(own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations, *,
                     schedule, decision_index, sham_sign, max_velocity, yaw_rate_max, lidar_far_m,
                     frozen_action=None, params=PARAMS):
    """Return B1--B5; optionally pass through the already computed B0 action.

    B3 selects an informed sign online; B4 uses its exogenous sign; B5 pursues.
    All three use the same registered prefix norm before identical safety rules.
    A coincident own/belief position yields zero command for all three, with an
    explicit invalid-direction diagnostic; it never invents a pursuit direction.
    """
    _validate(own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations,
              max_velocity, yaw_rate_max, lidar_far_m)
    n = own_pos_w.shape[0]
    rel, direction, normal, pts, info = _candidates(
        own_pos_w, vehicle_quat, belief_pos_w, lidar_m, lidar_angles, lidar_elevations, lidar_far_m, params)
    signs = torch.as_tensor(sham_sign, device=own_pos_w.device, dtype=own_pos_w.dtype)
    if signs.shape != (n,) or not bool(((signs == -1) | (signs == 1)).all()):
        raise ValueError("sham_sign requires exactly one -1/+1 per environment")
    indices = torch.as_tensor(decision_index, device=own_pos_w.device)
    if indices.ndim == 0:
        indices = indices.expand(n)
    if indices.shape != (n,) or not bool(torch.isfinite(indices).all()) or not bool(
            ((indices >= 0) & (indices < WINDOW_STEPS) & (indices == indices.long())).all()):
        raise ValueError("decision_index must be an integer in the registered 0..19 window")
    indices = indices.long()
    if schedule.get("window_steps") != WINDOW_STEPS or schedule.get("dt_s") != DT_S:
        raise ValueError("schedule contract differs from the registered window")
    lateral_all = schedule["lateral_abs_mps"]
    reference_all = schedule["reference_norm_mps"]
    for value in (lateral_all, reference_all):
        if value.shape != (n, WINDOW_STEPS) or value.device != own_pos_w.device or not bool(torch.isfinite(value).all()):
            raise ValueError("schedule shape/device/finite contract failed")
    if not bool(((lateral_all >= 0) & (lateral_all <= params["lateral_max_mps"])).all()):
        raise ValueError("schedule exceeds the registered lateral cap")
    if not torch.equal(lateral_all, lateral_all[:, :1].expand_as(lateral_all)):
        raise ValueError("schedule must retain its frozen constant prefix magnitude")
    expected = torch.sqrt(params["pursuit_speed_mps"] ** 2 + lateral_all.square())
    if not torch.equal(reference_all, expected):
        raise ValueError("schedule norm is inconsistent with its common lateral budget")
    if not bool((reference_all.double() <= float(max_velocity)).all()):
        raise ValueError("schedule exceeds the registered maximum nominal speed")
    rows = torch.arange(n, device=own_pos_w.device)
    lateral = lateral_all[rows, indices]
    reference = reference_all[rows, indices]
    pursuit = params["pursuit_speed_mps"] * direction
    nominal = {"B1": pursuit, "B2": pursuit,
               "B3": pursuit + (info["selected_side"] * lateral).unsqueeze(1) * normal,
               "B4": pursuit + (signs * lateral).unsqueeze(1) * normal,
               "B5": reference.unsqueeze(1) * direction}
    yaw = (params["yaw_gain_per_s"] * torch.atan2(rel[:, 1], rel[:, 0])).clamp(
        -yaw_rate_max, yaw_rate_max) / yaw_rate_max
    actions, stages = {}, {}
    for arm in SCRIPTED_ARMS:
        stage = v2._filter_stages(nominal[arm], pts, max_velocity, params)
        action = torch.zeros(n, 4, device=own_pos_w.device, dtype=own_pos_w.dtype)
        action[:, :2] = stage["post_cap_xy"] / max_velocity
        if arm != "B1":
            action[:, 3] = yaw
        actions[arm] = action.clamp(-1.0, 1.0)
        stage["safety_attenuation_mps"] = stage["nominal_norm_mps"] - stage["post_cap_norm_mps"]
        stages[arm] = stage
    if frozen_action is not None:
        if frozen_action.shape != (n, 4) or not bool(torch.isfinite(frozen_action).all()):
            raise ValueError("frozen_action must be a finite [N,4] actor output")
        actions["B0"] = frozen_action.clone()
    info.update(arms=stages, sham_sign=signs, lateral_abs_mps=lateral,
                reference_norm_mps=reference, zero_lateral=lateral == 0,
                decision_index=indices, reference_path_m=reference * DT_S * WINDOW_STEPS,
                reference_squared_speed_integral_m2_s=reference.square() * DT_S * WINDOW_STEPS)
    return actions, info
