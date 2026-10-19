"""07 §7 [ADAPTED/NEW]: sensed-point swept clearance and empirical stopping screen.

Unknown space always blocks safety, independently of the visibility-ablation setting.
This screen does not guarantee physical collision freedom or calibrated false alarms.
"""
import torch

from .dynamics import braking_distance
from .geometry import seg_point_dist, segment_in_known_free
from .types import FreeContext, Rollout


def _segment_clearance(a, b, points, valid):
    if not points.shape[1]:
        return a.new_full(a.shape[:-1], float("inf"))
    distance = seg_point_dist(a, b, points)
    mask = valid.reshape(valid.shape[0], *([1] * (distance.ndim - 2)), -1)
    return torch.where(mask, distance, torch.full_like(distance, float("inf"))).amin(-1)


def screen(roll, pts_obs, valid_obs, free_ctx, params):
    """Return hard_ok/stop_ok (N,K), clr_min (N,K,H), soft deficit (N,K).

    Every consecutive rollout substep is screened as a straight swept segment, with
    the larger endpoint speed for r_hard+speed*t_det. U4's ``initial_pos`` and
    ``initial_vel`` diagnostics are required for the initial-to-first-substep segment;
    no starting pose or speed is invented.
    clr_min is each decision interval's minimum swept sensed-point clearance.
    clr_soft_cost is the mean deficit in METRES below r_body+m_s (.398 m), diagnostic
    only. It is NOT §9's normalized d_soft=1 m cost, which the cost module computes.
    The endpoint stop uses braking_distance's reaction-inclusive distance ONCE, plus
    exact segment free-set containment and r_hard point clearance. Table extrapolation
    has the same limited validity as U4; the helper's warning remains available there.
    """
    assert isinstance(roll, Rollout) and isinstance(free_ctx, FreeContext)
    assert roll.pos.ndim == 5 and roll.pos.shape[-1] == 3 and roll.vel.shape == roll.pos.shape
    n, k, h, sub, _ = roll.pos.shape
    assert pts_obs.ndim == 3 and pts_obs.shape[0] == n and pts_obs.shape[-1] == 2
    assert valid_obs.shape == pts_obs.shape[:2]
    assert bool(torch.isfinite(roll.pos).all()) and bool(torch.isfinite(roll.vel).all())
    assert bool(torch.isfinite(pts_obs[valid_obs]).all())
    assert roll.diagnostics is not None and "initial_pos" in roll.diagnostics and "initial_vel" in roll.diagnostics
    initial = roll.diagnostics["initial_pos"]
    assert initial.shape == (n, k, 3) and bool(torch.isfinite(initial).all())
    points = roll.pos[..., :2].reshape(n, k, h * sub, 2)
    previous = torch.cat((initial[:, :, None, :2], points[:, :, :-1]), 2)
    speeds = (roll.vel * roll.vel).sum(-1).sqrt().reshape(n, k, h * sub)
    initial_vel = roll.diagnostics["initial_vel"]
    assert initial_vel.shape == (n, k, 3) and bool(torch.isfinite(initial_vel).all())
    initial_speed = (initial_vel ** 2).sum(-1).sqrt()
    previous_speed = torch.cat((initial_speed[:, :, None], speeds[:, :, :-1]), 2)
    threshold = params.r_hard + torch.maximum(speeds, previous_speed) * params.t_det
    start, end = previous.reshape(n, -1, 2), points.reshape(n, -1, 2)
    clearances = roll.pos.new_empty((n, k * h * sub))
    free = torch.empty(clearances.shape, dtype=torch.bool, device=roll.pos.device)
    for offset in range(0, start.shape[1], 256):
        aa, bb = start[:, offset:offset + 256], end[:, offset:offset + 256]
        clearances[:, offset:offset + 256] = _segment_clearance(aa, bb, pts_obs, valid_obs)
        free[:, offset:offset + 256] = segment_in_known_free(aa, bb, free_ctx)
    hard_ok = ((clearances.reshape(n, k, h * sub) >= threshold)
               & free.reshape(n, k, h * sub)).all(-1)
    clr_min = clearances.reshape(n, k, h, sub).amin(-1)
    clr_soft_cost = (params.r_body + params.m_s - clr_min).clamp(min=0).mean(-1)
    final_pos, final_vel = roll.pos[:, :, -1, -1], roll.vel[:, :, -1, -1]
    assert bool((final_vel[..., 2] == 0).all()), "the stopping model has no vertical authority"
    speed = (final_vel[..., :2] ** 2).sum(-1).sqrt()
    distance, _ = braking_distance(speed, t_react=params.t_react)
    direction = final_vel[..., :2] / speed.clamp(min=torch.finfo(speed.dtype).tiny)[..., None]
    stop_end = final_pos[..., :2] + distance[..., None] * direction
    stop_ok = ((_segment_clearance(final_pos[..., :2], stop_end, pts_obs, valid_obs) >= params.r_hard)
               & segment_in_known_free(final_pos[..., :2], stop_end, free_ctx))
    return hard_ok, stop_ok, clr_min, clr_soft_cost
