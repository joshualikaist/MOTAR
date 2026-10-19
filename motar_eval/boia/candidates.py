"""07 §8 [ADAPTED]: twenty deterministic commands, preserving Policy F at index zero.

Arc offsets use h_r3_arms.py's geometry (.25–2 m, gain 1.5/s, cap 1.5 m/s,
LOS .40 m and lateral-path .50 m). Its extra point filter is not imported/applied.
Unlike that historical helper's raw lateral-path scan, both tests use §4's caller-
supplied sensed obstacle set after belief-based exclusion. This adaptation is explicit.
"""
import math

import torch

from .geometry import seg_point_dist
from .types import OnlineInputs


def _clear(a, b, points, valid, clearance):
    if not points.shape[1]:
        return torch.ones(a.shape[:-1], dtype=torch.bool, device=a.device)
    distance = seg_point_dist(a, b, points)
    mask = valid.reshape(valid.shape[0], *([1] * (distance.ndim - 2)), -1)
    return ((distance >= clearance) | ~mask).all(-1)


def generate(a_F, pose, mu, pts_obs, valid_obs, params):
    """Return commands (N,20,4), family IDs (20,), and per-environment diagnostics.

    ``pose`` is OnlineInputs, including explicit tracker ``active`` and own velocity.
    Families: 0 F; 1 brake; 2 lattice; 3 left arc; 4 right arc; 5 yaw-only.
    Lattice order is bearing offsets -40,-20,0,20,40 degrees, each with .6,1.2,2 m/s.
    Inactive/zero-distance beliefs use velocity heading (yaw when stopped), without
    inventing a target range. Their two arc slots remain duplicate 1 m/s pursuits,
    offset zero, arc_found=False. Inactive yaw commands are zero, including brake.
    F must already be finite/in-box: it is rejected rather than silently clipped.
    """
    assert isinstance(pose, OnlineInputs)
    n = a_F.shape[0]
    assert a_F.shape == (n, 4) and mu.shape == (n, 6) and params.K == 20
    assert pose.pos.shape == pose.vel.shape == (n, 3) and pose.yaw.shape == pose.active.shape == (n,)
    assert pts_obs.ndim == 3 and pts_obs.shape[0] == n and pts_obs.shape[-1] == 2
    assert valid_obs.shape == pts_obs.shape[:2]
    if not bool(torch.isfinite(a_F).all()) or not bool((a_F.abs() <= 1).all()):
        raise ValueError("Policy F command must be finite and already inside its action box")
    assert bool(torch.isfinite(pose.pos).all()) and bool(torch.isfinite(pose.vel).all())
    assert bool(torch.isfinite(pose.yaw).all()) and bool(torch.isfinite(pts_obs[valid_obs]).all())
    active = pose.active.bool()
    assert bool(torch.isfinite(mu[active, :3]).all())
    speed = (pose.vel[:, :2] * pose.vel[:, :2]).sum(-1).sqrt()
    forward = torch.stack((pose.yaw.cos(), pose.yaw.sin()), -1)
    moving = speed > 1e-6
    heading = torch.where(moving[:, None], pose.vel[:, :2] / speed.clamp(min=1e-6)[:, None], forward)
    relative = torch.where(active[:, None], mu[:, :2] - pose.pos[:, :2], torch.zeros_like(heading))
    distance = (relative * relative).sum(-1).sqrt()
    direction_valid = active & (distance > 1e-6)
    direction = torch.where(direction_valid[:, None], relative / distance.clamp(min=1e-6)[:, None], heading)
    normal = torch.stack((-direction[:, 1], direction[:, 0]), -1)
    bearing = torch.atan2(direction[:, 1], direction[:, 0])
    yaw_error = torch.atan2((bearing - pose.yaw).sin(), (bearing - pose.yaw).cos())
    yaw_action = torch.where(direction_valid, (2.0 * yaw_error).clamp(-3.0, 3.0) / 3.0,
                             torch.zeros_like(yaw_error))
    world_vel = a_F.new_zeros((n, 20, 2))
    offsets = a_F.new_tensor((-40., -20., 0., 20., 40.)) * (math.pi / 180)
    speeds = a_F.new_tensor((.6, 1.2, 2.0))
    lattice_bearing = bearing[:, None, None] + offsets[None, :, None]
    lattice = torch.stack((lattice_bearing.cos(), lattice_bearing.sin()), -1) * speeds[None, None, :, None]
    world_vel[:, 2:17] = lattice.reshape(n, 15, 2)

    # Index17 is left, index18 right; select the first clear magnitude on each side.
    side = a_F.new_tensor((1.0, -1.0))
    magnitudes = a_F.new_tensor((.25, .5, .75, 1., 1.25, 1.5, 1.75, 2.))
    signed = side[:, None] * magnitudes[None, :]
    lateral_points = pose.pos[:, None, None, :2] + signed[None, :, :, None] * normal[:, None, None]
    goal = torch.where(direction_valid[:, None], mu[:, :2], pose.pos[:, :2])
    goals = goal[:, None, None].expand_as(lateral_points)
    starts = pose.pos[:, None, None, :2].expand_as(lateral_points)
    clear = (_clear(lateral_points, goals, pts_obs, valid_obs, .40)
             & _clear(starts, lateral_points, pts_obs, valid_obs, .50)
             & direction_valid[:, None, None])
    arc_found = clear.any(-1)
    first = clear.to(torch.int64).argmax(-1)
    arc_offsets = torch.where(arc_found, magnitudes[first] * side[None], torch.zeros_like(first, dtype=a_F.dtype))
    lateral_speed = (1.5 * arc_offsets).clamp(-1.5, 1.5)
    world_vel[:, 17:19] = direction[:, None] + lateral_speed[..., None] * normal[:, None]
    world_vel[:, 19] = .6 * heading
    commands = a_F.new_zeros((n, 20, 4))
    commands[..., 0] = (world_vel[..., 0] * pose.yaw.cos()[:, None] + world_vel[..., 1] * pose.yaw.sin()[:, None]) / 2.5
    commands[..., 1] = (-world_vel[..., 0] * pose.yaw.sin()[:, None] + world_vel[..., 1] * pose.yaw.cos()[:, None]) / 2.5
    commands[..., 3] = yaw_action[:, None]
    commands = commands.clamp(-1, 1)
    commands[:, 0] = a_F
    family_id = torch.tensor((0, 1) + (2,) * 15 + (3, 4, 5), dtype=torch.int64, device=a_F.device)
    diagnostics = {"inactive_belief": ~active, "bearing_fallback": ~direction_valid,
                   "direction_valid": direction_valid, "heading_from_velocity": moving,
                   "arc_offsets_m": arc_offsets, "arc_found": arc_found}
    return commands, family_id, diagnostics
