"""Simulator-independent local steering for NavRL's virtual moving target."""

import math

import torch


TARGET_MOTION_MODEL = "symmetric_local_steer_v1"


# Symmetric candidates keep obstacle avoidance from introducing a global left/right bias. The
# per-episode turn_sign only breaks exact +/- ties and is sampled 50:50 by NavRLTask.
TURN_ANGLES_DEG = (0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0, 180.0)


def steer_target_step(
    old_xy,
    desired_velocity,
    speed,
    dt,
    bars_xy,
    lo,
    hi,
    clearance,
    turn_sign,
):
    """Choose the smallest symmetric heading change whose full-speed endpoint is collision-free.

    Returns ``(new_xy, velocity, steered, clear)``. If no candidate is clear, the least-bad
    endpoint is returned with ``clear=False`` so the caller's projection fallback can recover it.
    """
    if old_xy.ndim != 2 or old_xy.shape[1] != 2:
        raise ValueError("old_xy must have shape [N, 2]")
    n = old_xy.shape[0]
    if desired_velocity.shape != old_xy.shape:
        raise ValueError("desired_velocity must match old_xy")
    if speed.shape != (n,) or turn_sign.shape != (n,):
        raise ValueError("speed and turn_sign must have shape [N]")
    if lo.shape != old_xy.shape or hi.shape != old_xy.shape:
        raise ValueError("lo and hi must match old_xy")
    if bars_xy.ndim != 3 or bars_xy.shape[0] != n or bars_xy.shape[2] != 2:
        raise ValueError("bars_xy must have shape [N, B, 2]")

    base_norm = desired_velocity.norm(dim=1, keepdim=True)
    base = desired_velocity / base_norm.clamp(min=1e-6)
    fallback = torch.zeros_like(base)
    fallback[:, 0] = 1.0
    base = torch.where(base_norm > 1e-6, base, fallback)

    angles = torch.tensor(
        [math.radians(value) for value in TURN_ANGLES_DEG],
        device=old_xy.device,
        dtype=old_xy.dtype,
    )
    cosine = torch.cos(angles).view(1, -1)
    sine = torch.sin(angles).view(1, -1)
    bx, by = base[:, 0:1], base[:, 1:2]
    directions = torch.stack(
        (bx * cosine - by * sine, bx * sine + by * cosine), dim=2
    )
    proposals = old_xy.unsqueeze(1) + directions * (
        speed.clamp(min=0.0) * float(dt)
    ).view(n, 1, 1)

    inside = ((proposals >= lo.unsqueeze(1)) & (proposals <= hi.unsqueeze(1))).all(
        dim=2
    )
    if bars_xy.shape[1] > 0 and float(clearance) > 0.0:
        min_bar = torch.cdist(proposals, bars_xy).amin(dim=2)
    else:
        min_bar = torch.full(
            inside.shape, float("inf"), device=old_xy.device, dtype=old_xy.dtype
        )
    clear = inside & (min_bar >= float(clearance) + 1e-4)

    turn_cost = angles.abs().view(1, -1)
    # A clear candidate always wins; among clear candidates prefer the smallest turn. Exact
    # +/- ties use the balanced per-episode sign instead of a fixed global side.
    tie = turn_sign.view(n, 1) * torch.sign(angles).view(1, -1) * 1e-3
    clear_score = 1000.0 - turn_cost + tie
    # If trapped, prefer staying inside the arena and maximizing bar distance. The caller then
    # applies its iterative projection as a final safety net.
    trapped_score = inside.to(old_xy.dtype) * 100.0 + min_bar.clamp(max=10.0)
    trapped_score = trapped_score - 0.01 * turn_cost + tie
    score = torch.where(clear, clear_score, trapped_score)
    chosen = score.argmax(dim=1)
    rows = torch.arange(n, device=old_xy.device)
    selected_xy = proposals[rows, chosen]
    selected_velocity = directions[rows, chosen] * speed.unsqueeze(1)
    selected_clear = clear[rows, chosen]
    return selected_xy, selected_velocity, chosen != 0, selected_clear
