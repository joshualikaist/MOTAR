"""Simulator-independent local steering for NavRL's virtual moving target."""

import math

import torch


TARGET_MOTION_MODEL = "symmetric_local_steer_v2_heading_continuity90"


# Symmetric candidates keep obstacle avoidance from introducing a global left/right bias. The
# per-episode turn_sign only breaks exact +/- ties and is sampled 50:50 by NavRLTask.
TURN_ANGLES_DEG = (0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0, 180.0)
HEADING_CONTINUITY_RAD = math.radians(90.0)
CV_INITIAL_HEADING_MODES = (
    "random",
    "toward",
    "tangent_left",
    "tangent_right",
    "away",
)


def initial_cv_velocity(mode, speed, target_xy, pursuer_xy, random_angle):
    """Return a CV velocity under an evaluation-only radial-heading intervention.

    ``left`` and ``right`` are defined looking from the pursuer toward the target: left is a
    +90-degree rotation of the pursuer->target radial vector.  The caller always samples and
    supplies ``random_angle`` even for controlled cells so subsequent RNG draws stay aligned.
    """
    if mode not in CV_INITIAL_HEADING_MODES:
        raise ValueError(
            "unknown CV initial heading %r (expected %s)"
            % (mode, "|".join(CV_INITIAL_HEADING_MODES))
        )
    if target_xy.ndim != 2 or target_xy.shape[1] != 2:
        raise ValueError("target_xy must have shape [N, 2]")
    if pursuer_xy.shape != target_xy.shape:
        raise ValueError("pursuer_xy must match target_xy")
    n = target_xy.shape[0]
    if speed.shape != (n,) or random_angle.shape != (n,):
        raise ValueError("speed and random_angle must have shape [N]")

    radial = target_xy - pursuer_xy
    radial_norm = radial.norm(dim=1, keepdim=True)
    fallback = torch.zeros_like(radial)
    fallback[:, 0] = 1.0
    away = torch.where(radial_norm > 1e-6, radial / radial_norm.clamp(min=1e-6), fallback)
    if mode == "random":
        direction = torch.stack((torch.cos(random_angle), torch.sin(random_angle)), dim=1)
    elif mode == "away":
        direction = away
    elif mode == "toward":
        direction = -away
    elif mode == "tangent_left":
        direction = torch.stack((-away[:, 1], away[:, 0]), dim=1)
    else:  # tangent_right
        direction = torch.stack((away[:, 1], -away[:, 0]), dim=1)
    return direction * speed.unsqueeze(1)


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
    previous_heading=None,
):
    """Choose a collision-free full-speed step with optional heading continuity.

    Returns ``(new_xy, velocity, steered, clear)``. If no candidate is clear, the least-bad
    endpoint is returned with ``clear=False`` so the caller's projection fallback can recover it.
    When ``previous_heading`` is provided, clear candidates within 90 degrees of the last flown
    heading are preferred. This is not a veto: if that window contains no clear candidate, the
    ordinary smallest-turn clear candidate still wins.
    """
    if old_xy.ndim != 2 or old_xy.shape[1] != 2:
        raise ValueError("old_xy must have shape [N, 2]")
    n = old_xy.shape[0]
    if desired_velocity.shape != old_xy.shape:
        raise ValueError("desired_velocity must match old_xy")
    if speed.shape != (n,) or turn_sign.shape != (n,):
        raise ValueError("speed and turn_sign must have shape [N]")
    if previous_heading is not None and previous_heading.shape != (n,):
        raise ValueError("previous_heading must have shape [N]")
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
    if previous_heading is None:
        score = torch.where(clear, clear_score, trapped_score)
    else:
        heading = torch.atan2(directions[:, :, 1], directions[:, :, 0])
        delta = torch.atan2(
            torch.sin(heading - previous_heading.view(n, 1)),
            torch.cos(heading - previous_heading.view(n, 1)),
        ).abs()
        in_window_clear = clear & (delta <= HEADING_CONTINUITY_RAD + 1e-9)
        has_window_clear = in_window_clear.any(dim=1, keepdim=True)
        eligible_clear = torch.where(has_window_clear, in_window_clear, clear)
        has_any_clear = clear.any(dim=1, keepdim=True)
        neg_inf = torch.full_like(clear_score, float("-inf"))
        score = torch.where(
            has_any_clear,
            torch.where(eligible_clear, clear_score, neg_inf),
            trapped_score,
        )
    chosen = score.argmax(dim=1)
    rows = torch.arange(n, device=old_xy.device)
    selected_xy = proposals[rows, chosen]
    selected_velocity = directions[rows, chosen] * speed.unsqueeze(1)
    selected_clear = clear[rows, chosen]
    return selected_xy, selected_velocity, chosen != 0, selected_clear
