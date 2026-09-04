"""Sensor-only horizontal speed governor for dense NavRL obstacle fields.

The policy already commands a body-frame velocity.  This module never changes its direction or
uses simulator geometry: it only scales the horizontal norm from the current LiDAR scan.  Keeping
the math here independent of Isaac Gym makes the safety contract CPU-testable.
"""

from dataclasses import dataclass
import math

import torch


VALID_SPEED_GOVERNOR_MODES = (
    "off", "fixed", "clearance", "ttc", "riskcap", "stopcap",
    # A4 baselines (prereg pending). Both answer "is the corridor the problem?" by changing
    # only the GEOMETRY of the measurement, keeping the stopping-distance law identical.
    "omni",     # same law, but clearance is the nearest return in ANY bearing
    "dwa_arc",  # same law, but clearance is measured along the arc the vehicle is on
)


def _finite_float(environ, name, default, *, minimum=None, strict_minimum=False):
    raw = str(environ.get(name, "")).strip()
    try:
        value = float(raw) if raw else float(default)
    except ValueError as exc:
        raise ValueError(f"{name} must be numeric; got {raw!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite; got {value!r}")
    if minimum is not None:
        bad = value <= minimum if strict_minimum else value < minimum
        if bad:
            relation = ">" if strict_minimum else ">="
            raise ValueError(f"{name} must be {relation} {minimum}; got {value!r}")
    return value


@dataclass(frozen=True)
class SpeedGovernorConfig:
    mode: str = "off"
    fixed_cap_mps: float = 2.0
    free_speed_cap_mps: float = math.sqrt(2.0) * 2.5
    path_half_width_m: float = 0.45
    hard_margin_m: float = 0.45
    slow_distance_m: float = 3.0
    release_distance_m: float = 5.0
    ttc_s: float = 1.0
    brake_mps2: float = 2.0
    reaction_s: float = 0.1

    @classmethod
    def from_environ(cls, environ):
        mode = str(environ.get("NAVRL_SPEED_GOVERNOR", "off")).strip().lower() or "off"
        if mode not in VALID_SPEED_GOVERNOR_MODES:
            raise ValueError(
                "NAVRL_SPEED_GOVERNOR must be one of %s; got %r"
                % (", ".join(VALID_SPEED_GOVERNOR_MODES), mode)
            )
        result = cls(
            mode=mode,
            fixed_cap_mps=_finite_float(
                environ, "NAVRL_SPEED_GOVERNOR_FIXED_MPS", 2.0, minimum=0.0,
                strict_minimum=True,
            ),
            free_speed_cap_mps=_finite_float(
                environ, "NAVRL_SPEED_GOVERNOR_FREE_MPS", math.sqrt(2.0) * 2.5,
                minimum=0.0, strict_minimum=True,
            ),
            path_half_width_m=_finite_float(
                environ, "NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M", 0.45,
                minimum=0.0, strict_minimum=True,
            ),
            hard_margin_m=_finite_float(
                environ, "NAVRL_SPEED_GOVERNOR_MARGIN_M", 0.45, minimum=0.0,
            ),
            slow_distance_m=_finite_float(
                environ, "NAVRL_SPEED_GOVERNOR_SLOW_M", 3.0, minimum=0.0,
                strict_minimum=True,
            ),
            release_distance_m=_finite_float(
                environ, "NAVRL_SPEED_GOVERNOR_RELEASE_M", 5.0, minimum=0.0,
                strict_minimum=True,
            ),
            ttc_s=_finite_float(
                environ, "NAVRL_SPEED_GOVERNOR_TTC_S", 1.0, minimum=0.0,
                strict_minimum=True,
            ),
            brake_mps2=_finite_float(
                environ, "NAVRL_SPEED_GOVERNOR_BRAKE_MPS2", 2.0, minimum=0.0,
                strict_minimum=True,
            ),
            reaction_s=_finite_float(
                environ, "NAVRL_SPEED_GOVERNOR_REACTION_S", 0.1, minimum=0.0,
            ),
        )
        if result.slow_distance_m <= result.hard_margin_m:
            raise ValueError(
                "NAVRL_SPEED_GOVERNOR_SLOW_M must exceed NAVRL_SPEED_GOVERNOR_MARGIN_M"
            )
        if result.mode == "riskcap":
            if result.release_distance_m <= result.slow_distance_m:
                raise ValueError(
                    "NAVRL_SPEED_GOVERNOR_RELEASE_M must exceed NAVRL_SPEED_GOVERNOR_SLOW_M "
                    "for riskcap"
                )
            if result.free_speed_cap_mps < result.fixed_cap_mps:
                raise ValueError(
                    "NAVRL_SPEED_GOVERNOR_FREE_MPS must be >= NAVRL_SPEED_GOVERNOR_FIXED_MPS "
                    "for riskcap"
                )
        if result.mode == "stopcap":
            if result.brake_mps2 <= 0.0:
                raise ValueError(
                    "NAVRL_SPEED_GOVERNOR_BRAKE_MPS2 must be strictly positive for stopcap"
                )
        return result


def directional_lidar_clearance(
    lidar_m,
    bearings_rad,
    command_xy,
    *,
    max_range_m,
    path_half_width_m,
    target_return_mask=None,
    vertical_fov_deg=(20.0, -10.0),
):
    """Return nearest sensor surface inside the swept horizontal command corridor.

    ``lidar_m`` is ``[N,V,H]`` slant range, ``bearings_rad`` is ``[H]``, and commands are in the
    same vehicle frame. ``vertical_fov_deg`` follows tensor row order (row 0, last row), which is
    +20 to -10 degrees for the NavRL Warp LiDAR. Target returns are removed because the target is a
    capture object, not a collision obstacle. No-return rays remain no-return after projection.
    """

    if lidar_m.ndim != 3:
        raise ValueError("lidar_m must be [batch, vertical_beams, horizontal_beams]")
    if command_xy.ndim != 2 or command_xy.shape[0] != lidar_m.shape[0] or command_xy.shape[1] != 2:
        raise ValueError("command_xy must be [batch, 2]")
    if bearings_rad.ndim != 1 or bearings_rad.shape[0] != lidar_m.shape[2]:
        raise ValueError("bearings_rad must match the horizontal beam dimension")
    if target_return_mask is not None and target_return_mask.shape != lidar_m.shape:
        raise ValueError("target_return_mask must match lidar_m")

    max_range = float(max_range_m)
    finite = torch.isfinite(lidar_m)
    valid = finite & (lidar_m >= 0.0) & (lidar_m < max_range * 0.995)
    if target_return_mask is not None:
        valid &= ~target_return_mask.bool()
    safe_range = torch.where(valid, lidar_m, torch.full_like(lidar_m, max_range))

    vertical_angles = torch.linspace(
        math.radians(float(vertical_fov_deg[0])),
        math.radians(float(vertical_fov_deg[1])),
        lidar_m.shape[1],
        device=lidar_m.device,
        dtype=lidar_m.dtype,
    )
    horizontal_range = safe_range * torch.cos(vertical_angles).view(1, -1, 1)
    horizontal_range = torch.where(valid, horizontal_range, torch.full_like(horizontal_range, max_range))
    nearest = horizontal_range.amin(dim=1)
    ray_valid = valid.any(dim=1)

    requested_speed = command_xy.norm(dim=1)
    command_bearing = torch.atan2(command_xy[:, 1], command_xy[:, 0])
    delta = torch.atan2(
        torch.sin(bearings_rad.view(1, -1) - command_bearing.view(-1, 1)),
        torch.cos(bearings_rad.view(1, -1) - command_bearing.view(-1, 1)),
    )
    forward = nearest * torch.cos(delta)
    lateral = (nearest * torch.sin(delta)).abs()
    in_path = (
        ray_valid
        & (forward > 0.0)
        & (lateral <= float(path_half_width_m))
    )
    clearance = torch.where(
        in_path, forward, torch.full_like(forward, max_range)
    ).amin(dim=1)
    return torch.where(
        requested_speed > 1e-6,
        clearance.clamp(min=0.0, max=max_range),
        torch.full_like(clearance, max_range),
    )


def _horizontal_nearest(lidar_m, valid, max_range_m, vertical_fov_deg=(20.0, -10.0)):
    """Per-bearing nearest range projected onto the horizontal plane.

    Shared with directional_lidar_clearance so that the A4 arms differ from the corridor in
    geometry ONLY. Forgetting this projection makes an arm look 6% further-sighted than the
    corridor at the +20 degree row, which would confound the comparison it exists to make.
    """
    vertical_angles = torch.linspace(
        math.radians(float(vertical_fov_deg[0])),
        math.radians(float(vertical_fov_deg[1])),
        lidar_m.shape[1],
        device=lidar_m.device,
        dtype=lidar_m.dtype,
    )
    safe = torch.where(valid, lidar_m, torch.full_like(lidar_m, float(max_range_m)))
    horiz = safe * torch.cos(vertical_angles).view(1, -1, 1)
    horiz = torch.where(valid, horiz, torch.full_like(horiz, float(max_range_m)))
    return horiz.amin(dim=1)


def omnidirectional_clearance(lidar_m, *, max_range_m, target_return_mask=None):
    """Nearest sensor surface in ANY bearing -- the corridor removed, nothing else changed.

    This is the crude answer the contact forensics invites: 57-58% of contacts were lateral to a
    0.45 m half-width corridor, so what happens if the corridor is simply dropped? It is the
    baseline a more careful geometric test has to beat, and the S1/Q1 history says that is not a
    formality -- riskcap did not beat a constant cap.
    """
    finite = torch.isfinite(lidar_m)
    valid = finite & (lidar_m >= 0.0) & (lidar_m < float(max_range_m) * 0.995)
    if target_return_mask is not None:
        valid &= ~target_return_mask.bool()
    if lidar_m.ndim != 3:
        raise ValueError("lidar_m must be [batch, vertical_beams, horizontal_beams]")
    return _horizontal_nearest(lidar_m, valid, max_range_m).amin(dim=1)


def arc_clearance(
    lidar_m,
    bearings_rad,
    command_xy,
    yaw_rate,
    *,
    max_range_m,
    path_half_width_m,
    target_return_mask=None,
):
    """Distance to the nearest obstacle along the CIRCULAR ARC the vehicle is turning onto.

    Fox, Burgard & Thrun (IEEE RAM 1997) Eq. 14 measures dist(v, omega) on the arc, not down a
    straight corridor, and their section 2 argues the straight-line decomposition is only
    justifiable "if infinite forces can be asserted on the robot". This arm quantifies that
    objection on our own data: same stopping law, same scan, arc instead of a line.

    For speed v and yaw rate w the instantaneous arc has radius R = v / w. A ray at bearing
    `delta` off the command intersects that arc where the chord subtends the same angle, giving
    an along-arc distance of R * 2 * delta for the ray that touches it. We keep the corridor's
    half-width as the tube radius about the arc so that only the geometry changes.
    """
    if lidar_m.ndim != 3:
        raise ValueError("lidar_m must be [batch, vertical_beams, horizontal_beams]")
    max_range = float(max_range_m)
    finite = torch.isfinite(lidar_m)
    valid = finite & (lidar_m >= 0.0) & (lidar_m < max_range * 0.995)
    if target_return_mask is not None:
        valid &= ~target_return_mask.bool()
    nearest = _horizontal_nearest(lidar_m, valid, max_range)
    ray_valid = valid.any(dim=1)

    speed = command_xy.norm(dim=1)
    command_bearing = torch.atan2(command_xy[:, 1], command_xy[:, 0])
    delta = torch.atan2(
        torch.sin(bearings_rad.view(1, -1) - command_bearing.view(-1, 1)),
        torch.cos(bearings_rad.view(1, -1) - command_bearing.view(-1, 1)),
    )
    # Signed turn radius; a near-zero yaw rate degenerates to the straight corridor by construction.
    w = yaw_rate.view(-1, 1)
    straight = w.abs() < 1e-3
    radius = torch.where(straight, torch.full_like(w, 1e6), speed.view(-1, 1) / w)

    # Perpendicular offset of the ray endpoint from the arc, and the along-arc travel to reach it.
    px = nearest * torch.cos(delta)
    py = nearest * torch.sin(delta)
    perp = (torch.sqrt(px.square() + (py - radius).square()) - radius.abs()).abs()
    along = torch.where(
        straight, px, (radius.abs() * (2.0 * delta.abs())).clamp(max=max_range)
    )
    on_arc = ray_valid & (along > 0.0) & (perp <= float(path_half_width_m))
    clearance = torch.where(on_arc, along, torch.full_like(along, max_range)).amin(dim=1)
    return torch.where(
        speed > 1e-6, clearance.clamp(0.0, max_range), torch.full_like(clearance, max_range)
    )


def apply_speed_governor(command_xy, clearance_m, config):
    """Scale horizontal velocity and return tensors required for causal diagnostics."""

    if command_xy.ndim != 2 or command_xy.shape[1] != 2:
        raise ValueError("command_xy must be [batch, 2]")
    if clearance_m.ndim != 1 or clearance_m.shape[0] != command_xy.shape[0]:
        raise ValueError("clearance_m must be [batch]")
    requested = command_xy.norm(dim=1)
    usable = (clearance_m - float(config.hard_margin_m)).clamp(min=0.0)

    if config.mode == "off":
        cap = requested
    elif config.mode == "fixed":
        cap = torch.full_like(requested, float(config.fixed_cap_mps))
    elif config.mode == "clearance":
        span = float(config.slow_distance_m - config.hard_margin_m)
        cap = float(config.free_speed_cap_mps) * (usable / span).clamp(0.0, 1.0)
    elif config.mode == "ttc":
        cap = (usable / float(config.ttc_s)).clamp(
            min=0.0, max=float(config.free_speed_cap_mps)
        )
    elif config.mode == "riskcap":
        # Minimum-intervention filter: preserve the fixed-2.0 positive control in clutter, but
        # release it smoothly in genuinely open command directions. Unlike clearance/TTC this
        # never creates a forced zero-speed deadlock; a policy request below the cap is untouched.
        release_span = float(config.release_distance_m - config.slow_distance_m)
        release = (
            (clearance_m - float(config.slow_distance_m)) / release_span
        ).clamp(0.0, 1.0)
        cap = float(config.fixed_cap_mps) + release * (
            float(config.free_speed_cap_mps) - float(config.fixed_cap_mps)
        )
    elif config.mode in ("stopcap", "omni", "dwa_arc"):
        # Stopping-distance admissible cap (DWA admissibility; RSS longitudinal rule with
        # a_accel=0 and a static obstacle): the largest v with
        #   usable >= v*reaction + v^2/(2*brake),
        # solved for v. Unlike riskcap there is no floor -- the cap reaches zero exactly at
        # usable=0, so stopping_margin_executed >= 0 holds by construction (up to inter-step
        # clearance change). hard_margin_m is live again in this mode via `usable`.
        brake = float(config.brake_mps2)
        reaction_reach = brake * float(config.reaction_s)
        cap = (
            torch.sqrt(reaction_reach * reaction_reach + 2.0 * brake * usable)
            - reaction_reach
        ).clamp(min=0.0, max=float(config.free_speed_cap_mps))
    else:  # Config construction is fail-closed, but keep direct callers safe.
        raise ValueError(f"unsupported speed governor mode: {config.mode!r}")

    executed_speed = torch.minimum(requested, cap)
    scale = torch.where(
        requested > 1e-6,
        executed_speed / requested.clamp(min=1e-6),
        torch.ones_like(requested),
    )
    governed = command_xy * scale.unsqueeze(1)

    def stopping_margin(speed):
        stopping_distance = (
            speed * float(config.reaction_s)
            + speed.square() / (2.0 * float(config.brake_mps2))
        )
        return usable - stopping_distance

    ttc_requested = torch.where(
        requested > 1e-6,
        usable / requested.clamp(min=1e-6),
        torch.full_like(requested, float("inf")),
    )
    return governed, {
        "requested_speed_mps": requested,
        "executed_speed_mps": executed_speed,
        "speed_cap_mps": cap,
        "scale": scale,
        "clearance_m": clearance_m,
        "ttc_requested_s": ttc_requested,
        "stopping_margin_requested_m": stopping_margin(requested),
        "stopping_margin_executed_m": stopping_margin(executed_speed),
    }
