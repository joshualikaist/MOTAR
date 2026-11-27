"""07 §§6–7 [ADAPTED]: riskcap mirrors and an unqualified planar response model.

Normal package import of the upstream controller would import the simulator. Only its
pure raw-scan/riskcap arithmetic is mirrored here; isolated CPU tests compare the original.
The moved-pose point corridor and first-order response are explicit planning approximations.
"""
import math
from dataclasses import dataclass

import torch

from .types import Rollout


@dataclass(frozen=True)
class RiskcapConfig:
    """Policy F receipt-bound values, M2 §0; upstream defaults differ (brake=2.0).

    The free-cap decimal is the recorded runtime value, not the rounded 3.5355 shorthand.
    Brake/reaction/margin only affect telemetry under riskcap, never the velocity cap law.
    """
    mode: str = "riskcap"
    fixed_cap_mps: float = 2.0
    free_speed_cap_mps: float = 3.53553390593
    path_half_width_m: float = 0.45
    hard_margin_m: float = 0.45
    slow_distance_m: float = 3.0
    release_distance_m: float = 5.0
    ttc_s: float = 1.2
    brake_mps2: float = 2.96
    reaction_s: float = 0.1
    max_range_m: float = 12.0

    def __post_init__(self):
        if self.mode != "riskcap":
            raise ValueError("BOIA v1 mirrors only the frozen riskcap mode")
        values = (self.fixed_cap_mps, self.free_speed_cap_mps, self.path_half_width_m,
                  self.hard_margin_m, self.slow_distance_m, self.release_distance_m,
                  self.ttc_s, self.brake_mps2, self.reaction_s, self.max_range_m)
        if not all(math.isfinite(x) and x >= 0 for x in values):
            raise ValueError("governor values must be finite and nonnegative")
        if min(self.path_half_width_m, self.brake_mps2, self.max_range_m) <= 0:
            raise ValueError("width, braking acceleration and sensor range must be positive")
        if self.release_distance_m <= self.slow_distance_m or self.free_speed_cap_mps < self.fixed_cap_mps:
            raise ValueError("invalid riskcap release interval or cap ordering")


def directional_lidar_clearance(lidar_m, bearings_rad, command_xy, *, max_range_m,
                                path_half_width_m, target_return_mask=None,
                                vertical_fov_deg=(20.0, -10.0)):
    """Exact raw-scan mirror of speed_governor.py:189–255; no simulator import.

    Inputs: ranges (N,V,B), bearings (B,), vehicle command (N,2); optional caller mask
    is a sensor-derived association, never an internally fetched privileged label.
    """
    assert lidar_m.ndim == 3 and command_xy.shape == (lidar_m.shape[0], 2)
    assert bearings_rad.shape == (lidar_m.shape[2],)
    assert target_return_mask is None or target_return_mask.shape == lidar_m.shape
    max_range = float(max_range_m)
    valid = torch.isfinite(lidar_m) & (lidar_m >= 0.0) & (lidar_m < max_range * 0.995)
    if target_return_mask is not None:
        valid &= ~target_return_mask.bool()
    safe_range = torch.where(valid, lidar_m, torch.full_like(lidar_m, max_range))
    vertical_angles = torch.linspace(math.radians(float(vertical_fov_deg[0])),
                                     math.radians(float(vertical_fov_deg[1])), lidar_m.shape[1],
                                     device=lidar_m.device, dtype=lidar_m.dtype)
    horizontal_range = safe_range * torch.cos(vertical_angles).view(1, -1, 1)
    horizontal_range = torch.where(valid, horizontal_range, torch.full_like(horizontal_range, max_range))
    nearest, ray_valid = horizontal_range.amin(dim=1), valid.any(dim=1)
    requested_speed = command_xy.norm(dim=1)
    command_bearing = torch.atan2(command_xy[:, 1], command_xy[:, 0])
    delta = torch.atan2(torch.sin(bearings_rad.view(1, -1) - command_bearing.view(-1, 1)),
                        torch.cos(bearings_rad.view(1, -1) - command_bearing.view(-1, 1)))
    forward, lateral = nearest * torch.cos(delta), (nearest * torch.sin(delta)).abs()
    width = path_half_width_m.to(lateral.dtype).view(-1, 1) if isinstance(path_half_width_m, torch.Tensor) else float(path_half_width_m)
    in_path = ray_valid & (forward > 0.0) & (lateral <= width)
    clearance = torch.where(in_path, forward, torch.full_like(forward, max_range)).amin(dim=1)
    return torch.where(requested_speed > 1e-6, clearance.clamp(min=0.0, max=max_range),
                        torch.full_like(clearance, max_range))


def apply_riskcap(command_xy, clearance_m, config):
    """Mirror original riskcap branch/scaling/telemetry at speed_governor.py:438–540."""
    assert command_xy.ndim == 2 and command_xy.shape[1] == 2
    assert clearance_m.shape == command_xy.shape[:1] and config.mode == "riskcap"
    requested = command_xy.norm(dim=1)
    usable = (clearance_m - float(config.hard_margin_m)).clamp(min=0.0)
    release_span = float(config.release_distance_m - config.slow_distance_m)
    release = ((clearance_m - float(config.slow_distance_m)) / release_span).clamp(0.0, 1.0)
    cap = float(config.fixed_cap_mps) + release * (float(config.free_speed_cap_mps) - float(config.fixed_cap_mps))
    executed_speed = torch.minimum(requested, cap)
    scale = torch.where(requested > 1e-6, executed_speed / requested.clamp(min=1e-6), torch.ones_like(requested))
    governed = command_xy * scale.unsqueeze(1)

    def stopping_margin(speed):
        return usable - (speed * float(config.reaction_s) + speed.square() / (2.0 * float(config.brake_mps2)))

    ttc_requested = torch.where(requested > 1e-6, usable / requested.clamp(min=1e-6), torch.full_like(requested, float("inf")))
    return governed, {"requested_speed_mps": requested, "executed_speed_mps": executed_speed,
                      "speed_cap_mps": cap, "scale": scale, "clearance_m": clearance_m,
                      "ttc_requested_s": ttc_requested,
                      "stopping_margin_requested_m": stopping_margin(requested),
                      "stopping_margin_executed_m": stopping_margin(executed_speed),
                      "lateral_cap_mps": torch.full_like(cap, float("inf")),
                      "lateral_clearance_m": torch.full_like(cap, float("inf"))}


def point_corridor_clearance(pts_obs, valid_obs, p, psi, command_xy, config):
    """ADAPTED moved-pose corridor: points (N,P,2), pose (N,K,3), command (N,K,2).

    This reprojects the original measured world points. It does not render a new scan,
    discover unseen space, or claim bitwise equivalence with the raw-scan geometry.
    """
    n, k, _ = p.shape
    assert pts_obs.ndim == 3 and pts_obs.shape[0] == n and pts_obs.shape[-1] == 2
    assert valid_obs.shape == pts_obs.shape[:2] and psi.shape == (n, k) and command_xy.shape == (n, k, 2)
    delta = pts_obs[:, None] - p[:, :, None, :2]
    local = torch.stack((delta[..., 0] * psi.cos()[..., None] + delta[..., 1] * psi.sin()[..., None],
                         -delta[..., 0] * psi.sin()[..., None] + delta[..., 1] * psi.cos()[..., None]), -1)
    speed = (command_xy * command_xy).sum(-1).sqrt()
    direction = command_xy / speed.clamp(min=1e-6)[..., None]
    forward = (local * direction[:, :, None]).sum(-1)
    lateral = (local[..., 0] * direction[..., 1, None] - local[..., 1] * direction[..., 0, None]).abs()
    valid = valid_obs[:, None] & torch.isfinite(local).all(-1)
    in_path = valid & (forward > 0) & (lateral <= config.path_half_width_m)
    clearance = torch.where(in_path, forward, torch.full_like(forward, config.max_range_m)).amin(-1) if pts_obs.shape[1] else torch.full_like(speed, config.max_range_m)
    return torch.where(speed > 1e-6, clearance.clamp(0, config.max_range_m), torch.full_like(speed, config.max_range_m))


def rollout(cmd, p, v, psi, pts_obs, valid_obs, gov_cfg, theta_dyn, H, S, dt, *, initial_yaw_rate=None):
    """07 §6 planar first-order surrogate; not a controller/safety qualification.

    cmd (N,K,4) is normalized; p/v (N,3), psi (N,). The returned Rollout holds every
    substep and step end. z is held at its initial value, including when input v_z!=0.
    governed_cmd is physical [m/s, m/s, 0, rad/s], not a normalized actor action.
    Positive tau_psi requires explicit initial yaw rate (N,), not an assumed body-z alias.
    """
    n, k, width = cmd.shape
    assert width == 4 and p.shape == v.shape == (n, 3) and psi.shape == (n,)
    assert isinstance(H, int) and isinstance(S, int) and min(H, S) > 0 and math.isfinite(dt) and dt > 0
    assert all(bool(torch.isfinite(x).all()) for x in (cmd, p, v, psi))
    tau, accel_max = float(theta_dyn.tau_v), float(theta_dyn.a_max)
    if not math.isfinite(tau) or not math.isfinite(accel_max) or min(tau, accel_max) <= 0:
        raise ValueError("response time and acceleration limit must be finite and positive")
    delta = dt / S
    if delta > tau:
        raise ValueError("response discretization must not overshoot a first-order step")
    tau_yaw = theta_dyn.tau_psi
    if tau_yaw is not None:
        if not math.isfinite(tau_yaw) or tau_yaw < delta or initial_yaw_rate is None:
            raise ValueError("positive yaw time constant requires an explicit initial rate and resolved substeps")
        assert initial_yaw_rate.shape == (n,) and bool(torch.isfinite(initial_yaw_rate).all())
    clipped = cmd.clamp(-1, 1)
    requested = 2.5 * clipped[..., :2]
    omega = 3.0 * clipped[..., 3]
    current_p = p[:, None].expand(n, k, 3).clone()
    current_v = v[:, None].expand(n, k, 3).clone()
    current_v[..., 2] = 0
    current_psi = psi[:, None].expand(n, k).clone()
    rate = initial_yaw_rate[:, None].expand(n, k).clone() if tau_yaw is not None else omega
    shape = (n, k, H, S)
    positions, velocities = p.new_empty(shape + (3,)), v.new_empty(shape + (3,))
    yaws, rates = psi.new_empty(shape), psi.new_empty(shape)
    commands = cmd.new_zeros((n, k, H, 4))
    clearances, caps, scales = cmd.new_empty((n, k, H)), cmd.new_empty((n, k, H)), cmd.new_empty((n, k, H))
    accelerations = cmd.new_empty(shape + (2,))
    for j in range(H):
        clearance = point_corridor_clearance(pts_obs, valid_obs, current_p, current_psi, requested, gov_cfg)
        governed, diag = apply_riskcap(requested.reshape(n * k, 2), clearance.reshape(n * k), gov_cfg)
        governed = governed.reshape(n, k, 2)
        commands[:, :, j, :2], commands[:, :, j, 3] = governed, omega
        clearances[:, :, j] = clearance
        caps[:, :, j], scales[:, :, j] = diag["speed_cap_mps"].reshape(n, k), diag["scale"].reshape(n, k)
        for s in range(S):
            target_v = torch.stack((governed[..., 0] * current_psi.cos() - governed[..., 1] * current_psi.sin(),
                                    governed[..., 0] * current_psi.sin() + governed[..., 1] * current_psi.cos()), -1)
            acceleration = (target_v - current_v[..., :2]) / tau
            norm = (acceleration * acceleration).sum(-1, keepdim=True).sqrt()
            acceleration = acceleration * (accel_max / norm.clamp(min=torch.finfo(norm.dtype).tiny)).clamp(max=1)
            current_v = torch.cat((current_v[..., :2] + delta * acceleration, torch.zeros_like(current_v[..., 2:])), -1)
            if tau_yaw is not None:
                rate = rate + delta / tau_yaw * (omega - rate)
            current_psi = current_psi + delta * rate
            current_p = current_p + delta * current_v
            positions[:, :, j, s], velocities[:, :, j, s] = current_p, current_v
            yaws[:, :, j, s], rates[:, :, j, s] = current_psi, rate
            accelerations[:, :, j, s] = acceleration
    diagnostics = {"initial_pos": p[:, None].expand(n, k, 3).clone(),
                   "initial_vel": torch.cat((v[:, :2], torch.zeros_like(v[:, 2:])), -1)[:, None].expand(n, k, 3).clone(),
                   "initial_yaw": psi[:, None].expand(n, k).clone(), "clipped_action": clipped,
                   "clearance_m": clearances, "speed_cap_mps": caps, "scale": scales,
                   "acceleration_xy": accelerations, "effective_yaw_rate": rates}
    return Rollout(positions, velocities, yaws, positions[:, :, :, -1], velocities[:, :, :, -1],
                   yaws[:, :, :, -1], commands, diagnostics)


def braking_distance(speed, table=((2.0, 0.82), (2.5, 1.05), (3.52, 1.57)), *, t_react):
    """Total reaction-plus-stop distance and an above-table warning flag, without fitting data.

    Below the first knot use a=v0²/(2D0). Interpolate table knots exactly; above the last
    use Dlast+(v²-vlast²)/(2a), the same curvature anchored continuously at that last knot.
    The caller must NOT add another reaction-distance term to the returned distance.
    """
    if len(table) < 2 or not math.isfinite(t_react) or t_react < 0:
        raise ValueError("at least two braking knots and a nonnegative reaction time are required")
    if any(not math.isfinite(x) or x <= 0 for row in table for x in row):
        raise ValueError("braking knots must be finite and positive")
    if any(table[i + 1][0] <= table[i][0] or table[i + 1][1] < table[i][1] for i in range(len(table) - 1)):
        raise ValueError("braking speeds must increase and distances must not decrease")
    assert bool(torch.isfinite(speed).all()) and bool((speed >= 0).all())
    first_v, first_d = table[0]
    accel = first_v ** 2 / (2 * first_d)
    component = speed.square() / (2 * accel)
    for (va, da), (vb, db) in zip(table[:-1], table[1:]):
        interpolated = da + (speed - va) / (vb - va) * (db - da)
        component = torch.where((speed >= va) & (speed <= vb), interpolated, component)
    last_v, last_d = table[-1]
    extrapolated = speed > last_v
    component = torch.where(extrapolated, last_d + (speed.square() - last_v ** 2) / (2 * accel), component)
    return speed * t_react + component, extrapolated
