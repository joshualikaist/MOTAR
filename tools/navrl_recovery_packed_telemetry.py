#!/usr/bin/env python3
"""Evaluation-only packed telemetry for the two-envelope physical-target gate.

The observer is deliberately outside the task.  It wraps the already installed physics callback
and samples tensors on the device; policy observations, rewards, actions, resets and target state
are never written.  One compact NPZ is transferred at the end of each cell.
"""

from __future__ import annotations

import hashlib
import importlib
import json
import math
import os
from pathlib import Path
from typing import Dict, Mapping

import numpy as np
import torch


SCHEMA = "navrl_target_recovery_packed_telemetry_v1"
STATE_OFF = -1
STATE_NORMAL = 0
STATE_BRAKE = 1
STATE_CONNECT = 2
STATE_ROUTE = 3
STATE_NO_CONNECTOR = 4
STATUS_TIMEOUT = 22
HARD_EPSILON_M = 1e-4


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_npz(path: Path, arrays: Mapping[str, np.ndarray]) -> None:
    path = path.resolve()
    if path.exists():
        raise RuntimeError("refusing to overwrite telemetry: %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(".%s.tmp.%d" % (path.name, os.getpid()))
    if temporary.exists():
        raise RuntimeError("stale telemetry temporary exists: %s" % temporary)
    try:
        with temporary.open("xb") as stream:
            np.savez_compressed(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()


def _signed_aabb_margin(position_xy, bars_xy, half_xy, lo_xy, hi_xy):
    """Signed exact-AABB margin; positive is free and touching is zero."""
    wall = torch.minimum(position_xy - lo_xy, hi_xy - position_xy).amin(dim=1)
    if bars_xy.shape[1] == 0:
        obstacle = torch.full_like(wall, float("inf"))
    else:
        delta = (position_xy.unsqueeze(1) - bars_xy).abs() - half_xy
        inside = (delta <= 0.0).all(dim=2)
        outside = delta.clamp(min=0.0).norm(dim=2)
        inside_depth = delta.amax(dim=2)
        obstacle = torch.where(inside, inside_depth, outside).amin(dim=1)
    return torch.minimum(wall, obstacle)


def _geometry_valid(position_xy, bars_xy, half_xy, lo_xy, hi_xy):
    valid = torch.isfinite(position_xy).all(dim=1)
    valid &= torch.isfinite(lo_xy).all(dim=1) & torch.isfinite(hi_xy).all(dim=1)
    valid &= (hi_xy > lo_xy).all(dim=1)
    if bars_xy.shape[1]:
        valid &= torch.isfinite(bars_xy).all(dim=(1, 2))
        valid &= torch.isfinite(half_xy).all(dim=(1, 2))
        valid &= (half_xy >= 0.0).all(dim=(1, 2))
    return valid


def _segments_hit_closed_aabb(p0, p1, bars, half):
    """Vectorized [N,C] continuous segment/AABB intersection."""
    if bars.shape[1] == 0:
        return torch.zeros(p0.shape[:2], dtype=torch.bool, device=p0.device)
    start = p0.unsqueeze(2)
    direction = (p1 - p0).unsqueeze(2)
    box_lo = bars.unsqueeze(1) - half.unsqueeze(1) - HARD_EPSILON_M
    box_hi = bars.unsqueeze(1) + half.unsqueeze(1) + HARD_EPSILON_M
    parallel = direction.abs() <= 1e-9
    parallel_inside = (~parallel) | ((start >= box_lo) & (start <= box_hi))
    safe_direction = torch.where(parallel, torch.ones_like(direction), direction)
    t0 = (box_lo - start) / safe_direction
    t1 = (box_hi - start) / safe_direction
    axis_enter = torch.where(parallel, torch.full_like(t0, float("-inf")), torch.minimum(t0, t1))
    axis_exit = torch.where(parallel, torch.full_like(t0, float("inf")), torch.maximum(t0, t1))
    enter = axis_enter.amax(dim=3)
    leave = axis_exit.amin(dim=3)
    hit = parallel_inside.all(dim=3) & (enter <= leave) & (leave >= 0.0) & (enter <= 1.0)
    return hit.any(dim=2)


def _candidate_metrics(call_args, call_kwargs, selected_velocity):
    """Recompute intended continuous CONNECT safety without changing the selected action."""
    from aerial_gym.task.navrl_task.target_motion import (
        BOUNDED_TURN_ANGLES_DEG,
        limit_planar_velocity,
    )

    names = (
        "old_xy", "current_velocity", "desired_velocity", "speed_limit", "dt", "bars_xy",
        "lo", "hi", "clearance", "turn_sign", "max_accel", "max_turn_rate", "lookahead_s",
        "bars_half_extents_xy", "exact_aabb_clearance",
    )
    values = {name: value for name, value in zip(names, call_args)}
    values.update(call_kwargs)
    old_xy = values["old_xy"]
    current = values["current_velocity"]
    desired = values["desired_velocity"]
    speed_limit = values["speed_limit"]
    dt = float(values["dt"])
    bars = values["bars_xy"]
    lo = values["lo"] + HARD_EPSILON_M
    hi = values["hi"] - HARD_EPSILON_M
    half = values.get("bars_half_extents_xy")
    max_accel = values["max_accel"]
    max_turn = values["max_turn_rate"]
    lookahead = float(values["lookahead_s"])
    if half is None:
        raise RuntimeError("exact CONNECT telemetry requires AABB half extents")
    n = old_xy.shape[0]
    norm = desired.norm(dim=1, keepdim=True)
    fallback = torch.zeros_like(desired)
    fallback[:, 0] = 1.0
    base = torch.where(norm > 1e-6, desired / norm.clamp(min=1e-6), fallback)
    angles = torch.tensor(
        [math.radians(value) for value in BOUNDED_TURN_ANGLES_DEG],
        device=old_xy.device, dtype=old_xy.dtype,
    )
    cosine, sine = torch.cos(angles).view(1, -1), torch.sin(angles).view(1, -1)
    bx, by = base[:, 0:1], base[:, 1:2]
    direction = torch.stack((bx * cosine - by * sine, bx * sine + by * cosine), dim=2)
    scales = torch.tensor((1.0, 0.5, 0.25), device=old_xy.device, dtype=old_xy.dtype)
    candidates = (
        direction.unsqueeze(1).expand(-1, len(scales), -1, -1)
        * scales.view(1, -1, 1, 1)
        * speed_limit.clamp(min=0.0).view(n, 1, 1, 1)
    ).reshape(n, -1, 2)
    candidates = torch.cat((candidates, torch.zeros((n, 1, 2), device=old_xy.device)), dim=1)
    count = candidates.shape[1]
    pos = old_xy.unsqueeze(1).expand(-1, count, -1).clone()
    vel = current.unsqueeze(1).expand(-1, count, -1).clone()
    alive = torch.ones((n, count), dtype=torch.bool, device=old_xy.device)
    prefix = torch.zeros((n, count), dtype=torch.int16, device=old_xy.device)
    first_velocity = None
    steps = max(1, int(math.ceil(lookahead / dt)))
    flat_speed = speed_limit[:, None].expand(-1, count).reshape(-1)
    flat_accel = max_accel[:, None].expand(-1, count).reshape(-1)
    flat_turn = max_turn[:, None].expand(-1, count).reshape(-1)
    flat_desired = candidates.reshape(-1, 2)
    for step in range(steps):
        prior = pos
        vel = limit_planar_velocity(
            vel.reshape(-1, 2), flat_desired, flat_speed, dt, flat_accel, flat_turn
        ).reshape(n, count, 2)
        pos = prior + vel * dt
        if step == 0:
            first_velocity = vel.clone()
        endpoint_safe = ((pos > lo[:, None]) & (pos < hi[:, None])).all(dim=2)
        segment_safe = ~_segments_hit_closed_aabb(prior, pos, bars, half)
        alive &= endpoint_safe & segment_safe
        prefix += alive.to(torch.int16)
    difference = ((first_velocity - selected_velocity[:, None]) ** 2).sum(dim=2)
    chosen = difference.argmin(dim=1)
    rows = torch.arange(n, device=old_xy.device)
    if bool((difference[rows, chosen] > 1e-8).any()):
        raise RuntimeError("cannot bind selected CONNECT command to candidate set")
    return {
        "old_xy": old_xy.detach().clone(),
        "count": count,
        "horizon": steps,
        "safe_prefix": prefix[rows, chosen],
        "full_horizon_safe": alive[rows, chosen],
    }


class _SubstepProxy:
    """Transparent task physics callback that records after the real controller callback."""

    def __init__(self, observer, controller):
        self.observer = observer
        self.controller = controller

    def __call__(self):
        return self.controller()

    def post_physics_step(self):
        self.controller.post_physics_step()
        self.observer.record_substep()

    def __getattr__(self, name):
        return getattr(self.controller, name)


class RecoveryPackedObserver:
    """Fixed-shape device-side observer for one density/speed/route cell."""

    def __init__(self, task, steps: int, physics_substeps: int):
        self.task = task
        self.ctrl = task._target_controller
        self.device = task.device
        self.steps = int(steps)
        self.physics_substeps = int(physics_substeps)
        self.envs = int(task.num_envs)
        if self.steps <= 0 or self.physics_substeps <= 0 or self.envs <= 0:
            raise ValueError("observer dimensions must be positive")
        self.route_enabled = bool(getattr(task, "_target_route_recovery_enabled", False))
        self.interval = -1
        self.substep = 0
        self._in_advance = False
        self._reset_calls_in_advance = torch.zeros(
            (self.steps,), dtype=torch.int32, device=self.device
        )
        self._install_reset_spies()
        self.original_callback = task.sim_env.physics_step_callback
        if self.original_callback is not self.ctrl:
            raise RuntimeError("physical target is not the installed physics callback")
        task.sim_env.set_physics_step_callback(_SubstepProxy(self, self.ctrl))

        shape = (self.steps, self.envs)
        subshape = (self.steps, self.physics_substeps, self.envs)
        self.i16 = {
            name: torch.full(shape, -1, dtype=torch.int16, device=self.device)
            for name in (
                "state_before", "state_after", "age_before", "age_after", "status_after",
                "anchor_cell_i", "anchor_cell_j", "candidate_count", "candidate_horizon_steps",
                "candidate_safe_prefix_steps", "candidate_full_horizon_safe",
            )
        }
        self.i32 = {
            name: torch.zeros(shape, dtype=torch.int32, device=self.device)
            for name in (
                "entry_delta", "resume_delta", "no_connector_delta", "hard_breach_delta",
                "timeout_event", "direct_position_write", "reset_call_during_advance",
                "runner_reset_after_interval",
            )
        }
        self.f32 = {
            name: torch.full(shape, float("nan"), dtype=torch.float32, device=self.device)
            for name in (
                "hard_margin_before_m", "soft_margin_before_m", "hard_margin_after_m",
                "soft_margin_after_m", "speed_before_mps", "speed_after_mps",
                "stop_distance_m", "stop_margin_m", "anchor_distance_m",
                "connector_clearance_m",
            )
        }
        self.xy = {
            name: torch.full(shape + (2,), float("nan"), dtype=torch.float32, device=self.device)
            for name in ("position_before_xy", "position_after_xy", "velocity_before_xy",
                         "velocity_after_xy", "command_xy", "anchor_xy")
        }
        self.sub_f32 = {
            name: torch.full(subshape, float("nan"), dtype=torch.float32, device=self.device)
            for name in (
                "hard_margin_m", "support_xy_m", "contact_force_n", "velocity_error_mps",
                "tilt_deg",
            )
        }
        self.sub_i8 = {
            name: torch.zeros(subshape, dtype=torch.int8, device=self.device)
            for name in ("geometry_valid", "obb_valid", "motor_saturated", "watchdog_breach")
        }
        self._before_position = None
        self._before_counters = None
        self._exact_calls = []
        self._install_candidate_probe()

    def _install_candidate_probe(self):
        self._task_module = importlib.import_module(self.task.__class__.__module__)
        self._original_bounded_step = self._task_module.bounded_drone_target_step

        def observed(*args, **kwargs):
            result = self._original_bounded_step(*args, **kwargs)
            exact = bool(kwargs.get("exact_aabb_clearance", False))
            if not exact and len(args) >= 15:
                exact = bool(args[14])
            if exact:
                self._exact_calls.append(_candidate_metrics(args, kwargs, result[1]))
            return result

        self._task_module.bounded_drone_target_step = observed

    def _install_reset_spies(self):
        self._task_reset_idx = self.task.reset_idx
        self._sim_reset_idx = self.task.sim_env.reset_idx

        def task_reset_idx(*args, **kwargs):
            if self._in_advance and self.interval >= 0:
                self._reset_calls_in_advance[self.interval] += 1
            return self._task_reset_idx(*args, **kwargs)

        def sim_reset_idx(*args, **kwargs):
            if self._in_advance and self.interval >= 0:
                self._reset_calls_in_advance[self.interval] += 1
            return self._sim_reset_idx(*args, **kwargs)

        self.task.reset_idx = task_reset_idx
        self.task.sim_env.reset_idx = sim_reset_idx

    def close(self):
        self.task.sim_env.set_physics_step_callback(self.original_callback)
        self.task.reset_idx = self._task_reset_idx
        self.task.sim_env.reset_idx = self._sim_reset_idx
        self._task_module.bounded_drone_target_step = self._original_bounded_step

    def _state(self):
        if not self.route_enabled or self.task._target_route_manager is None:
            return torch.full((self.envs,), STATE_OFF, dtype=torch.int16, device=self.device)
        return self.task._target_route_manager.recovery_state.to(torch.int16)

    def _age(self):
        if not self.route_enabled:
            return torch.full((self.envs,), -1, dtype=torch.int16, device=self.device)
        return self.task._target_route_manager.recovery_age_steps.to(torch.int16)

    def _status(self):
        if self.task._target_route_manager is None:
            return torch.full((self.envs,), -1, dtype=torch.int16, device=self.device)
        return self.task._target_route_manager.status_code.to(torch.int16)

    def _counters(self):
        if not self.route_enabled:
            return (0, 0, 0, 0)
        manager = self.task._target_route_manager
        return tuple(
            int(value.item())
            for value in (
                manager.recovery_entries, manager.recovery_route_resumes,
                manager.recovery_no_connector_count, manager.recovery_hard_breach_count,
            )
        )

    def _geometry(self, position_xy):
        if not self.route_enabled:
            nan = torch.full((self.envs,), float("nan"), device=self.device)
            return nan, nan
        bars = self.task.obs_dict["obstacle_position"][
            :, self.task._bar_offset:self.task._bar_offset + self.task.n_bars_active, :2
        ]
        half = self.task.obs_dict["asset_collision_half_extents"][
            :, self.task._bar_offset:self.task._bar_offset + self.task.n_bars_active, :2
        ]
        support = self.task._target_route_support_xy
        hard_free, soft_free, soft_margin, hard_margin, _, _, _ = (
            self.task._route_recovery_geometry(position_xy, bars, half, support)
        )
        # Preserve the signed values; booleans are independently reconstructable from <=0.
        del hard_free, soft_free
        return hard_margin, soft_margin

    def begin_interval(self, interval: int):
        if interval != self.interval + 1 or not 0 <= interval < self.steps:
            raise RuntimeError("non-monotone observer interval")
        self.interval = interval
        self.substep = 0
        pos = self.task.target_position[:, :2].detach()
        vel = self.task.target_vel_w[:, :2].detach()
        hard, soft = self._geometry(pos)
        self.i16["state_before"][interval] = self._state()
        self.i16["age_before"][interval] = self._age()
        self.f32["hard_margin_before_m"][interval] = hard
        self.f32["soft_margin_before_m"][interval] = soft
        self.f32["speed_before_mps"][interval] = vel.norm(dim=1)
        self.xy["position_before_xy"][interval] = pos
        self.xy["velocity_before_xy"][interval] = vel
        self._before_position = pos.clone()
        self._before_counters = self._counters()
        self.ctrl.begin_control_interval()

    def advance_target(self):
        if self.interval < 0 or self._before_position is None:
            raise RuntimeError("begin_interval must precede advance_target")
        self._in_advance = True
        self._exact_calls = []
        try:
            self.task._advance_target()
        finally:
            self._in_advance = False
        row = self.interval
        after_direct = self.task.target_position[:, :2].detach()
        changed = (after_direct != self._before_position).any(dim=1)
        self.i32["direct_position_write"][row] = changed.to(torch.int32)
        self.i32["reset_call_during_advance"][row] = self._reset_calls_in_advance[row]
        self.i16["state_after"][row] = self._state()
        self.i16["age_after"][row] = self._age()
        self.i16["status_after"][row] = self._status()
        after = self._counters()
        for name, before, final in zip(
            ("entry_delta", "resume_delta", "no_connector_delta", "hard_breach_delta"),
            self._before_counters, after,
        ):
            delta = final - before
            if delta < 0:
                raise RuntimeError("recovery counter decreased: %s" % name)
            # Scalar event counts are retained in env slot zero; the summary denominator is explicit.
            self.i32[name][row, 0] = delta
        self.i32["timeout_event"][row] = (
            self.i16["status_after"][row] == STATUS_TIMEOUT
        ).to(torch.int32)
        self.xy["command_xy"][row] = self.ctrl.velocity_command[:, :2]

        if self.route_enabled:
            manager = self.task._target_route_manager
            anchor = manager.recovery_anchor.detach()
            self.xy["anchor_xy"][row] = anchor
            distance = (anchor - self._before_position).norm(dim=1)
            active = self.i16["state_after"][row] == STATE_CONNECT
            self.f32["anchor_distance_m"][row] = torch.where(
                active, distance, torch.full_like(distance, float("nan"))
            )
            lo = self.task.obs_dict["env_bounds_min"][:, :2]
            resolution = float(self.task.tm.route_resolution_m)
            cell = torch.floor((anchor - lo) / resolution).to(torch.int16)
            self.i16["anchor_cell_i"][row] = torch.where(
                active, cell[:, 0], torch.full_like(cell[:, 0], -1)
            )
            self.i16["anchor_cell_j"][row] = torch.where(
                active, cell[:, 1], torch.full_like(cell[:, 1], -1)
            )
            # The local candidate set is frozen: 24 headings x 3 speed scales plus stop.
            self.i16["candidate_count"][row] = torch.where(
                active, torch.full_like(cell[:, 0], 73), torch.full_like(cell[:, 0], -1)
            )
            horizon = int(math.ceil(float(self.task.tm.avoidance_lookahead_s) / self.task.step_dt))
            self.i16["candidate_horizon_steps"][row] = torch.where(
                active, torch.full_like(cell[:, 0], horizon), torch.full_like(cell[:, 0], -1)
            )
            self.i16["candidate_safe_prefix_steps"][row] = -1
            self.i16["candidate_full_horizon_safe"][row] = -1
            for details in self._exact_calls:
                distance_to_global = torch.cdist(details["old_xy"], self._before_position)
                global_ids = distance_to_global.argmin(dim=1)
                rows = torch.arange(len(global_ids), device=self.device)
                if bool((distance_to_global[rows, global_ids] > 1e-7).any()):
                    raise RuntimeError("cannot bind CONNECT telemetry rows to task environments")
                if len(torch.unique(global_ids)) != len(global_ids):
                    raise RuntimeError("CONNECT telemetry environment binding is not unique")
                self.i16["candidate_count"][row, global_ids] = int(details["count"])
                self.i16["candidate_horizon_steps"][row, global_ids] = int(details["horizon"])
                self.i16["candidate_safe_prefix_steps"][row, global_ids] = details["safe_prefix"]
                self.i16["candidate_full_horizon_safe"][row, global_ids] = (
                    details["full_horizon_safe"].to(torch.int16)
                )

        speed = self.task.target_vel_w[:, :2].norm(dim=1)
        decel = float(getattr(self.task.tm, "recovery_brake_decel_p05", 0.0))
        stop = speed.square() / (2.0 * decel) if math.isfinite(decel) and decel > 0 else torch.full_like(speed, float("nan"))
        self.f32["stop_distance_m"][row] = stop
        self.f32["stop_margin_m"][row] = self.f32["hard_margin_before_m"][row] - stop

    def record_substep(self):
        if self.interval < 0 or self.substep >= self.physics_substeps:
            raise RuntimeError("unexpected physics callback count")
        i, j = self.interval, self.substep
        pos = self.ctrl.position[:, :2]
        bars = self.ctrl.watchdog_bars
        half = self.ctrl.watchdog_half
        lo = self.ctrl.watchdog_lo
        hi = self.ctrl.watchdog_hi
        if bars is None:
            margin = torch.full((self.envs,), float("nan"), device=self.device)
            valid = torch.zeros((self.envs,), dtype=torch.bool, device=self.device)
        else:
            margin = _signed_aabb_margin(pos, bars, half, lo, hi)
            valid = _geometry_valid(pos, bars, half, lo, hi)
        self.sub_f32["hard_margin_m"][i, j] = margin
        support = float(self.task._target_route_support_xy[0, 0]) if self.route_enabled else float("nan")
        self.sub_f32["support_xy_m"][i, j] = support
        self.sub_f32["contact_force_n"][i, j] = self.ctrl.contact_force.norm(dim=1)
        self.sub_f32["velocity_error_mps"][i, j] = self.ctrl.last_velocity_error.norm(dim=1)
        self.sub_f32["tilt_deg"][i, j] = torch.rad2deg(self.ctrl.last_tilt_rad)
        orientation = self.ctrl.orientation
        obb_valid = torch.isfinite(self.ctrl.position).all(dim=1)
        obb_valid &= torch.isfinite(orientation).all(dim=1)
        obb_valid &= orientation.norm(dim=1) > 1e-6
        self.sub_i8["geometry_valid"][i, j] = valid.to(torch.int8)
        self.sub_i8["obb_valid"][i, j] = obb_valid.to(torch.int8)
        self.sub_i8["motor_saturated"][i, j] = self.ctrl.last_saturated.to(torch.int8)
        self.sub_i8["watchdog_breach"][i, j] = self.ctrl.watchdog_breach.to(torch.int8)
        self.substep += 1

    def finish_interval(self):
        if self.substep != self.physics_substeps:
            raise RuntimeError(
                "physics callback count drift: %d != %d" % (self.substep, self.physics_substeps)
            )
        i = self.interval
        pos = self.task.target_position[:, :2].detach()
        vel = self.task.target_vel_w[:, :2].detach()
        hard, soft = self._geometry(pos)
        self.f32["hard_margin_after_m"][i] = hard
        self.f32["soft_margin_after_m"][i] = soft
        self.f32["speed_after_mps"][i] = vel.norm(dim=1)
        self.xy["position_after_xy"][i] = pos
        self.xy["velocity_after_xy"][i] = vel

    def mark_runner_reset(self, env_ids):
        if self.interval < 0:
            raise RuntimeError("runner reset before interval")
        self.i32["runner_reset_after_interval"][self.interval, env_ids] += 1

    def write(self, path: Path, metadata: Mapping[str, object]) -> Dict[str, object]:
        if self.interval + 1 != self.steps:
            raise RuntimeError("telemetry ended before fixed step count")
        arrays: Dict[str, np.ndarray] = {}
        for collection in (self.i16, self.i32, self.f32, self.xy, self.sub_f32, self.sub_i8):
            for name, tensor in collection.items():
                arrays[name] = tensor.detach().cpu().contiguous().numpy()
        meta = dict(metadata)
        meta.update({
            "schema": SCHEMA,
            "steps": self.steps,
            "envs": self.envs,
            "physics_substeps": self.physics_substeps,
            "interval_denominator": self.steps * self.envs,
            "substep_denominator": self.steps * self.physics_substeps * self.envs,
            "state_codes": {
                "off": STATE_OFF, "normal": STATE_NORMAL, "brake": STATE_BRAKE,
                "connect": STATE_CONNECT, "route": STATE_ROUTE,
                "no_connector": STATE_NO_CONNECTOR,
            },
            "missing_int16": -1,
            "missing_float": "NaN",
        })
        arrays["metadata_json_u8"] = np.frombuffer(
            json.dumps(meta, sort_keys=True, separators=(",", ":")).encode("utf-8"),
            dtype=np.uint8,
        )
        _atomic_npz(path, arrays)
        return {"path": str(path), "sha256": sha256_file(path), "metadata": meta}


def load_and_verify(path: Path, expected_sha256: str = "") -> Dict[str, object]:
    path = path.resolve()
    if not path.is_file():
        raise RuntimeError("telemetry artifact missing: %s" % path)
    actual = sha256_file(path)
    if expected_sha256 and actual != expected_sha256:
        raise RuntimeError("telemetry artifact SHA256 mismatch")
    with np.load(str(path), allow_pickle=False) as payload:
        required = {
            "state_before", "state_after", "age_before", "age_after", "status_after",
            "hard_margin_before_m", "soft_margin_before_m", "hard_margin_after_m",
            "soft_margin_after_m", "position_before_xy", "position_after_xy", "command_xy",
            "direct_position_write", "reset_call_during_advance", "runner_reset_after_interval",
            "candidate_count", "candidate_horizon_steps", "candidate_safe_prefix_steps",
            "candidate_full_horizon_safe",
            "hard_margin_m", "contact_force_n", "geometry_valid", "obb_valid",
            "motor_saturated", "watchdog_breach", "metadata_json_u8",
        }
        missing = required - set(payload.files)
        if missing:
            raise RuntimeError("telemetry fields missing: %s" % sorted(missing))
        metadata = json.loads(bytes(payload["metadata_json_u8"].tolist()).decode("utf-8"))
        if metadata.get("schema") != SCHEMA:
            raise RuntimeError("wrong packed telemetry schema")
        steps, envs, substeps = (
            int(metadata["steps"]), int(metadata["envs"]), int(metadata["physics_substeps"])
        )
        if payload["state_before"].shape != (steps, envs):
            raise RuntimeError("interval telemetry shape mismatch")
        if payload["hard_margin_m"].shape != (steps, substeps, envs):
            raise RuntimeError("substep telemetry shape mismatch")
        if int(metadata["interval_denominator"]) != steps * envs:
            raise RuntimeError("interval denominator drift")
        if int(metadata["substep_denominator"]) != steps * substeps * envs:
            raise RuntimeError("substep denominator drift")

        before = payload["state_before"].astype(np.int16)
        after = payload["state_after"].astype(np.int16)
        allowed = (
            (before == STATE_OFF) & (after == STATE_OFF)
            | np.isin(before, [STATE_NORMAL, STATE_ROUTE])
              & np.isin(after, [STATE_NORMAL, STATE_BRAKE, STATE_CONNECT, STATE_ROUTE, STATE_NO_CONNECTOR])
            | (before == STATE_BRAKE) & np.isin(after, [STATE_BRAKE, STATE_CONNECT, STATE_NO_CONNECTOR])
            | (before == STATE_CONNECT) & np.isin(after, [STATE_CONNECT, STATE_ROUTE, STATE_NO_CONNECTOR])
            | (before == STATE_NO_CONNECTOR) & (after == STATE_NO_CONNECTOR)
        )
        runner_reset = payload["runner_reset_after_interval"] > 0
        # A terminal runner reset occurs after the recorded state; the next interval legitimately
        # starts NORMAL and is therefore excluded from cross-interval transition accounting.
        if not bool(allowed.all()):
            raise RuntimeError("illegal within-interval recovery transition")
        direct_writes = int(payload["direct_position_write"].sum())
        reset_calls = int(payload["reset_call_during_advance"].sum())
        if direct_writes != 0 or reset_calls != 0:
            raise RuntimeError("recovery path wrote target position or invoked reset")
        active = after == STATE_CONNECT
        if active.any():
            if (payload["candidate_count"][active] != 73).any():
                raise RuntimeError("CONNECT candidate count drift")
            if (payload["candidate_horizon_steps"][active] <= 0).any():
                raise RuntimeError("CONNECT horizon missing")
            if (payload["candidate_safe_prefix_steps"][active] < 0).any():
                raise RuntimeError("CONNECT safe-prefix hook missing")
        if not np.isfinite(payload["hard_margin_m"]).all():
            raise RuntimeError("substep hard margin is nonfinite")
        if not (payload["geometry_valid"] == 1).all() or not (payload["obb_valid"] == 1).all():
            raise RuntimeError("geometry/OBB validity gate failed")
        no_connector = after == STATE_NO_CONNECTOR
        nonzero_no_connector = np.linalg.norm(payload["command_xy"], axis=2) > 1e-7
        if bool((no_connector & nonzero_no_connector).any()):
            raise RuntimeError("NO_CONNECTOR emitted nonzero planar command")
        return {
            "schema": SCHEMA,
            "sha256": actual,
            "steps": steps,
            "envs": envs,
            "physics_substeps": substeps,
            "interval_denominator": steps * envs,
            "substep_denominator": steps * substeps * envs,
            "recovery_entries": int(payload["entry_delta"][:, 0].sum()),
            "route_resumes": int(payload["resume_delta"][:, 0].sum()),
            "no_connector_events": int(payload["no_connector_delta"][:, 0].sum()),
            "hard_breach_events": int(payload["hard_breach_delta"][:, 0].sum()),
            "timeout_env_intervals": int(payload["timeout_event"].sum()),
            "watchdog_breach_substeps": int(payload["watchdog_breach"].sum()),
            "contact_substeps": int((payload["contact_force_n"] > 0.05).sum()),
            "runner_reset_envs": int(runner_reset.sum()),
            "direct_position_writes": direct_writes,
            "reset_calls_during_advance": reset_calls,
        }
