"""Time, frame and unit contract, and the KROC2027 target-motion gates G1 and G2.

Two gates are kept separate on purpose (KROC2027 plan v2, section B):

* **G1** judges the motion generator's own velocity state (the virtual target's ``target_vel_w``),
  evaluated in float64 with a relative tolerance of 1e-4 -- the tolerance that
  ``tools/audits/audit_e2_target_validity.py`` already uses for the same state.
* **G2** judges what was actually written back: finite differences of the float32 target positions.
  Its tolerances come from float32 rounding, ``u = ulp_f32(max |coordinate|)``, times a safety factor
  ``k``. A fixed absolute 1e-4 tolerance on these differences reports rounding as a contract breach:
  a CPU emulation of fault-free bounded motion at 31-41 m produced 2,402 false acceleration and
  349,313 false turn-rate violations in 2.45 M steps, and none under the bounds below.

Derivation of the G2 bounds (per 2-vector, ``dt`` = one RL step). The generator stores
``p(t+1) = fl32(p(t) + v(t+1) dt)``, so each stored coordinate carries a rounding error of at most
``u/2``; the subtraction of two nearby float32 values is exact (Sterbenz). Hence

* finite-difference velocity ``w(t) = (p(t+1) - p(t)) / dt`` differs from ``v(t+1)`` by at most
  ``sqrt(2) (u/2) / dt``                                         -> speed bound
* ``|p(t+1) - p(t) - v(t+1) dt| <= sqrt(2) (u/2)``                 -> position-correction bound
* ``(w(t+1) - w(t)) / dt`` differs from ``(v(t+2) - v(t+1)) / dt`` by at most ``sqrt(2) u / dt^2``
                                                                   -> acceleration bound
* the course of ``w`` differs from the course of ``v`` by at most ``asin(err / |v|)``, so a course
  change per step differs by at most ``sqrt(2) u / (v_min dt^2)`` rad/s   -> turn-rate bound

G2 turn rate is judged only where both finite-difference speeds are at least 0.20 m/s, far from the
generator's 0.10 m/s heading-valid threshold; between 0.10 and 0.20 m/s only G1 judges turning.

This module imports only the standard library and torch -- never ``aerial_gym`` or Isaac Gym.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Callable, Dict, Optional

import torch

CONTRACT_ID = "KROC2027_MOTION_GATE_V1"
FLOAT32_MANTISSA_BITS = 23
SQRT2 = math.sqrt(2.0)

# Frames used by the KROC2027 recorder. Values from different frames, or from different action
# indices, are never compared as if they were the same quantity.
FRAMES = ("world_env_local", "vehicle_yaw", "body", "camera")

GATES = (
    "g1_speed",
    "g1_accel",
    "g1_turn",
    "g2_speed",
    "g2_accel",
    "g2_turn",
    "g2_position",
    "nonfinite",
    "coordinate_bound",
)


def float32_ulp(magnitude: float) -> float:
    """Spacing of float32 numbers at ``|magnitude|``.

    Spacing is non-decreasing in magnitude, so this is also the largest spacing of any float32 value
    whose absolute value does not exceed ``magnitude``. ``float32_ulp(40.0) == 2**-18``.
    """
    m = abs(float(magnitude))
    if not math.isfinite(m):
        raise ValueError("magnitude must be finite")
    if m < 2.0 ** -126:
        return 2.0 ** -149
    _, exponent = math.frexp(m)  # m = f * 2**exponent with 0.5 <= f < 1
    return 2.0 ** (exponent - 1 - FLOAT32_MANTISSA_BITS)


@dataclass(frozen=True)
class FrameStamp:
    """When and in which frame a sample was taken.

    ``action_index`` counts RL actions since the episode's reset (0 = the reset state).
    """

    action_index: int
    rl_dt_s: float
    physics_steps_per_action: int
    frame: str

    def __post_init__(self):
        if self.action_index < 0:
            raise ValueError("action_index must be non-negative")
        if not (self.rl_dt_s > 0.0 and math.isfinite(self.rl_dt_s)):
            raise ValueError("rl_dt_s must be positive and finite")
        if self.physics_steps_per_action < 1:
            raise ValueError("physics_steps_per_action must be at least 1")
        if self.frame not in FRAMES:
            raise ValueError("unknown frame %r; expected one of %s" % (self.frame, FRAMES))

    @property
    def time_s(self) -> float:
        return self.action_index * self.rl_dt_s

    @property
    def physics_tick(self) -> int:
        return self.action_index * self.physics_steps_per_action


def require_same_instant(a: FrameStamp, b: FrameStamp) -> None:
    """Refuse to compare two samples taken at different instants or in different frames."""
    if a.action_index != b.action_index:
        raise ValueError(
            "samples from action %d and action %d are not the same instant"
            % (a.action_index, b.action_index)
        )
    if a.frame != b.frame:
        raise ValueError("samples in frames %s and %s are not comparable" % (a.frame, b.frame))
    if a.rl_dt_s != b.rl_dt_s or a.physics_steps_per_action != b.physics_steps_per_action:
        raise ValueError("samples use different time bases")


def require_consecutive_series(stamps) -> None:
    """A series must advance one action at a time, in one frame, on one time base."""
    stamps = list(stamps)
    for prev, cur in zip(stamps, stamps[1:]):
        if cur.frame != prev.frame:
            raise ValueError("series changes frame from %s to %s" % (prev.frame, cur.frame))
        if (cur.rl_dt_s, cur.physics_steps_per_action) != (
            prev.rl_dt_s,
            prev.physics_steps_per_action,
        ):
            raise ValueError("series changes time base")
        if cur.action_index != prev.action_index + 1:
            raise ValueError(
                "series jumps from action %d to %d" % (prev.action_index, cur.action_index)
            )


@dataclass(frozen=True)
class MotionLimits:
    """The target-motion contract. Planar; values are the TM bounded-lineage settings."""

    speed_max_mps: float = 1.5
    accel_max_mps2: float = 4.0
    turn_rate_max_deg_s: float = 150.0
    heading_valid_speed_mps: float = 0.10  # target_motion.HEADING_VALID_SPEED_MPS
    dt_s: float = 0.1

    @property
    def turn_rate_max_rad_s(self) -> float:
        return math.radians(self.turn_rate_max_deg_s)


@dataclass(frozen=True)
class GateTolerances:
    """Tolerances fixed before the canary (KROC2027 plan v2, section B)."""

    g1_relative: float = 1e-4
    # Only matters when a limit is exactly zero (E0 static): a relative tolerance of zero is zero.
    g1_speed_floor_mps: float = 1e-6
    # G1 does not judge turning inside this band above the heading-valid threshold, because the
    # generator decides "heading valid" in float32 and a float64 recomputation can land on the
    # other side of 0.10 m/s. Steps in the band are counted, not hidden.
    g1_heading_band_mps: float = 1e-6
    g2_safety_factor: float = 2.0
    g2_turn_min_speed_mps: float = 0.20
    declared_max_abs_coordinate_m: float = 40.0

    def ulp_m(self) -> float:
        return float32_ulp(self.declared_max_abs_coordinate_m)

    def g2_speed_mps(self, dt_s: float) -> float:
        return self.g2_safety_factor * SQRT2 * (self.ulp_m() / 2.0) / dt_s

    def g2_accel_mps2(self, dt_s: float) -> float:
        return self.g2_safety_factor * SQRT2 * self.ulp_m() / dt_s ** 2

    def g2_position_m(self) -> float:
        return self.g2_safety_factor * SQRT2 * (self.ulp_m() / 2.0)

    def g2_turn_rad_s(self, v_min_mps, dt_s: float):
        """Turn-rate tolerance; ``v_min_mps`` may be a float or a tensor."""
        return self.g2_safety_factor * SQRT2 * self.ulp_m() / (v_min_mps * dt_s ** 2)

    def describe(self, dt_s: float) -> Dict[str, float]:
        out = asdict(self)
        out.update(
            ulp_m=self.ulp_m(),
            g2_speed_mps=self.g2_speed_mps(dt_s),
            g2_accel_mps2=self.g2_accel_mps2(dt_s),
            g2_position_m=self.g2_position_m(),
            g2_turn_deg_s_at_min_speed=math.degrees(
                self.g2_turn_rad_s(self.g2_turn_min_speed_mps, dt_s)
            ),
        )
        return out


def course_change_rad(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """Unsigned angle between planar vectors ``a`` and ``b`` (last dim 2), in [0, pi]."""
    cross = a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]
    dot = (a * b).sum(dim=-1)
    return torch.atan2(cross.abs(), dot)


def g1_excess(v_prev, v_next, nominal_speed_mps, limits: MotionLimits, tol: GateTolerances):
    """Per-environment excess over the G1 bounds for one generator step (positive = violation).

    ``v_prev`` and ``v_next`` are the generator's planar velocity before and after the step;
    ``nominal_speed_mps`` is each episode's own speed limit (0 for a static target). Returns
    ``(speed, accel, turn, turn_judged, heading_band)``; ``turn`` is -inf where not judged.
    """
    v_prev = v_prev.double()
    v_next = v_next.double()
    nominal = nominal_speed_mps.double().clamp(min=0.0)
    rel = 1.0 + tol.g1_relative
    speed_limit = torch.minimum(nominal, torch.full_like(nominal, limits.speed_max_mps))
    speed = v_next.norm(dim=-1) - (speed_limit * rel + tol.g1_speed_floor_mps)
    accel = (v_next - v_prev).norm(dim=-1) / limits.dt_s - (
        limits.accel_max_mps2 * rel + tol.g1_speed_floor_mps / limits.dt_s
    )
    prev_speed = v_prev.norm(dim=-1)
    next_speed = v_next.norm(dim=-1)
    threshold = limits.heading_valid_speed_mps
    heading_band = (prev_speed > threshold) & (prev_speed <= threshold + tol.g1_heading_band_mps)
    turn_judged = (prev_speed > threshold + tol.g1_heading_band_mps) & (
        next_speed > tol.g1_speed_floor_mps
    )
    turn = course_change_rad(v_prev, v_next) / limits.dt_s - limits.turn_rate_max_rad_s * rel
    turn = torch.where(turn_judged, turn, torch.full_like(turn, -math.inf))
    return speed, accel, turn, turn_judged, heading_band


def g2_excess(p_prev, p_next, v_next, w_prev, have_w_prev, limits: MotionLimits, tol: GateTolerances):
    """Per-environment excess over the G2 bounds for one written-back step (positive = violation).

    ``p_prev``/``p_next`` are the stored float32 planar positions before and after the step,
    ``v_next`` the generator velocity that produced the step, ``w_prev`` the previous
    finite-difference velocity (valid where ``have_w_prev``). Returns
    ``(w, speed, position, accel, turn, accel_judged, turn_judged)``; unjudged entries are -inf.
    """
    dt = limits.dt_s
    p_prev = p_prev.double()
    p_next = p_next.double()
    v_next = v_next.double()
    w_prev = w_prev.double()
    disp = p_next - p_prev
    w = disp / dt
    speed = w.norm(dim=-1) - (limits.speed_max_mps + tol.g2_speed_mps(dt))
    position = (disp - v_next * dt).norm(dim=-1) - tol.g2_position_m()
    accel_judged = have_w_prev.clone()
    accel = (w - w_prev).norm(dim=-1) / dt - (limits.accel_max_mps2 + tol.g2_accel_mps2(dt))
    accel = torch.where(accel_judged, accel, torch.full_like(accel, -math.inf))
    w_speed = w.norm(dim=-1)
    w_prev_speed = w_prev.norm(dim=-1)
    turn_judged = (
        have_w_prev
        & (w_speed >= tol.g2_turn_min_speed_mps)
        & (w_prev_speed >= tol.g2_turn_min_speed_mps)
    )
    v_min = torch.minimum(w_speed, w_prev_speed).clamp(min=tol.g2_turn_min_speed_mps)
    turn = course_change_rad(w_prev, w) / dt - (
        limits.turn_rate_max_rad_s + tol.g2_turn_rad_s(v_min, dt)
    )
    turn = torch.where(turn_judged, turn, torch.full_like(turn, -math.inf))
    return w, speed, position, accel, turn, accel_judged, turn_judged


class MotionGateMonitor:
    """Streaming G1/G2 monitor for N environments.

    Call ``reset(env_ids, pos, vel)`` when episodes start (the reset state is a boundary, never a
    step), then ``step(pos, vel, nominal_speed)`` after every target update. Counters are kept per
    gate; the first violation of each gate is kept with enough state to reproduce it.
    """

    def __init__(
        self,
        num_envs: int,
        device="cpu",
        limits: MotionLimits = MotionLimits(),
        tolerances: GateTolerances = GateTolerances(),
        context: Optional[Callable[[int], Dict]] = None,
    ):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.limits = limits
        self.tol = tolerances
        self.context = context
        f64 = torch.float64
        self._p = torch.zeros(self.num_envs, 2, dtype=f64, device=self.device)
        self._v = torch.zeros(self.num_envs, 2, dtype=f64, device=self.device)
        self._w = torch.zeros(self.num_envs, 2, dtype=f64, device=self.device)
        self._has_p = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._has_w = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.violations = {gate: 0 for gate in GATES}
        self.judged = {gate: 0 for gate in GATES}
        self.max_excess = {gate: -math.inf for gate in GATES}
        self.first_violation: Dict[str, Dict] = {}
        self.heading_band_steps = 0
        self.steps = 0
        self.max_abs_coordinate_m = 0.0

    def reset(self, env_ids, pos_xy, vel_xy) -> None:
        env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        if env_ids.numel() == 0:
            return
        self._p[env_ids] = pos_xy[env_ids].to(self.device).double()
        self._v[env_ids] = vel_xy[env_ids].to(self.device).double()
        self._has_p[env_ids] = True
        self._has_w[env_ids] = False
        self._observe_coordinates(self._p[env_ids])

    def _observe_coordinates(self, p) -> None:
        if p.numel():
            finite = p[torch.isfinite(p)]
            if finite.numel():
                self.max_abs_coordinate_m = max(
                    self.max_abs_coordinate_m, float(finite.abs().max().item())
                )

    def step(self, pos_xy, vel_xy, nominal_speed_mps, active=None) -> bool:
        """Judge one target update for every environment that has a previous state.

        ``active`` (optional bool [N]) restricts judgement, for example to environments whose
        episode did not end on this step. Returns True when any gate was violated on this step.
        """
        p = pos_xy.to(self.device).double()
        v = vel_xy.to(self.device).double()
        nominal = nominal_speed_mps.to(self.device).double()
        live = self._has_p.clone()
        if active is not None:
            live &= active.to(self.device)
        self.steps += 1
        any_violation = False

        finite = torch.isfinite(p).all(dim=-1) & torch.isfinite(v).all(dim=-1)
        nonfinite = live & ~finite
        any_violation |= self._count("nonfinite", nonfinite.double() - 0.5, live, p, v)
        judged = live & finite

        self._observe_coordinates(p[judged])
        # float32 spacing of each row's largest coordinate, exactly: |x| = f * 2**e, 0.5 <= f < 1,
        # spacing 2**(e - 24). A row is out of contract when its spacing exceeds the declared ulp.
        largest = torch.nan_to_num(p.abs().amax(dim=-1), nan=0.0, posinf=0.0)
        _, exponent = torch.frexp(largest)
        coord_ulp = torch.ldexp(torch.ones_like(largest), exponent - (FLOAT32_MANTISSA_BITS + 1))
        coord_excess = coord_ulp - self.tol.ulp_m()
        any_violation |= self._count("coordinate_bound", coord_excess, judged, p, v)

        s1, a1, t1, t1_judged, band = g1_excess(self._v, v, nominal, self.limits, self.tol)
        self.heading_band_steps += int((band & judged).sum().item())
        any_violation |= self._count("g1_speed", s1, judged, p, v)
        any_violation |= self._count("g1_accel", a1, judged, p, v)
        any_violation |= self._count("g1_turn", t1, judged & t1_judged, p, v)

        w, s2, pos2, a2, t2, a2_judged, t2_judged = g2_excess(
            self._p, p, v, self._w, self._has_w, self.limits, self.tol
        )
        any_violation |= self._count("g2_speed", s2, judged, p, v)
        any_violation |= self._count("g2_position", pos2, judged, p, v)
        any_violation |= self._count("g2_accel", a2, judged & a2_judged, p, v)
        any_violation |= self._count("g2_turn", t2, judged & t2_judged, p, v)

        # Advance the per-environment history only where this step was judged.
        self._p = torch.where(judged.unsqueeze(-1), p, self._p)
        self._v = torch.where(judged.unsqueeze(-1), v, self._v)
        self._w = torch.where(judged.unsqueeze(-1), w, self._w)
        self._has_w |= judged
        return any_violation

    def _count(self, gate, excess, mask, p, v) -> bool:
        n_judged = int(mask.sum().item())
        if n_judged == 0:
            return False
        self.judged[gate] += n_judged
        masked = torch.where(mask, excess, torch.full_like(excess, -math.inf))
        worst = float(masked.max().item())
        if worst > self.max_excess[gate]:
            self.max_excess[gate] = worst
        bad = mask & (excess > 0.0)
        n_bad = int(bad.sum().item())
        if n_bad == 0:
            return False
        self.violations[gate] += n_bad
        if gate not in self.first_violation:
            env = int(torch.nonzero(bad, as_tuple=False)[0, 0].item())
            record = {
                "gate": gate,
                "monitor_step": self.steps,
                "env_id": env,
                "excess": float(excess[env].item()),
                "p_prev": self._p[env].tolist(),
                "p_next": p[env].tolist(),
                "v_prev": self._v[env].tolist(),
                "v_next": v[env].tolist(),
                "w_prev": self._w[env].tolist(),
                "w_prev_valid": bool(self._has_w[env].item()),
            }
            if self.context is not None:
                record["context"] = self.context(env)
            self.first_violation[gate] = record
        return True

    def summary(self) -> Dict:
        verdict = "PASS" if not any(self.violations.values()) else "FAIL"
        if self.steps == 0:
            verdict = "NOT_RUN"
        return {
            "contract": CONTRACT_ID,
            "verdict": verdict,
            "limits": asdict(self.limits),
            "tolerances": self.tol.describe(self.limits.dt_s),
            "steps": self.steps,
            "judged": dict(self.judged),
            "violations": dict(self.violations),
            "max_excess": {k: (None if v == -math.inf else v) for k, v in self.max_excess.items()},
            "g1_heading_band_steps": self.heading_band_steps,
            "max_abs_coordinate_m": self.max_abs_coordinate_m,
            "first_violation": dict(self.first_violation),
        }
