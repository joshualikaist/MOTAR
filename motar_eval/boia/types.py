"""07 §§1,6,14 [ADAPTED/NEW]: immutable records and uncalibrated design priors."""
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Final, Optional, Tuple

import torch


# resources/robots/quad/quad_navrl_collide.urdf:13-20; NavRLQuadCfg:21-27.
# The only collision box is .28 x .28 x .08 m, centred on base_link. Its 3-D
# half diagonal bounds its horizontal projection at ANY attitude, unlike the
# .19799 m level-XY radius. This is geometry only, not an uncertainty allowance.
VERIFIED_COLLISION_RADIUS_M: Final[float] = math.sqrt(.14 ** 2 + .14 ** 2 + .04 ** 2)


@dataclass(frozen=True)
class BoiaParams:
    """Design priors, not fitted parameters. An engagement threshold is always explicit."""
    H: int = 10
    dt: float = 0.1
    S: int = 10
    M: int = 32
    K: int = 20
    d_int: float = 3.0
    p_pre: float = 0.5
    H_pre: int = 5
    sigma2_lost: float = 1.0
    n_lost: int = 20
    r_hard: float = VERIFIED_COLLISION_RADIUS_M
    r_body: float = 0.198  # Legacy level-XY soft prior; NOT a full-attitude body bound.
    m_s: float = 0.20
    t_det: float = 0.09
    t_react: float = 0.1
    m_free: float = 0.1
    r_T: float = 0.15
    # aerial_gym/config/asset_config/env_object_config.py:813-820 selects a pool,
    # not one width: resources/models/environment_assets/bars_h3 has 40 URDFs.
    # bar_000.urdf:14 collision box is .7096 x .5756 x 3 m; pool XY half-width
    # components span .20145-.3951 m (source inventory in 13_design_deltas.md).
    # No unique scalar prior follows: the consumer must declare one explicitly.
    r_bar_est: Optional[float] = None
    s_Q: float = 1.0
    tau_v: float = 0.4
    tau_psi: Optional[float] = None
    a_max: float = 9.81
    weights_visible: Tuple[float, ...] = (1.0, 0.2, 0.1, 0.05, 0.5)
    weights_lost: Tuple[float, ...] = (1.0, 1.0, 0.3, 0.05, 1.0)
    tau_on: Optional[float] = None
    tau_off_ratio: float = 0.5
    n_min: int = 5
    n_off: int = 3
    min_pixels: float = 2.0
    p_miss: Optional[float] = None
    beta: float = 1.0
    n_0: float = 4.0
    unknown_blocks: bool = True
    R_camera_diag: Optional[Tuple[float, float, float]] = None

    def __post_init__(self):
        if self.tau_on is None or not math.isfinite(self.tau_on) or not self.tau_on > 0:
            raise ValueError("tau_on must be explicitly declared and positive")
        if self.r_bar_est is not None and (not math.isfinite(self.r_bar_est) or self.r_bar_est < 0):
            raise ValueError("r_bar_est must be a nonnegative explicit prior")
        if self.p_miss is not None and not 0 <= self.p_miss <= 1:
            raise ValueError("p_miss must be in [0,1]")
        assert min(self.H, self.S, self.M, self.K) > 0 and self.dt > 0


@dataclass(frozen=True)
class CameraParams:
    """Declared calibration; angles are radians and offset is in the yaw-only frame."""
    hfov: float
    vfov: float
    fx: float
    fy: float
    cx: float
    cy: float
    camera_offset: Tuple[float, float, float]
    target_radius: float = 0.15
    max_camera_range: float = 20.0
    min_pixels: float = 2.0


@dataclass(frozen=True)
class FreeContext:
    """One original scan: pos (N,3), yaw (N,), radii (N,B), angles (B,)."""
    pos: torch.Tensor
    yaw: torch.Tensor
    free_radius: torch.Tensor
    angles: torch.Tensor
    far_m: float


@dataclass(frozen=True)
class OnlineInputs:
    """Copied N-first tensors; pose/vel (N,3), belief (N,6)/(N,6,6), scan (N,P)."""
    pos: torch.Tensor
    vel: torch.Tensor
    vehicle_quat: torch.Tensor
    yaw: torch.Tensor
    body_angvel: torch.Tensor
    lidar_m: torch.Tensor
    angles: torch.Tensor
    elevations: torch.Tensor
    mu: torch.Tensor
    Sigma: torch.Tensor
    active: torch.Tensor
    age: torch.Tensor
    camera_visible: torch.Tensor
    policy_action: torch.Tensor
    cam: CameraParams
    lidar_far_m: float


@dataclass(frozen=True)
class Rollout:
    """pos/vel (N,K,H,S,3), psi (N,K,H,S); step_* omit S; command (N,K,H,4)."""
    pos: torch.Tensor
    vel: torch.Tensor
    psi: torch.Tensor
    step_pos: torch.Tensor
    step_vel: torch.Tensor
    step_psi: torch.Tensor
    governed_cmd: torch.Tensor
    diagnostics: Optional[Dict[str, torch.Tensor]] = None


@dataclass(frozen=True)
class Decision:
    """Actions (N,4), candidate commands (N,K,4), scores/flags (N,K), labels (N,)."""
    action: torch.Tensor
    candidate_commands: torch.Tensor
    hard_ok: torch.Tensor
    stop_ok: torch.Tensor
    cost: torch.Tensor
    terms: Dict[str, torch.Tensor]
    advantage: torch.Tensor
    mode: torch.Tensor
    engaged: torch.Tensor
    reason: torch.Tensor
    new_state: Any = None
    trace: Any = None
    diagnostics: Any = None


class SafetyStatus:
    """Opt-in C02 model results; none of these is a flight-safety certificate."""
    MODEL_SAFE = 0
    MODEL_UNSAFE = 1
    UNKNOWN = 2
    INVALID = 3


class ContainmentReason:
    CLEAR = 0
    UNKNOWN_SPACE = 1
    INVALID_INPUT = 2
    MODEL_COLLISION = 3
    INSUFFICIENT_PREFIX = 4
    INCOMPLETE_BRAKING = 5


@dataclass(frozen=True)
class FreeBoundary2D:
    """Explicit legacy planar interpolation, NOT sensor-certified free volume.

    A scan timestamp is provenance only; this version cannot qualify ageing or
    transform the set into a measured 3-D volume. Unknown time remains None.
    """
    context: FreeContext
    scan_timestamp_s: Optional[float] = None
    model_name: str = field(default="PLANAR_SECTOR_INTERPOLATION_V1", init=False)
    physical_qualification: str = field(default="PHYSICAL_QUALIFICATION_NOT_AVAILABLE", init=False)


@dataclass(frozen=True)
class SweepBounds:
    """Nominal model allowance and separately declared, unqualified error bounds.

    None is missing evidence, not a measured zero. Even a fully populated record
    does not resolve sparse coverage, plant uncertainty or the absent 3-D map.
    This record never converts a model result into physical qualification.
    """
    model_extra_radius_m: float = 0.0
    position_error_m: Optional[float] = None
    tracking_error_m: Optional[float] = None
    sensor_error_m: Optional[float] = None
    latency_s: Optional[float] = None
    attitude_error_rad: Optional[float] = None

    def __post_init__(self):
        for name in ("model_extra_radius_m", "position_error_m", "tracking_error_m",
                     "sensor_error_m", "latency_s", "attitude_error_rad"):
            value = getattr(self, name)
            if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                      or not math.isfinite(value) or value < 0):
                raise ValueError("{} must be a finite nonnegative declaration or None".format(name))
        if self.model_extra_radius_m is None:
            raise ValueError("nominal extra radius must be explicitly numeric")

    @property
    def missing_fields(self):
        return tuple(name for name in ("position_error_m", "tracking_error_m", "sensor_error_m",
                                       "latency_s", "attitude_error_rad")
                     if getattr(self, name) is None)


@dataclass(frozen=True)
class ContainmentResult:
    """N-first status/reason tensors and boundary distance in metres."""
    status: torch.Tensor
    min_clearance_m: torch.Tensor
    reason: torch.Tensor
    physical_qualification: str = field(default="PHYSICAL_QUALIFICATION_NOT_AVAILABLE", init=False)

    @property
    def model_safe(self):
        return self.status == int(SafetyStatus.MODEL_SAFE)

    @property
    def model_ok(self):
        return self.model_safe


@dataclass(frozen=True)
class NextCycleSafetyResult:
    """Standalone opt-in diagnostics; no planner/controller installs this result."""
    status: torch.Tensor
    prefix_ok: torch.Tensor
    backup_ok: torch.Tensor
    legacy_hard_ok: torch.Tensor
    legacy_stop_ok: torch.Tensor
    prefix_min_clearance_m: torch.Tensor
    backup_min_clearance_m: torch.Tensor
    backup_distance_m: torch.Tensor
    diagnostics: Dict[str, Any]
    physical_qualification: str = field(default="PHYSICAL_QUALIFICATION_NOT_AVAILABLE", init=False)

    @property
    def model_safe(self):
        return self.status == int(SafetyStatus.MODEL_SAFE)
