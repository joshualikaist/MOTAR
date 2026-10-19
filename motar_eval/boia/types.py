"""07 §§1,6,14 [ADAPTED/NEW]: immutable records and uncalibrated design priors."""
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import torch


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
    r_hard: float = 0.14
    r_body: float = 0.198
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
