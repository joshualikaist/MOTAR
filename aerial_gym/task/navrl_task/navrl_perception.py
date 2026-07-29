"""Perception front-end for the NavRL++-Target policy.

The actor never receives RGB-D pixels, LiDAR semantic IDs, or simulator target state.  This
module consumes raw RGB-D and raw LiDAR ranges, detects the target from appearance, associates
camera and LiDAR measurements, tracks target position/velocity/covariance, and emits a fixed-size
structured history for the Transformer policy.

The simulator may use ground truth to *render* sensor pixels, just as a real scene produces sensor
measurements.  Ground truth is intentionally absent from every public method in this file.
"""

import math
import os

import torch
import torch.nn as nn


ROBOT_HISTORY = 5
TARGET_HISTORY = 5
OBSTACLE_HISTORY = 5
# Obstacle-token capacity. Env-overridable so it can be swept; changing it changes
# STRUCTURED_OBS_DIM and therefore requires a FRESH policy (no warm-start across capacities).
MAX_OBSTACLES = int(os.environ.get("NAVRL_MAX_OBSTACLES", "").strip() or 5)
ROBOT_DIM = 10
TARGET_DIM = 16
OBSTACLE_DIM = 12

# LiDAR scan resolution. HBEAMS is THE source of truth for the horizontal beam count and must equal
# navrl_lidar_config.width (both read NAVRL_LIDAR_HBEAMS). Changing it changes STRUCTURED_OBS_DIM,
# hence a fresh policy is required. The old claim that token error was dominated by angular
# quantization was refuted by evaluation; see WORKLOG 2026-07-27 and bar-probe v2.
HBEAMS = int(os.environ.get("NAVRL_LIDAR_HBEAMS", "").strip() or 36)
VBEAMS = int(os.environ.get("NAVRL_LIDAR_VBEAMS", "").strip() or 4)
# Angular half-width blanked around an accepted obstacle token, in DEGREES (not bins: the bin size
# changes with HBEAMS). It stops one wide bar from consuming several token slots, but it also caps
# how many tokens can EVER be filled at 360/(2*this). The original +-2 bins at 36 beams was +-20 deg
# = at most ~7 tokens, so raising MAX_OBSTACLES alone silently wasted the extra slots. At +-10 deg
# the ceiling is 18, comfortably above any capacity we intend to sweep.
OBSTACLE_SUPPRESS_DEG = float(os.environ.get("NAVRL_OBSTACLE_SUPPRESS_DEG", "").strip() or 10.0)
# Angular sector (centered on body-forward, in DEGREES) the obstacle tokens are selected from.
# 360 = the original behavior: pick the nearest MAX_OBSTACLES bars anywhere around the drone.
# Why narrowing helps: the probe measured ~16 bars inside the horizon against 8 token slots, so a
# 360-degree search represents only ~50% of them -- which is exactly the measured hit_in_tokens
# (0.40-0.53). Spending slots on bars BEHIND the drone is what starves the ones ahead of it.
# Restricting the sector raises coverage geometrically (240 deg -> ~75%, 180 deg -> ~100%) WITHOUT
# changing the observation width, so a policy can warm-start across this change.
# Omnidirectional awareness is not lost: the static scan token still carries all HBEAMS bearings.
OBSTACLE_FOV_DEG = float(os.environ.get("NAVRL_OBSTACLE_FOV_DEG", "").strip() or 360.0)
if MAX_OBSTACLES <= 0:
    raise ValueError("NAVRL_MAX_OBSTACLES must be positive")
if HBEAMS <= 0 or VBEAMS <= 0:
    raise ValueError("NAVRL_LIDAR_HBEAMS and NAVRL_LIDAR_VBEAMS must be positive")
if not math.isfinite(OBSTACLE_SUPPRESS_DEG) or OBSTACLE_SUPPRESS_DEG < 0.0:
    raise ValueError("NAVRL_OBSTACLE_SUPPRESS_DEG must be finite and non-negative")
if not math.isfinite(OBSTACLE_FOV_DEG) or not 0.0 < OBSTACLE_FOV_DEG <= 360.0:
    raise ValueError("NAVRL_OBSTACLE_FOV_DEG must be in (0, 360]")
STATIC_DIM = VBEAMS * HBEAMS


def lidar_bin_bearings(device=None):
    """Physical body-frame bearing of each horizontal scan bin, in radians.

    THE single source of truth for bin -> bearing. It mirrors the warp ray generator
    (warp_lidar.py:67: azimuth = hfov_max - span * j/(W-1)), so the bearing DECREASES with the bin
    index: bin 0 = +180 deg, bin W-1 = -(180 - 360/W) deg.

    History: this module used to assume the INCREASING convention (linspace(-180+bin, +180)),
    which is the mirror image (assumed = bin_deg - true). Every obstacle token was therefore
    emitted on the wrong side of the drone, the camera fusion ghosted forward bars onto the
    opposite bearing, and the LiDAR target re-association read the mirrored bin. Physically
    adjudicated with tools/probe_lidar_bearing.py: back-projecting real returns through the
    increasing table lands on a GT bar only 13.9% of the time (mean error 2.5 m); through this
    decreasing table 94.8% (mean error 0.47 m ~= bar surface-to-center). Any edit here must keep
    tests/test_lidar_bearing_convention.py and that probe passing.
    """
    bin_rad = 2.0 * math.pi / HBEAMS
    return torch.linspace(math.pi, -math.pi + bin_rad, HBEAMS, device=device)


def camera_no_return_to_lidar_range(camera_col_min, camera_max_range, lidar_max_range):
    """Map camera columns that saw NOTHING to the LiDAR no-return value before min-fusion.

    The obstacle depth camera fills no-hit rays with its own far plane (camera_obstacle_max_range,
    10 m). When the LiDAR range exceeds that (12 m), min-fusing the raw fill value stamps a phantom
    10 m wall across the whole forward sector: the actor can never observe true free space ahead,
    and the token selector manufactures obstacles out of the camera's horizon. A column at (or
    beyond) the camera far plane carries no obstacle information, so it must fuse as lidar_max
    (= no return), leaving the LiDAR's own measurement untouched.
    """
    no_return = camera_col_min >= float(camera_max_range) - 1e-3
    return torch.where(
        no_return, torch.full_like(camera_col_min, float(lidar_max_range)), camera_col_min
    )
STRUCTURED_OBS_DIM = (
    ROBOT_HISTORY * ROBOT_DIM
    + TARGET_HISTORY * TARGET_DIM
    + OBSTACLE_HISTORY * MAX_OBSTACLES * OBSTACLE_DIM
    + STATIC_DIM
)


def _quat_rotate_xyzw(q, v):
    qvec = q[:, :3]
    uv = torch.cross(qvec, v, dim=1)
    uuv = torch.cross(qvec, uv, dim=1)
    return v + 2.0 * (q[:, 3:4] * uv + uuv)


def _quat_rotate_inverse_xyzw(q, v):
    qi = torch.cat([-q[:, :3], q[:, 3:4]], dim=1)
    return _quat_rotate_xyzw(qi, v)


class AppearanceTargetSegmenter(nn.Module):
    """Tiny learnable RGB-D pixel classifier with a usable red-target bootstrap.

    A trained checkpoint can replace the bootstrap weights through ``load_state_dict``.  The
    default is deterministic and recognizes appearance only; it has no access to simulator class
    IDs.  Keeping this head tiny makes 256 parallel camera streams practical on an 8 GB GPU.
    """

    def __init__(self):
        super().__init__()
        self.classifier = nn.Conv2d(4, 1, kernel_size=1)
        with torch.no_grad():
            self.classifier.weight.zero_()
            self.classifier.weight[0, 0, 0, 0] = 3.0
            self.classifier.weight[0, 1, 0, 0] = -2.0
            self.classifier.weight[0, 2, 0, 0] = -2.0
            self.classifier.bias.fill_(-0.9)

    def forward(self, rgb, depth, max_depth):
        depth_channel = (depth / max_depth).clamp(0.0, 1.0).unsqueeze(1)
        return torch.sigmoid(self.classifier(torch.cat([rgb, depth_channel], dim=1))).squeeze(1)


class BatchedConstantVelocityTracker:
    """GPU-vectorized 3-D constant-velocity Kalman filter."""

    def __init__(self, num_envs, device, dt, memory_s):
        self.num_envs = int(num_envs)
        self.device = device
        self.dt = float(dt)
        self.memory_s = float(memory_s)
        self.state = torch.zeros(self.num_envs, 6, device=device)
        self.cov = torch.eye(6, device=device).unsqueeze(0).repeat(self.num_envs, 1, 1)
        self.active = torch.zeros(self.num_envs, dtype=torch.bool, device=device)
        self.age = torch.full((self.num_envs,), self.memory_s, device=device)

        self.F = torch.eye(6, device=device)
        self.F[0, 3] = self.dt
        self.F[1, 4] = self.dt
        self.F[2, 5] = self.dt
        self.H = torch.zeros(3, 6, device=device)
        self.H[:, :3] = torch.eye(3, device=device)
        self.I = torch.eye(6, device=device)
        q_pos = 0.025 * self.dt * self.dt
        q_vel = 0.25 * self.dt
        self.Q = torch.diag(torch.tensor([q_pos] * 3 + [q_vel] * 3, device=device))

    def reset_idx(self, env_ids):
        self.state[env_ids] = 0.0
        self.cov[env_ids] = torch.eye(6, device=self.device)
        self.active[env_ids] = False
        self.age[env_ids] = self.memory_s

    def step(self, measurement_world, visible, measurement_var):
        active = self.active
        if bool(active.any()):
            x = self.state[active]
            p = self.cov[active]
            self.state[active] = torch.matmul(x, self.F.t())
            self.cov[active] = self.F.unsqueeze(0).matmul(p).matmul(self.F.t()) + self.Q
            self.age[active] += self.dt

        new = visible & ~self.active
        if bool(new.any()):
            self.state[new, :3] = measurement_world[new]
            self.state[new, 3:] = 0.0
            p0 = torch.zeros(int(new.sum()), 6, 6, device=self.device)
            p0[:, :3, :3] = torch.diag_embed(measurement_var[new])
            p0[:, 3:, 3:] = torch.eye(3, device=self.device).unsqueeze(0) * 1.0
            self.cov[new] = p0
            self.active[new] = True
            self.age[new] = 0.0

        self.correct(measurement_world, visible & ~new, measurement_var)

        expired = self.active & (self.age > self.memory_s)
        self.active[expired] = False
        return self.state, self.cov, self.active, self.age

    def correct(self, measurement_world, visible, measurement_var):
        """Apply a second sensor correction without advancing time again."""
        update = visible & self.active
        if bool(update.any()):
            x = self.state[update]
            p = self.cov[update]
            z = measurement_world[update]
            innovation = z - x[:, :3]
            r = torch.diag_embed(measurement_var[update])
            s = p[:, :3, :3] + r
            k = p[:, :, :3].matmul(torch.inverse(s))
            self.state[update] = x + k.matmul(innovation.unsqueeze(2)).squeeze(2)
            ikh = self.I.unsqueeze(0) - k.matmul(self.H.unsqueeze(0))
            self.cov[update] = ikh.matmul(p).matmul(ikh.transpose(1, 2)) + k.matmul(r).matmul(
                k.transpose(1, 2)
            )
            self.age[update] = 0.0


class NavRLPerceptionModule:
    """RGB-D/LiDAR fusion, tracking, uncertainty, and 2-second history encoding."""

    def __init__(self, num_envs, device, cfg, step_dt, camera_cfg):
        self.num_envs = int(num_envs)
        self.device = device
        self.cfg = cfg
        self.step_dt = float(step_dt)
        self.max_camera_range = float(camera_cfg.detector_max_range)
        self.lidar_max_range = float(getattr(cfg, "lidar_max_range", 4.0))
        # Far plane of the obstacle depth camera: columns at this value saw NOTHING and must not
        # min-fuse into a longer-range LiDAR scan (see camera_no_return_to_lidar_range).
        self.camera_obstacle_max_range = float(
            getattr(camera_cfg, "camera_obstacle_max_range", self.lidar_max_range)
        )
        self.hfov = math.radians(float(camera_cfg.detector_hfov_deg))
        self.vfov = math.radians(float(camera_cfg.detector_vfov_deg))
        self.width = int(camera_cfg.camera_width)
        self.height = int(camera_cfg.camera_height)
        self.fx = self.width / (2.0 * math.tan(self.hfov * 0.5))
        self.fy = self.height / (2.0 * math.tan(self.vfov * 0.5))
        self.cx = (self.width - 1) * 0.5
        self.cy = (self.height - 1) * 0.5
        self.camera_offset = torch.tensor(
            camera_cfg.camera_translation, dtype=torch.float32, device=device
        ).view(1, 3)
        self.target_radius = float(camera_cfg.camera_target_radius)
        self.min_pixels = int(getattr(cfg, "min_target_pixels", 2))
        self.pixel_threshold = float(getattr(cfg, "pixel_threshold", 0.55))
        self.dropout_prob = float(getattr(cfg, "detection_dropout_prob", 0.0))
        self.rgb_noise_std = float(getattr(cfg, "rgb_noise_std", 0.015))
        self.depth_noise_std = float(getattr(cfg, "depth_noise_std", 0.02))
        self.history_stride = max(1, int(round(float(cfg.history_interval_s) / self.step_dt)))
        self.step_count = 0

        self.segmenter = AppearanceTargetSegmenter().to(device).eval()
        checkpoint = str(getattr(cfg, "detector_checkpoint", "") or "").strip()
        if checkpoint:
            state = torch.load(checkpoint, map_location=device)
            self.segmenter.load_state_dict(state.get("model", state), strict=True)

        self.tracker = BatchedConstantVelocityTracker(
            num_envs, device, step_dt, float(camera_cfg.tracker_memory_s)
        )
        self.robot_history = torch.zeros(
            self.num_envs, ROBOT_HISTORY, ROBOT_DIM, device=device
        )
        self.target_history = torch.zeros(
            self.num_envs, TARGET_HISTORY, TARGET_DIM, device=device
        )
        self.obstacle_history = torch.zeros(
            self.num_envs, OBSTACLE_HISTORY, MAX_OBSTACLES, OBSTACLE_DIM, device=device
        )
        self.last_visible = torch.zeros(self.num_envs, dtype=torch.bool, device=device)
        self.last_confidence = torch.zeros(self.num_envs, device=device)

        self._u = torch.arange(self.width, device=device, dtype=torch.float32).view(1, 1, -1)
        self._v = torch.arange(self.height, device=device, dtype=torch.float32).view(1, -1, 1)
        # Bearings of the horizontal beams. The span stops 360/HBEAMS short of a full turn so the
        # first and last ray are not the same direction (matching navrl_lidar_config's fov choice):
        # HBEAMS=36 -> [-170, 180] deg at 10 deg spacing, HBEAMS=72 -> [-175, 180] at 5 deg.
        self._bin_deg = 360.0 / HBEAMS
        # Physical bin bearings (DECREASING with index, matching the warp ray generator).
        # See lidar_bin_bearings() for the mirror-bug history and the adjudication probe.
        self._lidar_angles = lidar_bin_bearings(device)
        # Suppression window converted from degrees to bins (at least 1 so a token always blanks
        # its own bin, never selecting the same bearing twice).
        self._suppress_bins = max(1, int(round(OBSTACLE_SUPPRESS_DEG / self._bin_deg)))
        # Bearings eligible to become obstacle tokens (see OBSTACLE_FOV_DEG). None = all of them.
        if OBSTACLE_FOV_DEG < 359.9:
            self._token_bearing_mask = (
                self._lidar_angles.abs() <= math.radians(OBSTACLE_FOV_DEG * 0.5)
            )
        else:
            self._token_bearing_mask = None

    def reset_idx(self, env_ids):
        self.tracker.reset_idx(env_ids)
        self.robot_history[env_ids] = 0.0
        self.target_history[env_ids] = 0.0
        self.obstacle_history[env_ids] = 0.0
        self.last_visible[env_ids] = False
        self.last_confidence[env_ids] = 0.0

    def _detect_rgbd(self, rgb, depth, training):
        if training and self.rgb_noise_std > 0.0:
            rgb = (rgb + torch.randn_like(rgb) * self.rgb_noise_std).clamp(0.0, 1.0)
        if training and self.depth_noise_std > 0.0:
            depth = (depth + torch.randn_like(depth) * self.depth_noise_std).clamp(
                0.0, self.max_camera_range
            )
        with torch.no_grad():
            score = self.segmenter(rgb, depth, self.max_camera_range)
        mask = (score >= self.pixel_threshold) & (depth < self.max_camera_range)
        count = mask.sum(dim=(1, 2))
        visible = count >= self.min_pixels
        if training and self.dropout_prob > 0.0:
            visible &= torch.rand(self.num_envs, device=self.device) >= self.dropout_prob
        mask &= visible.view(-1, 1, 1)
        denom = count.clamp(min=1).float()
        mf = mask.float()
        u = (mf * self._u).sum(dim=(1, 2)) / denom
        v = (mf * self._v).sum(dim=(1, 2)) / denom
        surface_range = (depth * mf).sum(dim=(1, 2)) / denom
        confidence = (score * mf).sum(dim=(1, 2)) / denom
        confidence *= (count.float() / max(1.0, float(self.min_pixels * 4))).clamp(max=1.0)
        confidence = torch.where(visible, confidence, torch.zeros_like(confidence))

        ray = torch.stack(
            [torch.ones_like(u), -(u - self.cx) / self.fx, -(v - self.cy) / self.fy], dim=1
        )
        ray = ray / ray.norm(dim=1, keepdim=True).clamp(min=1e-6)
        center_range = surface_range + self.target_radius
        measurement_vehicle = self.camera_offset + ray * center_range.unsqueeze(1)
        bearing = torch.atan2(measurement_vehicle[:, 1], measurement_vehicle[:, 0])
        return measurement_vehicle, surface_range, bearing, visible, confidence, mask

    def _fuse_static_and_extract_obstacles(
        self, lidar_m, raw_depth, target_pixels, target_surface_range, target_bearing, visible
    ):
        scan = lidar_m.view(self.num_envs, VBEAMS, HBEAMS).clone()

        # Camera-associated LiDAR returns are target evidence, not static obstacles. No semantic
        # ID is consulted: association uses only bearing and metric range agreement.
        angle_delta = torch.atan2(
            torch.sin(self._lidar_angles.view(1, 1, HBEAMS) - target_bearing.view(-1, 1, 1)),
            torch.cos(self._lidar_angles.view(1, 1, HBEAMS) - target_bearing.view(-1, 1, 1)),
        ).abs()
        target_like = (
            visible.view(-1, 1, 1)
            & (angle_delta < math.radians(15.0))
            & ((scan - target_surface_range.view(-1, 1, 1)).abs() < 0.55)
        )
        scan = torch.where(target_like, torch.full_like(scan, self.lidar_max_range), scan)

        # Fuse forward RGB-D geometry into the 360-degree LiDAR distance representation.
        depth_no_target = torch.where(
            target_pixels, torch.full_like(raw_depth, self.lidar_max_range), raw_depth
        )
        camera_col_min = camera_no_return_to_lidar_range(
            depth_no_target.amin(dim=1), self.camera_obstacle_max_range, self.lidar_max_range
        ).clamp(0.0, self.lidar_max_range)
        inside = self._lidar_angles.abs() <= self.hfov * 0.5
        camera_u = ((self.hfov * 0.5 - self._lidar_angles) / self.hfov * (self.width - 1)).round()
        camera_u = camera_u.long().clamp(0, self.width - 1)
        camera_ranges = camera_col_min[:, camera_u]
        fused_camera = camera_ranges.unsqueeze(1).expand(-1, VBEAMS, -1)
        scan[:, :, inside] = torch.minimum(scan[:, :, inside], fused_camera[:, :, inside])
        static_state = (scan / self.lidar_max_range).clamp(0.0, 1.0)

        # Nearest angularly separated obstacle proposals. Bars are static, hence velocity=0;
        # position and covariance come from range/angle geometry rather than simulator positions.
        nearest = scan.amin(dim=1)
        # Diagnostics only (no grad, no copy cost on the hot path): the per-bearing obstacle range
        # actually seen this step. The bar_contact probe uses it to measure how crowded the scene was
        # at the moment of a collision -- i.e. whether MAX_OBSTACLES truncated away the bar that hit.
        self.last_scan_nearest = nearest
        work = nearest.clone()
        if self._token_bearing_mask is not None:
            # Blank bearings outside the token sector so they can never win a slot. Applied to the
            # SELECTION copy only -- `static_state` above still encodes the full 360 deg scan.
            work = work.masked_fill(~self._token_bearing_mask.view(1, -1), self.lidar_max_range)
        tokens = torch.zeros(
            self.num_envs, MAX_OBSTACLES, OBSTACLE_DIM, device=self.device
        )
        rows = torch.arange(self.num_envs, device=self.device)
        for slot in range(MAX_OBSTACLES):
            r, idx = work.min(dim=1)
            valid = r < self.lidar_max_range * 0.995
            a = self._lidar_angles[idx]
            pos = torch.stack([r * torch.cos(a), r * torch.sin(a), torch.zeros_like(r)], dim=1)
            radial_sigma = 0.04 + 0.02 * r
            lateral_sigma = 0.05 + r * math.tan(math.radians(5.0))
            cov = torch.stack(
                [radial_sigma.square(), lateral_sigma.square(), torch.full_like(r, 0.12)], dim=1
            )
            feat = torch.cat(
                [
                    pos / self.lidar_max_range,
                    torch.zeros_like(pos),
                    torch.full_like(r.unsqueeze(1), 0.30 / self.lidar_max_range),
                    (1.0 - r / self.lidar_max_range).clamp(0.0, 1.0).unsqueeze(1),
                    cov / (self.lidar_max_range * self.lidar_max_range),
                    torch.ones_like(r.unsqueeze(1)),
                ],
                dim=1,
            )
            tokens[:, slot] = feat * valid.unsqueeze(1)
            for off in range(-self._suppress_bins, self._suppress_bins + 1):
                work[rows, (idx + off) % HBEAMS] = self.lidar_max_range
        return static_state.reshape(self.num_envs, -1), tokens

    def _associate_lidar_target(self, lidar_m, drone_pos_w, vehicle_quat, camera_visible):
        """Associate a raw LiDAR return with the predicted camera-initialized target track."""
        rel = _quat_rotate_inverse_xyzw(
            vehicle_quat, self.tracker.state[:, :3] - drone_pos_w
        )
        predicted_center_range = rel.norm(dim=1).clamp(min=1e-6)
        predicted_surface_range = (predicted_center_range - self.target_radius).clamp(min=0.0)
        bearing = torch.atan2(rel[:, 1], rel[:, 0])
        angular_error = torch.atan2(
            torch.sin(self._lidar_angles.view(1, -1) - bearing.unsqueeze(1)),
            torch.cos(self._lidar_angles.view(1, -1) - bearing.unsqueeze(1)),
        ).abs()
        h_idx = angular_error.argmin(dim=1)
        raw_scan = lidar_m.view(self.num_envs, VBEAMS, HBEAMS)
        rows = torch.arange(self.num_envs, device=self.device)
        ray_ranges = raw_scan[rows, :, h_idx]
        measured_surface, _ = ray_ranges.min(dim=1)
        pos_sigma = torch.diagonal(self.tracker.cov[:, :3, :3], dim1=1, dim2=2).sum(
            dim=1
        ).sqrt()
        gate = (0.35 + 2.0 * pos_sigma).clamp(max=1.0)
        valid = (
            self.tracker.active
            & ~camera_visible
            & (predicted_surface_range < self.lidar_max_range)
            & (measured_surface < self.lidar_max_range * 0.995)
            & ((measured_surface - predicted_surface_range).abs() < gate)
        )
        center_range = measured_surface + self.target_radius
        measured_vehicle = torch.stack(
            [
                center_range * torch.cos(bearing),
                center_range * torch.sin(bearing),
                rel[:, 2],
            ],
            dim=1,
        )
        measurement_world = drone_pos_w + _quat_rotate_xyzw(vehicle_quat, measured_vehicle)
        lidar_var = torch.tensor(
            [0.08**2, 0.15**2, 0.20**2], device=self.device
        ).view(1, 3).expand(self.num_envs, -1)
        self.tracker.correct(measurement_world, valid, lidar_var)
        confidence = torch.exp(
            -(measured_surface - predicted_surface_range).abs() / gate.clamp(min=1e-3)
        ) * valid.float()
        return valid, confidence, measured_surface, bearing

    def _target_features(
        self, drone_pos_w, drone_vel_w, vehicle_quat, camera_confidence, lidar_confidence
    ):
        state, cov, active, age = (
            self.tracker.state,
            self.tracker.cov,
            self.tracker.active,
            self.tracker.age,
        )
        rel_pos = _quat_rotate_inverse_xyzw(vehicle_quat, state[:, :3] - drone_pos_w)
        rel_vel = _quat_rotate_inverse_xyzw(vehicle_quat, state[:, 3:] - drone_vel_w)
        diag = torch.diagonal(cov, dim1=1, dim2=2)
        pos_var = diag[:, :3] / (self.max_camera_range * self.max_camera_range)
        vel_var = diag[:, 3:] / 4.0
        features = torch.cat(
            [
                rel_pos / self.max_camera_range,
                rel_vel / 2.0,
                pos_var.clamp(0.0, 1.0),
                vel_var.clamp(0.0, 1.0),
                camera_confidence.unsqueeze(1),
                lidar_confidence.unsqueeze(1),
                (age / self.tracker.memory_s).clamp(0.0, 1.0).unsqueeze(1),
                torch.full_like(age.unsqueeze(1), self.target_radius / self.lidar_max_range),
            ],
            dim=1,
        )
        return features * active.unsqueeze(1)

    def _update_histories(self, robot_now, target_now, obstacles_now):
        if self.step_count % self.history_stride == 0:
            self.robot_history = torch.roll(self.robot_history, shifts=-1, dims=1)
            self.target_history = torch.roll(self.target_history, shifts=-1, dims=1)
            self.obstacle_history = torch.roll(self.obstacle_history, shifts=-1, dims=1)
        self.robot_history[:, -1] = robot_now
        self.target_history[:, -1] = target_now
        self.obstacle_history[:, -1] = obstacles_now
        self.step_count += 1

    def observe(
        self,
        rgb,
        depth,
        lidar_m,
        drone_pos_w,
        drone_vel_w,
        vehicle_quat,
        yaw_rate,
        previous_action,
        max_velocity,
        flight_altitude,
        training=True,
    ):
        """Return actor-safe structured observation and sensor diagnostics."""
        meas_vehicle, surface_range, bearing, visible, confidence, pixels = self._detect_rgbd(
            rgb, depth, training
        )
        meas_world = drone_pos_w + _quat_rotate_xyzw(vehicle_quat, meas_vehicle)
        pixel_count = pixels.sum(dim=(1, 2)).float().clamp(min=1.0)
        sigma_r = 0.04 + 0.012 * surface_range + 0.15 / pixel_count.sqrt()
        sigma_lat = 0.03 + surface_range / max(self.fx, 1.0)
        measurement_var = torch.stack(
            [sigma_r.square(), sigma_lat.square(), sigma_lat.square()], dim=1
        )
        self.tracker.step(meas_world, visible, measurement_var)
        lidar_visible, lidar_confidence, lidar_surface, lidar_bearing = (
            self._associate_lidar_target(lidar_m, drone_pos_w, vehicle_quat, visible)
        )
        fused_visible = visible | lidar_visible
        fused_surface = torch.where(lidar_visible, lidar_surface, surface_range)
        fused_bearing = torch.where(lidar_visible, lidar_bearing, bearing)

        static_state, obstacles_now = self._fuse_static_and_extract_obstacles(
            lidar_m, depth, pixels, fused_surface, fused_bearing, fused_visible
        )
        target_now = self._target_features(
            drone_pos_w, drone_vel_w, vehicle_quat, confidence, lidar_confidence
        )
        robot_now = torch.cat(
            [
                _quat_rotate_inverse_xyzw(vehicle_quat, drone_vel_w) / max_velocity,
                yaw_rate.unsqueeze(1),
                previous_action,
                (drone_pos_w[:, 2:3] / max(flight_altitude * 3.0, 1e-6)),
                torch.ones(self.num_envs, 1, device=self.device),
            ],
            dim=1,
        )
        self._update_histories(robot_now, target_now, obstacles_now)
        self.last_visible[:] = fused_visible
        self.last_confidence[:] = torch.maximum(confidence, lidar_confidence)

        obs = torch.cat(
            [
                static_state,
                self.obstacle_history.reshape(self.num_envs, -1),
                self.robot_history.reshape(self.num_envs, -1),
                self.target_history.reshape(self.num_envs, -1),
            ],
            dim=1,
        )
        if obs.shape[1] != STRUCTURED_OBS_DIM:
            raise RuntimeError(
                "perception observation schema drift: got %d, expected %d"
                % (obs.shape[1], STRUCTURED_OBS_DIM)
            )
        diagnostics = {
            "visible": fused_visible,
            "camera_visible": visible,
            "lidar_visible": lidar_visible,
            "confidence": torch.maximum(confidence, lidar_confidence),
            "target_pixels": pixel_count,
            "track_age": self.tracker.age,
            "track_covariance": self.tracker.cov,
        }
        return obs, diagnostics
