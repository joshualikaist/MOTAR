"""Opt-in S-L1A/S-L1B sensor corrections; only delivered sensor evidence enters these helpers.

The legacy path does not call any helper. No simulator pose, semantic labels,
renderer-private target mask, tracker state or evaluator provenance is accepted here.
Lineages are cumulative: s_l1b_v1 keeps every S-L1A correction.
"""
import math
import os

import torch


SWITCH = "NAVRL_SENSOR_LINEAGE"
LINEAGE = "s_l1a_v1"
LINEAGES = ("s_l1a_v1", "s_l1b_v1")

# S-L1B declared calibration constants, bound by tests to the sensor configs
# (navrl_lidar_config: vertical_fov_deg_max/min, target_radius).
LIDAR_ELEVATION_MAX_DEG = 20.0
LIDAR_ELEVATION_MIN_DEG = -10.0
LIDAR_TARGET_PROXY_RADIUS_M = 0.20


def sensor_lineage_level(environ=None):
    """0 = legacy (unset/off), 1 = s_l1a_v1, 2 = s_l1b_v1; every unknown value fails closed."""
    value = (os.environ if environ is None else environ).get(SWITCH, "off")
    if value != "off" and value not in LINEAGES:
        raise ValueError(SWITCH + " must be off or one of " + ", ".join(LINEAGES))
    return 0 if value == "off" else LINEAGES.index(value) + 1


def sensor_lineage_enabled(environ=None):
    """True when the S-L1A corrections are active (s_l1a_v1 or a cumulative later lineage)."""
    return sensor_lineage_level(environ) >= 1


def conservative_carve_masks(lidar_ranges, lidar_angles, camera_depth, camera_candidates,
                             visible, target_bearing, *, target_radius=.15,
                             min_target_pixels=2, lidar_far_m=12., camera_far_m=20.,
                             max_angle_deg=15., upper_agreement_m=.55):
    """Return conservative LiDAR exclusion and camera blanking masks on the input device.

    Camera depth/candidates are current perception inputs [N,H,W], not a semantic
    renderer mask. An environment needs enough finite, positive, non-far pixels.
    The minimum of those measured ranges anchors a one-sided agreement window:
    no nearer LiDAR return is erased. Ambiguous/missing evidence authorizes nothing.
    This does not establish identity or correct camera/LiDAR registration/radii.
    """
    for value, name in ((target_radius, "target_radius"), (lidar_far_m, "lidar_far_m"),
                        (camera_far_m, "camera_far_m"), (max_angle_deg, "max_angle_deg"),
                        (upper_agreement_m, "upper_agreement_m")):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(name + " must be finite and positive")
    if (max_angle_deg >= 90 or isinstance(min_target_pixels, bool)
            or int(min_target_pixels) != min_target_pixels or min_target_pixels < 1):
        raise ValueError("invalid angle bound or minimum pixel count")
    tensors = (lidar_ranges, lidar_angles, camera_depth, camera_candidates, visible, target_bearing)
    if not all(isinstance(value, torch.Tensor) for value in tensors):
        raise ValueError("sensor inputs must be tensors")
    if (camera_depth.ndim != 3 or camera_candidates.shape != camera_depth.shape
            or camera_candidates.dtype != torch.bool):
        raise ValueError("camera depth/mask must be [N,H,W] with a boolean mask")
    n = camera_depth.shape[0]
    if (lidar_ranges.ndim != 3 or lidar_ranges.shape[0] != n
            or lidar_angles.shape != (lidar_ranges.shape[-1],)
            or visible.shape != (n,) or visible.dtype != torch.bool
            or target_bearing.shape != (n,)):
        raise ValueError("LiDAR arrays or per-environment flags have incompatible shapes/types")
    if any(value.device != camera_depth.device for value in tensors):
        raise ValueError("sensor inputs must share a device")
    if not all(value.is_floating_point() for value in (lidar_ranges, lidar_angles, camera_depth, target_bearing)):
        raise ValueError("sensor ranges and bearings must be floating point")
    valid_camera = (camera_candidates & torch.isfinite(camera_depth)
                    & (camera_depth > 0) & (camera_depth < camera_far_m))
    count = valid_camera.sum(dim=(1, 2))
    anchor = camera_depth.masked_fill(~valid_camera, math.inf).amin(dim=(1, 2))
    supported = (visible & torch.isfinite(target_bearing)
                 & (count >= min_target_pixels) & torch.isfinite(anchor))
    anchor = torch.where(supported, anchor, torch.zeros_like(anchor))
    angular = torch.asin(target_radius / (anchor + target_radius)).clamp(max=math.radians(max_angle_deg))
    angular = torch.where(supported, angular, torch.zeros_like(angular))
    delta = lidar_angles.view(1, -1) - target_bearing.view(-1, 1)
    delta = torch.atan2(torch.sin(delta), torch.cos(delta))
    valid_return = torch.isfinite(lidar_ranges) & (lidar_ranges > 0) & (lidar_ranges < lidar_far_m)
    lower, upper = anchor[:, None, None], (anchor + upper_agreement_m)[:, None, None]
    lidar = (supported[:, None, None] & valid_return & torch.isfinite(delta[:, None, :])
             & (delta[:, None, :].abs() < angular[:, None, None])
             & (lidar_ranges >= lower) & (lidar_ranges < upper))
    camera = (supported[:, None, None] & valid_camera
              & (camera_depth >= lower) & (camera_depth < upper))
    return {"lidar_carve_mask": lidar, "camera_blank_mask": camera, "anchor_m": anchor,
            "supported": supported, "angular_limit_rad": angular, "valid_candidate_count": count}


def camera_unit_rays(width, height, fx, fy, cx, cy, device=None):
    """Pinhole unit rays [H,W,3] in the camera frame (+x forward, +y left, +z up)."""
    rows = torch.arange(int(height), device=device, dtype=torch.float32)
    cols = torch.arange(int(width), device=device, dtype=torch.float32)
    vv, uu = torch.meshgrid(rows, cols, indexing="ij")
    rays = torch.stack([torch.ones_like(uu), -(uu - cx) / fx, -(vv - cy) / fy], dim=-1)
    return rays / rays.norm(dim=-1, keepdim=True)


def camera_points_lidar_frame(camera_depth, rays, rotation, translation, *, camera_far_m, blank=None):
    """S-L1B SE(3) reprojection of delivered slant depth: p = R d r_hat + t in the LiDAR frame.

    Returns per-pixel range, azimuth, elevation and validity. A pixel is valid when its depth is
    finite, positive and below the camera far plane minus 1e-3 (no-return convention) and it is
    not blanked as target.
    """
    if not math.isfinite(camera_far_m) or camera_far_m <= 0:
        raise ValueError("camera_far_m must be finite and positive")
    if (camera_depth.ndim != 3 or rays.shape != camera_depth.shape[1:] + (3,)
            or rotation.shape != (3, 3) or translation.shape != (3,)):
        raise ValueError("depth [N,H,W], rays [H,W,3], rotation [3,3], translation [3] required")
    valid = torch.isfinite(camera_depth) & (camera_depth > 0) & (camera_depth < camera_far_m - 1e-3)
    if blank is not None:
        if blank.shape != camera_depth.shape or blank.dtype != torch.bool:
            raise ValueError("blank mask must be boolean [N,H,W]")
        valid = valid & ~blank
    safe = torch.where(valid, camera_depth, torch.zeros_like(camera_depth))
    points = torch.matmul(safe.unsqueeze(-1) * rays.unsqueeze(0), rotation.transpose(0, 1)) + translation
    rho = points.norm(dim=-1)
    az = torch.atan2(points[..., 1], points[..., 0])
    el = torch.atan2(points[..., 2], torch.hypot(points[..., 0], points[..., 1]))
    return {"rho": rho, "az": az, "el": el, "valid": valid}


def lidar_envelope_bins(points, *, hbeams, vbeams, lidar_far_m, clamp,
                        elevation_max_deg=LIDAR_ELEVATION_MAX_DEG, elevation_min_deg=LIDAR_ELEVATION_MIN_DEG):
    """Minimum camera range per LiDAR (row, azimuth) bin [N,V,H]; empty bins are lidar_far_m.

    Azimuth bin j = round((pi - az)/dAz) mod H (production's decreasing bearing table); row
    e = round((el_max - el)/dEl). Strict bins drop points outside the rows' bands; the clamped,
    actor-facing envelope assigns them to the nearest edge row.
    """
    rho, valid = points["rho"], points["valid"]
    n = rho.shape[0]
    step = 2.0 * math.pi / hbeams
    j = torch.remainder(torch.round((math.pi - points["az"]) / step), hbeams).long()
    el_step = math.radians(elevation_max_deg - elevation_min_deg) / (vbeams - 1)
    e = torch.round((math.radians(elevation_max_deg) - points["el"]) / el_step).long()
    keep = valid & (rho < lidar_far_m)
    if clamp:
        e = e.clamp(0, vbeams - 1)
    else:
        keep = keep & (e >= 0) & (e <= vbeams - 1)
        e = e.clamp(0, vbeams - 1)
    env = torch.arange(n, device=rho.device).view(-1, *([1] * (rho.ndim - 1))).expand_as(rho)
    flat = (env * (vbeams * hbeams) + e * hbeams + j)[keep]
    out = torch.full((n * vbeams * hbeams,), float(lidar_far_m), device=rho.device, dtype=rho.dtype)
    out.scatter_reduce_(0, flat, rho[keep], reduce="amin", include_self=True)
    return out.view(n, vbeams, hbeams)


def common_frame_carve_masks(lidar_ranges, lidar_angles, camera_depth, camera_candidates,
                             visible, target_bearing, rays, rotation, translation, lidar_elevations, *,
                             target_radius=.15,
                             lidar_target_radius=LIDAR_TARGET_PROXY_RADIUS_M, min_target_pixels=2,
                             lidar_far_m=12., camera_far_m=20., max_angle_deg=15., upper_agreement_m=.55):
    """S-L1B carve: S-L1A support and camera blanking, LiDAR window in the common (body) frame.

    anchor_L is the minimum body-origin range of the valid candidate pixels (p = R d r_hat + t).
    The predicted LiDAR proxy surface s_L = anchor_L - (lidar_target_radius - target_radius)
    bounds the window [s_L, s_L + upper) from below with zero tolerance. Amendment 1: a LiDAR
    row is eligible only inside [min, max] body-frame elevation of the valid candidate pixels.
    """
    base = conservative_carve_masks(
        lidar_ranges, lidar_angles, camera_depth, camera_candidates, visible, target_bearing,
        target_radius=target_radius, min_target_pixels=min_target_pixels, lidar_far_m=lidar_far_m,
        camera_far_m=camera_far_m, max_angle_deg=max_angle_deg, upper_agreement_m=upper_agreement_m)
    if not math.isfinite(lidar_target_radius) or lidar_target_radius <= 0:
        raise ValueError("lidar_target_radius must be finite and positive")
    valid_camera = (camera_candidates & torch.isfinite(camera_depth)
                    & (camera_depth > 0) & (camera_depth < camera_far_m))
    safe = torch.where(valid_camera, camera_depth, torch.zeros_like(camera_depth))
    points = torch.matmul(safe.unsqueeze(-1) * rays.unsqueeze(0), rotation.transpose(0, 1)) + translation
    anchor = points.norm(dim=-1).masked_fill(~valid_camera, math.inf).amin(dim=(1, 2))
    el = torch.atan2(points[..., 2], torch.hypot(points[..., 0], points[..., 1]))
    el_lo = el.masked_fill(~valid_camera, math.inf).amin(dim=(1, 2))
    el_hi = el.masked_fill(~valid_camera, -math.inf).amax(dim=(1, 2))
    if lidar_elevations.shape != (lidar_ranges.shape[1],):
        raise ValueError("lidar_elevations must have one value per LiDAR row")
    row_ok = (lidar_elevations.view(1, -1) >= el_lo.view(-1, 1)) & (lidar_elevations.view(1, -1) <= el_hi.view(-1, 1))
    supported = base["supported"] & torch.isfinite(anchor)
    lower = torch.where(supported, anchor - (lidar_target_radius - target_radius), torch.zeros_like(anchor))
    angular = torch.asin((lidar_target_radius / (lower + lidar_target_radius)).clamp(max=1.0))
    angular = torch.where(supported, angular.clamp(max=math.radians(max_angle_deg)), torch.zeros_like(angular))
    delta = lidar_angles.view(1, -1) - target_bearing.view(-1, 1)
    delta = torch.atan2(torch.sin(delta), torch.cos(delta))
    valid_return = torch.isfinite(lidar_ranges) & (lidar_ranges > 0) & (lidar_ranges < lidar_far_m)
    lo, hi = lower[:, None, None], (lower + upper_agreement_m)[:, None, None]
    lidar = (supported[:, None, None] & valid_return & torch.isfinite(delta[:, None, :])
             & (delta[:, None, :].abs() < angular[:, None, None])
             & (lidar_ranges >= lo) & (lidar_ranges < hi) & row_ok[:, :, None])
    camera = base["camera_blank_mask"] & supported[:, None, None]
    return {"lidar_carve_mask": lidar, "camera_blank_mask": camera,
            "anchor_body_m": torch.where(supported, anchor, torch.zeros_like(anchor)),
            "lidar_surface_lower_m": lower, "supported": supported, "angular_limit_rad": angular,
            "valid_candidate_count": base["valid_candidate_count"]}
