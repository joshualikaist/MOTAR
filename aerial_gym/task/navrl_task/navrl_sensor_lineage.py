"""Opt-in S-L1A sensor correction; only delivered sensor evidence enters these masks.

The legacy path does not call the mask helper. No simulator pose, semantic labels,
renderer-private target mask, tracker state or evaluator provenance is accepted here.
"""
import math
import os

import torch


SWITCH = "NAVRL_SENSOR_LINEAGE"
LINEAGE = "s_l1a_v1"


def sensor_lineage_enabled(environ=None):
    """Unset/off preserves the legacy path; every unknown value fails closed."""
    value = (os.environ if environ is None else environ).get(SWITCH, "off")
    if value not in ("off", LINEAGE):
        raise ValueError(SWITCH + " must be off or " + LINEAGE)
    return value == LINEAGE


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
