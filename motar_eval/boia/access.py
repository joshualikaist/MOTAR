"""B2 / 07 §1: copy the declared sensor-and-belief interface without side effects.

This CPU-only adapter neither renders nor advances perception. Calibration comes
from the existing perception instance, not a second sensor configuration.
"""
import math

import torch

from .types import BoiaParams, CameraParams, OnlineInputs


# Audit data only. This immutable declaration must never drive object access.
FORBIDDEN_ATTRS = (
    "target_position", "target_vel_w", "target_orientation",
    "navrl_target_position", "navrl_target_orientation", "segmentation_pixels",
    "target_mask", "target_depth", "_targets", "_targets_wp", "_target_orientations",
    "DETECT_CHANNEL", "detect_*", "detect_", "obstacle_position", "states", "task_obs",
    "distractor_mask", "distractor_depth", "obstacle_depth", "target_camera_mask",
    "target_camera_depth", "obstacle_camera_depth", "navrl_target_contact",
    "navrl_target_invalid", "captured_now", "crashed_now", "silhouette_from_task",
    "recovery_visibility_probe", "_recovery_visibility_probe", "h_r3_p02_identity",
    "_episode_forensics", "_kroc_adapter", "E-L0", "KROC", "forensics",
)


def _copy_tensor(value, shape, label, *, boolean=False):
    if not isinstance(value, torch.Tensor):
        raise TypeError(label + " must be a tensor")
    if tuple(value.shape) != shape:
        raise ValueError(label + " has an incompatible shape")
    if boolean:
        if value.dtype != torch.bool:
            raise TypeError(label + " must have boolean dtype")
    elif not value.is_floating_point():
        raise TypeError(label + " must have floating dtype")
    if not bool(torch.isfinite(value).all()):
        raise ValueError(label + " must be finite")
    return value.detach().clone()


def _scalar(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(label + " must be a numeric calibration scalar")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(label + " is invalid")
    return result


def read_online(task, perception, policy_action, params):
    """Return independent copies in the frozen ``OnlineInputs`` record.

    Requires the Policy F scan contract: N environments, four elevation rows and
    72 decreasing azimuth bins. ``body_angvel`` is the measured body-frame vector,
    not a world-frame angular velocity or a claim of Euler yaw-rate equivalence.
    Calibration offset is cloned before conversion to an immutable 3-tuple.
    No perception/tracker state is advanced, corrected, reset, or written here.
    """
    if not isinstance(params, BoiaParams):
        raise TypeError("params must be a declared BoiaParams instance")
    obs = task.obs_dict
    position = obs["robot_position"]
    if not isinstance(position, torch.Tensor) or position.ndim != 2:
        raise ValueError("position must have shape (N,3)")
    n = position.shape[0]
    if n <= 0:
        raise ValueError("at least one environment is required")
    pos = _copy_tensor(position, (n, 3), "position")
    vel = _copy_tensor(obs["robot_linvel"], (n, 3), "velocity")
    quat = _copy_tensor(obs["robot_vehicle_orientation"], (n, 4), "vehicle quaternion")
    angvel = _copy_tensor(obs["robot_body_angvel"], (n, 3), "body angular velocity")
    scan = _copy_tensor(task._lidar_distance_m(), (n, 288), "LiDAR ranges")
    tracker = perception.tracker
    mu = _copy_tensor(tracker.state, (n, 6), "belief mean")
    covariance = _copy_tensor(tracker.cov, (n, 6, 6), "belief covariance")
    active = _copy_tensor(tracker.active, (n,), "track active", boolean=True)
    age = _copy_tensor(tracker.age, (n,), "track age")
    camera_visible = _copy_tensor(task._camera_visible_now, (n,), "camera visible", boolean=True)
    action = _copy_tensor(policy_action, (n, 4), "policy action")
    angles = _copy_tensor(perception._lidar_angles, (72,), "LiDAR angles")
    offset = _copy_tensor(perception.camera_offset, (1, 3), "camera offset")

    values = (vel, quat, angvel, scan, mu, covariance, active, age,
              camera_visible, action, angles, offset)
    if any(value.device != pos.device for value in values):
        raise ValueError("all tensor inputs must share one device")
    if bool((age < 0).any()):
        raise ValueError("track age must be nonnegative")
    norm_squared = (quat * quat).sum(-1)
    if not bool(torch.isfinite(norm_squared).all()) or bool(((norm_squared - 1).abs() > 1e-3).any()):
        raise ValueError("vehicle quaternion must be normalized")
    # xyzw convention: utils/math.py:145-150; homogeneous denominator also
    # tolerates harmless quaternion normalization error without changing the copy.
    x, y, z, w = quat.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), w * w + x * x - y * y - z * z)

    hfov = _scalar(perception.hfov, "horizontal FOV", positive=True)
    vfov = _scalar(perception.vfov, "vertical FOV", positive=True)
    if hfov >= math.pi or vfov >= math.pi:
        raise ValueError("pinhole FOV must be smaller than pi")
    fx = _scalar(perception.fx, "fx", positive=True)
    fy = _scalar(perception.fy, "fy", positive=True)
    cx = _scalar(perception.cx, "cx")
    cy = _scalar(perception.cy, "cy")
    radius = _scalar(perception.target_radius, "declared radius", positive=True)
    camera_far = _scalar(perception.max_camera_range, "camera far range", positive=True)
    lidar_far = _scalar(perception.lidar_max_range, "LiDAR far range", positive=True)
    # navrl_perception.py:800 reads cfg.min_target_pixels (default 2).
    # The separately named renderer setting with value 1 is not this contract.
    min_pixels = _scalar(perception.min_pixels, "minimum pixel support", positive=True)
    if min_pixels != int(min_pixels):
        raise ValueError("minimum pixel support must be integral")
    if bool(((scan < 0) | (scan > lidar_far)).any()):
        raise ValueError("LiDAR ranges must lie inside the declared sensor range")
    elevations = torch.tensor((20., 10., 0., -10.), dtype=angles.dtype,
                              device=angles.device) * (math.pi / 180)
    cam = CameraParams(hfov, vfov, fx, fy, cx, cy, tuple(offset[0].tolist()),
                       radius, camera_far, min_pixels)
    return OnlineInputs(pos, vel, quat, yaw, angvel, scan, angles, elevations,
                        mu, covariance, active, age, camera_visible, action, cam, lidar_far)
