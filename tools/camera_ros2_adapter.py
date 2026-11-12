"""ROS 2 message converters for the camera-only policy, independent of rclpy.

No subscription, publisher, camera or checkpoint is constructed here. Frames and
acquisition timestamps are explicit contracts; topic names are never evidence of
alignment, calibration or clock compatibility. Validate the binding with measured
evidence before entering a callback loop. Receipt age and command ownership belong
to the node which calls these converters.

Message semantics: the ROS 2 Humble common_interfaces source definitions:
https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg
https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg
https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg
https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

SCHEMA = "motar-ros2-rgbd-v1"
MAX_DIMENSION = 4096
MAX_SKEW_S = 0.025
# A camera-FLU vector becomes a ROS optical (right, down, forward) vector.
CAMERA_FLU_TO_OPTICAL = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])


class RosInputError(ValueError):
    pass


def unconfigured_template():
    """Engineering topic namespace only; no physical calibration/readiness implied."""
    return {"schema": SCHEMA, "status": "UNCONFIGURED",
            "topics": {"rgb": "/motar/camera/rgb/image_rect", "depth": "/motar/camera/depth/image_rect",
                       "camera_info": "/motar/camera/rgb/camera_info", "odometry": "/motar/state/odometry"},
            "frames": {"world": None, "body": None, "optical": None},
            "geometry": {"mount_flu_m": None, "optical_to_body_rotation": None, "provenance": None},
            "producer": {"rectified": None, "depth_aligned_to_rgb": None, "provenance": None},
            "conventions": {"world": "ENU", "body": "FLU", "depth": "optical_z_m"},
            "clock": {"domain": "ROS_TIME", "source": None}}


def _finite(value, shape, name):
    try:
        result = np.asarray(value, np.float64)
    except (ValueError, TypeError, OverflowError) as exc:
        raise RosInputError("invalid %s" % name) from exc
    if result.shape != shape or not np.isfinite(result).all():
        raise RosInputError("%s must be finite with shape %s" % (name, shape))
    return result


def _rotation(value, name):
    result = _finite(value, (3, 3), name)
    if (not np.allclose(result.T @ result, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(result), 1.0, atol=1e-6, rtol=0)):
        raise RosInputError("%s must be a proper rotation" % name)
    return result


def _evidence(value, name, verify_files):
    if not isinstance(value, dict) or set(value) != {"path", "sha256"}:
        raise RosInputError("%s measurement evidence path/SHA256 required" % name)
    path, digest = value["path"], value["sha256"]
    if (not isinstance(path, str) or not Path(path).is_absolute() or not isinstance(digest, str)
            or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest)):
        raise RosInputError("invalid %s evidence binding" % name)
    if verify_files:
        sha = hashlib.sha256()
        try:
            with Path(path).open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    sha.update(block)
        except OSError as exc:
            raise RosInputError("%s evidence unavailable" % name) from exc
        if sha.hexdigest() != digest:
            raise RosInputError("%s evidence SHA256 changed" % name)


def validate_binding(binding, *, verify_files=True):
    """Return a detached validated binding; reject unconfigured deployment templates.

    File verification belongs in node initialization. build_frame repeats structural
    checks with verify_files=False so no disk access enters the control callback.
    """
    if not isinstance(binding, dict) or binding.get("schema") != SCHEMA or binding.get("status") != "CONFIGURED":
        raise RosInputError("ROS camera binding is UNCONFIGURED or has an unsupported schema")
    if set(binding) != {"schema", "status", "topics", "frames", "geometry", "producer", "conventions", "clock"}:
        raise RosInputError("ROS camera binding keys differ from the supported schema")
    try:
        value = json.loads(json.dumps(binding, allow_nan=False))
    except (ValueError, TypeError) as exc:
        raise RosInputError("ROS camera binding must be finite JSON metadata") from exc
    topics, frames = value.get("topics"), value.get("frames")
    if not isinstance(topics, dict) or set(topics) != {"rgb", "depth", "camera_info", "odometry"}:
        raise RosInputError("explicit RGB/depth/CameraInfo/Odometry topics required")
    if (any(not isinstance(t, str) or not t.startswith("/") or t == "/" or "//" in t
            or any(c.isspace() for c in t) for t in topics.values()) or len(set(topics.values())) != 4):
        raise RosInputError("topics must be distinct explicit absolute ROS names")
    if not isinstance(frames, dict) or set(frames) != {"world", "body", "optical"}:
        raise RosInputError("explicit world/body/optical frames required")
    if (any(not isinstance(f, str) or not f or f.startswith("/") or any(c.isspace() for c in f)
            for f in frames.values()) or len(set(frames.values())) != 3):
        raise RosInputError("world/body/optical frames must be distinct nonempty frame IDs")
    if value.get("conventions") != {"world": "ENU", "body": "FLU", "depth": "optical_z_m"}:
        raise RosInputError("explicit ENU world, FLU body and optical-z depth conventions required")
    clock = value.get("clock")
    if (not isinstance(clock, dict) or set(clock) != {"domain", "source"} or clock.get("domain") != "ROS_TIME"
            or not isinstance(clock.get("source"), str) or not clock["source"].strip()):
        raise RosInputError("shared ROS acquisition clock domain and its source required")
    geometry, producer = value.get("geometry"), value.get("producer")
    if not isinstance(geometry, dict) or not isinstance(producer, dict):
        raise RosInputError("measured geometry and aligned/rectified producer attestation required")
    if (set(geometry) != {"mount_flu_m", "optical_to_body_rotation", "provenance"}
            or set(producer) != {"rectified", "depth_aligned_to_rgb", "provenance"}):
        raise RosInputError("geometry/producer keys differ from the supported binding schema")
    _finite(geometry.get("mount_flu_m"), (3,), "mount_flu_m")
    _rotation(geometry.get("optical_to_body_rotation"), "optical_to_body_rotation")
    if producer.get("rectified") is not True or producer.get("depth_aligned_to_rgb") is not True:
        raise RosInputError("producer must attest rectified RGB and depth aligned to that RGB camera")
    _evidence(geometry.get("provenance"), "mount/extrinsic", verify_files)
    _evidence(producer.get("provenance"), "RGB-depth producer", verify_files)
    return value


def stamp_seconds(stamp):
    """Preserve acquisition time; never replace a missing stamp with callback time."""
    sec, nanos = getattr(stamp, "sec", None), getattr(stamp, "nanosec", None)
    if type(sec) is not int or type(nanos) is not int or not 0 <= sec <= 2147483647 or not 0 <= nanos < 1000000000:
        raise RosInputError("ROS acquisition timestamp must have nonnegative integer sec/nanosec")
    if sec == 0 and nanos == 0:
        raise RosInputError("ROS acquisition timestamp must be nonzero; unstamped/latched data refused")
    return sec + nanos * 1e-9


def _header(msg, expected_frame):
    header = getattr(msg, "header", None)
    if header is None or getattr(header, "frame_id", None) != expected_frame:
        raise RosInputError("message frame_id does not match declared frame %r" % expected_frame)
    return stamp_seconds(getattr(header, "stamp", None))


def _image_bytes(msg, bytes_per_pixel):
    height, width, step, endian = (getattr(msg, k, None) for k in ("height", "width", "step", "is_bigendian"))
    if (type(height) is not int or type(width) is not int or not 1 <= height <= MAX_DIMENSION
            or not 1 <= width <= MAX_DIMENSION or type(step) is not int or step < width * bytes_per_pixel
            or step * height > 512 * 1024 * 1024 or type(endian) is not int or endian not in (0, 1)):
        raise RosInputError("malformed image dimensions/row step/endianness")
    try:
        data = memoryview(msg.data)
        if not data.c_contiguous:
            raise ValueError("non-contiguous ROS image buffer")
        data = data.cast("B")
    except (AttributeError, TypeError, ValueError) as exc:
        raise RosInputError("ROS image data must be a contiguous byte buffer") from exc
    if data.nbytes != step * height:
        raise RosInputError("image buffer size must equal row step times height exactly")
    rows = np.frombuffer(data, np.uint8).reshape(height, step)
    return rows[:, :width * bytes_per_pixel].copy(), height, width, endian


def decode_rgb(msg):
    if getattr(msg, "encoding", None) not in ("rgb8", "bgr8"):
        raise RosInputError("RGB Image encoding must be rgb8 or bgr8")
    data, height, width, _ = _image_bytes(msg, 3)
    rgb = data.reshape(height, width, 3)
    return rgb[:, :, ::-1].copy() if msg.encoding == "bgr8" else rgb.copy()


def decode_depth(msg):
    encoding = getattr(msg, "encoding", None)
    if encoding not in ("16UC1", "32FC1"):
        raise RosInputError("depth Image encoding must be 16UC1 millimetres or 32FC1 metres")
    bpp = 2 if encoding == "16UC1" else 4
    data, height, width, endian = _image_bytes(msg, bpp)
    dtype = np.dtype((">" if endian else "<") + ("u2" if bpp == 2 else "f4"))
    z = data.view(dtype).reshape(height, width).astype(np.float32)
    if encoding == "16UC1":
        z *= 0.001
    invalid = ~np.isfinite(z) | (z <= 0)
    z[invalid] = 0.0
    return z, invalid


def rectified_camera_info(msg, *, expected_frame, width, height):
    """Use rectified P, never raw K; unsupported ROI/binning/stereo offsets fail closed."""
    _header(msg, expected_frame)
    if (type(getattr(msg, "width", None)) is not int or type(getattr(msg, "height", None)) is not int
            or msg.width != width or msg.height != height):
        raise RosInputError("CameraInfo dimensions differ from delivered aligned images")
    if getattr(msg, "binning_x", None) not in (0, 1) or getattr(msg, "binning_y", None) not in (0, 1):
        raise RosInputError("non-default CameraInfo binning is unsupported")
    roi = getattr(msg, "roi", None)
    if (roi is None or any(getattr(roi, k, None) != 0 for k in ("x_offset", "y_offset", "height", "width"))
            or getattr(roi, "do_rectify", None) is not False):
        raise RosInputError("non-default CameraInfo ROI is unsupported")
    k = _finite(getattr(msg, "k", None), (9,), "CameraInfo.K").reshape(3, 3)
    p = _finite(getattr(msg, "p", None), (12,), "CameraInfo.P").reshape(3, 4)
    r = _rotation(_finite(getattr(msg, "r", None), (9,), "CameraInfo.R").reshape(3, 3), "CameraInfo.R")
    if k[0, 0] <= 0 or k[1, 1] <= 0:
        raise RosInputError("CameraInfo is uncalibrated")
    if not np.allclose(r, np.eye(3), atol=1e-8, rtol=0):
        raise RosInputError("nonidentity CameraInfo rectification rotation needs an explicit extrinsic adapter")
    if (p[0, 0] <= 0 or p[1, 1] <= 0 or not np.allclose(p[2], [0, 0, 1, 0], atol=1e-9, rtol=0)
            or abs(p[0, 1]) > 1e-9 or abs(p[1, 0]) > 1e-9 or abs(p[0, 3]) > 1e-9 or abs(p[1, 3]) > 1e-9):
        raise RosInputError("unsupported rectified projection/skew/stereo optical-center offset")
    if not -0.5 <= p[0, 2] <= width - 0.5 or not -0.5 <= p[1, 2] <= height - 0.5:
        raise RosInputError("CameraInfo principal point is outside the delivered image")
    return {"width": width, "height": height, "fx": float(p[0, 0]), "fy": float(p[1, 1]),
            "cx": float(p[0, 2]), "cy": float(p[1, 2])}


def _xyz(value, name):
    return _finite([getattr(value, k, None) for k in ("x", "y", "z")], (3,), name)


def odometry_state(msg, *, expected_world_frame, expected_body_frame):
    t = _header(msg, expected_world_frame)
    if getattr(msg, "child_frame_id", None) != expected_body_frame:
        raise RosInputError("Odometry child_frame_id does not match the declared FLU body")
    try:
        pose, twist = msg.pose.pose, msg.twist.twist
    except AttributeError as exc:
        raise RosInputError("malformed Odometry pose/twist") from exc
    pos = _xyz(getattr(pose, "position", None), "Odometry position")
    orientation = getattr(pose, "orientation", None)
    q = _finite([getattr(orientation, k, None) for k in ("x", "y", "z", "w")], (4,), "Odometry quaternion")
    norm = float(np.linalg.norm(q))
    if abs(norm - 1.0) > 1e-3:
        raise RosInputError("Odometry quaternion is not normalized")
    x, y, z, w = q / norm
    r = np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                  [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                  [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])
    vel = r @ _xyz(getattr(twist, "linear", None), "Odometry body velocity")
    angular = _xyz(getattr(twist, "angular", None), "Odometry body angular velocity")
    return {"pos": pos.tolist(), "vel": vel.tolist(), "quat_xyzw": (q / norm).tolist(),
            "yaw_rate_rps": float(angular[2]), "t": t}


def build_frame(rgb_msg, depth_msg, info_msg, odom_msg, binding):
    """Convert one synchronized sample; initialization must verify binding files first.

    No receipt-time substitution, transform lookup or interpolation is performed.
    All four acquisition timestamps must already share the declared ROS clock.
    A latched CameraInfo with an old/zero stamp is not supported by this contract;
    the producer must deliver capture-stamped calibration for the paired images.
    """
    binding = validate_binding(binding, verify_files=False)
    frames = binding["frames"]
    times = [_header(msg, frames["optical"]) for msg in (rgb_msg, depth_msg, info_msg)]
    state = odometry_state(odom_msg, expected_world_frame=frames["world"], expected_body_frame=frames["body"])
    if max(times + [state["t"]]) - min(times + [state["t"]]) > MAX_SKEW_S + 1e-9:
        raise RosInputError("RGB/depth/CameraInfo/Odometry acquisition skew exceeds 0.025 s")
    rgb, (depth, invalid) = decode_rgb(rgb_msg), decode_depth(depth_msg)
    if rgb.shape[:2] != depth.shape:
        raise RosInputError("depth is not delivered on the RGB pixel grid")
    height, width = depth.shape
    intrinsics = rectified_camera_info(info_msg, expected_frame=frames["optical"], width=width, height=height)
    geometry = binding["geometry"]
    camera_rotation = _rotation(geometry["optical_to_body_rotation"], "optical_to_body_rotation") @ CAMERA_FLU_TO_OPTICAL
    frame = {"rgb": rgb, "depth_z_m": depth, "invalid": invalid, "rgb_t": times[0], "depth_t": times[1],
             "intrinsics": intrinsics, "aligned": True, "rectified": True,
             "mount_flu_m": list(geometry["mount_flu_m"]), "camera_to_body_rotation": camera_rotation.tolist(),
             "ros_provenance": {"optical_frame": frames["optical"], "world_frame": frames["world"],
                                "body_frame": frames["body"], "camera_info_t": times[2],
                                "clock": binding["clock"], "geometry": geometry["provenance"],
                                "producer": binding["producer"]["provenance"]}}
    return frame, state
