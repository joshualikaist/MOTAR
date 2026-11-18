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
# Two odometry samples farther apart than the 0.15 s input age bound are never interpolated.
MAX_POSE_BRACKET_S = 0.15
# A camera-FLU vector becomes a ROS optical (right, down, forward) vector.
CAMERA_FLU_TO_OPTICAL = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])


class RosInputError(ValueError):
    pass


# Unit B (2026-11-18): PX4 VehicleLocalPosition z_valid, epv [m] and dead_reckoning, forwarded by the MOTAR PX4 gateway
# (tools/camera_ros2_px4_node.py) INSIDE its own private nav_msgs/Odometry, the same message whose pose the policy uses,
# so no second message is paired by time. Private encoding motar-px4-vertical-v1 (that Odometry already marks every
# covariance NaN = unavailable and must not feed an estimator):
#   pose.covariance[0] = VERTICAL_ENCODING_TAG (-20261118.0: a negative variance, never a valid covariance)
#   pose.covariance[1] = z_valid (1.0 true, 0.0 false)
#   pose.covariance[2] = epv in metres (finite, >= 0)
#   pose.covariance[3] = dead_reckoning (1.0 true, 0.0 false)
#   pose.covariance[4:36] and twist.covariance[0:36] = NaN
# Absent fields: every covariance NaN (the gateway's Odometry before unit B). A consumer reads the encoding only when
# its binding declares odometry_source == gateway_odometry_source(); an arbitrary Odometry producer is never trusted.
GATEWAY_ODOMETRY_PRODUCER = "motar_px4_gateway"
VERTICAL_ENCODING = "motar-px4-vertical-v1"
VERTICAL_ENCODING_TAG = -20261118.0
VERTICAL_FIELD_ORDER = ("z_valid", "epv", "dead_reckoning")     # pose.covariance[1:4]
COVARIANCE_SIZE = 36


def gateway_odometry_source():
    """The binding's explicit declaration that its odometry topic is the MOTAR PX4 gateway's private Odometry carrying
    the versioned vertical-field encoding (optional binding key `odometry_source`)."""
    return {"producer": GATEWAY_ODOMETRY_PRODUCER, "vertical_encoding": VERTICAL_ENCODING}


def declared_vertical_encoding(binding):
    """VERTICAL_ENCODING when the (validated) binding declares the gateway odometry source, else None."""
    source = binding.get("odometry_source") if isinstance(binding, dict) else None
    return VERTICAL_ENCODING if source == gateway_odometry_source() else None


def encode_vertical_covariance(fields):
    """(pose covariance, twist covariance) lists of 36 floats for the gateway's Odometry.

    fields = {"z_valid": bool, "epv": finite float >= 0, "dead_reckoning": bool} writes the encoding; None writes
    all-NaN covariances (fields absent). Anything else raises RosInputError: the gateway never publishes a guess."""
    nan = float("nan")
    twist = [nan] * COVARIANCE_SIZE
    if fields is None:
        return [nan] * COVARIANCE_SIZE, twist
    if not isinstance(fields, dict) or set(fields) != set(VERTICAL_FIELD_ORDER):
        raise RosInputError("vertical fields must be exactly %s" % (VERTICAL_FIELD_ORDER,))
    z_valid, epv, dead = fields["z_valid"], fields["epv"], fields["dead_reckoning"]
    if (type(z_valid) is not bool or type(dead) is not bool or isinstance(epv, bool)
            or not isinstance(epv, (int, float)) or not math.isfinite(epv) or epv < 0):
        raise RosInputError("vertical fields need boolean z_valid/dead_reckoning and a finite epv >= 0 m")
    pose = [VERTICAL_ENCODING_TAG, 1.0 if z_valid else 0.0, float(epv), 1.0 if dead else 0.0]
    return pose + [nan] * (COVARIANCE_SIZE - 4), twist


def _covariance(msg, part):
    value = getattr(getattr(msg, part, None), "covariance", None)
    if value is None:
        raise RosInputError("Odometry %s.covariance missing" % part)
    try:
        array = np.asarray(value, np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RosInputError("Odometry %s.covariance must be numeric" % part) from exc
    if array.shape != (COVARIANCE_SIZE,):
        raise RosInputError("Odometry %s.covariance must hold %d values" % (part, COVARIANCE_SIZE))
    return array


def decode_vertical_covariance(msg):
    """Read motar-px4-vertical-v1 from one Odometry message: {"status", "fields", "detail"}.

    status FORWARDED (fields = {z_valid, epv, dead_reckoning}), ABSENT (every covariance NaN: the gateway had no
    valid fields) or MALFORMED (anything else: another tag or version, filler that is not NaN, a flag that is not
    exactly 0 or 1, epv negative or not finite, missing or wrongly sized covariance). Never raises and never
    substitutes a default: without FORWARDED the caller attaches no field and the outdoor runtime refuses the frame."""
    try:
        pose, twist = _covariance(msg, "pose"), _covariance(msg, "twist")
    except RosInputError as exc:
        return {"status": "MALFORMED", "fields": None, "detail": str(exc)}
    if np.isnan(pose).all() and np.isnan(twist).all():
        return {"status": "ABSENT", "fields": None, "detail": "all covariances NaN: no vertical fields forwarded"}
    if pose[0] != VERTICAL_ENCODING_TAG:
        return {"status": "MALFORMED", "fields": None,
                "detail": "pose.covariance[0] is not the %s tag" % VERTICAL_ENCODING}
    if not np.isnan(pose[4:]).all() or not np.isnan(twist).all():
        return {"status": "MALFORMED", "fields": None,
                "detail": "%s requires NaN in pose.covariance[4:] and twist.covariance" % VERTICAL_ENCODING}
    z_valid, epv, dead = (float(x) for x in pose[1:4])
    if z_valid not in (0.0, 1.0) or dead not in (0.0, 1.0):
        return {"status": "MALFORMED", "fields": None, "detail": "z_valid/dead_reckoning must be exactly 0 or 1"}
    if not math.isfinite(epv) or epv < 0:
        return {"status": "MALFORMED", "fields": None, "detail": "epv must be a finite number of metres >= 0"}
    return {"status": "FORWARDED", "fields": {"z_valid": z_valid == 1.0, "epv": epv, "dead_reckoning": dead == 1.0},
            "detail": None}


def _vertical_state(messages, encoding):
    """Vertical fields for the pose built from these Odometry messages (one, or the two of an interpolation bracket).

    Every message must carry FORWARDED fields; a bracket combines them conservatively (z_valid only if both,
    epv the larger, dead_reckoning if either), so an interpolated pose is never better qualified than either sample.
    Returns (fields or None, provenance)."""
    decoded = [decode_vertical_covariance(m) for m in messages]
    provenance = {"encoding": encoding, "samples": [{"status": d["status"], "detail": d["detail"]} for d in decoded]}
    if any(d["status"] != "FORWARDED" for d in decoded):
        worst = next(d for d in decoded if d["status"] != "FORWARDED")
        provenance.update(status=worst["status"], detail=worst["detail"])
        return None, provenance
    fields = [d["fields"] for d in decoded]
    combined = {"z_valid": all(f["z_valid"] for f in fields), "epv": max(f["epv"] for f in fields),
                "dead_reckoning": any(f["dead_reckoning"] for f in fields)}
    provenance.update(status="FORWARDED", detail=None, fields=dict(combined))
    return combined, provenance


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
    keys = {"schema", "status", "topics", "frames", "geometry", "producer", "conventions", "clock"}
    # Unit B: the optional `odometry_source` declares the PX4 gateway's private Odometry (gateway_odometry_source());
    # without it (every binding before unit B) the odometry is treated as an arbitrary producer, as before.
    if set(binding) not in (keys, keys | {"odometry_source"}):
        raise RosInputError("ROS camera binding keys differ from the supported schema")
    if "odometry_source" in binding and binding["odometry_source"] != gateway_odometry_source():
        raise RosInputError("odometry_source must be exactly %s (the MOTAR PX4 gateway Odometry with the %s "
                            "encoding), or absent" % (json.dumps(gateway_odometry_source(), sort_keys=True),
                                                       VERTICAL_ENCODING))
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
    """Return (optical z [m] float32, sensor-invalid mask), with REP 117 semantics for 32FC1.

    REP 117 (https://ros.org/reps/rep-0117.html): +Inf means no return within range (too far),
    -Inf means too close to measure, NaN means invalid. +Inf is therefore NOT sensor-invalid: it is
    kept as z = +inf with invalid False, which the runtime classifies as z > MaxZ, i.e. no-return
    (never near). -Inf and NaN are sensor-invalid (a dense blob may become near, as in training),
    and so is a finite z <= 0. 16UC1 has no such codes: 0 stays invalid.
    """
    encoding = getattr(msg, "encoding", None)
    if encoding not in ("16UC1", "32FC1"):
        raise RosInputError("depth Image encoding must be 16UC1 millimetres or 32FC1 metres")
    bpp = 2 if encoding == "16UC1" else 4
    data, height, width, endian = _image_bytes(msg, bpp)
    dtype = np.dtype((">" if endian else "<") + ("u2" if bpp == 2 else "f4"))
    z = data.view(dtype).reshape(height, width).astype(np.float32)
    if encoding == "16UC1":
        z *= 0.001
    out_of_range = np.isposinf(z) if encoding == "32FC1" else np.zeros(z.shape, bool)
    invalid = ~out_of_range & (~np.isfinite(z) | (z <= 0))
    z[invalid] = 0.0
    return z, invalid


# CameraInfo distortion models that cv2.initUndistortRectifyMap implements (REP 104 / image_geometry names).
REMAP_DISTORTION_MODELS = ("plumb_bob", "rational_polynomial")
_COVERAGE_CACHE = {}
_COVERAGE_CACHE_SIZE = 4


def camera_info_coverage(msg, *, width, height):
    """Source coverage of the delivered rectified image from the CameraInfo's raw calibration.

    With nonzero D, a ones image of the raw size is remapped through
    cv2.initUndistortRectifyMap(K, D, R, P[:, :3]) with nearest interpolation and a zero border,
    exactly as tools/oakd_camera.py does for the device path: False marks rectified pixels that lie
    outside the raw image (the undistortion border), which the runtime treats as no-return, never
    as a dense invalid (near) blob. With D empty or all zero there is no undistortion border and
    coverage is all True. The case applied is recorded. Results are cached per CameraInfo content
    and returned read-only. Unsupported models with nonzero D refuse (fail closed).
    """
    k = _finite(getattr(msg, "k", None), (9,), "CameraInfo.K").reshape(3, 3)
    r = _finite(getattr(msg, "r", None), (9,), "CameraInfo.R").reshape(3, 3)
    p = _finite(getattr(msg, "p", None), (12,), "CameraInfo.P").reshape(3, 4)
    raw_d = getattr(msg, "d", None)
    if raw_d is None:
        raise RosInputError("CameraInfo.D missing; an empty D is required for undistorted images")
    try:
        d = np.asarray(list(raw_d), np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RosInputError("CameraInfo.D must be numeric") from exc
    if d.ndim != 1 or not np.isfinite(d).all():
        raise RosInputError("CameraInfo.D must be a finite vector")
    model = getattr(msg, "distortion_model", None)
    model = model if isinstance(model, str) else None
    key = (int(width), int(height), model, tuple(k.ravel().tolist()), tuple(d.tolist()),
           tuple(r.ravel().tolist()), tuple(p.ravel().tolist()))
    cached = _COVERAGE_CACHE.get(key)
    if cached is None:
        if not d.any():
            coverage = np.ones((height, width), bool)
            case = "zero_distortion_all_covered"
        else:
            if model not in REMAP_DISTORTION_MODELS or d.size not in (4, 5, 8, 12, 14):
                raise RosInputError("nonzero CameraInfo.D needs a %s model with 4/5/8/12/14 coefficients"
                                    % "/".join(REMAP_DISTORTION_MODELS))
            try:
                import cv2
            except ImportError as exc:
                raise RosInputError("OpenCV is required to derive undistortion coverage") from exc
            mapx, mapy = cv2.initUndistortRectifyMap(k, d, r, np.ascontiguousarray(p[:, :3]), (width, height),
                                                     cv2.CV_32FC1)
            coverage = cv2.remap(np.ones((height, width), np.uint8), mapx, mapy, cv2.INTER_NEAREST,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=0) == 1
            case = "undistortion_remap"
        coverage.flags.writeable = False
        provenance = {"case": case, "distortion_model": model, "uncovered_pixels": int((~coverage).sum()),
                      "method": "cv2.initUndistortRectifyMap(K, D, R, P), ones image, nearest, zero border"
                      if case == "undistortion_remap" else "D all zero: no undistortion border"}
        while len(_COVERAGE_CACHE) >= _COVERAGE_CACHE_SIZE:
            _COVERAGE_CACHE.pop(next(iter(_COVERAGE_CACHE)))
        cached = _COVERAGE_CACHE[key] = (coverage, provenance)
    return cached[0], dict(cached[1])


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


def _slerp(q0, q1, w):
    """Shortest-arc spherical interpolation of unit xyzw quaternions, w in [0, 1]."""
    q0, q1 = np.asarray(q0, np.float64), np.asarray(q1, np.float64)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    dot = min(dot, 1.0)
    angle = math.acos(dot)
    if angle < 1e-9:
        q = (1.0 - w) * q0 + w * q1
    else:
        q = (math.sin((1.0 - w) * angle) * q0 + math.sin(w * angle) * q1) / math.sin(angle)
    return q / np.linalg.norm(q)


def interpolate_state(before, after, t):
    """Estimator state at capture time t from two odometry states that bracket it.

    Position, world velocity and yaw rate are linear in time; orientation is slerped.
    Extrapolation and brackets longer than MAX_POSE_BRACKET_S are refused.
    """
    t0, t1, t = float(before["t"]), float(after["t"]), float(t)
    if not t0 <= t <= t1:
        raise RosInputError("odometry samples do not bracket the capture time; extrapolation refused")
    if t1 - t0 > MAX_POSE_BRACKET_S + 1e-9:
        raise RosInputError("odometry bracket exceeds %.3f s" % MAX_POSE_BRACKET_S)
    if t1 == t0:
        return dict(before)
    w = (t - t0) / (t1 - t0)
    mix = lambda key: ((1.0 - w) * np.asarray(before[key], np.float64) + w * np.asarray(after[key], np.float64))
    return {"pos": mix("pos").tolist(), "vel": mix("vel").tolist(),
            "quat_xyzw": _slerp(before["quat_xyzw"], after["quat_xyzw"], w).tolist(),
            "yaw_rate_rps": float((1.0 - w) * before["yaw_rate_rps"] + w * after["yaw_rate_rps"]), "t": t}


def build_frame(rgb_msg, depth_msg, info_msg, odom_msg, binding, *, odometry_bracket=None, vertical_encoding=None):
    """Convert one synchronized sample; initialization must verify binding files first.

    vertical_encoding (unit B; the session passes it for an outdoor-profile runtime only): None (default) reads no
    vertical field, exactly as before. VERTICAL_ENCODING requires the binding's odometry_source declaration and decodes
    z_valid/epv/dead_reckoning from the very Odometry message(s) whose pose is used (the nearest selection, or both
    samples of the interpolation bracket); FORWARDED fields are added to the state, otherwise none is added (the outdoor
    runtime then refuses the frame: estimator_vertical_fields_absent). frame['vertical_provenance'] records the case.

    No receipt-time substitution or transform lookup is performed. All acquisition
    timestamps must already share the declared ROS clock, and odom_msg (the nearest
    selection) must lie within 0.025 s of the images as before. When the caller also
    passes odometry_bracket = (before, after) with before.t <= rgb_t <= after.t, the
    pose is interpolated to the RGB capture time (t = rgb_t). Otherwise the nearest
    selection is used unchanged. frame['pose_provenance'] records which happened.
    frame['coverage'] is the CameraInfo-derived source coverage (camera_info_coverage) and
    frame['coverage_provenance'] records whether the undistortion remap or the zero-D case applied.
    A latched CameraInfo with an old/zero stamp is not supported by this contract;
    the producer must deliver capture-stamped calibration for the paired images.
    """
    binding = validate_binding(binding, verify_files=False)
    if vertical_encoding is not None and (vertical_encoding != VERTICAL_ENCODING
                                          or declared_vertical_encoding(binding) != vertical_encoding):
        raise RosInputError("vertical fields need the binding's odometry_source declaration of %s" % VERTICAL_ENCODING)
    frames = binding["frames"]
    times = [_header(msg, frames["optical"]) for msg in (rgb_msg, depth_msg, info_msg)]
    state = odometry_state(odom_msg, expected_world_frame=frames["world"], expected_body_frame=frames["body"])
    if max(times + [state["t"]]) - min(times + [state["t"]]) > MAX_SKEW_S + 1e-9:
        raise RosInputError("RGB/depth/CameraInfo/Odometry acquisition skew exceeds 0.025 s")
    pose = {"pose_interpolated": False, "odometry_t": [state["t"]], "pose_skew_s": abs(state["t"] - times[0])}
    pose_messages = (odom_msg,)
    if odometry_bracket is not None:
        before, after = (odometry_state(m, expected_world_frame=frames["world"], expected_body_frame=frames["body"])
                         for m in odometry_bracket)
        interpolated = interpolate_state(before, after, times[0])
        if after["t"] > before["t"]:
            state = interpolated
            pose = {"pose_interpolated": True, "odometry_t": [before["t"], after["t"]], "pose_skew_s": 0.0}
            pose_messages = tuple(odometry_bracket)
        else:
            # One sample exactly at the capture time: use it as is.
            state = before
            pose = {"pose_interpolated": False, "odometry_t": [before["t"]], "pose_skew_s": 0.0}
            pose_messages = (odometry_bracket[0],)
    vertical = None
    if vertical_encoding is not None:
        fields, vertical = _vertical_state(pose_messages, vertical_encoding)
        vertical["odometry_t"] = list(pose["odometry_t"])
        if fields is not None:
            state = dict(state, **fields)
    rgb, (depth, invalid) = decode_rgb(rgb_msg), decode_depth(depth_msg)
    if rgb.shape[:2] != depth.shape:
        raise RosInputError("depth is not delivered on the RGB pixel grid")
    height, width = depth.shape
    intrinsics = rectified_camera_info(info_msg, expected_frame=frames["optical"], width=width, height=height)
    coverage, coverage_provenance = camera_info_coverage(info_msg, width=width, height=height)
    geometry = binding["geometry"]
    camera_rotation = _rotation(geometry["optical_to_body_rotation"], "optical_to_body_rotation") @ CAMERA_FLU_TO_OPTICAL
    frame = {"rgb": rgb, "depth_z_m": depth, "invalid": invalid, "coverage": coverage,
             "coverage_provenance": coverage_provenance, "rgb_t": times[0], "depth_t": times[1],
             "intrinsics": intrinsics, "aligned": True, "rectified": True,
             "mount_flu_m": list(geometry["mount_flu_m"]), "camera_to_body_rotation": camera_rotation.tolist(),
             "ros_provenance": {"optical_frame": frames["optical"], "world_frame": frames["world"],
                                "body_frame": frames["body"], "camera_info_t": times[2],
                                "clock": binding["clock"], "geometry": geometry["provenance"],
                                "producer": binding["producer"]["provenance"]},
             "pose_provenance": pose}
    if vertical is not None:
        frame["vertical_provenance"] = vertical
    return frame, state
