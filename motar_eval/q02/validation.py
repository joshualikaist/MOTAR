"""Q02-A sensor validation: missing, stale, invalid and frame-mismatched inputs are reported, never repaired."""
from dataclasses import dataclass
import math

import numpy as np

from .contracts import ACCEPTED_CLOCK_PROVENANCE, CAMERA_OFFSET_M, DEPTH_CONVENTION, LIDAR_LAYOUT, YAW_VEHICLE
from .geometry import is_yaw_only

OK, MISSING, STALE, INVALID, FRAME_MISMATCH = "OK", "MISSING", "STALE", "INVALID", "FRAME_MISMATCH"


@dataclass(frozen=True)
class ValidationLimits:
    """Declared limits, not calibrated. The pinned Q01 profile delivers synchronous frames (age 0)."""
    max_camera_age_s: float = 0.1
    max_lidar_age_s: float = 0.1
    quat_tol: float = 1e-4
    cov_symmetry_tol: float = 1e-6
    cov_psd_tol: float = 1e-9


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str = ""


@dataclass(frozen=True)
class ValidationReport:
    checks: tuple

    def status(self, name):
        return dict((c.name, c.status) for c in self.checks)[name]

    def ok(self, *names):
        return all(self.status(n) == OK for n in names)

    @property
    def camera_usable(self):
        return self.ok("camera_clock", "rgb", "depth", "pose", "calibration")

    @property
    def lidar_usable(self):
        return self.ok("lidar_clock", "lidar", "pose", "calibration")

    @property
    def track_usable(self):
        return self.ok("track", "pose")

    def problems(self):
        return tuple("%s:%s%s" % (c.name, c.status, (" " + c.detail) if c.detail else "") for c in self.checks if c.status != OK)


def _finite(x):
    return x is not None and isinstance(x, (int, float)) and math.isfinite(x)


def _clock(name, clock, decision_s, max_age):
    if not (_finite(clock.capture_s) and _finite(clock.delivery_s)):
        return Check(name, MISSING, "no capture/delivery time")
    if clock.provenance not in ACCEPTED_CLOCK_PROVENANCE:
        return Check(name, MISSING, "unqualified provenance " + str(clock.provenance))
    if clock.frame != YAW_VEHICLE or clock.depth_convention != DEPTH_CONVENTION:
        return Check(name, FRAME_MISMATCH, "%s/%s" % (clock.frame, clock.depth_convention))
    if not clock.capture_s <= clock.delivery_s <= decision_s + 1e-9:
        return Check(name, INVALID, "clock order")
    if decision_s - clock.capture_s > max_age + 1e-9:
        return Check(name, STALE, "age %.3fs" % (decision_s - clock.capture_s))
    return Check(name, OK)


def _calibration(calibration, camera, lidar):
    if not calibration:
        return Check("calibration", MISSING, "no calibration tuple")
    try:
        frame, fx, fy, cx, cy, ox, oy, oz, convention, far, near, layout = calibration[:12]
    except ValueError:
        return Check("calibration", INVALID, "short calibration tuple")
    same = (frame == "YAW_ONLY" and convention == DEPTH_CONVENTION and layout == LIDAR_LAYOUT and
            all(abs(a - b) < 1e-4 for a, b in ((fx, camera.fx), (fy, camera.fy), (cx, camera.cx), (cy, camera.cy),
                                               (far, lidar.max_range_m), (near, lidar.min_range_m))) and
            all(abs(a - b) < 1e-6 for a, b in zip((ox, oy, oz), CAMERA_OFFSET_M)))
    return Check("calibration", OK if same else FRAME_MISMATCH, "" if same else "calibration differs from contract")


def validate(inp, camera, lidar, limits=ValidationLimits()):
    checks = [_clock("camera_clock", inp.camera_clock, inp.decision_s, limits.max_camera_age_s),
              _clock("lidar_clock", inp.lidar_clock, inp.decision_s, limits.max_lidar_age_s)]
    rgb = inp.rgb
    checks.append(Check("rgb", OK) if np.isfinite(rgb).all() and rgb.min() >= 0 and rgb.max() <= 1
                  else Check("rgb", INVALID, "non-finite or outside [0,1]"))
    checks.append(Check("depth", OK) if np.isfinite(inp.depth).all() and inp.depth.min() >= 0
                  else Check("depth", INVALID, "non-finite or negative"))
    lid = inp.lidar
    checks.append(Check("lidar", OK) if np.isfinite(lid).all() and lid.min() >= 0 and lid.max() <= lidar.max_range_m + 1e-6
                  else Check("lidar", INVALID, "non-finite or outside [0, max range]"))
    pose_ok = np.isfinite(inp.position).all() and np.isfinite(inp.velocity).all() and np.isfinite(inp.vehicle_quat).all()
    if not pose_ok:
        checks.append(Check("pose", INVALID, "non-finite own state"))
    elif not is_yaw_only(inp.vehicle_quat, limits.quat_tol):
        checks.append(Check("pose", FRAME_MISMATCH, "vehicle quaternion is not unit yaw-only"))
    else:
        checks.append(Check("pose", OK))
    cov = inp.track_cov
    if not inp.track_active:
        checks.append(Check("track", MISSING, "tracker inactive"))
    elif not (np.isfinite(inp.track_mean).all() and np.isfinite(cov).all() and math.isfinite(inp.track_age_s)
              and inp.track_age_s >= 0):
        checks.append(Check("track", INVALID, "non-finite track"))
    elif np.abs(cov - cov.T).max() > limits.cov_symmetry_tol or np.linalg.eigvalsh(0.5 * (cov + cov.T)).min() < -limits.cov_psd_tol:
        checks.append(Check("track", INVALID, "covariance not symmetric PSD"))
    else:
        checks.append(Check("track", OK))
    checks.append(_calibration(inp.calibration, camera, lidar))
    return ValidationReport(tuple(checks))
