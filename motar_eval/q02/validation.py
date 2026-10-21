"""Q02-A sensor validation: missing, stale, invalid and frame-mismatched inputs are reported, never repaired."""
from dataclasses import dataclass
import math

import numpy as np

from .contracts import (ACCEPTED_CLOCK_PROVENANCE, CAMERA_OFFSET_M, DEPTH_CONVENTION, FROZEN_F_PROFILE, INVALID_SAMPLE,
                        LIDAR_LAYOUT, NEAR_CLIPPED, NO_RETURN, VALID_HIT, YAW_VEHICLE)
from .geometry import RECOVERY_NOT_CONVERGED, RECOVERY_OK, is_yaw_only, recover_native_depth

OK, MISSING, STALE, INVALID, FRAME_MISMATCH = "OK", "MISSING", "STALE", "INVALID", "FRAME_MISMATCH"
UNAVAILABLE = "UNAVAILABLE"   # input is valid, but this derived quantity cannot be obtained from it
CONTACT_RANGE = "CONTACT_RANGE"


@dataclass(frozen=True)
class ValidationLimits:
    """Declared limits, not calibrated. The pinned Q01 profile delivers synchronous frames (age 0)."""
    max_camera_age_s: float = 0.1
    max_lidar_age_s: float = 0.1
    quat_tol: float = 1e-4
    cov_symmetry_tol: float = 1e-6
    cov_psd_tol: float = 1e-9
    max_pasted_fraction: float = 0.5   # beyond this too little of the obstacle render is left to read


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    detail: str = ""


@dataclass(frozen=True)
class ValidationReport:
    checks: tuple
    depth_native: object = None      # (24, 40) recovered native obstacle depth, NaN = unobserved;
                                     # None unless obstacle_depth is OK (never read it otherwise)
    depth_pasted: object = None      # (90, 160) pixels the obstacle render does not explain (target/distractor)
    depth_residual: object = None
    depth_excluded: object = None    # (90, 160) pixels left out of the native solve (pasted blobs grown)

    def status(self, name):
        return dict((c.name, c.status) for c in self.checks)[name]

    def ok(self, *names):
        return all(self.status(n) == OK for n in names)

    @property
    def camera_usable(self):
        return self.ok("camera_clock", "rgb", "depth", "depth_structure", "pose", "calibration")

    @property
    def obstacle_depth_usable(self):
        """The native obstacle grid can be read as ray samples (camera usable and not paste-dominated)."""
        return self.camera_usable and self.ok("obstacle_depth")

    @property
    def camera_contact_range(self):
        """The camera sees something within contact range (obstacle grid withheld; a safety warning)."""
        return dict((c.name, c.detail) for c in self.checks).get("obstacle_depth", "").startswith(CONTACT_RANGE)

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


def classify_depth_native(x, profile=FROZEN_F_PROFILE):
    """Recovered native obstacle-depth samples -> VALID_HIT / NO_RETURN / NEAR_CLIPPED / INVALID (unobserved)."""
    x = np.asarray(x, float)
    out = np.full(x.shape, VALID_HIT, dtype="<U12")
    out[x >= profile.obstacle_far_m - 1e-3] = NO_RETURN
    out[x < profile.depth_near_clip_m] = NEAR_CLIPPED
    out[~np.isfinite(x)] = INVALID_SAMPLE
    return out


def classify_lidar(r, model, profile=FROZEN_F_PROFILE):
    """Getter ranges -> VALID_HIT in [0.2, 12) / NO_RETURN at 12 / NEAR_CLIPPED at 0 / INVALID otherwise."""
    r = np.asarray(r, float)
    out = np.full(r.shape, INVALID_SAMPLE, dtype="<U12")
    finite = np.isfinite(r)
    out[finite & (r >= model.min_range_m) & (r < profile.lidar_no_return_m - 1e-6)] = VALID_HIT
    out[finite & (np.abs(r - profile.lidar_no_return_m) <= 1e-6)] = NO_RETURN
    out[finite & (np.abs(r - profile.lidar_near_clip_value) <= 1e-6)] = NEAR_CLIPPED
    return out


def _depth_structure(depth, profile, max_pasted_fraction):
    """The delivered depth must be a bilinear upsample of a native grid in [0, far] plus pasted pixels
    (depth_structure). Whether the obstacle grid can then be read is a separate check (obstacle_depth):
    a paste that dominates the image or spans opposite borders leaves the frame valid for the camera
    flag but unusable as obstacle ray samples."""
    try:
        rec = recover_native_depth(depth, profile.obstacle_native_wh, fill_value=profile.obstacle_far_m)
    except np.linalg.LinAlgError:
        return (Check("depth_structure", FRAME_MISMATCH, "native recovery failed"),
                Check("obstacle_depth", INVALID, "depth structure not verified"), None)
    obs = np.isfinite(rec.native)
    ok = (rec.residual <= 2e-3 and rec.status != RECOVERY_NOT_CONVERGED and
          (rec.native[obs] <= profile.obstacle_far_m + 2e-3).all() and (rec.native[obs] >= -2e-3).all() and
          (np.asarray(depth)[~rec.excluded] <= profile.obstacle_far_m + 2e-3).all())
    detail = "" if ok else "not a %dx%d upsample with far %.1f m (residual %.1e, pasted %.2f, %s)" % (
        profile.obstacle_native_wh[0], profile.obstacle_native_wh[1], profile.obstacle_far_m, rec.residual,
        rec.excluded.mean(), rec.status)
    structure = Check("depth_structure", OK if ok else FRAME_MISMATCH, detail)
    if not ok:
        obstacle = Check("obstacle_depth", INVALID, "depth structure not verified")
    elif rec.excluded.mean() > max_pasted_fraction:
        obstacle = Check("obstacle_depth", UNAVAILABLE, "PASTED_DOMINATES %.2f of the image" % rec.excluded.mean())
    elif rec.status != RECOVERY_OK:
        obstacle = Check("obstacle_depth", UNAVAILABLE, rec.status)
    elif obs.any() and rec.native[obs].min() < profile.obstacle_contact_m:
        # Something within contact range of the camera. A paste that fills the image with a smooth
        # surface (the target at contact range) is indistinguishable from an obstacle render here.
        obstacle = Check("obstacle_depth", UNAVAILABLE, "%s %.3f m" % (CONTACT_RANGE, rec.native[obs].min()))
    else:
        obstacle = Check("obstacle_depth", OK)
    return structure, obstacle, rec


def validate(inp, camera, lidar, limits=ValidationLimits(), profile=FROZEN_F_PROFILE):
    checks = [_clock("camera_clock", inp.camera_clock, inp.decision_s, limits.max_camera_age_s),
              _clock("lidar_clock", inp.lidar_clock, inp.decision_s, limits.max_lidar_age_s)]
    rgb = inp.rgb
    checks.append(Check("rgb", OK) if np.isfinite(rgb).all() and rgb.min() >= 0 and rgb.max() <= 1
                  else Check("rgb", INVALID, "non-finite or outside [0,1]"))
    depth = inp.depth
    rec = None
    if not (np.isfinite(depth).all() and depth.min() >= 0):
        checks.append(Check("depth", INVALID, "non-finite or negative"))
    elif (depth < profile.depth_near_clip_m).all():
        checks.append(Check("depth", INVALID, "whole frame near-clipped (all ~0): not a range image"))
    else:
        checks.append(Check("depth", OK))
    if checks[-1].status == OK:
        structure, obstacle, rec = _depth_structure(depth, profile, limits.max_pasted_fraction)
        checks += [structure, obstacle]
    else:
        checks += [Check("depth_structure", INVALID, "depth invalid"), Check("obstacle_depth", INVALID, "depth invalid")]
    lid = inp.lidar
    categories = classify_lidar(lid, lidar, profile)
    if (categories == INVALID_SAMPLE).any():
        checks.append(Check("lidar", INVALID, "%d samples outside the getter contract (NaN/Inf or (0, 0.2) m or > 12 m)"
                            % int((categories == INVALID_SAMPLE).sum())))
    elif (categories == NEAR_CLIPPED).all():
        checks.append(Check("lidar", INVALID, "whole scan near-clipped (all 0): not a range scan"))
    else:
        checks.append(Check("lidar", OK))
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
    if rec is None:
        return ValidationReport(tuple(checks))
    native = rec.native if dict((c.name, c.status) for c in checks)["obstacle_depth"] == OK else None
    return ValidationReport(tuple(checks), native, rec.foreign, rec.residual, rec.excluded)
