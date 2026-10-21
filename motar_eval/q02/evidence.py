"""Module A, Q02-A minimum: an explainable evidence record and a basic geometric hypothesis.

The hypothesis is a deterministic rule over measured quantities. It is not a calibrated
classifier and carries no probability: thresholds are declared priors (EvidenceParams), and
UNKNOWN is returned whenever the inputs cannot separate the cases. In the pinned Frozen F
profile the camera detection is oracle-grade (the renderer rasterizes the target from its
true pose and it is the only painted object), so agreement with that flag is not evidence
that this estimator would work with a real detector.
"""
from dataclasses import dataclass
import math

import numpy as np

from .contracts import FROZEN_F_CAMERA, FROZEN_F_LIDAR
from .geometry import (angle_diff, cov_world_to_vehicle, project, quat_yaw_xyzw, vehicle_to_camera,
                       world_to_vehicle)
from .validation import ValidationLimits, validate

TARGET_VISIBLE, OCCLUDED, OUT_OF_FOV = "TARGET_VISIBLE", "OCCLUDED", "OUT_OF_FOV"
DETECTION_MISS, IDENTITY_AMBIGUOUS, UNKNOWN = "DETECTION_MISS", "IDENTITY_AMBIGUOUS", "UNKNOWN"
HYPOTHESES = (TARGET_VISIBLE, OCCLUDED, OUT_OF_FOV, DETECTION_MISS, IDENTITY_AMBIGUOUS, UNKNOWN)


@dataclass(frozen=True)
class EvidenceParams:
    """Declared priors, NOT calibrated. Changing them changes only the hypothesis rule."""
    sigma_k: float = 2.0                 # interval half-width in standard deviations
    range_tol_m: float = 0.3             # surface-consistency tolerance beyond the interval
    foreground_margin_m: float = 0.5     # a return this far before the interval is foreground
    max_range_sigma_m: float = 1.5       # beyond this the belief is too diffuse to test
    max_belief_age_s: float = 2.0        # prediction-only horizon accepted for a hypothesis
    depth_window_min_px: int = 2
    depth_window_max_px: int = 12
    min_fraction: float = 0.5            # share of a window that must agree to count


@dataclass(frozen=True)
class EvidenceRecord:
    hypothesis: str
    reasons: tuple
    source: str
    camera_detected: bool
    confidence_raw: float
    belief_kind: str
    track_age_s: float
    predicted_vehicle_m: tuple
    predicted_range_m: float
    bearing_deg: float
    elevation_deg: float
    sigma_range_m: float
    sigma_lateral_m: float
    fov_points_inside: int
    fov_points_total: int
    camera_surface_interval_m: tuple
    lidar_surface_interval_m: tuple
    depth_pixels: int
    depth_foreground: int
    depth_consistent: int
    depth_free: int
    lidar_beams: int
    lidar_foreground: int
    lidar_consistent: int
    lidar_far: int
    validation_problems: tuple
    confidence_semantics: str = "SEGMENTER_SCORE_X_PIXEL_SUPPORT_NOT_A_PROBABILITY"
    ground_truth_used: bool = False
    utility: str = "ABSTAIN"
    calibration: str = "NONE_DECLARED_PRIORS"


def _record(hypothesis, reasons, source, inp, report, g=None):
    g = g or {}
    nan = float("nan")
    return EvidenceRecord(hypothesis, tuple(reasons), source, inp.camera_detected, inp.confidence,
                          "MEASUREMENT_UPDATED" if inp.fused_update else "PREDICTED_ONLY", inp.track_age_s,
                          tuple(float(x) for x in g.get("p_v", (nan, nan, nan))), g.get("range", nan),
                          g.get("bearing", nan), g.get("elevation", nan), g.get("sigma_r", nan), g.get("sigma_lat", nan),
                          g.get("fov_in", 0), g.get("fov_total", 0), g.get("cam_iv", (nan, nan)), g.get("lid_iv", (nan, nan)),
                          g.get("d_n", 0), g.get("d_fg", 0), g.get("d_cons", 0), g.get("d_free", 0),
                          g.get("l_n", 0), g.get("l_fg", 0), g.get("l_cons", 0), g.get("l_far", 0), report.problems())


def _belief_geometry(inp, camera, lidar, params):
    yaw = quat_yaw_xyzw(inp.vehicle_quat)
    p_v = world_to_vehicle(inp.track_mean[:3], inp.position, yaw)
    cov_v = cov_world_to_vehicle(inp.track_cov[:3, :3], yaw)
    rng = float(np.linalg.norm(p_v))
    n = p_v / max(rng, 1e-9)
    sigma_r = float(math.sqrt(max(n @ cov_v @ n, 0.0)))
    perp = np.eye(3) - np.outer(n, n)
    sigma_lat = float(math.sqrt(max(np.linalg.eigvalsh(perp @ cov_v @ perp).max(), 0.0)))
    lam, vec = np.linalg.eigh(0.5 * (cov_v + cov_v.T))
    offsets = [np.zeros(3)] + [s * params.sigma_k * math.sqrt(max(l, 0.0)) * vec[:, i]
                               for i, l in enumerate(lam) for s in (1.0, -1.0)]
    pts_cam = np.array([vehicle_to_camera(p_v + o, camera) for o in offsets])
    _, _, _, in_img = project(pts_cam, camera)
    k = params.sigma_k
    r_cam = float(np.linalg.norm(vehicle_to_camera(p_v, camera)))
    return dict(yaw=yaw, p_v=p_v, range=rng, sigma_r=sigma_r, sigma_lat=sigma_lat,
                bearing=math.degrees(math.atan2(p_v[1], p_v[0])),
                elevation=math.degrees(math.atan2(p_v[2], math.hypot(p_v[0], p_v[1]))),
                fov_in=int(in_img.sum()), fov_total=len(offsets), mean_in_image=bool(in_img[0]),
                cam_iv=(r_cam - camera.target_radius - k * sigma_r, r_cam - camera.target_radius + k * sigma_r),
                lid_iv=(rng - lidar.target_proxy_radius_m - k * sigma_r, rng - lidar.target_proxy_radius_m + k * sigma_r),
                r_cam=r_cam)


def _depth_evidence(inp, camera, params, g):
    u, v, _, inside = project(vehicle_to_camera(g["p_v"], camera)[None], camera)
    if not inside[0]:
        return dict(d_n=0, d_fg=0, d_cons=0, d_free=0)
    x = max(vehicle_to_camera(g["p_v"], camera)[0], 1e-6)
    half = int(min(params.depth_window_max_px, max(params.depth_window_min_px,
               math.ceil(camera.fx * (params.sigma_k * g["sigma_lat"] + camera.target_radius) / x))))
    uc, vc = int(round(float(u[0]))), int(round(float(v[0])))
    h, w = inp.depth.shape
    win = inp.depth[max(0, vc - half):min(h, vc + half + 1), max(0, uc - half):min(w, uc + half + 1)].astype(float)
    lo, hi = g["cam_iv"]
    tol, fg = params.range_tol_m, params.foreground_margin_m
    return dict(d_n=int(win.size), d_fg=int((win < lo - fg).sum()),
                d_cons=int(((win >= lo - tol) & (win <= hi + tol)).sum()), d_free=int((win > hi + tol).sum()))


def _lidar_evidence(inp, lidar, params, g):
    az, el = lidar.azimuth_rad(), lidar.elevation_rad()
    d_az, d_el = lidar.spacing_rad()
    rng = max(g["range"], 1e-6)
    spread = math.atan2(params.sigma_k * g["sigma_lat"] + lidar.target_proxy_radius_m, rng)
    sel = ((np.abs(angle_diff(az, math.radians(g["bearing"])))[None, :] <= spread + d_az / 2) &
           (np.abs(el - math.radians(g["elevation"]))[:, None] <= spread + d_el / 2))
    r = inp.lidar.astype(float)[sel]
    lo, hi = g["lid_iv"]
    tol, fg = params.range_tol_m, params.foreground_margin_m
    far = r >= lidar.max_range_m - 1e-6
    return dict(l_n=int(r.size), l_fg=int(((r < lo - fg) & ~far).sum()),
                l_cons=int(((r >= lo - tol) & (r <= hi + tol) & ~far).sum()), l_far=int(far.sum()))


def estimate(inp, camera=FROZEN_F_CAMERA, lidar=FROZEN_F_LIDAR, params=EvidenceParams(), limits=ValidationLimits()):
    """One evidence record for one decision. Pure: no RNG, no I/O, inputs are read-only."""
    report = validate(inp, camera, lidar, limits)
    if inp.camera_detected:
        if report.camera_usable:
            return _record(TARGET_VISIBLE, ("CURRENT_CAMERA_DETECTION",), "CAMERA_DETECTION", inp, report)
        return _record(UNKNOWN, ("CAMERA_DETECTION_WITH_UNUSABLE_CAMERA_INPUT",), "NONE", inp, report)
    if not report.track_usable:
        return _record(UNKNOWN, ("NO_USABLE_TRACK",), "NONE", inp, report)
    g = _belief_geometry(inp, camera, lidar, params)
    if not report.camera_usable:
        return _record(UNKNOWN, ("CAMERA_INPUT_UNUSABLE",), "TRACKER_PREDICTION", inp, report, g)
    if inp.track_age_s > params.max_belief_age_s:
        return _record(UNKNOWN, ("BELIEF_STALE",), "TRACKER_PREDICTION", inp, report, g)
    if g["sigma_r"] > params.max_range_sigma_m:
        return _record(UNKNOWN, ("BELIEF_TOO_DIFFUSE",), "TRACKER_PREDICTION", inp, report, g)
    if g["fov_in"] == 0:
        return _record(OUT_OF_FOV, ("ALL_BELIEF_POINTS_OUTSIDE_CAMERA_IMAGE",), "TRACKER_PREDICTION", inp, report, g)
    if g["fov_in"] < g["fov_total"]:
        return _record(UNKNOWN, ("FOV_BOUNDARY_UNCERTAIN",), "TRACKER_PREDICTION", inp, report, g)
    if g["cam_iv"][1] > camera.max_camera_range:
        return _record(UNKNOWN, ("BEYOND_DETECTION_RANGE",), "TRACKER_PREDICTION", inp, report, g)
    projected_px = math.pi * (camera.fx * camera.target_radius / max(g["r_cam"], 1e-6)) ** 2
    if projected_px < camera.min_pixels:
        return _record(UNKNOWN, ("BELOW_PIXEL_SUPPORT",), "TRACKER_PREDICTION", inp, report, g)
    g.update(_depth_evidence(inp, camera, params, g))
    reasons = []
    if report.lidar_usable:
        g.update(_lidar_evidence(inp, lidar, params, g))
        if g["l_n"] == 0:
            reasons.append("LIDAR_NO_BEAM_COVERAGE")
    else:
        reasons.append("LIDAR_INPUT_UNUSABLE")
    n = max(g["d_n"], 1)
    depth_fg = g["d_fg"] / n >= params.min_fraction
    depth_free = g["d_free"] / n >= params.min_fraction
    depth_cons = g["d_cons"] > 0
    lidar_fg, lidar_cons = g.get("l_fg", 0) > 0, g.get("l_cons", 0) > 0
    if depth_fg:
        reasons.append("DEPTH_FOREGROUND_BEFORE_EXPECTED_SURFACE")
        if lidar_cons:
            reasons.append("LIDAR_RETURN_ALSO_AT_EXPECTED_RANGE")
        return _record(OCCLUDED, reasons, "TRACKER_PREDICTION", inp, report, g)
    if depth_cons or lidar_cons:
        reasons.append("SURFACE_AT_EXPECTED_RANGE_WITHOUT_CAMERA_DETECTION")
        return _record(IDENTITY_AMBIGUOUS, reasons, "TRACKER_PREDICTION", inp, report, g)
    if lidar_fg:
        reasons.append("LIDAR_ONLY_FOREGROUND")
        return _record(OCCLUDED, reasons, "TRACKER_PREDICTION", inp, report, g)
    if depth_free:
        reasons += ["CAMERA_SEES_BEYOND_EXPECTED_SURFACE", "BELIEF_ERROR_NOT_EXCLUDED"]
        return _record(DETECTION_MISS, reasons, "TRACKER_PREDICTION", inp, report, g)
    reasons.append("INCONCLUSIVE_DEPTH_WINDOW")
    return _record(UNKNOWN, reasons, "TRACKER_PREDICTION", inp, report, g)
