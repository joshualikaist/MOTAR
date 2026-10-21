"""Module A (contract Q02A-v2.1): explainable sensor evidence about a tracked target's line of sight.

Deterministic rule over measured samples; not a calibrated classifier and no probability. UNKNOWN
is returned whenever the inputs cannot separate the cases, and UNKNOWN is never free space or a
safe action. v2 repairs the v1 defects found by the Fable review (Q02-A REVIEW.md):

* Camera depth is read as the production produces it: a 40x24 obstacle render (environment only,
  never the target) with a 10 m no-return fill, bilinearly upsampled, with target/distractor pixels
  pasted on top. The native grid is recovered exactly and each native ray is classified
  VALID_HIT / NO_RETURN / NEAR_CLIPPED / unobserved. A 10 m sample is "no hit within 10 m", never
  a surface, and space beyond 10 m is never observed free.
* Every sensor sample is tested by geometry, not by angle windows: a ray informs about the target
  only if it passes within the target envelope (sensor proxy radius + k * lateral sigma) of the
  predicted centre, from that sensor's own origin.
* LiDAR 0.0 is NEAR_CLIPPED (< 0.2 m): a near-obstacle safety warning, not a range for occlusion.
v2.1 repairs the P2 findings of the v2 re-review (q02a_v2_fable_rereview REVIEW.md R1-R4): the paste
exclusion is closed on curved/border-crossing blobs, the extent test looks only at neighbouring
beams, identity support needs pasted depth inside the track's range interval (a surface at that
range is not identity evidence), and a paste that dominates the image removes the obstacle ray
samples but not the camera flag.
In the Frozen F profile the camera flag is oracle-grade (true-pose rasterization), so agreement
with it is not evidence that this rule would work with a real detector.
"""
from dataclasses import dataclass
import math

import numpy as np

from .contracts import (EVIDENCE_CONTRACT_VERSION, FROZEN_F_CAMERA, FROZEN_F_LIDAR, FROZEN_F_PROFILE, NEAR_CLIPPED,
                        NO_RETURN, VALID_HIT)
from .geometry import (cov_world_to_vehicle, lidar_directions, native_camera_rays, pixel_rays, point_line_distance,
                       project, quat_yaw_xyzw, ray_envelope, vehicle_to_camera, world_to_vehicle)
from .validation import ValidationLimits, classify_depth_native, classify_lidar, validate

TARGET_VISIBLE, OCCLUDED, OUT_OF_FOV = "TARGET_VISIBLE", "OCCLUDED", "OUT_OF_FOV"
DETECTION_MISS, IDENTITY_AMBIGUOUS, UNKNOWN = "DETECTION_MISS", "IDENTITY_AMBIGUOUS", "UNKNOWN"
HYPOTHESES = (TARGET_VISIBLE, OCCLUDED, OUT_OF_FOV, DETECTION_MISS, IDENTITY_AMBIGUOUS, UNKNOWN)
CONFIDENCE_SEMANTICS = "MAX_OF_CAMERA_SEGMENTER_SUPPORT_AND_LIDAR_ASSOCIATION_GATE_NOT_A_PROBABILITY"


@dataclass(frozen=True)
class EvidenceParams:
    """Declared priors, NOT calibrated. Changing them changes only the hypothesis rule."""
    sigma_k: float = 2.0                 # envelope/interval half-width in standard deviations
    range_tol_m: float = 0.3             # surface-consistency tolerance beyond the interval
    foreground_margin_m: float = 0.5     # a hit this far before the interval is foreground
    max_range_sigma_m: float = 1.5       # line-of-sight sigma beyond which no ray test is made
    max_envelope_extra_m: float = 0.8    # cap on k * lateral sigma: wider beliefs are not tested
    max_belief_age_s: float = 2.0        # prediction-only horizon accepted for a hypothesis
    min_fraction: float = 0.5            # share of envelope rays that must agree
    extent_margin_m: float = 0.15        # foreground wider than proxy radius + this is not the target
    extent_range_window_m: float = 0.3
    extent_neighbour_rows: int = 1       # extent test: only hits within this many rows/columns of a
    extent_neighbour_cols: int = 2       # foreground ray (beam grid; LiDAR azimuth wraps)


@dataclass(frozen=True)
class SensorRays:
    """Counts over the rays of one sensor that pass through the target envelope."""
    rays: int = 0
    foreground: int = 0      # VALID_HIT before the interval
    consistent: int = 0      # VALID_HIT inside the interval
    clear: int = 0           # passes the envelope: hit after it, or no return with far range beyond it
    censored: int = 0        # no return, but the sensor's far range ends before the envelope exit
    near_clipped: int = 0
    unobserved: int = 0
    foreground_extended: bool = False


@dataclass(frozen=True)
class EvidenceRecord:
    hypothesis: str
    reasons: tuple
    source: str
    identity_status: str
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
    camera_envelope_m: float
    lidar_envelope_m: float
    camera: SensorRays
    lidar: SensorRays
    pasted_pixels_in_footprint: int
    pasted_pixels_at_track_range: int
    near_obstacle_warning: bool
    near_clipped_lidar_beams: int
    validation_problems: tuple
    contract_version: str = EVIDENCE_CONTRACT_VERSION
    confidence_semantics: str = CONFIDENCE_SEMANTICS
    ground_truth_used: bool = False
    utility: str = "ABSTAIN"
    calibration: str = "NONE_DECLARED_PRIORS"


def _record(hypothesis, reasons, source, identity, inp, report, g=None, cam=SensorRays(), lid=SensorRays(), pasted=(0, 0),
            near_beams=0):
    g, nan = g or {}, float("nan")
    return EvidenceRecord(hypothesis, tuple(dict.fromkeys(reasons)), source, identity, inp.camera_detected, inp.confidence,
                          "MEASUREMENT_UPDATED" if inp.fused_update else "PREDICTED_ONLY", inp.track_age_s,
                          tuple(float(x) for x in g.get("p_v", (nan, nan, nan))), g.get("range", nan),
                          g.get("bearing", nan), g.get("elevation", nan), g.get("sigma_r", nan), g.get("sigma_lat", nan),
                          g.get("fov_in", 0), g.get("fov_total", 0), g.get("env_cam", nan), g.get("env_lid", nan),
                          cam, lid, int(pasted[0]), int(pasted[1]), bool(near_beams > 0 or cam.near_clipped > 0), int(near_beams),
                          report.problems())


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
    extra = params.sigma_k * sigma_lat
    return dict(yaw=yaw, p_v=p_v, range=rng, sigma_r=sigma_r, sigma_lat=sigma_lat, extra=extra,
                bearing=math.degrees(math.atan2(p_v[1], p_v[0])),
                elevation=math.degrees(math.atan2(p_v[2], math.hypot(p_v[0], p_v[1]))),
                fov_in=int(in_img.sum()), fov_total=len(offsets),
                r_cam=float(np.linalg.norm(vehicle_to_camera(p_v, camera))),
                env_cam=camera.target_radius + extra, env_lid=lidar.target_proxy_radius_m + extra)


def _ray_counts(origin, dirs, ranges, cats, far_m, centre, radius, sigma_r, params, proxy_radius, grid=None):
    """Classify every ray that passes through the target envelope (from this sensor's own origin).
    `grid` = (rows, cols, azimuth_wraps) of the flattened ray layout, for the extent test."""
    inside, entry, exit_, _ = ray_envelope(origin, dirs, centre, radius)
    k, tol, fg_m = params.sigma_k, params.range_tol_m, params.foreground_margin_m
    lo, hi = entry - k * sigma_r, exit_ + k * sigma_r
    r, c = ranges[inside], cats[inside]
    lo, hi = lo[inside], hi[inside]
    hit = c == VALID_HIT
    fg = hit & (r < lo - fg_m)
    cons = hit & (r >= lo - tol) & (r <= hi + tol)
    beyond = hit & (r > hi + tol)
    noret = c == NO_RETURN
    clear = beyond | (noret & (far_m > hi + tol))
    censored = noret & ~(far_m > hi + tol)
    extended = False
    if fg.any() and grid is not None:
        # Is the foreground surface wider than the target proxy? Only hits on rays next to a
        # foreground ray and at its range count, so an unrelated object elsewhere cannot answer.
        rows, cols, wraps = grid
        t_fg = float(r[fg].min())
        all_r, all_c = np.divmod(np.arange(ranges.size), cols)
        fg_r, fg_c = np.divmod(np.flatnonzero(inside)[fg], cols)
        dc = np.abs(all_c[:, None] - fg_c[None, :])
        if wraps:
            dc = np.minimum(dc, cols - dc)
        adjacent = ((np.abs(all_r[:, None] - fg_r[None, :]) <= params.extent_neighbour_rows) &
                    (dc <= params.extent_neighbour_cols)).any(axis=1)
        near = adjacent & (cats == VALID_HIT) & (np.abs(ranges - t_fg) <= params.extent_range_window_m)
        los = (np.asarray(centre, float) - np.asarray(origin, float))
        los = los / max(np.linalg.norm(los), 1e-9)
        if near.any():
            pts = np.asarray(origin, float) + dirs[near] * ranges[near][:, None]
            extended = bool((point_line_distance(pts, origin, los) > proxy_radius + params.extent_margin_m).any())
    return SensorRays(int(inside.sum()), int(fg.sum()), int(cons.sum()), int(clear.sum()), int(censored.sum()),
                      int((c == NEAR_CLIPPED).sum()), int((~np.isin(c, (VALID_HIT, NO_RETURN, NEAR_CLIPPED))).sum()),
                      extended)


def _camera_evidence(inp, report, camera, profile, params, g):
    if not report.obstacle_depth_usable:
        return SensorRays()
    origin = np.asarray(camera.camera_offset, float)
    dirs = native_camera_rays(camera, profile.obstacle_native_wh).reshape(-1, 3)
    native = np.asarray(report.depth_native, float).reshape(-1)
    cats = classify_depth_native(native, profile)
    w, h = profile.obstacle_native_wh
    return _ray_counts(origin, dirs, np.nan_to_num(native, nan=-1.0), cats, profile.obstacle_far_m, g["p_v"], g["env_cam"],
                       g["sigma_r"], params, camera.target_radius, (h, w, False))


def _lidar_evidence(inp, lidar, profile, params, g):
    dirs = lidar_directions(lidar).reshape(-1, 3)
    r = np.asarray(inp.lidar, float).reshape(-1)
    cats = classify_lidar(r, lidar, profile)
    return _ray_counts(np.zeros(3), dirs, r, cats, profile.lidar_no_return_m, g["p_v"], g["env_lid"], g["sigma_r"],
                       params, lidar.target_proxy_radius_m, (lidar.vbeams, lidar.hbeams, True)), int((cats == NEAR_CLIPPED).sum())


def _pasted_in_footprint(inp, report, camera, params, g):
    """Pasted (target/distractor) pixels whose full-resolution rays pass through the camera envelope,
    and how many of them carry a depth inside the track's range interval on that ray."""
    if report.depth_pasted is None or not report.depth_pasted.any():
        return 0, 0
    v, u = np.nonzero(report.depth_pasted)
    rays = pixel_rays(u, v, camera)
    inside, entry, exit_, _ = ray_envelope(np.asarray(camera.camera_offset, float), rays, g["p_v"], g["env_cam"])
    d = np.asarray(inp.depth, float)[v, u]
    k, tol = params.sigma_k, params.range_tol_m
    at = inside & (d >= entry - k * g["sigma_r"] - tol) & (d <= exit_ + k * g["sigma_r"] + tol)
    return int(inside.sum()), int(at.sum())


def estimate(inp, camera=FROZEN_F_CAMERA, lidar=FROZEN_F_LIDAR, params=EvidenceParams(), limits=ValidationLimits(),
             profile=FROZEN_F_PROFILE):
    """One evidence record for one decision. Pure: no RNG, no I/O, inputs are read-only."""
    report = validate(inp, camera, lidar, limits, profile)
    lidar_ok = report.lidar_usable
    near_beams = int((classify_lidar(inp.lidar, lidar, profile) == NEAR_CLIPPED).sum()) if lidar_ok else 0
    track_ok = report.track_usable
    if inp.camera_detected:
        if not report.camera_usable:
            return _record(UNKNOWN, ("CAMERA_FLAG_WITH_UNUSABLE_CAMERA_INPUT",), "NONE", "NOT_TESTABLE", inp, report,
                           near_beams=near_beams)
        if profile.distractors > 0:
            return _record(IDENTITY_AMBIGUOUS, ("CAMERA_FLAG_WITH_DISTRACTORS_PRESENT",), "CAMERA_FLAG", "UNVERIFIABLE",
                           inp, report, near_beams=near_beams)
        if not track_ok:
            return _record(TARGET_VISIBLE, ("CAMERA_FLAG_VISIBLE", "NO_USABLE_TRACK_FOOTPRINT_NOT_TESTABLE"),
                           "CAMERA_FLAG", "NOT_TESTABLE", inp, report, near_beams=near_beams)
        g = _belief_geometry(inp, camera, lidar, params)
        cam = _camera_evidence(inp, report, camera, profile, params, g)
        lid, near_beams = _lidar_evidence(inp, lidar, profile, params, g) if lidar_ok else (SensorRays(), 0)
        pasted = _pasted_in_footprint(inp, report, camera, params, g)
        args = dict(g=g, cam=cam, lid=lid, pasted=pasted, near_beams=near_beams)
        notes = [] if report.obstacle_depth_usable else ["CAMERA_OBSTACLE_DEPTH_UNAVAILABLE"]
        if cam.consistent or lid.consistent:
            notes.append("SURFACE_AT_TRACK_RANGE_NOT_IDENTITY_EVIDENCE")
        if pasted[1] > 0:
            return _record(TARGET_VISIBLE, ["CAMERA_FLAG_VISIBLE", "PASTED_DEPTH_AT_TRACK_RANGE"] + notes, "CAMERA_FLAG",
                           "SUPPORTED_BY_FOOTPRINT", inp, report, **args)
        if pasted[0] > 0:
            return _record(IDENTITY_AMBIGUOUS, ["CAMERA_FLAG_PASTED_DEPTH_OUTSIDE_TRACK_RANGE"] + notes, "CAMERA_FLAG",
                           "RANGE_CONTRADICTED", inp, report, **args)
        if report.depth_pasted is not None and not report.depth_pasted.any():
            notes.append("NO_PASTED_PIXELS_IN_IMAGE")   # the oracle flag normally comes with pasted pixels
        if cam.rays and cam.clear / cam.rays >= params.min_fraction and cam.foreground == 0:
            return _record(IDENTITY_AMBIGUOUS, ["CAMERA_FLAG_NOT_AT_TRACKED_POSITION"] + notes, "CAMERA_FLAG",
                           "CONTRADICTED", inp, report, **args)
        return _record(TARGET_VISIBLE, ["CAMERA_FLAG_VISIBLE", "TRACK_FOOTPRINT_NOT_TESTABLE"] + notes, "CAMERA_FLAG",
                       "NOT_TESTABLE", inp, report, **args)
    if not track_ok:
        return _record(UNKNOWN, ("NO_USABLE_TRACK",), "NONE", "NOT_APPLICABLE", inp, report, near_beams=near_beams)
    g = _belief_geometry(inp, camera, lidar, params)
    base = ("TRACKER_PREDICTION", "NOT_APPLICABLE", inp, report)
    if not report.camera_usable:
        return _record(UNKNOWN, ("CAMERA_INPUT_UNUSABLE",), *base, g=g, near_beams=near_beams)
    if not report.obstacle_depth_usable:
        return _record(UNKNOWN, ("CAMERA_OBSTACLE_DEPTH_UNAVAILABLE",), *base, g=g, near_beams=near_beams)
    if inp.track_age_s > params.max_belief_age_s:
        return _record(UNKNOWN, ("BELIEF_STALE",), *base, g=g, near_beams=near_beams)
    if g["sigma_r"] > params.max_range_sigma_m:
        return _record(UNKNOWN, ("BELIEF_TOO_DIFFUSE",), *base, g=g, near_beams=near_beams)
    if g["extra"] > params.max_envelope_extra_m:
        return _record(UNKNOWN, ("BELIEF_TOO_DIFFUSE_FOR_RAY_TEST",), *base, g=g, near_beams=near_beams)
    if g["fov_in"] == 0:
        return _record(OUT_OF_FOV, ("ALL_BELIEF_POINTS_OUTSIDE_CAMERA_IMAGE",), *base, g=g, near_beams=near_beams)
    if g["fov_in"] < g["fov_total"]:
        return _record(UNKNOWN, ("FOV_BOUNDARY_UNCERTAIN",), *base, g=g, near_beams=near_beams)
    if g["r_cam"] - camera.target_radius + params.sigma_k * g["sigma_r"] > camera.max_camera_range:
        return _record(UNKNOWN, ("BEYOND_DETECTION_RANGE",), *base, g=g, near_beams=near_beams)
    if math.pi * (camera.fx * camera.target_radius / max(g["r_cam"], 1e-6)) ** 2 < camera.min_pixels:
        return _record(UNKNOWN, ("BELOW_PIXEL_SUPPORT",), *base, g=g, near_beams=near_beams)
    cam = _camera_evidence(inp, report, camera, profile, params, g)
    reasons = []
    if lidar_ok:
        lid, near_beams = _lidar_evidence(inp, lidar, profile, params, g)
        if lid.rays == 0:
            reasons.append("LIDAR_NO_BEAM_COVERAGE")
    else:
        lid = SensorRays(); reasons.append("LIDAR_INPUT_UNUSABLE")
    if cam.rays == 0:
        reasons.append("CAMERA_NATIVE_NO_COVERAGE")
    pasted = _pasted_in_footprint(inp, report, camera, params, g)
    args = dict(g=g, cam=cam, lid=lid, pasted=pasted, near_beams=near_beams)
    frac = lambda n, d: d > 0 and n / d >= params.min_fraction
    cam_fg, lid_fg = frac(cam.foreground, cam.rays), frac(lid.foreground, lid.rays)
    cam_clear, lid_clear = frac(cam.clear, cam.rays), frac(lid.clear, lid.rays)
    if cam.near_clipped or lid.near_clipped:
        return _record(UNKNOWN, reasons + ["NEAR_CLIPPED_IN_LINE_OF_SIGHT"], *base, **args)
    if pasted[0]:
        return _record(IDENTITY_AMBIGUOUS, reasons + ["PASTED_TARGET_LIKE_PIXELS_IN_FOOTPRINT_WITHOUT_DETECTION"],
                       *base, **args)
    if cam_fg:
        if lid_clear:
            return _record(UNKNOWN, reasons + ["SENSOR_DISAGREEMENT_CAMERA_FOREGROUND_LIDAR_CLEAR"], *base, **args)
        extra = ["LIDAR_FOREGROUND_AGREES"] if lid_fg else []
        return _record(OCCLUDED, reasons + ["CAMERA_OBSTACLE_FOREGROUND"] + extra + ["BELIEF_ERROR_NOT_EXCLUDED"],
                       *base, **args)
    if lid_fg:
        if cam_clear:
            return _record(UNKNOWN, reasons + ["SENSOR_DISAGREEMENT_LIDAR_FOREGROUND_CAMERA_CLEAR"], *base, **args)
        if lid.foreground_extended:
            return _record(OCCLUDED, reasons + ["LIDAR_EXTENDED_FOREGROUND", "BELIEF_ERROR_NOT_EXCLUDED"], *base, **args)
        return _record(UNKNOWN, reasons + ["FOREGROUND_COMPATIBLE_WITH_TARGET_SURFACE", "BELIEF_ERROR_NOT_EXCLUDED"],
                       *base, **args)
    if cam.foreground or lid.foreground:
        return _record(UNKNOWN, reasons + ["PARTIAL_FOREGROUND_IN_ENVELOPE"], *base, **args)
    if cam.consistent or lid.consistent:
        return _record(IDENTITY_AMBIGUOUS, reasons + ["SURFACE_AT_EXPECTED_RANGE_WITHOUT_CAMERA_DETECTION"], *base, **args)
    if cam_clear and (lid.rays == 0 or lid_clear or lid.censored == lid.rays):
        return _record(DETECTION_MISS, reasons + ["CAMERA_LINE_OF_SIGHT_CLEAR_THROUGH_ENVELOPE", "BELIEF_ERROR_NOT_EXCLUDED"],
                       *base, **args)
    if cam.rays and cam.censored == cam.rays:
        reasons.append("CAMERA_NO_RETURN_BEYOND_OBSTACLE_FAR_PLANE")
    return _record(UNKNOWN, reasons + ["INCONCLUSIVE"], *base, **args)
