#!/usr/bin/env python3
"""EQ-1A — sensor geometry: independent references against the frozen production sensor code (CPU).

    python -B tools/audits/eq1_geometry_reference.py [--out docs/evidence/.../eq1a_geometry.json]

Validation only: production code is executed unmodified (loaded by path, no Isaac Gym); the
references below never import it. Every expected value comes from analytic geometry: pinhole
tangent projection, explicit SE(3) transforms, analytic ray/sphere and ray/box intersections and
dense sub-pixel sampling of the target-alone silhouette. A test that reproduces a known
counterexample records the production verdict FAIL; it is not a PASS for that code. Each case also
records the verdict of a *proposed* correction (also reference code, never wired into production).

Gates (fixed before execution, from the EQ-1 plan): projected direction error <= 0.5 pixel;
transformed metric range error <= 1e-4 m on exact fixtures; target-radius conventions identical or
explicitly transformed; target-absent negative fixtures must not produce a target-specific accept;
full / partial / centre-only / FOV / scale labels must agree with the independent labels, and zero
rendered pixels alone never supplies full-occlusion ground truth.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eq1_common as common  # noqa: E402

PIXEL_GATE = 0.5
RANGE_GATE_M = 1e-4
CAM_W, CAM_H = 160, 90
HFOV, VFOV = math.radians(87.0), math.radians(58.0)
CX, CY = (CAM_W - 1) / 2.0, (CAM_H - 1) / 2.0
FX = CAM_W / (2.0 * math.tan(HFOV / 2.0))
FY = CAM_H / (2.0 * math.tan(VFOV / 2.0))
CAM_T = np.array([0.10, 0.0, 0.03])
CAMERA_RADIUS, LIDAR_RADIUS = 0.15, 0.20
LIDAR_RANGE, CAMERA_FAR = 12.0, 10.0
MIN_PIXELS = 2


# ------------------------------------------------------------------ independent references
def pinhole_column(theta):
    """Exact image column of a horizontal bearing theta (left positive) for the pinhole camera."""
    return CX - FX * math.tan(theta)


def column_bearing(u):
    return math.atan((CX - u) / FX)


def pixel_ray(u, v):
    d = np.array([1.0, -(u - CX) / FX, -(v - CY) / FY])
    return d / np.linalg.norm(d)


def ray_sphere(origin, direction, centre, radius):
    oc = origin - centre
    b = float(direction @ oc)
    c = float(oc @ oc) - radius * radius
    disc = b * b - c
    if disc < 0:
        return math.inf
    t = -b - math.sqrt(disc)
    return t if t > 0 else math.inf


def ray_box(origin, direction, lo, hi):
    t0, t1 = -math.inf, math.inf
    for a in range(3):
        if abs(direction[a]) < 1e-15:
            if origin[a] < lo[a] or origin[a] > hi[a]:
                return math.inf
            continue
        ta, tb = (lo[a] - origin[a]) / direction[a], (hi[a] - origin[a]) / direction[a]
        t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
    return t0 if t1 >= max(t0, 0.0) and t0 > 0 else math.inf


# ------------------------------------------------------------------ production stand-in
class _FusionStandIn:
    """Exactly the attributes production `_fuse_static_and_extract_obstacles` reads (1 env, CPU)."""

    def __init__(self, perception):
        self.num_envs = 1
        self.device = torch.device("cpu")
        self._lidar_angles = perception.lidar_bin_bearings(self.device)
        self.lidar_max_range = LIDAR_RANGE
        self.camera_obstacle_max_range = CAMERA_FAR
        self.hfov = HFOV
        self.width = CAM_W
        self.last_target_like = torch.zeros(1, perception.VBEAMS, perception.HBEAMS, dtype=torch.bool)
        self.last_scan_nearest = None
        self._suppress_bins = 0
        self._token_bearing_mask = None


def production_fused_scan(perception, raw_depth):
    """Run production fusion on a camera depth image with an empty LiDAR scan; returns [V, H] metres."""
    self = _FusionStandIn(perception)
    lidar = torch.full((1, perception.VBEAMS * perception.HBEAMS), LIDAR_RANGE)
    no_target = torch.zeros(1, CAM_H, CAM_W, dtype=torch.bool)
    zero = torch.zeros(1)
    static, _ = perception.NavRLPerceptionModule._fuse_static_and_extract_obstacles(
        self, lidar, torch.as_tensor(raw_depth, dtype=torch.float32).view(1, CAM_H, CAM_W), no_target,
        zero, zero, torch.zeros(1, dtype=torch.bool), drone_vel_w=torch.zeros(1, 3),
        vehicle_quat=torch.tensor([[0.0, 0.0, 0.0, 1.0]]))
    return (static.view(perception.VBEAMS, perception.HBEAMS) * LIDAR_RANGE).double().numpy(), \
        self._lidar_angles.double().numpy()


# ------------------------------------------------------------------ A1 column mapping
def case_column_mapping(perception):
    """Encode each column's index in its depth; read which column production fuses into each beam."""
    depth = np.tile(1.0 + 0.05 * np.arange(CAM_W), (CAM_H, 1))
    scan, bearings = production_fused_scan(perception, depth)
    rows = []
    for j, theta in enumerate(bearings):
        if abs(theta) > HFOV / 2:
            continue
        u_prod = round((scan[0, j] - 1.0) / 0.05)
        u_true = pinhole_column(theta)
        u_fix = int(round(u_true))
        rows.append({"beam_deg": round(math.degrees(theta), 4), "u_pinhole": u_true, "u_production": u_prod,
                     "pixel_error": abs(u_prod - u_true), "bearing_error_deg": abs(math.degrees(column_bearing(u_prod) - theta)),
                     "u_proposed": u_fix, "proposed_pixel_error": abs(u_fix - u_true)})
    worst = max(rows, key=lambda r: r["pixel_error"])
    at20 = next(r for r in rows if abs(r["beam_deg"] - 20.0) < 1e-6)
    return {"name": "A1 camera-column to LiDAR-bearing mapping", "gate": "pixel error <= %.1f" % PIXEL_GATE,
            "fx": FX, "cx": CX, "beams_in_fov": len(rows), "case_20deg": at20, "worst": worst,
            "production": common.verdict(worst["pixel_error"] <= PIXEL_GATE,
                                         "linear-in-angle map; tangent spacing ignored"),
            "proposed_correction": common.verdict(max(r["proposed_pixel_error"] for r in rows) <= PIXEL_GATE,
                                                  "u = round(cx - fx*tan(theta)); residual is column quantisation"),
            "rows": rows}


# ------------------------------------------------------------------ A2 common origin
def plane_depth(x_plane):
    """Camera radial depth image of the wall x = x_plane (body frame); obstacle depth is t along the unit ray."""
    depth = np.full((CAM_H, CAM_W), CAMERA_FAR)
    for v in range(CAM_H):
        for u in range(CAM_W):
            d = pixel_ray(u, v)
            t = (x_plane - CAM_T[0]) / d[0]
            if 0 < t < CAMERA_FAR:
                depth[v, u] = t
    return depth


def corrected_body_range(depth, theta, row):
    """Proposed correction (reference code only): SE(3)-transform camera hits into the body frame and
    intersect the body-origin beam with the chord between the two hits whose body bearings bracket it.
    Exact for piecewise-planar surfaces; the residual on curved surfaces is the chord sagitta."""
    pts = []
    for u in range(CAM_W):
        t = depth[row, u]
        if t >= CAMERA_FAR:
            pts.append(None)
            continue
        p = CAM_T + float(t) * pixel_ray(u, row)
        pts.append(p[:2])
    beam = np.array([math.cos(theta), math.sin(theta)])
    for a, b in zip(pts[:-1], pts[1:]):
        if a is None or b is None:
            continue
        ba, bb = math.atan2(a[1], a[0]), math.atan2(b[1], b[0])
        if min(ba, bb) <= theta <= max(ba, bb):
            # Solve s * beam = a + w (b - a) for s (range) and w (chord fraction).
            m = np.array([[beam[0], a[0] - b[0]], [beam[1], a[1] - b[1]]])
            s_range, _ = np.linalg.solve(m, a)
            return float(s_range)
    return None


def case_common_origin(perception, x_plane=5.0):
    depth = plane_depth(x_plane).astype(np.float32)  # the renderer delivers float32 depth
    scan, bearings = production_fused_scan(perception, depth)
    row = int(round(CY))
    rows = []
    for j, theta in enumerate(bearings):
        if abs(theta) > math.radians(40):
            continue
        true_range = x_plane / math.cos(theta)  # horizontal beam from the body origin to the wall
        proposed = corrected_body_range(depth, theta, row)
        rows.append({"beam_deg": round(math.degrees(theta), 4), "true_body_range_m": true_range,
                     "production_range_m": float(scan[0, j]), "error_m": float(scan[0, j]) - true_range,
                     "proposed_range_m": proposed, "proposed_error_m": None if proposed is None else proposed - true_range})
    on_axis = next(r for r in rows if abs(r["beam_deg"]) < 1e-9)
    worst = max(rows, key=lambda r: abs(r["error_m"]))
    proposed_worst = max(abs(r["proposed_error_m"]) for r in rows)
    return {"name": "A2 camera range reused as a body-origin LiDAR range (front-plane fixture)",
            "gate": "range error <= %g m" % RANGE_GATE_M, "plane_x_m": x_plane, "on_axis": on_axis, "worst": worst,
            "proposed_worst_error_m": proposed_worst,
            "production": common.verdict(abs(worst["error_m"]) <= RANGE_GATE_M,
                                         "camera translation not applied; off-axis also inherits the A1 bearing error"),
            "proposed_correction": common.verdict(proposed_worst <= RANGE_GATE_M,
                                                  "p_body = t_cam + t * ray(u, v); intersect the beam with the chord of bracketing hits"),
            "rows": rows}


# ------------------------------------------------------------------ A3 vertical aggregation
def case_vertical(perception):
    """An obstacle visible only in the top camera rows: production fuses it into every LiDAR row."""
    depth = np.full((CAM_H, CAM_W), CAMERA_FAR)
    depth[:10, :] = 3.0  # an overhang seen only above the horizon
    scan, bearings = production_fused_scan(perception, depth)
    forward = int(np.argmin(np.abs(bearings)))
    rows_fused = scan[:, forward].tolist()
    false_clear = any(r > 3.0 + 1e-6 for r in rows_fused)
    return {"name": "A3 vertical aggregation (column minimum over all rows)", "rows_fused_m": rows_fused,
            "production_conservative": common.verdict(not false_clear, "min over rows never reports a false clear"),
            "production_metric_per_elevation": {"status": "FAIL", "note": "all LiDAR elevations receive the same camera minimum; elevation is not preserved"},
            "interpretation": "acceptable only as an explicitly conservative 2D envelope"}


# ------------------------------------------------------------------ A4 radius convention
def camera_mean_depth_centre(distance):
    """Production's camera range estimator on the analytic silhouette: mean pixel-centre surface depth
    over the target mask plus target_radius (navrl_perception: surface_range = sum(depth*mask)/count)."""
    target = CAM_T + np.array([distance, 0.0, 0.0])
    hits = []
    for v in range(CAM_H):
        for u in range(CAM_W):
            t = ray_sphere(CAM_T, pixel_ray(u, v), target, CAMERA_RADIUS)
            if not math.isinf(t):
                hits.append(t)
    return (float(np.mean(hits)) + CAMERA_RADIUS if hits else None), len(hits)


def case_radius(distance=6.0):
    """LiDAR: a beam through an aligned clear target returns d - 0.20 (the LiDAR proxy sphere);
    association adds the camera's 0.15. Camera: mean surface depth over the mask plus 0.15."""
    surface = distance - LIDAR_RADIUS
    centre_estimate = surface + CAMERA_RADIUS
    camera_rows = []
    for d in (3.0, 5.0, 8.0, 10.0):
        est, n = camera_mean_depth_centre(d)
        camera_rows.append({"distance_from_camera_m": d, "mask_pixels": n, "centre_estimate_m": est,
                            "bias_m": None if est is None else est - d})
    return {"name": "A4 camera/LiDAR target-radius and centre-range convention", "target_centre_m": distance,
            "lidar_surface_m": surface, "association_centre_m": centre_estimate, "bias_m": centre_estimate - distance,
            "camera_centre_range": camera_rows, "camera_bias_limit_m": CAMERA_RADIUS / 3.0,
            "production": common.verdict(abs(centre_estimate - distance) <= RANGE_GATE_M,
                                         "LiDAR proxy sphere 0.20 m (navrl_lidar_config.target_radius) vs perception 0.15 m"),
            "production_camera_estimator": common.verdict(all(abs(r["bias_m"]) <= RANGE_GATE_M for r in camera_rows),
                                                          "mean surface depth + r is biased towards +r/3 as the mask grows"),
            "proposed_correction": common.verdict(abs((surface + LIDAR_RADIUS) - distance) <= RANGE_GATE_M,
                                                  "add the LiDAR proxy's own radius, or use one radius in both sensors; "
                                                  "camera: invert the sphere geometry per pixel or use the minimum depth"),
            "note": "camera and LiDAR centre estimates of the same target disagree by up to about 0.10 m"}


# ------------------------------------------------------------------ A5 occlusion classes
def project(p_cam):
    """Pinhole image coordinates of a point given relative to the camera origin (x forward, y left, z up)."""
    return CX - FX * p_cam[1] / p_cam[0], CY - FY * p_cam[2] / p_cam[0]


def silhouette(target, occluders, sub=8, eye=CAM_T):
    """Target-alone and visible support of the r = 0.15 m target: pixel-centre counts (what the
    renderer samples) and sub-pixel area (sub x sub samples per pixel) for the analytic silhouette."""
    rel = target - eye
    dist = float(np.linalg.norm(rel))
    in_fov = rel[0] > 0
    if in_fov:
        uc, vc = project(rel)
        in_fov = -0.5 <= uc <= CAM_W - 0.5 and -0.5 <= vc <= CAM_H - 0.5
    centre_blocked = any(ray_box(eye, rel / dist, lo, hi) < dist for lo, hi in occluders)
    out = {"in_fov": bool(in_fov), "alone_pixels": 0, "visible_pixels": 0, "alone_area_px": 0.0,
           "visible_area_px": 0.0, "centre_blocked": bool(centre_blocked), "distance_m": dist}
    if rel[0] <= CAMERA_RADIUS:
        return out
    uc, vc = project(rel)
    half = FX * CAMERA_RADIUS / (rel[0] - CAMERA_RADIUS) * (1.0 + (rel[1] / rel[0]) ** 2) + 3
    for v in range(max(0, int(vc - half)), min(CAM_H - 1, int(vc + half) + 1) + 1):
        for u in range(max(0, int(uc - half)), min(CAM_W - 1, int(uc + half) + 1) + 1):
            for su in range(sub):
                for sv in range(sub):
                    d = pixel_ray(u - 0.5 + (su + 0.5) / sub, v - 0.5 + (sv + 0.5) / sub)
                    t = ray_sphere(eye, d, target, CAMERA_RADIUS)
                    if not math.isinf(t):
                        out["alone_area_px"] += 1.0 / sub ** 2
                        out["visible_area_px"] += (not any(ray_box(eye, d, lo, hi) < t for lo, hi in occluders)) / sub ** 2
            d = pixel_ray(u, v)  # the renderer's single pixel-centre ray
            t = ray_sphere(eye, d, target, CAMERA_RADIUS)
            if not math.isinf(t):
                out["alone_pixels"] += 1
                out["visible_pixels"] += not any(ray_box(eye, d, lo, hi) < t for lo, hi in occluders)
    return out


def reference_label(s):
    """Independent label from the analytic silhouette (visible area fraction, centre ray, target-alone support)."""
    if not s["in_fov"]:
        return "FOV_EXIT"
    frac = s["visible_area_px"] / s["alone_area_px"] if s["alone_area_px"] > 0 else 0.0
    if frac <= 1e-9:
        return "FULL"
    if s["alone_pixels"] < MIN_PIXELS:
        return "SCALE_LOSS"  # undetectable even with no occluder (perception min_target_pixels = 2)
    if frac < 0.999:
        return "CENTRE_ONLY" if s["centre_blocked"] else "PARTIAL"
    return "VISIBLE"


def current_label(s, rim_full):
    """The OCC indicator set (occlusion_pilot_v2 'Indicators') read as a classifier."""
    if not s["in_fov"]:
        return "FOV_EXIT"            # out_of_view
    if rim_full:
        return "FULL"                # geometric_full_in_view: centre and all 8 rim rays blocked
    if s["visible_pixels"] == 0:
        return "FULL"                # rendered_full_in_view: zero rendered target pixels in view
    if s["centre_blocked"]:
        return "CENTRE_ONLY"         # center_occlusion_in_view ('partial' in the G0 scene check)
    return "VISIBLE"


def box(cx, cy, hx, hy, z0=-1.5, z1=1.5):
    """Upright bar (body-frame box). Bars span the flight altitude, as the 3 m bars_h3 pool does."""
    return (np.array([cx - hx, cy - hy, z0]), np.array([cx + hx, cy + hy, z1]))


Z = CAM_T[2]  # target at camera height unless stated
OCCLUSION_CASES = [
    ("clear 3 m", (3.0, 0, Z), []),
    ("clear 10 m", (10.0, 0, Z), []),
    ("clear 20 m, centre on a pixel corner (scale)", (20.0, 0, Z), []),
    ("clear 10 m at 40 deg", (10 * math.cos(math.radians(40)), 10 * math.sin(math.radians(40)), Z), []),
    ("FOV exit 50 deg", (10 * math.cos(math.radians(50)), 10 * math.sin(math.radians(50)), Z), []),
    ("full: wide bar at 5 m, target 10 m", (10.0, 0, Z), [box(5.0, 0.0, 0.2, 0.4)]),
    ("partial: bar over one side, centre clear", (4.0, 0, Z), [box(2.0, 0.09, 0.05, 0.05)]),
    ("centre-only: thin bar over the centre", (3.0, 0, Z), [box(1.5, 0.0, 0.02, 0.012)]),
    ("rim-proxy gap: two bars block all 9 rays, a strip stays visible", (3.0, 0, Z),
     [box(2.6, -0.12, 0.025, 0.04), box(2.6, 0.075, 0.025, 0.085)]),
    ("occluder behind the target", (5.0, 0, Z), [box(7.0, 0.0, 0.3, 0.6)]),
    ("full at 20 m (scale and occluder)", (20.0, 0, Z), [box(10.0, 0.0, 0.2, 0.4)]),
]


def occlusion_fixtures():
    from motar_eval import occlusion_geometry as og

    common.assert_in_tree(og)
    rows = []
    for name, target, occ in OCCLUSION_CASES:
        target = np.array(target, dtype=np.float64)
        s = silhouette(target, occ)
        centres = np.array([(lo + hi) / 2 for lo, hi in occ]).reshape(-1, 3)
        halves = np.array([(hi - lo) / 2 for lo, hi in occ]).reshape(-1, 3)
        los = og.line_of_sight(CAM_T[None], target[None], centres, halves, radius=CAMERA_RADIUS, rim_count=8)
        rim_full = bool(los["geometric_full_occlusion"][0])
        ref, cur = reference_label(s), current_label(s, rim_full)
        rows.append({"case": name, "reference": ref, "current_indicators": cur, "agree": ref == cur,
                     "camera_detectable": s["visible_pixels"] >= MIN_PIXELS, "rim_full": rim_full,
                     "rim_blocked_fraction": float(los["rim_blocked_fraction"][0]),
                     "offline_centre_blocked": bool(los["center_blocked"][0]), **s})
    return rows


def scale_loss_band(step=0.25, offsets=9):
    """Clear on-axis target: the range where pixel-centre support first can fall below 2, and where it
    always does, over a grid of sub-pixel placements of the target centre."""
    first_possible = always = None
    table = []
    for d in np.arange(4.0, 30.0 + 1e-9, step):
        counts = []
        for i in range(offsets):
            for j in range(offsets):
                du, dv = (i + 0.5) / offsets - 0.5, (j + 0.5) / offsets - 0.5
                # Place the centre at pixel (80 + du, 45 + dv): back-project to a point at range d.
                ray = pixel_ray(80 + du, 45 + dv)
                counts.append(silhouette(CAM_T + d * ray, [], sub=1)["alone_pixels"])
        lo, hi = min(counts), max(counts)
        table.append({"range_m": float(d), "min_pixels": lo, "max_pixels": hi})
        if first_possible is None and lo < MIN_PIXELS:
            first_possible = float(d)
        if always is None and hi < MIN_PIXELS:
            always = float(d)
    return {"min_target_pixels": MIN_PIXELS, "first_range_with_possible_scale_loss_m": first_possible,
            "first_range_with_certain_scale_loss_m": always, "table": table}


def case_occlusion_classes():
    rows = occlusion_fixtures()
    disagreements = [r["case"] for r in rows if not r["agree"]]
    zero_px_not_occluded = [r["case"] for r in rows if r["in_fov"] and r["visible_pixels"] == 0 and r["reference"] != "FULL"]
    return {"name": "A5 full / partial / centre-only / FOV / scale classification",
            "gate": "current indicator classes equal the independent silhouette labels; zero pixels never alone imply FULL",
            "rows": rows, "disagreements": disagreements,
            "in_view_zero_pixels_without_occlusion": zero_px_not_occluded, "scale_loss": scale_loss_band(),
            "production_indicators": common.verdict(not disagreements and not zero_px_not_occluded,
                                                    "zero pixels conflate scale loss with occlusion; centre-visible partial "
                                                    "occlusion reads as clear; the 9-ray proxy can call a visible target full"),
            "proposed_correction": {"status": "PASS_BY_CONSTRUCTION",
                                    "note": "label from target-alone support (render the target without occluders) and visible "
                                            "support; SCALE_LOSS when target-alone support < min_target_pixels"}}


# ------------------------------------------------------------------ A6 identity
def case_false_association(perception):
    """Target absent; a bar at the predicted range on the predicted bearing. Production accepts it as the target."""
    tracker = perception.BatchedConstantVelocityTracker(1, torch.device("cpu"), 0.1, 5.0)
    tracker.active[:] = True
    tracker.state[0, :3] = torch.tensor([5.0, 0.0, 1.0])
    tracker.cov[0] = torch.eye(6) * 0.01
    tracker.age[:] = 1.0

    class S:
        pass

    s = S()
    s.tracker, s.target_radius, s.num_envs, s.device = tracker, CAMERA_RADIUS, 1, torch.device("cpu")
    s._lidar_angles = perception.lidar_bin_bearings(s.device)
    s.lidar_max_range, s.lidar_assoc_gate_m = LIDAR_RANGE, 0.0
    s.lidar_range_only_update, s.lidar_silent_correct = False, False
    results = {}
    for label, bar_surface in (("bar at predicted range (no target)", 4.85), ("bar 1.5 m beyond prediction", 6.35)):
        tracker.state[0, :3] = torch.tensor([5.0, 0.0, 1.0])
        tracker.age[:] = 1.0
        scan = torch.full((1, perception.VBEAMS, perception.HBEAMS), LIDAR_RANGE)
        j = int(torch.argmin(s._lidar_angles.abs()))
        scan[0, :, j] = bar_surface
        valid, conf, measured, bearing = perception.NavRLPerceptionModule._associate_lidar_target(
            s, scan.view(1, -1), torch.tensor([[0.0, 0.0, 1.0]]), torch.tensor([[0.0, 0.0, 0.0, 1.0]]),
            torch.zeros(1, dtype=torch.bool))
        results[label] = {"accepted_as_target": bool(valid[0]), "age_after": float(tracker.age[0]),
                          "measured_surface_m": float(measured[0])}
    false_accept = results["bar at predicted range (no target)"]["accepted_as_target"]
    return {"name": "A6 LiDAR association identity (target-absent negative fixture)", "fixtures": results,
            "production_identity": common.verdict(not false_accept,
                                                  "range/bearing gate only; a compatible bar resets the track age as the target"),
            "negative_control_rejected": not results["bar 1.5 m beyond prediction"]["accepted_as_target"],
            "proposed_correction": {"status": "NOT_TESTED",
                                    "note": "persistent identity needs appearance/re-ID evidence; otherwise label 'range association' and never claim identity"}}


# ------------------------------------------------------------------ A7 camera target measurement
CAMERA_TARGETS = [("on-axis 5 m", (5.0, 0.0, 0.0)), ("20 deg left, 6 m", (6 * math.cos(math.radians(20)),
                  6 * math.sin(math.radians(20)), 0.0)), ("15 deg right, 0.5 m up, 4 m", (4 * math.cos(math.radians(15)),
                  -4 * math.sin(math.radians(15)), 0.5)), ("0.5 m below, 3 m", (3.0, 0.0, -0.5))]


def render_target(centre_vehicle):
    """Analytic RGB-D frame of the red r = 0.15 m target alone (pixel-centre rays, radial depth)."""
    rgb = np.full((3, CAM_H, CAM_W), 0.5, dtype=np.float32)
    depth = np.full((CAM_H, CAM_W), 15.0, dtype=np.float32)
    for v in range(CAM_H):
        for u in range(CAM_W):
            t = ray_sphere(CAM_T, pixel_ray(u, v), np.asarray(centre_vehicle, dtype=np.float64), CAMERA_RADIUS)
            if not math.isinf(t):
                rgb[:, v, u] = (1.0, 0.0, 0.0)
                depth[v, u] = t
    return rgb, depth


def case_camera_measurement():
    """Production _detect_rgbd on analytic frames: intrinsics, camera extrinsic and vertical sign."""
    m, _, _ = common.build_perception(1)
    drone = torch.tensor([[10.0, 10.0, 1.0]])
    quat = torch.tensor([[0.0, 0.0, 0.0, 1.0]])
    rows = []
    for name, p in CAMERA_TARGETS:
        p = np.asarray(p, dtype=np.float64)
        rgb, depth = render_target(p)
        meas, surface, bearing, visible, conf, pixels = m._detect_rgbd(
            torch.from_numpy(rgb)[None], torch.from_numpy(depth)[None], False, drone_pos_w=drone, vehicle_quat=quat)
        mv = meas[0].double().numpy()
        true_dir = (p - CAM_T) / np.linalg.norm(p - CAM_T)
        meas_dir = (mv - CAM_T) / np.linalg.norm(mv - CAM_T)
        ang = math.acos(min(1.0, float(true_dir @ meas_dir)))
        # Discriminating power: the same measured ray read from the body origin (offset ignored).
        body_dir = p / np.linalg.norm(p)
        ang_no_offset = math.acos(min(1.0, float(body_dir @ meas_dir)))
        rows.append({"case": name, "visible": bool(visible[0]), "pixels": int((depth < 15.0).sum()),
                     "true_vehicle_m": p.tolist(), "measured_vehicle_m": mv.tolist(),
                     "direction_error_px": ang * FX, "range_error_m": float(np.linalg.norm(mv - CAM_T) - np.linalg.norm(p - CAM_T)),
                     "direction_error_px_if_offset_ignored": ang_no_offset * FX})
    return {"name": "A7 camera target measurement (production _detect_rgbd on analytic target-alone frames)",
            "gate": "direction <= %.1f px; centre range <= %g m" % (PIXEL_GATE, RANGE_GATE_M), "rows": rows,
            "production_direction": common.verdict(all(r["direction_error_px"] <= PIXEL_GATE for r in rows),
                                                   "pinhole intrinsics, camera_offset and vertical sign consistent"),
            "production_range": common.verdict(all(abs(r["range_error_m"]) <= RANGE_GATE_M for r in rows),
                                               "mean surface depth + r over-estimates the centre range (A4)")}


def run():
    with common.production(common.receipt_env()):
        return _run(common.load_production("aerial_gym.task.navrl_task.navrl_perception"))


def _run(perception):
    cases = [case_column_mapping(perception), case_common_origin(perception), case_vertical(perception),
             case_radius(), case_occlusion_classes(), case_false_association(perception), case_camera_measurement()]
    return {"schema": "motar.eq1a-geometry.v1", "package": "EQ-1A sensor geometry (CPU, validation only)",
            "source": common.source_binding(["aerial_gym/task/navrl_task/navrl_perception.py",
                                             "aerial_gym/task/navrl_task/navrl_detector.py",
                                             "aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py",
                                             "motar_eval/occlusion_geometry.py"]),
            "environment": "OCC-P2 receipt" if common.RECEIPT.exists() else "Policy F LiDAR grid only",
            "camera": {"width": CAM_W, "height": CAM_H, "hfov_deg": 87.0, "vfov_deg": 58.0, "fx": FX, "fy": FY,
                       "cx": CX, "cy": CY, "translation_m": CAM_T.tolist()},
            "cases": cases}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    record = run()
    if args.out:
        common.write_record(args.out, record)
    for c in record["cases"]:
        verdicts = {k: v["status"] for k, v in c.items() if k.startswith("production") and isinstance(v, dict)}
        print("%-72s %s | proposed %s" % (c["name"], verdicts, (c.get("proposed_correction") or {}).get("status", "-")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
