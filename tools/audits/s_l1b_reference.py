#!/usr/bin/env python3
"""Independent float64 reference for S-L1B camera-LiDAR calibration (CPU, NumPy only).

Never imports production sensor code. Ray/box/sphere first hits come from the committed S-L1A
reference. Frames follow the preregistration: LiDAR origin at the yaw-only vehicle origin
(+x forward, +y left, +z up); camera point p = R d r_hat + t; azimuth bins decrease from +pi in
2pi/H steps; elevation rows start at +20 deg and step by -10 deg.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s_l1a_reference as ref  # noqa: E402

CONTRACT = "MOTAR_S_L1B_FLOAT64_REFERENCE_V1"
CAMERA_TRANSLATION = (0.10, 0.0, 0.03)
CAMERA_TARGET_RADIUS_M = 0.15
LIDAR_PROXY_RADIUS_M = 0.20
GRAZING_DEG = 80.0


def rotation_matrix(roll=0., pitch=0., yaw=0.):
    """Body-to-parent rotation R = Rz(yaw) Ry(pitch) Rx(roll), radians."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _quat_from_matrix(m):
    """xyzw quaternion of a proper rotation matrix (for ref.rotate_rays)."""
    m = np.asarray(m, dtype=np.float64)
    w = math.sqrt(max(0., 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    if w > 1e-8:
        q = np.array([(m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w])
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(0., 1 + m[i, i] - m[j, j] - m[k, k])) * 2
        q = np.zeros(4)
        q[i], q[3] = s / 4, (m[k, j] - m[j, k]) / s
        q[j], q[k] = (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return q / np.linalg.norm(q)


def camera_points(depth, *, rotation=np.eye(3), translation=CAMERA_TRANSLATION, hfov_deg=87., vfov_deg=58.,
                  camera_far_m=10., blank=None):
    """Per-pixel LiDAR-frame points of delivered slant depth: p = R d r_hat + t."""
    d = np.asarray(depth, dtype=np.float64)
    if d.ndim == 2:
        d = d[None]
    if d.ndim != 3:
        raise ValueError("depth must be [H,W] or [N,H,W]")
    r = np.asarray(rotation, dtype=np.float64)
    t = np.asarray(translation, dtype=np.float64)
    if r.shape != (3, 3) or not np.allclose(r @ r.T, np.eye(3), atol=1e-9) or np.linalg.det(r) < 0 or t.shape != (3,):
        raise ValueError("rotation must be proper [3,3]; translation [3]")
    n, h, w = d.shape
    rays = ref.normalized_pixel_rays(w, h, hfov_deg, vfov_deg)
    valid = np.isfinite(d) & (d > 0) & (d < camera_far_m - 1e-3)
    if blank is not None:
        valid &= ~np.broadcast_to(np.asarray(blank, dtype=bool), d.shape)
    safe = np.where(valid, d, 0.)
    p = np.einsum("ij,nhwj->nhwi", r, safe[..., None] * rays[None]) + t
    rho = np.linalg.norm(p, axis=-1)
    az = np.arctan2(p[..., 1], p[..., 0])
    el = np.arctan2(p[..., 2], np.hypot(p[..., 0], p[..., 1]))
    return {"points": p, "rho": rho, "az": az, "el": el, "valid": valid}


def bin_index(az, el, *, hbeams=72, vbeams=4, el_max_deg=20., el_min_deg=-10., az_shift=0., el_shift=0.):
    step = 2 * math.pi / hbeams
    j = np.mod(np.rint((math.pi - (np.asarray(az) + az_shift)) / step), hbeams).astype(np.int64)
    el_step = math.radians(el_max_deg - el_min_deg) / (vbeams - 1)
    e = np.rint((math.radians(el_max_deg) - (np.asarray(el) + el_shift)) / el_step).astype(np.int64)
    return j, e


def lidar_bins(cam, *, clamp, hbeams=72, vbeams=4, lidar_far_m=12., az_shift=0., el_shift=0.):
    """Minimum LiDAR-frame range per (row, azimuth) bin; empty bins are lidar_far_m."""
    rho, valid = cam["rho"], cam["valid"]
    j, e = bin_index(cam["az"], cam["el"], hbeams=hbeams, vbeams=vbeams, az_shift=az_shift, el_shift=el_shift)
    keep = valid & (rho < lidar_far_m)
    if clamp:
        e = np.clip(e, 0, vbeams - 1)
    else:
        keep &= (e >= 0) & (e <= vbeams - 1)
    n = rho.shape[0]
    out = np.full((n, vbeams, hbeams), float(lidar_far_m))
    env = np.broadcast_to(np.arange(n).reshape((n,) + (1,) * (rho.ndim - 1)), rho.shape)
    np.minimum.at(out, (env[keep], e[keep], j[keep]), rho[keep])
    return out


def robust_bins(cam, *, clamp, eps=1e-5, **kw):
    """Bins whose value is unchanged by every +-eps azimuth/elevation boundary shift."""
    base = lidar_bins(cam, clamp=clamp, **kw)
    robust = np.ones(base.shape, dtype=bool)
    for da in (-eps, 0., eps):
        for de in (-eps, 0., eps):
            if da == de == 0:
                continue
            robust &= lidar_bins(cam, clamp=clamp, az_shift=da, el_shift=de, **kw) == base
    return base, robust


def _box_normal(points, centers, half, index):
    """Outward face normal of the box face containing each hit point."""
    normals = np.zeros(points.shape)
    hit = index >= 0
    c, h = centers[index[hit]], half[index[hit]]
    rel = (points[hit] - c) / np.where(h > 0, h, 1.)
    axis = np.argmax(np.abs(rel), axis=-1)
    sel = np.zeros(rel.shape)
    sel[np.arange(len(axis)), axis] = np.sign(rel[np.arange(len(axis)), axis])
    normals[hit] = sel
    return normals


def true_surface(box_centers=(), box_half_extents=(), *, sphere_center=None, sphere_radius=CAMERA_TARGET_RADIUS_M,
                 rotation=np.eye(3), translation=CAMERA_TRANSLATION, width=160, height=90, hfov_deg=87.,
                 vfov_deg=58., far_m=10.):
    """Analytic first-hit surface point of every pixel ray, in the LiDAR (body-origin) frame.

    Body origin at (0,0,0), yaw 0. Includes pixel robustness (S-L1A rule) and grazing (> 80 deg).
    """
    t = np.asarray(translation, dtype=np.float64)
    q = _quat_from_matrix(rotation)
    out = ref.render_reference(box_centers, box_half_extents, sphere_center=sphere_center, sphere_radius=sphere_radius,
                               width=width, height=height, hfov_deg=hfov_deg, vfov_deg=vfov_deg, origin=tuple(t),
                               orientation_xyzw=tuple(q), far_m=far_m)
    rays = ref.rotate_rays(ref.normalized_pixel_rays(width, height, hfov_deg, vfov_deg), q)
    depth = out["depth_m"]
    p = t + depth[..., None] * rays
    centers, half = ref._boxes(box_centers, box_half_extents)
    box = out["hit_class"] == ref.BOX
    index = np.where(box, out["hit_index"], -1)
    normal = _box_normal(p, centers, half, index) if len(centers) else np.zeros(p.shape)
    cos_inc = np.abs(np.sum(normal * rays, axis=-1))
    grazing = box & (cos_inc < math.cos(math.radians(GRAZING_DEG)))
    return {"depth_m": depth, "hit_class": out["hit_class"], "robust_mask": out["robust_mask"], "grazing": grazing,
            "rho": np.linalg.norm(p, axis=-1), "az": np.arctan2(p[..., 1], p[..., 0]),
            "el": np.arctan2(p[..., 2], np.hypot(p[..., 0], p[..., 1])), "box": box}


def common_frame_carve_masks(lidar_ranges, lidar_angles, camera_depth, camera_candidates, visible, target_bearing, *,
                             rotation=np.eye(3), translation=CAMERA_TRANSLATION, hfov_deg=87., vfov_deg=58.,
                             camera_radius=CAMERA_TARGET_RADIUS_M, lidar_radius=LIDAR_PROXY_RADIUS_M,
                             min_target_pixels=2, lidar_far_m=12., camera_far_m=20., max_angle_deg=15.,
                             upper_agreement_m=.55, lidar_elevations_deg=(20., 10., 0., -10.)):
    """S-L1B carve: S-L1A support/camera rules, LiDAR window from the body-range anchor.

    Amendment 1: a LiDAR row is eligible only inside [min, max] body-frame elevation of the valid
    candidate pixels.
    """
    base = ref.carve_masks(lidar_ranges, lidar_angles, camera_depth, camera_candidates, visible, target_bearing,
                           target_radius=camera_radius, min_target_pixels=min_target_pixels, lidar_far_m=lidar_far_m,
                           camera_far_m=camera_far_m, max_angle_deg=max_angle_deg, upper_agreement_m=upper_agreement_m)
    depth = np.asarray(camera_depth, dtype=np.float64)
    candidates = np.asarray(camera_candidates, dtype=bool)
    n, h, w = depth.shape
    rays = ref.normalized_pixel_rays(w, h, hfov_deg, vfov_deg)
    valid = candidates & np.isfinite(depth) & (depth > 0) & (depth < camera_far_m)
    p = np.einsum("ij,nhwj->nhwi", np.asarray(rotation, dtype=np.float64),
                  np.where(valid, depth, 0.)[..., None] * rays[None]) + np.asarray(translation, dtype=np.float64)
    anchor_l = np.min(np.where(valid, np.linalg.norm(p, axis=-1), np.inf), axis=(1, 2))
    el = np.arctan2(p[..., 2], np.hypot(p[..., 0], p[..., 1]))
    el_lo = np.min(np.where(valid, el, np.inf), axis=(1, 2))
    el_hi = np.max(np.where(valid, el, -np.inf), axis=(1, 2))
    rows = np.radians(np.asarray(lidar_elevations_deg, dtype=np.float64))
    row_ok = (rows[None, :] >= el_lo[:, None]) & (rows[None, :] <= el_hi[:, None])
    supported = base["supported"] & np.isfinite(anchor_l)
    s_l = np.where(supported, anchor_l - (lidar_radius - camera_radius), 0.)
    angular = np.where(supported, np.minimum(math.radians(max_angle_deg),
                                            np.arcsin(np.clip(lidar_radius / (s_l + lidar_radius), 0, 1))), 0.)
    scan = np.asarray(lidar_ranges, dtype=np.float64)
    angles = np.asarray(lidar_angles, dtype=np.float64)
    bearing = np.asarray(target_bearing, dtype=np.float64)
    delta = np.arctan2(np.sin(angles[None, :] - bearing[:, None]), np.cos(angles[None, :] - bearing[:, None]))
    valid_return = np.isfinite(scan) & (scan > 0) & (scan < lidar_far_m)
    lidar = (supported[:, None, None] & valid_return & np.isfinite(delta[:, None, :])
             & (np.abs(delta[:, None, :]) < angular[:, None, None])
             & (scan >= s_l[:, None, None]) & (scan < s_l[:, None, None] + upper_agreement_m)
             & row_ok[:, :, None])
    return {"lidar_carve_mask": lidar, "camera_blank_mask": base["camera_blank_mask"] & supported[:, None, None],
            "anchor_body_m": np.where(supported, anchor_l, 0.), "lidar_surface_lower_m": s_l,
            "supported": supported, "angular_limit_rad": angular}


def pinhole_column(theta, *, width=160, hfov_deg=87.):
    fx = width / (2 * math.tan(math.radians(hfov_deg) / 2))
    return (width - 1) / 2 - fx * math.tan(theta)


def self_test():
    checks = {}
    # On-axis wall x = 5 (body): the pixel nearest the optical axis reaches range ~5 from the body origin.
    surf = true_surface([[5.1, 0, 0]], [[.1, 20, 20]])
    cam = camera_points(surf["depth_m"][None])
    checks["point_matches_truth"] = bool(np.allclose(cam["rho"][0][cam["valid"][0]], surf["rho"][cam["valid"][0]], atol=1e-9))
    bins = lidar_bins(cam, clamp=True)
    checks["on_axis_bin_near_5m"] = bool(abs(bins[0, 2, 36] - 5.0) < 1e-3)
    # Rotation path: identical geometry rendered with a pitched mount and reprojected with the same R.
    r = rotation_matrix(pitch=math.radians(10), yaw=math.radians(5))
    surf_r = true_surface([[5.1, 0, 0]], [[.1, 20, 20]], rotation=r, translation=(.2, .05, -.04))
    cam_r = camera_points(surf_r["depth_m"][None], rotation=r, translation=(.2, .05, -.04))
    checks["rotated_point_matches_truth"] = bool(np.allclose(cam_r["rho"][0][cam_r["valid"][0]], surf_r["rho"][cam_r["valid"][0]], atol=1e-9))
    checks["bin_convention_pi_is_bin0"] = bool(bin_index(np.array([math.pi]), np.array([0.]))[0][0] == 0)
    checks["row0_is_plus20"] = bool(bin_index(np.array([0.]), np.array([math.radians(20)]))[1][0] == 0)
    return checks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    checks = self_test()
    print(json.dumps({"contract": CONTRACT, "checks": checks}, indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
