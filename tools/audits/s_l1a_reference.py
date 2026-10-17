#!/usr/bin/env python3
"""Independent float64 S-L1A evaluation oracle; NumPy/stdlib only.

Coordinates: +x forward, +y left, +z up; image u right, v down. Returned
class/index labels are evaluator-only geometry, never online sensor inputs.
All far-plane no-returns have MISS/-1 labels. No production sensor imports.
"""
import argparse
import json
import math
import numpy as np

MISS, BOX, SPHERE = 0, 1, 2
CONTRACT = "MOTAR_S_L1A_FLOAT64_REFERENCE_V1"


def _positive(value, name):
    if not np.isfinite(value) or value <= 0:
        raise ValueError(name + " must be finite and positive")


def normalized_pixel_rays(width=160, height=90, hfov_deg=87., vfov_deg=58.,
                          pixel_offset_xy=(0., 0.)):
    """Unit pinhole rays [height,width,3], including subpixel perturbations."""
    if isinstance(width, bool) or isinstance(height, bool) or int(width) != width or int(height) != height or min(width, height) < 1:
        raise ValueError("image dimensions must be positive integers")
    if not (0 < hfov_deg < 180 and 0 < vfov_deg < 180):
        raise ValueError("FOV must be between zero and 180 degrees")
    offset = np.asarray(pixel_offset_xy, dtype=np.float64)
    if offset.shape != (2,) or not np.isfinite(offset).all():
        raise ValueError("pixel offset must contain two finite scalars")
    width, height = int(width), int(height)
    fx = width / (2 * math.tan(math.radians(hfov_deg) / 2))
    fy = height / (2 * math.tan(math.radians(vfov_deg) / 2))
    v, u = np.meshgrid(np.arange(height, dtype=np.float64) + offset[1],
                       np.arange(width, dtype=np.float64) + offset[0], indexing="ij")
    rays = np.stack((np.ones_like(u), -(u - (width - 1) / 2) / fx,
                     -(v - (height - 1) / 2) / fy), axis=-1)
    return rays / np.linalg.norm(rays, axis=-1, keepdims=True)


def rotate_rays(rays, orientation_xyzw=(0., 0., 0., 1.)):
    """Rotate camera rays into world coordinates without changing input arrays."""
    q = np.asarray(orientation_xyzw, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.dot(q, q) - 1) > 1e-10:
        raise ValueError("orientation must be a normalized xyzw quaternion")
    rays = np.asarray(rays, dtype=np.float64)
    cross = np.cross(q[:3], rays)
    return rays + 2 * (q[3] * cross + np.cross(q[:3], cross))


def _boxes(centers, half_extents):
    c, h = np.asarray(centers, dtype=np.float64), np.asarray(half_extents, dtype=np.float64)
    if c.size == h.size == 0:
        c, h = np.empty((0, 3)), np.empty((0, 3))
    if c.ndim != 2 or c.shape[-1] != 3 or h.shape != c.shape or not np.isfinite(c).all() or not np.isfinite(h).all() or (h < 0).any():
        raise ValueError("boxes require finite [B,3] centers and nonnegative half extents")
    return c, h


def first_hits(origins, directions, box_centers=(), box_half_extents=(), *,
               sphere_center=None, sphere_radius=.15, far_m=10., box_chunk=64):
    """First nonnegative ray-surface slant range and evaluator class/object index.

    directions must be normalized [...,3]; origins broadcasts to that shape.
    Inside solids, the first forward exit surface is returned. Closed tangencies
    count. Box/sphere exact ties prefer BOX. Hits at/after far_m are no-return.
    A sphere uses index 0, boxes their input row, misses -1.
    """
    _positive(far_m, "far_m")
    d = np.asarray(directions, dtype=np.float64)
    if d.ndim < 1 or d.shape[-1] != 3 or not np.isfinite(d).all() or not np.allclose(np.linalg.norm(d, axis=-1), 1, atol=1e-10, rtol=0):
        raise ValueError("directions must be finite unit vectors [...,3]")
    try:
        o = np.broadcast_to(np.asarray(origins, dtype=np.float64), d.shape)
    except ValueError as exc:
        raise ValueError("origins must broadcast to ray shape") from exc
    if not np.isfinite(o).all():
        raise ValueError("origins must be finite")
    if isinstance(box_chunk, bool) or int(box_chunk) != box_chunk or box_chunk < 1:
        raise ValueError("box_chunk must be a positive integer")
    box_chunk = int(box_chunk)
    c, h = _boxes(box_centers, box_half_extents)
    shape = d.shape[:-1]
    d, o = d.reshape(-1, 3), o.reshape(-1, 3)
    best = np.full(len(d), float(far_m), dtype=np.float64)
    labels = np.full(len(d), MISS, dtype=np.int8)
    indices = np.full(len(d), -1, dtype=np.int64)
    for start in range(0, len(c), int(box_chunk)):
        lo, hi = c[start:start + box_chunk] - h[start:start + box_chunk], c[start:start + box_chunk] + h[start:start + box_chunk]
        origin, direction = o[:, None, :], d[:, None, :]
        parallel = direction == 0
        outside_parallel = parallel & ((origin < lo) | (origin > hi))
        divisor = np.where(parallel, 1., direction)
        a, b = (lo - origin) / divisor, (hi - origin) / divisor
        near = np.where(parallel, -np.inf, np.minimum(a, b)).max(axis=-1)
        far = np.where(parallel, np.inf, np.maximum(a, b)).min(axis=-1)
        valid = ~outside_parallel.any(axis=-1) & (near <= far) & (far >= 0)
        distance = np.where(valid, np.where(near >= 0, near, far), np.inf)
        pick = distance.argmin(axis=1)
        picked = distance[np.arange(len(d)), pick]
        replace = picked < best
        best[replace] = picked[replace]
        labels[replace], indices[replace] = BOX, start + pick[replace]
    if sphere_center is not None:
        s = np.asarray(sphere_center, dtype=np.float64)
        if s.shape != (3,) or not np.isfinite(s).all():
            raise ValueError("sphere center must contain three finite scalars")
        _positive(sphere_radius, "sphere_radius")
        relative = o - s
        b = np.einsum("ij,ij->i", relative, d)
        constant = np.einsum("ij,ij->i", relative, relative) - sphere_radius ** 2
        discriminant = b * b - constant
        root = np.sqrt(np.maximum(discriminant, 0))
        q = -b - np.copysign(root, b)
        second = np.divide(constant, q, out=np.zeros_like(q), where=q != 0)
        first, second = np.minimum(q, second), np.maximum(q, second)
        distance = np.where(first >= 0, first, second)
        valid = (discriminant >= 0) & (distance >= 0)
        replace = valid & (distance < best)
        best[replace] = distance[replace]
        labels[replace], indices[replace] = SPHERE, 0
    return {"depth_m": best.reshape(shape), "hit_class": labels.reshape(shape),
            "hit_index": indices.reshape(shape)}


def render_reference(box_centers=(), box_half_extents=(), *, sphere_center=None,
                     sphere_radius=.15, width=160, height=90, hfov_deg=87., vfov_deg=58.,
                     origin=(0., 0., 0.), orientation_xyzw=(0., 0., 0., 1.),
                     far_m=10., robustness_px=.01):
    """Pixel first hits; robust iff all eight ±epsilon neighboring labels agree.

    Robustness concerns the actual first object/class, not a depth-error threshold;
    slant ranges may legitimately vary smoothly with pixel position.
    """
    _positive(robustness_px, "robustness_px")
    def query(offset):
        rays = rotate_rays(normalized_pixel_rays(width, height, hfov_deg, vfov_deg, offset), orientation_xyzw)
        return first_hits(origin, rays, box_centers, box_half_extents,
                          sphere_center=sphere_center, sphere_radius=sphere_radius, far_m=far_m)
    out = query((0., 0.))
    robust = np.ones(out["depth_m"].shape, dtype=bool)
    for y in (-robustness_px, 0., robustness_px):
        for x in (-robustness_px, 0., robustness_px):
            if x == y == 0:
                continue
            other = query((x, y))
            robust &= (other["hit_class"] == out["hit_class"]) & (other["hit_index"] == out["hit_index"])
    out["robust_mask"] = robust
    return out


def carve_masks(lidar_ranges, lidar_angles, camera_depth, camera_candidates, visible,
                target_bearing, *, target_radius=.15, min_target_pixels=2,
                lidar_far_m=12., camera_far_m=20., max_angle_deg=15., upper_agreement_m=.55):
    """Independent expected S-L1A masks from delivered sensor scalars/arrays only.

    Camera arrays [N,H,W], LiDAR [N,V,Hbeam] and azimuths [Hbeam].
    Unsupported anchor is 0 (with supported=False), never NaN. No-return camera
    pixels cannot support the minimum count. Nearer returns are always retained.
    Camera blanking is limited to valid candidate pixels in the same anchor window.
    """
    for value, name in [(target_radius, "target_radius"), (lidar_far_m, "lidar_far_m"),
                        (camera_far_m, "camera_far_m"), (max_angle_deg, "max_angle_deg"),
                        (upper_agreement_m, "upper_agreement_m")]:
        _positive(value, name)
    if max_angle_deg >= 90 or isinstance(min_target_pixels, bool) or int(min_target_pixels) != min_target_pixels or min_target_pixels < 1:
        raise ValueError("invalid angle bound or minimum pixel count")
    depth = np.asarray(camera_depth, dtype=np.float64)
    candidates = np.asarray(camera_candidates)
    scan = np.asarray(lidar_ranges, dtype=np.float64)
    angles = np.asarray(lidar_angles, dtype=np.float64)
    v, bearing = np.asarray(visible), np.asarray(target_bearing, dtype=np.float64)
    if depth.ndim != 3 or candidates.shape != depth.shape or candidates.dtype != np.bool_:
        raise ValueError("camera depth/mask must be [N,H,W] with a boolean mask")
    n = len(depth)
    if scan.ndim != 3 or len(scan) != n or angles.shape != (scan.shape[-1],) or v.shape != (n,) or v.dtype != np.bool_ or bearing.shape != (n,):
        raise ValueError("LiDAR arrays or per-environment flags have incompatible shapes/types")
    valid_camera = candidates & np.isfinite(depth) & (depth > 0) & (depth < camera_far_m)
    count = valid_camera.sum(axis=(1, 2))
    anchor = np.min(np.where(valid_camera, depth, np.inf), axis=(1, 2))
    supported = v & np.isfinite(bearing) & (count >= min_target_pixels) & np.isfinite(anchor)
    anchor = np.where(supported, anchor, 0.)
    angular = np.where(supported, np.minimum(math.radians(max_angle_deg),
                                            np.arcsin(target_radius / (anchor + target_radius))), 0.)
    delta = np.arctan2(np.sin(angles[None, :] - bearing[:, None]), np.cos(angles[None, :] - bearing[:, None]))
    valid_return = np.isfinite(scan) & (scan > 0) & (scan < lidar_far_m)
    lidar = (supported[:, None, None] & valid_return & np.isfinite(delta[:, None, :])
             & (np.abs(delta[:, None, :]) < angular[:, None, None])
             & (scan >= anchor[:, None, None]) & (scan < anchor[:, None, None] + upper_agreement_m))
    camera = (supported[:, None, None] & valid_camera & (depth >= anchor[:, None, None])
              & (depth < anchor[:, None, None] + upper_agreement_m))
    return {"lidar_carve_mask": lidar, "camera_blank_mask": camera, "anchor_m": anchor,
            "supported": supported, "angular_limit_rad": angular, "valid_candidate_count": count}


def bilinear_resize(depth, out_height, out_width):
    """Independent half-pixel-centre bilinear reference (edge-clamped)."""
    image = np.asarray(depth, dtype=np.float64)
    if image.ndim != 2 or min(out_height, out_width) < 1 or int(out_height) != out_height or int(out_width) != out_width:
        raise ValueError("resize expects a 2-D image and positive integer output dimensions")
    h, w = image.shape
    y = np.clip((np.arange(out_height) + .5) * h / out_height - .5, 0, h - 1)
    x = np.clip((np.arange(out_width) + .5) * w / out_width - .5, 0, w - 1)
    y0, x0 = np.floor(y).astype(int), np.floor(x).astype(int)
    y1, x1 = np.minimum(y0 + 1, h - 1), np.minimum(x0 + 1, w - 1)
    wy, wx = y - y0, x - x0
    return ((1-wy[:, None]) * ((1-wx[None, :])*image[y0[:, None], x0[None, :]] + wx[None, :]*image[y0[:, None], x1[None, :]])
            + wy[:, None] * ((1-wx[None, :])*image[y1[:, None], x0[None, :]] + wx[None, :]*image[y1[:, None], x1[None, :]]))


def self_test():
    """Small exact hand checks; comprehensive independent fixtures live in unittest."""
    rays = np.array([[1., 0., 0.], [0., 1., 0.]])
    hit = first_hits([0., 0., 0.], rays, [[3., 0., 0.]], [[1., 1., 1.]])
    assert hit["depth_m"].tolist() == [2., 10.]
    assert hit["hit_class"].tolist() == [BOX, MISS]
    sphere = first_hits([0., 0., 0.], rays[:1], sphere_center=[3., 0., 0.], sphere_radius=1.)
    assert sphere["depth_m"].item() == 2.
    return {"contract": CONTRACT, "status": "PASS", "checks": 4, "precision": "float64", "device": "cpu"}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--self-test", action="store_true")
    args = p.parse_args()
    if not args.self_test:
        p.error("this independent oracle exposes --self-test only; no simulator launch command")
    print(json.dumps(self_test(), sort_keys=True))


if __name__ == "__main__":
    main()
