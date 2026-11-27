"""Offline 3D geometric line of sight for recorded occlusion traces (numpy, CPU, evaluation only).

Independent of ``motar_eval.geometry`` (torch) so the two can check each other. Obstacles are boxes
``center +/- half``; a non-identity quaternion is handled by moving the ray into the box frame, so
rotated boxes are exact too (the bars_h3 pool is upright, unrotated and its collision box equals
its visual box). The camera sits on the yaw-only vehicle frame, x forward, y left, z up.

Three geometric quantities are kept apart, and none of them is the rendered camera image:

* ``center_blocked`` -- the eye-to-target-centre segment touches a box (closed test, grazing counts);
* ``rim_blocked_fraction`` -- the share of rays to points on the target's silhouette rim (a sphere of
  the rendered target radius, rim taken perpendicular to the line of sight) that touch a box;
  ``geometric_full_occlusion`` is centre and every rim ray blocked (a geometric proxy, not pixels);
* ``in_frustum`` / ``in_range`` -- the target centre against the pinhole frustum and range.
"""

from __future__ import annotations

import math

import numpy as np

_EPS = 1e-12


def quat_rotate(quat, vec):
    """Rotate ``vec`` [..., 3] by unit quaternions ``quat`` [..., 4] (xyzw)."""
    q = np.asarray(quat, dtype=np.float64)
    v = np.asarray(vec, dtype=np.float64)
    qv = q[..., :3]
    t = 2.0 * np.cross(qv, v)
    return v + q[..., 3:4] * t + np.cross(qv, t)


def quat_rotate_inverse(quat, vec):
    q = np.asarray(quat, dtype=np.float64).copy()
    q[..., :3] *= -1.0
    return quat_rotate(q, vec)


def camera_eye(position, vehicle_quat, camera_translation):
    offset = np.broadcast_to(np.asarray(camera_translation, dtype=np.float64), np.shape(position))
    return np.asarray(position, dtype=np.float64) + quat_rotate(vehicle_quat, offset)


def in_frustum(eye, vehicle_quat, target, hfov_rad, vfov_rad, max_range_m):
    """(centre inside the angular frustum, centre within range), each [...] bool."""
    rel = quat_rotate_inverse(vehicle_quat, np.asarray(target, dtype=np.float64) - eye)
    x, y, z = rel[..., 0], rel[..., 1], rel[..., 2]
    angular = (x > 0.0) & (np.abs(y) <= x * math.tan(0.5 * hfov_rad)) & (np.abs(z) <= x * math.tan(0.5 * vfov_rad))
    return angular, np.linalg.norm(rel, axis=-1) <= max_range_m


def segment_hits_boxes(p0, p1, center, half, quat=None):
    """[N] segments against [B] boxes -> [N, B] bool (closed slab test; touching counts as a hit)."""
    p0 = np.asarray(p0, dtype=np.float64)[:, None, :]
    p1 = np.asarray(p1, dtype=np.float64)[:, None, :]
    center = np.asarray(center, dtype=np.float64)[None]
    half = np.asarray(half, dtype=np.float64)[None]
    if quat is not None:
        quat = np.asarray(quat, dtype=np.float64)
        if not np.allclose(quat, [0.0, 0.0, 0.0, 1.0], atol=1e-7):
            qb = np.broadcast_to(quat[None], (p0.shape[0],) + quat.shape)
            p0 = quat_rotate_inverse(qb, p0 - center)
            p1 = quat_rotate_inverse(qb, p1 - center)
            center = np.zeros_like(center)
    lo, hi = center - half, center + half
    d = p1 - p0
    parallel = np.abs(d) <= _EPS
    safe = np.where(parallel, 1.0, d)
    t0, t1 = (lo - p0) / safe, (hi - p0) / safe
    t_near, t_far = np.minimum(t0, t1), np.maximum(t0, t1)
    inside = (p0 >= lo) & (p0 <= hi)
    t_near = np.where(parallel, np.where(inside, -np.inf, np.inf), t_near)
    t_far = np.where(parallel, np.where(inside, np.inf, -np.inf), t_far)
    enter, leave = t_near.max(axis=-1), t_far.min(axis=-1)
    return (enter <= leave) & (leave >= 0.0) & (enter <= 1.0)


def rim_points(eye, target, radius, count=8):
    """[N, count, 3] points on the target sphere's silhouette rim as seen from ``eye``."""
    eye, target = np.asarray(eye, dtype=np.float64), np.asarray(target, dtype=np.float64)
    axis = target - eye
    axis /= np.maximum(np.linalg.norm(axis, axis=-1, keepdims=True), _EPS)
    helper = np.where(np.abs(axis[:, 2:3]) < 0.9, np.array([[0.0, 0.0, 1.0]]), np.array([[1.0, 0.0, 0.0]]))
    u = np.cross(axis, helper)
    u /= np.maximum(np.linalg.norm(u, axis=-1, keepdims=True), _EPS)
    w = np.cross(axis, u)
    angles = 2.0 * math.pi * np.arange(count) / count
    ring = np.cos(angles)[None, :, None] * u[:, None, :] + np.sin(angles)[None, :, None] * w[:, None, :]
    return target[:, None, :] + radius * ring


def line_of_sight(eye, target, center, half, quat=None, radius=0.15, rim_count=8, chunk=4096):
    """Per-frame centre blockage, rim blocked fraction and the geometric full-occlusion proxy."""
    eye, target = np.atleast_2d(eye), np.atleast_2d(target)
    n = len(eye)
    centre = np.zeros(n, dtype=bool)
    fraction = np.zeros(n, dtype=np.float64)
    for a in range(0, n, chunk):
        b = min(n, a + chunk)
        if len(center) == 0:
            continue
        centre[a:b] = segment_hits_boxes(eye[a:b], target[a:b], center, half, quat).any(axis=1)
        if rim_count:
            rim = rim_points(eye[a:b], target[a:b], radius, rim_count)
            m = b - a
            hits = segment_hits_boxes(np.repeat(eye[a:b], rim_count, axis=0), rim.reshape(m * rim_count, 3),
                                      center, half, quat).any(axis=1).reshape(m, rim_count)
            fraction[a:b] = hits.mean(axis=1)
    return {"center_blocked": centre, "rim_blocked_fraction": fraction,
            "geometric_full_occlusion": centre & (fraction >= 1.0) if rim_count else np.zeros(n, dtype=bool)}


__all__ = ["quat_rotate", "quat_rotate_inverse", "camera_eye", "in_frustum", "segment_hits_boxes",
           "rim_points", "line_of_sight"]
