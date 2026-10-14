"""Ground-truth geometry for the KROC2027 recorder: line of sight, camera frustum, swept clearance.

Everything here is evaluation-only. None of it may reach the actor observation (AGENTS.md
observation contract); the recorder writes it to sidecar files.

Conventions
-----------
* Positions are in the environment-local world frame shared by ``robot_position``,
  ``obstacle_position`` and the virtual ``target_position``.
* Quaternions are Isaac Gym ``xyzw``.
* Obstacles are axis-aligned boxes ``center +/- half`` (the bar URDFs have one centred collision box
  and no yaw; ``env_object_config.py`` places bars without rotation).
* The NavRL camera is mounted level on the yaw-only vehicle frame (the detector renders with
  ``robot_vehicle_orientation``), x forward, y left, z up.

All functions are vectorised with torch and broadcast over leading dimensions; they work on CPU and
CUDA tensors alike and import nothing from ``aerial_gym``.
"""

from __future__ import annotations

import math

import torch

_PARALLEL_EPS = 1e-12


def wrap_angle(angle):
    """Wrap radians to [-pi, pi)."""
    return torch.remainder(angle + math.pi, 2.0 * math.pi) - math.pi


def yaw_quat_xyzw(yaw):
    """Yaw-only rotation as an ``xyzw`` quaternion, the layout the NavRL spawn code writes."""
    quat = torch.zeros(yaw.shape + (4,), dtype=yaw.dtype, device=yaw.device)
    quat[..., 2] = torch.sin(0.5 * yaw)
    quat[..., 3] = torch.cos(0.5 * yaw)
    return quat


def quat_rotate_xyzw(quat, vec):
    """Rotate ``vec`` [..., 3] by the unit quaternion ``quat`` [..., 4] (xyzw)."""
    q_vec = quat[..., 0:3]
    q_w = quat[..., 3:4]
    t = 2.0 * torch.cross(q_vec, vec, dim=-1)
    return vec + q_w * t + torch.cross(q_vec, t, dim=-1)


def quat_rotate_inverse_xyzw(quat, vec):
    """Rotate ``vec`` by the inverse of the unit quaternion ``quat`` (xyzw)."""
    conj = torch.cat((-quat[..., 0:3], quat[..., 3:4]), dim=-1)
    return quat_rotate_xyzw(conj, vec)


def camera_eye(position, vehicle_quat_xyzw, camera_translation):
    """Camera centre in the world frame for a camera fixed in the yaw-only vehicle frame."""
    offset = torch.as_tensor(camera_translation, dtype=position.dtype, device=position.device)
    offset = offset.expand(position.shape)
    return position + quat_rotate_xyzw(vehicle_quat_xyzw.to(position.dtype), offset)


def segment_hits_aabb(p0, p1, box_lo, box_hi):
    """Closed-box slab test: does the segment ``p0 -> p1`` touch each box?

    ``p0``/``p1`` are [..., D]; ``box_lo``/``box_hi`` are [..., B, D] (D = 2 or 3). Returns
    [..., B] bool. Touching a face, edge or corner counts as a hit, so a grazing ray is blocked.
    """
    origin = p0.unsqueeze(-2)
    direction = (p1 - p0).unsqueeze(-2)
    parallel = direction.abs() <= _PARALLEL_EPS
    safe = torch.where(parallel, torch.ones_like(direction), direction)
    t0 = (box_lo - origin) / safe
    t1 = (box_hi - origin) / safe
    t_near = torch.minimum(t0, t1)
    t_far = torch.maximum(t0, t1)
    inside_slab = (origin >= box_lo) & (origin <= box_hi)
    inf = torch.full_like(t_near, math.inf)
    t_near = torch.where(parallel, torch.where(inside_slab, -inf, inf), t_near)
    t_far = torch.where(parallel, torch.where(inside_slab, inf, -inf), t_far)
    enter = t_near.amax(dim=-1)
    leave = t_far.amin(dim=-1)
    return (enter <= leave) & (leave >= 0.0) & (enter <= 1.0)


def los_clear(eye, target, obstacle_center, obstacle_half, obstacle_valid=None):
    """True where the straight ray from ``eye`` to the target centre touches no obstacle box.

    ``eye``/``target`` [..., 3]; ``obstacle_center``/``obstacle_half`` [..., B, 3];
    ``obstacle_valid`` (optional) [..., B] bool excludes parked or inactive slots.
    """
    if obstacle_center.shape[-2] == 0:
        return torch.ones(eye.shape[:-1], dtype=torch.bool, device=eye.device)
    hits = segment_hits_aabb(
        eye, target, obstacle_center - obstacle_half, obstacle_center + obstacle_half
    )
    if obstacle_valid is not None:
        hits = hits & obstacle_valid
    return ~hits.any(dim=-1)


def in_camera_frustum(eye, camera_quat_xyzw, target, hfov_rad, vfov_rad, max_range_m):
    """True where the target centre lies inside a symmetric pinhole frustum.

    The camera looks along its +x axis (y left, z up). ``max_range_m`` bounds the Euclidean range;
    the detector's own cut-off is on rendered depth, which can differ near the frustum edge.
    """
    rel = quat_rotate_inverse_xyzw(camera_quat_xyzw.to(eye.dtype), target - eye)
    x, y, z = rel[..., 0], rel[..., 1], rel[..., 2]
    tan_h = math.tan(0.5 * float(hfov_rad))
    tan_v = math.tan(0.5 * float(vfov_rad))
    return (
        (x > 0.0)
        & (y.abs() <= x * tan_h)
        & (z.abs() <= x * tan_v)
        & (rel.norm(dim=-1) <= float(max_range_m))
    )


def relative_bearing(position, vehicle_quat_xyzw, target):
    """Bearing of the target in the vehicle frame (radians, 0 = straight ahead, + = left)."""
    rel = quat_rotate_inverse_xyzw(vehicle_quat_xyzw.to(position.dtype), target - position)
    return torch.atan2(rel[..., 1], rel[..., 0])


def point_rect_distance_2d(point, center, half):
    """Distance from ``point`` [..., 2] (broadcastable) to closed rectangles ``center +/- half``."""
    return ((point - center).abs() - half).clamp(min=0.0).norm(dim=-1)


_CORNER_SIGNS = ((1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0))


def segment_rect_distance_2d(p0, p1, center, half):
    """Exact distance between the segment ``p0 -> p1`` [..., 2] and rectangles [..., B, 2].

    Zero where the segment touches a rectangle. Otherwise the minimum distance between a segment
    and a disjoint convex polygon is attained at a vertex of one of them: a segment endpoint against
    the rectangle, or a rectangle corner against the segment.
    """
    hit = segment_hits_aabb(p0, p1, center - half, center + half)
    d0 = point_rect_distance_2d(p0.unsqueeze(-2), center, half)
    d1 = point_rect_distance_2d(p1.unsqueeze(-2), center, half)
    signs = torch.tensor(_CORNER_SIGNS, dtype=center.dtype, device=center.device)
    corners = center.unsqueeze(-2) + half.unsqueeze(-2) * signs  # [..., B, 4, 2]
    a = p0.unsqueeze(-2).unsqueeze(-2)
    seg = (p1 - p0).unsqueeze(-2).unsqueeze(-2)
    denom = (seg * seg).sum(dim=-1).clamp(min=1e-18)
    t = (((corners - a) * seg).sum(dim=-1) / denom).clamp(0.0, 1.0)
    closest = a + t.unsqueeze(-1) * seg
    d_corner = (corners - closest).norm(dim=-1).amin(dim=-1)
    distance = torch.minimum(torch.minimum(d0, d1), d_corner)
    return torch.where(hit, torch.zeros_like(distance), distance)


def swept_proxy_clearance(p0, p1, center, half, valid=None):
    """Smallest distance from the swept target centre ``p0 -> p1`` to any obstacle footprint.

    A disk proxy of radius ``r`` touches an obstacle during the step exactly when this value is
    below ``r``. Returns [...]; ``inf`` when there is no valid obstacle.
    """
    if center.shape[-2] == 0:
        return torch.full(p0.shape[:-1], math.inf, dtype=p0.dtype, device=p0.device)
    distance = segment_rect_distance_2d(p0, p1, center, half)
    if valid is not None:
        distance = torch.where(valid, distance, torch.full_like(distance, math.inf))
    return distance.amin(dim=-1)
