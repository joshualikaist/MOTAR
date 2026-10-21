"""Frame transforms and sensor geometry matching the audited Frozen F conventions (numpy, CPU)."""
import math

import numpy as np


def quat_yaw_xyzw(q):
    x, y, z, w = (float(c) for c in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def is_yaw_only(q, tol=1e-4):
    q = np.asarray(q, dtype=float)
    return abs(np.linalg.norm(q) - 1.0) < tol and abs(q[0]) < tol and abs(q[1]) < tol


def yaw_rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def world_to_vehicle(p_world, position, yaw):
    """YAW_VEHICLE coordinates of world points: R(yaw)^T (p - position)."""
    return (np.asarray(p_world, dtype=float) - np.asarray(position, dtype=float)) @ yaw_rotation(yaw)


def vehicle_to_world(p_vehicle, position, yaw):
    return np.asarray(p_vehicle, dtype=float) @ yaw_rotation(yaw).T + np.asarray(position, dtype=float)


def cov_world_to_vehicle(cov3, yaw):
    r = yaw_rotation(yaw)
    return r.T @ np.asarray(cov3, dtype=float) @ r


def vehicle_to_camera(p_vehicle, camera):
    """Camera axes equal YAW_VEHICLE axes (nominal R=I); only the origin moves."""
    return np.asarray(p_vehicle, dtype=float) - np.asarray(camera.camera_offset, dtype=float)


def project(p_camera, camera):
    """Pinhole inverse of navrl_perception's ray = [1, -(u-cx)/fx, -(v-cy)/fy]. Returns
    (u, v, in_front, in_image); u/v are NaN behind the camera."""
    p = np.asarray(p_camera, dtype=float)
    x = p[..., 0]
    in_front = x > 1e-6
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.where(in_front, camera.cx - camera.fx * p[..., 1] / x, np.nan)
        v = np.where(in_front, camera.cy - camera.fy * p[..., 2] / x, np.nan)
    width, height = 2 * camera.cx + 1, 2 * camera.cy + 1
    in_image = in_front & (u >= -0.5) & (u <= width - 0.5) & (v >= -0.5) & (v <= height - 0.5)
    return u, v, in_front, in_image


def pixel_rays(u, v, camera):
    r = np.stack([np.ones_like(np.asarray(u, dtype=float)), -(np.asarray(u, dtype=float) - camera.cx) / camera.fx,
                  -(np.asarray(v, dtype=float) - camera.cy) / camera.fy], axis=-1)
    return r / np.linalg.norm(r, axis=-1, keepdims=True)


def lidar_directions(model):
    """(vbeams, hbeams, 3) unit rays: (cos az cos el, sin az cos el, sin el)."""
    az, el = model.azimuth_rad()[None, :], model.elevation_rad()[:, None]
    return np.stack([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el) * np.ones_like(az)], axis=-1)


def angle_diff(a, b):
    return np.arctan2(np.sin(np.asarray(a) - b), np.cos(np.asarray(a) - b))


def sphere_first_hit(origin, directions, centre, radius):
    """Distance along unit rays to a sphere surface (G4-R5 B04: t = u.c - sqrt(R^2 - |c_perp|^2)); inf if missed."""
    c = np.asarray(centre, dtype=float) - np.asarray(origin, dtype=float)
    d = np.asarray(directions, dtype=float)
    along = d @ c
    perp2 = float(c @ c) - along * along
    disc = radius * radius - perp2
    t = along - np.sqrt(np.clip(disc, 0.0, None))
    return np.where((disc >= 0.0) & (t > 0.0), t, np.inf)
