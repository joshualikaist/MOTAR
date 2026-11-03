"""CPU replay of the MOTAR 72x4 LiDAR from recorded B1-R traces (M2; docs/plans/prereg_cplanner_m2_v1.md).

The scan is cast with MOTAR's own Warp kernel `LidarWarpKernels.draw_optimized_kernel_range_segmentation_target`
(loaded from its file, because importing the `aerial_gym` package imports Isaac Gym), on the Warp CPU device,
against a static mesh of the recorded active bars (obstacle_position + collision half extents, recorded quaternion)
and the analytic target sphere at the recorded target position. Ray vectors, range limits and normalisation repeat
`warp_lidar.initialize_ray_vectors` and `warp_sensor.apply_range_limits / normalize_observation` in float32.

The raw GPU scans were never recorded (`RESEARCH_PLAN_AFTER_P0R.md` §6), so this is a reconstruction from the same
code path, not a copy of the delivered data. V1 qualifies the status mapping on these replayed scans against an
independent float64 ray caster; what the replay cannot show is listed in the M2 preregistration.
"""
import importlib.util
import math
import pathlib

import numpy as np
import torch
import warp as wp

REPO = pathlib.Path(__file__).resolve().parents[2]
KERNEL_FILE = REPO / "aerial_gym" / "sensors" / "warp" / "warp_kernels" / "warp_lidar_kernels.py"
H, W = 4, 72
HFOV = (-175.0, 180.0)            # navrl_lidar_config: -180 + 360/72, 180
VFOV = (-10.0, 20.0)
MAX_RANGE, MIN_RANGE = 12.0, 0.2
TARGET_RADIUS = 0.20              # navrl_lidar_config.target_radius (analytic LiDAR sphere)
TARGET_SEMANTIC = 15              # label only; never read by the adapter
DEVICE = "cpu"

_kernels = None


def kernels():
    global _kernels
    if _kernels is None:
        spec = importlib.util.spec_from_file_location("motar_warp_lidar_kernels", str(KERNEL_FILE))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        wp.init()
        _kernels = mod
    return _kernels


def ray_vectors():
    """(H, W, 3) float32, exactly as warp_lidar.initialize_ray_vectors (torch float32, then normalised)."""
    hmin, hmax = math.radians(HFOV[0]), math.radians(HFOV[1])
    vmin, vmax = math.radians(VFOV[0]), math.radians(VFOV[1])
    rv = torch.zeros((H, W, 3), dtype=torch.float32)
    for i in range(H):
        for j in range(W):
            az = hmax - (hmax - hmin) * (j / (W - 1))
            el = vmax - (vmax - vmin) * (i / (H - 1))
            rv[i, j, 0] = math.cos(az) * math.cos(el)
            rv[i, j, 1] = math.sin(az) * math.cos(el)
            rv[i, j, 2] = math.sin(el)
    rv = rv / torch.norm(rv, dim=2, keepdim=True)
    return rv.numpy()


_FACES = np.array([(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),
                   (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)], np.int32)


def _quat_rotate_xyzw(q, v):
    x, y, z, w = q
    u = np.array([x, y, z], np.float64)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def box_mesh(centers, halves, quats):
    """Merged triangle mesh (points float32 (8B, 3), indices int32 (36B,)) of oriented boxes."""
    pts, idx = [], []
    for b, (c, h, q) in enumerate(zip(centers, halves, quats)):
        corners = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
                           np.float64)
        corners = np.array([_quat_rotate_xyzw(q, p) for p in corners]) + np.asarray(c, np.float64)
        pts.append(corners)
        idx.append(_FACES + 8 * b)
    return np.concatenate(pts).astype(np.float32), np.concatenate(idx).reshape(-1).astype(np.int32)


class Scene:
    """One static layout; `scan` casts a normalised (H, W) float32 scan for a sensor pose and target position."""

    def __init__(self, centers, halves, quats):
        k = kernels()
        self.k = k
        p, i = box_mesh(centers, halves, quats)
        self.mesh = wp.Mesh(points=wp.array(p, dtype=wp.vec3, device=DEVICE),
                            indices=wp.array(i, dtype=wp.int32, device=DEVICE),
                            velocities=wp.array(np.zeros_like(p), dtype=wp.vec3, device=DEVICE))
        self.ids = wp.array([self.mesh.id], dtype=wp.uint64, device=DEVICE)
        self.rv = wp.array(ray_vectors(), dtype=wp.vec3, device=DEVICE)

    def raw(self, sensor_pos, sensor_quat_xyzw, target_pos):
        """Kernel output (metres; miss = 1000) before range limits, float32 (H, W), and segmentation."""
        pos = wp.array(np.asarray(sensor_pos, np.float32).reshape(1, 1, 3), dtype=wp.vec3, device=DEVICE)
        quat = wp.array(np.asarray(sensor_quat_xyzw, np.float32).reshape(1, 1, 4), dtype=wp.quat, device=DEVICE)
        pix = wp.zeros((1, 1, H, W), dtype=float, device=DEVICE)
        seg = wp.zeros((1, 1, H, W), dtype=wp.int32, device=DEVICE)
        tpos = wp.array(np.asarray(target_pos, np.float32).reshape(1, 3), dtype=wp.vec3, device=DEVICE)
        tq = wp.array(np.array([[0, 0, 0, 1]], np.float32), dtype=wp.quat, device=DEVICE)
        wp.launch(self.k.LidarWarpKernels.draw_optimized_kernel_range_segmentation_target, dim=(1, 1, H, W),
                  inputs=[self.ids, pos, quat, self.rv, float(MAX_RANGE), pix, seg, tpos, tq, float(TARGET_RADIUS),
                          wp.vec3(0.0, 0.0, 0.0), 0, TARGET_SEMANTIC], device=DEVICE)
        return pix.numpy()[0, 0].copy(), seg.numpy()[0, 0].copy()

    def scan(self, sensor_pos, sensor_quat_xyzw, target_pos):
        """Normalised (H, W) float32 scan, as `depth_range_pixels` after range limits and normalisation."""
        px, _ = self.raw(sensor_pos, sensor_quat_xyzw, target_pos)
        t = torch.from_numpy(px)
        t[t > MAX_RANGE] = MAX_RANGE
        t[t < MIN_RANGE] = -MAX_RANGE
        t = t / MAX_RANGE
        return t.numpy()


def sensor_yaw(p_quat_xyzw):
    """(header yaw float64 in [-pi, pi], sensor quaternion float32 xyzw) of the yaw-only attach, as production computes
    it: get_euler_xyz in float32 (atan2, then % 2 pi) and quat_from_euler_xyz(0, 0, yaw) (warp_sensor.update;
    utils/math.py get_euler_xyz / quat_from_euler_xyz). The header angle is wrapped because float32(2 pi) exceeds
    2 pi and the certifier refuses |yaw| > 2 pi; the rotation is the same."""
    q = torch.tensor(np.asarray(p_quat_xyzw, np.float32).reshape(1, 4))
    qx, qy, qz, qw = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = qw * qw + qx * qx - qy * qy - qz * qz
    yaw = torch.atan2(siny_cosp, cosy_cosp) % (2 * torch.pi)
    quat = np.array([0.0, 0.0, float(torch.sin(yaw * 0.5)[0]), float(torch.cos(yaw * 0.5)[0])], np.float32)
    return math.remainder(float(yaw[0]), 2.0 * math.pi), quat
