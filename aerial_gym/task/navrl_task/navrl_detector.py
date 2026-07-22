"""Forward camera perception for ``NAVRL_VISION=1``.

This class adds a separate forward pinhole camera without going through Aerial Gym's single-Warp-
sensor slot (which cannot host a camera and LiDAR at the same time).  It produces both (1) a
low-resolution full-scene obstacle depth image for the policy and (2) a higher-resolution target
semantic/depth image for detection.  Target pixels are checked against the same environment mesh,
so closer bars remove them.  Bearing, elevation and range are then computed from the rendered
pixels -- never from the ground-truth relative target vector.

The target is an analytic sphere only inside the renderer.  This is equivalent to a simulator
using the true pose to rasterize a target mesh: the pose is not exposed to the policy.  A later
RGB detector can replace the semantic mask while retaining the same 8-D detector interface.
"""

import math

import torch
import torch.nn.functional as F
import warp as wp

from aerial_gym.utils.math import quat_rotate


@wp.kernel
def _render_obstacle_depth_kernel(
    mesh_ids: wp.array(dtype=wp.uint64),
    origins: wp.array(dtype=wp.vec3),
    orientations: wp.array(dtype=wp.quat),
    ray_vectors: wp.array2d(dtype=wp.vec3),
    far_plane: float,
    obstacle_depth: wp.array(dtype=float, ndim=3),
):
    """Render full-scene metric depth for every low-resolution camera ray."""
    env_id, row, col = wp.tid()
    ro = origins[env_id]
    rd = wp.normalize(wp.quat_rotate(orientations[env_id], ray_vectors[row, col]))
    t = float(0.0)
    u = float(0.0)
    v = float(0.0)
    sign = float(0.0)
    normal = wp.vec3()
    face = int(0)
    dist = far_plane
    if wp.mesh_query_ray(mesh_ids[env_id], ro, rd, far_plane, t, u, v, sign, normal, face):
        dist = t
    obstacle_depth[env_id, row, col] = dist


@wp.kernel
def _render_target_camera_kernel(
    mesh_ids: wp.array(dtype=wp.uint64),
    origins: wp.array(dtype=wp.vec3),
    orientations: wp.array(dtype=wp.quat),
    ray_vectors: wp.array2d(dtype=wp.vec3),
    target_positions: wp.array(dtype=wp.vec3),
    target_radius: float,
    far_plane: float,
    target_mask: wp.array(dtype=wp.int32, ndim=3),
    target_depth: wp.array(dtype=float, ndim=3),
):
    """Render only target pixels; non-target scene pixels are irrelevant to the detector.

    A mesh ray query is issued only for a ray that first intersects the analytic target sphere.
    This preserves exact obstacle occlusion while avoiding a costly full-scene camera render for
    every pixel in every parallel environment.
    """
    env_id, row, col = wp.tid()
    ro = origins[env_id]
    rd = wp.normalize(wp.quat_rotate(orientations[env_id], ray_vectors[row, col]))

    target_mask[env_id, row, col] = wp.int32(0)
    target_depth[env_id, row, col] = far_plane

    # Analytic ray-sphere intersection in world coordinates.
    oc = ro - target_positions[env_id]
    b = wp.dot(oc, rd)
    c = wp.dot(oc, oc) - target_radius * target_radius
    disc = b * b - c
    if disc >= 0.0:
        root = wp.sqrt(disc)
        t_target = -b - root
        if t_target < 0.0:
            t_target = -b + root
        if t_target >= 0.0 and t_target < far_plane:
            # A target pixel survives only if no bar surface lies before it.
            t = float(0.0)
            u = float(0.0)
            v = float(0.0)
            sign = float(0.0)
            normal = wp.vec3()
            face = int(0)
            blocked = wp.mesh_query_ray(
                mesh_ids[env_id], ro, rd, t_target, t, u, v, sign, normal, face
            )
            if not blocked:
                target_mask[env_id, row, col] = wp.int32(1)
                target_depth[env_id, row, col] = t_target


class NavRLTargetDetector:
    """Pixel-derived camera detection plus short detector-side tracking memory."""

    def __init__(self, warp_env, num_envs, device, vis_cfg, step_dt):
        self.num_envs = int(num_envs)
        self.device = device
        self.width = int(getattr(vis_cfg, "camera_width", 160))
        self.height = int(getattr(vis_cfg, "camera_height", 90))
        self.max_range = float(vis_cfg.detector_max_range)
        self.hfov = math.radians(float(vis_cfg.detector_hfov_deg))
        self.vfov = math.radians(float(vis_cfg.detector_vfov_deg))
        self.half_hfov = self.hfov * 0.5
        self.half_vfov = self.vfov * 0.5
        self.target_radius = float(getattr(vis_cfg, "camera_target_radius", 0.15))
        self.min_pixels = max(1, int(getattr(vis_cfg, "camera_min_target_pixels", 1)))
        self.obstacle_width = int(getattr(vis_cfg, "camera_obstacle_width", 40))
        self.obstacle_height = int(getattr(vis_cfg, "camera_obstacle_height", 24))
        self.obstacle_max_range = float(
            getattr(vis_cfg, "camera_obstacle_max_range", self.max_range)
        )
        self.memory_s = max(1e-3, float(vis_cfg.tracker_memory_s))
        self.step_dt = float(step_dt)

        self.fx = self.width / (2.0 * math.tan(self.half_hfov))
        self.fy = self.height / (2.0 * math.tan(self.half_vfov))
        self.cx = (self.width - 1) * 0.5
        self.cy = (self.height - 1) * 0.5

        # Vehicle camera frame: +x forward, +y left, +z up. Image u grows right and v down.
        rows = torch.arange(self.height, device=device, dtype=torch.float32)
        cols = torch.arange(self.width, device=device, dtype=torch.float32)
        vv, uu = torch.meshgrid(rows, cols, indexing="ij")
        rays = torch.stack(
            [
                torch.ones_like(uu),
                -(uu - self.cx) / self.fx,
                -(vv - self.cy) / self.fy,
            ],
            dim=-1,
        )
        rays = rays / rays.norm(dim=-1, keepdim=True).clamp(min=1e-9)

        obstacle_rows = torch.arange(
            self.obstacle_height, device=device, dtype=torch.float32
        )
        obstacle_cols = torch.arange(
            self.obstacle_width, device=device, dtype=torch.float32
        )
        obstacle_vv, obstacle_uu = torch.meshgrid(
            obstacle_rows, obstacle_cols, indexing="ij"
        )
        obstacle_fx = self.obstacle_width / (2.0 * math.tan(self.half_hfov))
        obstacle_fy = self.obstacle_height / (2.0 * math.tan(self.half_vfov))
        obstacle_cx = (self.obstacle_width - 1) * 0.5
        obstacle_cy = (self.obstacle_height - 1) * 0.5
        obstacle_rays = torch.stack(
            [
                torch.ones_like(obstacle_uu),
                -(obstacle_uu - obstacle_cx) / obstacle_fx,
                -(obstacle_vv - obstacle_cy) / obstacle_fy,
            ],
            dim=-1,
        )
        obstacle_rays = obstacle_rays / obstacle_rays.norm(
            dim=-1, keepdim=True
        ).clamp(min=1e-9)

        self.mesh_ids = wp.array(warp_env.CONST_WARP_MESH_ID_LIST, dtype=wp.uint64, device=device)
        self._ray_vectors_wp = wp.from_torch(rays.contiguous(), dtype=wp.vec3)
        self._obstacle_ray_vectors_wp = wp.from_torch(
            obstacle_rays.contiguous(), dtype=wp.vec3
        )
        self._origins = torch.zeros((self.num_envs, 3), dtype=torch.float32, device=device)
        self._orientations = torch.zeros(
            (self.num_envs, 4), dtype=torch.float32, device=device
        )
        self._orientations[:, 3] = 1.0
        self._targets = torch.zeros((self.num_envs, 3), dtype=torch.float32, device=device)
        self.target_mask = torch.zeros(
            (self.num_envs, self.height, self.width), dtype=torch.int32, device=device
        )
        self.target_depth = torch.full(
            (self.num_envs, self.height, self.width), self.max_range,
            dtype=torch.float32, device=device,
        )
        self.obstacle_depth = torch.full(
            (self.num_envs, self.obstacle_height, self.obstacle_width),
            self.obstacle_max_range,
            dtype=torch.float32,
            device=device,
        )
        self._origins_wp = wp.from_torch(self._origins, dtype=wp.vec3)
        self._orientations_wp = wp.from_torch(self._orientations, dtype=wp.quat)
        self._targets_wp = wp.from_torch(self._targets, dtype=wp.vec3)
        self._mask_wp = wp.from_torch(self.target_mask, dtype=wp.int32)
        self._depth_wp = wp.from_torch(self.target_depth, dtype=wp.float32)
        self._obstacle_depth_wp = wp.from_torch(self.obstacle_depth, dtype=wp.float32)

        self._u_grid = uu.unsqueeze(0)
        self._v_grid = vv.unsqueeze(0)
        self.camera_offset_vehicle = torch.tensor(
            getattr(vis_cfg, "camera_translation", [0.10, 0.0, 0.03]),
            dtype=torch.float32,
            device=device,
        ).view(1, 3)

        self.last_bearing_sin = torch.zeros(self.num_envs, device=device)
        self.last_bearing_cos = torch.zeros(self.num_envs, device=device)
        self.time_since_seen = torch.full((self.num_envs,), self.memory_s, device=device)
        self.last_bbox = torch.full((self.num_envs, 4), -1.0, device=device)
        self.last_pixel_count = torch.zeros(self.num_envs, dtype=torch.long, device=device)

    def reset_idx(self, env_ids):
        self.last_bearing_sin[env_ids] = 0.0
        self.last_bearing_cos[env_ids] = 0.0
        self.time_since_seen[env_ids] = self.memory_s
        self.last_bbox[env_ids] = -1.0
        self.last_pixel_count[env_ids] = 0

    def _render(self, drone_pos_w, vehicle_quat, target_pos_w):
        offset = self.camera_offset_vehicle.expand(self.num_envs, -1)
        self._origins[:] = drone_pos_w + quat_rotate(vehicle_quat, offset)
        self._orientations[:] = vehicle_quat
        self._targets[:] = target_pos_w
        wp.launch(
            kernel=_render_target_camera_kernel,
            dim=(self.num_envs, self.height, self.width),
            inputs=[
                self.mesh_ids,
                self._origins_wp,
                self._orientations_wp,
                self._ray_vectors_wp,
                self._targets_wp,
                self.target_radius,
                self.max_range,
                self._mask_wp,
                self._depth_wp,
            ],
            device=str(self.device),
        )
        wp.launch(
            kernel=_render_obstacle_depth_kernel,
            dim=(self.num_envs, self.obstacle_height, self.obstacle_width),
            inputs=[
                self.mesh_ids,
                self._origins_wp,
                self._orientations_wp,
                self._obstacle_ray_vectors_wp,
                self.obstacle_max_range,
                self._obstacle_depth_wp,
            ],
            device=str(self.device),
        )

    def render_raw_rgbd(self, drone_pos_w, vehicle_quat, target_pos_w):
        """Render an RGB-D camera frame; semantic buffers stay renderer-private.

        A simulator necessarily uses scene pose/geometry to rasterize pixels.  The information
        firewall is downstream: perception receives only ``rgb`` and ``depth`` returned here,
        never ``target_mask`` or ``target_pos_w``.  Bars use a neutral depth-shaded appearance and
        the target drone uses a red appearance matching its URDF color.  This is deliberately a
        simple sim renderer; color/measurement perturbations are applied by the perception module.
        """
        self._render(drone_pos_w, vehicle_quat, target_pos_w)

        obstacle_depth_hi = F.interpolate(
            self.obstacle_depth.unsqueeze(1),
            size=(self.height, self.width),
            mode="bilinear",
            align_corners=False,
        ).squeeze(1)
        obstacle_depth_hi = torch.nan_to_num(
            obstacle_depth_hi,
            nan=self.obstacle_max_range,
            posinf=self.obstacle_max_range,
            neginf=self.obstacle_max_range,
        ).clamp(0.0, self.obstacle_max_range)

        # Neutral background/obstacle texture. It contains geometry cues but no semantic ID.
        proximity = (1.0 - obstacle_depth_hi / self.obstacle_max_range).clamp(0.0, 1.0)
        luminance = 0.08 + 0.42 * proximity
        rgb = torch.stack(
            [luminance * 0.92, luminance, luminance * 1.05], dim=1
        )
        depth = obstacle_depth_hi.clone()

        # Renderer-only class mask paints the visible target mesh appearance. The mask itself is
        # never returned to the perception module or actor.
        visible_target_pixels = self.target_mask > 0
        target_color = torch.tensor(
            [0.88, 0.08, 0.045], dtype=rgb.dtype, device=rgb.device
        ).view(1, 3, 1, 1)
        rgb = torch.where(visible_target_pixels.unsqueeze(1), target_color, rgb)
        depth = torch.where(visible_target_pixels, self.target_depth, depth)
        return rgb.contiguous(), depth.contiguous()

    def detect(self, drone_pos_w, vehicle_quat, target_pos_w, update_tracker=True):
        """Return the existing 8-D interface, derived exclusively from camera pixels."""
        self._render(drone_pos_w, vehicle_quat, target_pos_w)
        mask = self.target_mask > 0
        count = mask.sum(dim=(1, 2))
        visible = count >= self.min_pixels
        denom = count.clamp(min=1).float()

        mask_f = mask.float()
        u_center = (mask_f * self._u_grid).sum(dim=(1, 2)) / denom
        v_center = (mask_f * self._v_grid).sum(dim=(1, 2)) / denom
        bearing = torch.atan((self.cx - u_center) / self.fx)
        elevation = torch.atan((self.cy - v_center) / self.fy)
        surface_range = (self.target_depth * mask_f).sum(dim=(1, 2)) / denom

        # Pixel bounding box is kept for diagnostics and future confidence/noise models.
        u_min = torch.where(mask, self._u_grid.long(), self.width).amin(dim=(1, 2)).float()
        u_max = torch.where(mask, self._u_grid.long(), -1).amax(dim=(1, 2)).float()
        v_min = torch.where(mask, self._v_grid.long(), self.height).amin(dim=(1, 2)).float()
        v_max = torch.where(mask, self._v_grid.long(), -1).amax(dim=(1, 2)).float()
        bbox = torch.stack([u_min, v_min, u_max, v_max], dim=1)
        self.last_bbox[:] = torch.where(visible.unsqueeze(1), bbox, -torch.ones_like(bbox))
        self.last_pixel_count[:] = count

        if update_tracker:
            self.time_since_seen += self.step_dt
            self.last_bearing_sin = torch.where(visible, torch.sin(bearing), self.last_bearing_sin)
            self.last_bearing_cos = torch.where(visible, torch.cos(bearing), self.last_bearing_cos)
            self.time_since_seen = torch.where(
                visible, torch.zeros_like(self.time_since_seen), self.time_since_seen
            )

        vis_f = visible.float()
        vec = torch.stack(
            [
                vis_f,
                vis_f * torch.sin(bearing),
                vis_f * torch.cos(bearing),
                vis_f * elevation / self.half_vfov,
                torch.where(
                    visible,
                    (surface_range / self.max_range).clamp(0.0, 1.0),
                    torch.ones_like(surface_range),
                ),
                self.last_bearing_sin,
                self.last_bearing_cos,
                (self.time_since_seen / self.memory_s).clamp(max=1.0),
            ],
            dim=1,
        )
        return vec, visible
