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
import os

import torch
import torch.nn.functional as F
import warp as wp

from aerial_gym.utils.math import quat_mul, quat_rotate


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
    target_orientations: wp.array(dtype=wp.quat),
    target_radius: float,
    target_half_extents: wp.vec3,
    target_use_oriented_box: int,
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

    t_target = -1.0
    if target_use_oriented_box != 0:
        q_inv = wp.quat_inverse(target_orientations[env_id])
        ro_l = wp.quat_rotate(q_inv, ro - target_positions[env_id])
        rd_l = wp.quat_rotate(q_inv, rd)
        tmin = -1.0e20
        tmax = 1.0e20
        valid = int(1)
        if wp.abs(rd_l[0]) < 1.0e-8:
            if wp.abs(ro_l[0]) > target_half_extents[0]: valid = int(0)
        else:
            a = (-target_half_extents[0] - ro_l[0]) / rd_l[0]
            b0 = (target_half_extents[0] - ro_l[0]) / rd_l[0]
            tmin = wp.max(tmin, wp.min(a, b0)); tmax = wp.min(tmax, wp.max(a, b0))
        if wp.abs(rd_l[1]) < 1.0e-8:
            if wp.abs(ro_l[1]) > target_half_extents[1]: valid = int(0)
        else:
            a = (-target_half_extents[1] - ro_l[1]) / rd_l[1]
            b0 = (target_half_extents[1] - ro_l[1]) / rd_l[1]
            tmin = wp.max(tmin, wp.min(a, b0)); tmax = wp.min(tmax, wp.max(a, b0))
        if wp.abs(rd_l[2]) < 1.0e-8:
            if wp.abs(ro_l[2]) > target_half_extents[2]: valid = int(0)
        else:
            a = (-target_half_extents[2] - ro_l[2]) / rd_l[2]
            b0 = (target_half_extents[2] - ro_l[2]) / rd_l[2]
            tmin = wp.max(tmin, wp.min(a, b0)); tmax = wp.min(tmax, wp.max(a, b0))
        if valid != 0 and tmax >= wp.max(tmin, 0.0):
            t_target = tmin
            if t_target < 0.0: t_target = tmax
    else:
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


def _rotate_hue_rgb(rgb, radians):
    """Rotate RGB rows (N,3) about the grey axis by per-row angles (N,) -- a pure hue shift.

    Rodrigues rotation about (1,1,1)/sqrt(3): luminance-neutral, so it changes WHAT colour the
    target is without changing how bright it is. Zero angle returns the input exactly.
    """
    axis = rgb.new_tensor([1.0, 1.0, 1.0]) / math.sqrt(3.0)
    cos = torch.cos(radians).unsqueeze(1)
    sin = torch.sin(radians).unsqueeze(1)
    cross = torch.linalg.cross(axis.expand_as(rgb), rgb, dim=1)
    dot = (rgb * axis).sum(dim=1, keepdim=True)
    return (rgb * cos + cross * sin + axis * dot * (1.0 - cos)).clamp(0.0, 1.0)


def _small_random_quat(max_angle_rad, n, device):
    """n random unit quaternions (xyzw) with rotation angle uniform in [-max, +max]."""
    axis = torch.randn(n, 3, device=device)
    axis = axis / axis.norm(dim=1, keepdim=True).clamp(min=1e-9)
    half = (torch.rand(n, device=device) * 2.0 - 1.0) * max_angle_rad * 0.5
    return torch.cat(
        [axis * torch.sin(half).unsqueeze(1), torch.cos(half).unsqueeze(1)], dim=1
    )


class _DetectResolutionChannel:
    """One-frame hand-off of the high-resolution detection summary, renderer -> perception.

    Why a module-level channel rather than an argument: ``render_raw_rgbd`` returns exactly
    ``(rgb, depth)`` and ``NavRLPerceptionModule.observe`` takes no detector handle -- both
    signatures are consumed by files this change may not touch. The channel carries the frame
    between them and ``consume`` EMPTIES it, so a perception step can never silently read a
    stale frame: a second consume without an intervening publish raises.

    The payload is a REDUCED summary -- pixel count, integer centroid sums, depth sum, and the
    flat RGB value a target pixel carries -- never an image and never a semantic mask. Perception
    still decides what is a target by running its OWN segmenter on that RGB value. This is the
    same information a high-resolution RGB-D render would have handed it, in the sparse form the
    zero-perturbation renderer makes exact (see the fail-closed list in NavRLTargetDetector).
    """

    def __init__(self):
        self._frame = None
        self.published = 0
        self.consumed = 0

    def publish(self, frame):
        self._frame = frame
        self.published += 1

    def consume(self):
        if self._frame is None:
            raise RuntimeError(
                "NavRL detect-resolution channel is empty: perception asked for a "
                "detect-resolution frame that the renderer never published (or that was "
                "already consumed this step). The renderer must run render_raw_rgbd() exactly "
                "once per perception observe()."
            )
        frame = self._frame
        self._frame = None
        self.consumed += 1
        return frame

    def clear(self):
        self._frame = None


# Process-wide singleton. Only ever written by a detector whose detect resolution differs from
# its camera resolution, and only ever read by the matching perception module.
DETECT_CHANNEL = _DetectResolutionChannel()

# Peak pixels (envs x rows x cols) materialised at once by the detect-resolution render. The
# detect render is row-blocked to this budget so its VRAM is bounded by the budget instead of
# num_envs*detect_width*detect_height: at 128 envs x 1920x1200 a full-frame mask+depth pair
# would be 2.4 GB on an 8 GB card. Per-row partial sums are stored, so the reduction result does
# NOT depend on the block size -- the knob trades peak VRAM against the number of launches and
# cannot change a number the policy sees.
DETECT_PIXEL_BUDGET = int(
    os.environ.get("NAVRL_DETECT_PIXEL_BUDGET", "").strip() or 16_000_000
)
if DETECT_PIXEL_BUDGET <= 0:
    raise ValueError("NAVRL_DETECT_PIXEL_BUDGET must be positive")


class NavRLTargetDetector:
    """Pixel-derived camera detection plus short detector-side tracking memory."""

    def __init__(self, warp_env, num_envs, device, vis_cfg, step_dt):
        self.num_envs = int(num_envs)
        self.device = device
        self.width = int(getattr(vis_cfg, "camera_width", 160))
        self.height = int(getattr(vis_cfg, "camera_height", 90))
        # Detection resolution. Equal to the camera resolution by default, in which case
        # `detect_decoupled` is False and EVERY code path below is exactly the historical one:
        # no extra ray table, no extra buffers, no extra kernel launch, no extra arithmetic.
        self.detect_width = int(getattr(vis_cfg, "detect_width", self.width))
        self.detect_height = int(getattr(vis_cfg, "detect_height", self.height))
        if self.detect_width <= 0 or self.detect_height <= 0:
            raise ValueError(
                "NAVRL_DETECT_WIDTH/HEIGHT must be positive, got %dx%d"
                % (self.detect_width, self.detect_height)
            )
        if self.detect_width < self.width or self.detect_height < self.height:
            raise ValueError(
                "detect resolution %dx%d is smaller than the camera resolution %dx%d. The "
                "decoupling is only defined upwards: a detect resolution below the RGB "
                "resolution would DISCARD detections the RGB path can already see."
                % (self.detect_width, self.detect_height, self.width, self.height)
            )
        self.detect_decoupled = (self.detect_width != self.width) or (
            self.detect_height != self.height
        )
        self.max_range = float(vis_cfg.detector_max_range)
        self.hfov = math.radians(float(vis_cfg.detector_hfov_deg))
        self.vfov = math.radians(float(vis_cfg.detector_vfov_deg))
        self.half_hfov = self.hfov * 0.5
        self.half_vfov = self.vfov * 0.5
        self.target_radius = float(getattr(vis_cfg, "camera_target_radius", 0.15))
        self.target_use_oriented_box = (
            os.environ.get("NAVRL_TARGET_DYNAMICS", "legacy").strip().lower() == "physical"
        )
        self.target_half_extents = wp.vec3(0.14, 0.14, 0.06)
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

        # Intrinsics of the DETECT image. fx = (W/2)/tan(hfov/2) is resolution-dependent, so a
        # centroid measured on the detect-resolution mask must be converted to an angle with
        # THESE, and a centroid measured on the camera-resolution image with self.fx/cx above.
        # Mixing them scales the bearing by detect_width/width -- a silent bearing bias. When the
        # two resolutions are equal these are the same expression on the same operands and
        # therefore the same floats.
        self.detect_fx = self.detect_width / (2.0 * math.tan(self.half_hfov))
        self.detect_fy = self.detect_height / (2.0 * math.tan(self.half_vfov))
        self.detect_cx = (self.detect_width - 1) * 0.5
        self.detect_cy = (self.detect_height - 1) * 0.5

        # Renderer-side FOV calibration error: the RAY TABLE is built from a scaled FOV while
        # self.fx/fy above (used by consumers to interpret pixels) stay nominal -- so every
        # back-projection downstream uses the wrong camera model, exactly like a real
        # mis-calibration. Per-run, because the table is baked once and uploaded to Warp.
        self.fov_scale_err = float(getattr(vis_cfg, "camera_fov_scale_err", 0.0))
        render_half_hfov = self.half_hfov * (1.0 + self.fov_scale_err)
        render_half_vfov = self.half_vfov * (1.0 + self.fov_scale_err)
        render_fx = self.width / (2.0 * math.tan(render_half_hfov))
        render_fy = self.height / (2.0 * math.tan(render_half_vfov))

        # Vehicle camera frame: +x forward, +y left, +z up. Image u grows right and v down.
        rows = torch.arange(self.height, device=device, dtype=torch.float32)
        cols = torch.arange(self.width, device=device, dtype=torch.float32)
        vv, uu = torch.meshgrid(rows, cols, indexing="ij")
        rays = torch.stack(
            [
                torch.ones_like(uu),
                -(uu - self.cx) / render_fx,
                -(vv - self.cy) / render_fy,
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
        obstacle_fx = self.obstacle_width / (2.0 * math.tan(render_half_hfov))
        obstacle_fy = self.obstacle_height / (2.0 * math.tan(render_half_vfov))
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
        self._target_orientations = torch.zeros(
            (self.num_envs, 4), dtype=torch.float32, device=device
        )
        self._target_orientations[:, 3] = 1.0
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
        self._target_orientations_wp = wp.from_torch(
            self._target_orientations, dtype=wp.quat
        )
        self._mask_wp = wp.from_torch(self.target_mask, dtype=wp.int32)
        self._depth_wp = wp.from_torch(self.target_depth, dtype=wp.float32)
        self._obstacle_depth_wp = wp.from_torch(self.obstacle_depth, dtype=wp.float32)

        # -- detect-resolution target render (only when the two resolutions differ).
        # The SAME kernel, the same origins/orientations and the same target geometry are used;
        # only the ray table changes. The table is built from the RENDER half-FOV, exactly like
        # the camera table above, so a camera_fov_scale_err mis-calibration is reproduced at the
        # detect resolution too (and self.detect_fx/cx stay nominal, so consumers keep
        # back-projecting with the wrong model, which is the point of that knob).
        if self.detect_decoupled:
            detect_rows = torch.arange(self.detect_height, device=device, dtype=torch.float32)
            detect_cols = torch.arange(self.detect_width, device=device, dtype=torch.float32)
            detect_vv, detect_uu = torch.meshgrid(detect_rows, detect_cols, indexing="ij")
            detect_render_fx = self.detect_width / (2.0 * math.tan(render_half_hfov))
            detect_render_fy = self.detect_height / (2.0 * math.tan(render_half_vfov))
            detect_ray = torch.stack(
                [
                    torch.ones_like(detect_uu),
                    -(detect_uu - self.detect_cx) / detect_render_fx,
                    -(detect_vv - self.detect_cy) / detect_render_fy,
                ],
                dim=-1,
            )
            self._detect_rays = (
                detect_ray / detect_ray.norm(dim=-1, keepdim=True).clamp(min=1e-9)
            ).contiguous()
            del detect_ray, detect_vv, detect_uu

            rows_per_block = DETECT_PIXEL_BUDGET // max(1, self.num_envs * self.detect_width)
            rows_per_block = int(min(self.detect_height, max(1, rows_per_block)))
            self.detect_rows_per_block = rows_per_block
            # One warp view per row block, built once: the ray table never changes, so no wp
            # array is created on the hot path. The slices are views of self._detect_rays, which
            # keeps the storage alive.
            self._detect_ray_blocks = []
            for row0 in range(0, self.detect_height, rows_per_block):
                rows = min(rows_per_block, self.detect_height - row0)
                self._detect_ray_blocks.append(
                    (
                        row0,
                        rows,
                        wp.from_torch(self._detect_rays[row0 : row0 + rows], dtype=wp.vec3),
                    )
                )
            self._detect_mask_block = torch.zeros(
                (self.num_envs, rows_per_block, self.detect_width),
                dtype=torch.int32,
                device=device,
            )
            self._detect_depth_block = torch.full(
                (self.num_envs, rows_per_block, self.detect_width),
                self.max_range,
                dtype=torch.float32,
                device=device,
            )
            self._detect_mask_block_wp = wp.from_torch(self._detect_mask_block, dtype=wp.int32)
            self._detect_depth_block_wp = wp.from_torch(
                self._detect_depth_block, dtype=wp.float32
            )
            # Per-ROW partial sums. Summing within a row and then across rows makes the reduction
            # independent of the block size (the integer sums are exact regardless).
            self._detect_row_count = torch.zeros(
                (self.num_envs, self.detect_height), dtype=torch.int64, device=device
            )
            self._detect_row_depth = torch.zeros(
                (self.num_envs, self.detect_height), dtype=torch.float32, device=device
            )
            self._detect_col_count = torch.zeros(
                (self.num_envs, self.detect_width), dtype=torch.int64, device=device
            )
            self._detect_u_index = torch.arange(
                self.detect_width, dtype=torch.int64, device=device
            ).view(1, -1)
            self._detect_v_index = torch.arange(
                self.detect_height, dtype=torch.int64, device=device
            ).view(1, -1)
            # Reduced products, refreshed by every _render_detect().
            self.detect_count = torch.zeros(self.num_envs, dtype=torch.int64, device=device)
            self.detect_u_sum = torch.zeros(self.num_envs, device=device)
            self.detect_v_sum = torch.zeros(self.num_envs, device=device)
            self.detect_depth_sum = torch.zeros(self.num_envs, device=device)
            self.detect_bbox = torch.full((self.num_envs, 4), -1.0, device=device)

        self._u_grid = uu.unsqueeze(0)
        self._v_grid = vv.unsqueeze(0)
        self.camera_offset_vehicle = torch.tensor(
            getattr(vis_cfg, "camera_translation", [0.10, 0.0, 0.03]),
            dtype=torch.float32,
            device=device,
        ).view(1, 3)

        # -- appearance domain shift state (검증 2). Nominal literals live HERE once; the paint
        # path below reads only these buffers, so zero knobs reproduce the historical render
        # bit-for-bit and non-zero knobs are per-episode draws (resampled in reset_idx).
        self.app_hue_deg = float(getattr(vis_cfg, "appearance_hue_deg", 0.0))
        self.app_light_gain = float(getattr(vis_cfg, "appearance_light_gain", 0.0))
        self.app_albedo_jitter = float(getattr(vis_cfg, "appearance_albedo_jitter", 0.0))
        self.app_texture_std = float(getattr(vis_cfg, "appearance_texture_std", 0.0))
        self.app_motion_blur = float(getattr(vis_cfg, "appearance_motion_blur", 0.0))
        self.mount_rot_deg = float(getattr(vis_cfg, "camera_mount_rot_deg", 0.0))
        self.mount_trans_m = float(getattr(vis_cfg, "camera_mount_trans_m", 0.0))
        if self.detect_decoupled:
            self._assert_detect_decoupling_is_equivalent()
        self._nominal_target_color = torch.tensor(
            [0.88, 0.08, 0.045], dtype=torch.float32, device=device
        )
        self._nominal_tint = torch.tensor(
            [0.92, 1.00, 1.05], dtype=torch.float32, device=device
        )
        self.target_color = self._nominal_target_color.view(1, 3).repeat(self.num_envs, 1)
        self.albedo_base = torch.full((self.num_envs, 1, 1), 0.08, device=device)
        self.albedo_gain = torch.full((self.num_envs, 1, 1), 0.42, device=device)
        self.albedo_tint = self._nominal_tint.view(1, 3, 1, 1).repeat(self.num_envs, 1, 1, 1)
        self.texture_field = torch.ones(self.num_envs, self.height, self.width, device=device)
        self.light_gain = torch.ones(self.num_envs, 1, 1, 1, device=device)
        self.mount_quat = torch.zeros(self.num_envs, 4, device=device)
        self.mount_quat[:, 3] = 1.0
        self.mount_trans = torch.zeros(self.num_envs, 3, device=device)
        self._blur_prev = torch.zeros(
            self.num_envs, 3, self.height, self.width, device=device
        )
        self._blur_valid = torch.zeros(self.num_envs, dtype=torch.bool, device=device)
        self._resample_appearance(torch.arange(self.num_envs, device=device))

        self.last_bearing_sin = torch.zeros(self.num_envs, device=device)
        self.last_bearing_cos = torch.zeros(self.num_envs, device=device)
        self.time_since_seen = torch.full((self.num_envs,), self.memory_s, device=device)
        self.last_bbox = torch.full((self.num_envs, 4), -1.0, device=device)
        self.last_pixel_count = torch.zeros(self.num_envs, dtype=torch.long, device=device)

    def _assert_detect_decoupling_is_equivalent(self):
        """Refuse to decouple the resolutions whenever the RGB round-trip stops being an identity.

        Decoupling replaces "render a high-resolution RGB image, then segment it" with "read the
        high-resolution target mask the ray-cast already produced". That substitution is exact
        only while the renderer paints target pixels a flat, per-env colour over a background the
        colour rule never fires on. Each knob below breaks it in a way a per-env scalar cannot
        reproduce, so it is refused rather than silently approximated:

          appearance_hue_deg      target colour varies per env -> still flat per env, BUT it can
                                  cross the segmenter's decision boundary, so which pixels are
                                  target pixels is no longer "all of them".
          appearance_light_gain   multiplies target AND background after the paint, so the
                                  background can cross the boundary and the target can fall off
                                  it; neither is expressible as "the mask, times a constant".
          appearance_albedo_jitter background reflectance moves toward the target's colour.
          appearance_texture_std  per-PIXEL luminance field: not constant over the mask, and
                                  drawn at the camera resolution, so it has no high-resolution
                                  counterpart at all.
          appearance_motion_blur  mixes the PREVIOUS frame's image into this one; a per-frame
                                  scalar summary cannot carry a per-pixel temporal trail.

        Checked and deliberately ALLOWED, because they perturb both resolutions identically:
          camera_mount_rot_deg / camera_mount_trans_m -- the detect render uses the same
            perturbed origin/orientation as the camera render, so the high-resolution mask is
            exactly what a high-resolution render with the same mount error would produce.
          camera_fov_scale_err -- the detect ray table is built from the same scaled render FOV
            while self.detect_fx/cx stay nominal, reproducing the mis-calibration one-for-one.
          NAVRL_TARGET_DYNAMICS=physical (oriented-box target) -- same kernel, same geometry.
        """
        offenders = []
        for name, value in (
            ("appearance_hue_deg", self.app_hue_deg),
            ("appearance_light_gain", self.app_light_gain),
            ("appearance_albedo_jitter", self.app_albedo_jitter),
            ("appearance_texture_std", self.app_texture_std),
            ("appearance_motion_blur", self.app_motion_blur),
        ):
            if value > 0.0:
                offenders.append("%s=%g" % (name, value))
        if offenders:
            raise RuntimeError(
                "NavRL detect-resolution decoupling (%dx%d detect vs %dx%d camera) is NOT "
                "equivalent under appearance perturbation: %s. The decoupling replaces the "
                "high-resolution RGB render + segmentation with the high-resolution target "
                "mask, which is an identity only while the target is painted a flat colour "
                "over a background the colour rule never fires on. Set the appearance knobs to "
                "zero, or set NAVRL_DETECT_WIDTH/HEIGHT equal to NAVRL_CAMERA_WIDTH/HEIGHT."
                % (
                    self.detect_width,
                    self.detect_height,
                    self.width,
                    self.height,
                    ", ".join(offenders),
                )
            )

    def _render_detect(self):
        """Ray-cast the target at DETECT resolution and reduce it to per-env scalars.

        Row-blocked to DETECT_PIXEL_BUDGET pixels so peak VRAM does not scale with
        num_envs*detect_width*detect_height. Per-row partial sums are written to (N, H) buffers
        and summed once at the end, so the block size changes nothing about the result.

        No high-resolution image is ever built: the products are a pixel count, two integer
        centroid sums (exact), a depth sum, and a bounding box.
        """
        self._detect_col_count.zero_()
        for row0, rows, ray_block in self._detect_ray_blocks:
            wp.launch(
                kernel=_render_target_camera_kernel,
                dim=(self.num_envs, rows, self.detect_width),
                inputs=[
                    self.mesh_ids,
                    self._origins_wp,
                    self._orientations_wp,
                    ray_block,
                    self._targets_wp,
                    self._target_orientations_wp,
                    self.target_radius,
                    self.target_half_extents,
                    int(self.target_use_oriented_box),
                    self.max_range,
                    self._detect_mask_block_wp,
                    self._detect_depth_block_wp,
                ],
                device=str(self.device),
            )
            mask_block = self._detect_mask_block[:, :rows]
            depth_block = self._detect_depth_block[:, :rows]
            self._detect_row_count[:, row0 : row0 + rows] = mask_block.sum(dim=2)
            self._detect_row_depth[:, row0 : row0 + rows] = (depth_block * mask_block).sum(dim=2)
            self._detect_col_count += mask_block.sum(dim=1)

        count = self._detect_row_count.sum(dim=1)
        # Integer centroid sums: exact, and independent of reduction order.
        self.detect_count = count
        self.detect_u_sum = (self._detect_col_count * self._detect_u_index).sum(dim=1).float()
        self.detect_v_sum = (self._detect_row_count * self._detect_v_index).sum(dim=1).float()
        self.detect_depth_sum = self._detect_row_depth.sum(dim=1)

        occupied_cols = self._detect_col_count > 0
        occupied_rows = self._detect_row_count > 0
        any_pixel = count > 0
        u_min = occupied_cols.to(torch.float32).argmax(dim=1)
        u_max = self.detect_width - 1 - occupied_cols.to(torch.float32).flip(1).argmax(dim=1)
        v_min = occupied_rows.to(torch.float32).argmax(dim=1)
        v_max = self.detect_height - 1 - occupied_rows.to(torch.float32).flip(1).argmax(dim=1)
        # Same empty-mask convention as the camera-resolution path: min = size, max = -1.
        u_min = torch.where(any_pixel, u_min, torch.full_like(u_min, self.detect_width))
        u_max = torch.where(any_pixel, u_max, torch.full_like(u_max, -1))
        v_min = torch.where(any_pixel, v_min, torch.full_like(v_min, self.detect_height))
        v_max = torch.where(any_pixel, v_max, torch.full_like(v_max, -1))
        self.detect_bbox = torch.stack(
            [u_min.float(), v_min.float(), u_max.float(), v_max.float()], dim=1
        )

        # Sparse hand-off to perception. RGB is the value a target pixel carries, so perception
        # can run its own segmenter on it rather than being told what is a target.
        DETECT_CHANNEL.publish(
            {
                "num_envs": self.num_envs,
                "width": self.detect_width,
                "height": self.detect_height,
                "far_plane": self.max_range,
                "count": self.detect_count,
                "u_sum": self.detect_u_sum,
                "v_sum": self.detect_v_sum,
                "depth_sum": self.detect_depth_sum,
                "rgb": self.target_color.clamp(0.0, 1.0),
                "depth_probe": self.detect_depth_sum / count.clamp(min=1).float(),
            }
        )

    def _resample_appearance(self, env_ids):
        """Per-episode appearance draw. Knobs at zero leave every nominal buffer untouched."""
        if not isinstance(env_ids, torch.Tensor):
            env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        n = int(env_ids.numel())
        if n == 0:
            return
        device = self.device
        if self.app_hue_deg > 0.0:
            angles = (
                (torch.rand(n, device=device) * 2.0 - 1.0)
                * math.radians(self.app_hue_deg)
            )
            self.target_color[env_ids] = _rotate_hue_rgb(
                self._nominal_target_color.view(1, 3).expand(n, 3), angles
            )
        if self.app_albedo_jitter > 0.0:
            jitter = self.app_albedo_jitter

            def uniform(shape):
                return 1.0 + (torch.rand(*shape, device=device) * 2.0 - 1.0) * jitter

            self.albedo_base[env_ids] = 0.08 * uniform((n, 1, 1))
            self.albedo_gain[env_ids] = 0.42 * uniform((n, 1, 1))
            self.albedo_tint[env_ids] = self._nominal_tint.view(1, 3, 1, 1) * uniform(
                (n, 3, 1, 1)
            )
        if self.app_texture_std > 0.0:
            self.texture_field[env_ids] = (
                1.0
                + torch.randn(n, self.height, self.width, device=device)
                * self.app_texture_std
            ).clamp(min=0.0)
        if self.app_light_gain > 0.0:
            self.light_gain[env_ids] = 1.0 + (
                torch.rand(n, 1, 1, 1, device=device) * 2.0 - 1.0
            ) * self.app_light_gain
        if self.mount_rot_deg > 0.0:
            self.mount_quat[env_ids] = _small_random_quat(
                math.radians(self.mount_rot_deg), n, device
            )
        if self.mount_trans_m > 0.0:
            self.mount_trans[env_ids] = (
                torch.rand(n, 3, device=device) * 2.0 - 1.0
            ) * self.mount_trans_m
        # A new episode must not inherit the previous episode's frame through the blur EMA.
        self._blur_valid[env_ids] = False

    def reset_idx(self, env_ids):
        self._resample_appearance(env_ids)
        self.last_bearing_sin[env_ids] = 0.0
        self.last_bearing_cos[env_ids] = 0.0
        self.time_since_seen[env_ids] = self.memory_s
        self.last_bbox[env_ids] = -1.0
        self.last_pixel_count[env_ids] = 0

    def _render(self, drone_pos_w, vehicle_quat, target_pos_w, target_quat=None):
        offset = self.camera_offset_vehicle.expand(self.num_envs, -1)
        if self.mount_trans_m > 0.0:
            # Renderer-only mount error: perception keeps its nominal camera_offset copy, so the
            # rendered geometry and the back-projection model disagree -- a real extrinsic
            # mis-calibration. (Perturbing both copies would cancel and measure nothing.)
            offset = offset + self.mount_trans
        self._origins[:] = drone_pos_w + quat_rotate(vehicle_quat, offset)
        if self.mount_rot_deg > 0.0:
            self._orientations[:] = quat_mul(vehicle_quat, self.mount_quat)
        else:
            self._orientations[:] = vehicle_quat
        self._targets[:] = target_pos_w
        if target_quat is not None:
            self._target_orientations[:] = target_quat
        wp.launch(
            kernel=_render_target_camera_kernel,
            dim=(self.num_envs, self.height, self.width),
            inputs=[
                self.mesh_ids,
                self._origins_wp,
                self._orientations_wp,
                self._ray_vectors_wp,
                self._targets_wp,
                self._target_orientations_wp,
                self.target_radius,
                self.target_half_extents,
                int(self.target_use_oriented_box),
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
        if self.detect_decoupled:
            self._render_detect()

    def render_raw_rgbd(self, drone_pos_w, vehicle_quat, target_pos_w, target_quat=None):
        """Render an RGB-D camera frame; semantic buffers stay renderer-private.

        A simulator necessarily uses scene pose/geometry to rasterize pixels.  The information
        firewall is downstream: perception receives only ``rgb`` and ``depth`` returned here,
        never ``target_mask`` or ``target_pos_w``.  Bars use a neutral depth-shaded appearance and
        the target drone uses a red appearance matching its URDF color.  This is deliberately a
        simple sim renderer; color/measurement perturbations are applied by the perception module.
        """
        self._render(drone_pos_w, vehicle_quat, target_pos_w, target_quat)

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
        # Reflectance comes from the per-env appearance buffers; at zero knobs they hold exactly
        # the historical literals (base 0.08, gain 0.42, tint 0.92/1.00/1.05).
        proximity = (1.0 - obstacle_depth_hi / self.obstacle_max_range).clamp(0.0, 1.0)
        luminance = self.albedo_base + self.albedo_gain * proximity
        if self.app_texture_std > 0.0:
            luminance = luminance * self.texture_field
        rgb = luminance.unsqueeze(1) * self.albedo_tint
        depth = obstacle_depth_hi.clone()

        # Renderer-only class mask paints the visible target mesh appearance. The mask itself is
        # never returned to the perception module or actor.
        visible_target_pixels = self.target_mask > 0
        rgb = torch.where(
            visible_target_pixels.unsqueeze(1), self.target_color.view(-1, 3, 1, 1), rgb
        )
        depth = torch.where(visible_target_pixels, self.target_depth, depth)
        # Global illumination multiplies AFTER the target paint so it hits target and background
        # alike -- that is what a lighting change does, and what the fixed red rule in the
        # bootstrap segmenter has never seen.
        if self.app_light_gain > 0.0:
            rgb = rgb * self.light_gain
        if self.app_motion_blur > 0.0:
            # Exponential trail: cheap, temporal, and directionally correct for a rolling camera.
            # Depth is deliberately NOT blurred -- a depth sensor does not blur like an RGB
            # exposure, and the depth channel has its own noise knob.
            prev = torch.where(
                self._blur_valid.view(-1, 1, 1, 1), self._blur_prev, rgb
            )
            rgb = (1.0 - self.app_motion_blur) * rgb + self.app_motion_blur * prev
            self._blur_prev.copy_(rgb)
            self._blur_valid[:] = True
        rgb = rgb.clamp(0.0, 1.0)
        return rgb.contiguous(), depth.contiguous()

    def detect(
        self, drone_pos_w, vehicle_quat, target_pos_w, target_quat=None, update_tracker=True
    ):
        """Return the existing 8-D interface, derived exclusively from camera pixels."""
        self._render(drone_pos_w, vehicle_quat, target_pos_w, target_quat)
        if self.detect_decoupled:
            # Every pixel-derived quantity comes from the DETECT-resolution mask and is converted
            # to an angle with the DETECT intrinsics. min_pixels is applied at that resolution
            # too -- that is the whole point of the knob: at 160x90 the 0.30 m target spans
            # 1.27 px at 20 m, at 1920x1200 it spans 15.2 px. The bounding box is likewise in
            # detect-resolution pixel coordinates (diagnostics only; nothing consumes it).
            count = self.detect_count
            visible = count >= self.min_pixels
            denom = count.clamp(min=1).float()
            u_center = self.detect_u_sum / denom
            v_center = self.detect_v_sum / denom
            bearing = torch.atan((self.detect_cx - u_center) / self.detect_fx)
            elevation = torch.atan((self.detect_cy - v_center) / self.detect_fy)
            surface_range = self.detect_depth_sum / denom
            bbox = self.detect_bbox
        else:
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
