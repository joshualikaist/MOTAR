"""Modeled onboard target detector for the Phase-3 vision pivot (NAVRL_VISION=1).

A perfect-detection abstraction of a forward segmentation camera: the policy receives the
target's bearing/elevation/range ONLY when a real camera would see it --
  (a) inside the camera FOV (the camera is gimbal-level, matching the yaw-only LiDAR attach),
  (b) within detector range, and
  (c) with unobstructed line of sight (a single warp ray against the per-env bar mesh).
Everything else the policy knows about the target comes from the semantic LiDAR channel.
Stage 2 replaces this with the real depth+segmentation pixel pipeline (target_detector.py's
mask->centroid->bearing recipe) behind the same output interface.

A detector-side TRACKER (last-seen bearing + time-since-seen) provides the standard onboard
filter memory; NavRL itself runs an onboard tracker for dynamic obstacles.

Ground-truth positions are used INTERNALLY to evaluate visibility geometry -- exactly like a
renderer would -- but are exposed to the policy only when the visibility gate passes.
"""

import math

import torch
import warp as wp


@wp.kernel
def _los_blocked_kernel(
    mesh_ids: wp.array(dtype=wp.uint64),
    origins: wp.array(dtype=wp.vec3),
    directions: wp.array(dtype=wp.vec3),  # unit vectors toward the target
    max_dists: wp.array(dtype=float),     # ray length = dist_to_target - occlusion_margin
    blocked: wp.array(dtype=wp.int32),
):
    tid = wp.tid()
    t = float(0.0)
    u = float(0.0)
    v = float(0.0)
    sign = float(0.0)
    n = wp.vec3()
    f = int(0)
    hit = wp.mesh_query_ray(
        mesh_ids[tid], origins[tid], directions[tid], max_dists[tid], t, u, v, sign, n, f
    )
    if hit:
        blocked[tid] = 1
    else:
        blocked[tid] = 0


class NavRLTargetDetector:
    """Per-env visibility gate + detection vector + tracker. All buffers preallocated; the warp
    kernel is launched once per query on in-place-updated tensors (a single N-thread launch,
    negligible next to the LiDAR render)."""

    def __init__(self, warp_env, num_envs, device, vis_cfg, step_dt):
        self.num_envs = num_envs
        self.device = device
        self.max_range = float(vis_cfg.detector_max_range)
        self.half_hfov = math.radians(float(vis_cfg.detector_hfov_deg)) * 0.5
        self.half_vfov = math.radians(float(vis_cfg.detector_vfov_deg)) * 0.5
        self.occ_margin = float(vis_cfg.occlusion_margin)
        self.memory_s = max(1e-3, float(vis_cfg.tracker_memory_s))
        self.step_dt = float(step_dt)

        self.mesh_ids = wp.array(warp_env.CONST_WARP_MESH_ID_LIST, dtype=wp.uint64, device=device)
        self._origins = torch.zeros((num_envs, 3), dtype=torch.float32, device=device)
        self._dirs = torch.zeros((num_envs, 3), dtype=torch.float32, device=device)
        self._dists = torch.zeros(num_envs, dtype=torch.float32, device=device)
        self._blocked = torch.zeros(num_envs, dtype=torch.int32, device=device)
        self._origins_wp = wp.from_torch(self._origins, dtype=wp.vec3)
        self._dirs_wp = wp.from_torch(self._dirs, dtype=wp.vec3)
        self._dists_wp = wp.from_torch(self._dists, dtype=wp.float32)
        self._blocked_wp = wp.from_torch(self._blocked, dtype=wp.int32)

        # tracker state (reset per episode): last seen bearing (sin/cos) + time since seen
        self.last_bearing_sin = torch.zeros(num_envs, device=device)
        self.last_bearing_cos = torch.zeros(num_envs, device=device)
        self.time_since_seen = torch.full((num_envs,), self.memory_s, device=device)

    def reset_idx(self, env_ids):
        self.last_bearing_sin[env_ids] = 0.0
        self.last_bearing_cos[env_ids] = 0.0
        self.time_since_seen[env_ids] = self.memory_s  # "never seen" == saturated

    def _los_blocked(self, origins, rpos_w, dist):
        """1 where a bar blocks the ray origin->target (world frame). The ray is shortened by
        occlusion_margin so it cannot count the target's own surface as an occluder (the target
        is NOT in the bar mesh anyway -- it is the analytic sphere -- but the margin also absorbs
        capture-range geometry where origin ~ target)."""
        self._origins[:] = origins
        self._dirs[:] = rpos_w / dist.unsqueeze(1).clamp(min=1e-6)
        self._dists[:] = (dist - self.occ_margin).clamp(min=1e-3)
        wp.launch(
            _los_blocked_kernel,
            dim=self.num_envs,
            inputs=[self.mesh_ids, self._origins_wp, self._dirs_wp, self._dists_wp,
                    self._blocked_wp],
            device=str(self.device),
        )
        return self._blocked.bool()

    def detect(self, drone_pos_w, rpos_veh, rpos_w, update_tracker):
        """Compute the 8-dim detection vector.

        drone_pos_w:  (N, 3) world drone position (ray origin)
        rpos_veh:     (N, 3) target - drone in the VEHICLE (yaw-only) frame
        rpos_w:       (N, 3) target - drone in the world frame
        update_tracker: advance/refresh the tracker state (call with True exactly once per step
                        -- the obs pass; the reward pass calls visible_only()).

        Returns (vec (N, 8), visible (N,)).
        """
        dist = rpos_w.norm(dim=1)
        horiz = rpos_veh[:, :2].norm(dim=1).clamp(min=1e-6)
        bearing = torch.atan2(rpos_veh[:, 1], rpos_veh[:, 0])
        elevation = torch.atan2(rpos_veh[:, 2], horiz)

        in_fov = (bearing.abs() < self.half_hfov) & (elevation.abs() < self.half_vfov)
        in_range = dist < self.max_range
        visible = in_fov & in_range & ~self._los_blocked(drone_pos_w, rpos_w, dist)

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
                torch.where(visible, dist / self.max_range, torch.ones_like(dist)),
                self.last_bearing_sin,
                self.last_bearing_cos,
                (self.time_since_seen / self.memory_s).clamp(max=1.0),
            ],
            dim=1,
        )
        return vec, visible

    def visible_only(self, drone_pos_w, rpos_veh, rpos_w):
        """Visibility gate alone (no tracker update) -- used by the reward pass."""
        dist = rpos_w.norm(dim=1)
        horiz = rpos_veh[:, :2].norm(dim=1).clamp(min=1e-6)
        bearing = torch.atan2(rpos_veh[:, 1], rpos_veh[:, 0])
        elevation = torch.atan2(rpos_veh[:, 2], horiz)
        in_fov = (bearing.abs() < self.half_hfov) & (elevation.abs() < self.half_vfov)
        return in_fov & (dist < self.max_range) & ~self._los_blocked(drone_pos_w, rpos_w, dist)
