import numpy as np
import torch
from gym.spaces import Box, Dict

from aerial_gym.task.base_task import BaseTask
from aerial_gym.task.navrl_task.train_dashboard import record_navrl_epoch_episodes
from aerial_gym.sim.sim_builder import SimBuilder
from aerial_gym.utils.math import quat_rotate, quat_rotate_inverse, torch_rand_float_tensor
from aerial_gym.utils.logging import CustomLogger

logger = CustomLogger("navrl_task")


def vec_to_goal_frame(vec, goal_direction):
    """Express world-frame vector(s) in the goal coordinate frame.

    The goal frame has its x-axis along the (horizontal) start->goal direction, y-axis in the
    horizontal plane, z-axis up. Ported from NavRL's utils.vec_to_new_frame.

    vec:            (N, 3) or (N, M, 3)
    goal_direction: (N, 3)  -- world-frame direction toward the goal (z-component may be 0)
    returns:        same leading shape as vec, last dim 3.
    """
    single = vec.dim() == 2
    if single:
        vec = vec.unsqueeze(1)  # (N, 1, 3)
    n = vec.shape[0]

    gx = goal_direction / goal_direction.norm(dim=-1, keepdim=True).clamp(min=1e-6)
    z = torch.tensor([0.0, 0.0, 1.0], device=vec.device).expand_as(gx)
    gy = torch.cross(z, gx, dim=-1)
    gy = gy / gy.norm(dim=-1, keepdim=True).clamp(min=1e-6)
    gz = torch.cross(gx, gy, dim=-1)
    gz = gz / gz.norm(dim=-1, keepdim=True).clamp(min=1e-6)

    vx = torch.bmm(vec, gx.view(n, 3, 1))
    vy = torch.bmm(vec, gy.view(n, 3, 1))
    vz = torch.bmm(vec, gz.view(n, 3, 1))
    out = torch.cat((vx, vy, vz), dim=-1)  # (N, M, 3)
    return out.squeeze(1) if single else out


def goal_frame_to_world(vec, goal_direction):
    """Inverse of vec_to_goal_frame: express goal-frame vector(s) back in the world frame.

    Ported from NavRL's utils.vec_to_world.
    """
    world_x = torch.tensor([1.0, 0.0, 0.0], device=vec.device).expand_as(goal_direction)
    world_basis_in_goal = vec_to_goal_frame(world_x, goal_direction)
    return vec_to_goal_frame(vec, world_basis_in_goal)


class NavRLTask(BaseTask):
    def __init__(
        self, task_config, seed=None, num_envs=None, headless=None, device=None, use_warp=None
    ):
        if seed is not None:
            task_config.seed = seed
        if num_envs is not None:
            task_config.num_envs = num_envs
        if headless is not None:
            task_config.headless = headless
        if device is not None:
            task_config.device = device
        if use_warp is not None:
            task_config.use_warp = use_warp
        super().__init__(task_config)
        self.device = self.task_config.device

        logger.info(
            "Building NavRL task | sim=%s env=%s robot=%s controller=%s"
            % (
                self.task_config.sim_name,
                self.task_config.env_name,
                self.task_config.robot_name,
                self.task_config.controller_name,
            )
        )
        self.sim_env = SimBuilder().build_env(
            sim_name=self.task_config.sim_name,
            env_name=self.task_config.env_name,
            robot_name=self.task_config.robot_name,
            controller_name=self.task_config.controller_name,
            args=self.task_config.args,
            device=self.device,
            num_envs=self.task_config.num_envs,
            use_warp=self.task_config.use_warp,
            headless=self.task_config.headless,
        )
        self.num_envs = self.sim_env.num_envs

        # --- task buffers
        self.target_position = torch.zeros((self.num_envs, 3), device=self.device)
        self.target_dir_2d = torch.zeros((self.num_envs, 3), device=self.device)
        self.target_dir_2d[:, 0] = 1.0  # placeholder unit direction before first reset
        self.height_range = torch.zeros((self.num_envs, 2), device=self.device)  # [min, max]
        self.prev_vel_w = torch.zeros((self.num_envs, 3), device=self.device)
        # per-episode diagnostics: closest approach and whether the goal was ever reached
        self.ep_min_goal_dist = torch.full((self.num_envs,), float("inf"), device=self.device)
        self.ep_reached = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        self.target_min_ratio = torch.tensor(
            self.task_config.target_min_ratio, device=self.device
        ).expand(self.num_envs, -1)
        self.target_max_ratio = torch.tensor(
            self.task_config.target_max_ratio, device=self.device
        ).expand(self.num_envs, -1)

        # goal-distance curriculum state
        self.cur = self.task_config.curriculum
        self.cur_goal_dist_max = float(self.cur.goal_dist_start)
        self._cur_reach_agg = 0
        self._cur_fin_agg = 0

        rp = self.task_config.reward_parameters
        self.rw = {k: float(v) for k, v in rp.items()}

        # --- shared views into the environment tensors
        self.obs_dict = self.sim_env.get_obs()
        self.terminations = self.obs_dict["crashes"]
        self.truncations = self.obs_dict["truncations"]
        self.rewards = torch.zeros(self.num_envs, device=self.device)

        # --- spaces
        self.observation_space = Dict(
            {
                "observations": Box(
                    low=-np.inf,
                    high=np.inf,
                    shape=(self.task_config.observation_space_dim,),
                    dtype=np.float32,
                )
            }
        )
        self.action_space = Box(
            low=-1.0, high=1.0, shape=(self.task_config.action_space_dim,), dtype=np.float32
        )
        self.task_obs = {
            "observations": torch.zeros(
                (self.num_envs, self.task_config.observation_space_dim), device=self.device
            )
        }
        self.command = torch.zeros((self.num_envs, 4), device=self.device)  # controller input
        self.infos = {}
        self.num_task_steps = 0

        # counters for periodic success/crash/timeout logging
        self._succ_agg = 0
        self._crash_agg = 0
        self._to_agg = 0
        self._reach_agg = 0
        self._fin_agg = 0
        self._mindist_sum = 0.0

    def close(self):
        self.sim_env.delete_env()

    # ------------------------------------------------------------------ reset
    def reset(self):
        # Respawn the robots (and re-place obstacles) BEFORE sampling goals. Without this, a
        # full reset leaves the robots at their build pose (overlapping the bars near the env
        # origin), so the first step crashes every env at once — which ends rl_games play mode
        # after a single step. Mid-episode resets don't need it: the env manager has already
        # respawned those envs by the time reset_idx() is called from step().
        self.sim_env.reset()
        self.reset_idx(torch.arange(self.num_envs, device=self.device))
        # render once so the first observation carries a valid LiDAR scan
        self.sim_env.render(render_components="sensors")
        return self.get_return_tuple()

    def reset_idx(self, env_ids):
        if len(env_ids) == 0:
            return
        # robot has already been respawned by the env manager when this is called mid-episode,
        # so robot_position holds the fresh start pose.
        start_pos = self.obs_dict["robot_position"][env_ids]
        b_min = self.obs_dict["env_bounds_min"][env_ids]
        b_max = self.obs_dict["env_bounds_max"][env_ids]

        if self.cur.use_curriculum:
            n = len(env_ids)
            # random horizontal direction, distance in [min, current max]
            theta = 2.0 * torch.pi * torch.rand(n, device=self.device)
            dist = self.cur.goal_dist_min + (
                self.cur_goal_dist_max - self.cur.goal_dist_min
            ) * torch.rand(n, device=self.device)
            goal = start_pos.clone()
            goal[:, 0] = start_pos[:, 0] + dist * torch.cos(theta)
            goal[:, 1] = start_pos[:, 1] + dist * torch.sin(theta)
            # 2D navigation: keep the goal inside the arena horizontally, pin it to the fixed
            # flight altitude (the drone flies XY-only at flight_altitude).
            margin = 1.0
            goal[:, 0:2] = torch.maximum(goal[:, 0:2], b_min[:, 0:2] + margin)
            goal[:, 0:2] = torch.minimum(goal[:, 0:2], b_max[:, 0:2] - margin)
            goal[:, 2] = self.task_config.flight_altitude
            self.target_position[env_ids] = goal
        else:
            ratio = torch_rand_float_tensor(self.target_min_ratio, self.target_max_ratio)[env_ids]
            self.target_position[env_ids] = b_min + (b_max - b_min) * ratio

        d = self.target_position[env_ids] - start_pos
        d[:, 2] = 0.0  # horizontal goal direction defines the goal frame
        self.target_dir_2d[env_ids] = d

        self.height_range[env_ids, 0] = torch.minimum(
            start_pos[:, 2], self.target_position[env_ids, 2]
        )
        self.height_range[env_ids, 1] = torch.maximum(
            start_pos[:, 2], self.target_position[env_ids, 2]
        )
        self.prev_vel_w[env_ids] = 0.0
        self.ep_min_goal_dist[env_ids] = float("inf")
        self.ep_reached[env_ids] = False

    def render(self):
        return self.sim_env.render()

    # ------------------------------------------------------------------ step
    def transform_action_to_command(self, actions):
        """NavRL 3D goal-frame velocity action -> vehicle-frame velocity command for the controller."""
        vel_goal = torch.clamp(actions, -1.0, 1.0) * self.task_config.max_velocity  # (N, 3)
        vel_world = goal_frame_to_world(vel_goal, self.target_dir_2d)
        vel_vehicle = quat_rotate_inverse(self.obs_dict["robot_vehicle_orientation"], vel_world)
        self.command[:, 0:3] = vel_vehicle
        # 2D flight: hold altitude. The vehicle frame is yaw-only (level), so vehicle-z == world-z;
        # zeroing the vertical velocity command keeps the drone at its 1 m spawn altitude.
        self.command[:, 2] = 0.0
        self.command[:, 3] = 0.0  # no yaw-rate command; heading is held from reset
        return self.command

    def step(self, actions):
        command = self.transform_action_to_command(actions)
        self.sim_env.step(actions=command)

        # state-based reward + termination (LiDAR-based safety reward is added after rendering)
        self.compute_state_reward_and_terminations()

        self.truncations[:] = torch.where(
            self.sim_env.sim_steps > self.task_config.episode_len_steps,
            torch.ones_like(self.truncations),
            torch.zeros_like(self.truncations),
        )

        dist_to_goal = torch.norm(
            self.target_position - self.obs_dict["robot_position"], dim=1
        )
        # per-episode closest approach / ever-reached (updated before any env is reset)
        self.ep_min_goal_dist = torch.minimum(self.ep_min_goal_dist, dist_to_goal)
        self.ep_reached |= dist_to_goal < self.task_config.success_radius

        successes = self.truncations * (dist_to_goal < self.task_config.success_radius)
        successes = torch.where(self.terminations > 0, torch.zeros_like(successes), successes)
        timeouts = torch.where(
            self.truncations > 0, torch.logical_not(successes), torch.zeros_like(successes)
        )
        timeouts = torch.where(self.terminations > 0, torch.zeros_like(timeouts), timeouts)
        self.infos = {"successes": successes, "timeouts": timeouts, "crashes": self.terminations}

        finished = (self.terminations > 0) | (self.truncations > 0)
        self._log_progress(successes, self.terminations, timeouts, finished)
        self._update_curriculum(finished)
        self._record_epoch_dashboard(successes, timeouts, finished)

        if self.task_config.return_state_before_reset:
            return_tuple = self.get_return_tuple()

        # render (raycast LiDAR from the new state) and reset finished envs
        reset_envs = self.sim_env.post_reward_calculation_step()
        if len(reset_envs) > 0:
            self.reset_idx(reset_envs)

        # LiDAR-based static-safety reward, using the freshly rendered scan
        self.add_static_safety_reward()

        self.num_task_steps += 1
        if not self.task_config.return_state_before_reset:
            return_tuple = self.get_return_tuple()
        return return_tuple

    def _lidar_distance_m(self):
        """Per-ray distance in meters, shape (N, vbeams*hbeams). Normalized pixels * max_range."""
        pix = self.obs_dict["depth_range_pixels"].squeeze(1)  # (N, vbeams, hbeams), in [0, 1]
        pix = torch.nan_to_num(pix, nan=1.0, posinf=1.0, neginf=1.0).clamp(0.0, 1.0)
        return (pix * self.task_config.lidar_max_range).reshape(self.num_envs, -1)

    def compute_state_reward_and_terminations(self):
        pos = self.obs_dict["robot_position"]
        vel_w = self.obs_dict["robot_linvel"]

        rpos = self.target_position - pos
        dist = rpos.norm(dim=1).clamp(min=1e-6)
        vel_dir = rpos / dist.unsqueeze(1)
        reward_vel = (vel_w * vel_dir).sum(dim=1)

        penalty_smooth = (vel_w - self.prev_vel_w).norm(dim=1)
        self.prev_vel_w[:] = vel_w

        z = pos[:, 2]
        m = self.rw["height_margin"]
        hi = self.height_range[:, 1] + m
        lo = self.height_range[:, 0] - m
        penalty_height = torch.zeros_like(z)
        penalty_height = torch.where(z > hi, (z - hi) ** 2, penalty_height)
        penalty_height = torch.where(z < lo, (lo - z) ** 2, penalty_height)

        self.rewards[:] = (
            self.rw["vel_weight"] * reward_vel
            + self.rw["alive_weight"]
            - self.rw["smooth_weight"] * penalty_smooth
            - self.rw["height_weight"] * penalty_height
        )

        crashed = self.obs_dict["crashes"] > 0
        below = z < self.task_config.lower_height_bound
        above = z > self.task_config.upper_height_bound
        terminated = crashed | below | above
        self.rewards[:] = torch.where(
            terminated, torch.full_like(self.rewards, self.rw["collision_penalty"]), self.rewards
        )
        self.terminations[:] = terminated.to(self.terminations.dtype)

    def add_static_safety_reward(self):
        # NavRL r_ss = mean over rays of log(distance to obstacle), clamped to (0, range].
        dist_m = self._lidar_distance_m().clamp(min=1e-6, max=self.task_config.lidar_max_range)
        r_ss = torch.log(dist_m).mean(dim=1)
        # do not reward-shape envs that just terminated (their reward is the collision penalty)
        alive = self.terminations <= 0
        self.rewards[alive] += self.rw["safety_static_weight"] * r_ss[alive]

    # ------------------------------------------------------------------ obs
    def get_return_tuple(self):
        self.process_obs_for_task()
        return (self.task_obs, self.rewards, self.terminations, self.truncations, self.infos)

    def process_obs_for_task(self):
        pos = self.obs_dict["robot_position"]
        vel_w = self.obs_dict["robot_linvel"]
        rpos = self.target_position - pos
        dist = rpos.norm(dim=1, keepdim=True).clamp(min=1e-6)
        dist_2d = rpos[:, :2].norm(dim=1, keepdim=True)
        dist_z = rpos[:, 2:3]

        rpos_unit_g = vec_to_goal_frame(rpos / dist, self.target_dir_2d)
        vel_g = vec_to_goal_frame(vel_w, self.target_dir_2d)

        s_int = torch.cat([rpos_unit_g, dist_2d, dist_z, vel_g], dim=1)  # (N, 8)

        # LiDAR scan, normalized [0,1] (1 = no obstacle within range), flattened to 144
        lidar = self._lidar_distance_m() / self.task_config.lidar_max_range

        self.task_obs["observations"][:, : self.task_config.internal_state_dim] = s_int
        self.task_obs["observations"][:, self.task_config.internal_state_dim :] = lidar

    def _update_curriculum(self, finished):
        if not self.cur.use_curriculum or not finished.any():
            return
        self._cur_reach_agg += int(torch.sum(self.ep_reached & finished).item())
        self._cur_fin_agg += int(torch.sum(finished).item())
        if self._cur_fin_agg >= self.cur.check_after_episodes:
            reach_rate = self._cur_reach_agg / max(1, self._cur_fin_agg)
            if (
                reach_rate >= self.cur.reach_rate_to_expand
                and self.cur_goal_dist_max < self.cur.goal_dist_max
            ):
                self.cur_goal_dist_max = min(
                    self.cur_goal_dist_max + self.cur.expand_step, self.cur.goal_dist_max
                )
                logger.warning(
                    "curriculum ↑ | reach_rate=%.2f -> goal_dist_max=%.1f m"
                    % (reach_rate, self.cur_goal_dist_max)
                )
            self._cur_reach_agg = 0
            self._cur_fin_agg = 0

    def _record_epoch_dashboard(self, successes, timeouts, finished):
        """Feed finished-episode outcomes to the per-epoch train dashboard (console + TB)."""
        n_fin = int(torch.sum(finished).item())
        if n_fin == 0:
            return
        record_navrl_epoch_episodes(
            num_finished=n_fin,
            num_reached=int(torch.sum(self.ep_reached & finished).item()),
            num_success_timeout=int(torch.sum(successes).item()),
            num_crash=int(torch.sum(self.terminations > 0).item()),
            num_timeout=int(torch.sum(timeouts).item()),
            closest_sum=float(torch.sum(self.ep_min_goal_dist[finished]).item()),
            closest_count=n_fin,
            goal_dist_max=self.cur_goal_dist_max if self.cur.use_curriculum else None,
        )

    def _log_progress(self, successes, crashes, timeouts, finished=None):
        self._succ_agg += int(torch.sum(successes).item())
        self._crash_agg += int(torch.sum(crashes > 0).item())
        self._to_agg += int(torch.sum(timeouts).item())
        if finished is not None and finished.any():
            self._reach_agg += int(torch.sum(self.ep_reached & finished).item())
            self._mindist_sum += float(torch.sum(self.ep_min_goal_dist[finished]).item())
            self._fin_agg += int(torch.sum(finished).item())
        total = self._succ_agg + self._crash_agg + self._to_agg
        if total >= 2048:
            reach_rate = self._reach_agg / max(1, self._fin_agg)
            mean_min_dist = self._mindist_sum / max(1, self._fin_agg)
            logger.warning(
                "NavRL progress | success@timeout=%.3f ever_reached=%.3f crash=%.3f timeout=%.3f "
                "mean_closest_approach=%.2fm (n=%d)"
                % (
                    self._succ_agg / total,
                    reach_rate,
                    self._crash_agg / total,
                    self._to_agg / total,
                    mean_min_dist,
                    total,
                )
            )
            self._succ_agg = self._crash_agg = self._to_agg = 0
            self._reach_agg = self._fin_agg = 0
            self._mindist_sum = 0.0
