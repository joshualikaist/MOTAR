import math
import os

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
        # PBRS progress reward (A): previous-step distance to goal, potential Phi = -dist.
        self.prev_dist = torch.zeros(self.num_envs, device=self.device)
        # --- F segment-based capture (DISABLED): uncomment to track the previous position so the
        #     capture test can use the prev_pos->pos segment instead of the sampled endpoint.
        # self.prev_pos = torch.zeros((self.num_envs, 3), device=self.device)
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
        self.cur_goal_dist_max = float(self.cur.k_start)  # current goal-x ceiling (epoch-driven)
        self.cur_goal_dist_min = float(self.cur.k_min)    # current goal-x floor (epoch-driven)
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
        self._yaw_cmd = torch.zeros(self.num_envs, device=self.device)  # (b) last yaw action a[:,3], for the smoothness penalty
        self.infos = {}
        self.num_task_steps = 0

        # counters for periodic success/crash/timeout logging
        self._succ_agg = 0
        self._crash_agg = 0
        self._to_agg = 0
        self._reach_agg = 0
        self._fin_agg = 0
        self._mindist_sum = 0.0  # sum of closest approach over NON-CRASH finished episodes
        self._nc_agg = 0         # count of non-crash finished episodes
        self._closest_min = None  # best (min) closest approach in the window

    def close(self):
        self.sim_env.delete_env()

    # ------------------------------------------------------------------ checkpoint state
    def get_env_state(self):
        """Saved into the rl_games checkpoint ('env_state') so the epoch-proportional curriculum
        survives a --checkpoint resume. num_task_steps is otherwise in-memory and would restart at 0,
        resetting k_max/k_min to the easy start."""
        return {"num_task_steps": int(self.num_task_steps)}

    def set_env_state(self, state):
        if isinstance(state, dict) and state.get("num_task_steps") is not None:
            self.num_task_steps = int(state["num_task_steps"])

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
            goal = start_pos.clone()
            goal[:, 2] = self.task_config.flight_altitude
            m = float(self.cur.wall_margin)
            clearance = getattr(self.task_config, "goal_min_bar_clearance", 0.0)
            bars_xy = self.obs_dict["obstacle_position"][env_ids][:, :, 0:2]
            # "Cross the bar field": the drone spawns at x~0, so placing the goal at x=k on the far
            # side forces a left->right traversal of the bars. k ~ U[k_min, k_max(epoch)] (k_max
            # grows with training via _goal_x_max), y is free across the arena minus a wall margin.
            # Resample any goal within `clearance` of a bar so the 0.5 m capture sphere is flyable.
            k_max = self._goal_x_max()
            k_min = self._goal_x_min()
            self.cur_goal_dist_max = k_max  # surfaced to the dashboard as "curriculum max"
            self.cur_goal_dist_min = k_min  # surfaced to the dashboard as "curriculum min"
            todo = torch.ones(n, dtype=torch.bool, device=self.device)
            for _ in range(10):
                if not todo.any():
                    break
                j = int(todo.sum())
                gx = k_min + (k_max - k_min) * torch.rand(j, device=self.device)
                gx = gx.clamp(max=(b_max[todo, 0] - m))  # keep the capture sphere off the far wall
                gy = (b_min[todo, 1] + m) + (
                    b_max[todo, 1] - b_min[todo, 1] - 2.0 * m
                ) * torch.rand(j, device=self.device)
                goal[todo, 0] = gx
                goal[todo, 1] = gy
                if clearance <= 0.0:
                    break
                d_bar = (
                    torch.cdist(goal[todo, 0:2].unsqueeze(1), bars_xy[todo])
                    .squeeze(1)
                    .min(dim=1)
                    .values
                )
                still_bad = d_bar < clearance
                idx = todo.nonzero(as_tuple=False).squeeze(-1)
                todo = torch.zeros_like(todo)
                todo[idx[still_bad]] = True
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
        # PBRS progress reward (A): seed prev_dist with the start->goal distance for reset envs.
        self.prev_dist[env_ids] = torch.norm(self.target_position[env_ids] - start_pos, dim=1)
        # --- F segment-based capture (DISABLED): uncomment alongside the __init__/step prev_pos lines.
        # self.prev_pos[env_ids] = start_pos
        self.ep_min_goal_dist[env_ids] = float("inf")
        self.ep_reached[env_ids] = False

    def render(self):
        return self.sim_env.render()

    # ------------------------------------------------------------------ step
    def transform_action_to_command(self, actions):
        """NavRL 3D goal-frame velocity action -> vehicle-frame velocity command for the controller."""
        vel_goal = torch.clamp(actions[:, 0:3], -1.0, 1.0) * self.task_config.max_velocity  # (N, 3)
        vel_world = goal_frame_to_world(vel_goal, self.target_dir_2d)
        vel_vehicle = quat_rotate_inverse(self.obs_dict["robot_vehicle_orientation"], vel_world)
        self.command[:, 0:3] = vel_vehicle
        # 2D flight: hold altitude. The vehicle frame is yaw-only (level), so vehicle-z == world-z;
        # zeroing the vertical velocity command keeps the drone at its 1 m spawn altitude.
        self.command[:, 2] = 0.0
        # (b) learned yaw-rate: action[:, 3] in [-1, 1] -> euler yaw-rate (was held at 0). yaw_rate_max
        # matches the NavRL-scoped controller clamp (2.5 rad/s) so the mapping is linear (no dead band).
        self._yaw_cmd[:] = torch.clamp(actions[:, 3], -1.0, 1.0)
        self.command[:, 3] = self._yaw_cmd * self.task_config.yaw_rate_max
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

        if getattr(self.task_config, "terminate_on_capture", False):
            # capture ends the episode; timeouts are truncations that never captured
            successes = self.captured_now
            crashes = self.crashed_now
            timeouts = (self.truncations > 0) & ~successes & ~crashes
        else:
            successes = self.truncations * (dist_to_goal < self.task_config.success_radius)
            successes = torch.where(
                self.terminations > 0, torch.zeros_like(successes), successes
            )
            crashes = self.terminations > 0
            timeouts = torch.where(
                self.truncations > 0, torch.logical_not(successes), torch.zeros_like(successes)
            )
            timeouts = torch.where(self.terminations > 0, torch.zeros_like(timeouts), timeouts)
        self.infos = {"successes": successes, "timeouts": timeouts, "crashes": crashes}

        finished = (self.terminations > 0) | (self.truncations > 0)
        self._log_progress(successes, crashes, timeouts, finished)
        self._update_curriculum(finished)
        self._record_epoch_dashboard(successes, crashes, timeouts, finished)

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

        # (b) Learned-yaw shaping. Penalize crabbing so the 0.28 m box leads with its 0.28 m face,
        # not its 0.40 m diagonal, through the gaps. <= 0 and speed-gated -> no standing income (the
        # loiter optimum stays closed); yaw is decoupled from the goal-frame velocity command, so this
        # shapes ONLY the yaw DOF and cannot move the nav optimum. Added BEFORE the crash/capture
        # overwrites below, so a crashed env is still overwritten to the collision penalty.
        vel_veh = self.obs_dict["robot_vehicle_linvel"]
        speed_xy = vel_veh[:, :2].norm(dim=1)
        cos_crab = vel_veh[:, 0] / speed_xy.clamp(min=1e-6)
        misalign = 0.5 * (1.0 - cos_crab)  # 0 = nose-aligned .. 1 = flying backward
        align_gate = (speed_xy / self.task_config.yaw_align_speed_ref).clamp(0.0, 1.0)
        self.rewards[:] = self.rewards - self.rw["yaw_align_weight"] * misalign * align_gate
        self.rewards[:] = self.rewards - self.rw["yaw_rate_smooth_weight"] * self._yaw_cmd.pow(2)

        crashed = self.obs_dict["crashes"] > 0
        below = z < self.task_config.lower_height_bound
        above = z > self.task_config.upper_height_bound
        crashed_out = crashed | below | above

        # Interception: touching the capture radius ends the episode as a success (bonus
        # instead of the step reward). Capture wins over a same-step contact.
        if getattr(self.task_config, "terminate_on_capture", False):
            captured = dist < self.task_config.success_radius
            # --- F segment-based capture (DISABLED): uncomment to catch fast fly-throughs that
            #     tunnel over the 0.5 m sphere between 0.2 m/step samples. Tests the closest
            #     distance of the prev_pos->pos segment to the goal. Needs the prev_pos buffer
            #     (__init__ / reset_idx) and the `self.prev_pos[:] = pos` update below; replaces
            #     the point test on the line above.
            # ab = pos - self.prev_pos
            # t = (((self.target_position - self.prev_pos) * ab).sum(1)
            #      / ab.pow(2).sum(1).clamp(min=1e-9)).clamp(0.0, 1.0)
            # seg_dist = (self.target_position - (self.prev_pos + t.unsqueeze(1) * ab)).norm(dim=1)
            # captured = seg_dist < self.task_config.success_radius
        else:
            captured = torch.zeros_like(crashed_out)
        crashed_out = crashed_out & ~captured
        self.captured_now = captured
        self.crashed_now = crashed_out

        # A: PBRS progress reward -- dense "got closer" signal. F = gamma*Phi(s') - Phi(s) with
        # Phi = -progress_weight*dist  =>  reward = progress_weight*(prev_dist - gamma*dist). Zero
        # the next-state potential on TRUE terminals (capture/crash) per Grzes 2017; a timeout is a
        # truncation (not in `term`) so it keeps its gradient. This term is optimality-preserving,
        # so it cannot by itself move the "loiter" optimum -- it MUST ride alongside B1/B3 (which
        # do move it to "capture"). Disable by setting progress_weight = 0.0.
        if self.rw.get("progress_weight", 0.0) != 0.0:
            gamma = self.task_config.progress_gamma
            term = self.captured_now | self.crashed_now
            phi_next = torch.where(term, torch.zeros_like(dist), gamma * dist)
            self.rewards[:] = self.rewards + self.rw["progress_weight"] * (self.prev_dist - phi_next)
        self.prev_dist[:] = dist

        # --- C1 finish-funnel (DISABLED): uncomment to reward STILL MOVING INWARD in the
        #     0.5-1.0 m shell just outside capture, pulling the drone through the last band instead
        #     of braking ~1 m short. From shooting_moving_target_task (reached 97.5% success).
        #     Also uncomment funnel_coef / funnel_outer / funnel_width in navrl_task_config.
        # zone = ((self.rw["funnel_outer"] - dist) / self.rw["funnel_width"]).clamp(0.0, 1.0)
        # self.rewards[:] = self.rewards + self.rw["funnel_coef"] * zone * reward_vel.clamp(min=0.0)

        # --- F segment-based capture (DISABLED): uncomment to store this step's position for the
        #     segment capture test above.
        # self.prev_pos[:] = pos

        self.rewards[:] = torch.where(
            crashed_out, torch.full_like(self.rewards, self.rw["collision_penalty"]), self.rewards
        )
        self.rewards[:] = torch.where(
            captured, self.rewards + self.rw["capture_bonus"], self.rewards
        )
        self.terminations[:] = (crashed_out | captured).to(self.terminations.dtype)

    def add_static_safety_reward(self):
        # NavRL r_ss = mean over rays of log(distance to obstacle), clamped to (0, range].
        dist_m = self._lidar_distance_m().clamp(min=1e-6, max=self.task_config.lidar_max_range)
        # B1: re-baseline so OPEN SPACE (all rays at max range) scores 0 instead of +log(range).
        # This subtracts a constant log(range) per step, so the obstacle-avoidance GRADIENT is
        # byte-for-byte unchanged, but it deletes the standing "loiter income" (~+log(4)=+1.39/step
        # in the open) that made hovering short optimal (V_loiter >> V_capture -> ~7% capture).
        r_ss = (torch.log(dist_m) - math.log(self.task_config.lidar_max_range)).mean(dim=1)
        # do not reward-shape envs that just terminated (their reward is the collision penalty)
        alive = self.terminations <= 0
        self.rewards[alive] += self.rw["safety_static_weight"] * r_ss[alive]

        # B/C: near-obstacle clearance penalty (DEFAULT OFF via clearance_weight = 0.0). The log
        # safety term above is gentle; this adds a firmer buffer -- penalize being within
        # `clearance_margin` of the NEAREST obstacle (NavRL treats a 0.3 m proximity as a collision).
        # With task_config.clearance_speed_gated = True it becomes speed x proximity (option C):
        # only FAST approaches near obstacles are punished, so agility in the open is untouched.
        cw = self.rw.get("clearance_weight", 0.0)
        if cw != 0.0:
            min_dist = dist_m.min(dim=1).values
            pen = torch.relu(self.rw["clearance_margin"] - min_dist)  # > 0 only inside the margin
            if getattr(self.task_config, "clearance_speed_gated", False):
                pen = pen * self.obs_dict["robot_linvel"].norm(dim=1)  # C: scale by speed
            self.rewards[alive] -= cw * pen[alive]

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
        # (b) heading info in the vehicle (yaw-only) frame so the agent can OBSERVE its crab angle
        # (vel_body_xy: lateral slip = which way to yaw) and where the goal is relative to its NOSE
        # (goal_bearing_body, always defined even at rest), re-anchoring the body-frame LiDAR now that
        # yaw is free. robot_vehicle_orientation is yaw-only; robot_vehicle_linvel is precomputed.
        q_veh = self.obs_dict["robot_vehicle_orientation"]
        goal_veh = quat_rotate_inverse(q_veh, rpos)
        goal_bearing_body = goal_veh[:, :2] / goal_veh[:, :2].norm(dim=1, keepdim=True).clamp(min=1e-6)
        vel_body_xy = self.obs_dict["robot_vehicle_linvel"][:, :2] / self.task_config.max_velocity

        s_int = torch.cat(
            [rpos_unit_g, dist_2d, dist_z, vel_g, goal_bearing_body, vel_body_xy], dim=1
        )  # (N, 12)

        # LiDAR scan, normalized [0,1] (1 = no obstacle within range), flattened to 144
        lidar = self._lidar_distance_m() / self.task_config.lidar_max_range

        self.task_obs["observations"][:, : self.task_config.internal_state_dim] = s_int
        self.task_obs["observations"][:, self.task_config.internal_state_dim :] = lidar

    def _goal_x_max(self):
        """Epoch-proportional goal-x ceiling: ramps k_start -> k_final over k_warmup_epochs, then
        plateaus. Uses num_task_steps as an epoch proxy (rl_games collects ppo_horizon env-steps per
        epoch, so epoch ~= num_task_steps / ppo_horizon). Resets with the process (in-memory), like
        the old curriculum -- on --checkpoint resume k_max restarts at k_start and re-ramps."""
        warmup_steps = max(1, int(self.cur.k_warmup_epochs) * int(self.cur.ppo_horizon))
        frac = min(1.0, self.num_task_steps / warmup_steps)
        return self.cur.k_start + (self.cur.k_final - self.cur.k_start) * frac

    def _goal_x_min(self):
        """Goal-x floor: stays at k_min early, then ramps k_min -> k_min_final over
        [k_min_ramp_start_epochs, +k_min_ramp_epochs] so late episodes drop the easy near goals and
        focus on deep crossings. The start is independent of the k_max ramp (they may overlap). Kept
        at least 1 m below k_max so the [min, max] window stays valid."""
        k_min_final = getattr(self.cur, "k_min_final", self.cur.k_min)
        h = int(self.cur.ppo_horizon)
        hold = int(getattr(self.cur, "full_scale_hold_epochs", 0))
        # k_min starts rising at k_min_ramp_start_epochs (explicit). Legacy fallback: after the k_max
        # ramp + full-scale hold, i.e. k_warmup_epochs + hold.
        start_epochs = int(getattr(self.cur, "k_min_ramp_start_epochs",
                                   int(self.cur.k_warmup_epochs) + hold))
        start_steps = start_epochs * h
        ramp_steps = max(1, int(getattr(self.cur, "k_min_ramp_epochs", self.cur.k_warmup_epochs)) * h)
        frac = min(1.0, max(0.0, (self.num_task_steps - start_steps) / ramp_steps))
        k_min = self.cur.k_min + (k_min_final - self.cur.k_min) * frac
        return min(k_min, self._goal_x_max() - 1.0)

    def _update_curriculum(self, finished):
        # The curriculum is now epoch-proportional (goal-x ceiling via _goal_x_max, applied in
        # reset_idx), so there is nothing to update per finished episode. Kept for the step() hook.
        return

    def _record_epoch_dashboard(self, successes, crashes, timeouts, finished):
        """Feed finished-episode outcomes to the per-epoch train dashboard (console + TB)."""
        n_fin = int(torch.sum(finished).item())
        if n_fin == 0:
            return
        # Closest approach EXCLUDING crashes: a crash dies far from the goal and only inflates the
        # mean, so aggregate over non-crash finished episodes and also surface the best (min).
        nocrash = finished & ~(crashes > 0)
        n_nc = int(torch.sum(nocrash).item())
        closest_nc_sum = float(torch.sum(self.ep_min_goal_dist[nocrash]).item()) if n_nc else 0.0
        closest_min = float(torch.min(self.ep_min_goal_dist[nocrash]).item()) if n_nc else None
        record_navrl_epoch_episodes(
            num_finished=n_fin,
            num_reached=int(torch.sum(self.ep_reached & finished).item()),
            num_captured=int(torch.sum(successes).item()),
            num_crash=int(torch.sum(crashes > 0).item()),
            num_timeout=int(torch.sum(timeouts).item()),
            closest_nocrash_sum=closest_nc_sum,
            closest_nocrash_count=n_nc,
            closest_min=closest_min,
            goal_dist_max=self.cur_goal_dist_max if self.cur.use_curriculum else None,
            goal_dist_min=self.cur_goal_dist_min if self.cur.use_curriculum else None,
        )

    def _log_progress(self, successes, crashes, timeouts, finished=None):
        self._succ_agg += int(torch.sum(successes).item())
        self._crash_agg += int(torch.sum(crashes > 0).item())
        self._to_agg += int(torch.sum(timeouts).item())
        if finished is not None and finished.any():
            self._reach_agg += int(torch.sum(self.ep_reached & finished).item())
            nocrash = finished & ~(crashes > 0)
            if nocrash.any():
                self._mindist_sum += float(torch.sum(self.ep_min_goal_dist[nocrash]).item())
                self._nc_agg += int(torch.sum(nocrash).item())
                m = float(torch.min(self.ep_min_goal_dist[nocrash]).item())
                self._closest_min = m if self._closest_min is None else min(self._closest_min, m)
            self._fin_agg += int(torch.sum(finished).item())
        total = self._succ_agg + self._crash_agg + self._to_agg
        if total >= 2048:
            reach_rate = self._reach_agg / max(1, self._fin_agg)
            mean_nc = self._mindist_sum / max(1, self._nc_agg)
            best = self._closest_min if self._closest_min is not None else float("nan")
            _run = os.environ.get("AERIAL_RUN_NAME", "").strip()
            logger.warning(
                "NavRL progress%s | captured=%.3f ever_reached=%.3f crash=%.3f timeout=%.3f "
                "closest_nocrash=%.2fm best=%.2fm (n=%d)"
                % (
                    (" [" + _run + "]") if _run else "",
                    self._succ_agg / total,
                    reach_rate,
                    self._crash_agg / total,
                    self._to_agg / total,
                    mean_nc,
                    best,
                    total,
                )
            )
            self._succ_agg = self._crash_agg = self._to_agg = 0
            self._reach_agg = self._fin_agg = 0
            self._mindist_sum = 0.0
            self._nc_agg = 0
            self._closest_min = None
