import math
import os

import numpy as np
import torch
from gym.spaces import Box, Dict

from aerial_gym.task.base_task import BaseTask
from aerial_gym.task.navrl_task.train_dashboard import record_navrl_epoch_episodes
from aerial_gym.sim.sim_builder import SimBuilder
from aerial_gym.utils.math import quat_rotate_inverse
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
        # Previous drone position: anchors the PBRS progress term to the target's CURRENT position
        # (credits only the drone's own motion when the target moves) and, together with prev_rel,
        # provides the swept-segment capture test.
        self.prev_pos = torch.zeros((self.num_envs, 3), device=self.device)
        # Previous drone-position-minus-target-position (relative frame). The capture test sweeps
        # the segment prev_rel -> (pos - target) against the capture sphere at the origin, so a
        # fast fly-through cannot tunnel between 0.1 s samples even when BOTH agents move.
        self.prev_rel = torch.zeros((self.num_envs, 3), device=self.device)
        # per-episode diagnostics: closest approach and whether the goal was ever reached
        self.ep_min_goal_dist = torch.full((self.num_envs,), float("inf"), device=self.device)
        self.ep_reached = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        # goal-distance curriculum state
        self.cur = self.task_config.curriculum
        self.cur_goal_dist_max = float(self.cur.k_start)  # current goal-x ceiling (epoch-driven)
        self.cur_goal_dist_min = float(self.cur.k_min)    # current goal-x floor (epoch-driven)

        # --- Phase 3 moving target: a VIRTUAL point (task-side coordinates only — no actor, no
        # mesh, invisible to the LiDAR). All-zero speeds (the default) keep the task byte-
        # compatible with the static Phases 1-2.
        self.tm = self.task_config.target_motion
        self.target_vel_w = torch.zeros((self.num_envs, 3), device=self.device)  # realized vel
        self._tm_speed = torch.zeros(self.num_envs, device=self.device)          # per-episode speed
        self._tm_pattern = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)  # 0=cv 1=wp 2=circle
        self._tm_cv_vel = torch.zeros((self.num_envs, 2), device=self.device)    # cv desired velocity
        self._tm_waypoint = torch.zeros((self.num_envs, 2), device=self.device)  # waypoint target
        self._tm_circle_center = torch.zeros((self.num_envs, 2), device=self.device)
        self._tm_circle_angvel = torch.zeros(self.num_envs, device=self.device)  # signed rad/s

        rp = self.task_config.reward_parameters
        self.rw = {k: float(v) for k, v in rp.items()}

        # --- shared views into the environment tensors
        self.obs_dict = self.sim_env.get_obs()
        self.density = getattr(self.task_config, "density", None)
        self.max_bars_available = self._get_max_bars_available()
        initial_bars_requested = self._initial_active_bars()
        self.n_bars_active = 0
        self._density_succ_agg = 0
        self._density_fin_agg = 0
        self._set_active_bars(initial_bars_requested)
        logger.warning(
            "NavRL density | active_bars=%d max_bars=%d curriculum=%s"
            % (
                self.n_bars_active,
                self.max_bars_available,
                bool(getattr(self.density, "use_density_curriculum", False)),
            )
        )
        # One RL step = num_physics_steps x physics dt (0.1 s here). obs_dict["dt"] alone is the
        # PHYSICS dt (0.01 s) — integrating the target with it would move the target at 1/10 of its
        # nominal speed (the shooting_moving_target task gets away with obs_dict["dt"] only because
        # its env runs 1 physics step per RL step).
        try:
            n_phys = int(self.sim_env.cfg.env.num_physics_steps_per_env_step_mean)
        except AttributeError:
            n_phys = 10
            logger.warning("navrl_task: env config not reachable for physics-steps; assuming 10.")
        self.step_dt = float(self.obs_dict["dt"]) * n_phys
        if float(self.tm.speed_final) > 0.0 or float(self.tm.speed_fixed) >= 0.0:
            logger.warning(
                "NavRL moving target | pattern=%s speed_final=%.2f speed_fixed=%.2f rl_dt=%.3fs"
                % (self.tm.pattern, self.tm.speed_final, self.tm.speed_fixed, self.step_dt)
            )

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
        self._yaw_cmd = torch.zeros(self.num_envs, device=self.device)  # (b) current-step yaw action a[:,3], penalized quadratically (magnitude damping)
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

    def _get_max_bars_available(self):
        for key in ("obstacle_position", "env_asset_state_tensor"):
            tensor = self.obs_dict.get(key, None)
            if tensor is not None and len(tensor.shape) >= 2:
                return int(tensor.shape[1])
        logger.warning(
            "NavRL density | obstacle-state tensor not found in obs_dict "
            "(tried obstacle_position, env_asset_state_tensor) -> max_bars=0. Density control is "
            "INERT (0 bars active); check the env build / obs key names."
        )
        return 0

    def _initial_active_bars(self):
        if self.density is None:
            requested = self.max_bars_available
        elif getattr(self.density, "use_density_curriculum", False):
            requested = getattr(self.density, "n_start", self.max_bars_available)
        else:
            requested = getattr(self.density, "num_bars_active", self.max_bars_available)
        return requested

    def _clamp_active_bars(self, n_bars):
        try:
            requested = int(n_bars)
        except (TypeError, ValueError):
            requested = 0
        return min(max(0, requested), self.max_bars_available)

    def _set_active_bars(self, n_bars, log=True):
        try:
            requested = int(n_bars)
        except (TypeError, ValueError):
            requested = 0
        clamped = self._clamp_active_bars(requested)
        if log and clamped != requested:
            logger.warning(
                "Requested %d active bars but only %d were built; using %d."
                % (requested, self.max_bars_available, clamped)
            )
        self.n_bars_active = clamped
        self.obs_dict["num_obstacles_in_env"] = clamped
        return clamped

    def close(self):
        self.sim_env.delete_env()

    # ------------------------------------------------------------------ checkpoint state
    def get_env_state(self):
        """Saved into the rl_games checkpoint ('env_state') so the epoch-proportional goal
        curriculum and optional density curriculum survive a --checkpoint resume."""
        return {
            "num_task_steps": int(self.num_task_steps),
            "n_bars_active": int(self.n_bars_active),
        }

    def set_env_state(self, state):
        if isinstance(state, dict) and state.get("num_task_steps") is not None:
            self.num_task_steps = int(state["num_task_steps"])
        if isinstance(state, dict) and state.get("n_bars_active") is not None:
            # An explicit NAVRL_NUM_BARS wins over the checkpoint: density-sweep evals must run at
            # the REQUESTED density, not silently at whatever density the checkpoint trained on.
            if os.environ.get("NAVRL_NUM_BARS", "").strip():
                logger.warning(
                    "NAVRL_NUM_BARS set explicitly; ignoring checkpoint n_bars_active=%s (keeping %d)."
                    % (state.get("n_bars_active"), self.n_bars_active)
                )
            else:
                self._set_active_bars(state["n_bars_active"])

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

        n = len(env_ids)
        goal = start_pos.clone()
        goal[:, 2] = self.task_config.flight_altitude
        m = float(self.cur.wall_margin)
        clearance = float(getattr(self.task_config, "goal_min_bar_clearance", 0.0))
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
        if clearance > 0.0 and bool(todo.any()):
            # At high density a few goals can survive 10 rejection rounds still inside the bar
            # clearance. Snap them radially away from the nearest bar (best effort) instead of
            # silently keeping a goal whose capture sphere is not flyable.
            gxy = goal[todo, 0:2]
            d_bar, j_bar = torch.cdist(gxy.unsqueeze(1), bars_xy[todo]).squeeze(1).min(dim=1)
            near = bars_xy[todo][torch.arange(len(j_bar), device=self.device), j_bar]
            away = gxy - near
            away = away / away.norm(dim=1, keepdim=True).clamp(min=1e-6)
            goal[todo, 0:2] = near + away * clearance
            logger.warning(
                "navrl reset: %d goals snapped out of bar clearance after 10 rejection rounds."
                % int(todo.sum())
            )
        self.target_position[env_ids] = goal

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
        # Seed the PBRS/segment-capture buffers with the spawn state. First-step progress is then
        # ||start - target|| - gamma*||pos - target||, identical to the old prev_dist seeding.
        self.prev_pos[env_ids] = start_pos
        self.prev_rel[env_ids] = start_pos - self.target_position[env_ids]
        # Phase 3: per-episode target speed + trajectory pattern (all-static when the speed
        # ceiling is 0 -> Phases 1-2 behavior).
        self._sample_target_motion(env_ids)
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
        # Phase 3: move the virtual target FIRST — both agents move during this 0.1 s control
        # interval, and the end-of-interval reward is computed against the target's NEW position.
        # (No-op while all per-episode target speeds are 0, i.e. the static Phases 1-2 task.)
        self._advance_target()
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

        # Interception semantics (always on): capture ends the episode; timeouts are truncations
        # that never captured.
        successes = self.captured_now
        crashes = self.crashed_now
        timeouts = (self.truncations > 0) & ~successes & ~crashes
        self.infos = {"successes": successes, "timeouts": timeouts, "crashes": crashes}

        finished = (self.terminations > 0) | (self.truncations > 0)
        self._log_progress(successes, crashes, timeouts, finished)
        self._update_curriculum(successes, finished)
        self._record_epoch_dashboard(successes, crashes, timeouts, finished)

        # render (raycast LiDAR from the new state) and reset finished envs
        reset_envs = self.sim_env.post_reward_calculation_step()
        if len(reset_envs) > 0:
            self.reset_idx(reset_envs)

        # LiDAR-based static-safety reward, using the freshly rendered scan
        self.add_static_safety_reward()

        self.num_task_steps += 1
        return self.get_return_tuple()

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
        # Range-rate (closing speed): the component of the RELATIVE velocity toward the target.
        # With a static target (target_vel_w == 0, the Phases 1-2 default) the subtraction is
        # IEEE-exact zero, so this reduces to NavRL's velocity-toward-goal term bit-for-bit.
        reward_vel = ((vel_w - self.target_vel_w) * vel_dir).sum(dim=1)

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

        # Interception (always on): touching the capture radius ends the episode as a success
        # (terminal bonus instead of continued step reward). Capture wins over a same-step contact.
        # Swept-SEGMENT test in the target-relative frame: sweep prev_rel -> rel against the capture
        # sphere at the origin. This linearizes BOTH agents' motion over the 0.1 s step, so a fast
        # fly-through (closing speed up to 4 m/s = 0.4 m/step) cannot tunnel between samples. With a
        # static target and 0.2 m steps it is a strict superset of the old point test (adds only
        # rare grazing captures; unit-tested in tools/test_navrl_p3_math.py).
        rel = pos - self.target_position
        seg = rel - self.prev_rel
        t_close = (-(self.prev_rel * seg).sum(dim=1) / (seg * seg).sum(dim=1).clamp(min=1e-9)).clamp(0.0, 1.0)
        seg_dist = (self.prev_rel + t_close.unsqueeze(1) * seg).norm(dim=1)
        captured = seg_dist < self.task_config.success_radius
        crashed_out = crashed_out & ~captured
        self.captured_now = captured
        self.crashed_now = crashed_out

        # A: PBRS progress reward -- dense "got closer" signal. F = gamma*Phi(s') - Phi(s) with
        # Phi = -progress_weight*dist. RE-ANCHORED to the target's CURRENT position:
        #   progress = ||prev_pos - target_new|| - gamma*||pos_new - target_new||
        # so only the drone's OWN motion is credited — the naive prev_dist form would reward/punish
        # the drone for the target's uncontrollable move (biased shaping under pursuit). With a
        # static target both forms are identical (unit-tested: tools/test_navrl_p3_math.py). Zero the next-state potential on
        # TRUE terminals (capture/crash) per Grzes 2017; a timeout is a truncation (not in `term`)
        # so it keeps its gradient. Disable by setting progress_weight = 0.0.
        if self.rw.get("progress_weight", 0.0) != 0.0:
            gamma = self.task_config.progress_gamma
            term = self.captured_now | self.crashed_now
            prev_dist_anchored = (self.target_position - self.prev_pos).norm(dim=1)
            phi_next = torch.where(term, torch.zeros_like(dist), gamma * dist)
            self.rewards[:] = self.rewards + self.rw["progress_weight"] * (
                prev_dist_anchored - phi_next
            )

        # roll the swept-segment / PBRS buffers forward (reset_idx re-seeds them for reset envs)
        self.prev_pos[:] = pos
        self.prev_rel[:] = rel

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
        # Do not reward-shape envs that just FINISHED (crash reward is the collision penalty;
        # capture reward is the terminal bonus; TRUNCATED envs were already reset before this
        # render, so their scan belongs to the NEXT episode's spawn — shaping them would leak
        # next-episode state into this step's reward).
        alive = (self.terminations <= 0) & (self.truncations <= 0)
        self.rewards[alive] += self.rw["safety_static_weight"] * r_ss[alive]
        # (The B/C/D near-obstacle clearance penalty that used to live here was removed: three
        #  null results — crash proved geometric, not reward-driven. See CRASH_TUNING_LOG.md;
        #  code in git history @44e86a2.)

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
        epoch, so epoch ~= num_task_steps / ppo_horizon). num_task_steps is saved/restored via
        get_env_state/set_env_state, so a --checkpoint resume (and --play) continues at the saved
        curriculum position."""
        warmup_steps = max(1, int(self.cur.k_warmup_epochs) * int(self.cur.ppo_horizon))
        frac = min(1.0, self.num_task_steps / warmup_steps)
        return self.cur.k_start + (self.cur.k_final - self.cur.k_start) * frac

    def _goal_x_min(self):
        """Goal-x floor: stays at k_min early, then ramps k_min -> k_min_final over
        [k_min_ramp_start_epochs, +k_min_ramp_epochs] so late episodes drop the easy near goals and
        focus on deep crossings. The start is independent of the k_max ramp (they may overlap). Kept
        at least 1 m below k_max so the [min, max] window stays valid."""
        h = int(self.cur.ppo_horizon)
        start_steps = int(self.cur.k_min_ramp_start_epochs) * h
        ramp_steps = max(1, int(self.cur.k_min_ramp_epochs) * h)
        frac = min(1.0, max(0.0, (self.num_task_steps - start_steps) / ramp_steps))
        k_min = self.cur.k_min + (self.cur.k_min_final - self.cur.k_min) * frac
        return min(k_min, self._goal_x_max() - 1.0)

    def _target_speed_max(self):
        """Phase 3: epoch-proportional target-speed ceiling — 0 -> speed_final over
        [speed_ramp_start_epochs, +speed_ramp_epochs], then holds. Same num_task_steps epoch proxy
        as _goal_x_max, so it survives --checkpoint resume and is restored at --play. An explicit
        NAVRL_TARGET_SPEED (speed_fixed >= 0, evaluation cells) bypasses the curriculum entirely."""
        if float(self.tm.speed_fixed) >= 0.0:
            return float(self.tm.speed_fixed)
        final = float(self.tm.speed_final)
        if final <= 0.0:
            return 0.0
        h = int(self.cur.ppo_horizon)
        start_steps = int(self.tm.speed_ramp_start_epochs) * h
        ramp_steps = max(1, int(self.tm.speed_ramp_epochs) * h)
        frac = min(1.0, max(0.0, (self.num_task_steps - start_steps) / ramp_steps))
        return final * frac

    def _sample_target_motion(self, env_ids):
        """Per-episode target speed + trajectory pattern for reset envs. Training samples
        speed ~ U[0, v_max(epoch)] so static episodes stay in-distribution (the v_t=0 skill is
        never forgotten); NAVRL_TARGET_SPEED forces the exact speed instead (evaluation cells)."""
        n = len(env_ids)
        if n == 0:
            return
        v_max = self._target_speed_max()
        if float(self.tm.speed_fixed) >= 0.0:
            speed = torch.full((n,), float(self.tm.speed_fixed), device=self.device)
        else:
            speed = v_max * torch.rand(n, device=self.device)
        self._tm_speed[env_ids] = speed

        pat = str(self.tm.pattern)
        if pat == "mixed":
            code = torch.randint(0, 2, (n,), device=self.device)  # cv | waypoint, 50:50
        elif pat == "cv":
            code = torch.zeros(n, dtype=torch.long, device=self.device)
        elif pat == "waypoint":
            code = torch.ones(n, dtype=torch.long, device=self.device)
        elif pat == "circle":
            code = torch.full((n,), 2, dtype=torch.long, device=self.device)
        else:
            raise ValueError(
                f"unknown NAVRL_TARGET_PATTERN '{pat}' (expected cv|waypoint|circle|mixed)"
            )
        self._tm_pattern[env_ids] = code

        # cv: random persistent heading (reflected at walls while the episode runs)
        ang = 2.0 * math.pi * torch.rand(n, device=self.device)
        self._tm_cv_vel[env_ids, 0] = speed * torch.cos(ang)
        self._tm_cv_vel[env_ids, 1] = speed * torch.sin(ang)
        # waypoint: first waypoint anywhere inside the wall margins
        self._tm_waypoint[env_ids] = self._sample_waypoints(env_ids)
        # circle: ring around the (bar-clear) spawn goal, random direction
        self._tm_circle_center[env_ids] = self.target_position[env_ids, 0:2]
        r = max(1e-6, float(self.tm.circle_radius))
        sign = torch.where(
            torch.rand(n, device=self.device) < 0.5,
            torch.full((n,), -1.0, device=self.device),
            torch.full((n,), 1.0, device=self.device),
        )
        self._tm_circle_angvel[env_ids] = sign * speed / r
        # realized velocity starts at zero; _advance_target sets it from actual displacement
        self.target_vel_w[env_ids] = 0.0

    def _sample_waypoints(self, env_ids):
        """Uniform random XY waypoints inside the wall margins (per-env bounds)."""
        b_min = self.obs_dict["env_bounds_min"][env_ids]
        b_max = self.obs_dict["env_bounds_max"][env_ids]
        m = float(self.cur.wall_margin)
        lo = b_min[:, 0:2] + m
        hi = b_max[:, 0:2] - m
        return lo + (hi - lo) * torch.rand(len(env_ids), 2, device=self.device)

    def _advance_target(self):
        """Phase 3: integrate the virtual target one RL step (step_dt = 0.1 s). Patterns:
        cv (heading held, reflected at the wall margins), waypoint (random waypoints), circle
        (parametric ring; the angle is re-derived from the current position each step, so bar
        push-outs simply slide the target around the ring). All patterns are then pushed out of
        bar clearance and clamped inside the wall margins; target_vel_w is set from the REALIZED
        displacement so the range-rate reward always matches the actual motion (reflections and
        push-outs included). Static episodes (speed 0 — the Phases 1-2 default) exit immediately,
        keeping the task byte-identical."""
        moving = self._tm_speed > 1e-6
        if not bool(moving.any()):
            return  # target_vel_w stays exactly zero -> range-rate == static vel term

        dt = self.step_dt
        old_xy = self.target_position[:, 0:2].clone()
        new_xy = old_xy.clone()
        b_min = self.obs_dict["env_bounds_min"]
        b_max = self.obs_dict["env_bounds_max"]
        m = float(self.cur.wall_margin)
        lo = b_min[:, 0:2] + m
        hi = b_max[:, 0:2] - m

        # -- cv: integrate the held heading, reflect position AND velocity at the wall margins
        cv = moving & (self._tm_pattern == 0)
        if bool(cv.any()):
            p = old_xy[cv] + self._tm_cv_vel[cv] * dt
            v = self._tm_cv_vel[cv]
            for ax in (0, 1):
                below = p[:, ax] < lo[cv, ax]
                above = p[:, ax] > hi[cv, ax]
                p[:, ax] = torch.where(below, 2.0 * lo[cv, ax] - p[:, ax], p[:, ax])
                p[:, ax] = torch.where(above, 2.0 * hi[cv, ax] - p[:, ax], p[:, ax])
                v[:, ax] = torch.where(below | above, -v[:, ax], v[:, ax])
            self._tm_cv_vel[cv] = v
            new_xy[cv] = p

        # -- waypoint: head toward the waypoint at the episode speed; resample on arrival
        wp = moving & (self._tm_pattern == 1)
        if bool(wp.any()):
            to_wp = self._tm_waypoint[wp] - old_xy[wp]
            d_wp = to_wp.norm(dim=1, keepdim=True).clamp(min=1e-6)
            step_len = (self._tm_speed[wp] * dt).unsqueeze(1)
            # do not overshoot the waypoint within a step
            move = to_wp / d_wp * torch.minimum(step_len, d_wp)
            new_xy[wp] = old_xy[wp] + move
            reached = (self._tm_waypoint[wp] - new_xy[wp]).norm(dim=1) < float(
                self.tm.waypoint_reach_m
            )
            if bool(reached.any()):
                wp_idx = wp.nonzero(as_tuple=False).squeeze(-1)
                self._tm_waypoint[wp_idx[reached]] = self._sample_waypoints(wp_idx[reached])

        # -- circle: orbit the center at the CURRENT radius (adaptive). Deriving both angle and
        # radius from the current position each step means a bar push-out simply enlarges the
        # orbit instead of fighting a fixed-radius snap-back (which oscillated in testing).
        ci = moving & (self._tm_pattern == 2)
        if bool(ci.any()):
            rel_c = old_xy[ci] - self._tm_circle_center[ci]
            r_cur = rel_c.norm(dim=1).clamp(min=max(0.5, 1e-6))
            theta = torch.atan2(rel_c[:, 1], rel_c[:, 0])
            # keep the TANGENTIAL speed at the episode speed regardless of the current radius
            omega = torch.sign(self._tm_circle_angvel[ci]) * self._tm_speed[ci] / r_cur
            theta = theta + omega * dt
            new_xy[ci] = self._tm_circle_center[ci] + r_cur.unsqueeze(1) * torch.stack(
                (torch.cos(theta), torch.sin(theta)), dim=1
            )

        # -- keep the capture sphere flyable: push the target out of bar clearance. With bars only
        # 1.5 m apart and a 1.0 m clearance the exclusion discs OVERLAP: a naive radial push out of
        # the nearest bar PING-PONGS forever inside the lens between two discs (verified by probe —
        # push out of A lands in B's disc and vice versa). Instead push along the COMPOSITE of the
        # unit away-vectors of ALL violating bars: in a symmetric lens that resolves to the
        # perpendicular escape direction. Iterated with the wall clamp so a wall-adjacent bar
        # slides the target along the wall instead of fighting the clamp.
        clearance = float(getattr(self.task_config, "goal_min_bar_clearance", 0.0))
        pushed_any = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        if clearance > 0.0:
            mov_idx = moving.nonzero(as_tuple=False).squeeze(-1)
            bars_xy = self.obs_dict["obstacle_position"][mov_idx][:, :, 0:2]  # (M, B, 2)
            arangeM = torch.arange(len(mov_idx), device=self.device)
            for _ in range(6):
                new_xy = torch.maximum(torch.minimum(new_xy, hi), lo)  # walls first
                diff = new_xy[mov_idx].unsqueeze(1) - bars_xy          # (M, B, 2)
                d_all = diff.norm(dim=2)                               # (M, B)
                viol = d_all < clearance
                rows = viol.any(dim=1)
                if not bool(rows.any()):
                    break
                unit = diff / d_all.clamp(min=1e-6).unsqueeze(2)
                comp = (unit * viol.unsqueeze(2).float()).sum(dim=1)   # (M, 2)
                comp_n = comp.norm(dim=1, keepdim=True)
                # degenerate (dead-center of a symmetric lens): fall back to away-from-nearest
                jmin = d_all.min(dim=1).indices
                fallback = new_xy[mov_idx] - bars_xy[arangeM, jmin]
                fallback = fallback / fallback.norm(dim=1, keepdim=True).clamp(min=1e-6)
                dirn = torch.where(comp_n > 1e-6, comp / comp_n.clamp(min=1e-6), fallback)
                step_out = (clearance - d_all).clamp(min=0.0).max(dim=1).values + 1e-3
                sel = mov_idx[rows]
                new_xy[sel] = new_xy[sel] + dirn[rows] * step_out[rows].unsqueeze(1)
                pushed_any[sel] = True
            # a pushed WAYPOINT target was heading somewhere unreachable next to a bar —
            # resample its waypoint so it does not keep fighting the push-out every step
            re_wp = pushed_any & (self._tm_pattern == 1)
            if bool(re_wp.any()):
                re_idx = re_wp.nonzero(as_tuple=False).squeeze(-1)
                self._tm_waypoint[re_idx] = self._sample_waypoints(re_idx)

        # -- final wall clamp (physical bound) and write-back
        new_xy = torch.maximum(torch.minimum(new_xy, hi), lo)
        self.target_position[:, 0:2] = torch.where(
            moving.unsqueeze(1), new_xy, old_xy
        )
        self.target_position[:, 2] = self.task_config.flight_altitude
        # realized world velocity (z = 0): what the range-rate reward sees
        self.target_vel_w[:, 0:2] = torch.where(
            moving.unsqueeze(1),
            (self.target_position[:, 0:2] - old_xy) / dt,
            torch.zeros_like(old_xy),
        )
        self.target_vel_w[:, 2] = 0.0

    def _update_curriculum(self, successes, finished):
        # Goal-distance curriculum stays epoch-proportional via _goal_x_max/_goal_x_min. This hook
        # only promotes the optional Phase-2 density curriculum.
        if self.density is None or not getattr(self.density, "use_density_curriculum", False):
            return
        # Gate on warmup FIRST so the capture accumulators only collect POST-warmup episodes.
        # Accumulating during warmup (early low-capture training) would make the first promotion check
        # use a lifetime average dragged below threshold and stall the first promotion by one window.
        horizon = int(getattr(self.cur, "ppo_horizon", 1))
        warmup_steps = int(getattr(self.density, "warmup_epochs", 0)) * max(1, horizon)
        if self.num_task_steps < warmup_steps:
            return
        n_fin = int(torch.sum(finished).item())
        if n_fin <= 0:
            return

        self._density_succ_agg += int(torch.sum(successes).item())
        self._density_fin_agg += n_fin

        check_after = max(1, int(getattr(self.density, "check_after_episodes", 2048)))
        if self._density_fin_agg < check_after:
            return

        capture_rate = self._density_succ_agg / max(1, self._density_fin_agg)
        threshold = float(getattr(self.density, "success_threshold", 0.8))
        final_bars = self._clamp_active_bars(getattr(self.density, "n_final", self.n_bars_active))
        if capture_rate >= threshold and self.n_bars_active < final_bars:
            step = max(1, int(getattr(self.density, "promote_step", 15)))
            old_bars = self.n_bars_active
            self._set_active_bars(min(final_bars, old_bars + step))
            logger.warning(
                "NavRL density curriculum promoted | bars %d -> %d after %d eps, capture=%.3f"
                % (old_bars, self.n_bars_active, self._density_fin_agg, capture_rate)
            )
        else:
            logger.info(
                "NavRL density curriculum held | bars=%d capture=%.3f over %d eps"
                % (self.n_bars_active, capture_rate, self._density_fin_agg)
            )

        self._density_succ_agg = 0
        self._density_fin_agg = 0

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
        tm_on = float(self.tm.speed_final) > 0.0 or float(self.tm.speed_fixed) >= 0.0
        record_navrl_epoch_episodes(
            num_finished=n_fin,
            num_captured=int(torch.sum(successes).item()),
            num_crash=int(torch.sum(crashes > 0).item()),
            num_timeout=int(torch.sum(timeouts).item()),
            closest_nocrash_sum=closest_nc_sum,
            closest_nocrash_count=n_nc,
            closest_min=closest_min,
            goal_dist_max=self.cur_goal_dist_max,
            goal_dist_min=self.cur_goal_dist_min,
            n_bars_active=self.n_bars_active,
            target_speed_max=self._target_speed_max() if tm_on else None,
            target_speed_mean=float(self._tm_speed.mean().item()) if tm_on else None,
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
