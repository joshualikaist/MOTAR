import json
import math
import os
from pathlib import Path

import numpy as np
import torch
from gym.spaces import Box, Dict

from aerial_gym.task.base_task import BaseTask
from aerial_gym.task.navrl_task.train_dashboard import record_navrl_epoch_episodes
from aerial_gym.sim.sim_builder import SimBuilder
from aerial_gym.utils.math import quat_rotate, quat_rotate_inverse
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

        # In vision mode the LiDAR renderer owns a per-environment analytic target center. The
        # task mirrors the moving target into that buffer before every LiDAR render. Camera
        # perception uses the same task target only inside its renderer; neither path exposes the
        # ground-truth coordinate directly to the actor.
        self._sensor_target = self.obs_dict.get("navrl_target_position", None)
        self.has_vision_target = self._sensor_target is not None

        self.max_bars_available = self._get_max_bars_available()
        initial_bars_requested = self._initial_active_bars()
        self.n_bars_active = 0
        self._density_succ_agg = 0
        self._density_fin_agg = 0
        # competence-gated goal-DISTANCE window (NAVRL_K_COMPETENCE): advances by measured capture
        # instead of by epoch. Seeded to the shallow start; persisted across --checkpoint resume.
        self._k_max_cur = float(self.task_config.curriculum.k_start)
        self._k_min_cur = float(self.task_config.curriculum.k_min)
        self._kcomp_succ = 0
        self._kcomp_fin = 0
        self._set_active_bars(initial_bars_requested)
        logger.warning(
            "NavRL density config | initial_bars=%d max_bars=%d curriculum=%s "
            "final=%d step=%d threshold=%.3f check_eps=%d"
            % (
                self.n_bars_active,
                self.max_bars_available,
                bool(getattr(self.density, "use_density_curriculum", False)),
                int(getattr(self.density, "n_final", self.n_bars_active)),
                int(getattr(self.density, "promote_step", 0)),
                float(getattr(self.density, "success_threshold", 0.0)),
                int(getattr(self.density, "check_after_episodes", 0)),
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

        # --- Phase-3 vision pivot (NAVRL_VISION=1): sensor-only actor. See task_config.vision.
        self.vis_cfg = getattr(self.task_config, "vision", None)
        self.vision_mode = bool(self.vis_cfg is not None and self.vis_cfg.enable)
        self.perception_cfg = getattr(self.task_config, "perception", None)
        self.perception_mode = bool(
            self.vision_mode
            and self.perception_cfg is not None
            and self.perception_cfg.enable
        )
        self.detector = None
        self.perception = None
        self.prev_action = torch.zeros((self.num_envs, 4), device=self.device)
        self._visible_now = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        if self.vision_mode:
            if not self.has_vision_target:
                raise RuntimeError(
                    "NAVRL_VISION=1 requires the LiDAR target buffer "
                    "(navrl_target_position missing)."
                )
            from aerial_gym.task.navrl_task.navrl_detector import NavRLTargetDetector

            self.detector = NavRLTargetDetector(
                warp_env=self.sim_env.warp_env,
                num_envs=self.num_envs,
                device=self.device,
                vis_cfg=self.vis_cfg,
                step_dt=self.step_dt,
            )
            if self.perception_mode:
                from aerial_gym.task.navrl_task.navrl_perception import (
                    NavRLPerceptionModule,
                    STRUCTURED_OBS_DIM,
                )

                if int(self.task_config.observation_space_dim) != int(STRUCTURED_OBS_DIM):
                    raise RuntimeError(
                        "NavRL perception schema mismatch: task=%d module=%d"
                        % (self.task_config.observation_space_dim, STRUCTURED_OBS_DIM)
                    )
                self.perception = NavRLPerceptionModule(
                    num_envs=self.num_envs,
                    device=self.device,
                    cfg=self.perception_cfg,
                    step_dt=self.step_dt,
                    camera_cfg=self.vis_cfg,
                )
            logger.warning(
                "NavRL %s mode | actor obs=%d, "
                "critic states=%d, body-frame actions, detector range=%.1fm hfov=%.0fdeg"
                % (
                    "PERCEPTION+TRANSFORMER" if self.perception_mode else "VISION-ORACLE-BASELINE",
                    self.task_config.observation_space_dim,
                    self.task_config.state_space_dim,
                    self.vis_cfg.detector_max_range,
                    self.vis_cfg.detector_hfov_deg,
                )
            )
            if self.perception_mode:
                # Echo the knobs that change the obstacle REPRESENTATION. Without this a run leaves
                # no trace of how its tokens were selected, and a finished run cannot be interpreted
                # after the fact -- exactly what happened to ppo_260727_0930, whose token FOV could
                # not be recovered from either the log or the checkpoint.
                from aerial_gym.task.navrl_task.navrl_perception import (
                    HBEAMS,
                    MAX_OBSTACLES,
                    OBSTACLE_FOV_DEG,
                    OBSTACLE_SUPPRESS_DEG,
                    VBEAMS,
                )

                logger.warning(
                    "NavRL obstacle representation | tokens=%d token_fov=%.0fdeg "
                    "suppress=+-%.0fdeg scan=%dx%d lidar_range=%.1fm"
                    % (
                        MAX_OBSTACLES,
                        OBSTACLE_FOV_DEG,
                        OBSTACLE_SUPPRESS_DEG,
                        VBEAMS,
                        HBEAMS,
                        self.task_config.lidar_max_range,
                    )
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
        # vision mode: privileged critic input = actor obs + GT extras (asymmetric actor-critic;
        # rl_games central_value reads obs dict key 'states', forwarded by ExtractObsWrapper)
        if self.vision_mode:
            self.task_obs["states"] = torch.zeros(
                (self.num_envs, self.task_config.state_space_dim), device=self.device
            )
        self.command = torch.zeros((self.num_envs, 4), device=self.device)  # controller input
        self._yaw_cmd = torch.zeros(self.num_envs, device=self.device)  # (b) current-step yaw action a[:,3], penalized quadratically (magnitude damping)
        self._z_err_integral = torch.zeros(self.num_envs, device=self.device)  # altitude-hold PI integral term (see transform_action_to_command)
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
        # Machine-readable vectorized evaluation. rl_games' player summary contains reward/length
        # only, while the periodic task summary historically waited for 2048 episodes. As a result,
        # a perfectly valid 1000-game screen completed without preserving capture/crash data. In
        # bulk mode, make the requested player game count the summary interval and atomically write
        # one result document before the vector player exits.
        self._bulk_eval_mode = os.environ.get(
            "NAVRL_BULK_EVAL", "0"
        ).strip().lower() in ("1", "true", "yes", "on")
        try:
            self._bulk_eval_target = max(
                1, int(os.environ.get("PLAY_GAMES_NUM", "1000"))
            )
        except ValueError:
            self._bulk_eval_target = 1000
        self._bulk_eval_output = os.environ.get(
            "NAVRL_BULK_EVAL_JSON", ""
        ).strip()
        self._bulk_eval_exported = False
        self._progress_log_interval = (
            self._bulk_eval_target if self._bulk_eval_mode else 2048
        )
        # Policy-side action diagnostics. `prev_action` stores the post-clamp observation and cannot
        # reveal how much Gaussian probability was collapsed onto +/-1, so measure the actor output
        # immediately on entry to step(). This is instrumentation only; it never changes commands,
        # observations, rewards, or terminations.
        self._action_diag_enabled = self._bulk_eval_mode or os.environ.get(
            "NAVRL_ACTION_DIAG", "0"
        ).strip().lower() in ("1", "true", "yes", "on")
        self._action_diag = self._empty_action_diag()
        self._action_diag_prev = torch.zeros(
            (self.num_envs, self.task_config.action_space_dim), device=self.device
        )
        self._action_diag_prev_valid = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._action_front_mask = None
        # --- crash-cause diagnosis (NAVRL_CRASH_DIAG=1): split the aggregate "crash" number into
        # its termination source (bar contact / height bound / out-of-arena side) so a stuck run
        # can be diagnosed from measured counts instead of guesses. Off by default: zero overhead.
        self._oob_probe = os.environ.get("NAVRL_OOB_PROBE", "0").strip().lower() in (
            "1", "true", "yes", "on"
        )
        self._crash_diag = self._bulk_eval_mode or self._oob_probe or os.environ.get(
            "NAVRL_CRASH_DIAG", "0"
        ).strip().lower() in ("1", "true", "yes", "on")
        self._diag = {k: 0 for k in ("contact", "below", "above", "oob", "oob_w", "oob_e", "oob_s", "oob_n")}
        # "oob" is the EXACT per-env count; W/E/S/N are informational side buckets (a rare
        # diagonal corner exit can land in two of them, so their sum may exceed "oob").
        self._diag_steps = {"contact": 0.0, "oob": 0.0, "below": 0.0}  # steps-to-death sums per cause
        self._diag_below_tilt = 0.0  # sum of tilt angle [deg] at the moment of each below-death
        self._diag_x_sum = 0.0  # death-x sum for bar contacts (bar band starts ~3.1 m)
        # NAVRL_OOB_PROBE=1 is evaluation-only instrumentation. None of these tensors enter the
        # actor/critic observation. Side-aligned values are positive toward the wall that was
        # crossed, letting N and S exits be pooled without hiding a directional policy bias.
        self._probe_ep_start_y = torch.zeros(self.num_envs, device=self.device)
        self._probe_ep_target_start_y = torch.zeros(self.num_envs, device=self.device)
        self._probe_ep_bar_mean_y = torch.zeros(self.num_envs, device=self.device)
        self._probe_ep_y_min = torch.zeros(self.num_envs, device=self.device)
        self._probe_ep_y_max = torch.zeros(self.num_envs, device=self.device)
        self._probe = {
            k: 0.0
            for k in (
                "n",
                "start_y",
                "goal_pull_side",
                "goal_now_pull_side",
                "bar_bias_side",
                "world_vy_side",
                "command_vy_side",
                "action_y_side",
                "excursion_side",
                "visible",
                "track_age",
                "track_cov_pos",
            )
        }
        # NAVRL_BAR_PROBE=1: evaluation-only bar-contact forensics (zero overhead when off).
        # Probe v2 uses both bearing and range to associate LiDAR surface tokens with GT bars. It
        # also reports collisions inside the token-selection FOV separately; comparing all 240-deg
        # hits against a geometric coverage estimate silently counted the excluded rear 120 deg.
        self._bar_probe = os.environ.get("NAVRL_BAR_PROBE", "0").strip().lower() in (
            "1", "true", "yes", "on"
        )
        if self._bar_probe:
            self._crash_diag = True
        self._bprobe = {
            k: 0.0
            for k in (
                "n",                    # bar-contact deaths sampled
                "bars_in_range",        # GT bars within the LiDAR horizon at impact
                "bars_in_token_fov",    # in-range GT bars eligible for token selection
                "occupied_bins",        # scan bearings returning an obstacle
                "hit_dist",             # center distance to the struck bar
                "hit_in_token_fov",     # struck bar is inside the configured token FOV
                "hit_in_tokens",        # v2 surface-ray/range association to the struck bar
                "hit_in_tokens_in_fov", # v2 match and the struck bar is inside the token FOV
                "valid_tokens",         # valid token slots at impact
                "associated_tokens",    # tokens associated with any GT bar
                "unique_token_bars",    # distinct GT bars represented by associated tokens
                "duplicate_tokens",     # associated slots spent on an already represented GT bar
                "hit_center_offset",    # token surface point to struck-bar center (not an error)
                "hit_cross_track",      # lateral distance from token ray to struck-bar center
                "hit_radial_gap",       # bar-center range minus token surface range
                "hit_token_rank",       # matched slot (0 = nearest)
            )
        }

        # Native 3-D application controls. They are completely disabled during ordinary train/play
        # runs and never become actor observations. The debug target overlay uses GT only for the
        # human viewer; sensor/perception tensors remain the policy's sole target input.
        self.interactive_mode = os.environ.get("NAVRL_INTERACTIVE", "0").strip().lower() in (
            "1", "true", "yes", "on"
        )
        self.general_eval_mode = os.environ.get("NAVRL_GENERAL_EVAL", "0").strip().lower() in (
            "1", "true", "yes", "on"
        )
        self.general_train_mode = os.environ.get("NAVRL_GENERAL_TRAIN", "0").strip().lower() in (
            "1", "true", "yes", "on"
        )
        self.general_spawn_mode = self.general_eval_mode or self.general_train_mode
        self.general_density_min = int(os.environ.get("NAVRL_GENERAL_DENSITY_MIN", "25"))
        self.general_density_max = int(os.environ.get("NAVRL_GENERAL_DENSITY_MAX", "110"))
        self.general_num_trials = max(1, int(os.environ.get("NAVRL_GENERAL_NUM_TRIALS", "10")))
        self.general_trial_index = 0
        self.general_completed_trials = 0
        self.general_successes = 0
        self.general_crashes = 0
        self.general_timeouts = 0
        self.general_trial_records = []
        self._general_results_exported = False
        self._hud = None
        self._hud_last_outcome = ""
        if self.general_eval_mode and self.num_envs != 1:
            raise RuntimeError("NAVRL_GENERAL_EVAL currently requires exactly one viewer env.")
        self._interactive_reset_requested = False
        self._interactive_show_lidar = True
        self._interactive_manual = False
        self._interactive_manual_keys = {}
        self._interactive_manual_action = torch.zeros(
            (self.num_envs, self.task_config.action_space_dim), device=self.device
        )
        self._interactive_target_trail = []
        self._runtime_target_speed = None
        if self.interactive_mode:
            self._register_interactive_viewer()

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
        # An explicit NAVRL_NUM_BARS ALWAYS wins — even with the density curriculum flag left on —
        # so density-sweep evals/resumes run at the REQUESTED density instead of silently falling
        # back to n_start (this mirrors the same "NAVRL_NUM_BARS wins" rule in set_env_state).
        if self.density is not None and os.environ.get("NAVRL_NUM_BARS", "").strip():
            requested = getattr(self.density, "num_bars_active", self.max_bars_available)
        elif self.density is None:
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

    def set_runtime_bars(self, n_bars):
        """Change density for subsequent resets and request an immediate all-env reset."""
        value = self._set_active_bars(n_bars)
        self._interactive_reset_requested = True
        return value

    def set_runtime_target_speed(self, speed_mps):
        """Force one exact target speed for interactive episodes (not a training curriculum)."""
        self._runtime_target_speed = max(0.0, float(speed_mps))
        self._interactive_reset_requested = True
        return self._runtime_target_speed

    def set_runtime_drone_speed(self, speed_mps):
        """Set the action-to-velocity scale used by the controller."""
        self.task_config.max_velocity = max(0.25, float(speed_mps))
        return float(self.task_config.max_velocity)

    def _sample_general_density(self):
        """Sample one randomized clutter level for the next single-env evaluation trial."""
        lo = self._clamp_active_bars(min(self.general_density_min, self.general_density_max))
        hi = self._clamp_active_bars(max(self.general_density_min, self.general_density_max))
        value = int(torch.randint(lo, hi + 1, (1,), device=self.device).item())
        self._set_active_bars(value, log=False)
        self.general_trial_index += 1
        logger.warning(
            "NavRL general trial %d/%d | randomized bars=%d"
            % (self.general_trial_index, self.general_num_trials, value)
        )

    def _record_general_result(self, successes, crashes, timeouts, finished):
        if not self.general_eval_mode or not bool(finished.any()):
            return
        self.general_completed_trials += int(finished.sum().item())
        self.general_successes += int(successes.sum().item())
        self.general_crashes += int((crashes > 0).sum().item())
        self.general_timeouts += int(timeouts.sum().item())
        outcome = "captured" if bool(successes.any()) else (
            "crashed" if bool((crashes > 0).any()) else "timeout"
        )
        env_ids = finished.nonzero(as_tuple=False).squeeze(-1)
        if env_ids.ndim == 0:
            env_ids = env_ids.unsqueeze(0)
        for env_id in env_ids.tolist():
            self.general_trial_records.append(
                {
                    "trial": int(self.general_completed_trials),
                    "bars": int(self.n_bars_active),
                    "outcome": outcome,
                    "min_goal_dist_m": float(self.ep_min_goal_dist[env_id].item()),
                    "steps": int(self.sim_env.sim_steps[env_id].item()),
                    "target_speed_mps": float(self._tm_speed[env_id].item()),
                }
            )
        logger.warning(
            "NavRL general result %d/%d | %s"
            % (
                min(self.general_completed_trials, self.general_num_trials),
                self.general_num_trials,
                outcome,
            )
        )
        if self._hud is not None:
            from aerial_gym.apps.navrl_3d_hud import NavRL3DHud, build_hud_lines, build_hud_pip

            flash_text = outcome.upper()
            if outcome == "captured":
                self._hud.flash(flash_text, color=NavRL3DHud.OK)
            elif outcome == "crashed":
                self._hud.flash(flash_text, color=NavRL3DHud.BAD)
            else:
                self._hud.flash(flash_text, color=NavRL3DHud.WARN)
        if self.general_completed_trials >= self.general_num_trials:
            logger.warning(
                "NavRL general summary | captured=%d crash=%d timeout=%d / %d"
                % (
                    self.general_successes,
                    self.general_crashes,
                    self.general_timeouts,
                    self.general_num_trials,
                )
            )
            self._export_general_results_json()

    def _export_general_results_json(self):
        if self._general_results_exported or not self.general_eval_mode:
            return
        path = os.environ.get("NAVRL_GENERAL_RESULTS_JSON", "").strip()
        if not path:
            return
        payload = {
            "num_trials": int(self.general_num_trials),
            "density_min": int(self.general_density_min),
            "density_max": int(self.general_density_max),
            "target_speed_mps": float(os.environ.get("NAVRL_TARGET_SPEED", "0") or 0.0),
            "drone_max_speed_mps": float(self.task_config.max_velocity),
            "summary": {
                "captured": int(self.general_successes),
                "crash": int(self.general_crashes),
                "timeout": int(self.general_timeouts),
            },
            "trials": list(self.general_trial_records),
        }
        try:
            out = Path(path)
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
            logger.warning("NavRL general results saved -> %s" % out)
            if self._hud is not None:
                cap = payload["summary"]["captured"]
                self._hud.set_summary(
                    [
                        "Evaluation complete",
                        "Captured %d / %d" % (cap, self.general_num_trials),
                        "Crash %d  Timeout %d" % (
                            payload["summary"]["crash"],
                            payload["summary"]["timeout"],
                        ),
                        "Saved %s" % out.name,
                    ]
                )
        except OSError as exc:
            logger.warning("NavRL general results export failed: %s" % exc)
        self._general_results_exported = True

    def _randomize_general_drone_spawn(self, env_ids):
        """Place the drone at a collision-free random XY/yaw for generalized evaluation."""
        n = len(env_ids)
        if n == 0:
            return
        b_min = self.obs_dict["env_bounds_min"][env_ids]
        b_max = self.obs_dict["env_bounds_max"][env_ids]
        bars = self.obs_dict["obstacle_position"][env_ids, : self.n_bars_active, 0:2]
        lo = b_min[:, 0:2] + 1.0
        hi = b_max[:, 0:2] - 1.0
        chosen = lo + (hi - lo) * torch.rand((n, 2), device=self.device)
        todo = torch.ones(n, dtype=torch.bool, device=self.device)
        for _ in range(64):
            if not bool(todo.any()):
                break
            ids = todo.nonzero(as_tuple=False).squeeze(-1)
            candidate = lo[ids] + (hi[ids] - lo[ids]) * torch.rand(
                (len(ids), 2), device=self.device
            )
            if bars.shape[1] > 0:
                clearance = torch.cdist(candidate.unsqueeze(1), bars[ids]).squeeze(1).min(1).values
                accepted = clearance >= 0.65
            else:
                accepted = torch.ones(len(ids), dtype=torch.bool, device=self.device)
            chosen[ids[accepted]] = candidate[accepted]
            todo[ids[accepted]] = False

        self.obs_dict["robot_position"][env_ids, 0:2] = chosen
        self.obs_dict["robot_position"][env_ids, 2] = float(self.task_config.flight_altitude)
        yaw = -math.pi + 2.0 * math.pi * torch.rand(n, device=self.device)
        quat = torch.zeros((n, 4), device=self.device)
        quat[:, 2] = torch.sin(0.5 * yaw)
        quat[:, 3] = torch.cos(0.5 * yaw)
        self.obs_dict["robot_orientation"][env_ids] = quat
        self.obs_dict["robot_linvel"][env_ids] = 0.0
        self.obs_dict["robot_angvel"][env_ids] = 0.0
        self.sim_env.robot_manager.robot.update_states()
        self.sim_env.IGE_env.write_to_sim()

    def _sample_general_target(self, env_ids, start_pos, b_min, b_max, bars_xy):
        """Sample a visible-range, collision-free target independently of the drone spawn."""
        n = len(env_ids)
        lo = b_min[:, 0:2] + 1.0
        hi = b_max[:, 0:2] - 1.0
        chosen = lo + (hi - lo) * torch.rand((n, 2), device=self.device)
        todo = torch.ones(n, dtype=torch.bool, device=self.device)
        for _ in range(96):
            if not bool(todo.any()):
                break
            ids = todo.nonzero(as_tuple=False).squeeze(-1)
            candidate = lo[ids] + (hi[ids] - lo[ids]) * torch.rand(
                (len(ids), 2), device=self.device
            )
            drone_dist = torch.norm(candidate - start_pos[ids, 0:2], dim=1)
            max_dist = 18.0 if self.general_eval_mode else min(18.0, float(self._goal_x_max()))
            min_dist = min(4.0, max(1.0, max_dist - 1.0))
            accepted = (drone_dist >= min_dist) & (drone_dist <= max_dist)
            if bars_xy.shape[1] > 0:
                bar_dist = (
                    torch.cdist(candidate.unsqueeze(1), bars_xy[ids, : self.n_bars_active])
                    .squeeze(1)
                    .min(1)
                    .values
                )
                accepted &= bar_dist >= 0.65
            chosen[ids[accepted]] = candidate[accepted]
            todo[ids[accepted]] = False
        goal = start_pos.clone()
        goal[:, 0:2] = chosen
        goal[:, 2] = float(self.task_config.flight_altitude)
        return goal

    def _register_interactive_viewer(self):
        """Attach NavRL controls and overlays to the already-created Isaac Gym viewer."""
        from isaacgym import gymapi

        viewer = getattr(getattr(self.sim_env, "IGE_env", None), "viewer", None)
        if viewer is None or getattr(viewer, "viewer", None) is None:
            raise RuntimeError("NAVRL_INTERACTIVE=1 requires headless=False (no viewer was created).")

        def on_press(fn):
            return lambda value: fn() if value > 0 else None

        if not self.general_eval_mode:
            viewer.subscribe_keyboard_event(
                gymapi.KEY_LEFT_BRACKET,
                "navrl_bars_down",
                on_press(lambda: self._interactive_change_bars(-5)),
            )
            viewer.subscribe_keyboard_event(
                gymapi.KEY_RIGHT_BRACKET,
                "navrl_bars_up",
                on_press(lambda: self._interactive_change_bars(5)),
            )
        viewer.subscribe_keyboard_event(
            gymapi.KEY_COMMA,
            "navrl_target_speed_down",
            on_press(lambda: self._interactive_change_target_speed(-0.25)),
        )
        viewer.subscribe_keyboard_event(
            gymapi.KEY_PERIOD,
            "navrl_target_speed_up",
            on_press(lambda: self._interactive_change_target_speed(0.25)),
        )
        viewer.subscribe_keyboard_event(
            gymapi.KEY_MINUS,
            "navrl_drone_speed_down",
            on_press(lambda: self._interactive_change_drone_speed(-0.25)),
        )
        viewer.subscribe_keyboard_event(
            gymapi.KEY_EQUAL,
            "navrl_drone_speed_up",
            on_press(lambda: self._interactive_change_drone_speed(0.25)),
        )
        viewer.subscribe_keyboard_event(
            gymapi.KEY_G,
            "navrl_toggle_lidar",
            on_press(self._interactive_toggle_lidar),
        )
        viewer.subscribe_keyboard_event(
            gymapi.KEY_M,
            "navrl_toggle_manual",
            on_press(self._interactive_toggle_manual),
        )
        viewer.subscribe_keyboard_event(
            gymapi.KEY_N,
            "navrl_reset",
            on_press(self._interactive_request_reset),
        )
        for key, action in (
            (gymapi.KEY_I, "manual_forward"),
            (gymapi.KEY_K, "manual_back"),
            (gymapi.KEY_J, "manual_left"),
            (gymapi.KEY_L, "manual_right"),
            (gymapi.KEY_U, "manual_yaw_left"),
            (gymapi.KEY_O, "manual_yaw_right"),
        ):
            viewer.subscribe_keyboard_event(
                key,
                "navrl_" + action,
                lambda value, name=action: self._interactive_manual_key(name, value),
            )
        viewer.add_render_callback(self._draw_interactive_overlay)
        if os.environ.get("NAVRL_3D_HUD", "1").strip().lower() not in ("0", "false", "no", "off"):
            try:
                from aerial_gym.apps.navrl_3d_hud import NavRL3DHud, build_hud_lines, build_hud_pip

                self._hud = NavRL3DHud()
            except Exception as exc:
                logger.warning("NavRL 3D HUD unavailable: %s" % exc)
                self._hud = None
        density_help = "" if self.general_eval_mode else "[/] bars±5  "
        logger.warning(
            "NavRL 3D controls | %s,/. target-speed±0.25  -/= drone-speed±0.25\n"
            "                     G LiDAR  N reset  M policy/manual  I/K/J/L move  U/O yaw\n"
            "                     red wireframe=debug target (never given to actor)"
            % density_help
        )

    def _interactive_request_reset(self):
        self._interactive_reset_requested = True
        logger.warning("NavRL 3D | reset requested")

    def _interactive_change_bars(self, delta):
        value = self.set_runtime_bars(self.n_bars_active + int(delta))
        logger.warning("NavRL 3D | bars=%d (reset requested)" % value)

    def _interactive_change_target_speed(self, delta):
        current = (
            float(self._runtime_target_speed)
            if self._runtime_target_speed is not None
            else float(self._tm_speed[0].item())
        )
        value = self.set_runtime_target_speed(current + float(delta))
        logger.warning("NavRL 3D | target speed=%.2f m/s (reset requested)" % value)

    def _interactive_change_drone_speed(self, delta):
        value = self.set_runtime_drone_speed(float(self.task_config.max_velocity) + float(delta))
        logger.warning("NavRL 3D | drone max speed=%.2f m/s" % value)

    def _interactive_toggle_lidar(self):
        self._interactive_show_lidar = not self._interactive_show_lidar
        logger.warning("NavRL 3D | LiDAR overlay=%s" % self._interactive_show_lidar)

    def _interactive_toggle_manual(self):
        self._interactive_manual = not self._interactive_manual
        self._interactive_manual_keys.clear()
        self._interactive_manual_action.zero_()
        logger.warning(
            "NavRL 3D | control=%s" % ("MANUAL" if self._interactive_manual else "POLICY")
        )

    def _interactive_manual_key(self, name, value):
        self._interactive_manual_keys[name] = max(0.0, float(value))
        fwd = self._interactive_manual_keys.get("manual_forward", 0.0)
        back = self._interactive_manual_keys.get("manual_back", 0.0)
        left = self._interactive_manual_keys.get("manual_left", 0.0)
        right = self._interactive_manual_keys.get("manual_right", 0.0)
        yaw_l = self._interactive_manual_keys.get("manual_yaw_left", 0.0)
        yaw_r = self._interactive_manual_keys.get("manual_yaw_right", 0.0)
        self._interactive_manual_action[:, 0] = fwd - back
        self._interactive_manual_action[:, 1] = left - right
        self._interactive_manual_action[:, 2] = 0.0
        self._interactive_manual_action[:, 3] = yaw_l - yaw_r

    def _draw_interactive_overlay(self):
        """Draw debug target/velocity and the selected environment's actual LiDAR scan."""
        viewer_ctl = self.sim_env.IGE_env.viewer
        gym = viewer_ctl.gym
        viewer = viewer_ctl.viewer
        env_id = int(viewer_ctl.current_target_env)
        env_handle = viewer_ctl.env_handles[env_id]
        gym.clear_lines(viewer)

        target = self.target_position[env_id].detach().cpu().numpy().astype(np.float32)
        # The target itself is a 0.3 m virtual drone. Draw a larger human-only marker and a short
        # trajectory trail so motion remains obvious in a 24x24 m overview camera.
        size = np.float32(0.35)
        corners = np.asarray(
            [[target[0] + sx * size, target[1] + sy * size, target[2] + sz * size]
             for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
            dtype=np.float32,
        )
        edges = []
        for a in range(8):
            for bit in (1, 2, 4):
                b = a ^ bit
                if a < b:
                    edges.extend((corners[a], corners[b]))
        velocity_tip = target + self.target_vel_w[env_id].detach().cpu().numpy().astype(np.float32)
        edges.extend((target, velocity_tip))
        target_vertices = np.ascontiguousarray(np.asarray(edges, dtype=np.float32))
        target_colors = np.ascontiguousarray(
            np.vstack(
                [np.tile(np.asarray([[1.0, 0.08, 0.08]], dtype=np.float32), (12, 1)),
                 np.asarray([[0.1, 0.4, 1.0]], dtype=np.float32)]
            )
        )
        gym.add_lines(viewer, env_handle, 13, target_vertices, target_colors)

        if self._interactive_target_trail:
            jump = np.linalg.norm(target - self._interactive_target_trail[-1])
            if jump > 2.0:
                self._interactive_target_trail.clear()
        if not self._interactive_target_trail or np.linalg.norm(
            target - self._interactive_target_trail[-1]
        ) >= 0.03:
            self._interactive_target_trail.append(target.copy())
            self._interactive_target_trail = self._interactive_target_trail[-40:]
        if len(self._interactive_target_trail) >= 2:
            trail = np.asarray(self._interactive_target_trail, dtype=np.float32)
            trail_vertices = np.stack((trail[:-1], trail[1:]), axis=1).reshape(-1, 3)
            alpha = np.linspace(0.25, 1.0, len(trail) - 1, dtype=np.float32)
            trail_colors = np.stack((np.ones_like(alpha), 0.15 + 0.25 * alpha, 0.05 * alpha), axis=1)
            gym.add_lines(
                viewer,
                env_handle,
                len(trail) - 1,
                np.ascontiguousarray(trail_vertices),
                np.ascontiguousarray(trail_colors),
            )

        if self._interactive_show_lidar:
            ranges = self._lidar_distance_m()[env_id]
            # Ray directions must follow the warp generator's ordering: azimuth DECREASES with the
            # bin index (see navrl_perception.lidar_bin_bearings) and elevation DECREASES with the
            # scan line (warp_lidar.py: vfov_max at row 0). The old hard-coded 36-beam increasing
            # tables drew every ray mirrored left-right (and were stale at 72 beams).
            from aerial_gym.task.navrl_task.navrl_perception import VBEAMS, lidar_bin_bearings

            az = lidar_bin_bearings(self.device)
            el = torch.deg2rad(torch.linspace(20.0, -10.0, VBEAMS, device=self.device))
            ee, aa = torch.meshgrid(el, az, indexing="ij")
            dirs = torch.stack(
                (torch.cos(ee) * torch.cos(aa), torch.cos(ee) * torch.sin(aa), torch.sin(ee)),
                dim=-1,
            ).reshape(-1, 3)
            quat = self.obs_dict["robot_vehicle_orientation"][env_id].expand(dirs.shape[0], -1)
            dirs_w = quat_rotate(quat, dirs)
            origin = self.obs_dict["robot_position"][env_id]
            tips = origin.unsqueeze(0) + dirs_w * ranges.unsqueeze(1)
            lidar_vertices = torch.stack(
                (origin.expand_as(tips), tips), dim=1
            ).reshape(-1, 3).detach().cpu().numpy().astype(np.float32)
            hit = (ranges < float(self.task_config.lidar_max_range) * 0.99).detach().cpu().numpy()
            lidar_colors = np.zeros((len(hit), 3), dtype=np.float32)
            lidar_colors[~hit] = (0.10, 0.45, 0.10)
            lidar_colors[hit] = (1.00, 0.55, 0.05)
            gym.add_lines(
                viewer,
                env_handle,
                len(hit),
                np.ascontiguousarray(lidar_vertices),
                np.ascontiguousarray(lidar_colors),
            )

        if self._hud is not None and getattr(self._hud, "enabled", False):
            from aerial_gym.apps.navrl_3d_hud import build_hud_lines, build_hud_pip

            self._hud.update(build_hud_lines(self, env_id), build_hud_pip(self, env_id))

    def close(self):
        if self._hud is not None:
            self._hud.close()
            self._hud = None
        if self.general_eval_mode and not self._general_results_exported:
            self._export_general_results_json()
        self.sim_env.delete_env()

    # ------------------------------------------------------------------ checkpoint state
    def get_env_state(self):
        """Saved into the rl_games checkpoint ('env_state') so the epoch-proportional goal
        curriculum and optional density curriculum survive a --checkpoint resume."""
        representation = self._obstacle_representation_or_zero()
        return {
            "num_task_steps": int(self.num_task_steps),
            "n_bars_active": int(self.n_bars_active),
            "k_max_cur": float(self._k_max_cur),
            "k_min_cur": float(self._k_min_cur),
            # Scaling-critical settings, recorded so set_env_state() can warn when a checkpoint is
            # replayed under a different config. These are NOT restored (an eval may legitimately
            # override them) -- they exist purely to make a silent mismatch loud. lidar_max_range in
            # particular is BOTH the sensor horizon and the observation divisor (scan/range), so
            # evaluating an 8 m policy at 4 m rescales every scan input and the policy misreads the
            # world entirely -- a failure that looks like "the policy is broken", not like a config
            # error. Learned the hard way twice in one session.
            "cfg_lidar_max_range": float(self.task_config.lidar_max_range),
            "cfg_max_velocity": float(self.task_config.max_velocity),
            "cfg_yaw_rate_max": float(self.task_config.yaw_rate_max),
            "cfg_max_obstacles": int(representation["max_obstacles"]),
            "cfg_token_fov_deg": float(representation["token_fov_deg"]),
            "cfg_obstacle_suppress_deg": float(representation["suppress_deg"]),
            "cfg_lidar_hbeams": int(representation["hbeams"]),
            "cfg_lidar_vbeams": int(representation["vbeams"]),
            # Action-distribution provenance. Bounded and legacy models intentionally share the
            # same state_dict keys, so without this an eval can load successfully under the wrong
            # likelihood and silently measure a different policy.
            "cfg_action_policy": os.environ.get("NAVRL_ACTION_POLICY", "legacy"),
            "cfg_action_std": os.environ.get("NAVRL_ACTION_STD", ""),
            "cfg_action_mu_scale": os.environ.get("NAVRL_ACTION_MU_SCALE", "1"),
            "cfg_action_entropy_coef": float(
                os.environ.get("NAVRL_ENTROPY_COEF", "0") or 0.0
            ),
            "cfg_action_learning_rate": float(
                os.environ.get("NAVRL_LEARNING_RATE", "0") or 0.0
            ),
            "cfg_ppo_log_ratio_clamp": float(
                os.environ.get("NAVRL_PPO_LOG_RATIO_CLAMP", "0") or 0.0
            ),
            "cfg_ppo_kl_stop": float(
                os.environ.get("NAVRL_PPO_KL_STOP", "0") or 0.0
            ),
            "cfg_lateral_latent_margin_y": float(
                os.environ.get("NAVRL_LATENT_MARGIN_Y", "0") or 0.0
            ),
            "cfg_lateral_latent_margin_coef": float(
                os.environ.get("NAVRL_LATENT_MARGIN_COEF", "0") or 0.0
            ),
            "cfg_lateral_bias_coef": float(
                os.environ.get("NAVRL_LATERAL_BIAS_COEF", "0") or 0.0
            ),
            "cfg_reflection_coef": float(
                os.environ.get("NAVRL_REFLECTION_COEF", "0") or 0.0
            ),
            "cfg_truncated_dmin": float(
                os.environ.get("NAVRL_TRUNCATED_DMIN", "0.01") or 0.01
            ),
            # Preserve the in-progress competence window. With a 16k-episode density gate, dropping
            # these counters on every periodic-checkpoint resume can discard hours of evidence and
            # indefinitely postpone the next promotion.
            "density_succ_agg": int(self._density_succ_agg),
            "density_fin_agg": int(self._density_fin_agg),
            "cfg_density_final": int(getattr(self.density, "n_final", self.n_bars_active)),
            "cfg_density_step": int(getattr(self.density, "promote_step", 0)),
            "cfg_density_threshold": float(
                getattr(self.density, "success_threshold", 0.0)
            ),
            "cfg_density_check_eps": int(
                getattr(self.density, "check_after_episodes", 0)
            ),
        }

    @staticmethod
    def _obstacle_representation_or_zero():
        """Policy obstacle-representation settings, or zeros when perception is unavailable."""
        try:
            from aerial_gym.task.navrl_task.navrl_perception import (
                HBEAMS,
                MAX_OBSTACLES,
                OBSTACLE_FOV_DEG,
                OBSTACLE_SUPPRESS_DEG,
                VBEAMS,
            )

            return {
                "max_obstacles": int(MAX_OBSTACLES),
                "token_fov_deg": float(OBSTACLE_FOV_DEG),
                "suppress_deg": float(OBSTACLE_SUPPRESS_DEG),
                "hbeams": int(HBEAMS),
                "vbeams": int(VBEAMS),
            }
        except Exception:
            return {
                "max_obstacles": 0,
                "token_fov_deg": 0.0,
                "suppress_deg": 0.0,
                "hbeams": 0,
                "vbeams": 0,
            }

    @classmethod
    def _token_fov_or_zero(cls):
        return float(cls._obstacle_representation_or_zero()["token_fov_deg"])

    def _record_bar_contact_probe(self, hit_mask, pos):
        """Measure which current obstacle tokens geometrically represent a struck bar.

        Evaluation-only (NAVRL_BAR_PROBE=1). Uses ground-truth bar positions, which is legitimate
        here because nothing computed in this method reaches the actor, the critic, the reward, or
        any termination. Probe v2 separates FOV eligibility, range+bearing association, lateral ray
        offset, radial surface gap, and duplicate token use instead of collapsing them into the old
        ambiguous ``token_err``.
        """
        from aerial_gym.task.navrl_task.bar_probe import associate_surface_tokens_to_bars
        from aerial_gym.task.navrl_task.navrl_perception import (
            MAX_OBSTACLES,
            OBSTACLE_DIM,
            OBSTACLE_FOV_DEG,
        )

        idx = hit_mask.nonzero(as_tuple=False).squeeze(1)
        if idx.numel() == 0:
            return
        rng = float(self.task_config.lidar_max_range)

        # True bars, expressed in the drone's vehicle frame (yaw-only, matching the LiDAR frame).
        bars_w = self.obs_dict["obstacle_position"][idx][:, : self.n_bars_active, 0:3]  # (K, B, 3)
        rel_w = bars_w - pos[idx].unsqueeze(1)
        quat = self.obs_dict["robot_vehicle_orientation"][idx]  # (K, 4)
        k, b = rel_w.shape[0], rel_w.shape[1]
        rel_v = quat_rotate_inverse(
            quat.unsqueeze(1).expand(k, b, 4).reshape(k * b, 4), rel_w.reshape(k * b, 3)
        ).reshape(k, b, 3)
        bar_dist = rel_v[:, :, 0:2].norm(dim=2)  # (K, B) horizontal distance to each bar
        bar_bearing = torch.atan2(rel_v[:, :, 1], rel_v[:, :, 0])

        # The bar that was struck = the closest one at the moment of contact.
        hit_d, hit_i = bar_dist.min(dim=1)
        rows = torch.arange(k, device=self.device)

        # Crowding, two independent ways: GT bars inside the range/FOV and scan bearings returning
        # something.  The FOV split is essential: a 240-deg token sector cannot represent a struck
        # bar in the excluded rear 120 deg, although the full static scan still observes it.
        bars_in_range = (bar_dist < rng).sum(dim=1).float()
        if OBSTACLE_FOV_DEG < 359.9:
            in_token_fov = bar_bearing.abs() <= math.radians(OBSTACLE_FOV_DEG * 0.5)
        else:
            in_token_fov = torch.ones_like(bar_bearing, dtype=torch.bool)
        bars_in_token_fov = ((bar_dist < rng) & in_token_fov).sum(dim=1).float()
        hit_in_token_fov = in_token_fov[rows, hit_i]
        scan = getattr(self.perception, "last_scan_nearest", None)
        occupied = (
            (scan[idx] < rng * 0.995).sum(dim=1).float()
            if scan is not None
            else torch.zeros_like(bars_in_range)
        )

        # Associate each LiDAR surface token to a plausible GT bar using BOTH bearing and range.
        # The old +/-15-deg bearing-only matcher routinely attached a nearer token to a farther bar
        # in dense scenes and then mislabeled surface-to-center distance as token position error.
        tokens = self.perception.obstacle_history[idx, -1].view(k, MAX_OBSTACLES, OBSTACLE_DIM)
        tok_pos = tokens[:, :, 0:2] * rng  # positions are stored normalized by the LiDAR range
        tok_valid = tokens[:, :, 11] > 0.5
        association = associate_surface_tokens_to_bars(tok_pos, tok_valid, rel_v[:, :, 0:2])
        token_bar = association["bar_index"]
        token_associated = association["associated"]
        hit_token = token_associated & (token_bar == hit_i.unsqueeze(1))
        hit_offsets = torch.where(
            hit_token,
            association["center_offset"],
            torch.full_like(association["center_offset"], float("inf")),
        )
        best_offset, best_slot = hit_offsets.min(dim=1)
        matched = torch.isfinite(best_offset)

        represented = (
            torch.nn.functional.one_hot(token_bar, num_classes=b).bool()
            & token_associated.unsqueeze(2)
        ).any(dim=1)
        associated_count = token_associated.sum(dim=1).float()
        unique_count = represented.sum(dim=1).float()

        self._bprobe["n"] += float(k)
        self._bprobe["bars_in_range"] += float(bars_in_range.sum().item())
        self._bprobe["bars_in_token_fov"] += float(bars_in_token_fov.sum().item())
        self._bprobe["occupied_bins"] += float(occupied.sum().item())
        self._bprobe["hit_dist"] += float(hit_d.sum().item())
        self._bprobe["hit_in_token_fov"] += float(hit_in_token_fov.sum().item())
        self._bprobe["hit_in_tokens"] += float(matched.sum().item())
        self._bprobe["hit_in_tokens_in_fov"] += float(
            (matched & hit_in_token_fov).sum().item()
        )
        self._bprobe["valid_tokens"] += float(tok_valid.sum().item())
        self._bprobe["associated_tokens"] += float(associated_count.sum().item())
        self._bprobe["unique_token_bars"] += float(unique_count.sum().item())
        self._bprobe["duplicate_tokens"] += float((associated_count - unique_count).sum().item())
        if bool(matched.any()):
            m = matched.nonzero(as_tuple=False).squeeze(1)
            slot = best_slot[m]
            self._bprobe["hit_center_offset"] += float(best_offset[m].sum().item())
            self._bprobe["hit_cross_track"] += float(
                association["cross_track"][m, slot].sum().item()
            )
            self._bprobe["hit_radial_gap"] += float(
                association["radial_gap"][m, slot].sum().item()
            )
            self._bprobe["hit_token_rank"] += float(best_slot[m].float().sum().item())

    @staticmethod
    def _max_obstacles_or_zero():
        """Obstacle-token capacity, or 0 when perception is off (import stays lazy on purpose)."""
        return int(NavRLTask._obstacle_representation_or_zero()["max_obstacles"])

    def set_env_state(self, state):
        bars_before_restore = int(self.n_bars_active)
        if isinstance(state, dict):
            representation = self._obstacle_representation_or_zero()
            saved_action_policy = state.get("cfg_action_policy")
            current_action_policy = os.environ.get("NAVRL_ACTION_POLICY", "legacy")
            if (
                saved_action_policy is not None
                and str(saved_action_policy).strip() != str(current_action_policy).strip()
            ):
                logger.warning(
                    "NavRL ACTION POLICY MISMATCH | checkpoint=%s running=%s. "
                    "The state_dict is shape-compatible but the action likelihood is not."
                    % (saved_action_policy, current_action_policy)
                )
            saved_action_std = str(state.get("cfg_action_std", "")).strip()
            current_action_std = os.environ.get("NAVRL_ACTION_STD", "").strip()
            if (
                saved_action_std
                and current_action_std
                and saved_action_std != current_action_std
            ):
                logger.warning(
                    "NavRL ACTION STD MISMATCH | checkpoint=%s running=%s."
                    % (saved_action_std, current_action_std)
                )
            saved_mu_scale = str(state.get("cfg_action_mu_scale", "")).strip()
            current_mu_scale = os.environ.get("NAVRL_ACTION_MU_SCALE", "1").strip()
            if saved_mu_scale and saved_mu_scale != current_mu_scale:
                logger.warning(
                    "NavRL ACTION MU-SCALE MISMATCH | checkpoint=%s running=%s."
                    % (saved_mu_scale, current_mu_scale)
                )
            # Loud config-drift guard (warn, never override: an eval may deliberately change these).
            # A mismatch here silently invalidates the run -- see get_env_state() for why.
            for key, current, name in (
                ("cfg_lidar_max_range", float(self.task_config.lidar_max_range), "NAVRL_LIDAR_RANGE"),
                ("cfg_max_velocity", float(self.task_config.max_velocity), "NAVRL_MAX_VELOCITY"),
                ("cfg_yaw_rate_max", float(self.task_config.yaw_rate_max), "NAVRL_YAW_RATE_MAX"),
                (
                    "cfg_max_obstacles",
                    float(representation["max_obstacles"]),
                    "MAX_OBSTACLES (navrl_perception.py)",
                ),
                (
                    "cfg_token_fov_deg",
                    float(representation["token_fov_deg"]),
                    "NAVRL_OBSTACLE_FOV_DEG",
                ),
                (
                    "cfg_obstacle_suppress_deg",
                    float(representation["suppress_deg"]),
                    "NAVRL_OBSTACLE_SUPPRESS_DEG",
                ),
                (
                    "cfg_lidar_hbeams",
                    float(representation["hbeams"]),
                    "NAVRL_LIDAR_HBEAMS",
                ),
                (
                    "cfg_lidar_vbeams",
                    float(representation["vbeams"]),
                    "NAVRL_LIDAR_VBEAMS",
                ),
            ):
                saved = state.get(key)
                if saved is None:
                    continue  # checkpoint predates this guard
                if abs(float(saved) - current) > 1e-6:
                    logger.warning(
                        "NavRL CONFIG MISMATCH | %s: checkpoint trained with %.3f, running with %.3f. "
                        "This changes policy inputs -- results are NOT comparable unless intentional."
                        % (name, float(saved), current)
                    )
            if bool(getattr(self.density, "use_density_curriculum", False)):
                for key, current, name in (
                    (
                        "cfg_density_final",
                        float(getattr(self.density, "n_final", self.n_bars_active)),
                        "NAVRL_DENSITY_FINAL",
                    ),
                    (
                        "cfg_density_step",
                        float(getattr(self.density, "promote_step", 0)),
                        "NAVRL_DENSITY_STEP",
                    ),
                    (
                        "cfg_density_threshold",
                        float(getattr(self.density, "success_threshold", 0.0)),
                        "NAVRL_DENSITY_THRESHOLD",
                    ),
                    (
                        "cfg_density_check_eps",
                        float(getattr(self.density, "check_after_episodes", 0)),
                        "NAVRL_DENSITY_CHECK_EPS",
                    ),
                ):
                    saved = state.get(key)
                    if saved is not None and abs(float(saved) - current) > 1e-6:
                        logger.warning(
                            "NavRL CURRICULUM CONFIG MISMATCH | %s: checkpoint used %.3f, "
                            "running with %.3f. The promotion schedule is changing intentionally "
                            "or the runs are not directly comparable."
                            % (name, float(saved), current)
                        )
        if isinstance(state, dict) and state.get("num_task_steps") is not None:
            self.num_task_steps = int(state["num_task_steps"])
        if isinstance(state, dict) and state.get("k_max_cur") is not None:
            # restore the competence-gated distance window across a --checkpoint resume
            self._k_max_cur = float(state["k_max_cur"])
            self._k_min_cur = float(state.get("k_min_cur", self._k_min_cur))
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
        if isinstance(state, dict):
            restored_fin = max(0, int(state.get("density_fin_agg", 0)))
            restored_succ = max(0, int(state.get("density_succ_agg", 0)))
            self._density_fin_agg = restored_fin
            self._density_succ_agg = min(restored_succ, restored_fin)
            logger.warning(
                "NavRL checkpoint state restored | task_steps=%d bars=%d->%d "
                "k_min=%.1f k_max=%.1f density_window=%d/%d eps"
                % (
                    int(self.num_task_steps),
                    bars_before_restore,
                    int(self.n_bars_active),
                    float(self._k_min_cur),
                    float(self._k_max_cur),
                    int(self._density_fin_agg),
                    int(getattr(self.density, "check_after_episodes", 0)),
                )
            )

    # ------------------------------------------------------------------ reset
    def reset(self):
        # Respawn the robots (and re-place obstacles) BEFORE sampling goals. Without this, a
        # full reset leaves the robots at their build pose (overlapping the bars near the env
        # origin), so the first step crashes every env at once — which ends rl_games play mode
        # after a single step. Mid-episode resets don't need it: the env manager has already
        # respawned those envs by the time reset_idx() is called from step().
        if self.general_eval_mode:
            self._sample_general_density()
        self.sim_env.reset()
        self.reset_idx(torch.arange(self.num_envs, device=self.device))
        self._sync_target_to_sensor()
        # render once so the first observation carries a valid LiDAR scan
        self.sim_env.render(render_components="sensors")
        return self.get_return_tuple()

    def reset_idx(self, env_ids):
        if len(env_ids) == 0:
            return
        if self.general_spawn_mode:
            self._randomize_general_drone_spawn(env_ids)
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
        if self.general_spawn_mode:
            goal = self._sample_general_target(env_ids, start_pos, b_min, b_max, bars_xy)
            sampled_dist = torch.norm(goal[:, 0:2] - start_pos[:, 0:2], dim=1)
            k_min = float(sampled_dist.min().item())
            k_max = float(sampled_dist.max().item())
        else:
            k_max = self._goal_x_max()
            k_min = self._goal_x_min()
        self.cur_goal_dist_max = k_max  # surfaced to the dashboard as "curriculum max"
        self.cur_goal_dist_min = k_min  # surfaced to the dashboard as "curriculum min"
        todo = torch.zeros(n, dtype=torch.bool, device=self.device) if self.general_spawn_mode else torch.ones(
            n, dtype=torch.bool, device=self.device
        )
        for _ in range(10):
            if not todo.any():
                break
            j = int(todo.sum())
            gx = k_min + (k_max - k_min) * torch.rand(j, device=self.device)
            # Density is a different source of difficulty from distance. During the density stage,
            # retain some short/medium episodes instead of training exclusively on the final far
            # window. This is enabled only by the staged launch recipe; default behavior is intact.
            mix_prob = float(getattr(self.density, "easy_goal_mix_prob", 0.0))
            horizon = max(1, int(getattr(self.cur, "ppo_horizon", 1)))
            density_start = int(getattr(self.density, "warmup_epochs", 0)) * horizon
            if mix_prob > 0.0 and self.num_task_steps >= density_start:
                easy = torch.rand(j, device=self.device) < min(1.0, max(0.0, mix_prob))
                easy_lo = float(getattr(self.density, "easy_goal_min", self.cur.k_min))
                easy_hi = min(
                    float(getattr(self.density, "easy_goal_max", k_max)), float(k_max)
                )
                if easy_hi > easy_lo and bool(easy.any()):
                    gx[easy] = easy_lo + (easy_hi - easy_lo) * torch.rand(
                        int(easy.sum()), device=self.device
                    )
            gx = gx.clamp(max=(b_max[todo, 0] - m))  # keep the capture sphere off the far wall
            gy = (b_min[todo, 1] + m) + (
                b_max[todo, 1] - b_min[todo, 1] - 2.0 * m
            ) * torch.rand(j, device=self.device)
            if self.vision_mode and float(getattr(self.vis_cfg, "fov_curriculum_epochs", 0.0)) > 0.0:
                # Cold-start visibility: keep the goal inside the camera FOV early so the detector
                # acquires the target (the KF activates -> the actor finally gets a bearing to act
                # on), then widen the allowed bearing to the full arena. The +/- spawn-yaw headroom
                # keeps the target visible regardless of the spawn heading. Without this a sensor-
                # only from-scratch policy is goal-blind and never leaves the ~100% crash basin.
                horizon_e = max(1, int(getattr(self.cur, "ppo_horizon", 1)))
                frac = min(1.0, (self.num_task_steps / horizon_e) / float(self.vis_cfg.fov_curriculum_epochs))
                half_fov = math.radians(float(self.vis_cfg.detector_hfov_deg) * 0.5)
                yaw_head = math.radians(float(getattr(self.vis_cfg, "spawn_yaw_max_deg", 30.0)))
                bearing0 = max(math.radians(8.0), 0.85 * (half_fov - yaw_head))
                bearing_lim = bearing0 + frac * (0.5 * math.pi - bearing0)
                dy_max = (gx - start_pos[todo, 0]).clamp(min=0.5) * math.tan(bearing_lim)
                sy = start_pos[todo, 1]
                lo = torch.maximum(b_min[todo, 1] + m, sy - dy_max)
                hi = torch.maximum(torch.minimum(b_max[todo, 1] - m, sy + dy_max), lo)
                gy = lo + (hi - lo) * torch.rand(j, device=self.device)
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
        if self._oob_probe:
            self._probe_ep_start_y[env_ids] = start_pos[:, 1]
            self._probe_ep_target_start_y[env_ids] = goal[:, 1]
            if self.n_bars_active > 0:
                self._probe_ep_bar_mean_y[env_ids] = bars_xy[
                    :, : self.n_bars_active, 1
                ].mean(dim=1)
            else:
                self._probe_ep_bar_mean_y[env_ids] = 0.5 * (
                    b_min[:, 1] + b_max[:, 1]
                )
            self._probe_ep_y_min[env_ids] = start_pos[:, 1]
            self._probe_ep_y_max[env_ids] = start_pos[:, 1]

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
        self._z_err_integral[env_ids] = 0.0  # fresh episode -> no carried-over altitude-hold bias
        # Seed the PBRS/segment-capture buffers with the spawn state. First-step progress is then
        # ||start - target|| - gamma*||pos - target||, identical to the old prev_dist seeding.
        self.prev_pos[env_ids] = start_pos
        self.prev_rel[env_ids] = start_pos - self.target_position[env_ids]
        # vision mode: fresh episode -> tracker knows nothing, no previous action yet
        if self.vision_mode:
            self.detector.reset_idx(env_ids)
            if self.perception is not None:
                self.perception.reset_idx(env_ids)
            self.prev_action[env_ids] = 0.0
            self._visible_now[env_ids] = False
        if self._action_diag_enabled:
            self._action_diag_prev_valid[env_ids] = False
        # Phase 3: per-episode target speed + trajectory pattern (all-static when the speed
        # ceiling is 0 -> Phases 1-2 behavior).
        self._sample_target_motion(env_ids)
        self.ep_min_goal_dist[env_ids] = float("inf")
        self.ep_reached[env_ids] = False

    def render(self):
        return self.sim_env.render()

    # ------------------------------------------------------------------ step
    def transform_action_to_command(self, actions):
        """Action -> vehicle-frame velocity command for the controller.

        Default: NavRL 3D goal-frame velocity (the goal frame is defined by the KNOWN start->goal
        direction). Vision mode: the actor does not know the target position, so a goal frame is
        undefinable from its observations — actions are the vehicle-frame velocity directly
        (fly toward what the sensors show; the frame matches the body-frame scan and detector)."""
        if self.vision_mode:
            vel_vehicle = torch.clamp(actions[:, 0:3], -1.0, 1.0) * self.task_config.max_velocity
        else:
            vel_goal = torch.clamp(actions[:, 0:3], -1.0, 1.0) * self.task_config.max_velocity
            vel_world = goal_frame_to_world(vel_goal, self.target_dir_2d)
            vel_vehicle = quat_rotate_inverse(self.obs_dict["robot_vehicle_orientation"], vel_world)
        self.command[:, 0:3] = vel_vehicle
        # 2D flight: hold altitude. The vehicle frame is yaw-only (level), so vehicle-z == world-z.
        # A plain vz=0 command is OPEN-LOOP: the velocity controller carries no z-position feedback
        # (setpoint_position tracks the current position), so altitude bled away during aggressive
        # lateral/yaw maneuvers — measured with NAVRL_CRASH_DIAG on the first perception run, 39%
        # of all crashes were floor strikes (z < 0.1 m after ~4.5 s of flight). Close the loop with
        # a proportional altitude-hold velocity command instead (policy-independent stabilization;
        # the action space is unchanged — the actor still cannot command vertical motion).
        z_err = self.task_config.flight_altitude - self.obs_dict["robot_position"][:, 2]
        # Symmetric vertical authority. The prior +/-1 m/s hold lost to the +/-2 m/s tilt-induced
        # altitude sag during sustained lateral+yaw weaving; once the 8 m LiDAR horizon let episodes
        # survive in open space long enough (bar contacts 76% -> 14%), that latent bleed surfaced as
        # floor strikes (below 0% -> 71%). NOTE: there is NO floor mesh to hit (create_ground_plane
        # =False; the warp LiDAR raycasts only bar meshes), so this is a control-authority fix, not a
        # perception one. Match the lateral command's gain and authority so vertical recovery keeps up.
        # PI, not just P: sustained lateral+yaw weaving holds the vehicle tilted for multi-step
        # bursts, and during that tilt the attitude-tracking transient (desired vs actual body-z
        # axis) biases the achieved vertical acceleration low even though thrust magnitude has
        # plenty of headroom (T/W ~= 3.3) -- a proportional term alone settles to a nonzero
        # steady-state z_err under a persistent bias. The integral term removes that steady-state
        # sag; anti-windup clamp keeps it from overshooting once the bias clears (e.g. after a
        # crash-avoidance turn ends). Reset per-episode in reset_idx.
        # Vertical authority is alt_hold_vmax, NOT max_velocity: tying it to the horizontal speed
        # limit made every pursuer-speed sweep confound "slower pursuer crashes less" with "slower
        # pursuer has proportionally weaker altitude hold" (at 0.75 m/s it kept only ~30% of its
        # authority AND a 3x tighter anti-windup bound).
        _mv = float(getattr(self.task_config, "alt_hold_vmax", self.task_config.max_velocity))
        self._z_err_integral += z_err * self.step_dt
        _ki = 1.0
        _i_bound = _mv / _ki
        self._z_err_integral.clamp_(-_i_bound, _i_bound)
        self.command[:, 2] = torch.clamp(4.0 * z_err + _ki * self._z_err_integral, -_mv, _mv)
        # (b) learned yaw-rate: action[:, 3] in [-1, 1] -> euler yaw-rate (was held at 0). yaw_rate_max
        # matches the NavRL-scoped controller clamp (2.5 rad/s) so the mapping is linear (no dead band).
        self._yaw_cmd[:] = torch.clamp(actions[:, 3], -1.0, 1.0)
        self.command[:, 3] = self._yaw_cmd * self.task_config.yaw_rate_max
        return self.command

    def step(self, actions):
        if self.interactive_mode and self._interactive_manual:
            actions = self._interactive_manual_action
        self._record_action_diagnostics(actions)
        # Phase 3: move the virtual target FIRST — both agents move during this 0.1 s control
        # interval, and the end-of-interval reward is computed against the target's NEW position.
        # (No-op while all per-episode target speeds are 0, i.e. the static Phases 1-2 task.)
        self._advance_target()
        command = self.transform_action_to_command(actions)
        if self.vision_mode:
            # remembered as "previous action" in the NEXT observation (ego proprioception)
            self.prev_action[:] = torch.clamp(actions[:, 0:4], -1.0, 1.0)
        self.sim_env.step(actions=command)

        # state-based reward + termination (LiDAR-based safety reward is added after rendering)
        self.compute_state_reward_and_terminations()

        self.truncations[:] = torch.where(
            self.sim_env.sim_steps > self.task_config.episode_len_steps,
            torch.ones_like(self.truncations),
            torch.zeros_like(self.truncations),
        )
        if self._interactive_reset_requested:
            self.truncations[:] = 1

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
        self._record_general_result(successes, crashes, timeouts, finished)
        self._log_progress(successes, crashes, timeouts, finished)
        self._update_curriculum(successes, finished)
        self._record_epoch_dashboard(successes, crashes, timeouts, finished)

        # The LiDAR reads this buffer inside its captured render graph. Keep it synchronized with
        # the same moving target that the camera renderer observes.
        self._sync_target_to_sensor()

        # render (raycast LiDAR from the new state) and reset finished envs
        reset_envs = self.sim_env.post_reward_calculation_step()
        if len(reset_envs) > 0:
            self.reset_idx(reset_envs)
            # The env manager rendered once before the task sampled its new generalized drone and
            # target poses. Refresh both target injection and LiDAR so the first policy observation
            # of the next trial matches the newly randomized scene.
            self._sync_target_to_sensor()
            self.sim_env.render(render_components="sensors")
        self._interactive_reset_requested = False

        # LiDAR-based static-safety reward, using the freshly rendered scan
        self.add_static_safety_reward()

        self.num_task_steps += 1
        return self.get_return_tuple()

    @staticmethod
    def _empty_action_diag():
        return {
            "n": 0,
            "raw_oob": [0.0, 0.0, 0.0, 0.0],
            "exec_edge": [0.0, 0.0, 0.0, 0.0],
            "exec_edge95": [0.0, 0.0, 0.0, 0.0],
            "exec_edge99": [0.0, 0.0, 0.0, 0.0],
            "abs_sum": [0.0, 0.0, 0.0, 0.0],
            "signed_y_sum": 0.0,
            "positive_y": 0.0,
            "negative_y": 0.0,
            "high80_y": 0.0,
            "delta_y_sum": 0.0,
            "delta_y_n": 0,
            "sign_flip_y": 0.0,
            "front_clear_n": 0.0,
            "front_clear_abs_y": 0.0,
            "front_blocked_n": 0.0,
            "front_blocked_abs_y": 0.0,
            "goal_centered_n": 0.0,
            "goal_centered_abs_y": 0.0,
            "goal_offcenter_n": 0.0,
            "goal_offcenter_abs_y": 0.0,
            "clear_centered_n": 0.0,
            "clear_centered_abs_y": 0.0,
            "target_visible_n": 0.0,
            "target_visible_abs_y": 0.0,
            "target_hidden_n": 0.0,
            "target_hidden_abs_y": 0.0,
        }

    def _record_action_diagnostics(self, actions):
        """Accumulate action tails plus context needed to separate avoidance from policy bias."""
        if not self._action_diag_enabled:
            return
        with torch.no_grad():
            action = actions[:, :4].detach()
            if action.shape[1] != 4:
                return
            finite = torch.isfinite(action)
            safe = torch.where(finite, action, torch.zeros_like(action))
            self._action_diag["n"] += int(action.shape[0])
            raw_oob = (safe.abs() > 1.0) & finite
            executed = safe.clamp(-1.0, 1.0)
            exec_edge = (executed.abs() >= 0.98) & finite
            exec_edge95 = (executed.abs() >= 0.95) & finite
            exec_edge99 = (executed.abs() >= 0.99) & finite
            abs_sum = safe.abs() * finite
            for axis in range(4):
                self._action_diag["raw_oob"][axis] += float(
                    raw_oob[:, axis].sum().item()
                )
                self._action_diag["exec_edge"][axis] += float(
                    exec_edge[:, axis].sum().item()
                )
                self._action_diag["exec_edge95"][axis] += float(
                    exec_edge95[:, axis].sum().item()
                )
                self._action_diag["exec_edge99"][axis] += float(
                    exec_edge99[:, axis].sum().item()
                )
                self._action_diag["abs_sum"][axis] += float(
                    abs_sum[:, axis].sum().item()
                )

            valid_y_now = finite[:, 1]
            ay = safe[:, 1]
            abs_y = ay.abs()
            self._action_diag["signed_y_sum"] += float(ay[valid_y_now].sum().item())
            self._action_diag["positive_y"] += float(
                ((ay > 0.1) & valid_y_now).sum().item()
            )
            self._action_diag["negative_y"] += float(
                ((ay < -0.1) & valid_y_now).sum().item()
            )
            self._action_diag["high80_y"] += float(
                ((abs_y >= 0.8) & valid_y_now).sum().item()
            )

            # Diagnostics only: classify the command using the same LiDAR frame as the actor.
            # Target returns are excluded so chasing a centered target is not mislabeled as an
            # obstacle. "Blocked" means a static return within 4 m in the forward +/-30 degree
            # sector. This never enters observations, rewards, terminations or commands.
            depth = self.obs_dict.get("depth_range_pixels")
            if isinstance(depth, torch.Tensor) and depth.ndim >= 4:
                scan = torch.nan_to_num(
                    depth.squeeze(1), nan=1.0, posinf=1.0, neginf=1.0
                ).clamp(0.0, 1.0)
                hbeams = int(scan.shape[-1])
                if (
                    self._action_front_mask is None
                    or int(self._action_front_mask.numel()) != hbeams
                ):
                    from aerial_gym.task.navrl_task.navrl_perception import (
                        HBEAMS as _HB,
                        lidar_bin_bearings,
                    )

                    if hbeams == _HB:
                        angles = torch.rad2deg(lidar_bin_bearings(self.device))
                    else:  # non-perception scan shape: derive locally, same DECREASING convention
                        bin_deg = 360.0 / max(1, hbeams)
                        angles = torch.linspace(
                            180.0, -180.0 + bin_deg, hbeams, device=self.device
                        )
                    self._action_front_mask = angles.abs() <= 30.0
                segmentation = self.obs_dict.get("segmentation_pixels")
                if isinstance(segmentation, torch.Tensor):
                    target_return = segmentation.squeeze(1) == 50
                    if target_return.shape == scan.shape:
                        scan = torch.where(target_return, torch.ones_like(scan), scan)
                front_min = scan[:, :, self._action_front_mask].amin(dim=(1, 2))
                blocked_threshold = min(
                    0.999,
                    4.0 / max(1e-6, float(self.task_config.lidar_max_range)),
                )
                front_blocked = (front_min < blocked_threshold) & valid_y_now
                front_clear = ~front_blocked & valid_y_now
                for name, mask in (
                    ("front_clear", front_clear),
                    ("front_blocked", front_blocked),
                ):
                    self._action_diag[name + "_n"] += float(mask.sum().item())
                    self._action_diag[name + "_abs_y"] += float(abs_y[mask].sum().item())
            else:
                front_clear = torch.zeros_like(valid_y_now)

            # Ground truth is used only to label this diagnostic. The future v3 gate is derived
            # from the actor's structured target track instead; no oracle feature is introduced.
            rpos = self.target_position - self.obs_dict["robot_position"]
            goal_vehicle = quat_rotate_inverse(
                self.obs_dict["robot_vehicle_orientation"], rpos
            )
            lateral_sine = goal_vehicle[:, 1].abs() / goal_vehicle[:, :2].norm(
                dim=1
            ).clamp(min=1e-6)
            goal_centered = (lateral_sine <= math.sin(math.radians(15.0))) & valid_y_now
            goal_offcenter = ~goal_centered & valid_y_now
            for name, mask in (
                ("goal_centered", goal_centered),
                ("goal_offcenter", goal_offcenter),
            ):
                self._action_diag[name + "_n"] += float(mask.sum().item())
                self._action_diag[name + "_abs_y"] += float(abs_y[mask].sum().item())
            clear_centered = front_clear & goal_centered
            self._action_diag["clear_centered_n"] += float(clear_centered.sum().item())
            self._action_diag["clear_centered_abs_y"] += float(
                abs_y[clear_centered].sum().item()
            )
            visible = self._visible_now & valid_y_now
            hidden = ~self._visible_now & valid_y_now
            for name, mask in (("target_visible", visible), ("target_hidden", hidden)):
                self._action_diag[name + "_n"] += float(mask.sum().item())
                self._action_diag[name + "_abs_y"] += float(abs_y[mask].sum().item())

            valid = self._action_diag_prev_valid & finite[:, 1]
            if bool(valid.any()):
                current_y = safe[valid, 1]
                previous_y = self._action_diag_prev[valid, 1]
                self._action_diag["delta_y_sum"] += float(
                    (current_y - previous_y).abs().sum().item()
                )
                self._action_diag["delta_y_n"] += int(valid.sum().item())
                sign_flip = (
                    (current_y * previous_y < 0.0)
                    & (current_y.abs() > 0.1)
                    & (previous_y.abs() > 0.1)
                )
                self._action_diag["sign_flip_y"] += float(sign_flip.sum().item())

            self._action_diag_prev[:] = safe
            self._action_diag_prev_valid[:] = finite.all(dim=1)

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

        # -- vision mode add-ons (before the terminal overwrites, like the other shaping terms)
        if self.vision_mode:
            # The camera is rendered once while building the observation.  Use its most recent
            # pixel-derived visibility here (one control interval old) instead of secretly
            # querying GT geometry a second time for reward shaping.
            self.rewards[:] = self.rewards + float(
                self.vis_cfg.visibility_bonus
            ) * self._visible_now.float()

        crashed = self.obs_dict["crashes"] > 0
        below = z < self.task_config.lower_height_bound
        above = z > self.task_config.upper_height_bound
        crashed_out = crashed | below | above
        if self.vision_mode:
            # out-of-arena termination: with the target unobserved there is no implicit goal
            # attraction keeping the drone in-bounds, and there are no physical walls to crash on.
            m_oob = float(self.vis_cfg.oob_margin)
            b_min = self.obs_dict["env_bounds_min"][:, 0:2]
            b_max = self.obs_dict["env_bounds_max"][:, 0:2]
            oob = ((pos[:, 0:2] < b_min - m_oob) | (pos[:, 0:2] > b_max + m_oob)).any(dim=1)
            crashed_out = crashed_out | oob
            if self._oob_probe:
                self._probe_ep_y_min = torch.minimum(self._probe_ep_y_min, pos[:, 1])
                self._probe_ep_y_max = torch.maximum(self._probe_ep_y_max, pos[:, 1])

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

        if self._crash_diag:
            # Attribute each crash to ONE cause, priority contact > below > above > oob (matching
            # the sources OR-ed into crashed_out above). Same-step captures already excluded.
            d_contact = crashed & crashed_out
            d_below = below & ~crashed & crashed_out
            d_above = above & ~crashed & ~below & crashed_out
            self._diag["contact"] += int(d_contact.sum().item())
            self._diag["below"] += int(d_below.sum().item())
            self._diag["above"] += int(d_above.sum().item())
            steps = self.sim_env.sim_steps.float()
            if bool(d_contact.any()):
                self._diag_steps["contact"] += float(steps[d_contact].sum().item())
                self._diag_x_sum += float(pos[d_contact, 0].sum().item())
                if self._bar_probe and self.perception is not None:
                    self._record_bar_contact_probe(d_contact, pos)
            if bool(d_below.any()):
                # below-death forensics: WHEN it dies (early sharp-turn transient vs late drift) and
                # HOW TILTED it is at death (tilt-induced thrust sag vs level sink). b3_z from the
                # quaternion directly: R[2][2] = 1 - 2*(qx^2 + qy^2), tilt = acos(b3_z).
                self._diag_steps["below"] += float(steps[d_below].sum().item())
                q = self.obs_dict["robot_orientation"][d_below]
                b3z = (1.0 - 2.0 * (q[:, 0] ** 2 + q[:, 1] ** 2)).clamp(-1.0, 1.0)
                self._diag_below_tilt += float(torch.rad2deg(torch.acos(b3z)).sum().item())
            if self.vision_mode:
                d_oob = oob & ~crashed & ~below & ~above & crashed_out
                if bool(d_oob.any()):
                    self._diag["oob"] += int(d_oob.sum().item())
                    self._diag["oob_w"] += int((d_oob & (pos[:, 0] < b_min[:, 0] - m_oob)).sum().item())
                    self._diag["oob_e"] += int((d_oob & (pos[:, 0] > b_max[:, 0] + m_oob)).sum().item())
                    self._diag["oob_s"] += int((d_oob & (pos[:, 1] < b_min[:, 1] - m_oob)).sum().item())
                    self._diag["oob_n"] += int((d_oob & (pos[:, 1] > b_max[:, 1] + m_oob)).sum().item())
                    self._diag_steps["oob"] += float(steps[d_oob].sum().item())
                    if self._oob_probe:
                        north = d_oob & (pos[:, 1] > b_max[:, 1] + m_oob)
                        south = d_oob & (pos[:, 1] < b_min[:, 1] - m_oob)
                        lateral = north | south
                        if bool(lateral.any()):
                            side = torch.where(
                                north, torch.ones_like(pos[:, 1]), -torch.ones_like(pos[:, 1])
                            )
                            arena_mid_y = 0.5 * (b_min[:, 1] + b_max[:, 1])
                            arena_half_y = 0.5 * (b_max[:, 1] - b_min[:, 1]).clamp(min=1e-6)
                            command_world = quat_rotate(
                                self.obs_dict["robot_vehicle_orientation"],
                                self.command[:, 0:3],
                            )
                            excursion = torch.where(
                                north,
                                self._probe_ep_y_max - self._probe_ep_start_y,
                                self._probe_ep_start_y - self._probe_ep_y_min,
                            )
                            p = self._probe
                            p["n"] += float(lateral.sum().item())
                            p["start_y"] += float(self._probe_ep_start_y[lateral].sum().item())
                            p["goal_pull_side"] += float(
                                (
                                    (self._probe_ep_target_start_y - self._probe_ep_start_y)
                                    * side
                                )[lateral].sum().item()
                            )
                            p["goal_now_pull_side"] += float(
                                (
                                    (self.target_position[:, 1] - self._probe_ep_start_y) * side
                                )[lateral].sum().item()
                            )
                            p["bar_bias_side"] += float(
                                (
                                    (self._probe_ep_bar_mean_y - arena_mid_y)
                                    / arena_half_y
                                    * side
                                )[lateral].sum().item()
                            )
                            p["world_vy_side"] += float(
                                (self.obs_dict["robot_linvel"][:, 1] * side)[lateral].sum().item()
                            )
                            p["command_vy_side"] += float(
                                (command_world[:, 1] * side)[lateral].sum().item()
                            )
                            p["action_y_side"] += float(
                                (self.prev_action[:, 1] * side)[lateral].sum().item()
                            )
                            p["excursion_side"] += float(excursion[lateral].sum().item())
                            p["visible"] += float(self._visible_now[lateral].sum().item())
                            if self.perception is not None:
                                tracker = self.perception.tracker
                                p["track_age"] += float(tracker.age[lateral].sum().item())
                                cov_pos = torch.diagonal(
                                    tracker.cov[:, :3, :3], dim1=1, dim2=2
                                ).sum(dim=1)
                                p["track_cov_pos"] += float(cov_pos[lateral].sum().item())

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
        if self.vision_mode:
            # Target returns are valid perception but are not collision obstacles for r_ss.
            seg = self.obs_dict["segmentation_pixels"].squeeze(1).reshape(self.num_envs, -1)
            dist_m = torch.where(
                seg == 50, torch.full_like(dist_m, self.task_config.lidar_max_range), dist_m
            )
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
        if self.perception_mode:
            self._process_obs_perception()
            return
        if self.vision_mode:
            self._process_obs_vision()
            return
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

    def _process_obs_vision(self):
        """Sensor-only ACTOR observation + privileged CRITIC 'states' (vision mode).

        actor (1265) = ego 9 [vel_vehicle/vmax(3), yaw_rate/max(1), prev_action(4), height(1)]
                    + detector 8 [visible, bearing sin/cos, elev, range | tracker 3]
                    + LiDAR range (144) + LiDAR target mask (144)
                    + forward camera obstacle depth (40x24=960)
        states (1273) = actor obs + GT extras [rpos_unit_veh(3), dist/24(1), tvel_veh/2(3),
                       closing/(vmax+2)(1)] — read ONLY by the central-value critic at train time.

        No ground-truth target quantity enters the actor slice: the target appears only through
        LiDAR returns and the FOV/occlusion-gated camera detector. Both LiDAR and camera also
        observe obstacle geometry."""
        pos = self.obs_dict["robot_position"]
        q_veh = self.obs_dict["robot_vehicle_orientation"]
        rpos_w = self.target_position - pos
        rpos_veh = quat_rotate_inverse(q_veh, rpos_w)

        det_vec, visible = self.detector.detect(
            pos, q_veh, self.target_position, update_tracker=True
        )
        self._visible_now[:] = visible
        # Expose raw target-camera products for evaluation/debugging. Target mask/depth are reduced
        # to det_vec; the separate obstacle depth image is concatenated into the actor below.
        self.obs_dict["target_camera_mask"] = self.detector.target_mask
        self.obs_dict["target_camera_depth"] = self.detector.target_depth
        self.obs_dict["obstacle_camera_depth"] = self.detector.obstacle_depth

        vel_veh = self.obs_dict["robot_vehicle_linvel"] / self.task_config.max_velocity
        yaw_rate = self.obs_dict["robot_body_angvel"][:, 2:3] / self.task_config.yaw_rate_max
        height = pos[:, 2:3] / 3.0
        ego = torch.cat([vel_veh, yaw_rate, self.prev_action, height], dim=1)  # (N, 9)

        lidar = self._lidar_distance_m() / self.task_config.lidar_max_range  # (N, 144)
        seg = self.obs_dict["segmentation_pixels"].squeeze(1).reshape(self.num_envs, -1)
        lidar_target = (seg == 50).float()
        camera_obstacle = torch.nan_to_num(
            self.detector.obstacle_depth,
            nan=self.vis_cfg.camera_obstacle_max_range,
            posinf=self.vis_cfg.camera_obstacle_max_range,
            neginf=self.vis_cfg.camera_obstacle_max_range,
        ).clamp(0.0, self.vis_cfg.camera_obstacle_max_range)
        camera_obstacle = (
            camera_obstacle / self.vis_cfg.camera_obstacle_max_range
        ).reshape(self.num_envs, -1)

        obs = self.task_obs["observations"]
        d0 = self.vis_cfg.ego_dim
        d1 = d0 + self.vis_cfg.detector_dim
        d2 = d1 + lidar.shape[1]
        d3 = d2 + lidar_target.shape[1]
        obs[:, :d0] = ego
        obs[:, d0:d1] = det_vec
        obs[:, d1:d2] = lidar
        obs[:, d2:d3] = lidar_target
        if not bool(getattr(self.vis_cfg, "legacy_actor_305", False)):
            obs[:, d3:] = camera_obstacle

        # privileged critic extras (train-time only; the player ignores 'states')
        dist = rpos_w.norm(dim=1, keepdim=True).clamp(min=1e-6)
        tvel_veh = quat_rotate_inverse(q_veh, self.target_vel_w)
        closing = ((self.obs_dict["robot_linvel"] - self.target_vel_w) * (rpos_w / dist)).sum(
            dim=1, keepdim=True
        )
        states = self.task_obs["states"]
        states[:, : obs.shape[1]] = obs
        states[:, obs.shape[1] :] = torch.cat(
            [
                rpos_veh / dist,
                dist / 24.0,
                tvel_veh / 2.0,
                closing / (self.task_config.max_velocity + 2.0),
            ],
            dim=1,
        )

    def _process_obs_perception(self):
        """Raw RGB-D/LiDAR -> perception tracks -> actor-safe structured history.

        ``target_position`` is passed only to the simulator-side renderer, exactly as scene pose is
        used by a physical renderer. The perception module API has no target/semantic argument;
        its output is therefore structurally unable to read the oracle target state.
        """
        pos = self.obs_dict["robot_position"]
        vel_w = self.obs_dict["robot_linvel"]
        q_veh = self.obs_dict["robot_vehicle_orientation"]

        raw_rgb, raw_depth = self.detector.render_raw_rgbd(
            pos, q_veh, self.target_position
        )
        lidar_m = self._lidar_distance_m()
        structured, diagnostics = self.perception.observe(
            rgb=raw_rgb,
            depth=raw_depth,
            lidar_m=lidar_m,
            drone_pos_w=pos,
            drone_vel_w=vel_w,
            vehicle_quat=q_veh,
            yaw_rate=self.obs_dict["robot_body_angvel"][:, 2]
            / self.task_config.yaw_rate_max,
            previous_action=self.prev_action,
            max_velocity=self.task_config.max_velocity,
            flight_altitude=self.task_config.flight_altitude,
            training=bool(self.perception_cfg.enable_perturbations),
        )
        self.task_obs["observations"][:] = structured
        self._visible_now[:] = diagnostics["visible"]

        # Raw sensor and tracker diagnostics are available to evaluators, never concatenated into
        # actor observations. Semantic renderer buffers intentionally remain private.
        self.obs_dict["navrl_raw_rgb"] = raw_rgb
        self.obs_dict["navrl_raw_depth"] = raw_depth
        self.obs_dict["navrl_track_confidence"] = diagnostics["confidence"]
        self.obs_dict["navrl_track_age"] = diagnostics["track_age"]
        self.obs_dict["navrl_track_covariance"] = diagnostics["track_covariance"]

        # Asymmetric critic: oracle quantities are appended only to the physically separate
        # states buffer. rl_games' player drops this entire tensor at deployment.
        rpos_w = self.target_position - pos
        dist = rpos_w.norm(dim=1, keepdim=True).clamp(min=1e-6)
        rpos_veh = quat_rotate_inverse(q_veh, rpos_w)
        tvel_veh = quat_rotate_inverse(q_veh, self.target_vel_w)
        closing = ((vel_w - self.target_vel_w) * (rpos_w / dist)).sum(
            dim=1, keepdim=True
        )
        obs = self.task_obs["observations"]
        states = self.task_obs["states"]
        states[:, : obs.shape[1]] = obs
        states[:, obs.shape[1] :] = torch.cat(
            [
                rpos_veh / dist,
                dist / 24.0,
                tvel_veh / 2.0,
                closing / (self.task_config.max_velocity + 2.0),
            ],
            dim=1,
        )

    def _goal_x_max(self):
        """Epoch-proportional goal-x ceiling: ramps k_start -> k_final over k_warmup_epochs, then
        plateaus. Uses num_task_steps as an epoch proxy (rl_games collects ppo_horizon env-steps per
        epoch, so epoch ~= num_task_steps / ppo_horizon). num_task_steps is saved/restored via
        get_env_state/set_env_state, so a --checkpoint resume (and --play) continues at the saved
        curriculum position."""
        if getattr(self.cur, "use_competence", False):
            return self._k_max_cur  # competence-gated: advanced only in _update_curriculum
        warmup_steps = max(1, int(self.cur.k_warmup_epochs) * int(self.cur.ppo_horizon))
        frac = min(1.0, self.num_task_steps / warmup_steps)
        return self.cur.k_start + (self.cur.k_final - self.cur.k_start) * frac

    def _goal_x_min(self):
        """Goal-x floor: stays at k_min early, then ramps k_min -> k_min_final over
        [k_min_ramp_start_epochs, +k_min_ramp_epochs] so late episodes drop the easy near goals and
        focus on deep crossings. The start is independent of the k_max ramp (they may overlap). Kept
        at least 1 m below k_max so the [min, max] window stays valid."""
        if getattr(self.cur, "use_competence", False):
            return min(self._k_min_cur, self._goal_x_max() - 1.0)
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
        if self._runtime_target_speed is not None:
            return float(self._runtime_target_speed)
        if float(self.tm.speed_fixed) >= 0.0:
            return float(self.tm.speed_fixed)
        final = float(self.tm.speed_final)
        minimum = max(0.0, float(getattr(self.tm, "speed_min", 0.0)))
        if final <= 0.0:
            return 0.0
        h = int(self.cur.ppo_horizon)
        start_steps = int(self.tm.speed_ramp_start_epochs) * h
        ramp_steps = max(1, int(self.tm.speed_ramp_epochs) * h)
        frac = min(1.0, max(0.0, (self.num_task_steps - start_steps) / ramp_steps))
        return max(minimum, final * frac)

    def _sample_target_motion(self, env_ids):
        """Per-episode target speed + trajectory pattern for reset envs. Training samples
        speed ~ U[speed_min, v_max(epoch)]; the default speed_min=0 keeps static/slow episodes
        in-distribution. NAVRL_TARGET_SPEED forces the exact speed instead (evaluation cells)."""
        n = len(env_ids)
        if n == 0:
            return
        v_max = self._target_speed_max()
        if self._runtime_target_speed is not None:
            speed = torch.full((n,), float(self._runtime_target_speed), device=self.device)
        elif float(self.tm.speed_fixed) >= 0.0:
            speed = torch.full((n,), float(self.tm.speed_fixed), device=self.device)
        else:
            v_min = min(v_max, max(0.0, float(getattr(self.tm, "speed_min", 0.0))))
            speed = v_min + (v_max - v_min) * torch.rand(n, device=self.device)
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

    def _sync_target_to_sensor(self):
        """Mirror the moving target into the analytic semantic-LiDAR target buffer."""
        if self._sensor_target is not None:
            self._sensor_target[:] = self.target_position

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
        # (A) Competence-gated goal-DISTANCE curriculum (NAVRL_K_COMPETENCE=1): deepen the goal
        # window only when measured capture clears the threshold -- the same self-pacing the density
        # curriculum uses below. When off, distance stays epoch-proportional in _goal_x_max/_min.
        if getattr(self.cur, "use_competence", False):
            n_fin_d = int(torch.sum(finished).item())
            if n_fin_d > 0:
                self._kcomp_succ += int(torch.sum(successes).item())
                self._kcomp_fin += n_fin_d
                if self._kcomp_fin >= max(1, int(self.cur.k_comp_check)):
                    rate = self._kcomp_succ / max(1, self._kcomp_fin)
                    if rate >= float(self.cur.k_comp_threshold) and self._k_max_cur < float(self.cur.k_final):
                        old = self._k_max_cur
                        self._k_max_cur = min(float(self.cur.k_final), self._k_max_cur + float(self.cur.k_comp_step))
                        self._k_min_cur = min(
                            float(self.cur.k_min_final), max(float(self.cur.k_min), self._k_max_cur - 3.0)
                        )
                        logger.warning(
                            "NavRL distance curriculum promoted | k_max %.1f -> %.1f window [%.1f, %.1f] "
                            "after %d eps, capture=%.3f"
                            % (old, self._k_max_cur, self._k_min_cur, self._k_max_cur, self._kcomp_fin, rate)
                        )
                    else:
                        logger.info(
                            "NavRL distance curriculum held | window [%.1f, %.1f] capture=%.3f over %d eps"
                            % (self._k_min_cur, self._k_max_cur, rate, self._kcomp_fin)
                        )
                    self._kcomp_succ = 0
                    self._kcomp_fin = 0
        # (B) Optional Phase-2 density curriculum (also capture-gated; orthogonal to distance above).
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
            # Training runs suppress INFO logs, so an INFO-only hold made "no promotion line"
            # ambiguous: the gate may not have been evaluated yet, or it may have failed. Keep the
            # competence decision visible at the same level as a promotion.
            logger.warning(
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

    def _export_bulk_eval_result(self, total, reach_rate, mean_nc, best):
        """Persist the exact outcome window consumed by a vectorized rl_games player."""
        if not self._bulk_eval_mode or self._bulk_eval_exported:
            return

        ad = self._action_diag
        n_action = max(1, int(ad["n"]))
        n_delta = max(1, int(ad["delta_y_n"]))
        d = self._diag
        n_crash_causes = d["contact"] + d["below"] + d["above"] + d["oob"]
        n_cause_den = max(1, n_crash_causes)

        def _float_env(name, default):
            try:
                return float(os.environ.get(name, str(default)) or default)
            except ValueError:
                return float(default)

        payload = {
            "schema_version": 1,
            "requested_episodes": int(self._bulk_eval_target),
            "actual_episodes": int(total),
            "checkpoint": os.environ.get("NAVRL_EVAL_CHECKPOINT", ""),
            "condition": {
                "bars": int(self.n_bars_active),
                "target_pattern": os.environ.get("NAVRL_TARGET_PATTERN", "static"),
                "target_speed_mps": _float_env("NAVRL_TARGET_SPEED", 0.0),
                "pursuer_max_speed_mps": float(self.task_config.max_velocity),
                "episode_len_steps": int(self.task_config.episode_len_steps),
                "num_envs": int(self.num_envs),
            },
            "outcome": {
                "captured": int(self._succ_agg),
                "crash": int(self._crash_agg),
                "timeout": int(self._to_agg),
                "capture_rate": float(self._succ_agg / max(1, total)),
                "crash_rate": float(self._crash_agg / max(1, total)),
                "timeout_rate": float(self._to_agg / max(1, total)),
                "ever_reached_rate": float(reach_rate),
                "closest_nocrash_mean_m": float(mean_nc),
                "closest_nocrash_best_m": None if math.isnan(best) else float(best),
                "closest_nocrash_count": int(self._nc_agg),
            },
            "action": {
                "policy": os.environ.get("NAVRL_ACTION_POLICY", "legacy"),
                "samples": int(ad["n"]),
                "task_input_oob_rate": [
                    float(value / n_action) for value in ad["raw_oob"]
                ],
                "executed_edge98_rate": [
                    float(value / n_action) for value in ad["exec_edge"]
                ],
                "executed_edge95_rate": [
                    float(value / n_action) for value in ad["exec_edge95"]
                ],
                "executed_edge99_rate": [
                    float(value / n_action) for value in ad["exec_edge99"]
                ],
                "mean_abs": [float(value / n_action) for value in ad["abs_sum"]],
                "signed_mean_y": float(ad["signed_y_sum"] / n_action),
                "positive_y_rate": float(ad["positive_y"] / n_action),
                "negative_y_rate": float(ad["negative_y"] / n_action),
                "high80_y_rate": float(ad["high80_y"] / n_action),
                "mean_abs_delta_y": float(ad["delta_y_sum"] / n_delta),
                "sign_flip_y_rate": float(ad["sign_flip_y"] / n_delta),
                "context": {
                    name: {
                        "samples": int(ad[name + "_n"]),
                        "fraction": float(ad[name + "_n"] / n_action),
                        "mean_abs_y": float(
                            ad[name + "_abs_y"] / max(1.0, ad[name + "_n"])
                        ),
                    }
                    for name in (
                        "front_clear",
                        "front_blocked",
                        "goal_centered",
                        "goal_offcenter",
                        "clear_centered",
                        "target_visible",
                        "target_hidden",
                    )
                },
            },
            "crash_causes": {
                "count": int(n_crash_causes),
                "bar_contact": int(d["contact"]),
                "below": int(d["below"]),
                "above": int(d["above"]),
                "out_of_bounds": int(d["oob"]),
                "bar_contact_share": float(d["contact"] / n_cause_den),
                "below_share": float(d["below"] / n_cause_den),
                "above_share": float(d["above"] / n_cause_den),
                "out_of_bounds_share": float(d["oob"] / n_cause_den),
            },
        }
        compact = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        print("NAVRL_BULK_EVAL_RESULT " + compact, flush=True)

        if not self._bulk_eval_output:
            logger.warning(
                "NavRL bulk eval result was printed but NAVRL_BULK_EVAL_JSON is unset."
            )
            self._bulk_eval_exported = True
            return
        try:
            out = Path(self._bulk_eval_output)
            out.parent.mkdir(parents=True, exist_ok=True)
            tmp = out.with_name(out.name + ".tmp")
            tmp.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
            tmp.replace(out)
            logger.warning("NavRL bulk eval results saved -> %s" % out)
            self._bulk_eval_exported = True
        except OSError as exc:
            logger.warning("NavRL bulk eval results export failed: %s" % exc)

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
        if total >= self._progress_log_interval:
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
            self._export_bulk_eval_result(total, reach_rate, mean_nc, best)
            if self._action_diag_enabled and self._action_diag["n"] > 0:
                ad = self._action_diag
                n_action = max(1, int(ad["n"]))
                n_delta = max(1, int(ad["delta_y_n"]))
                raw_oob = [value / n_action for value in ad["raw_oob"]]
                exec_edge = [value / n_action for value in ad["exec_edge"]]
                mean_abs = [value / n_action for value in ad["abs_sum"]]
                logger.warning(
                    "NavRL actiondiag | policy=%s std=%s "
                    "task_input_oob[x,y,z,yaw]=[%.4f,%.4f,%.4f,%.4f] "
                    "exec_edge98=[%.4f,%.4f,%.4f,%.4f] "
                    "mean_abs=[%.3f,%.3f,%.3f,%.3f] signed_y=%.3f "
                    "pos_y=%.3f neg_y=%.3f high80_y=%.3f "
                    "delta_y=%.3f sign_flip_y=%.3f (n=%d)"
                    % (
                        os.environ.get("NAVRL_ACTION_POLICY", "legacy"),
                        os.environ.get("NAVRL_ACTION_STD", "learned"),
                        *raw_oob,
                        *exec_edge,
                        *mean_abs,
                        ad["signed_y_sum"] / n_action,
                        ad["positive_y"] / n_action,
                        ad["negative_y"] / n_action,
                        ad["high80_y"] / n_action,
                        ad["delta_y_sum"] / n_delta,
                        ad["sign_flip_y"] / n_delta,
                        n_action,
                    )
                )
                logger.warning(
                    "NavRL actioncontext | clear=%.3f/|y|%.3f blocked=%.3f/|y|%.3f "
                    "centered=%.3f/|y|%.3f offcenter=%.3f/|y|%.3f "
                    "clear_centered=%.3f/|y|%.3f visible=%.3f/|y|%.3f"
                    % (
                        ad["front_clear_n"] / n_action,
                        ad["front_clear_abs_y"] / max(1.0, ad["front_clear_n"]),
                        ad["front_blocked_n"] / n_action,
                        ad["front_blocked_abs_y"] / max(1.0, ad["front_blocked_n"]),
                        ad["goal_centered_n"] / n_action,
                        ad["goal_centered_abs_y"] / max(1.0, ad["goal_centered_n"]),
                        ad["goal_offcenter_n"] / n_action,
                        ad["goal_offcenter_abs_y"] / max(1.0, ad["goal_offcenter_n"]),
                        ad["clear_centered_n"] / n_action,
                        ad["clear_centered_abs_y"] / max(1.0, ad["clear_centered_n"]),
                        ad["target_visible_n"] / n_action,
                        ad["target_visible_abs_y"] / max(1.0, ad["target_visible_n"]),
                    )
                )
                self._action_diag = self._empty_action_diag()
            if self._crash_diag:
                d = self._diag
                n_raw = d["contact"] + d["below"] + d["above"] + d["oob"]
                n_all = max(1, n_raw)  # division guard only; the printed count is the raw sum
                logger.warning(
                    "NavRL crashdiag | bar_contact=%.3f (mean_x=%.1fm steps=%.0f) below=%.3f "
                    "(steps=%.0f tilt=%.0fdeg) "
                    "above=%.3f oob=%.3f [W=%d E=%d S=%d N=%d steps=%.0f] (n_crash=%d)"
                    % (
                        d["contact"] / n_all,
                        self._diag_x_sum / max(1, d["contact"]),
                        self._diag_steps["contact"] / max(1, d["contact"]),
                        d["below"] / n_all,
                        self._diag_steps["below"] / max(1, d["below"]),
                        self._diag_below_tilt / max(1, d["below"]),
                        d["above"] / n_all,
                        d["oob"] / n_all,
                        d["oob_w"],
                        d["oob_e"],
                        d["oob_s"],
                        d["oob_n"],
                        self._diag_steps["oob"] / max(1, d["oob"]),
                        n_raw,
                    )
                )
                if self._oob_probe and self._probe["n"] > 0:
                    p = self._probe
                    n_probe = p["n"]
                    logger.warning(
                        "NavRL oobprobe | lateral_n=%d start_y=%.2fm "
                        "goal_pull_side=%.2fm goal_now_pull_side=%.2fm "
                        "bar_bias_side=%.3f outward_vy=%.2fm/s outward_cmd_vy=%.2fm/s "
                        "action_y_side=%.3f excursion=%.2fm visible=%.3f "
                        "track_age=%.2fs track_cov_pos=%.3f"
                        % (
                            int(n_probe),
                            p["start_y"] / n_probe,
                            p["goal_pull_side"] / n_probe,
                            p["goal_now_pull_side"] / n_probe,
                            p["bar_bias_side"] / n_probe,
                            p["world_vy_side"] / n_probe,
                            p["command_vy_side"] / n_probe,
                            p["action_y_side"] / n_probe,
                            p["excursion_side"] / n_probe,
                            p["visible"] / n_probe,
                            p["track_age"] / n_probe,
                            p["track_cov_pos"] / n_probe,
                        )
                    )
                if self._bar_probe and self._bprobe["n"] > 0:
                    bp = self._bprobe
                    nb = bp["n"]
                    n_match = max(1.0, bp["hit_in_tokens"])
                    n_hit_fov = max(1.0, bp["hit_in_token_fov"])
                    logger.warning(
                        "NavRL barprobe v2 | n=%d bars_range=%.1f bars_fov=%.1f occupied_bins=%.1f "
                        "hit_dist=%.2fm hit_fov=%.3f hit_token=%.3f hit_token_given_fov=%.3f "
                        "tokens=%.1f associated=%.1f unique=%.1f duplicate=%.1f "
                        "center_offset=%.2fm cross_track=%.2fm radial_gap=%.2fm rank=%.1f "
                        "(capacity=%d)"
                        % (
                            int(nb),
                            bp["bars_in_range"] / nb,
                            bp["bars_in_token_fov"] / nb,
                            bp["occupied_bins"] / nb,
                            bp["hit_dist"] / nb,
                            bp["hit_in_token_fov"] / nb,
                            bp["hit_in_tokens"] / nb,
                            bp["hit_in_tokens_in_fov"] / n_hit_fov,
                            bp["valid_tokens"] / nb,
                            bp["associated_tokens"] / nb,
                            bp["unique_token_bars"] / nb,
                            bp["duplicate_tokens"] / nb,
                            bp["hit_center_offset"] / n_match,
                            bp["hit_cross_track"] / n_match,
                            bp["hit_radial_gap"] / n_match,
                            bp["hit_token_rank"] / n_match,
                            self._max_obstacles_or_zero(),
                        )
                    )
                self._diag = {k: 0 for k in self._diag}
                self._diag_steps = {"contact": 0.0, "oob": 0.0, "below": 0.0}
                self._diag_below_tilt = 0.0
                self._bprobe = {k: 0.0 for k in self._bprobe}
                self._diag_x_sum = 0.0
                self._probe = {k: 0.0 for k in self._probe}
            self._succ_agg = self._crash_agg = self._to_agg = 0
            self._reach_agg = self._fin_agg = 0
            self._mindist_sum = 0.0
            self._nc_agg = 0
            self._closest_min = None
