import os
import sys


def _warn_bad_env(name, raw, default):
    # A malformed knob (e.g. NAVRL_K_FINAL=abc or a stray unit like "1.5m") used to be swallowed
    # silently and trained on the default — a nasty invisible confound. Make it loud instead.
    print(
        "[navrl config] WARNING: %s=%r could not be parsed; using default %r."
        % (name, raw, default),
        file=sys.stderr,
    )


def _env_int(name, default):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return int(default)
    try:
        return int(raw)
    except ValueError:
        _warn_bad_env(name, raw, default)
        return int(default)


def _env_float(name, default):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return float(default)
    try:
        return float(raw)
    except ValueError:
        _warn_bad_env(name, raw, default)
        return float(default)


def _env_bool(name, default=False):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return bool(default)
    s = raw.strip().lower()
    if s in ("1", "true", "yes", "on"):
        return True
    if s in ("0", "false", "no", "off"):
        return False
    # Tolerate numeric forms like "1.0"/"0.0" (a common footgun: NAVRL_VISION=1.0 used to be False).
    try:
        return float(s) != 0.0
    except ValueError:
        _warn_bad_env(name, raw, default)
        return bool(default)


class task_config:
    """NavRL reimplementation: static bar field + capture task (Phases 1-3).

    Observation and reward follow NavRL (Xu et al., RA-L 2025), static branch, adapted for
    interception-style capture and a (Phase 3) moving target:
      state  = S_int (12: goal-frame direction/distance/velocity 8 + body-frame goal bearing 2
               + body-frame velocity 2) concatenated with the flattened 36x4 LiDAR scan -> 156.
      action = 4-D: 3D velocity command in the goal frame (scaled to +/- max_velocity)
               + learned yaw-rate (option b, scaled to +/- yaw_rate_max).
      reward = range_rate + time_cost(alive<0) + static_safety - smooth - height - yaw_align
               - yaw_rate_damping + PBRS_progress, with a terminal +capture_bonus (episode ends
               on capture) and collision_penalty on a crash. With a static target (target_motion
               speed 0, the default) range_rate reduces exactly to NavRL's velocity-toward-goal.
    Dynamic obstacles are added in a later phase.
    """

    # NOTE: --seed only reaches the task in --train mode; the rl_games PLAYER drops it. For
    # reproducible evals use NAVRL_SEED=<n> ./play_navrl.sh ... instead.
    seed = _env_int("NAVRL_SEED", 42)
    # Default "base_sim" (8 GB main machine). A 4 GB secondary machine sets
    # AERIAL_GYM_SIM_NAME=base_sim_4gb (via GPU4GB=1 ./train_navrl.sh) to shrink the PhysX
    # GPU buffers so training fits in 4 GB VRAM. See base_sim_4gb_config.py.
    sim_name = os.environ.get("AERIAL_GYM_SIM_NAME", "base_sim")
    # Controlled arena: empty space + density-controlled static bars (no walls/panels).
    # See navrl_bars_env.py and class density below.
    env_name = "navrl_bars_env"
    robot_name = "navrl_quad"
    controller_name = "lee_velocity_control_navrl"  # NavRL-scoped: raises yaw-rate clamp pi/3 -> 2.5
    args = {}
    num_envs = 256
    use_warp = True
    headless = True
    device = "cuda:0"

    # LiDAR scan geometry -- must stay in sync with NavRLLidarConfig.
    lidar_hbeams = 36
    lidar_vbeams = 4
    lidar_max_range = 4.0

    # ------------------------------------------------------------------ Phase-3 vision pivot
    class vision:
        """NAVRL_VISION=1: the ACTOR perceives the target only through its sensors -- no ground-
        truth target position in the policy observation. Reward/termination/critic keep GT
        (privileged reward shaping + asymmetric critic are standard and do not leak into the
        deployed actor). Default OFF keeps the verified Phases 1-2 injected-goal task
        byte-compatible.

        Perception channels:
          1. semantic LiDAR: the 36x4 range image observes both environment geometry and the
             target; a second per-ray channel marks target returns.
          2. forward depth/segmentation CAMERA: a low-resolution full-scene depth image observes
             obstacles, while a higher-resolution target mask/depth image provides detection.
             Bars remove occluded target pixels, and bearing/elevation/range are calculated only
             from surviving pixels. The target pose is renderer-only and never enters the actor.
          3. a detector-side tracker (last-seen bearing + time-since-seen) -- the standard
             onboard filter memory (NavRL itself tracks dynamic obstacles onboard).
        """

        enable = _env_bool("NAVRL_VISION", False)
        # Read-only checkpoint compatibility for the 2026-07 semantic CNN runs. Those actors were
        # trained before the 40x24 obstacle-camera channel was added and therefore require the
        # original 17 + 144 + 144 = 305 observation exactly. Never use this for new training.
        legacy_actor_305 = _env_bool("NAVRL_LEGACY_VISION", False)

        # -- forward target camera (gimbal-level like the yaw-only LiDAR attach)
        detector_max_range = 20.0   # [m] detection range (a segmentation camera covers this)
        detector_hfov_deg = 87.0    # matches the D455-style forward depth camera
        detector_vfov_deg = 58.0
        tracker_memory_s = 5.0      # time_since_seen saturates at this many seconds
        camera_width = 160          # efficient target-only semantic render; aspect ratio 16:9
        camera_height = 90
        camera_target_radius = 0.15 # [m], matches the 0.30 m target-drone footprint
        camera_min_target_pixels = 1
        camera_translation = [0.10, 0.0, 0.03]  # vehicle frame, forward/left/up [m]
        # Full-scene depth supplied to the actor. Kept lower-resolution than target detection to
        # preserve vectorized PPO throughput while still adding forward obstacle geometry.
        camera_obstacle_width = 40
        camera_obstacle_height = 24
        camera_obstacle_max_range = 10.0
        camera_obstacle_dim = camera_obstacle_width * camera_obstacle_height

        # -- reward add-ons (vision mode only)
        visibility_bonus = 0.02     # per-step bonus while the detector sees the target
        oob_margin = 0.5            # [m] outside the arena bounds by this much -> terminate as a
                                    # crash (the goal attraction that implicitly kept the drone
                                    # in-bounds is gone when the target is unobserved)

        # -- observation layout (actor)
        ego_dim = 9        # vel_vehicle(3) + yaw_rate(1) + prev_action(4) + height(1)
        detector_dim = 8   # visible, bearing sin/cos, elev, range | last bearing sin/cos, t_since
        privileged_dim = 8 # critic-only extras: rpos_unit_veh(3), dist, target_vel_veh(3), closing

    class perception:
        """Real sensor-to-track path used by the NavRL++-Target Transformer.

        Enable with ``NAVRL_VISION=1 NAVRL_PERCEPTION=1``. The legacy semantic prototype remains
        available only for baseline/checkpoint compatibility when NAVRL_PERCEPTION is unset.
        """

        enable = _env_bool("NAVRL_PERCEPTION", False)
        history_steps = 5
        history_interval_s = 0.5
        max_obstacles = 5
        lidar_max_range = 4.0
        min_target_pixels = _env_int("NAVRL_DETECTOR_MIN_PIXELS", 2)
        pixel_threshold = _env_float("NAVRL_DETECTOR_THRESHOLD", 0.55)
        detector_checkpoint = os.environ.get("NAVRL_DETECTOR_CHECKPOINT", "")
        # Perturbations are opt-in for the post-training stage, not silently applied to clean runs.
        enable_perturbations = _env_bool("NAVRL_PERCEPTION_PERTURB", False)
        detection_dropout_prob = _env_float("NAVRL_DETECTION_DROPOUT", 0.3)
        rgb_noise_std = _env_float("NAVRL_RGB_NOISE_STD", 0.015)
        depth_noise_std = _env_float("NAVRL_DEPTH_NOISE_STD", 0.02)
        # [static 144 | obstacle history 5x5x12 | robot history 5x10 |
        #  target history 5x16] -> 574. The network converts these to 17 tokens.
        observation_dim = 574

    # Observation:
    #   default      = S_int(12) + LiDAR range(144)                            = 156
    #   NAVRL_VISION = ego+detector(17) + LiDAR range+target(288)
    #                  + camera obstacle depth(40x24=960)                      = 1265
    #   critic states = actor obs + privileged extras(8)                       = 1273
    if vision.enable and perception.enable:
        internal_state_dim = perception.observation_dim
        observation_space_dim = perception.observation_dim
        state_space_dim = observation_space_dim + vision.privileged_dim
    elif vision.enable:
        internal_state_dim = vision.ego_dim + vision.detector_dim  # network state/scan split
        observation_space_dim = (
            internal_state_dim
            + 2 * lidar_hbeams * lidar_vbeams
            + (0 if vision.legacy_actor_305 else vision.camera_obstacle_dim)
        )
        state_space_dim = observation_space_dim + vision.privileged_dim
    else:
        internal_state_dim = 12  # (b) 8 nav dims + goal_bearing_body(2) + vel_body_xy(2)
        observation_space_dim = internal_state_dim + lidar_hbeams * lidar_vbeams
        state_space_dim = 0  # no asymmetric critic in the injected-goal task

    # Action = 3D velocity in the goal frame (NavRL) + a learned 4th action = yaw-rate (option b).
    # Yaw used to be HELD; now action[:, 3] commands the euler yaw-rate so the agent points its nose
    # along its travel direction (shrinks the swept footprint 0.40 m diagonal -> 0.28 m -> fewer clips).
    action_space_dim = 4

    episode_len_steps = _env_int("NAVRL_EPISODE_LEN_STEPS", 300)  # RL steps (300 for far goals;
    # ~150 steps straight / ~225 weaving at 2 m/s, so 300 keeps timeout from being the failure mode)

    max_velocity = _env_float("NAVRL_MAX_VELOCITY", 2.0)  # NavRL v_lim [m/s]

    # (b) Learned yaw control. yaw_rate_max MUST equal lee_controller_config_navrl.max_yaw_rate so
    # action[:, 3] in [-1, 1] maps linearly onto the controller's yaw-rate clamp (no dead band).
    yaw_rate_max = 2.5          # [rad/s] max commanded euler yaw-rate for action[:, 3]
    yaw_align_speed_ref = 1.0   # [m/s] speed at which the crab-alignment penalty reaches full weight

    # 2D navigation: the drone flies at this fixed altitude and tracks the goal in XY only.
    # The drone spawns here (navrl_quad init z-ratio) and the task zeroes the vertical velocity
    # command; the goal is placed (and, in Phase 3, moves) at this same altitude.
    flight_altitude = 1.0  # [m]

    # Goal placement: "cross the bar field" -- the drone spawns at the left edge (x~0) and the
    # goal is placed on the far side at x=k, so every episode traverses the whole bar field.
    # k grows epoch-proportionally: near goals first, then progressively deeper crossings.
    class curriculum:
        # The drone spawns at x~0 and the goal is placed at x=k on the far side, so every episode
        # must traverse the bars (band x in ~[3.1, 23.0]). k grows with training (epoch-proportional).
        #   goal x ~ U[k_min, k_max(t)],  goal y ~ U[wall_margin, arena_y - wall_margin]
        #   k_max(t) = k_start + (k_final - k_start) * min(1, epoch / k_warmup_epochs)
        # k_final / k_min_final are env-overridable so a SENSOR-ONLY run can keep goals inside the
        # detector's reliable range (vision.detector_max_range = 20 m). Deep goals (24 m) at high
        # density made the sensor-only policy over-cautious and time out; NAVRL_K_FINAL=16
        # NAVRL_K_MIN_FINAL=10 keeps every goal perceivable. Defaults reproduce the LiDAR task.
        k_min = 5.0              # [m] initial nearest goal x (shallow: just inside the bar band, ~3.1 m)
        k_min_final = _env_float("NAVRL_K_MIN_FINAL", 20.0)  # [m] LATE nearest goal x. k_min ramps
        #   k_min -> k_min_final linearly over [k_min_ramp_start_epochs, +k_min_ramp_epochs],
        #   narrowing the goal window to deep crossings. Set k_min_final = k_min to disable.
        k_min_ramp_start_epochs = _env_int("NAVRL_K_MIN_RAMP_START", 2000)
        # epoch k_min STARTS rising. Independent of the k_max ramp --
        #   it overlaps it (k_max keeps ramping to k_warmup_epochs=3000 while k_min already climbs).
        k_min_ramp_epochs = _env_int("NAVRL_K_MIN_RAMP_EPOCHS", 3000)
        # epochs to ramp k_min -> k_min_final => default hits max at 2000+3000 = 5000
        k_start = 7.0            # [m] initial k_max (first bar rows)
        k_final = _env_float("NAVRL_K_FINAL", 24.0)  # [m] final k_max (far wall; clamped to arena-margin)
        k_warmup_epochs = _env_int("NAVRL_K_WARMUP", 3000)  # epochs to ramp k_start->k_final, then plateau
        ppo_horizon = 32         # rl_games horizon_length (MUST match ppo_navrl_cnn.yaml) -> steps/epoch
        wall_margin = 0.5        # [m] keep drone/goal this far from the y walls

    # Phase 2 density sweep: obstacle size stays fixed; only the active bar count changes.
    # NAVRL_MAX_BARS controls the build-time ceiling in env_object_config.py.
    class density:
        # Density (clutter) curriculum: start sparse, add promote_step bars each time the capture
        # rate over check_after_episodes clears success_threshold, until n_final. Orthogonal to the
        # goal-DISTANCE curriculum above (that one ramps by epoch; this one ramps by capture).
        # All knobs are env-overridable because the defaults were tuned for the GT-injected LiDAR
        # task; the sensor-only (vision) task caps lower at high density and masters sparse density
        # fast, so it wants a SHORTER warmup and a LOWER threshold (see the vision launch recipe).
        use_density_curriculum = _env_bool("NAVRL_DENSITY_CURRICULUM", False)
        num_bars_active = _env_int("NAVRL_NUM_BARS", 48)
        n_start = _env_int("NAVRL_DENSITY_START", 25)
        n_final = _env_int("NAVRL_DENSITY_FINAL", 150)
        success_threshold = _env_float("NAVRL_DENSITY_THRESHOLD", 0.8)
        promote_step = _env_int("NAVRL_DENSITY_STEP", 15)
        warmup_epochs = _env_int("NAVRL_DENSITY_WARMUP", 2500)
        check_after_episodes = _env_int("NAVRL_DENSITY_CHECK_EPS", 2048)
        # Once density training starts, keep a fraction of short/medium crossings in every batch.
        # This prevents catastrophic forgetting of avoidance/reacquisition at close range while
        # the main distance window remains at its final (hard) range. Zero preserves old runs.
        easy_goal_mix_prob = _env_float("NAVRL_DENSITY_EASY_GOAL_MIX", 0.0)
        easy_goal_min = _env_float("NAVRL_DENSITY_EASY_GOAL_MIN", 5.0)
        easy_goal_max = _env_float("NAVRL_DENSITY_EASY_GOAL_MAX", 10.0)

    # Phase 3 moving target (RQ2). The target is a VIRTUAL point (task-side coordinates, no
    # actor/mesh -> VRAM neutral; the LiDAR never sees it). speed_final = 0 (default) keeps the
    # target STATIC and the whole task byte-compatible with Phases 1-2.
    #   train:  NAVRL_TARGET_SPEED_FINAL=1.5 ./train_navrl.sh      (curriculum 0 -> 1.5 m/s)
    #   eval:   NAVRL_TARGET_SPEED=1.0 NAVRL_TARGET_PATTERN=cv ./play_navrl.sh <ckpt>  (exact cell)
    class target_motion:
        # Per-episode speed ~ U[0, v_max(epoch)] keeps static episodes in-distribution;
        # v_max(epoch) = speed_final * clamp((epoch - ramp_start) / ramp_epochs, 0, 1).
        # The epoch proxy is num_task_steps / ppo_horizon (checkpoint-persisted, resume-safe).
        speed_final = _env_float("NAVRL_TARGET_SPEED_FINAL", 0.0)  # [m/s] curriculum ceiling; 0 = static
        speed_ramp_start_epochs = 0
        speed_ramp_epochs = 3000       # full target speed by epoch 3000 (mirrors k_warmup)
        # Evaluation override: force the EXACT per-episode speed (heatmap cells). < 0 disables.
        speed_fixed = _env_float("NAVRL_TARGET_SPEED", -1.0)      # [m/s]
        # Trajectory pattern: "cv" (constant velocity, reflected at the wall margins),
        # "waypoint" (random waypoints), "circle" (eval-only held-out pattern),
        # "mixed" (cv/waypoint 50:50 per episode -- the training default).
        pattern = os.environ.get("NAVRL_TARGET_PATTERN", "mixed").strip().lower()
        waypoint_reach_m = 0.5   # [m] resample the waypoint when the target gets this close
        circle_radius = 2.5      # [m] circle pattern radius around the spawn point

    # Success / termination.
    # NavRL's own reach_goal condition is distance < 0.5 m (env.py:583) — an interception-
    # grade capture radius (two ~0.3 m quads with centers 0.5 m apart nearly touch).
    # Interception semantics (deliberate divergence from NavRL, whose navigation env never
    # terminates on reach): touching the capture radius ENDS the episode as a success. The
    # capture test is a swept-segment test in the target-relative frame, so a fast fly-through
    # (closing speed up to 4 m/s = 0.4 m/step) cannot tunnel between samples.
    success_radius = 0.5  # [m]
    # Keep the goal at least this far (XY) from the nearest bar so the capture sphere is
    # actually flyable; also used to push a MOVING target back out of bar clearance.
    goal_min_bar_clearance = 1.0  # [m]
    lower_height_bound = 0.1  # [m] crash if below
    upper_height_bound = 4.0  # [m] crash if above (NavRL uses 4)

    # PBRS progress reward (A) discount -- MUST equal the PPO gamma (ppo_navrl*.yaml: gamma 0.99)
    # or the distance shaping stops being optimality-preserving.
    progress_gamma = 0.99

    # Reward weights. NavRL's static branch (env.py) is:
    #   r = 1*reward_vel + 1(alive) + 1*r_safety_static - 0.1*penalty_smooth - 8*penalty_height
    #
    # NavRL keeps a constant +1 "alive" survival bonus (present in code, absent from paper
    # Eqn. 7) so that flying stays net-positive even when the safety/height terms go negative
    # near obstacles — the classic antidote to a "suicidal agent" that would crash early to
    # stop accumulating negative reward. It is safe there because NavRL's ONLY terminations are
    # bad (collision / out-of-bounds); reaching the goal is never a termination (env.py:583-587
    # feeds reach_goal to stats only). Once we terminate on capture, a positive per-step alive
    # bonus flips from protective to harmful: ending the episode early forfeits the remaining
    # steps of (alive + safety + vel) reward for a small capture bonus, so the agent learns to
    # loiter just outside the capture radius instead of entering it (observed as ~10% capture).
    # Fix (Option A): replace the survival bonus with a small per-step time cost so that reaching
    # the goal quickly is strictly optimal, and raise the terminal capture bonus to cover the
    # forfeited future reward.
    #
    # (Removed dead ends, kept in git history + CRASH_TUNING_LOG.md: the B/C/D near-obstacle
    #  clearance penalty — three null results, crash is geometric not reward-driven — and the
    #  C1 finish-funnel, superseded by PBRS + capture_bonus + learned yaw.)
    reward_parameters = {
        # Phase 3: applied to the RELATIVE velocity toward the target (range-rate,
        # (v_drone - v_target) . dir). With a static target this is EXACTLY NavRL's
        # velocity-toward-goal term.
        "vel_weight": 1.0,
        "alive_weight": -0.05,  # time cost per step (was +1 survival bonus; see note above)
        # A(crash): raised 1.0 -> 1.5 so obstacle clearance is valued more relative to the
        # velocity-toward-goal reward (the drone was shaving bars while rushing to far goals).
        # The log-distance gradient is unchanged, only its weight.
        "safety_static_weight": 1.5,
        "smooth_weight": 0.1,
        "height_weight": 8.0,
        "height_margin": 0.2,  # NavRL's +/-0.2 m tolerance band
        # A: PBRS progress reward weight -- dense per-step "got closer to the goal" gradient,
        # re-anchored to the target's CURRENT position so only the drone's own motion is credited
        # (reward = w*(||prev_pos - target|| - progress_gamma*||pos - target||)). Set 0.0 to disable.
        "progress_weight": 1.0,
        # B3: raised -10 -> -20 so a no-capture episode (~ -0.05*300 = -15 once B1 removes the
        # open-space safety income) stays strictly better than crashing -- the suicide guard that
        # the removed +1 alive bonus used to provide. NavRL leaves this commented out entirely.
        "collision_penalty": -20.0,
        # Terminal bonus when the capture radius is touched (episode ends as a success). Sized to
        # outweigh the future reward given up by ending the episode early.
        "capture_bonus": 30.0,
        # (b) Learned-yaw shaping. Dense, speed-gated crab PENALTY (<=0 so it cannot create standing
        # income / re-open the loiter optimum): punishes moving crab-wise so the drone leads with its
        # 0.28 m face, not its 0.40 m diagonal. Plus a tiny yaw-rate^2 damping. Set both 0.0 to disable.
        "yaw_align_weight": 0.3,
        "yaw_rate_smooth_weight": 0.02,
    }
