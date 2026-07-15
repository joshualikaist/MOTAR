import os

import torch


def _env_int(name, default):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return int(default)
    try:
        return int(raw)
    except ValueError:
        return int(default)


def _env_bool(name, default=False):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return bool(default)
    return raw.strip().lower() in ("1", "true", "yes", "on")


class task_config:
    """Phase 1 NavRL reimplementation: static obstacles + stationary goal.

    Observation and reward follow NavRL (Xu et al., RA-L 2025), static-only branch:
      state  = S_int (goal-frame internal state, 8) concatenated with the flattened
               36x4 LiDAR scan (see NavRLLidarConfig / navrl_quad robot).
      action = 3D velocity command in the goal frame, scaled to +/- max_velocity.
      reward = vel + time_cost(alive<0) + static_safety - smooth - height + PBRS_progress,
               with a terminal +capture_bonus on reaching the goal and -collision_penalty on a
               crash (see reward_parameters). NavRL static branch + interception-capture shaping.
    Dynamic obstacles and the moving target are added in later phases.
    """

    seed = 42
    sim_name = "base_sim"
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

    # Observation = S_int (8) + flattened LiDAR (36 * 4 = 144) = 152.
    internal_state_dim = 12  # (b) 8 nav dims + goal_bearing_body(2) + vel_body_xy(2) for learned yaw
    observation_space_dim = internal_state_dim + lidar_hbeams * lidar_vbeams
    privileged_observation_space_dim = 0

    # Action = 3D velocity in the goal frame (NavRL) + a learned 4th action = yaw-rate (option b).
    # Yaw used to be HELD; now action[:, 3] commands the euler yaw-rate so the agent points its nose
    # along its travel direction (shrinks the swept footprint 0.40 m diagonal -> 0.28 m -> fewer clips).
    action_space_dim = 4

    episode_len_steps = 300  # RL steps (300 for the far-side goals: a diagonal goal can be ~30 m,
    # ~150 steps straight / ~225 weaving at 2 m/s, so 300 keeps timeout from being the failure mode)

    return_state_before_reset = False

    max_velocity = 2.0  # NavRL v_lim [m/s]

    # (b) Learned yaw control. yaw_rate_max MUST equal lee_controller_config_navrl.max_yaw_rate so
    # action[:, 3] in [-1, 1] maps linearly onto the controller's yaw-rate clamp (no dead band).
    yaw_rate_max = 2.5          # [rad/s] max commanded euler yaw-rate for action[:, 3]
    yaw_align_speed_ref = 1.0   # [m/s] speed at which the crab-alignment penalty reaches full weight

    # 2D navigation: the drone flies at this fixed altitude and tracks the goal in XY only.
    # The drone spawns here (navrl_quad init z-ratio) and the task zeroes the vertical velocity
    # command; the goal is placed at this same altitude. (A future fleeing target also moves in XY.)
    flight_altitude = 1.0  # [m]

    # Stationary goal placement.
    #  - Curriculum ON (default): "cross the bar field" -- the drone spawns at the left edge (x~0)
    #    and the goal is placed on the far side at x=k, so every episode traverses the whole bar
    #    field. k grows epoch-proportionally (see class curriculum below): near goals first, then
    #    progressively deeper crossings as training proceeds.
    #  - Curriculum OFF: fall back to sampling the goal as a ratio of the environment bounds.
    class curriculum:
        use_curriculum = True
        # "Cross the bar field" scheme: the drone spawns at x~0 and the goal is placed at x=k on the
        # far side, so every episode must traverse the bars (x in ~[6.2, 22]). k grows with training
        # (epoch-proportional) so the drone learns to cross progressively deeper into the field.
        #   goal x ~ U[k_min, k_max(t)],  goal y ~ U[wall_margin, arena_y - wall_margin]
        #   k_max(t) = k_start + (k_final - k_start) * min(1, epoch / k_warmup_epochs)
        k_min = 5.0              # [m] initial nearest goal x (just before the bars -> easy anchor)
        k_min_final = 20.0       # [m] LATE nearest goal x. k_min ramps k_min -> k_min_final linearly
        #   over epochs [k_min_ramp_start_epochs, k_min_ramp_start_epochs + k_min_ramp_epochs],
        #   narrowing the goal window to deep crossings. Set k_min_final = k_min to disable.
        k_min_ramp_start_epochs = 2000  # epoch k_min STARTS rising. Independent of the k_max ramp --
        #   it overlaps it (k_max keeps ramping to k_warmup_epochs=3000 while k_min already climbs).
        k_min_ramp_epochs = 3000        # epochs to ramp k_min -> k_min_final => hits max at 2000+3000 = 5000
        full_scale_hold_epochs = 500   # (legacy) fallback k_min start = k_warmup+hold, only if
        #   k_min_ramp_start_epochs is unset. Ignored now that k_min_ramp_start_epochs is set.
        k_start = 7.0            # [m] initial k_max (first bar rows)
        k_final = 24.0           # [m] final k_max (far wall; goal x is clamped to arena_x - margin = 23.5)
        k_warmup_epochs = 3000   # epochs to ramp k_max from k_start to k_final (linear, then plateau)
        ppo_horizon = 32         # rl_games horizon_length (MUST match ppo_navrl_cnn.yaml) -> steps/epoch
        wall_margin = 0.5        # [m] keep drone/goal this far from the y walls

    # Phase 2 density sweep: obstacle size stays fixed; only the active bar count changes.
    # NAVRL_MAX_BARS controls the build-time ceiling in env_object_config.py.
    class density:
        use_density_curriculum = _env_bool("NAVRL_DENSITY_CURRICULUM", False)
        num_bars_active = _env_int("NAVRL_NUM_BARS", 48)
        n_start = 25
        n_final = 150
        success_threshold = 0.8
        promote_step = 15
        warmup_epochs = 2500
        check_after_episodes = 2048

    class eval:
        densities = [25, 50, 75, 110, 150]
        episodes_per_condition = 500
        seeds = [1, 2, 3, 4, 5]

    # Fallback goal placement (curriculum OFF): ratio of the environment bounds.
    target_min_ratio = [0.85, 0.10, 0.30]
    target_max_ratio = [0.95, 0.90, 0.70]

    # Success / termination
    # NavRL's own reach_goal condition is distance < 0.5 m (env.py:583) — an interception-
    # grade capture radius (two ~0.3 m quads with centers 0.5 m apart nearly touch), not a
    # loose "passed nearby" test. Also gates the goal-distance curriculum.
    success_radius = 0.5  # [m]
    # Interception semantics: touching the capture radius ends the episode as a success.
    # (Deliberate divergence from NavRL, whose navigation env never terminates on reach —
    # the 6000-epoch run showed the drone tags the goal then wanders, since the velocity
    # reward gives no incentive to stay: 82% timeouts with only 17% still at the goal.)
    terminate_on_capture = True
    # Keep the goal at least this far (XY) from the nearest bar so the capture sphere is
    # actually flyable; without it a goal can spawn flush against a bar.
    goal_min_bar_clearance = 1.0  # [m]
    lower_height_bound = 0.1  # [m] crash if below
    upper_height_bound = 4.0  # [m] crash if above (NavRL uses 4)

    # PBRS progress reward (A) discount -- MUST equal the PPO gamma (ppo_navrl*.yaml: gamma 0.99)
    # or the distance shaping stops being optimality-preserving.
    progress_gamma = 0.99

    # B/C clearance-penalty mode (only active when reward clearance_weight > 0):
    #   False -> plain proximity penalty (B): -clearance_weight * relu(margin - nearest_obstacle_dist)
    #   True  -> speed x proximity (C): also multiply by |velocity|, so only FAST approaches near
    #            obstacles are punished (agility in open space is untouched).
    # Run D RESULT (run ppo_260714_2207): speed-gated cw=6 SLOWED the drone 32% (ep_len 88->116) and
    # added a few timeouts but did NOT reduce crash (0.35->0.32). Conclusion: the residual crash is
    # geometric/perception (corner-clip = gap-centering precision), NOT reward-shaving, so no reward
    # weight fixes it. Clearance is OFF again (clearance_weight=0) for a clean Phase-2 baseline; the
    # real levers if we revisit are perception (LiDAR 36->72 beams) or yaw control. See CRASH_TUNING_LOG.md.
    clearance_speed_gated = False

    # Reward weights. NavRL's static branch (env.py) is:
    #   r = 1*reward_vel + 1(alive) + 1*r_safety_static - 0.1*penalty_smooth - 8*penalty_height
    #
    # NavRL keeps a constant +1 "alive" survival bonus (present in code, absent from paper
    # Eqn. 7) so that flying stays net-positive even when the safety/height terms go negative
    # near obstacles — the classic antidote to a "suicidal agent" that would crash early to
    # stop accumulating negative reward. It is safe there because NavRL's ONLY terminations are
    # bad (collision / out-of-bounds); reaching the goal is never a termination (env.py:583-587
    # feeds reach_goal to stats only). Once we terminate on capture (terminate_on_capture=True),
    # a positive per-step alive bonus flips from protective to harmful: ending the episode early
    # forfeits ~100 remaining steps of (alive + safety + vel) reward (hundreds of points) for a
    # small capture bonus, so the agent learns to loiter just outside the capture radius instead
    # of entering it (observed as ~10% capture). Fix (Option A): replace the survival bonus with
    # a small per-step time cost so that reaching the goal quickly is strictly optimal, and raise
    # the terminal capture bonus to cover the forfeited future reward.
    reward_parameters = {
        "vel_weight": 1.0,
        "alive_weight": -0.05,  # time cost per step (was +1 survival bonus; see note above)
        # A(crash): raised 1.0 -> 1.5 so obstacle clearance is valued more relative to the
        # velocity-toward-goal reward (the drone was shaving bars while rushing to far 18 m goals
        # -> ~14% crashes). The log-distance gradient is unchanged, only its weight.
        "safety_static_weight": 1.5,
        "smooth_weight": 0.1,
        "height_weight": 8.0,
        "height_margin": 0.2,  # NavRL's +/-0.2 m tolerance band
        # A: PBRS progress reward weight -- dense per-step "got closer to the goal" gradient
        # (reward = progress_weight*(prev_dist - progress_gamma*dist)). Optimality-preserving and
        # bounded ~progress_weight*(v_max*dt + (1-gamma)*d_max) per step. Set 0.0 to disable.
        "progress_weight": 1.0,
        # B3: raised -10 -> -20 so a no-capture episode (~ -0.05*250 = -12.5 once B1 removes the
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
        # B/C(crash): near-obstacle clearance penalty (DEFAULT OFF). Set clearance_weight > 0 (try
        # 1.5) to penalize being within clearance_margin of the nearest bar -- a firmer collision
        # buffer than the gentle log safety term. clearance_speed_gated (above) picks B vs C mode.
        # OFF (0.0). B(1.5) / C(1.5 speed-gated) / D(6.0 speed-gated) ALL failed to reduce crash --
        # D even slowed the drone 32% yet crash held at ~0.32, proving the residual crash is geometric
        # (corner-clip), not reward-driven. Kept here (with the live code path) as a documented dead
        # end; the real levers are perception (LiDAR 36->72 beams) or yaw control. See CRASH_TUNING_LOG.md.
        "clearance_weight": 0.0,
        # D: 0.6 -> 0.5. Calibrated to the worst gap: two ~0.8 m AXIS-ALIGNED bars (bars do NOT rotate)
        # at the 1.8 m min centre spacing leave a ~1.0 m free gap -> a CENTERED pass reads min_dist
        # ~0.5 m -> relu(0.5-0.5)=0 -> zero added cost (byte-identical to the 1904 reward on normal
        # passes). Only off-centre/shave passes (d < 0.5) are taxed, and only if fast (speed-gated).
        "clearance_margin": 0.5,       # [m] start penalizing inside this distance to the nearest bar
        # --- C1 finish-funnel params (DISABLED). Uncomment these together with the funnel block in
        #     navrl_task.py to reward closing through the 0.5-1.0 m shell just outside capture.
        # "funnel_coef": 1.0,
        # "funnel_outer": 1.0,   # [m] outer radius of the funnel shell
        # "funnel_width": 0.5,   # [m] shell thickness (outer_radius - capture_radius)
    }
