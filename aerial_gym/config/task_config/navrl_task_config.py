import torch


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
    # Controlled Phase-1 arena: empty space + 48 static bars (no walls/panels). See navrl_bars_env.py.
    env_name = "navrl_bars_env"
    robot_name = "navrl_quad"
    controller_name = "lee_velocity_control"
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
    internal_state_dim = 8
    observation_space_dim = internal_state_dim + lidar_hbeams * lidar_vbeams
    privileged_observation_space_dim = 0

    # Action = 3D velocity in the goal frame (NavRL). Yaw is left uncommanded (held at reset heading).
    action_space_dim = 3

    episode_len_steps = 300  # RL steps (300 for the far-side goals: a diagonal goal can be ~30 m,
    # ~150 steps straight / ~225 weaving at 2 m/s, so 300 keeps timeout from being the failure mode)

    return_state_before_reset = False

    max_velocity = 2.0  # NavRL v_lim [m/s]

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
        k_min_final = 20.0       # [m] LATE nearest goal x. Schedule after k_max plateaus (at epoch
        #   k_warmup_epochs): (B) HOLD full scale [k_min, k_max] for full_scale_hold_epochs, THEN
        #   (C) ramp k_min k_min->k_min_final over k_min_ramp_epochs (narrows to deep goals).
        #   Set k_min_final = k_min to disable the narrowing.
        full_scale_hold_epochs = 500   # (B) epochs held at full range [k_min, k_max] before k_min rises
        k_min_ramp_epochs = 2500       # (C) epochs to ramp k_min after the hold
        k_start = 7.0            # [m] initial k_max (first bar rows)
        k_final = 24.0           # [m] final k_max (far wall; goal x is clamped to arena_x - margin = 23.5)
        k_warmup_epochs = 3000   # epochs to ramp k_max from k_start to k_final (linear, then plateau)
        ppo_horizon = 32         # rl_games horizon_length (MUST match ppo_navrl_cnn.yaml) -> steps/epoch
        wall_margin = 0.5        # [m] keep drone/goal this far from the y walls

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
    # D(crash, 2026-07-14): speed-gated ON. In the NEW cross-field scheme every episode crosses the
    # 48-bar field with the 0.28 m box, so crashes are dominated by "shaving a bar at cruise speed"
    # (all 3 diagnostic lenses converge on this). Speed-gating scales the clearance penalty by |v|, so
    # only FAST approaches near a bar are punished -> the drone SLOWS through tight gaps rather than
    # detouring/freezing (open-lane agility untouched). See CRASH_TUNING_LOG.md run D.
    clearance_speed_gated = True

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
        # B/C(crash): near-obstacle clearance penalty (DEFAULT OFF). Set clearance_weight > 0 (try
        # 1.5) to penalize being within clearance_margin of the nearest bar -- a firmer collision
        # buffer than the gentle log safety term. clearance_speed_gated (above) picks B vs C mode.
        # D: 0.0 -> 6.0. The prior null runs (B/C at 1.5) failed because the penalty (~0.75/step at a
        # 0.30 m shave) was WEAKER than the +2/step velocity reward, so shaving stayed net-positive.
        # At cw=6, speed-gated, a 0.30 m shave at 2 m/s costs 6*relu(0.5-0.30)*2 = 2.40/step > 2.0, so
        # the effective velocity coefficient (1 - 6*relu(0.5-d)) goes NEGATIVE inside 0.333 m: fast-
        # toward-a-close-bar is now punished, not merely offset. ~4x the failed cw=1.5 at the shave pt.
        "clearance_weight": 6.0,
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
