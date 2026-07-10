import torch


class task_config:
    """Phase 1 NavRL reimplementation: static obstacles + stationary goal.

    Observation and reward follow NavRL (Xu et al., RA-L 2025), static-only branch:
      state  = S_int (goal-frame internal state, 8) concatenated with the flattened
               36x4 LiDAR scan (see NavRLLidarConfig / navrl_quad robot).
      action = 3D velocity command in the goal frame, scaled to +/- max_velocity.
      reward = vel + alive + static-safety - smooth - height  (NavRL weights).
    Dynamic obstacles and the moving target are added in later phases.
    """

    seed = 42
    sim_name = "base_sim"
    # Controlled Phase-1 arena: empty space + 16 static bars (no walls/panels). See navrl_bars_env.py.
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

    episode_len_steps = 150  # RL steps; wall-clock = this * physics_steps_per_step * sim.dt

    return_state_before_reset = False

    max_velocity = 2.0  # NavRL v_lim [m/s]

    # 2D navigation: the drone flies at this fixed altitude and tracks the goal in XY only.
    # The drone spawns here (navrl_quad init z-ratio) and the task zeroes the vertical velocity
    # command; the goal is placed at this same altitude. (A future fleeing target also moves in XY.)
    flight_altitude = 1.0  # [m]

    # Stationary goal placement.
    #  - Curriculum ON (default): the goal is sampled at a random horizontal direction and a
    #    distance in [goal_dist_min, cur_max] from the spawn, starting easy (nearby goals) and
    #    expanding as the reach rate improves. This is the fix for the "goal too far, 0% reached"
    #    result of the first training run -- get the simple static case succeeding first.
    #  - Curriculum OFF: fall back to sampling the goal as a ratio of the environment bounds.
    class curriculum:
        use_curriculum = True
        goal_dist_min = 2.5          # [m] nearest goal
        goal_dist_start = 5.0        # [m] initial max goal distance (easy)
        goal_dist_max = 18.0         # [m] final max goal distance
        expand_step = 1.5            # [m] added to the max distance per successful check
        reach_rate_to_expand = 0.60  # ever_reached rate needed to expand difficulty
        check_after_episodes = 4096  # evaluate the reach rate over this many finished episodes
        goal_height_jitter = 1.0     # [m] goal z = spawn z +/- U(this), clamped to height bounds

    # Fallback goal placement (curriculum OFF): ratio of the environment bounds.
    target_min_ratio = [0.85, 0.10, 0.30]
    target_max_ratio = [0.95, 0.90, 0.70]

    # Success / termination
    success_radius = 1.0  # [m]
    lower_height_bound = 0.1  # [m] crash if below
    upper_height_bound = 4.0  # [m] crash if above (NavRL uses 4)

    # NavRL static-branch reward weights (env.py):
    #   r = 1*reward_vel + 1(alive) + 1*r_safety_static - 0.1*penalty_smooth - 8*penalty_height
    reward_parameters = {
        "vel_weight": 1.0,
        "alive_weight": 1.0,
        "safety_static_weight": 1.0,
        "smooth_weight": 0.1,
        "height_weight": 8.0,
        "height_margin": 0.2,  # NavRL's +/-0.2 m tolerance band
        # NavRL leaves the terminal collision penalty commented out; a modest value is used here
        # to discourage crashing while training the skeleton. Tune/remove in ablations.
        "collision_penalty": -10.0,
    }
