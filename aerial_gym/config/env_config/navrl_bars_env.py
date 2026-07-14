from aerial_gym.config.asset_config.env_object_config import bar_asset_params


class NavRLBarsEnvCfg:
    """Controlled Phase-1 arena: an otherwise-empty space with a field of static bars.

    Deliberately minimal (a static-bars navigation field): no walls, no panels, no random
    clutter -- just N vertical bars the drone must fly through to reach the goal.

    Geometry is ground-referenced (z = height above the floor), so it matches the navrl_task
    height bounds (0.1..4.0 m). The drone spawns near the low-x edge (robot init x-ratio ~0.1-0.2)
    and the goal is placed 2.5 m up to the curriculum max (41 m) away, i.e. into/through the bar
    field. The arena is 24 x 24 m with 48 bars (~8 bars / 100 m^2) scattered across the interior
    band so the curriculum goals are reachable without being clamped to the walls.
    """

    class env:
        num_envs = 64  # overridden by the task/runner num_envs
        num_env_actions = 4
        env_spacing = 5.0  # not used with warp meshes, kept for parity

        num_physics_steps_per_env_step_mean = 10
        num_physics_steps_per_env_step_std = 0

        render_viewer_every_n_steps = 1
        reset_on_collision = True
        collision_force_threshold = 0.05  # [N]
        create_ground_plane = False  # no physical floor (matches env_with_obstacles); z=0 is floor level
        sample_timestep_for_latency = True
        perturb_observations = True
        keep_same_env_for_num_episodes = 1
        write_to_sim_at_every_timestep = False

        use_warp = True  # required for the warp LiDAR

        # Guarantee the bars never overlap and keep a clear gap between them: minimum XY
        # center-to-center distance. Bars are up to 0.8 m wide, so a 1.8 m center distance leaves
        # >= 1.0 m clear gap even between two largest neighbours -- comfortably flyable at this
        # larger arena scale.
        min_obstacle_xy_spacing = 1.8

        # Fixed 24 x 24 x 3 m arena (min == max so every env is identical). Ground-referenced:
        # z in [0, 3]. Enlarged from 10 x 10 so the goal-distance curriculum's goals are actually
        # reachable inside the arena (they were clamped to ~10 m in the old 10 x 10 box).
        # Drone spawns near the low-x edge (init x-ratio 0.1-0.2 -> x in [2.4, 4.8]); the goal is
        # placed up to the curriculum max (41 m, the integer part of the 24*sqrt(3) box diagonal)
        # away, well inside the [0, 24] x [0, 24] bounds.
        lower_bound_min = [0.0, 0.0, 0.0]
        lower_bound_max = [0.0, 0.0, 0.0]
        upper_bound_min = [24.0, 24.0, 3.0]
        upper_bound_max = [24.0, 24.0, 3.0]

    class env_config:
        # Only bars. Everything else (walls, panels, objects, trees) is OFF.
        include_asset_type = {
            "bars": True,
        }

        asset_type_to_dict_map = {
            "bars": bar_asset_params,
        }
