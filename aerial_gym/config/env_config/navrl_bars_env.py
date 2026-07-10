from aerial_gym.config.asset_config.env_object_config import bar_asset_params


class NavRLBarsEnvCfg:
    """Controlled Phase-1 arena: an otherwise-empty space with a field of static bars.

    Deliberately minimal (the "16 static bars" smoke test): no walls, no panels, no random
    clutter -- just N vertical bars the drone must fly through to reach a nearby goal.

    Geometry is ground-referenced (z = height above the floor), so it matches the navrl_task
    height bounds (0.1..4.0 m). The drone spawns near the low-x edge (robot init x-ratio ~0.1-0.2)
    and the goal is placed 2.5-5 m away by the curriculum, i.e. into/through the bar field.
    A literal 2x2 m box is intentionally NOT used: it is smaller than the 4 m LiDAR range and the
    2.5-5 m goal distances, which would make the task degenerate. Instead the "2 m" is the bar
    spacing -- 16 bars scattered across a ~10x10 m footprint (~2 m apart) form the field.
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
        # center-to-center distance. Bars are up to 0.8 m wide, so a 1.3 m center distance
        # leaves >= 0.5 m clear gap even between two largest-size neighbours (1.3 - 0.4 - 0.4).
        min_obstacle_xy_spacing = 1.3

        # Fixed 10 x 10 x 3 m arena (min == max so every env is identical). Ground-referenced:
        # z in [0, 3]. Drone init z-ratio 0.15-0.85 -> z in [0.45, 2.55], safely inside [0.1, 4.0].
        lower_bound_min = [-1.0, -5.0, 0.0]
        lower_bound_max = [-1.0, -5.0, 0.0]
        upper_bound_min = [9.0, 5.0, 3.0]
        upper_bound_max = [9.0, 5.0, 3.0]

    class env_config:
        # Only bars. Everything else (walls, panels, objects, trees) is OFF.
        include_asset_type = {
            "bars": True,
        }

        asset_type_to_dict_map = {
            "bars": bar_asset_params,
        }
