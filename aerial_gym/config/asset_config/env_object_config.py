from aerial_gym import AERIAL_GYM_DIRECTORY

import os
import numpy as np

THIN_SEMANTIC_ID = 1
TREE_SEMANTIC_ID = 2
OBJECT_SEMANTIC_ID = 3
PANEL_SEMANTIC_ID = 20
FRONT_WALL_SEMANTIC_ID = 9
BACK_WALL_SEMANTIC_ID = 10
LEFT_WALL_SEMANTIC_ID = 11
RIGHT_WALL_SEMANTIC_ID = 12
BOTTOM_WALL_SEMANTIC_ID = 13
TOP_WALL_SEMANTIC_ID = 14


def _env_int(name, default):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return int(default)
    try:
        return int(raw)
    except ValueError:
        return int(default)


class asset_state_params:
    num_assets = 1  # number of assets to include

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets"
    file = None  # if file=None, random assets will be selected. If not None, this file will be used

    min_position_ratio = [0.5, 0.5, 0.5]  # min position as a ratio of the bounds
    max_position_ratio = [0.5, 0.5, 0.5]  # max position as a ratio of the bounds

    collision_mask = 1

    disable_gravity = False
    replace_cylinder_with_capsule = (
        True  # replace collision cylinders with capsules, leads to faster/more stable simulation
    )
    flip_visual_attachments = True  # Some .obj meshes must be flipped from y-up to z-up
    density = 0.001
    angular_damping = 0.1
    linear_damping = 0.1
    max_angular_velocity = 100.0
    max_linear_velocity = 100.0
    armature = 0.001

    collapse_fixed_joints = True
    fix_base_link = True
    specific_filepath = None  # if not None, use this folder instead randomizing
    color = None
    keep_in_env = False

    body_semantic_label = 0
    link_semantic_label = 0
    per_link_semantic = False
    semantic_masked_links = {}
    place_force_sensor = False
    force_sensor_parent_link = "base_link"
    force_sensor_transform = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]  # position, quat x, y, z, w

    use_collision_mesh_instead_of_visual = False


class panel_asset_params(asset_state_params):
    num_assets = 3

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/panels"

    collision_mask = 1  # objects with the same collision mask will not collide

    min_position_ratio = [0.3, 0.05, 0.05]  # max position as a ratio of the bounds
    max_position_ratio = [0.85, 0.95, 0.95]  # min position as a ratio of the bounds

    specified_position = [
        -1000.0,
        -1000.0,
        -1000.0,
    ]  # if > -900, use this value instead of randomizing   the ratios

    min_euler_angles = [0.0, 0.0, -np.pi / 3.0]  # min euler angles
    max_euler_angles = [0.0, 0.0, np.pi / 3.0]  # max euler angles

    min_state_ratio = [
        0.3,
        0.05,
        0.05,
        0.0,
        0.0,
        -np.pi / 3.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.85,
        0.95,
        0.95,
        0.0,
        0.0,
        np.pi / 3.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    keep_in_env = True

    collapse_fixed_joints = True
    per_link_semantic = False
    semantic_id = -1  # will be assigned incrementally per instance
    color = [170, 66, 66]


class tile_asset_params(asset_state_params):
    num_assets = 1

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/tile_meshes"

    collision_mask = 1  # objects with the same collision mask will not collide

    min_position_ratio = [0.3, 0.05, 0.05]  # max position as a ratio of the bounds
    max_position_ratio = [0.85, 0.95, 0.95]  # min position as a ratio of the bounds

    specified_position = [
        -1000.0,
        -1000.0,
        -1000.0,
    ]  # if > -900, use this value instead of randomizing   the ratios

    min_euler_angles = [0.0, 0.0, 0.0]  # min euler angles
    max_euler_angles = [0.0, 0.0, 0.0]  # max euler angles

    min_state_ratio = [
        0.5,
        0.5,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.5,
        0.5,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    keep_in_env = True

    collapse_fixed_joints = True
    per_link_semantic = False
    semantic_id = -1  # will be assigned incrementally per instance
    # color = [170, 66, 66]


class thin_asset_params(asset_state_params):
    num_assets = 0

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/thin"

    collision_mask = 1  # objects with the same collision mask will not collide

    min_state_ratio = [
        0.3,
        0.05,
        0.05,
        -np.pi,
        -np.pi,
        -np.pi,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.85,
        0.95,
        0.95,
        np.pi,
        np.pi,
        np.pi,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    collapse_fixed_joints = True
    per_link_semantic = False
    semantic_id = -1  # will be assigned incrementally per instance
    color = [170, 66, 66]


class tree_asset_params(asset_state_params):
    num_assets = 1

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/trees"

    collision_mask = 1  # objects with the same collision mask will not collide

    min_state_ratio = [
        0.1,
        0.1,
        0.0,
        0,
        -np.pi / 6.0,
        -np.pi,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.9,
        0.9,
        0.0,
        0,
        np.pi / 6.0,
        np.pi,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    collapse_fixed_joints = True
    per_link_semantic = True
    keep_in_env = True

    semantic_id = -1  # TREE_SEMANTIC_ID
    color = [70, 200, 100]

    semantic_masked_links = {}


class object_asset_params(asset_state_params):
    num_assets = 35

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/objects"

    min_state_ratio = [
        0.30,
        0.05,
        0.05,
        -np.pi,
        -np.pi,
        -np.pi,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.85,
        0.9,
        0.9,
        np.pi,
        np.pi,
        np.pi,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    keep_in_env = False
    per_link_semantic = False
    semantic_id = -1  # will be assigned incrementally per instance

    # color = [80,255,100]


class left_wall(asset_state_params):
    num_assets = 1

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/walls"
    file = "left_wall.urdf"

    collision_mask = 1  # objects with the same collision mask will not collide

    min_state_ratio = [
        0.5,
        1.0,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.5,
        1.0,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    keep_in_env = True

    collapse_fixed_joints = True
    specific_filepath = "cube.urdf"
    per_link_semantic = False
    semantic_id = LEFT_WALL_SEMANTIC_ID
    color = [100, 200, 210]


class right_wall(asset_state_params):
    num_assets = 1

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/walls"
    file = "right_wall.urdf"

    min_state_ratio = [
        0.5,
        0.0,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.5,
        0.0,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    keep_in_env = True

    collapse_fixed_joints = True
    per_link_semantic = False
    specific_filepath = "cube.urdf"
    semantic_id = RIGHT_WALL_SEMANTIC_ID
    color = [100, 200, 210]


class top_wall(asset_state_params):
    num_assets = 1

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/walls"
    file = "top_wall.urdf"

    collision_mask = 1  # objects with the same collision mask will not collide

    min_state_ratio = [
        0.5,
        0.5,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.5,
        0.5,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    keep_in_env = True

    collapse_fixed_joints = True
    specific_filepath = "cube.urdf"
    per_link_semantic = False
    semantic_id = TOP_WALL_SEMANTIC_ID
    color = [100, 200, 210]


class bottom_wall(asset_state_params):
    num_assets = 1
    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/walls"
    file = "bottom_wall.urdf"

    collision_mask = 1  # objects with the same collision mask will not collide

    min_state_ratio = [
        0.5,
        0.5,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.5,
        0.5,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    keep_in_env = True

    collapse_fixed_joints = True
    specific_filepath = "cube.urdf"
    per_link_semantic = False
    semantic_id = BOTTOM_WALL_SEMANTIC_ID
    color = [100, 150, 150]


class front_wall(asset_state_params):
    num_assets = 1

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/walls"
    file = "front_wall.urdf"

    collision_mask = 1  # objects with the same collision mask will not collide

    min_state_ratio = [
        1.0,
        0.5,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        1.0,
        0.5,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    keep_in_env = True

    collapse_fixed_joints = True
    specific_filepath = "cube.urdf"
    per_link_semantic = False
    semantic_id = FRONT_WALL_SEMANTIC_ID
    color = [100, 200, 210]


class back_wall(asset_state_params):
    num_assets = 1

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/walls"
    file = "back_wall.urdf"

    collision_mask = 1  # objects with the same collision mask will not collide

    min_state_ratio = [
        0.0,
        0.5,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.0,
        0.5,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    keep_in_env = True

    collapse_fixed_joints = True
    specific_filepath = "cube.urdf"
    per_link_semantic = False
    semantic_id = BACK_WALL_SEMANTIC_ID
    color = [100, 200, 210]


INTERCEPT_TARGET_SEMANTIC_ID = 50


class intercept_quad_target_params(asset_state_params):
    """
    Intercept target: center collision sphere R=0.05 m only (no arm geometry).
    Matches chaser `quad/quad.urdf` base_link collision sphere and
    `intercept_success_radius` (center distance ≤ 0.11 m at first touch).
    """

    num_assets = 1
    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/objects"
    file = "intercept_target_quad_scale.urdf"
    collision_mask = 0
    disable_gravity = True
    density = 0.000001
    replace_cylinder_with_capsule = False  # align with flying quad robot_asset
    flip_visual_attachments = True
    collapse_fixed_joints = False  # align with quad.urdf as used by base_quadrotor
    fix_base_link = True
    keep_in_env = True
    per_link_semantic = False
    semantic_id = INTERCEPT_TARGET_SEMANTIC_ID
    color = [220, 60, 60]
    # Offset from center of empty_env cube (see EmptyEnvCfg bounds ± env_spacing)
    min_state_ratio = [
        0.72,
        0.48,
        0.52,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.72,
        0.48,
        0.52,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]


class navrl_target_params(asset_state_params):
    """NavRL Phase-3 moving target drone (semantic_id=50).

    A single drone-sized body the pursuer must LOCATE from its onboard sensors (no ground-truth
    position). It is registered as an env asset so its mesh is baked into the raycastable warp
    scene and shows up in the LiDAR/camera SEGMENTATION channel as id=50 (bars are id=3). It is
    kept_in_env (so it lands at obstacle index 0 -- the task slices obstacle_position[:, 1:] for
    bars and drives obstacle_position[:, 0] itself each step) and never deactivated.

    fix_base_link + disable_gravity + collision_mask=0: the body is teleported kinematically by
    NavRLTask (no physics dynamics, no crash on contact -- capture stays the geometric point test).

    The XY/Z state ratio is DELIBERATELY the same as bar_asset_params: AssetManager._placement_band
    reads asset index 0's ratio as the placement band for ALL obstacles, so a divergent ratio here
    would collapse the bar-scatter band. The initial pose is overwritten by the task at reset
    anyway, so the sampled placement is irrelevant -- only the band-protection matters.
    """

    num_assets = 1
    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/objects"
    file = "navrl_target_drone.urdf"

    collision_mask = 0
    disable_gravity = True
    density = 0.000001
    replace_cylinder_with_capsule = False
    flip_visual_attachments = True
    collapse_fixed_joints = False
    fix_base_link = True
    keep_in_env = True
    per_link_semantic = False
    semantic_id = INTERCEPT_TARGET_SEMANTIC_ID  # 50
    color = [220, 40, 40]

    # Same band as bar_asset_params (see docstring): x in [0.13, 0.96], y in [0, 1], z = 1/3.
    min_state_ratio = [0.13, 0.0, 0.3333, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    max_state_ratio = [0.96, 1.0, 0.3333, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class bar_asset_params(asset_state_params):
    """Variable-size vertical obstacle bars ("막대") for the controlled NavRL bars environment.

    Bars are randomly picked (file=None) from a pool of generated box URDFs whose horizontal
    footprint (width, depth) is random in [0.4, 0.8] m and whose height is fixed at 2 m
    (see resources/models/environment_assets/bars, generated by tools/generate_bar_assets.py).
    Each box is centered at its origin, so the z-ratio puts the center at 1 m => the bar spans
    0..2 m in the ground-referenced [0, 3] m arena. num_assets bars are scattered in a central
    band so the drone (which spawns near the low-x edge and flies at 1 m altitude) must weave
    through them to reach nearby goals. No walls, no panels -- an otherwise empty space.

    Non-overlap and a minimum inter-bar gap are enforced separately by the environment via
    NavRLBarsEnvCfg.env.min_obstacle_xy_spacing (a minimum center-to-center XY distance).
    """

    # Build-time ceiling for the density sweep. Runtime active bars are controlled by
    # task_config.density / NAVRL_NUM_BARS; inactive bars are moved to -1000 by AssetManager.
    num_assets = max(0, _env_int("NAVRL_MAX_BARS", 150))

    # Bar pool selection. "bars" = legacy 2.0 m pool (fly-over possible in a 3 m arena);
    # "bars_h3" = full-height 3.0 m pool (v2 search arena: removes the fly-over option,
    # matching NavRL's mostly-unoverflyable obstacles). Same footprints/seed either way, so
    # this changes ONLY the height. The bar z-ratio must put the box center at height/2:
    # 2 m pool -> 1.0 m (ratio 1/3 in a 3 m arena), 3 m pool -> 1.5 m (ratio 1/2).
    _bar_pool = (os.environ.get("NAVRL_BAR_POOL", "").strip() or "bars")
    _bar_z_ratio = 0.5 if _bar_pool == "bars_h3" else 0.3333

    asset_folder = f"{AERIAL_GYM_DIRECTORY}/resources/models/environment_assets/{_bar_pool}"
    file = None  # random pick from the pool -> each bar gets a random footprint

    collision_mask = 1  # bars do not collide with each other

    # [x_ratio, y_ratio, z_ratio, roll, pitch, yaw, 1.0, vx,vy,vz, wx,wy,wz]
    # x band starts past the drone spawn strip (drone x-ratio ~0, x <~ 1); z ratio places the box
    # center at bar_height/2 (pool-dependent, see _bar_z_ratio) so bars stand on the floor; upright.
    min_state_ratio = [
        0.13,
        0.0,
        _bar_z_ratio,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    max_state_ratio = [
        0.96,
        1.0,
        _bar_z_ratio,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    keep_in_env = False

    collapse_fixed_joints = True
    per_link_semantic = False
    semantic_id = OBJECT_SEMANTIC_ID
    color = [170, 66, 66]
