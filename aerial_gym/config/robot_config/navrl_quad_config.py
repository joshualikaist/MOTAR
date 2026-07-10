import numpy as np

from aerial_gym.config.robot_config.base_quad_config import BaseQuadCfg
from aerial_gym.config.sensor_config.lidar_config.navrl_lidar_config import (
    NavRLLidarConfig,
)


class NavRLQuadCfg(BaseQuadCfg):
    """Base quadrotor equipped with the NavRL-style yaw-only ray-cast LiDAR (no camera).

    Static obstacles are perceived through a 36x4 range scan (see NavRLLidarConfig),
    matching the NavRL navigation environment. Requires use_warp=True; LiDAR rendering
    is not supported by Isaac Gym's native camera path.

    For the Phase-1 bars environment the drone flies in 2D at a fixed altitude: it spawns at
    1 m (z-ratio 1/3 of the [0, 3] m arena) with no initial vertical velocity, and navrl_task
    zeroes the vertical velocity command so it stays at 1 m and tracks the goal in XY only.
    """

    class init_config(BaseQuadCfg.init_config):
        # [ratio_x, ratio_y, ratio_z, roll, pitch, yaw, 1.0, vx, vy, vz, wx, wy, wz]
        # z-ratio fixed at 1/3 -> spawn altitude = 1.0 m; initial vz = 0 (index 9).
        min_init_state = [
            0.1, 0.15, 0.3333, 0.0, 0.0, -np.pi / 6, 1.0, -0.2, -0.2, 0.0, -0.2, -0.2, -0.2,
        ]
        max_init_state = [
            0.2, 0.85, 0.3333, 0.0, 0.0, np.pi / 6, 1.0, 0.2, 0.2, 0.0, 0.2, 0.2, 0.2,
        ]

    class sensor_config(BaseQuadCfg.sensor_config):
        enable_camera = False  # NavRL static perception uses LiDAR, not a depth camera

        enable_lidar = True
        lidar_config = NavRLLidarConfig

        enable_imu = False
