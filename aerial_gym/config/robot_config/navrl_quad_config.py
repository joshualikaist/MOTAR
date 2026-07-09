from aerial_gym.config.robot_config.base_quad_config import BaseQuadCfg
from aerial_gym.config.sensor_config.lidar_config.navrl_lidar_config import (
    NavRLLidarConfig,
)


class NavRLQuadCfg(BaseQuadCfg):
    """Base quadrotor equipped with the NavRL-style yaw-only ray-cast LiDAR (no camera).

    Static obstacles are perceived through a 36x4 range scan (see NavRLLidarConfig),
    matching the NavRL navigation environment. Requires use_warp=True; LiDAR rendering
    is not supported by Isaac Gym's native camera path.
    """

    class sensor_config(BaseQuadCfg.sensor_config):
        enable_camera = False  # NavRL static perception uses LiDAR, not a depth camera

        enable_lidar = True
        lidar_config = NavRLLidarConfig

        enable_imu = False
