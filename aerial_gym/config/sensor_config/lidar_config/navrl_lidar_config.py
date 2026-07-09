from aerial_gym.config.sensor_config.lidar_config.base_lidar_config import (
    BaseLidarConfig,
)


class NavRLLidarConfig(BaseLidarConfig):
    """Ray-cast LiDAR matching NavRL's RayCaster (Xu et al., "NavRL", RA-L 2025).

    Reference (reference/NavRL/isaac-training/training/cfg/drone.yaml):
        lidar_range  = 4.0          # meters
        lidar_vfov   = [-10, 20]    # degrees
        lidar_vbeams = 4            # vertical beams
        lidar_hres   = 10           # degrees  -> 360/10 = 36 horizontal beams
    The RayCaster is attached yaw-only to the drone body.
    """

    # --- scan resolution: 36 horizontal x 4 vertical (NavRL lidar_hres=10, lidar_vbeams=4)
    width = 36  # horizontal beams
    height = 4  # vertical beams

    # --- vertical field of view: NavRL lidar_vfov = [-10, 20] -> beams at {-10, 0, 10, 20} deg
    vertical_fov_deg_min = -10.0
    vertical_fov_deg_max = 20.0

    # --- horizontal 360 deg.
    # Aerial Gym generates rays with inclusive endpoints (linspace over [min, max]). A full
    # [-180, 180] span at width=36 would duplicate the +/-180 ray and give 360/35 = 10.29 deg
    # spacing. Using a 350 deg span yields exactly 36 distinct beams at 10 deg, matching
    # NavRL's lidar_hres=10. The absolute azimuth offset is irrelevant (drone-relative scan).
    horizontal_fov_deg_min = -170.0
    horizontal_fov_deg_max = 180.0

    # --- range: NavRL lidar_range = 4.0 m
    max_range = 4.0
    min_range = 0.2

    # Out-of-range fill (recomputed here because the base class evaluated these against its own
    # max_range=10.0; with normalize_range=True a no-hit ray should map to 1.0 after dividing by
    # max_range, so the far fill must equal *this* max_range).
    normalize_range = True
    far_out_of_range_value = max_range  # -> 1.0 after normalization (no obstacle in that ray)
    near_out_of_range_value = -max_range

    # --- attach yaw-only: scan plane stays level under body roll/pitch (NavRL attach_yaw_only)
    yaw_only_attach = True

    # --- range image only; static-obstacle scan does not need segmentation
    return_pointcloud = False
    pointcloud_in_world_frame = False
    segmentation_camera = False

    # --- NavRL ray caster: sensor at body center, no placement randomization, no noise
    randomize_placement = False
    min_translation = [0.0, 0.0, 0.0]
    max_translation = [0.0, 0.0, 0.0]
    min_euler_rotation_deg = [0.0, 0.0, 0.0]
    max_euler_rotation_deg = [0.0, 0.0, 0.0]
    nominal_position = [0.0, 0.0, 0.0]
    nominal_orientation_euler_deg = [0.0, 0.0, 0.0]
    euler_frame_rot_deg = [0.0, 0.0, 0.0]

    class sensor_noise:
        enable_sensor_noise = False
        std_a = 0.0
        std_b = 0.0
        std_c = 0.0
        mean_offset = 0.0
        pixel_dropout_prob = 0.0
