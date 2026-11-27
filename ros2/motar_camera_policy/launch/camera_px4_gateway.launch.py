"""Explicit, independently configured PX4 gateway process; no launch on import."""


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    return LaunchDescription([
        DeclareLaunchArgument('source_root', description='Absolute MOTAR source tree'),
        DeclareLaunchArgument('config', description='Explicit PX4 bridge JSON; observation only by default'),
        Node(package='motar_camera_policy', executable='camera_px4_gateway', output='screen',
             arguments=['--source-root', LaunchConfiguration('source_root'),
                        '--config', LaunchConfiguration('config')]),
    ])
