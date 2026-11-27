"""Explicit paths are required; importing this file does not launch a ROS graph."""


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    return LaunchDescription([
        DeclareLaunchArgument('source_root', description='Absolute MOTAR source tree to execute'),
        DeclareLaunchArgument('config', description='Measured and source-bound camera-policy JSON configuration'),
        Node(
            package='motar_camera_policy',
            executable='camera_policy',
            output='screen',
            arguments=['--source-root', LaunchConfiguration('source_root'),
                       '--config', LaunchConfiguration('config')],
        ),
    ])
