"""Launch the ROS 2 simulation-side AirMouse nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("nidar_airmouse_sim"), "config", "airmouse.yaml"
    )
    config = LaunchConfiguration("config")
    auto_start = LaunchConfiguration("auto_start")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="AirMouse simulation parameter file",
            ),
            DeclareLaunchArgument("auto_start", default_value="false"),
            Node(
                package="nidar_airmouse_sim",
                executable="mission-manager",
                name="nidar_airmouse_mission",
                parameters=[config, {"auto_start": auto_start}],
                output="screen",
            ),
            Node(
                package="nidar_airmouse_sim",
                executable="survivor-localizer",
                name="nidar_airmouse_survivor_localizer",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="nidar_airmouse_sim",
                executable="sim-survivor-detector",
                name="nidar_airmouse_sim_survivor_detector",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="nidar_airmouse_sim",
                executable="gcs-status",
                name="nidar_airmouse_gcs_status",
                parameters=[config],
                output="screen",
            ),
        ]
    )
