"""Launch the ROS 2 simulation-side AirMouse nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# World name -> parameter file that ships with the package.
WORLD_CONFIGS = {
    "maze": "airmouse.yaml",
    "maze_compete": "airmouse_compete.yaml",
}

# Sentinel so an explicit config:= always wins over the world default.
AUTO_CONFIG = "auto"

NODES = (
    ("mission-manager", "nidar_airmouse_mission"),
    ("lidar-safety", "nidar_airmouse_lidar_safety"),
    ("survivor-localizer", "nidar_airmouse_survivor_localizer"),
    ("sim-survivor-detector", "nidar_airmouse_sim_survivor_detector"),
    ("gcs-status", "nidar_airmouse_gcs_status"),
)


def resolve_config_for_world(world):
    """Return the installed parameter path for *world*, or raise if unknown."""
    share = get_package_share_directory("nidar_airmouse_sim")
    try:
        filename = WORLD_CONFIGS[world]
    except KeyError:
        raise RuntimeError(
            "Unknown world '%s'. Known worlds: %s"
            % (world, ", ".join(sorted(WORLD_CONFIGS)))
        ) from None
    return os.path.join(share, "config", filename)


def _launch_nodes(context):
    world = LaunchConfiguration("world").perform(context)
    config = LaunchConfiguration("config").perform(context)
    auto_start = LaunchConfiguration("auto_start").perform(context)

    if config == AUTO_CONFIG:
        config = resolve_config_for_world(world)

    nodes = []
    for executable, node_name in NODES:
        parameters = [config]
        if executable == "mission-manager":
            parameters.append({"auto_start": auto_start})
        nodes.append(
            Node(
                package="nidar_airmouse_sim",
                executable=executable,
                name=node_name,
                parameters=parameters,
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="maze",
                description=(
                    "Arena to fly: 'maze' (practice course, 30x30 m) or "
                    "'maze_compete' (competition course, 15x15 m). Selects the "
                    "matching parameter file unless config:= is given."
                ),
            ),
            DeclareLaunchArgument(
                "config",
                default_value=AUTO_CONFIG,
                description=(
                    "AirMouse simulation parameter file. Defaults to the file "
                    "matching the 'world' argument."
                ),
            ),
            DeclareLaunchArgument("auto_start", default_value="false"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
