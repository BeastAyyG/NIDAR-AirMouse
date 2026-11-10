"""Start Gazebo Sim with an AirMouse world plus the gz -> ROS bridge.

Usage:
    ros2 launch airmouse_sim sim_bringup.launch.py
    ros2 launch airmouse_sim sim_bringup.launch.py world:=l_corridor.sdf

`world` may be a bare filename resolved inside the installed
share/airmouse_sim/worlds directory, or an absolute path to any SDF world.
The bridge configuration lives in airmouse_sensors/config/gz_bridge.yaml.

Note: this launch file starts Gazebo only.  PX4 SITL (when flying with the
PX4 controller) is launched separately, as in the existing setup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_world(context):
    """Resolve the `world` argument to an absolute path."""
    world = LaunchConfiguration('world').perform(context)
    if os.path.isabs(world):
        return world
    worlds_dir = os.path.join(
        get_package_share_directory('airmouse_sim'), 'worlds')
    return os.path.join(worlds_dir, world)


def _launch_setup(context, *args, **kwargs):
    world_path = _resolve_world(context)
    bridge_config = LaunchConfiguration('bridge_config').perform(context)

    gz_sim = ExecuteProcess(
        # -r runs the simulation immediately; ogre2 is required by rgbd_camera.
        cmd=['gz', 'sim', '-r', '--render-engine', 'ogre2', world_path],
        output='screen',
        name='gz_sim',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    return [gz_sim, bridge]


def generate_launch_description():
    models_dir = PathJoinSubstitution(
        [FindPackageShare('airmouse_sim'), 'models'])
    default_bridge = PathJoinSubstitution(
        [FindPackageShare('airmouse_sensors'), 'config', 'gz_bridge.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='full_mission.sdf',
            description='World filename (in share/airmouse_sim/worlds) or '
                        'an absolute path to an SDF world.',
        ),
        DeclareLaunchArgument(
            'bridge_config',
            default_value=default_bridge,
            description='ros_gz_bridge YAML config file.',
        ),
        # model:// URIs in the worlds must resolve to our shipped models.
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                models_dir,
                os.pathsep,
                EnvironmentVariable(
                    'GZ_SIM_RESOURCE_PATH', default_value=''),
            ],
        ),
        OpaqueFunction(function=_launch_setup),
    ])
