"""Start the ros_gz_bridge with the AirMouse sensor mapping.

Usage:
    ros2 launch airmouse_sensors sensors.launch.py
    ros2 launch airmouse_sensors sensors.launch.py \
        bridge_config:=/abs/path/to/gz_bridge.yaml

`bridge_config` may be a bare filename resolved inside the installed
share/airmouse_sensors/config directory or an absolute path.

This launch file only starts the bridge; Gazebo is launched by
airmouse_sim (sim_bringup.launch.py already starts the same bridge) and PX4
SITL is launched separately.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_config(context):
    config = LaunchConfiguration('bridge_config').perform(context)
    if os.path.isabs(config):
        return config
    config_dir = os.path.join(
        get_package_share_directory('airmouse_sensors'), 'config')
    return os.path.join(config_dir, config)


def _launch_setup(context, *args, **kwargs):
    bridge_config = _resolve_config(context)
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_config',
            default_value='gz_bridge.yaml',
            description='Bridge YAML: filename in share/airmouse_sensors/'
                        'config or an absolute path.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
