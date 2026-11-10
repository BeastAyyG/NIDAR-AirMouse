"""Full AirMouse mission: Gazebo world + bridge + every autonomy node.

Usage:
    ros2 launch airmouse_sim mission.launch.py
    ros2 launch airmouse_sim mission.launch.py world:=full_maze_15x15.sdf

Includes sim_bringup.launch.py (Gazebo Sim + ros_gz_bridge) and
autonomy_bringup.launch.py (mapping, perception, survivor, exploration,
navigation, mission, localization nodes).  PX4 SITL is launched separately
when flying with the PX4 controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_launch = PathJoinSubstitution(
        [FindPackageShare('airmouse_sim'), 'launch', 'sim_bringup.launch.py'])
    autonomy_launch = PathJoinSubstitution(
        [FindPackageShare('airmouse_sim'), 'launch',
         'autonomy_bringup.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='full_mission.sdf',
            description='World filename in share/airmouse_sim/worlds.',
        ),
        DeclareLaunchArgument(
            'bridge_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('airmouse_sensors'), 'config',
                 'gz_bridge.yaml']),
            description='ros_gz_bridge YAML config file.',
        ),
        DeclareLaunchArgument(
            'config',
            default_value='config/mission.yaml',
            description='Mission YAML path relative to the launch cwd.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'bridge_config': LaunchConfiguration('bridge_config'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autonomy_launch),
            launch_arguments={
                'config': LaunchConfiguration('config'),
            }.items(),
        ),
    ])
