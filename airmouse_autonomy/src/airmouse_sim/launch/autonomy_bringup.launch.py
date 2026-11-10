"""Start every AirMouse autonomy node with parameters from config/.

Usage:
    ros2 launch airmouse_sim autonomy_bringup.launch.py
    ros2 launch airmouse_sim autonomy_bringup.launch.py \
        config:=/abs/path/to/airmouse_autonomy/config/mission.yaml

`config` is the mission YAML (default: config/mission.yaml, resolved against
the current working directory, i.e. the workspace root when launched with
`ros2 launch` from airmouse_autonomy/).  The sibling files mapping.yaml,
perception.yaml and survivor.yaml are loaded from the same directory.

The workspace YAML files are plain application configs (no ros__parameters
section), so this launch file converts each section into a flat ROS
parameter dict and passes it to the matching node.  Missing files or keys
fall back to the defaults embedded below, which mirror the config defaults
(cell_size 0.10 m, depth 0.2-8.0 m, confirm after 3 sightings, etc.).

Executables (one thin rclpy adapter per package):
    airmouse_mapping/mapping_node
    airmouse_perception/perception_node
    airmouse_survivor/survivor_node
    airmouse_exploration/exploration_node
    airmouse_navigation/navigation_node
    airmouse_mission/mission_node
    airmouse_localization/localization_node
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:  # PyYAML ships with the ROS 2 launch environment.
    import yaml
except ImportError:  # pragma: no cover - launch environment always has yaml
    yaml = None

MISSION_DEFAULTS = {
    'maze_width_m': 15.0,
    'maze_height_m': 15.0,
    'max_survivors': 6,
    'time_limit_sim_s': 1800.0,
    'replan_interval_s': 2.0,
    'goal_reach_radius_m': 0.4,
    'investigate_after_s': 4.0,
    'max_investigate_attempts': 2,
    'return_on_no_frontiers': True,
}

FSM_DEFAULTS = {
    'sensor_loss_timeout_s': 2.0,
    'loc_loss_timeout_s': 2.0,
    'comms_loss_timeout_s': 1.0,
    'hold_timeout_s': 10.0,
}

DRONE_DEFAULTS = {
    'speed_mps': 2.0,
    'footprint_radius_m': 0.10,
    'waypoint_pop_tolerance_m': 0.15,
    'stuck_watchdog_s': 3.0,
}

MAPPING_DEFAULTS = {
    'cell_size_m': 0.10,
    'width_m': 15.0,
    'height_m': 15.0,
    'max_range_m': 6.0,
    'clearance_m': 0.10,
    'min_frontier_cluster': 3,
    'goal_search_radius': 3,
    'flight_band_zmin_m': 0.3,
    'flight_band_zmax_m': 2.0,
}

PERCEPTION_DEFAULTS = {
    'depth_min_m': 0.2,
    'depth_max_m': 8.0,
    'floor_z_m': 0.15,
    'ceil_z_m': 2.2,
    'flight_band_zmin_m': 0.3,
    'flight_band_zmax_m': 2.0,
    'voxel_size_m': 0.05,
    'hfov_rad': 1.2,
    'regions': 3,
    'representative_percentile': 20,
}

SURVIVOR_DEFAULTS = {
    'detect_range_m': 5.0,
    'base_confidence': 0.85,
    'merge_radius_m': 1.2,
    'confirm_confidence': 0.85,
    'confirm_sightings': 3,
    'decay_per_sec': 0.02,
    'drop_confidence': 0.12,
    'position_noise_m': 0.35,
    'false_positive_rate': 0.03,
}


def _load_yaml(path):
    """Return the YAML mapping at `path`, or {} when absent/invalid."""
    if yaml is None:
        return {}
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError):
        return {}
    return data if isinstance(data, dict) else {}


def _section(document, section):
    """Return a copy of `document[section]` if it is a mapping, else {}."""
    value = document.get(section, {})
    if not isinstance(value, dict):
        return {}
    return dict(value)


def _params(defaults, *overrides):
    """Build a new parameter dict: defaults <- later sections (immutable)."""
    merged = dict(defaults)
    for override in overrides:
        merged.update(override)
    return merged


def _config_dir(config_path):
    if os.path.isabs(config_path):
        return os.path.dirname(config_path)
    return os.path.dirname(os.path.abspath(config_path))


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config').perform(context)
    config_dir = _config_dir(config_path)

    mission_doc = _load_yaml(config_path)
    mapping_doc = _load_yaml(os.path.join(config_dir, 'mapping.yaml'))
    perception_doc = _load_yaml(os.path.join(config_dir, 'perception.yaml'))
    survivor_doc = _load_yaml(os.path.join(config_dir, 'survivor.yaml'))

    mission_params = _params(
        MISSION_DEFAULTS,
        _section(mission_doc, 'mission'),
        _section(mission_doc, 'fsm'),
        {'config_file': os.path.abspath(config_path)},
    )
    mapping_params = _params(
        MAPPING_DEFAULTS, _section(mapping_doc, 'mapping'))
    navigation_params = _params(
        DRONE_DEFAULTS,
        _section(mapping_doc, 'mapping'),
        _section(mission_doc, 'mission'),
    )
    perception_params = _params(
        PERCEPTION_DEFAULTS, _section(perception_doc, 'perception'))
    survivor_params = _params(
        SURVIVOR_DEFAULTS, _section(survivor_doc, 'survivor'))
    exploration_params = _params(
        MISSION_DEFAULTS,
        _section(mission_doc, 'mission'),
        _section(mapping_doc, 'mapping'),
    )
    localization_params = {
        'publish_tf': True,
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'camera_frame': 'camera_link',
        'truth_only_for_evaluation': True,
    }

    common = {'use_sim_time': True}
    nodes = [
        Node(
            package='airmouse_mapping',
            executable='mapping_node',
            name='mapping_node',
            parameters=[common, mapping_params],
            output='screen',
        ),
        Node(
            package='airmouse_perception',
            executable='perception_node',
            name='perception_node',
            parameters=[common, perception_params],
            output='screen',
        ),
        Node(
            package='airmouse_survivor',
            executable='survivor_node',
            name='survivor_node',
            parameters=[common, survivor_params],
            output='screen',
        ),
        Node(
            package='airmouse_exploration',
            executable='exploration_node',
            name='exploration_node',
            parameters=[common, exploration_params],
            output='screen',
        ),
        Node(
            package='airmouse_navigation',
            executable='navigation_node',
            name='navigation_node',
            parameters=[common, navigation_params],
            output='screen',
        ),
        Node(
            package='airmouse_mission',
            executable='mission_node',
            name='mission_node',
            parameters=[common, mission_params],
            output='screen',
        ),
        Node(
            package='airmouse_localization',
            executable='localization_node',
            name='localization_node',
            parameters=[common, localization_params],
            output='screen',
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join('config', 'mission.yaml'),
            description='Mission YAML path (sibling mapping.yaml, '
                        'perception.yaml and survivor.yaml are auto-loaded).',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
