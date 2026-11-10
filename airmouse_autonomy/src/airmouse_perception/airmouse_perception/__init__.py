"""AirMouse perception cores (ROS-independent).

Depth back-projection, point-cloud height filtering and left/centre/right
obstacle region summaries. These modules import only numpy + stdlib; the
rclpy adapter lives in a separate ``*_node.py`` (added in a later wave).
"""

from __future__ import annotations

from .depth_processor import depth_to_points
from .obstacle_detection import detect_regions
from .pointcloud_filter import filter_band, filter_floor_ceiling

__all__ = [
    "depth_to_points",
    "filter_band",
    "filter_floor_ceiling",
    "detect_regions",
]

__version__ = "0.1.0"