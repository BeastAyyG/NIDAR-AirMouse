"""AirMouse navigation: ROS-independent planning and collision-check cores.

Cores accept any duck-typed mapper exposing .grid (int8 2D), .cell_size,
.plan_path(start, goal), .frontiers(), .is_known_free(x, y). No rclpy here;
the rclpy adapter lives in a separate *_node.py module (wave 3).
"""
from .astar_planner import plan
from .collision_checker import FREE, OCCUPIED, UNKNOWN, disk_clear, segment_clear
from .local_planner import LocalPlanner

__all__ = [
    "FREE",
    "OCCUPIED",
    "UNKNOWN",
    "LocalPlanner",
    "disk_clear",
    "plan",
    "segment_clear",
]