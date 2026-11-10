"""AirMouse exploration: ROS-independent frontier goal-selection core.

The core accepts any duck-typed mapper exposing .frontiers(),
.plan_path(start, goal), .is_known_free(x, y) and a tracks_fn callable.
No rclpy here; the rclpy adapter lives in a separate *_node.py (wave 3).
"""
from .frontier_explorer import FrontierExplorer

__all__ = ["FrontierExplorer"]