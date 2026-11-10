"""airmouse_mapping: ROS-independent occupancy mapping cores (lane M).

Ported from the validated NIDAR SAR sim. numpy + stdlib only; the rclpy
adapter lives in a separate ``mapping_node`` module (wave 3).
"""
from .grid_manager import cell_center, grid_shape, origin_for, resized_copy, world_to_cell
from .map_exporter import save_pgm_yaml, save_png
from .occupancy_core import FREE, OCCUPIED, UNKNOWN, OccupancyMapper, supercover_line

__all__ = [
    "FREE",
    "OCCUPIED",
    "UNKNOWN",
    "OccupancyMapper",
    "supercover_line",
    "cell_center",
    "grid_shape",
    "origin_for",
    "resized_copy",
    "world_to_cell",
    "save_pgm_yaml",
    "save_png",
]