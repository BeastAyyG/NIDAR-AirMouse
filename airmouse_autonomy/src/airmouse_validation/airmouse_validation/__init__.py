"""airmouse_validation: offline replay harness, truth metrics, report generator.

All modules here are ROS-independent (numpy + stdlib) so the full autonomy loop
can be validated on a machine without ROS 2 / Gazebo / PX4. Ground truth is
consumed only by the metrics, never by the autonomy pipeline.
"""
from .replay import DEFAULT_CONFIG, build_truth_world, run
from .validation_metrics import (
    collisions_from_truth,
    map_f1,
    occupancy_counts,
    survivor_match_count,
)
from .validation_report import generate

__all__ = [
    "DEFAULT_CONFIG",
    "build_truth_world",
    "run",
    "collisions_from_truth",
    "map_f1",
    "occupancy_counts",
    "survivor_match_count",
    "generate",
]
