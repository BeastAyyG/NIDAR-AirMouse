"""AirMouse survivor lane: ROS-independent detection and tracking cores.

Ported from the validated NIDAR SAR sim (``sar/detection/tracker.py``).
rclpy adapters live in ``survivor_node.py`` (runtime wave, not this core).
"""
from .color_blob_detector import detect_survivors
from .survivor_detector import Detector, SimTargetDetector, YoloDetector
from .survivor_tracker import SurvivorTracker

__all__ = [
    "Detector",
    "SimTargetDetector",
    "YoloDetector",
    "SurvivorTracker",
    "detect_survivors",
]