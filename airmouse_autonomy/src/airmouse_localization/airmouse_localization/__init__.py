"""airmouse_localization: truth-pose baseline + localization evaluator (lane V).

ROS-independent cores live in ``localization_eval`` and ``truth_baseline``;
the rclpy adapter lives in ``localization_node`` (DEV baseline only, wave 3).

IMPORTANT: Gazebo ground truth is a DEVELOPMENT reference. It feeds the
evaluator (estimated-vs-truth drift) and is never consumed by the autonomy
pipeline. Actual VIO / RGB-D SLAM is a Goal-4 runtime follow-up.
"""
from .localization_eval import compute_errors
from .truth_baseline import format_truth_pose, validate_truth

__all__ = [
    "compute_errors",
    "format_truth_pose",
    "validate_truth",
]
