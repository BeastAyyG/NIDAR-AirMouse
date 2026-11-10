"""Ground-truth pose formatting/validation helpers (ROS-independent core).

**DEVELOPMENT REFERENCE ONLY.** Gazebo ground truth (``/ground_truth/pose``,
bridged from ``ros_gz``) exists so the localization evaluator can measure
estimated-vs-truth drift. It must NEVER be fed into the autonomy pipeline
(mapping, exploration, navigation, mission). Production localization is
VIO / RGB-D SLAM (Goal-4 runtime follow-up); until then the truth-pose TF is a
clearly-labeled DEV baseline.

This module is deliberately tiny and ROS-free: the rclpy adapter
(``localization_node``) converts a ``geometry_msgs/PoseStamped`` into the same
plain dict via :func:`format_truth_pose`.
"""
from __future__ import annotations

import math

_REQUIRED_KEYS = ("x", "y", "z", "yaw")


def _wrap_angle(angle: float) -> float:
    """Wrap an angle in radians to (-pi, pi]."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def _coerce(value, name: str) -> float:
    """Best-effort float conversion; non-numeric input becomes NaN."""
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric, got {value!r}") from exc


def format_truth_pose(x, y, z=0.0, yaw=0.0) -> dict:
    """Normalize a ground-truth pose into a plain report dict.

    Returns ``{"x", "y", "z", "yaw", "valid"}``. ``yaw`` is wrapped to
    (-pi, pi]; ``valid`` is False when any component is non-finite (NaN/inf),
    so downstream evaluators can drop a bad sample instead of corrupting
    statistics. Non-numeric input raises ``ValueError`` (a programming error,
    not a data-quality issue).

    DEVELOPMENT REFERENCE ONLY: never feed this into the autonomy pipeline.
    """
    values = {
        "x": _coerce(x, "x"),
        "y": _coerce(y, "y"),
        "z": _coerce(z, "z"),
        "yaw": _coerce(yaw, "yaw"),
    }
    valid = all(math.isfinite(value) for value in values.values())
    if valid:
        values["yaw"] = _wrap_angle(values["yaw"])
    return {**values, "valid": valid}


def validate_truth(pose) -> bool:
    """True when ``pose`` is a usable ground-truth sample.

    Requires all of ``x, y, z, yaw`` to be present, numeric and finite; when a
    ``valid`` flag is present it must also be true. Never raises for bad data
    (the evaluator wants to skip bad samples, not crash a recording run).
    """
    if not isinstance(pose, dict):
        return False
    for key in _REQUIRED_KEYS:
        if key not in pose:
            return False
        try:
            value = float(pose[key])
        except (TypeError, ValueError):
            return False
        if not math.isfinite(value):
            return False
    if "valid" in pose and not pose["valid"]:
        return False
    return True
