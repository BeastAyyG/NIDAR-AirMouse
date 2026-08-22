"""Pure LiDAR safety checks used by the ROS 2 safety node and unit tests."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class LidarAssessment:
    """Result of checking one LiDAR scan."""

    ready: bool
    safe: bool
    emergency: bool
    valid_beams: int
    minimum_range: float | None
    reason: str


def assess_scan(
    ranges: Iterable[float],
    range_min: float,
    range_max: float,
    stop_distance: float,
    emergency_distance: float,
    minimum_valid_beams: int,
) -> LidarAssessment:
    """Classify a scan without depending on ROS message types.

    Invalid, NaN, and infinite measurements are ignored. A scan is not ready
    until it contains enough valid beams. The current implementation uses the
    closest valid beam as a conservative all-around guard; a future planner
    can use the same validated ranges directionally.
    """
    valid_ranges = [
        float(value)
        for value in ranges
        if math.isfinite(float(value))
        and float(value) >= float(range_min)
        and float(value) <= float(range_max)
    ]
    if len(valid_ranges) < int(minimum_valid_beams):
        return LidarAssessment(
            ready=False,
            safe=False,
            emergency=False,
            valid_beams=len(valid_ranges),
            minimum_range=min(valid_ranges) if valid_ranges else None,
            reason="insufficient_valid_ranges",
        )

    minimum_range = min(valid_ranges)
    if minimum_range < float(emergency_distance):
        return LidarAssessment(
            ready=True,
            safe=False,
            emergency=True,
            valid_beams=len(valid_ranges),
            minimum_range=minimum_range,
            reason="obstacle_emergency_distance",
        )
    if minimum_range < float(stop_distance):
        return LidarAssessment(
            ready=True,
            safe=False,
            emergency=False,
            valid_beams=len(valid_ranges),
            minimum_range=minimum_range,
            reason="obstacle_stop_distance",
        )
    return LidarAssessment(
        ready=True,
        safe=True,
        emergency=False,
        valid_beams=len(valid_ranges),
        minimum_range=minimum_range,
        reason="clear",
    )
