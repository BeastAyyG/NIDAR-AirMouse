"""Global planner: thin delegation to the mapper's proven A* implementation.

The occupancy mapper owns the single A* (8-connected, no corner-cut,
footprint-aware LOS smoothing). This module keeps the navigation-side API
stable and normalizes waypoints to plain float tuples. ROS-independent.
"""
from __future__ import annotations


def plan(mapper, start, goal) -> list[tuple[float, float]] | None:
    """Return a list of (x, y) waypoints from start to goal, or None.

    Delegates to ``mapper.plan_path(start, goal)``; None means unreachable.
    The mapper is duck-typed (any object exposing plan_path).
    """
    path = mapper.plan_path(start, goal)
    if path is None:
        return None
    return [(float(x), float(y)) for x, y in path]