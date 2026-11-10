"""Waypoint-following local planner with a stuck watchdog.

Ported from the NIDAR SAR sim mission loop (``main.py:_follow``): pop
waypoints inside the pop tolerance, drive toward the next waypoint at
min(speed, dist/dt), and flag stuck when the drone has not moved more than
a small epsilon for longer than the watchdog window. ROS-independent.
"""
from __future__ import annotations

import math

DEFAULT_WAYPOINT_POP_TOLERANCE_M = 0.15
DEFAULT_STUCK_WATCHDOG_S = 3.0
DEFAULT_SPEED_MPS = 2.0
_MOVEMENT_EPSILON_M = 0.05  # movement below this does not reset the watchdog


class LocalPlanner:
    """Velocity command generator for a mutable waypoint path.

    params keys (all optional):
        waypoint_pop_tolerance_m (0.15): pop waypoints within this distance
        stuck_watchdog_s (3.0): no-movement window before stuck=True
        speed_mps (2.0): maximum commanded speed
    """

    def __init__(self, params: dict | None = None) -> None:
        p = dict(params or {})
        self.waypoint_pop_tolerance_m = float(
            p.get("waypoint_pop_tolerance_m", DEFAULT_WAYPOINT_POP_TOLERANCE_M))
        self.stuck_watchdog_s = float(
            p.get("stuck_watchdog_s", DEFAULT_STUCK_WATCHDOG_S))
        self.speed_mps = float(p.get("speed_mps", DEFAULT_SPEED_MPS))
        if self.waypoint_pop_tolerance_m < 0.0:
            raise ValueError("waypoint_pop_tolerance_m must be non-negative")
        if self.stuck_watchdog_s <= 0.0:
            raise ValueError("stuck_watchdog_s must be positive")
        if self.speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive")
        self.reset()

    def reset(self) -> None:
        """Clear watchdog state (call when a new goal/path is issued)."""
        self._anchor: tuple[float, float] | None = None
        self._stuck_elapsed_s = 0.0

    def follow(self, pose, path, dt) -> tuple[float, float, bool, bool]:
        """Return (vx, vy, reached, stuck) for the current pose and path.

        pose = (x, y, theta); path is a mutable list of (x, y) waypoints and
        is popped in place. reached=True once the path empties. stuck=True
        fires once per watchdog window while movement stays below epsilon;
        the watchdog re-arms after firing (mirrors the sim's notify cadence).
        """
        dt = float(dt)
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        x, y = float(pose[0]), float(pose[1])
        self._pop_reached_waypoints(x, y, path)
        if not path:
            return 0.0, 0.0, True, False
        vx, vy = self._drive_toward(x, y, path[0], dt)
        stuck = self._update_watchdog(x, y, dt)
        return vx, vy, False, stuck

    def _pop_reached_waypoints(self, x: float, y: float, path: list) -> None:
        tol = self.waypoint_pop_tolerance_m
        while path and math.hypot(path[0][0] - x, path[0][1] - y) <= tol:
            path.pop(0)

    def _drive_toward(self, x: float, y: float, target, dt: float
                      ) -> tuple[float, float]:
        dx, dy = float(target[0]) - x, float(target[1]) - y
        dist = math.hypot(dx, dy)  # > 0: equal points were popped above
        speed = min(self.speed_mps, dist / dt)
        return dx / dist * speed, dy / dist * speed

    def _update_watchdog(self, x: float, y: float, dt: float) -> bool:
        if self._anchor is None:
            self._anchor = (x, y)
            return False
        moved = math.hypot(x - self._anchor[0], y - self._anchor[1])
        if moved > _MOVEMENT_EPSILON_M:
            self._anchor = (x, y)
            self._stuck_elapsed_s = 0.0
            return False
        self._stuck_elapsed_s += dt
        if self._stuck_elapsed_s > self.stuck_watchdog_s:
            self._anchor = (x, y)
            self._stuck_elapsed_s = 0.0
            return True
        return False