"""Frontier-based exploration decision layer.

Ported from the NIDAR SAR sim goal planner (``sar/planner/goals.py``),
decoupled from the concrete tracker via ``tracks_fn`` and from ROS.
ROS-independent: stdlib only.

Priority: INVESTIGATE aged uncertain tracks (2) > EXPLORE the best frontier
(1) > RETURN to entry (0, mission complete). Replanning happens at most
every ``replan_interval_s`` seconds, or immediately when a goal
completes/fails. The mission loop follows ``path``, pops waypoints, and
reports outcomes via the notify_* hooks.

Goals are plain dicts: {goal_id, kind, target, priority, status, created_at}.
"""
from __future__ import annotations

import math

KIND_EXPLORE = "EXPLORE"
KIND_INVESTIGATE = "INVESTIGATE"
KIND_RETURN = "RETURN"

STATUS_PENDING = "pending"
STATUS_ACTIVE = "active"
STATUS_DONE = "done"
STATUS_ABORTED = "aborted"

INVESTIGATE_PRIORITY = 2.0
EXPLORE_PRIORITY = 1.0
RETURN_PRIORITY = 0.0

# Defaults mirror airmouse_autonomy/config/mission.yaml + mapping.yaml.
DEFAULT_REPLAN_INTERVAL_S = 2.0
DEFAULT_GOAL_REACH_RADIUS_M = 0.4
DEFAULT_INVESTIGATE_AFTER_S = 4.0
DEFAULT_MAX_INVESTIGATE_ATTEMPTS = 2
DEFAULT_LIDAR_RANGE_M = 6.0

_UNCERTAIN = "uncertain"


class FrontierExplorer:
    """Selects goals from mapper frontiers and survivor track indications.

    mapper (duck-typed): .frontiers() largest-first world-meter centroids,
        .plan_path(start, goal) -> waypoints | None, .is_known_free(x, y).
    tracks_fn: callable returning track dicts with keys survivor_id,
        position_x, position_y, state, first_seen, confidence.
    entry: (x, y) mission entry point, target of the RETURN goal.
    params keys (all optional): replan_interval_s (2.0),
        goal_reach_radius_m (0.4), investigate_after_s (4.0),
        max_investigate_attempts (2), lidar_range_m (6.0, ranking cost).
    """

    def __init__(self, mapper, tracks_fn, entry, params: dict | None = None) -> None:
        p = dict(params or {})
        self.mapper = mapper
        self.tracks_fn = tracks_fn
        self.entry = (float(entry[0]), float(entry[1]))
        self.replan_interval_s = float(
            p.get("replan_interval_s", DEFAULT_REPLAN_INTERVAL_S))
        self.goal_reach_radius_m = float(
            p.get("goal_reach_radius_m", DEFAULT_GOAL_REACH_RADIUS_M))
        self.investigate_after_s = float(
            p.get("investigate_after_s", DEFAULT_INVESTIGATE_AFTER_S))
        self.max_investigate_attempts = int(
            p.get("max_investigate_attempts", DEFAULT_MAX_INVESTIGATE_ATTEMPTS))
        self.lidar_range_m = float(p.get("lidar_range_m", DEFAULT_LIDAR_RANGE_M))
        if self.replan_interval_s <= 0.0 or self.goal_reach_radius_m <= 0.0:
            raise ValueError("replan_interval_s and goal_reach_radius_m must be positive")
        if self.lidar_range_m <= 0.0:
            raise ValueError("lidar_range_m must be positive")
        if self.max_investigate_attempts < 0:
            raise ValueError("max_investigate_attempts must be non-negative")

        self.current_goal: dict | None = None
        self.path: list[tuple[float, float]] = []
        self.mission_complete = False

        self._goals: dict[int, dict] = {}
        self._goal_track: dict[int, int] = {}  # goal_id -> survivor_id
        self._investigate_attempts: dict[int, int] = {}  # survivor_id -> attempts
        self._blacklist: list[tuple[float, float]] = []  # unreachable frontiers
        self._next_goal_id = 1
        self._last_replan_time = float("-inf")
        self._forced_replan = True
        # Best position estimate: entry, last reached target, or next waypoint.
        self._position = self.entry
        self._pose: tuple[float, float] | None = None  # fed by set_pose

    # -- public API ----------------------------------------------------------

    def set_pose(self, x: float, y: float) -> None:
        """Actual drone position from the mission loop; authoritative for planning."""
        self._pose = (float(x), float(y))

    def decide(self, now: float) -> dict | None:
        """Return the active goal dict (issuing a new one when due), or None."""
        if self.mission_complete:
            return None
        if not self._replan_due(now):
            return self.current_goal
        self._last_replan_time = now
        self._forced_replan = False

        # An active goal with an empty path was trivially reached (the local
        # planner pops waypoints in place); settle it before selecting anew.
        if (self.current_goal is not None
                and self.current_goal["status"] == STATUS_ACTIVE
                and not self.path):
            self.notify_goal_reached(self.current_goal["goal_id"])

        frontiers = self.mapper.frontiers()
        candidates = self._investigate_candidates(now)

        if self.current_goal is not None:
            if self._current_goal_valid(frontiers):
                preempt = (self.current_goal["kind"] == KIND_EXPLORE
                           and bool(candidates))
                if not preempt:
                    return self.current_goal
                # Superseded, not failed: keep the frontier retryable.
                self._retire_current(STATUS_PENDING)
            else:
                self._retire_invalid_current(frontiers)

        for track in candidates:
            goal = self._issue_goal(
                KIND_INVESTIGATE,
                (track["position_x"], track["position_y"]),
                INVESTIGATE_PRIORITY, now, track["survivor_id"])
            if goal is not None:
                return goal
        for frontier in self._ranked_frontiers(frontiers):
            goal = self._issue_goal(KIND_EXPLORE, frontier, EXPLORE_PRIORITY, now, None)
            if goal is not None:
                return goal
        return self._issue_return(now)

    def notify_goal_reached(self, goal_id: int) -> None:
        goal = self._goals.get(goal_id)
        if goal is None or goal["status"] != STATUS_ACTIVE:
            return
        if goal["kind"] == KIND_INVESTIGATE:
            self._count_attempt_for_goal(goal)
        if self.current_goal is goal:
            self._retire_current(STATUS_DONE)
        else:
            goal["status"] = STATUS_DONE
        self._position = goal["target"]
        if goal["kind"] == KIND_RETURN:
            self.mission_complete = True
            self.current_goal = None
            self.path = []

    def notify_goal_failed(self, goal_id: int) -> None:
        goal = self._goals.get(goal_id)
        if goal is None or goal["status"] != STATUS_ACTIVE:
            return
        self._apply_failure_side_effects(goal)
        if self.current_goal is goal:
            self._retire_current(STATUS_ABORTED)
        else:
            goal["status"] = STATUS_ABORTED

    # -- goal selection helpers ------------------------------------------------

    def _replan_due(self, now: float) -> bool:
        return (self._forced_replan
                or self.current_goal is None
                or now - self._last_replan_time >= self.replan_interval_s)

    def _investigate_candidates(self, now: float) -> list[dict]:
        eligible = [
            track for track in (self.tracks_fn() or [])
            if str(track["state"]).lower() == _UNCERTAIN
            and now - float(track["first_seen"]) > self.investigate_after_s
            and self._investigate_attempts.get(track["survivor_id"], 0)
            < self.max_investigate_attempts
        ]
        eligible.sort(key=lambda track: float(track["first_seen"]))  # oldest first
        return eligible

    def _ranked_frontiers(self, frontiers: list) -> list[tuple[float, float]]:
        """Nearest-largest heuristic: cluster-size rank bonus minus distance cost.

        Frontiers arrive sorted largest-first, so rank (n - index) is the size
        proxy; one rank step is worth one lidar_range of travel.
        """
        start = self._start_position()
        total = len(frontiers)
        scored = []
        for index, frontier in enumerate(frontiers):
            if self._is_blacklisted(frontier):
                continue
            size_proxy = float(total - index)
            distance = math.hypot(frontier[0] - start[0], frontier[1] - start[1])
            score = size_proxy - distance / self.lidar_range_m
            scored.append((score, index, frontier))
        scored.sort(key=lambda item: (-item[0], item[1]))
        return [frontier for _, _, frontier in scored]

    def _current_goal_valid(self, frontiers: list) -> bool:
        goal = self.current_goal
        if goal is None or goal["status"] != STATUS_ACTIVE:
            return False
        if not self._path_valid():
            return False
        if goal["kind"] == KIND_EXPLORE:
            return self._frontier_near(goal["target"], frontiers)
        if goal["kind"] == KIND_INVESTIGATE:
            track = self._track_by_id(self._goal_track.get(goal["goal_id"]))
            return track is not None and str(track["state"]).lower() == _UNCERTAIN
        return True  # RETURN stays valid until reached

    def _path_valid(self) -> bool:
        if not self.path:
            return False
        # The final waypoint may be the raw (possibly non-free) goal target
        # that the mapper snapped internally, so only validate the prefix.
        return all(self.mapper.is_known_free(x, y) for x, y in self.path[:-1])

    def _retire_invalid_current(self, frontiers: list) -> None:
        goal = self.current_goal
        if goal is None:
            return
        if goal["kind"] == KIND_EXPLORE and not self._frontier_near(goal["target"], frontiers):
            # Frontier covered by other observations: goal satisfied, retryable.
            self._retire_current(STATUS_DONE)
            return
        if goal["kind"] == KIND_INVESTIGATE:
            track = self._track_by_id(self._goal_track.get(goal["goal_id"]))
            if track is None or str(track["state"]).lower() != _UNCERTAIN:
                # Confirmed or decayed while en route: nothing to investigate.
                self._retire_current(STATUS_DONE)
                return
        self._apply_failure_side_effects(goal)
        self._retire_current(STATUS_ABORTED)

    # -- goal issuance ---------------------------------------------------------

    def _issue_goal(self, kind: str, target, priority: float,
                    now: float, track_id: int | None) -> dict | None:
        """Create a goal and plan its path; None means failed or trivially done."""
        path = self.mapper.plan_path(self._start_position(), target)
        goal = {
            "goal_id": self._next_goal_id,
            "kind": kind,
            "target": (float(target[0]), float(target[1])),
            "priority": priority,
            "status": STATUS_PENDING,
            "created_at": now,
        }
        self._next_goal_id += 1
        self._goals[goal["goal_id"]] = goal
        if track_id is not None:
            self._goal_track[goal["goal_id"]] = track_id
        if path is None:
            self._apply_failure_side_effects(goal)
            goal["status"] = STATUS_ABORTED
            return None
        if not path:
            # Already at the target: count the investigation opportunity, done.
            if track_id is not None:
                self._investigate_attempts[track_id] = (
                    self._investigate_attempts.get(track_id, 0) + 1)
            goal["status"] = STATUS_DONE
            return None
        goal["status"] = STATUS_ACTIVE
        self.current_goal = goal
        self.path = [(float(x), float(y)) for x, y in path]
        return goal

    def _issue_return(self, now: float) -> dict | None:
        goal = self._issue_goal(KIND_RETURN, self.entry, RETURN_PRIORITY, now, None)
        if goal is None:
            # Entry unreachable (or already there): exploration is over either way.
            self.mission_complete = True
            self.current_goal = None
            self.path = []
            return None
        return goal

    # -- bookkeeping -----------------------------------------------------------

    def _retire_current(self, status: str) -> None:
        if self.path:
            self._position = self.path[0]
        if self.current_goal is not None:
            self.current_goal["status"] = status
        self.current_goal = None
        self.path = []
        self._forced_replan = True

    def _apply_failure_side_effects(self, goal: dict) -> None:
        if goal["kind"] == KIND_EXPLORE:
            self._blacklist.append(goal["target"])
        elif goal["kind"] == KIND_INVESTIGATE:
            self._count_attempt_for_goal(goal)

    def _count_attempt_for_goal(self, goal: dict) -> None:
        track_id = self._goal_track.get(goal["goal_id"])
        if track_id is not None:
            self._investigate_attempts[track_id] = (
                self._investigate_attempts.get(track_id, 0) + 1)

    def _start_position(self) -> tuple[float, float]:
        if self._pose is not None:
            return self._pose
        return self.path[0] if self.path else self._position

    def _is_blacklisted(self, position) -> bool:
        radius = self.goal_reach_radius_m
        return any(math.hypot(position[0] - x, position[1] - y) <= radius
                   for x, y in self._blacklist)

    def _frontier_near(self, target, frontiers: list) -> bool:
        radius = self.goal_reach_radius_m
        return any(math.hypot(target[0] - x, target[1] - y) <= radius
                   for x, y in frontiers)

    def _track_by_id(self, track_id: int | None) -> dict | None:
        if track_id is None:
            return None
        return next((t for t in (self.tracks_fn() or [])
                     if t["survivor_id"] == track_id), None)