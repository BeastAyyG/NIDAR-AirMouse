"""Pytest suite for the ROS-independent frontier exploration core.

Uses a local stub mapper (duck-typed: .grid, .cell_size, .plan_path,
.frontiers, .is_known_free) and plain track dicts via tracks_fn.
Does NOT import airmouse_mapping or airmouse_survivor.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

# Standalone bootstrap (the repo-root conftest also adds src/* to sys.path).
_PKG_DIR = str(Path(__file__).resolve().parents[1])
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from airmouse_exploration.frontier_explorer import FrontierExplorer

PARAMS = {
    "replan_interval_s": 1.0,
    "goal_reach_radius_m": 0.3,
    "investigate_after_s": 5.0,
    "max_investigate_attempts": 2,
    "lidar_range_m": 4.0,
}
ENTRY = (0.0, 0.0)


class StubMapper:
    """Minimal duck-typed mapper; plan_path returns None for unreachable goals."""

    def __init__(self, frontiers=(), unreachable=(), cell_size=0.1):
        self._frontiers = [tuple(f) for f in frontiers]
        self._unreachable = {tuple(t) for t in unreachable}
        self.cell_size = cell_size
        self.grid = np.zeros((20, 20), dtype=np.int8)
        self.plan_calls = []

    def set_frontiers(self, frontiers):
        self._frontiers = [tuple(f) for f in frontiers]

    def frontiers(self):
        return list(self._frontiers)

    def plan_path(self, start, goal):
        self.plan_calls.append((tuple(start), tuple(goal)))
        if tuple(goal) in self._unreachable:
            return None
        return [tuple(start), tuple(goal)]

    def is_known_free(self, x, y):
        return True


def make_track(sid, x, y, first_seen=0.0, state="uncertain", confidence=0.4):
    return {
        "survivor_id": sid,
        "position_x": x,
        "position_y": y,
        "state": state,
        "first_seen": first_seen,
        "confidence": confidence,
    }


def make_explorer(frontiers=(), tracks=(), unreachable=(), entry=ENTRY):
    mapper = StubMapper(frontiers, unreachable)
    track_list = list(tracks)
    explorer = FrontierExplorer(mapper, lambda: track_list, entry, PARAMS)
    return explorer, mapper, track_list


def test_initial_state_and_goal_dict_schema():
    explorer, _, _ = make_explorer(frontiers=[(1.0, 1.0)])
    assert explorer.mission_complete is False
    assert explorer.current_goal is None
    assert explorer.path == []
    goal = explorer.decide(0.0)
    assert set(goal) == {"goal_id", "kind", "target", "priority", "status", "created_at"}
    assert goal["kind"] == "EXPLORE"
    assert goal["target"] == (1.0, 1.0)
    assert goal["priority"] == 1.0
    assert goal["status"] == "active"
    assert goal["created_at"] == 0.0


def test_picks_nearest_large_frontier():
    # Largest-first list: far large cluster vs near small cluster.
    # score = size_proxy - distance/lidar_range -> the near one wins here.
    explorer, _, _ = make_explorer(frontiers=[(5.0, 0.0), (0.5, 0.0)])
    goal = explorer.decide(0.0)
    assert goal["kind"] == "EXPLORE"
    assert goal["target"] == (0.5, 0.0)
    # At (nearly) equal distance the largest cluster (index 0) wins.
    explorer, _, _ = make_explorer(frontiers=[(0.5, 0.0), (0.6, 0.0)])
    goal = explorer.decide(0.0)
    assert goal["target"] == (0.5, 0.0)


def test_set_pose_is_used_for_ranking():
    explorer, _, _ = make_explorer(frontiers=[(0.5, 0.0), (9.0, 0.0)])
    explorer.set_pose(9.0, 0.0)
    goal = explorer.decide(0.0)
    assert goal["target"] == (9.0, 0.0)


def test_blacklists_unreachable_frontier():
    explorer, mapper, _ = make_explorer(
        frontiers=[(1.0, 1.0), (2.0, 2.0)], unreachable=[(1.0, 1.0)]
    )
    goal = explorer.decide(0.0)
    # First candidate (1,1) fails to plan -> blacklisted -> (2,2) issued.
    assert goal["kind"] == "EXPLORE"
    assert goal["target"] == (2.0, 2.0)
    assert mapper.plan_calls == [((0.0, 0.0), (1.0, 1.0)), ((0.0, 0.0), (2.0, 2.0))]
    # (2,2) frontier gets covered by observation; (1,1) persists but is
    # blacklisted, so the next replan must not retry it.
    mapper.set_frontiers([(1.0, 1.0)])
    goal = explorer.decide(1.5)
    assert goal["kind"] == "RETURN"
    assert sum(1 for _, g in mapper.plan_calls if g == (1.0, 1.0)) == 1


def test_failed_explore_goal_blacklists_target():
    explorer, mapper, _ = make_explorer(frontiers=[(1.0, 1.0)])
    goal = explorer.decide(0.0)
    explorer.notify_goal_failed(goal["goal_id"])
    assert goal["status"] == "aborted"
    goal = explorer.decide(0.1)  # failure forces an immediate replan
    assert goal["kind"] == "RETURN"
    assert sum(1 for _, g in mapper.plan_calls if g == (1.0, 1.0)) == 1


def test_returns_return_goal_when_no_frontiers():
    explorer, _, _ = make_explorer(frontiers=[])
    goal = explorer.decide(0.0)
    assert goal["kind"] == "RETURN"
    assert goal["target"] == ENTRY
    assert goal["priority"] == 0.0
    assert goal["status"] == "active"
    assert explorer.mission_complete is False


def test_notify_goal_reached_return_sets_mission_complete():
    explorer, _, _ = make_explorer(frontiers=[])
    goal = explorer.decide(0.0)
    explorer.notify_goal_reached(goal["goal_id"])
    assert goal["status"] == "done"
    assert explorer.mission_complete is True
    assert explorer.current_goal is None
    assert explorer.path == []
    assert explorer.decide(10.0) is None


def test_unreachable_entry_completes_mission():
    explorer, _, _ = make_explorer(frontiers=[], unreachable=[ENTRY])
    assert explorer.decide(0.0) is None
    assert explorer.mission_complete is True


def test_investigate_preempts_explore_for_aged_uncertain_track():
    explorer, _, track_list = make_explorer(frontiers=[(1.0, 1.0)])
    explore_goal = explorer.decide(0.0)
    assert explore_goal["kind"] == "EXPLORE"
    track_list.append(make_track(7, 3.0, 3.0, first_seen=0.0))
    goal = explorer.decide(10.0)  # track age 10 s > investigate_after_s 5 s
    assert goal["kind"] == "INVESTIGATE"
    assert goal["target"] == (3.0, 3.0)
    assert goal["priority"] == 2.0
    # Superseded EXPLORE goal stays retryable, not blacklisted.
    assert explore_goal["status"] == "pending"


def test_investigate_ignores_fresh_and_confirmed_tracks():
    tracks = [
        make_track(1, 3.0, 3.0, first_seen=9.0),  # age 1 s < 5 s
        make_track(2, 4.0, 4.0, first_seen=0.0, state="confirmed"),
    ]
    explorer, _, _ = make_explorer(frontiers=[(1.0, 1.0)], tracks=tracks)
    goal = explorer.decide(10.0)
    assert goal["kind"] == "EXPLORE"


def test_investigate_attempt_cap_falls_back_to_explore():
    tracks = [make_track(7, 3.0, 3.0, first_seen=0.0)]
    explorer, _, _ = make_explorer(frontiers=[(1.0, 1.0)], tracks=tracks)
    goal = explorer.decide(10.0)
    assert goal["kind"] == "INVESTIGATE"
    explorer.notify_goal_failed(goal["goal_id"])  # attempt 1
    goal = explorer.decide(11.0)
    assert goal["kind"] == "INVESTIGATE"
    explorer.notify_goal_failed(goal["goal_id"])  # attempt 2 == cap
    goal = explorer.decide(12.0)
    assert goal["kind"] == "EXPLORE"
    assert goal["target"] == (1.0, 1.0)


def test_replan_cadence_holds_current_goal_between_intervals():
    explorer, mapper, _ = make_explorer(frontiers=[(1.0, 1.0)])
    goal = explorer.decide(0.0)
    assert len(mapper.plan_calls) == 1
    # Not due yet: same goal returned even if the frontier disappears.
    assert explorer.decide(0.5) is goal
    mapper.set_frontiers([])
    assert explorer.decide(0.9) is goal
    assert len(mapper.plan_calls) == 1
    # Due now: stale EXPLORE goal retired, no frontiers left -> RETURN.
    goal = explorer.decide(1.1)
    assert goal["kind"] == "RETURN"
    assert len(mapper.plan_calls) == 2


def test_notify_goal_reached_explore_retires_goal_and_clears_path():
    explorer, _, _ = make_explorer(frontiers=[(1.0, 1.0)])
    goal = explorer.decide(0.0)
    assert explorer.path != []
    explorer.notify_goal_reached(goal["goal_id"])
    assert goal["status"] == "done"
    assert explorer.current_goal is None
    assert explorer.path == []
    assert explorer.mission_complete is False