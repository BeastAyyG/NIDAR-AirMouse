"""Pytest suite for the ROS-independent navigation cores.

Uses a local stub mapper (duck-typed: .grid, .cell_size, .plan_path,
.frontiers, .is_known_free). Does NOT import airmouse_mapping.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

# Standalone bootstrap (the repo-root conftest also adds src/* to sys.path).
_PKG_DIR = str(Path(__file__).resolve().parents[1])
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from airmouse_navigation.astar_planner import plan
from airmouse_navigation.collision_checker import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    disk_clear,
    segment_clear,
)
from airmouse_navigation.local_planner import LocalPlanner

CELL = 0.1


class StubMapper:
    """Minimal duck-typed mapper for navigation tests."""

    def __init__(self, grid, cell_size=CELL):
        self.grid = np.asarray(grid, dtype=np.int8)
        self.cell_size = cell_size
        self.plan_calls = []
        self.path_result = [(0.0, 0.0), (1.0, 1.0)]

    def plan_path(self, start, goal):
        self.plan_calls.append((start, goal))
        return self.path_result

    def frontiers(self):
        return []

    def is_known_free(self, x, y):
        row, col = int(y / self.cell_size), int(x / self.cell_size)
        rows, cols = self.grid.shape
        if row < 0 or col < 0 or row >= rows or col >= cols:
            return False
        return self.grid[row, col] == FREE


def free_grid(size=10):
    return np.zeros((size, size), dtype=np.int8)


# -- collision_checker: disk_clear -----------------------------------------


def test_cell_value_constants():
    assert (FREE, OCCUPIED, UNKNOWN) == (0, 100, -1)


def test_disk_clear_true_in_open_free_area():
    mapper = StubMapper(free_grid())
    assert disk_clear(mapper, 0.55, 0.55, 0.1) is True


def test_disk_clear_false_near_wall():
    grid = free_grid()
    grid[:, 8] = OCCUPIED  # wall spanning x in [0.8, 0.9]
    mapper = StubMapper(grid)
    # Disk reaches x=0.85 -> overlaps the wall column.
    assert disk_clear(mapper, 0.75, 0.55, 0.1) is False
    # Disk stops at x=0.75 -> short of the wall.
    assert disk_clear(mapper, 0.65, 0.55, 0.1) is True


def test_disk_clear_false_on_unknown_cell():
    grid = free_grid()
    grid[5, 5] = UNKNOWN
    mapper = StubMapper(grid)
    assert disk_clear(mapper, 0.55, 0.55, 0.05) is False


def test_disk_clear_false_out_of_bounds():
    mapper = StubMapper(free_grid())
    # Disk extends past the left edge of the grid.
    assert disk_clear(mapper, 0.02, 0.55, 0.1) is False


def test_disk_clear_zero_radius_checks_single_cell():
    grid = free_grid()
    grid[5, 5] = OCCUPIED
    mapper = StubMapper(grid)
    assert disk_clear(mapper, 0.55, 0.55, 0.0) is False
    assert disk_clear(mapper, 0.45, 0.45, 0.0) is True


def test_disk_clear_rejects_negative_radius():
    mapper = StubMapper(free_grid())
    with pytest.raises(ValueError):
        disk_clear(mapper, 0.5, 0.5, -0.1)


# -- collision_checker: segment_clear --------------------------------------


def test_segment_clear_true_on_free_segment():
    mapper = StubMapper(free_grid())
    assert segment_clear(mapper, (0.15, 0.15), (0.35, 0.25), 0.05) is True


def test_segment_clear_false_through_wall():
    grid = free_grid()
    grid[:, 8] = OCCUPIED
    mapper = StubMapper(grid)
    assert segment_clear(mapper, (0.2, 0.55), (0.95, 0.55), 0.05) is False


def test_segment_clear_false_on_corner_clip():
    # Diagonal wall cells (4,5) and (5,4); the centerline passes through the
    # free diagonal gap (4,4)->(5,5) but the drone disk clips the corner.
    grid = free_grid()
    grid[4, 5] = OCCUPIED
    grid[5, 4] = OCCUPIED
    mapper = StubMapper(grid)
    assert grid[4, 4] == FREE and grid[5, 5] == FREE  # centerline cells free
    assert disk_clear(mapper, 0.5, 0.5, 0.05) is False
    assert segment_clear(mapper, (0.45, 0.45), (0.55, 0.55), 0.05) is False


def test_segment_clear_checks_endpoints():
    grid = free_grid()
    grid[5, 5] = OCCUPIED
    mapper = StubMapper(grid)
    assert segment_clear(mapper, (0.55, 0.55), (0.15, 0.15), 0.0) is False


# -- astar_planner ----------------------------------------------------------


def test_astar_plan_delegates_to_mapper():
    mapper = StubMapper(free_grid())
    mapper.path_result = [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)]
    start, goal = (0.0, 0.0), (1.0, 1.0)
    result = plan(mapper, start, goal)
    assert result == mapper.path_result
    assert mapper.plan_calls == [(start, goal)]


def test_astar_plan_none_when_unreachable():
    mapper = StubMapper(free_grid())
    mapper.path_result = None
    assert plan(mapper, (0.0, 0.0), (1.0, 1.0)) is None


def test_astar_plan_normalizes_waypoints_to_float_tuples():
    mapper = StubMapper(free_grid())
    mapper.path_result = [(0, 0), (1, 2)]
    result = plan(mapper, (0.0, 0.0), (1.0, 2.0))
    assert result == [(0.0, 0.0), (1.0, 2.0)]
    assert all(isinstance(x, float) and isinstance(y, float) for x, y in result)


# -- local_planner ----------------------------------------------------------


def test_local_planner_pops_waypoint_and_reports_reached():
    planner = LocalPlanner()
    path = [(0.1, 0.0), (1.0, 0.0)]
    vx, vy, reached, stuck = planner.follow((0.0, 0.0, 0.0), path, 0.1)
    # First waypoint is inside the 0.15 m pop tolerance -> popped in place.
    assert path == [(1.0, 0.0)]
    assert vx == pytest.approx(2.0)
    assert vy == pytest.approx(0.0)
    assert reached is False and stuck is False
    vx, vy, reached, stuck = planner.follow((1.0, 0.0, 0.0), path, 0.1)
    assert path == []
    assert reached is True
    assert vx == 0.0 and vy == 0.0


def test_local_planner_empty_path_is_reached():
    planner = LocalPlanner()
    assert planner.follow((0.0, 0.0, 0.0), [], 0.1) == (0.0, 0.0, True, False)


def test_local_planner_speed_capped_by_distance_over_dt():
    planner = LocalPlanner({"waypoint_pop_tolerance_m": 0.01})
    path = [(0.05, 0.0)]
    vx, vy, reached, _ = planner.follow((0.0, 0.0, 0.0), path, 0.1)
    assert vx == pytest.approx(0.5)  # dist/dt = 0.5 < speed_mps 2.0
    assert vy == pytest.approx(0.0)
    assert reached is False


def test_local_planner_drives_toward_next_waypoint():
    planner = LocalPlanner()
    path = [(0.0, 3.0)]
    vx, vy, reached, _ = planner.follow((0.0, 0.0, 1.57), path, 0.1)
    assert vx == pytest.approx(0.0)
    assert vy == pytest.approx(2.0)  # +y, theta is not used for steering
    assert reached is False


def test_stuck_watchdog_fires_after_timeout_without_movement():
    planner = LocalPlanner()  # watchdog 3.0 s
    path = [(5.0, 0.0)]
    flags = []
    for _ in range(8):
        vx, vy, reached, stuck = planner.follow((0.0, 0.0, 0.0), path, 0.5)
        flags.append(stuck)
        assert vx == pytest.approx(2.0)  # still commands motion while stuck
        assert reached is False
    # 0.5 s per call: elapsed exceeds 3.0 s only on the 8th call.
    assert not any(flags[:7])
    assert flags[7] is True


def test_stuck_watchdog_reset_by_movement():
    planner = LocalPlanner()
    path = [(100.0, 0.0)]
    for i in range(20):
        x = 0.1 * i  # moves 0.1 m per call (> 0.05 m epsilon)
        _, _, _, stuck = planner.follow((x, 0.0, 0.0), path, 0.5)
        assert stuck is False


def test_reset_clears_stuck_watchdog():
    planner = LocalPlanner({"stuck_watchdog_s": 1.0})
    path = [(5.0, 0.0)]
    flags = [planner.follow((0.0, 0.0, 0.0), path, 0.5)[3] for _ in range(3)]
    assert not any(flags)  # elapsed 1.0 s, not yet > 1.0 s
    planner.reset()
    flags = [planner.follow((0.0, 0.0, 0.0), path, 0.5)[3] for _ in range(2)]
    assert not any(flags)  # without reset the next call would have fired


def test_local_planner_rejects_nonpositive_dt():
    planner = LocalPlanner()
    with pytest.raises(ValueError):
        planner.follow((0.0, 0.0, 0.0), [(1.0, 0.0)], 0.0)


def test_local_planner_rejects_invalid_params():
    with pytest.raises(ValueError):
        LocalPlanner({"speed_mps": 0.0})
    with pytest.raises(ValueError):
        LocalPlanner({"stuck_watchdog_s": -1.0})
    with pytest.raises(ValueError):
        LocalPlanner({"waypoint_pop_tolerance_m": -0.1})