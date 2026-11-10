"""Clearance-aware planning tests (at 0.10 m cells the drone disk covers a cell).

Lesson from the NIDAR sim: a point-robot A* that ignores the drone footprint
plans paths the vehicle physically cannot fly, wedging it against walls. The
planner must expand only cells whose disk is clear of obstacles, while still
allowing gaps wide enough for the drone (1-cell doors at 0.25 m resolution).
"""
import numpy as np

from airmouse_mapping.occupancy_core import FREE, OCCUPIED, OccupancyMapper
from airmouse_mapping.grid_manager import mapper_from_grid


def test_corridor_lane_keeps_clearance_at_fine_resolution():
    # 6 m x 6 m at 0.10 m cells; corridor between wall columns 24 and 30.
    m = OccupancyMapper(0.10, 6.0, 6.0, 5.0, 0.10)
    m.grid[:] = FREE
    m.grid[:, 24] = OCCUPIED
    m.grid[:, 30] = OCCUPIED
    # Free lane: x in (2.5, 3.0); disk radius 0.10 -> legal centers in (2.6, 2.9).
    path = m.plan_path((2.75, 0.55), (2.75, 5.45))
    assert path is not None
    assert all(2.6 - 1e-6 <= x <= 2.9 + 1e-6 for x, _ in path), path


def test_one_cell_door_passes_at_coarse_resolution():
    # NIDAR regression: 0.25 m cells, 0.10 m drone fits a 0.25 m doorway.
    m = OccupancyMapper(0.25, 10.0, 10.0, 8.0, 0.10)
    m.grid[:] = FREE
    m.grid[10, :] = OCCUPIED
    m.grid[10, 10] = FREE  # single-cell door
    path = m.plan_path((1.25, 1.25), (1.25, 3.75))
    assert path is not None


def test_gap_narrower_than_drone_is_impassable():
    # 0.10 m cells: a 0.10 m gap is narrower than the 0.20 m drone diameter.
    m = OccupancyMapper(0.10, 4.0, 4.0, 5.0, 0.10)
    m.grid[:] = FREE
    m.grid[20, :] = OCCUPIED
    m.grid[20, 20] = FREE  # 1-cell gap = 0.10 m
    assert m.plan_path((2.05, 1.0), (2.05, 3.0)) is None
    # A 3-cell gap (0.30 m) is flyable.
    m2 = OccupancyMapper(0.10, 4.0, 4.0, 5.0, 0.10)
    m2.grid[:] = FREE
    m2.grid[20, :] = OCCUPIED
    m2.grid[20, 19:22] = FREE
    assert m2.plan_path((2.05, 1.0), (2.05, 3.0)) is not None


def test_mapper_from_grid_marks_observed_cells():
    grid = np.full((10, 10), FREE, dtype=np.int8)
    grid[:, 5] = OCCUPIED
    view = mapper_from_grid(grid, 0.5, 5.0, 0.1)
    assert view.explored_fraction() == 1.0  # every cell is known
    assert view.observed_mask().all()


def test_mapper_from_grid_observed_excludes_unknown():
    grid = np.full((10, 10), -1, dtype=np.int8)
    grid[2:5, 2:5] = FREE
    view = mapper_from_grid(grid, 0.5, 5.0, 0.1)
    assert view.explored_fraction() == 9 / 100