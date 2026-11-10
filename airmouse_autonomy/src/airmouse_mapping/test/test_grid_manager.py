"""Tests for grid_manager helpers, including the /map -> planner view adapter."""
import numpy as np

from airmouse_mapping.grid_manager import mapper_from_grid
from airmouse_mapping.occupancy_core import FREE, OCCUPIED


def test_mapper_from_grid_copies_and_plans():
    grid = np.full((10, 10), FREE, dtype=np.int8)
    grid[:, 5] = OCCUPIED
    mapper = mapper_from_grid(grid, cell_size=0.5, max_range=5.0, clearance=0.1)
    assert mapper.cell_size == 0.5
    assert mapper.grid[0, 5] == OCCUPIED
    # planner sees the wall segment
    assert mapper.plan_path((1.25, 1.25), (3.75, 1.25)) is None
    # detour via gaps at row extremes is reachable
    grid[0, 5] = FREE
    mapper2 = mapper_from_grid(grid, 0.5, 5.0, 0.1)
    path = mapper2.plan_path((1.25, 1.25), (3.75, 1.25))
    assert path is not None and path[-1] == (3.75, 1.25)
    # input array is copied, not aliased
    mapper2.grid[1, 1] = OCCUPIED
    assert grid[1, 1] == FREE