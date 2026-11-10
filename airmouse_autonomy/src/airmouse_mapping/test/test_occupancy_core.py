"""Regression tests for airmouse_mapping.occupancy_core (Task 2, lane M).

Ported from the validated NIDAR SAR suite (tests/test_mapping.py, 141 green
upstream) and adapted to the ROS-independent API:
    OccupancyMapper(cell_size, width_m, height_m, max_range, clearance)
    pose = (x, y, theta); rays = list[(angle, distance, hit)]

Coordinate convention: meters, x right, y DOWN, grid[row, col],
row = int(y / cell_size), col = int(x / cell_size).
Run from repo root: python -m pytest airmouse_autonomy/src/airmouse_mapping/test -q
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from airmouse_mapping.occupancy_core import FREE, OCCUPIED, UNKNOWN, OccupancyMapper

CELL_SIZE = 0.5
WIDTH_M = 10.0
HEIGHT_M = 10.0
MAX_RANGE = 8.0
CLEARANCE = 0.1  # upstream drone_radius / roadmap default footprint


@pytest.fixture()
def mapper() -> OccupancyMapper:
    # 10 m x 10 m at 0.5 m cells -> 20 x 20 grid, fast tests.
    return OccupancyMapper(CELL_SIZE, WIDTH_M, HEIGHT_M, MAX_RANGE, CLEARANCE)


def center(row: int, col: int, cs: float = CELL_SIZE) -> tuple[float, float]:
    """World-meter center of a grid cell."""
    return ((col + 0.5) * cs, (row + 0.5) * cs)


# --------------------------------------------------------------------------
# Construction
# --------------------------------------------------------------------------
def test_initial_grid_all_unknown(mapper):
    assert mapper.grid.shape == (20, 20)
    assert mapper.grid.dtype == np.int8
    assert np.all(mapper.grid == UNKNOWN)
    assert mapper.cell_size == CELL_SIZE
    assert mapper.width_m == WIDTH_M
    assert mapper.height_m == HEIGHT_M
    assert mapper.explored_fraction() == 0.0
    assert not mapper.observed_mask().any()


def test_constructor_rejects_nonpositive_params():
    with pytest.raises(ValueError):
        OccupancyMapper(0.0, WIDTH_M, HEIGHT_M, MAX_RANGE, CLEARANCE)
    with pytest.raises(ValueError):
        OccupancyMapper(CELL_SIZE, -1.0, HEIGHT_M, MAX_RANGE, CLEARANCE)
    with pytest.raises(ValueError):
        OccupancyMapper(CELL_SIZE, WIDTH_M, HEIGHT_M, 0.0, CLEARANCE)
    with pytest.raises(ValueError):
        OccupancyMapper(CELL_SIZE, WIDTH_M, HEIGHT_M, MAX_RANGE, -0.1)


def test_constructor_rejects_nonfinite_params():
    with pytest.raises(ValueError):
        OccupancyMapper(float("nan"), WIDTH_M, HEIGHT_M, MAX_RANGE, CLEARANCE)
    with pytest.raises(ValueError):
        OccupancyMapper(CELL_SIZE, WIDTH_M, HEIGHT_M, MAX_RANGE, float("inf"))


# --------------------------------------------------------------------------
# update: free along ray, occupied at hit, unknown elsewhere
# --------------------------------------------------------------------------
def test_update_marks_free_and_occupied(mapper):
    pose = (1.0, 1.0, 0.0)  # cell (row 2, col 2)
    rays = [(0.0, 3.0, True)]  # +x, endpoint x=4 -> col 8
    mapper.update(pose, rays)
    g = mapper.grid
    assert g[2, 2] == FREE            # pose cell
    assert np.all(g[2, 3:8] == FREE)  # traversed
    assert g[2, 8] == OCCUPIED        # hit endpoint
    assert g[10, 10] == UNKNOWN       # untouched elsewhere
    assert g[2, 9] == UNKNOWN         # beyond the hit stays unknown


def test_update_no_hit_marks_endpoint_free(mapper):
    mapper.update((1.0, 1.0, 0.0), [(0.0, 3.0, False)])
    assert np.all(mapper.grid[2, 2:9] == FREE)
    assert mapper.grid[2, 9] == UNKNOWN


def test_update_caps_distance_at_max_range(mapper):
    # distance beyond max_range (8 m) with no hit -> capped at 8 m -> col 18
    mapper.update((1.0, 1.0, 0.0), [(0.0, 50.0, False)])
    assert mapper.grid[2, 18] == FREE
    assert mapper.grid[2, 19] == UNKNOWN


def test_observed_mask_tracks_rays_and_is_copy(mapper):
    mapper.update((1.0, 1.0, 0.0), [(0.0, 3.0, True)])
    obs = mapper.observed_mask()
    assert obs.dtype == bool
    assert np.all(obs[2, 2:9])
    assert not obs[10, 10]
    obs[:] = False  # mutate the returned copy
    assert mapper.observed_mask()[2, 4]  # internal unaffected


# --------------------------------------------------------------------------
# Leak-free wall
# --------------------------------------------------------------------------
def test_synthetic_wall_is_leak_free(mapper):
    # Vertical wall at x=5 (col 10). Fan of rays from (1,5) hits it across rows.
    pose = (1.0, 5.0, 0.0)
    rays = [(float(a), float(4.0 / math.cos(a)), True)
            for a in np.linspace(-0.5, 0.5, 201)]
    mapper.update(pose, rays)
    g = mapper.grid
    # Endpoint column fully occupied across the covered rows (no gaps).
    assert np.all(g[5:15, 10] == OCCUPIED)
    # Nothing behind the wall was marked free (no leak).
    assert np.all(g[5:15, 11] == UNKNOWN)
    assert not np.any(g[5:15, 11:] == FREE)


# --------------------------------------------------------------------------
# Frontiers
# --------------------------------------------------------------------------
def test_frontiers_appear_at_free_unknown_boundary(mapper):
    mapper.grid[2:6, 2:6] = FREE  # free block surrounded by unknown
    fronts = mapper.frontiers()
    assert len(fronts) == 1
    x, y = fronts[0]
    assert 0.0 <= x <= WIDTH_M and 0.0 <= y <= HEIGHT_M
    # Centroid lies near the block (world x,y in [1,3] for cols/rows 2..5).
    assert 1.0 <= x <= 3.0 and 1.0 <= y <= 3.0


def test_frontiers_vanish_without_unknown_adjacency(mapper):
    mapper.grid[1:7, 1:7] = OCCUPIED  # occupied ring
    mapper.grid[2:6, 2:6] = FREE      # free interior, no unknown neighbor
    assert mapper.frontiers() == []


def test_frontiers_drop_small_clusters(mapper):
    mapper.grid[5, 5] = FREE
    mapper.grid[5, 6] = FREE
    assert mapper.frontiers() == []  # cluster of 2 < 3 dropped
    mapper.grid[5, 7] = FREE
    assert len(mapper.frontiers()) == 1  # cluster of 3 kept


def test_frontiers_sorted_largest_first(mapper):
    mapper.grid[2:8, 2:8] = FREE      # 6x6 block -> larger frontier cluster
    mapper.grid[2:5, 12:15] = FREE    # 3x3 block -> smaller cluster
    fronts = mapper.frontiers()
    assert len(fronts) == 2
    assert fronts[0][0] < fronts[1][0]  # larger block (smaller x) first


def test_frontiers_vanish_after_full_observation(mapper):
    mapper.grid[2:6, 2:6] = FREE
    assert mapper.frontiers()  # present while unknown surrounds
    mapper.grid[:, :] = FREE   # observe everything -> no unknown left
    assert mapper.frontiers() == []


# --------------------------------------------------------------------------
# A* path planning
# --------------------------------------------------------------------------
def test_path_around_obstacle(mapper):
    mapper.grid[:, :] = FREE
    mapper.grid[2:18, 10] = OCCUPIED  # wall with open top/bottom
    start = center(10, 2)
    goal = center(10, 17)
    path = mapper.plan_path(start, goal)
    assert path is not None
    assert len(path) >= 2
    assert path[0] == pytest.approx(start)
    assert path[-1] == pytest.approx(goal)
    assert all(mapper.is_known_free(x, y) for x, y in path)
    # Must detour around the wall (reach row < 2 or row >= 18).
    assert any(y < 1.0 or y > 9.0 for _, y in path)


def test_path_none_when_walled_off(mapper):
    mapper.grid[:, :] = FREE
    mapper.grid[0:20, 10] = OCCUPIED  # full-height wall
    assert mapper.plan_path(center(10, 2), center(10, 17)) is None


def test_path_rejects_occupied_goal_without_free_neighbor(mapper):
    mapper.grid[:, :] = OCCUPIED
    mapper.grid[2, 2] = FREE  # only the start area is free
    assert mapper.plan_path(center(2, 2), center(17, 17)) is None


def test_path_goal_falls_back_to_nearby_free(mapper):
    mapper.grid[:, :] = FREE
    mapper.grid[10, 10] = OCCUPIED  # goal cell blocked
    start = center(2, 2)
    goal = center(10, 10)
    path = mapper.plan_path(start, goal)
    assert path is not None
    gx, gy = path[-1]
    gr, gc = int(gy / CELL_SIZE), int(gx / CELL_SIZE)
    assert mapper.is_known_free(gx, gy)
    assert max(abs(gr - 10), abs(gc - 10)) <= 3  # within 3 cells of goal


def test_path_smoothing_straight_line_is_two_points(mapper):
    mapper.grid[:, :] = FREE
    path = mapper.plan_path(center(2, 2), center(15, 15))
    assert path is not None
    assert len(path) == 2  # clear line of sight -> smoothed to endpoints


def test_path_smoothing_keeps_segments_on_free_cells(mapper):
    mapper.grid[:, :] = FREE
    mapper.grid[2:18, 10] = OCCUPIED
    path = mapper.plan_path(center(10, 2), center(10, 17))
    assert path is not None
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        for t in np.linspace(0.0, 1.0, 25):
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            assert mapper.is_known_free(x, y), f"segment left free space at {(x, y)}"


def test_path_unknown_not_traversable(mapper):
    # Start free, goal free, but a band of UNKNOWN between them blocks travel.
    mapper.grid[10, 2] = FREE
    mapper.grid[10, 17] = FREE
    assert mapper.plan_path(center(10, 2), center(10, 17)) is None


# --------------------------------------------------------------------------
# reachable_free_mask
# --------------------------------------------------------------------------
def test_reachable_excludes_disconnected_island(mapper):
    mapper.update((1.0, 1.0, 0.0), [(0.0, 1.0, False)])  # sets seed cell (2,2)
    mapper.grid[:, :] = UNKNOWN
    mapper.grid[2:8, 2:8] = FREE     # connected to seed
    mapper.grid[2:8, 10] = OCCUPIED  # wall
    mapper.grid[2:8, 12:16] = FREE   # disconnected island
    mask = mapper.reachable_free_mask()
    assert mask.dtype == bool
    assert mask[2, 2] and mask[5, 5]
    assert not mask[5, 13]           # island unreachable
    mask[:] = False                  # returned copy is independent
    assert mapper.reachable_free_mask()[2, 2]


def test_reachable_cache_invalidated_on_update(mapper):
    mapper.update((1.0, 1.0, 0.0), [(0.0, 1.0, False)])
    m1 = mapper.reachable_free_mask()
    assert m1[2, 4] and not m1[2, 8]
    # Extend the corridor; grid changed -> cache must recompute.
    mapper.update((1.0, 1.0, 0.0), [(0.0, 3.0, False)])
    m2 = mapper.reachable_free_mask()
    assert m2[2, 8]


# --------------------------------------------------------------------------
# is_known_free / explored_fraction
# --------------------------------------------------------------------------
def test_is_known_free_bounds_safe(mapper):
    assert not mapper.is_known_free(-1.0, 5.0)
    assert not mapper.is_known_free(1.0, -0.1)
    assert not mapper.is_known_free(100.0, 100.0)
    assert not mapper.is_known_free(1.0, 1.0)  # unknown
    mapper.update((1.0, 1.0, 0.0), [(0.0, 3.0, True)])
    assert mapper.is_known_free(1.0, 1.0)
    assert not mapper.is_known_free(4.0, 1.0)  # occupied endpoint


def test_explored_fraction_monotonic(mapper):
    fractions = [mapper.explored_fraction()]
    for dist in (1.0, 2.0, 3.0, 4.0):
        mapper.update((1.0, 1.0, 0.0), [(0.0, dist, False)])
        fractions.append(mapper.explored_fraction())
    assert all(b >= a for a, b in zip(fractions, fractions[1:]))
    assert fractions[-1] > 0.0
    assert fractions[-1] <= 1.0


# --------------------------------------------------------------------------
# Noisy-endpoint robustness (hit/miss counting, not sticky binary)
# --------------------------------------------------------------------------
def test_noisy_endpoint_does_not_permanently_seal_free_cell(mapper):
    pose = (1.0, 1.0, 0.0)
    # Short noisy hit: endpoint x=3.6 -> col 7, a truth-free cell before the wall.
    mapper.update(pose, [(0.0, 2.6, True)])
    assert mapper.grid[2, 7] == OCCUPIED
    # Later rays pass through it (hit beyond): 2 clean passes must clear it.
    mapper.update(pose, [(0.0, 5.0, False)])
    mapper.update(pose, [(0.0, 5.0, False)])
    assert mapper.grid[2, 7] == FREE


def test_repeated_hits_keep_wall_despite_single_grazing_miss(mapper):
    pose = (1.0, 1.0, 0.0)
    for _ in range(3):
        mapper.update(pose, [(0.0, 3.0, True)])  # wall at col 8
    assert mapper.grid[2, 8] == OCCUPIED
    mapper.update(pose, [(0.0, 5.0, False)])  # one grazing free observation
    assert mapper.grid[2, 8] == OCCUPIED


def test_smoothed_path_keeps_drone_clearance_of_wall_corners(mapper):
    """A 1-cell door must not be 'shortcut' by a segment clipping the frame.

    The drone has a physical footprint (clearance); every smoothed segment
    must keep the whole disk on FREE cells, not just the centerline.
    """
    mapper.grid[:, :] = FREE
    mapper.grid[10, :] = OCCUPIED
    mapper.grid[10, 10] = FREE  # single-cell door
    start = center(12, 8)
    goal = center(8, 12)
    path = mapper.plan_path(start, goal)
    assert path is not None
    r = CLEARANCE
    cs = mapper.cell_size
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(2, int(d / 0.05))
        for i in range(n + 1):
            x = x0 + (x1 - x0) * i / n
            y = y0 + (y1 - y0) * i / n
            for rr in range(int((y - r) / cs), int((y + r) / cs) + 1):
                for cc in range(int((x - r) / cs), int((x + r) / cs) + 1):
                    assert mapper.grid[rr, cc] == FREE, (
                        f"segment {path} clips occupied cell ({rr},{cc}) at {(x, y)}"
                    )


# --------------------------------------------------------------------------
# Port additions: resize lifecycle + supercover corner behavior
# --------------------------------------------------------------------------
def test_resize_preserves_overlap_and_resets_seed(mapper):
    mapper.update((1.0, 1.0, 0.0), [(0.0, 3.0, True)])
    mapper.resize(15.0, 12.0)
    assert mapper.grid.shape == (24, 30)
    assert mapper.width_m == 15.0 and mapper.height_m == 12.0
    assert mapper.grid[2, 8] == OCCUPIED       # overlap preserved
    assert np.all(mapper.grid[20:, :] == UNKNOWN)  # new rows unknown
    assert np.all(mapper.grid[:, 20:] == UNKNOWN)  # new cols unknown
    assert mapper.reachable_free_mask()[2, 2]  # re-seeded from preserved free
    mapper.resize(5.0, 5.0)  # shrink keeps the top-left overlap
    assert mapper.grid.shape == (10, 10)
    assert mapper.grid[2, 8] == OCCUPIED


def test_resize_rejects_nonpositive_dimensions(mapper):
    with pytest.raises(ValueError):
        mapper.resize(0.0, 5.0)
    with pytest.raises(ValueError):
        mapper.resize(5.0, float("inf"))


def test_supercover_matches_upstream_traversal():
    from airmouse_mapping.occupancy_core import supercover_line
    # Exact diagonal: upstream steps corner-to-corner (verified behavior).
    assert supercover_line(0, 0, 2, 2) == [(0, 0), (1, 1), (2, 2)]
    # Off-axis segment visits every touched cell between the endpoints.
    assert supercover_line(0, 0, 1, 2) == [(0, 0), (0, 1), (1, 1), (1, 2)]
    # Endpoints included for a degenerate segment.
    assert supercover_line(3, 4, 3, 4) == [(3, 4)]