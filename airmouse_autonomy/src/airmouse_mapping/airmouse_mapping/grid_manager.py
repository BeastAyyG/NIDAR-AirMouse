"""Grid lifecycle and world-frame transform helpers for airmouse_mapping.

Pure numpy + stdlib; no ROS imports. Shared convention: meters, x right,
y DOWN, ``grid[row, col]`` with ``row = int(y / cell_size)`` and
``col = int(x / cell_size)``.
"""
from __future__ import annotations

import math

import numpy as np

# Absorbs float representation error (e.g. 12.0 / 0.1 == 119.999...) while
# keeping the upstream truncation semantics for non-integral ratios.
_DIM_EPS = 1e-9


def _validate_cell_size(cell_size: float) -> float:
    cs = float(cell_size)
    if not math.isfinite(cs) or cs <= 0.0:
        raise ValueError(f"cell_size must be finite and > 0, got {cell_size!r}")
    return cs


def _validate_extent(size_m: float, name: str) -> float:
    size = float(size_m)
    if not math.isfinite(size) or size <= 0.0:
        raise ValueError(f"{name} must be finite and > 0, got {size_m!r}")
    return size


def grid_shape(width_m: float, height_m: float, cell_size: float) -> tuple[int, int]:
    """Return ``(rows, cols)`` for a world extent at the given resolution.

    Truncates like the upstream sim (``int(size / cell)``) with a tiny epsilon
    so exact multiples are not lost to float error; always at least 1 cell.
    """
    cs = _validate_cell_size(cell_size)
    width = _validate_extent(width_m, "width_m")
    height = _validate_extent(height_m, "height_m")
    rows = max(1, int(height / cs + _DIM_EPS))
    cols = max(1, int(width / cs + _DIM_EPS))
    return rows, cols


def world_to_cell(x: float, y: float, cell_size: float) -> tuple[int, int]:
    """World meters -> ``(row, col)`` (y DOWN). No bounds checking."""
    cs = _validate_cell_size(cell_size)
    return int(y / cs), int(x / cs)


def cell_center(row: int, col: int, cell_size: float) -> tuple[float, float]:
    """``(row, col)`` -> world-meter center ``(x, y)``."""
    cs = _validate_cell_size(cell_size)
    return ((col + 0.5) * cs, (row + 0.5) * cs)


def origin_for(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> tuple[float, float, float]:
    """Map origin triple ``(x, y, yaw)`` for nav2 map_server YAML export.

    The grid frame has y DOWN and row 0 at the top, so the default origin is
    the world corner ``(0, 0)`` with zero yaw.
    """
    return (float(x), float(y), float(yaw))


def resized_copy(grid: np.ndarray, rows: int, cols: int, fill: int | bool = -1) -> np.ndarray:
    """Return a NEW ``(rows, cols)`` grid preserving the top-left overlap.

    Cells outside the old extent are set to ``fill`` (default -1 = UNKNOWN).
    The input array is never mutated. Used for grid-lifecycle resize.
    """
    if grid.ndim != 2:
        raise ValueError(f"grid must be 2-D, got ndim={grid.ndim}")
    if int(rows) < 1 or int(cols) < 1:
        raise ValueError(f"resize target must be >= 1 cell, got ({rows}, {cols})")
    rows, cols = int(rows), int(cols)
    out = np.full((rows, cols), fill, dtype=grid.dtype)
    keep_r = min(rows, grid.shape[0])
    keep_c = min(cols, grid.shape[1])
    out[:keep_r, :keep_c] = grid[:keep_r, :keep_c]
    return out


def mapper_from_grid(grid: np.ndarray, cell_size: float,
                     max_range: float, clearance: float):
    """Rebuild a read-only planner view from a received occupancy grid.

    Nodes that only consume ``/map`` (exploration, navigation) use this to get
    an ``OccupancyMapper``-compatible object (same ``plan_path`` / ``frontiers``
    / ``is_known_free`` behavior) without owning sensor updates. The first FREE
    cell seeds the reachable mask. The input array is copied, never aliased.
    """
    from .occupancy_core import OccupancyMapper  # local import avoids cycles

    arr = np.asarray(grid, dtype=np.int8)
    if arr.ndim != 2:
        raise ValueError(f"grid must be 2-D, got ndim={arr.ndim}")
    rows, cols = arr.shape
    mapper = OccupancyMapper(cell_size, cols * cell_size, rows * cell_size,
                             max_range, clearance)
    mapper.grid[:, :] = arr
    mapper._observed[:, :] = arr != -1  # every known cell counts as observed
    free = np.argwhere(mapper.grid == 0)
    if len(free):
        mapper._seed = (int(free[0, 0]), int(free[0, 1]))
    return mapper