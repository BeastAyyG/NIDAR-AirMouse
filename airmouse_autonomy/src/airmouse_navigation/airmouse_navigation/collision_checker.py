"""Drone-footprint collision checks against a duck-typed occupancy mapper.

Ported from the NIDAR SAR sim mapper's disk-clearance check
(``sar/mapping/occupancy.py:_line_free``). ROS-independent: stdlib only.

Mapper contract (duck-typed):
    mapper.grid       int8 2D array, grid[row, col], row=y/cell, col=x/cell
    mapper.cell_size  float, meters per cell (positive)

Cell values: UNKNOWN=-1, FREE=0, OCCUPIED=100. Only FREE is traversable;
UNKNOWN and out-of-bounds cells are treated as blocked.
"""
from __future__ import annotations

import math

FREE = 0
OCCUPIED = 100
UNKNOWN = -1

_MARCH_STEP_FACTOR = 0.5  # fine-march sample spacing = half a cell


def disk_clear(mapper, x: float, y: float, radius: float) -> bool:
    """True iff every grid cell overlapping the disk (x, y, radius) is FREE.

    Uses a true circle/cell overlap test (closest point on the cell square),
    so a wall one cell away does not block a disk that stops short of it.
    Out-of-bounds cells overlapping the disk count as blocked.
    """
    cs = _cell_size(mapper)
    r = float(radius)
    if r < 0.0:
        raise ValueError("radius must be non-negative")
    grid = mapper.grid
    rows, cols = grid.shape[0], grid.shape[1]
    row_lo, row_hi = math.floor((y - r) / cs), math.floor((y + r) / cs)
    col_lo, col_hi = math.floor((x - r) / cs), math.floor((x + r) / cs)
    r_sq = r * r
    for row in range(row_lo, row_hi + 1):
        for col in range(col_lo, col_hi + 1):
            if not _cell_overlaps_disk(row, col, cs, x, y, r_sq):
                continue
            if row < 0 or col < 0 or row >= rows or col >= cols:
                return False
            if int(grid[row, col]) != FREE:
                return False
    return True


def segment_clear(mapper, p0, p1, radius: float) -> bool:
    """True iff the drone disk stays on FREE cells along segment p0 -> p1.

    Fine-marches at half-cell steps (cell-index supercover alone misses
    corner clips near cell boundaries) and disk-checks every sample,
    including both endpoints.
    """
    cs = _cell_size(mapper)
    x0, y0 = float(p0[0]), float(p0[1])
    x1, y1 = float(p1[0]), float(p1[1])
    dist = math.hypot(x1 - x0, y1 - y0)
    steps = max(1, math.ceil(dist / (cs * _MARCH_STEP_FACTOR)))
    for i in range(steps + 1):
        t = i / steps
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        if not disk_clear(mapper, x, y, radius):
            return False
    return True


def _cell_size(mapper) -> float:
    cs = float(mapper.cell_size)
    if cs <= 0.0:
        raise ValueError("mapper.cell_size must be positive")
    return cs


def _cell_overlaps_disk(row: int, col: int, cs: float,
                        x: float, y: float, r_sq: float) -> bool:
    """Closest-point test: does the cell square intersect the disk?"""
    near_x = min(max(x, col * cs), (col + 1) * cs)
    near_y = min(max(y, row * cs), (row + 1) * cs)
    dx, dy = x - near_x, y - near_y
    return dx * dx + dy * dy <= r_sq