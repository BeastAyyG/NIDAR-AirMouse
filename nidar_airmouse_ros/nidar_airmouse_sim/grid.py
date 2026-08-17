"""Book-defined world-to-grid conversion utilities.

The fine SLAM map stays in metres. Survivor tags are projected into a coarse
one-metre grid using an explicit map origin so the convention is testable and
does not depend on a hard-coded home position.
"""

from dataclasses import dataclass
from math import floor


@dataclass(frozen=True)
class GridConfig:
    """Parameters defining the competition/grid coordinate convention."""

    origin_x: float = 0.0
    origin_y: float = 0.0
    cell_size: float = 1.0


def world_to_grid(x: float, y: float, config: GridConfig) -> tuple[int, int]:
    """Convert map-frame metres to a stable integer grid cell."""

    if config.cell_size <= 0.0:
        raise ValueError("cell_size must be greater than zero")
    return (
        floor((x - config.origin_x) / config.cell_size),
        floor((y - config.origin_y) / config.cell_size),
    )


def grid_to_world(grid_x: int, grid_y: int, config: GridConfig) -> tuple[float, float]:
    """Return the centre point of a grid cell in the map frame."""

    return (
        config.origin_x + (grid_x + 0.5) * config.cell_size,
        config.origin_y + (grid_y + 0.5) * config.cell_size,
    )


def cell_key(grid_x: int, grid_y: int) -> str:
    """Return the machine-readable cell identifier used in survivor JSON."""

    return f"{grid_x}:{grid_y}"
