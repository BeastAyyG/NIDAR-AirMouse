"""Point-cloud height filters (ROS-independent core).

Points are Nx3 arrays where column 2 is the height z. These filters keep
the flight-path collision volume by removing floor / ceiling returns or by
selecting an inclusive height band. Only numpy + stdlib are used.
"""

from __future__ import annotations

import numpy as np

__all__ = ["filter_band", "filter_floor_ceiling"]


def _as_points(points) -> np.ndarray:
    """Normalise input to a float Nx3 array (empty -> shape (0, 3))."""
    arr = np.asarray(points, dtype=np.float64)
    if arr.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError("points must be an Nx3 array")
    return arr


def filter_band(points, zmin: float, zmax: float) -> np.ndarray:
    """Keep points whose height z lies in the inclusive band [zmin, zmax].

    Args:
        points: Nx3 array (x, y, z); column 2 is height.
        zmin, zmax: inclusive height bounds (metres).

    Returns:
        Mx3 float array of the in-band points (order preserved). Empty
        input yields a (0, 3) array.

    Raises:
        ValueError: if points is not Nx3 or zmin > zmax.
    """
    arr = _as_points(points)
    if zmin > zmax:
        raise ValueError("zmin must be <= zmax")
    if arr.shape[0] == 0:
        return arr
    z = arr[:, 2]
    mask = (z >= zmin) & (z <= zmax)
    return arr[mask]


def filter_floor_ceiling(points, floor_z: float, ceil_z: float) -> np.ndarray:
    """Keep points strictly between floor and ceiling: floor_z < z < ceil_z.

    Points at or below ``floor_z`` (floor) and at or above ``ceil_z``
    (ceiling) are removed.

    Args:
        points: Nx3 array (x, y, z); column 2 is height.
        floor_z, ceil_z: floor / ceiling height thresholds (metres).

    Returns:
        Mx3 float array of the retained points (order preserved). Empty
        input yields a (0, 3) array.

    Raises:
        ValueError: if points is not Nx3 or floor_z > ceil_z.
    """
    arr = _as_points(points)
    if floor_z > ceil_z:
        raise ValueError("floor_z must be <= ceil_z")
    if arr.shape[0] == 0:
        return arr
    z = arr[:, 2]
    mask = (z > floor_z) & (z < ceil_z)
    return arr[mask]