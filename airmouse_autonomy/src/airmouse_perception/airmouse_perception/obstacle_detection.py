"""Left / centre / right obstacle region summaries (ROS-independent core).

The horizontal field of view [-hfov/2, +hfov/2] is split into ``regions``
equal angular bins using the point bearing atan2(y, x). For each bin we
report the minimum obstacle distance, a representative percentile distance
and the point count. Points are Nx3 (x, y, z); x and y give the bearing and
norm(x, y) the horizontal distance. Only numpy + stdlib are used.
"""

from __future__ import annotations

import math

import numpy as np

__all__ = ["detect_regions"]

_REGION_NAMES = ("left", "centre", "right")
_DEFAULT_REGIONS = 3
_DEFAULT_PERCENTILE = 20.0


def _region_keys(regions: int) -> list[str]:
    if regions == _DEFAULT_REGIONS:
        return list(_REGION_NAMES)
    return [f"region_{i}" for i in range(regions)]


def _empty_summary() -> dict:
    return {"min": math.inf, "representative": math.inf, "count": 0}


def detect_regions(
    points,
    hfov: float,
    regions: int = _DEFAULT_REGIONS,
    percentile: float = _DEFAULT_PERCENTILE,
) -> dict:
    """Summarise obstacle distances per angular region.

    Args:
        points: Nx3 array (x, y, z). x, y set the bearing; distance is
            norm(x, y). Empty input yields all-empty regions.
        hfov: horizontal field of view (radians); must be > 0.
        regions: number of equal angular bins (default 3 -> left/centre/right).
        percentile: percentile (0-100) used for the representative distance.

    Returns:
        Dict keyed 'left', 'centre', 'right' (for regions=3), each mapping
        to {'min': float, 'representative': float, 'count': int}. An empty
        region has min=inf, representative=inf, count=0.

    Raises:
        ValueError: on invalid hfov, regions, percentile or point shape.
    """
    if hfov <= 0:
        raise ValueError("hfov must be positive")
    if regions < 1:
        raise ValueError("regions must be >= 1")
    if not 0 <= percentile <= 100:
        raise ValueError("percentile must be in [0, 100]")

    keys = _region_keys(regions)
    result = {key: _empty_summary() for key in keys}

    arr = np.asarray(points, dtype=np.float64)
    if arr.size == 0:
        return result
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError("points must be an Nx3 array")

    x = arr[:, 0]
    y = arr[:, 1]
    angles = np.arctan2(y, x)
    distances = np.hypot(x, y)

    half = hfov / 2.0
    in_fov = (angles >= -half) & (angles <= half)
    if not np.any(in_fov):
        return result

    bin_width = hfov / regions
    indices = np.floor((angles[in_fov] + half) / bin_width).astype(int)
    indices = np.clip(indices, 0, regions - 1)
    fov_distances = distances[in_fov]

    for bin_index, key in enumerate(keys):
        region_distances = fov_distances[indices == bin_index]
        count = int(region_distances.size)
        if count == 0:
            continue
        result[key] = {
            "min": float(np.min(region_distances)),
            "representative": float(np.percentile(region_distances, percentile)),
            "count": count,
        }
    return result