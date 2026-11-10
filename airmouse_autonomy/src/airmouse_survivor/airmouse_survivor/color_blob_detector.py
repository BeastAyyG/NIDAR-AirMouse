"""Colour-blob survivor detector core (ROS-independent: numpy only).

Finds high-visibility survivor jackets (red / high-vis orange) in an RGB
image, measures the median valid depth over each blob, and back-projects the
blob centroid to a world ``(x, y)`` position using the shared camera model
from :func:`airmouse_perception.depth_processor.depth_to_points` (the single
source of truth for depth -> world geometry). No rclpy / ROS imports.

Pipeline
--------
1. Threshold the RGB image against ``color_ranges`` (inclusive per-channel
   bounds) -> boolean mask.
2. Extract 4-connected components of the mask (simple flood fill; no scipy).
3. Drop components with fewer than ``min_pixels`` pixels (noise rejection).
4. For each surviving component: centroid pixel, median of the valid depths
   inside the component, back-project with the camera intrinsics and the
   drone pose -> world point.
5. Confidence grows with blob size: ``min(0.99, 0.5 + pixels / 1000)``.

Tracker record schema note
--------------------------
Detections are returned as ``(x, y, confidence)`` in world meters. The caller
adds the observation timestamp and feeds ``(x, y, confidence, timestamp)``
tuples to :class:`airmouse_survivor.survivor_tracker.SurvivorTracker`. Tracker
output records (and therefore ``airmouse_interfaces/msg/SurvivorTrack``) have
EXACTLY these keys / fields::

    survivor_id, detected, position_x, position_y, confidence,
    timestamp, state, sightings

``state`` is ``'uncertain'`` or ``'confirmed'``; ``detected`` is True when the
track was seen in the most recent update.
"""

from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np

from airmouse_perception.depth_processor import depth_to_points

__all__ = [
    "DEFAULT_COLOR_RANGES",
    "TRACKER_RECORD_KEYS",
    "detect_survivors",
]

#: High-vis survivor jacket colour bounds: (low_rgb, high_rgb), inclusive.
#: Red first, then high-vis orange; both are common SAR jacket colours.
DEFAULT_COLOR_RANGES: tuple[tuple[tuple[int, int, int], tuple[int, int, int]], ...] = (
    ((200, 30, 30), (255, 110, 110)),
    ((200, 80, 20), (255, 180, 90)),
)

#: Keys of every tracker record (mirrors SurvivorTrack.msg field order).
TRACKER_RECORD_KEYS: tuple[str, ...] = (
    "survivor_id",
    "detected",
    "position_x",
    "position_y",
    "confidence",
    "timestamp",
    "state",
    "sightings",
)

DEFAULT_MIN_PIXELS = 60
DEFAULT_DEPTH_MIN = 0.2
DEFAULT_DEPTH_MAX = 8.0

# Confidence model: 0.5 for a minimal blob, +1.0 per 1000 pixels, cap 0.99.
_CONFIDENCE_BASE = 0.5
_CONFIDENCE_PER_PIXEL = 1.0 / 1000.0
_CONFIDENCE_CAP = 0.99

_RGB_CHANNELS = 3
_NEIGHBOUR_OFFSETS = ((-1, 0), (1, 0), (0, -1), (0, 1))  # 4-connected


def detect_survivors(
    rgb,
    depth,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    pose: Sequence[float],
    cam_offset: Sequence[float],
    color_ranges: Iterable[Sequence[Sequence[int]]] | None = None,
    min_pixels: int = DEFAULT_MIN_PIXELS,
    depth_min: float = DEFAULT_DEPTH_MIN,
    depth_max: float = DEFAULT_DEPTH_MAX,
) -> list[tuple[float, float, float]]:
    """Detect colour-blob survivors and localise them in world meters.

    Args:
        rgb: HxWx3 (or HxWx4, alpha ignored) uint8 colour image.
        depth: HxW metric depth image in meters; 0 / NaN / inf are invalid.
        fx, fy: focal lengths in pixels (non-zero).
        cx, cy: principal point in pixels.
        pose: ``(x, y, theta)`` drone pose in the world (x right, y down,
            theta 0 = +x, clockwise positive).
        cam_offset: ``(ox, oy)`` camera position relative to the drone.
        color_ranges: iterable of ``((r_lo, g_lo, b_lo), (r_hi, g_hi, b_hi))``
            inclusive bounds; defaults to :data:`DEFAULT_COLOR_RANGES`
            (red + high-vis orange jacket colours).
        min_pixels: minimum component size in pixels (default 60).
        depth_min, depth_max: valid depth band in meters (defaults 0.2 / 8.0).

    Returns:
        List of ``(x, y, confidence)`` world-meter detections in deterministic
        (row-major blob discovery) order. Blobs with no valid depth inside the
        configured band are skipped (they cannot be localised).

    Raises:
        ValueError: on malformed image/pose inputs, zero focal length,
            ``min_pixels < 1``, ``depth_min > depth_max`` or bad colour ranges.
    """
    rgb_arr = _as_rgb(rgb)
    depth_arr = _as_depth(depth, rgb_arr.shape[:2])
    _validate_camera(fx, fy, min_pixels, depth_min, depth_max)
    pose_vals = _as_pose(pose)
    offset_vals = _as_cam_offset(cam_offset)
    ranges = (
        DEFAULT_COLOR_RANGES if color_ranges is None
        else _as_color_ranges(color_ranges)
    )

    if not ranges:
        return []

    mask = _color_mask(rgb_arr, ranges)
    detections: list[tuple[float, float, float]] = []
    for rows, cols in _components(mask):
        pixel_count = rows.size
        if pixel_count < min_pixels:
            continue
        world = _component_world_point(
            rows, cols, depth_arr, fx, fy, cx, cy,
            pose_vals, offset_vals, depth_min, depth_max,
        )
        if world is None:
            continue
        confidence = min(
            _CONFIDENCE_CAP,
            _CONFIDENCE_BASE + pixel_count * _CONFIDENCE_PER_PIXEL,
        )
        detections.append((world[0], world[1], confidence))
    return detections


# ---------------------------------------------------------------- validation
def _as_rgb(rgb) -> np.ndarray:
    arr = np.asarray(rgb)
    if arr.ndim != 3 or arr.shape[2] < _RGB_CHANNELS:
        raise ValueError(
            f"rgb must be HxWx3 or HxWx4, got shape {arr.shape}"
        )
    if arr.shape[0] == 0 or arr.shape[1] == 0:
        raise ValueError(f"rgb must be non-empty, got shape {arr.shape}")
    # int16 avoids uint8 wrap-around in the threshold comparisons.
    return arr[:, :, :_RGB_CHANNELS].astype(np.int16, copy=False)


def _as_depth(depth, image_shape: tuple[int, int]) -> np.ndarray:
    arr = np.asarray(depth, dtype=np.float64)
    if arr.ndim != 2:
        raise ValueError(f"depth must be a 2-D HxW array, got shape {arr.shape}")
    if arr.shape != tuple(image_shape):
        raise ValueError(
            f"depth shape {arr.shape} does not match image shape {tuple(image_shape)}"
        )
    return arr


def _validate_camera(fx: float, fy: float, min_pixels: int,
                     depth_min: float, depth_max: float) -> None:
    if fx == 0 or fy == 0:
        raise ValueError("fx and fy must be non-zero")
    if isinstance(min_pixels, bool) or int(min_pixels) < 1:
        raise ValueError(f"min_pixels must be an integer >= 1, got {min_pixels!r}")
    if depth_min > depth_max:
        raise ValueError("depth_min must be <= depth_max")


def _as_pose(pose: Sequence[float]) -> tuple[float, float, float]:
    vals = np.asarray(pose, dtype=np.float64).ravel()
    if vals.size != 3:
        raise ValueError("pose must be (x, y, theta)")
    return float(vals[0]), float(vals[1]), float(vals[2])


def _as_cam_offset(cam_offset: Sequence[float]) -> tuple[float, float]:
    vals = np.asarray(cam_offset, dtype=np.float64).ravel()
    if vals.size != 2:
        raise ValueError("cam_offset must be (ox, oy)")
    return float(vals[0]), float(vals[1])


def _as_color_ranges(
    color_ranges: Iterable[Sequence[Sequence[int]]],
) -> list[tuple[tuple[int, int, int], tuple[int, int, int]]]:
    parsed: list[tuple[tuple[int, int, int], tuple[int, int, int]]] = []
    for entry in color_ranges:
        try:
            low, high = entry[0], entry[1]
            low_rgb = tuple(int(v) for v in low)
            high_rgb = tuple(int(v) for v in high)
        except (TypeError, IndexError, ValueError) as exc:
            raise ValueError(
                f"each colour range must be ((r_lo, g_lo, b_lo), "
                f"(r_hi, g_hi, b_hi)), got {entry!r}"
            ) from exc
        if len(low_rgb) != _RGB_CHANNELS or len(high_rgb) != _RGB_CHANNELS:
            raise ValueError(
                f"colour range bounds must have 3 channels, got {entry!r}"
            )
        if any(lo > hi for lo, hi in zip(low_rgb, high_rgb)):
            raise ValueError(f"colour range low bound exceeds high bound: {entry!r}")
        parsed.append((low_rgb, high_rgb))
    return parsed


# ----------------------------------------------------------------- detection
def _color_mask(rgb: np.ndarray,
                ranges: Sequence[tuple[tuple[int, int, int], tuple[int, int, int]]]
                ) -> np.ndarray:
    mask = np.zeros(rgb.shape[:2], dtype=bool)
    for low, high in ranges:
        in_range = np.ones(rgb.shape[:2], dtype=bool)
        for channel, (lo, hi) in enumerate(zip(low, high)):
            values = rgb[:, :, channel]
            in_range &= (values >= lo) & (values <= hi)
        mask |= in_range
    return mask


def _components(mask: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    """4-connected components as ``(rows, cols)`` index arrays, row-major order."""
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    components: list[tuple[np.ndarray, np.ndarray]] = []
    for start_r, start_c in np.argwhere(mask):
        r0, c0 = int(start_r), int(start_c)
        if visited[r0, c0]:
            continue
        visited[r0, c0] = True
        stack: list[tuple[int, int]] = [(r0, c0)]
        pixels: list[tuple[int, int]] = []
        while stack:
            r, c = stack.pop()
            pixels.append((r, c))
            for dr, dc in _NEIGHBOUR_OFFSETS:
                nr, nc = r + dr, c + dc
                if (0 <= nr < height and 0 <= nc < width
                        and mask[nr, nc] and not visited[nr, nc]):
                    visited[nr, nc] = True
                    stack.append((nr, nc))
        rows, cols = zip(*pixels)
        components.append((np.asarray(rows, dtype=np.int64),
                           np.asarray(cols, dtype=np.int64)))
    return components


def _component_world_point(rows: np.ndarray, cols: np.ndarray, depth: np.ndarray,
                           fx: float, fy: float, cx: float, cy: float,
                           pose: tuple[float, float, float],
                           cam_offset: tuple[float, float],
                           depth_min: float, depth_max: float
                           ) -> tuple[float, float] | None:
    """Back-project the component centroid at its median valid depth."""
    depths = depth[rows, cols]
    valid = (np.isfinite(depths) & (depths > 0.0)
             & (depths >= depth_min) & (depths <= depth_max))
    if not np.any(valid):
        return None
    median_depth = float(np.median(depths[valid]))

    v_centroid = int(round(float(np.mean(rows))))
    u_centroid = int(round(float(np.mean(cols))))
    # Single valid pixel at the centroid: reuse the shared camera model
    # verbatim instead of duplicating the projection maths here.
    probe = np.zeros((v_centroid + 1, u_centroid + 1), dtype=np.float64)
    probe[v_centroid, u_centroid] = median_depth
    point = depth_to_points(probe, fx, fy, cx, cy, pose, cam_offset,
                            depth_min=depth_min, depth_max=depth_max)
    if point.shape[0] == 0:
        return None
    return float(point[0, 0]), float(point[0, 1])
