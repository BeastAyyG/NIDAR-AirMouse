"""Live camera + distance + fused 2D map composition (ROS-free).

The Jetson-side adapter (``scripts/live_view.py``) feeds camera frames and
the fused occupancy grid in here; this module renders the browser view.
Only numpy + cv2, so the layout and distance math are testable offline.
Arrays are RGB uint8 throughout; the adapter converts to BGR for JPEG.
"""
from __future__ import annotations

import cv2
import numpy as np

UNKNOWN_COLOR = (128, 128, 128)
FREE_COLOR = (255, 255, 255)
OCCUPIED_COLOR = (0, 0, 0)
POSE_COLOR = (255, 96, 0)
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def center_distance_m(depth_mm, patch: int = 16) -> float | None:
    """Median of the valid centre patch, in metres (None if all invalid)."""
    arr = np.asarray(depth_mm)
    if arr.ndim != 2:
        raise ValueError("depth must be a 2D array")
    h, w = arr.shape
    cy, cx = h // 2, w // 2
    half = max(1, int(patch) // 2)
    window = arr[max(0, cy - half):cy + half, max(0, cx - half):cx + half]
    valid = window[window > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) / 1000.0


def nearest_distance_m(depth_mm, percentile: float = 5.0) -> float | None:
    """Robust nearest return in metres: low percentile of valid depths.

    A plain minimum would latch onto single noisy pixels; the 5th
    percentile needs a real surface covering a few percent of the frame.
    """
    arr = np.asarray(depth_mm)
    valid = arr[arr > 0]
    if valid.size == 0:
        return None
    return float(np.percentile(valid, percentile)) / 1000.0


def render_map_panel(grid, size_px: int, pose_xy=None,
                     resolution: float = 0.1) -> np.ndarray:
    """Occupancy grid -> square RGB panel (grey unknown, white free,
    black occupied) with an optional drone marker at map-frame metres."""
    grid = np.asarray(grid)
    if grid.ndim != 2:
        raise ValueError("grid must be 2D")
    panel = np.empty((*grid.shape, 3), dtype=np.uint8)
    panel[:] = UNKNOWN_COLOR
    panel[grid == 0] = FREE_COLOR
    panel[grid == 100] = OCCUPIED_COLOR
    out = cv2.resize(panel, (size_px, size_px),
                     interpolation=cv2.INTER_NEAREST)
    if pose_xy is not None:
        scale = size_px / (grid.shape[0] * float(resolution))
        px = int(round(pose_xy[0] * scale))
        py = int(round(pose_xy[1] * scale))
        cv2.circle(out, (px, py), max(2, size_px // 100), POSE_COLOR, -1)
    return out


def compose_frame(color_rgb, depth_mm, grid, pose_xy=None, *,
                  resolution: float = 0.1, width: int = 1280,
                  height: int = 600) -> np.ndarray:
    """Camera panel (crosshair + centre distance) | map panel, HUD below.

    ``color_rgb`` None renders a waiting placeholder so the stream is up
    before the first camera frame arrives.
    """
    canvas = np.full((height, width, 3), 24, dtype=np.uint8)
    if color_rgb is None:
        cv2.putText(canvas, "waiting for camera + map ...", (40, height // 2),
                    _FONT, 1.0, (200, 200, 200), 2)
        return canvas

    hud_h = 64
    panel_h = height - hud_h
    half = width // 2
    color = cv2.resize(np.asarray(color_rgb), (half, panel_h))
    center = center_distance_m(depth_mm) if depth_mm is not None else None
    nearest = nearest_distance_m(depth_mm) if depth_mm is not None else None

    cx, cy = half // 2, panel_h // 2
    cv2.drawMarker(color, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
    if center is not None:
        cv2.putText(color, f"{center:.2f} m", (cx + 16, cy - 12),
                    _FONT, 0.9, (0, 255, 0), 2)
    cv2.putText(color, "LIVE CAMERA", (12, 28), _FONT, 0.7,
                (255, 255, 255), 2)
    canvas[:panel_h, :half] = color

    explored = None
    if grid is not None:
        grid = np.asarray(grid)
        explored = float(np.count_nonzero(grid >= 0)) / grid.size
        map_panel = render_map_panel(grid, panel_h, pose_xy, resolution)
        x0 = half + (half - panel_h) // 2
        canvas[:panel_h, x0:x0 + panel_h] = map_panel
        cv2.putText(canvas, "2D MAP (sensor fusion)", (half + 12, 28),
                    _FONT, 0.7, (255, 255, 255), 2)

    center_text = f"{center:.2f} m" if center is not None else "--"
    nearest_text = f"{nearest:.2f} m" if nearest is not None else "--"
    explored_text = f"{explored:.0%}" if explored is not None else "--"
    cv2.putText(canvas,
                f"centre {center_text}   nearest {nearest_text}   "
                f"explored {explored_text}",
                (16, height - 22), _FONT, 0.7, (255, 255, 255), 2)
    return canvas