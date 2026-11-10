"""Map export for airmouse_mapping: nav2 map_server PGM+YAML and PNG.

Pure numpy + stdlib (zlib/struct PNG encoder); NO pygame/PIL/ROS imports.

PGM follows the nav2 map_server trinary convention:
white (255) = FREE, black (0) = OCCUPIED, gray (205) = UNKNOWN.
Grid rows are written top-to-bottom (row 0 first), matching the y-DOWN
world frame; the YAML ``origin`` is the world pose of the grid corner.

PNG uses the same palette plus colored overlays for survivor tracks and
room status markers (NIDAR presentation-map export style).
"""
from __future__ import annotations

import os
import struct
import zlib

import numpy as np

from .occupancy_core import FREE, OCCUPIED

# Palette (grayscale values shared by PGM and PNG).
_GRAY_FREE = 255
_GRAY_OCCUPIED = 0
_GRAY_UNKNOWN = 205

# nav2 map_server trinary thresholds (probability interpretation).
_OCCUPIED_THRESH = 0.65
_FREE_THRESH = 0.25

# Overlay marker colors (RGB) and half-size of the square marker in cells.
_TRACK_COLORS = {
    "CONFIRMED": (0, 180, 0),
    "UNCERTAIN": (255, 165, 0),
}
_TRACK_DEFAULT_COLOR = (0, 120, 255)
_ROOM_DONE_STATUSES = frozenset({"SEARCHED", "CLEARED", "COMPLETE", "DONE"})
_ROOM_DONE_COLOR = (60, 120, 255)
_ROOM_PENDING_COLOR = (200, 0, 200)
_MARKER_RADIUS = 1  # 3x3 cell marker centered on the overlay cell

_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _validate_grid(grid: np.ndarray) -> np.ndarray:
    if grid.ndim != 2 or grid.size == 0:
        raise ValueError(f"grid must be a non-empty 2-D array, got shape {grid.shape}")
    return grid


def _to_gray(grid: np.ndarray) -> np.ndarray:
    """Map cell values to the trinary grayscale palette (uint8)."""
    g = np.asarray(grid)
    return np.where(g == FREE, _GRAY_FREE,
                    np.where(g == OCCUPIED, _GRAY_OCCUPIED, _GRAY_UNKNOWN)).astype(np.uint8)


def _ensure_parent(path: str) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)


def _strip_map_extension(path: str) -> str:
    base = str(path)
    for ext in (".pgm", ".yaml", ".yml", ".png"):
        if base.lower().endswith(ext):
            return base[: -len(ext)]
    return base


# --------------------------------------------------------------------------
# PGM + YAML (nav2 map_server)
# --------------------------------------------------------------------------
def save_pgm_yaml(grid: np.ndarray, cell_size: float,
                  origin: tuple[float, float, float],
                  path: str) -> tuple[str, str]:
    """Write ``<path>.pgm`` + ``<path>.yaml`` in nav2 map_server format.

    ``grid`` is the int8 occupancy grid (UNKNOWN=-1/FREE=0/OCCUPIED=100),
    ``cell_size`` meters/cell, ``origin = (x, y, yaw)`` world pose of the
    grid corner. Returns ``(pgm_path, yaml_path)``.
    """
    grid = _validate_grid(grid)
    if not np.issubdtype(grid.dtype, np.integer):
        raise ValueError(f"grid must have an integer dtype, got {grid.dtype}")
    cs = float(cell_size)
    if not np.isfinite(cs) or cs <= 0.0:
        raise ValueError(f"cell_size must be finite and > 0, got {cell_size!r}")
    if len(origin) != 3:
        raise ValueError(f"origin must be (x, y, yaw), got {origin!r}")

    base = _strip_map_extension(path)
    pgm_path = base + ".pgm"
    yaml_path = base + ".yaml"
    _ensure_parent(pgm_path)

    gray = _to_gray(grid)
    rows, cols = gray.shape
    header = f"P5\n{cols} {rows}\n255\n".encode("ascii")
    with open(pgm_path, "wb") as fh:
        fh.write(header)
        fh.write(gray.tobytes())

    ox, oy, oyaw = (float(v) for v in origin)
    yaml_text = (
        f"image: {os.path.basename(pgm_path)}\n"
        f"mode: trinary\n"
        f"resolution: {cs}\n"
        f"origin: [{ox}, {oy}, {oyaw}]\n"
        f"negate: 0\n"
        f"occupied_thresh: {_OCCUPIED_THRESH}\n"
        f"free_thresh: {_FREE_THRESH}\n"
    )
    with open(yaml_path, "w", encoding="utf-8") as fh:
        fh.write(yaml_text)
    return pgm_path, yaml_path


# --------------------------------------------------------------------------
# Pure-python PNG encoder (8-bit truecolor RGB, zlib + struct)
# --------------------------------------------------------------------------
def _png_chunk(chunk_type: bytes, payload: bytes) -> bytes:
    return (struct.pack(">I", len(payload)) + chunk_type + payload
            + struct.pack(">I", zlib.crc32(chunk_type + payload) & 0xFFFFFFFF))


def _encode_png(rgb: np.ndarray) -> bytes:
    """Encode an (H, W, 3) uint8 array as a PNG (filter type 0 per row)."""
    height, width = rgb.shape[:2]
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(height))
    return (_PNG_SIGNATURE
            + _png_chunk(b"IHDR", ihdr)
            + _png_chunk(b"IDAT", zlib.compress(raw, 9))
            + _png_chunk(b"IEND", b""))


def _draw_marker(rgb: np.ndarray, row: int, col: int,
                 color: tuple[int, int, int],
                 radius: int = _MARKER_RADIUS) -> None:
    """Draw a small filled square centered on (row, col); skip if OOB."""
    height, width = rgb.shape[:2]
    if not (0 <= row < height and 0 <= col < width):
        return
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            rr, cc = row + dr, col + dc
            if 0 <= rr < height and 0 <= cc < width:
                rgb[rr, cc] = color


def save_png(grid: np.ndarray, tracks: list[dict], rooms: list[dict],
             path: str, cell_size: float = 0.1, scale: int = 1) -> str:
    """Write a presentation PNG of the grid with track/room overlays.

    ``tracks`` items: dicts with ``position_x``, ``position_y`` (world meters)
    and ``state`` (e.g. CONFIRMED/UNCERTAIN). ``rooms`` items: dicts with
    ``center = (x, y)`` world meters and ``status``. Overlays whose center
    cell falls outside the grid are ignored. ``cell_size`` (meters/cell,
    default 0.1 = roadmap resolution) converts world positions to pixels;
    ``scale`` upscales the image (nearest neighbor) for presentation.
    Returns the written path.
    """
    grid = _validate_grid(grid)
    cs = float(cell_size)
    if not np.isfinite(cs) or cs <= 0.0:
        raise ValueError(f"cell_size must be finite and > 0, got {cell_size!r}")
    scale = int(scale)
    if scale < 1:
        raise ValueError(f"scale must be >= 1, got {scale}")

    gray = _to_gray(grid)
    if scale > 1:
        gray = np.repeat(np.repeat(gray, scale, axis=0), scale, axis=1)
    rgb = np.repeat(gray[:, :, None], 3, axis=2)
    marker_radius = _MARKER_RADIUS * scale

    for room in rooms:
        x, y = room["center"]
        row, col = int(y / cs) * scale, int(x / cs) * scale
        status = str(room.get("status", "")).upper()
        color = _ROOM_DONE_COLOR if status in _ROOM_DONE_STATUSES else _ROOM_PENDING_COLOR
        _draw_marker(rgb, row, col, color, radius=marker_radius)

    for track in tracks:
        row = int(track["position_y"] / cs) * scale
        col = int(track["position_x"] / cs) * scale
        state = str(track.get("state", "")).upper()
        color = _TRACK_COLORS.get(state, _TRACK_DEFAULT_COLOR)
        _draw_marker(rgb, row, col, color, radius=marker_radius)

    _ensure_parent(path)
    with open(path, "wb") as fh:
        fh.write(_encode_png(rgb))
    return str(path)