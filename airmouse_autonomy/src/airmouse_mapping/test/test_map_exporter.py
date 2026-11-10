"""Tests for airmouse_mapping.map_exporter (Task 2, lane M).

Covers the nav2 map_server PGM+YAML export (round-trip equals the source
grid), the pure-Python PNG encoder (zlib+struct, no pygame/PIL), and
grid_manager world/cell transform helpers.
Run from repo root: python -m pytest airmouse_autonomy/src/airmouse_mapping/test -q
"""
from __future__ import annotations

import struct
import zlib

import numpy as np
import pytest

from airmouse_mapping.grid_manager import cell_center, grid_shape, origin_for, world_to_cell
from airmouse_mapping.map_exporter import save_pgm_yaml, save_png
from airmouse_mapping.occupancy_core import FREE, OCCUPIED, UNKNOWN, OccupancyMapper


@pytest.fixture()
def grid() -> np.ndarray:
    """20x20 int8 grid with all three cell values, at 0.5 m/cell."""
    g = np.full((20, 20), UNKNOWN, dtype=np.int8)
    g[2:8, 2:8] = FREE
    g[10, :] = OCCUPIED
    g[10, 10] = FREE  # door
    return g


# --------------------------------------------------------------------------
# PGM + YAML (nav2 map_server format)
# --------------------------------------------------------------------------
def test_pgm_yaml_round_trip_equals_grid(tmp_path, grid):
    base = tmp_path / "map"
    pgm_path, yaml_path = save_pgm_yaml(grid, 0.5, (0.0, 0.0, 0.0), str(base))
    assert pgm_path == str(base) + ".pgm"
    assert yaml_path == str(base) + ".yaml"

    data = (tmp_path / "map.pgm").read_bytes()
    # P5 header: magic, "width height", maxval, then raw pixel bytes.
    magic, dims, maxval, payload = data.split(b"\n", 3)
    assert magic == b"P5"
    w, h = dims.split()
    assert (int(w), int(h)) == grid.shape[::-1]
    assert int(maxval) == 255
    pixels = np.frombuffer(payload, dtype=np.uint8)
    assert pixels.size == grid.size
    pixels = pixels.reshape(grid.shape)
    # nav2 map_server convention: white=free, black=occupied, gray=unknown.
    assert np.all(pixels[grid == FREE] == 255)
    assert np.all(pixels[grid == OCCUPIED] == 0)
    assert np.all(pixels[grid == UNKNOWN] == 205)
    # And the full inverse mapping recovers the grid exactly.
    recovered = np.where(pixels == 255, FREE, np.where(pixels == 0, OCCUPIED, UNKNOWN))
    assert np.array_equal(recovered.astype(np.int8), grid)


def test_yaml_has_map_server_fields(tmp_path, grid):
    save_pgm_yaml(grid, 0.5, (1.0, 2.0, 0.0), str(tmp_path / "map"))
    text = (tmp_path / "map.yaml").read_text(encoding="utf-8")
    assert "image: map.pgm" in text
    assert "mode: trinary" in text
    assert "resolution: 0.5" in text
    assert "origin: [1.0, 2.0, 0.0]" in text
    assert "negate: 0" in text
    assert "occupied_thresh: 0.65" in text
    assert "free_thresh: 0.25" in text


def test_pgm_yaml_creates_parent_directories(tmp_path, grid):
    base = tmp_path / "nested" / "dir" / "map"
    save_pgm_yaml(grid, 0.5, (0.0, 0.0, 0.0), str(base))
    assert (tmp_path / "nested" / "dir" / "map.pgm").is_file()
    assert (tmp_path / "nested" / "dir" / "map.yaml").is_file()


def test_pgm_yaml_rejects_invalid_grid(tmp_path):
    with pytest.raises(ValueError):
        save_pgm_yaml(np.zeros((0, 5), dtype=np.int8), 0.5, (0.0, 0.0, 0.0),
                      str(tmp_path / "map"))
    with pytest.raises(ValueError):
        save_pgm_yaml(np.zeros((5, 5), dtype=np.float32), 0.5, (0.0, 0.0, 0.0),
                      str(tmp_path / "map"))
    with pytest.raises(ValueError):
        save_pgm_yaml(np.zeros((5, 5), dtype=np.int8), 0.0, (0.0, 0.0, 0.0),
                      str(tmp_path / "map"))


def test_pgm_yaml_round_trip_from_mapper_update(tmp_path):
    """Export a sensor-built map (not just a hand-filled grid)."""
    mapper = OccupancyMapper(0.5, 10.0, 10.0, 8.0, 0.3)
    mapper.update((1.0, 1.0, 0.0), [(0.0, 3.0, True), (1.5707963, 2.0, False)])
    save_pgm_yaml(mapper.grid, mapper.cell_size, (0.0, 0.0, 0.0), str(tmp_path / "m"))
    data = (tmp_path / "m.pgm").read_bytes()
    pixels = np.frombuffer(data.split(b"\n", 3)[3], dtype=np.uint8).reshape(mapper.grid.shape)
    recovered = np.where(pixels == 255, FREE, np.where(pixels == 0, OCCUPIED, UNKNOWN))
    assert np.array_equal(recovered.astype(np.int8), mapper.grid)


# --------------------------------------------------------------------------
# PNG (pure-python zlib+struct encoder)
# --------------------------------------------------------------------------
def _parse_png(data: bytes) -> tuple[int, int, np.ndarray]:
    """Minimal PNG reader for 8-bit RGB: returns (width, height, RGB array)."""
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos = 8
    width = height = 0
    idat = bytearray()
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        ctype = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + length]
        (crc,) = struct.unpack(">I", data[pos + 8 + length:pos + 12 + length])
        assert crc == zlib.crc32(ctype + chunk) & 0xFFFFFFFF
        if ctype == b"IHDR":
            width, height, depth, color = struct.unpack(">IIBB", chunk[:10])
            assert depth == 8 and color == 2  # 8-bit truecolor RGB
        elif ctype == b"IDAT":
            idat.extend(chunk)
        pos += 12 + length
    raw = zlib.decompress(bytes(idat))
    stride = width * 3
    assert len(raw) == height * (stride + 1)
    rows = []
    for y in range(height):
        row = raw[y * (stride + 1):(y + 1) * (stride + 1)]
        assert row[0] == 0  # filter type None
        rows.append(np.frombuffer(row[1:], dtype=np.uint8).reshape(width, 3))
    return width, height, np.stack(rows)


def test_png_is_valid_and_encodes_grid_colors(tmp_path, grid):
    path = str(tmp_path / "map.png")
    save_png(grid, [], [], path)
    data = (tmp_path / "map.png").read_bytes()
    w, h, rgb = _parse_png(data)
    assert (w, h) == grid.shape[::-1]
    # Unknown gray, free white, occupied black (same palette as PGM).
    assert np.all(rgb[grid == UNKNOWN] == (205, 205, 205))
    assert np.all(rgb[grid == FREE] == (255, 255, 255))
    assert np.all(rgb[grid == OCCUPIED] == (0, 0, 0))


def test_png_overlays_tracks_and_rooms(tmp_path, grid):
    tracks = [
        {"position_x": 2.25, "position_y": 2.25, "state": "CONFIRMED"},
        {"position_x": 3.25, "position_y": 2.25, "state": "UNCERTAIN"},
    ]
    rooms = [{"center": (5.25, 5.25), "status": "SEARCHED"}]
    path = str(tmp_path / "overlay.png")
    save_png(grid, tracks, rooms, path, cell_size=0.5)
    w, h, rgb = _parse_png((tmp_path / "overlay.png").read_bytes())
    assert (w, h) == grid.shape[::-1]
    # Track markers differ from the bare grid pixels at those cells.
    bare = np.full((3,), 255, dtype=np.uint8)  # cells (4,4)/(4,6) are FREE
    assert not np.array_equal(rgb[4, 4], bare)   # confirmed marker
    assert not np.array_equal(rgb[4, 6], bare)   # uncertain marker
    assert not np.array_equal(rgb[4, 4], rgb[4, 6])  # states are distinguishable
    # Room marker present near its center cell (10,10) area.
    assert not np.array_equal(rgb[10, 10], np.array([255, 255, 255], dtype=np.uint8))


def test_png_ignores_out_of_bounds_overlays(tmp_path, grid):
    tracks = [{"position_x": -5.0, "position_y": 99.0, "state": "CONFIRMED"}]
    rooms = [{"center": (100.0, -3.0), "status": "UNSEARCHED"}]
    save_png(grid, tracks, rooms, str(tmp_path / "oob.png"), cell_size=0.5)
    w, h, rgb = _parse_png((tmp_path / "oob.png").read_bytes())
    assert (w, h) == grid.shape[::-1]
    # No overlay landed inside the image.
    assert np.all(rgb[grid == FREE] == (255, 255, 255))


def test_png_creates_parent_directories(tmp_path, grid):
    save_png(grid, [], [], str(tmp_path / "deep" / "nest" / "map.png"))
    assert (tmp_path / "deep" / "nest" / "map.png").is_file()


# --------------------------------------------------------------------------
# grid_manager helpers
# --------------------------------------------------------------------------
def test_world_to_cell_and_cell_center_round_trip():
    assert world_to_cell(1.2, 3.7, 0.5) == (7, 2)  # (row, col), y down
    assert world_to_cell(0.0, 0.0, 0.5) == (0, 0)
    x, y = cell_center(7, 2, 0.5)
    assert (x, y) == (1.25, 3.75)
    assert world_to_cell(x, y, 0.5) == (7, 2)


def test_grid_shape_and_origin():
    assert grid_shape(10.0, 10.0, 0.5) == (20, 20)
    assert grid_shape(15.0, 12.0, 0.1) == (120, 150)
    assert origin_for(0.0, 0.0) == (0.0, 0.0, 0.0)


def test_grid_manager_helpers_reject_bad_cell_size():
    with pytest.raises(ValueError):
        world_to_cell(1.0, 1.0, 0.0)
    with pytest.raises(ValueError):
        grid_shape(10.0, 10.0, -0.5)

def test_save_png_scale_upscales_nearest_neighbor(tmp_path):
    grid = np.full((6, 8), FREE, dtype=np.int8)
    grid[2, 3] = OCCUPIED
    path = str(tmp_path / "scaled.png")
    save_png(grid, [], [], path, cell_size=0.5, scale=3)
    w, h, _ = _parse_png((tmp_path / "scaled.png").read_bytes())
    assert (w, h) == (8 * 3, 6 * 3)
