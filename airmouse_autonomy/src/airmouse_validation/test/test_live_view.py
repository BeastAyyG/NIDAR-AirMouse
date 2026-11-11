"""Tests for the live video+distance+map view composition (no ROS required)."""
import numpy as np
import pytest

from airmouse_validation.live_view import (
    center_distance_m,
    compose_frame,
    nearest_distance_m,
    render_map_panel,
)


def test_center_distance_reads_center_patch_in_metres():
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    depth[230:250, 310:330] = 3000
    assert center_distance_m(depth) == pytest.approx(3.0)


def test_center_distance_ignores_zero_and_none_when_all_invalid():
    depth = np.zeros((480, 640), dtype=np.uint16)
    assert center_distance_m(depth) is None
    depth[239, 320] = 1500
    assert center_distance_m(depth) == pytest.approx(1.5)


def test_nearest_distance_ignores_single_noisy_pixel():
    depth = np.full((240, 320), 4000, dtype=np.uint16)
    depth[0, 0] = 100
    assert nearest_distance_m(depth) == pytest.approx(4.0, abs=0.05)


def test_nearest_distance_finds_real_obstacle():
    depth = np.full((240, 320), 8000, dtype=np.uint16)
    depth[:, :64] = 1500  # 20% of the frame at 1.5 m
    assert nearest_distance_m(depth) == pytest.approx(1.5, abs=0.05)
    assert nearest_distance_m(np.zeros((10, 10), dtype=np.uint16)) is None


def test_map_panel_colors_cells_and_marks_pose():
    grid = np.full((10, 10), -1, dtype=np.int16)
    grid[5, 5] = 100
    grid[2, 3] = 0
    panel = render_map_panel(grid, 100, pose_xy=(0.15, 0.85), resolution=0.1)
    assert panel.shape == (100, 100, 3)
    assert panel.dtype == np.uint8
    assert tuple(panel[55, 55]) == (0, 0, 0)        # occupied -> black
    assert tuple(panel[25, 35]) == (255, 255, 255)  # free -> white
    assert tuple(panel[5, 5]) == (128, 128, 128)    # unknown -> grey
    assert tuple(panel[85, 15]) == (255, 96, 0)     # drone marker


def test_compose_frame_layout_and_hud():
    color = np.full((480, 640, 3), 120, dtype=np.uint8)
    depth = np.full((270, 480), 2500, dtype=np.uint16)
    grid = np.full((150, 150), -1, dtype=np.int16)
    grid[75, 75] = 100
    frame = compose_frame(color, depth, grid, pose_xy=(7.5, 7.5),
                          resolution=0.1)
    assert frame.ndim == 3 and frame.dtype == np.uint8
    assert frame.shape[1] == 1280
    assert not np.array_equal(frame[:, :640], frame[:, 640:])


def test_compose_frame_placeholder_before_first_messages():
    frame = compose_frame(None, None, None)
    assert frame.ndim == 3 and frame.dtype == np.uint8
    assert frame.shape[1] == 1280