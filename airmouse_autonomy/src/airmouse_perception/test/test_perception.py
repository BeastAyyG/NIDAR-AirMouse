"""Unit tests for the AirMouse perception cores (ROS-independent).

Run from the repo root:
    python -m pytest airmouse_autonomy/src/airmouse_perception/test -q
The repo conftest adds every src/<pkg> dir to sys.path so the inner
``airmouse_perception`` package imports without installation.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from airmouse_perception import (
    depth_to_points,
    detect_regions,
    filter_band,
    filter_floor_ceiling,
)

IDENTITY_POSE = (0.0, 0.0, 0.0)
ZERO_OFFSET = (0.0, 0.0)


# --------------------------------------------------------------------------
# depth_processor.depth_to_points
# --------------------------------------------------------------------------
def test_back_projects_center_pixel_to_known_metric_point():
    # fx=fy=1, cx=cy=0 => the centre pixel is (u, v) = (0, 0).
    # depth 2 along +x with identity pose/offset -> world (2, 0, 0).
    depth = np.zeros((3, 3), dtype=np.float64)
    depth[0, 0] = 2.0
    points = depth_to_points(depth, 1.0, 1.0, 0.0, 0.0, IDENTITY_POSE, ZERO_OFFSET)
    assert points.shape == (1, 3)
    np.testing.assert_allclose(points[0], [2.0, 0.0, 0.0], atol=1e-9)


def test_off_center_pixels_map_image_axes_to_world():
    # pixel (u=1, v=0): X=(1)*2=2 -> height z; Y=0 -> world y; Z=2 -> world x.
    depth = np.zeros((3, 3), dtype=np.float64)
    depth[0, 1] = 2.0
    points = depth_to_points(depth, 1.0, 1.0, 0.0, 0.0, IDENTITY_POSE, ZERO_OFFSET)
    np.testing.assert_allclose(points[0], [2.0, 0.0, 2.0], atol=1e-9)

    # pixel (u=0, v=1): Y=(1)*2=2 -> world y; X=0 -> height z; Z=2 -> world x.
    depth = np.zeros((3, 3), dtype=np.float64)
    depth[1, 0] = 2.0
    points = depth_to_points(depth, 1.0, 1.0, 0.0, 0.0, IDENTITY_POSE, ZERO_OFFSET)
    np.testing.assert_allclose(points[0], [2.0, 2.0, 0.0], atol=1e-9)


def test_pose_rotation_and_translation():
    depth = np.zeros((1, 1), dtype=np.float64)
    depth[0, 0] = 2.0
    # theta = +pi/2 (clockwise, y-down): (2, 0) -> (0, 2); then + pose (1, 2).
    points = depth_to_points(depth, 1.0, 1.0, 0.0, 0.0, (1.0, 2.0, math.pi / 2), ZERO_OFFSET)
    np.testing.assert_allclose(points[0], [1.0, 4.0, 0.0], atol=1e-9)


def test_camera_offset_is_applied_in_drone_frame():
    depth = np.zeros((1, 1), dtype=np.float64)
    depth[0, 0] = 2.0
    points = depth_to_points(depth, 1.0, 1.0, 0.0, 0.0, IDENTITY_POSE, (0.5, -0.25))
    np.testing.assert_allclose(points[0], [2.5, -0.25, 0.0], atol=1e-9)


def test_rejects_nan_zero_inf_and_out_of_range():
    depth = np.array(
        [
            [2.0, np.nan, np.inf],   # only 2.0 (row 0, col 0) is valid
            [-np.inf, -1.0, 0.1],    # 0.1 below depth_min=0.2
            [9.0, 8.5, 0.0],         # 9.0 / 8.5 above depth_max=8.0; 0 invalid
        ],
        dtype=np.float64,
    )
    points = depth_to_points(depth, 1.0, 1.0, 0.0, 0.0, IDENTITY_POSE, ZERO_OFFSET)
    # Only the single valid depth (2.0 at the centre pixel) survives -> (2, 0, 0).
    assert points.shape == (1, 3)
    np.testing.assert_allclose(points[0], [2.0, 0.0, 0.0], atol=1e-9)


def test_depth_range_kwargs_are_configurable():
    depth = np.array([[0.5, 1.5, 2.5, 3.5]], dtype=np.float64)
    points = depth_to_points(
        depth, 1.0, 1.0, 0.0, 0.0, IDENTITY_POSE, ZERO_OFFSET, depth_min=1.0, depth_max=3.0
    )
    # Keeps 1.5 and 2.5 only; world x equals depth (identity, u maps to height).
    assert points.shape == (2, 3)
    np.testing.assert_allclose(np.sort(points[:, 0]), [1.5, 2.5], atol=1e-9)


def test_all_invalid_returns_empty():
    depth = np.array([[0.0, np.nan], [np.inf, -1.0]], dtype=np.float64)
    points = depth_to_points(depth, 1.0, 1.0, 0.0, 0.0, IDENTITY_POSE, ZERO_OFFSET)
    assert points.shape == (0, 3)


def test_invalid_intrinsics_raise():
    depth = np.ones((2, 2), dtype=np.float64)
    with pytest.raises(ValueError):
        depth_to_points(depth, 0.0, 1.0, 0.0, 0.0, IDENTITY_POSE, ZERO_OFFSET)


def test_non_2d_depth_raises():
    with pytest.raises(ValueError):
        depth_to_points(np.ones((2, 2, 2)), 1.0, 1.0, 0.0, 0.0, IDENTITY_POSE, ZERO_OFFSET)


# --------------------------------------------------------------------------
# pointcloud_filter
# --------------------------------------------------------------------------
def test_filter_band_keeps_only_in_band_z_inclusive():
    points = np.array(
        [
            [0.0, 0.0, 0.1],
            [1.0, 1.0, 0.3],
            [2.0, 2.0, 1.0],
            [3.0, 3.0, 2.0],
            [4.0, 4.0, 2.5],
        ],
        dtype=np.float64,
    )
    kept = filter_band(points, 0.3, 2.0)
    np.testing.assert_allclose(kept[:, 2], [0.3, 1.0, 2.0], atol=1e-9)
    # x, y preserved for the retained rows.
    np.testing.assert_allclose(kept[:, 0], [1.0, 2.0, 3.0], atol=1e-9)


def test_filter_band_empty_input():
    kept = filter_band(np.empty((0, 3)), 0.3, 2.0)
    assert kept.shape == (0, 3)


def test_filter_floor_ceiling_removes_floor_and_ceiling():
    points = np.array(
        [
            [0.0, 0.0, 0.10],  # below floor -> removed
            [0.0, 0.0, 0.15],  # exactly floor -> removed (strict)
            [0.0, 0.0, 0.50],  # kept
            [0.0, 0.0, 2.20],  # exactly ceiling -> removed (strict)
            [0.0, 0.0, 2.50],  # above ceiling -> removed
        ],
        dtype=np.float64,
    )
    kept = filter_floor_ceiling(points, 0.15, 2.2)
    assert kept.shape == (1, 3)
    np.testing.assert_allclose(kept[0], [0.0, 0.0, 0.5], atol=1e-9)


def test_filter_invalid_shape_raises():
    with pytest.raises(ValueError):
        filter_band(np.ones((3, 2)), 0.0, 1.0)


# --------------------------------------------------------------------------
# obstacle_detection.detect_regions
# --------------------------------------------------------------------------
def _polar_point(distance: float, angle: float) -> list[float]:
    return [distance * math.cos(angle), distance * math.sin(angle), 1.0]


def test_detect_regions_synthetic_cloud_min_representative_count():
    hfov = 1.2  # bins: left [-0.6,-0.2], centre [-0.2,0.2], right [0.2,0.6]
    points = np.array(
        [
            _polar_point(3.0, -0.4),  # left
            _polar_point(5.0, -0.4),  # left
            _polar_point(2.0, 0.0),   # centre
            _polar_point(4.0, 0.0),   # centre
            _polar_point(1.5, 0.4),   # right
        ],
        dtype=np.float64,
    )
    regions = detect_regions(points, hfov)
    assert set(regions) == {"left", "centre", "right"}

    assert regions["left"]["count"] == 2
    assert regions["left"]["min"] == pytest.approx(3.0)
    assert regions["left"]["representative"] == pytest.approx(3.4)  # p20 of [3,5]

    assert regions["centre"]["count"] == 2
    assert regions["centre"]["min"] == pytest.approx(2.0)
    assert regions["centre"]["representative"] == pytest.approx(2.4)  # p20 of [2,4]

    assert regions["right"]["count"] == 1
    assert regions["right"]["min"] == pytest.approx(1.5)
    assert regions["right"]["representative"] == pytest.approx(1.5)


def test_detect_regions_empty_region_is_inf():
    points = np.array([_polar_point(2.0, 0.0)], dtype=np.float64)  # centre only
    regions = detect_regions(points, 1.2)
    for key in ("left", "right"):
        assert regions[key]["min"] == math.inf
        assert regions[key]["representative"] == math.inf
        assert regions[key]["count"] == 0
    assert regions["centre"]["count"] == 1
    assert regions["centre"]["min"] == pytest.approx(2.0)


def test_detect_regions_all_empty():
    regions = detect_regions(np.empty((0, 3)), 1.2)
    for key in ("left", "centre", "right"):
        assert regions[key] == {"min": math.inf, "representative": math.inf, "count": 0}


def test_detect_regions_excludes_outside_fov():
    points = np.array(
        [_polar_point(2.0, 0.0), _polar_point(9.0, 1.0)],  # 1.0 rad > hfov/2=0.6
        dtype=np.float64,
    )
    regions = detect_regions(points, 1.2)
    assert regions["centre"]["count"] == 1
    assert regions["left"]["count"] == 0
    assert regions["right"]["count"] == 0


def test_detect_regions_percentile_kwarg():
    points = np.array([_polar_point(3.0, -0.4), _polar_point(5.0, -0.4)], dtype=np.float64)
    p0 = detect_regions(points, 1.2, percentile=0)
    p100 = detect_regions(points, 1.2, percentile=100)
    assert p0["left"]["representative"] == pytest.approx(3.0)   # min
    assert p100["left"]["representative"] == pytest.approx(5.0)  # max


def test_detect_regions_summary_types():
    regions = detect_regions(np.array([_polar_point(2.0, 0.0)]), 1.2)
    centre = regions["centre"]
    assert set(centre) == {"min", "representative", "count"}
    assert isinstance(centre["min"], float)
    assert isinstance(centre["representative"], float)
    assert isinstance(centre["count"], int)


def test_detect_regions_invalid_args_raise():
    pts = np.array([_polar_point(2.0, 0.0)])
    with pytest.raises(ValueError):
        detect_regions(pts, 0.0)
    with pytest.raises(ValueError):
        detect_regions(pts, 1.2, regions=0)
    with pytest.raises(ValueError):
        detect_regions(pts, 1.2, percentile=150)