"""TDD suite for airmouse_mapping.scan_fusion_core (scan-fusion core).

ROS-independent contract: 4 x LaserScan-style ``(angles, ranges)`` arrays plus
one front point cloud -> ``(angle, distance, hit)`` rays for
``OccupancyMapper.update``. Convention follows the mapper: metres, x right,
y DOWN, absolute ray angles (theta 0 = +x, clockwise).

Run from airmouse_autonomy/: python -m pytest
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from airmouse_mapping.occupancy_core import OCCUPIED, OccupancyMapper
from airmouse_mapping.scan_fusion_core import (
    RateGate,
    cloud_to_rays,
    optical_to_body,
    scan_to_rays,
)
from airmouse_perception.pointcloud_filter import filter_band

HALF_PI = 0.5 * math.pi


def _wall_cloud(distance: float, extent: float, steps: int = 15) -> np.ndarray:
    """Synthetic RGB-D wall return in an OPTICAL frame (x right, y down,
    z forward): a square patch at ``distance`` metres ahead."""
    axis = np.linspace(-extent, extent, steps)
    lateral, vertical = np.meshgrid(axis, axis)
    depth = np.full(lateral.shape, distance, dtype=np.float64)
    return np.column_stack(
        (lateral.ravel(), vertical.ravel(), depth.ravel())
    )


# --------------------------------------------------------------------------
# optical_to_body: RGB-D optical frames (RealSense) -> body axes
# --------------------------------------------------------------------------
def test_optical_to_body_permutes_axes():
    # Optical: 1 m right, 2 m down, 3 m forward.
    body = optical_to_body(np.array([[1.0, 2.0, 3.0]]))
    # Body: x forward, y left, z up.
    assert body[0][0] == pytest.approx(3.0)
    assert body[0][1] == pytest.approx(-1.0)
    assert body[0][2] == pytest.approx(-2.0)


def test_optical_to_body_keeps_empty_input_and_nan():
    empty = optical_to_body(np.empty((0, 3), dtype=np.float64))
    assert empty.shape == (0, 3)
    noisy = optical_to_body(np.array([[np.nan, 0.0, 3.0]]))
    assert noisy[0][0] == pytest.approx(3.0)  # forward stays finite
    assert np.isnan(noisy[0][1])              # the NaN moved to lateral


def test_optical_to_body_rejects_bad_shape():
    with pytest.raises(ValueError):
        optical_to_body(np.array([1.0, 2.0, 3.0]))
    with pytest.raises(ValueError):
        optical_to_body(np.ones((4, 2)))


def test_front_wall_in_optical_cloud_maps_to_hits_ahead():
    """Regression: the D435i cloud is optical, so a 3.3 m wall must land as
    hits in front, not as noise at the drone."""
    optical = _wall_cloud(distance=3.3, extent=1.2, steps=41)
    body = optical_to_body(optical)
    body[:, 2] += 1.05  # camera height in the map frame (bench TF)
    kept = filter_band(body, 0.3, 2.0)
    rays = cloud_to_rays(kept, (0.0, 0.0, 0.0), fov_center_yaw=0.0,
                         fov_width=math.radians(87.0), max_range=6.0,
                         min_range=0.1, ray_count=90)
    hits = [ray for ray in rays if ray[2]]
    assert len(hits) >= 30
    assert all(abs(angle) <= math.radians(44.0) for angle, _, _ in hits)
    # Ray distance is slant range: 3.3 m straight ahead, more at the edges.
    straight = [distance for angle, distance, _ in hits if abs(angle) < 0.02]
    assert straight and straight[0] == pytest.approx(3.3, abs=0.03)
    assert max(distance for _, distance, _ in hits) <= 3.3 / math.cos(
        math.atan(1.2 / 3.3)) + 1e-6
    # Floor returns (large positive optical y) must not survive the band.
    floor = np.column_stack((np.zeros(5), np.full(5, 1.5), np.full(5, 2.0)))
    floor_body = optical_to_body(floor)
    floor_body[:, 2] += 1.05
    assert filter_band(floor_body, 0.3, 2.0).shape[0] == 0



# --------------------------------------------------------------------------
# scan_to_rays: LaserScan samples -> absolute map-frame rays
# --------------------------------------------------------------------------
def test_scan_to_rays_bins_to_sector_centres_with_hits():
    angles = np.array([-0.1, 0.0, 0.1])
    ranges = np.array([2.0, 3.0, 4.0])
    rays = scan_to_rays(angles, ranges, pose_yaw=0.0, mount_yaw=0.0,
                        max_range=6.0, ray_count=3)
    width = 0.2 / 3
    expected = [(-0.1 + (k + 0.5) * width, d)
                for k, d in enumerate((2.0, 3.0, 4.0))]
    assert len(rays) == 3
    for (angle, distance, hit), (exp_angle, exp_dist) in zip(rays, expected):
        assert angle == pytest.approx(exp_angle)
        assert distance == pytest.approx(exp_dist)
        assert hit is True


def test_scan_to_rays_applies_pose_and_mount_yaw():
    angles = np.array([-0.05, 0.0, 0.05])
    ranges = np.array([2.0, 2.0, 2.0])
    rays = scan_to_rays(angles, ranges, pose_yaw=0.5, mount_yaw=1.0,
                        max_range=6.0, ray_count=3)
    width = 0.1 / 3
    # Middle sector centre is 0.0 -> absolute angle = pose + mount = 1.5.
    assert rays[1][0] == pytest.approx(1.5)
    assert rays[0][0] == pytest.approx(1.5 - 0.05 + 0.5 * width)
    assert rays[2][0] == pytest.approx(1.5 + 0.05 - 0.5 * width)


def test_scan_to_rays_drops_invalid_samples_and_fills_misses():
    angles = np.array([-0.2, -0.1, 0.0, 0.1, 0.2])
    ranges = np.array([0.0, np.inf, np.nan, 0.02, 3.0])
    rays = scan_to_rays(angles, ranges, pose_yaw=0.0, mount_yaw=0.0,
                        max_range=6.0, min_range=0.05, ray_count=5)
    assert [r[2] for r in rays] == [False, False, False, False, True]
    assert all(r[1] == pytest.approx(6.0) for r in rays[:4])
    assert rays[4][1] == pytest.approx(3.0)


def test_scan_to_rays_range_beyond_max_becomes_miss_at_max():
    angles = np.array([-0.05, 0.05])
    ranges = np.array([50.0, 50.0])
    rays = scan_to_rays(angles, ranges, pose_yaw=0.0, mount_yaw=0.0,
                        max_range=6.0, ray_count=1)
    assert len(rays) == 1
    angle, distance, hit = rays[0]
    assert hit is False
    assert distance == pytest.approx(6.0)
    assert angle == pytest.approx(0.0)


def test_scan_to_rays_empty_sectors_inside_fov_report_misses():
    angles = np.linspace(-0.5, 0.5, 5)
    ranges = np.array([np.inf, np.inf, 3.0, np.inf, np.inf])
    rays = scan_to_rays(angles, ranges, pose_yaw=0.0, mount_yaw=0.0,
                        max_range=6.0, ray_count=5)
    assert [r[2] for r in rays] == [False, False, True, False, False]


def test_scan_to_rays_keeps_nearest_sample_per_sector():
    angles = np.linspace(-0.1, 0.1, 120)
    ranges = np.full(120, 5.0)
    ranges[60] = 2.0
    rays = scan_to_rays(angles, ranges, pose_yaw=0.0, mount_yaw=0.0,
                        max_range=6.0, ray_count=10)
    assert len(rays) == 10
    assert all(r[2] is True for r in rays)
    assert min(r[1] for r in rays) == pytest.approx(2.0)
    assert max(r[1] for r in rays) == pytest.approx(5.0)


def test_scan_to_rays_rejects_bad_args():
    angles = np.array([-0.1, 0.1])
    ranges = np.array([1.0, 1.0])
    with pytest.raises(ValueError):
        scan_to_rays(angles, ranges, pose_yaw=0.0, mount_yaw=0.0,
                     max_range=6.0, ray_count=0)
    with pytest.raises(ValueError):
        scan_to_rays(np.array([0.0]), np.array([1.0]), pose_yaw=0.0,
                     mount_yaw=0.0, max_range=6.0, ray_count=4)
    with pytest.raises(ValueError):
        scan_to_rays(angles, np.array([1.0]), pose_yaw=0.0, mount_yaw=0.0,
                     max_range=6.0, ray_count=4)
    with pytest.raises(ValueError):
        scan_to_rays(angles, ranges, pose_yaw=0.0, mount_yaw=0.0,
                     max_range=6.0, min_range=7.0, ray_count=4)


# --------------------------------------------------------------------------
# cloud_to_rays: front depth cloud -> fov-binned hit/miss rays
# --------------------------------------------------------------------------
def test_cloud_to_rays_projects_point_in_fov_and_fills_misses():
    pose = (1.0, 1.0, 0.0)
    points = np.array([
        [4.0, 1.0, 0.5],     # dead ahead, 3 m -> middle sector hit
        [-2.0, 1.0, 0.5],    # behind the drone -> outside the front FOV
    ])
    rays = cloud_to_rays(points, pose, fov_center_yaw=0.0, fov_width=HALF_PI,
                         max_range=6.0, min_range=0.1, ray_count=3)
    assert len(rays) == 3
    assert rays[1][2] is True
    assert rays[1][1] == pytest.approx(3.0)
    for index in (0, 2):
        assert rays[index][2] is False
        assert rays[index][1] == pytest.approx(6.0)


def test_cloud_to_rays_fov_rotates_with_heading():
    pose = (1.0, 1.0, HALF_PI)
    points = np.array([[1.0, 4.0, 0.5]])  # 3 m towards +y
    rays = cloud_to_rays(points, pose, fov_center_yaw=HALF_PI,
                         fov_width=HALF_PI, max_range=6.0,
                         min_range=0.1, ray_count=3)
    assert rays[1][2] is True
    assert rays[1][1] == pytest.approx(3.0)


def test_cloud_to_rays_ignores_points_beyond_max_range():
    pose = (1.0, 1.0, 0.0)
    points = np.array([[10.0, 1.0, 0.5]])  # 9 m > 6 m cutoff
    rays = cloud_to_rays(points, pose, fov_center_yaw=0.0, fov_width=HALF_PI,
                         max_range=6.0, min_range=0.1, ray_count=3)
    assert all(r[2] is False and r[1] == pytest.approx(6.0) for r in rays)


def test_cloud_to_rays_ignores_min_range_noise():
    pose = (1.0, 1.0, 0.0)
    points = np.array([[1.01, 1.0, 0.5]])  # 1 cm ahead -> sensor self-noise
    rays = cloud_to_rays(points, pose, fov_center_yaw=0.0, fov_width=HALF_PI,
                         max_range=6.0, min_range=0.1, ray_count=3)
    assert all(r[2] is False for r in rays)


def test_cloud_to_rays_ignores_nonfinite_points():
    pose = (1.0, 1.0, 0.0)
    points = np.array([
        [np.nan, np.nan, np.nan],
        [np.inf, 1.0, 0.5],
        [4.0, 1.0, 0.5],
    ])
    rays = cloud_to_rays(points, pose, fov_center_yaw=0.0, fov_width=HALF_PI,
                         max_range=6.0, min_range=0.1, ray_count=3)
    assert [r[2] for r in rays] == [False, True, False]


def test_cloud_to_rays_empty_cloud_is_all_misses():
    pose = (1.0, 1.0, 0.0)
    empty = np.empty((0, 3), dtype=np.float64)
    rays = cloud_to_rays(empty, pose, fov_center_yaw=0.0, fov_width=HALF_PI,
                         max_range=6.0, min_range=0.1, ray_count=4)
    assert len(rays) == 4
    assert all(r[2] is False and r[1] == pytest.approx(6.0) for r in rays)


def test_cloud_to_rays_keeps_nearest_point_per_sector():
    pose = (0.0, 0.0, 0.0)
    points = np.array([
        [4.0, 0.0, 0.5],
        [3.5, 0.0, 1.0],
        [3.0, 0.0, 0.2],
    ])
    rays = cloud_to_rays(points, pose, fov_center_yaw=0.0, fov_width=HALF_PI,
                         max_range=6.0, min_range=0.1, ray_count=3)
    assert rays[1][2] is True
    assert rays[1][1] == pytest.approx(3.0)


def test_cloud_to_rays_rejects_bad_args():
    pose = (0.0, 0.0, 0.0)
    points = np.array([[1.0, 0.0, 0.5]])
    with pytest.raises(ValueError):
        cloud_to_rays(np.array([1.0, 2.0]), pose, fov_center_yaw=0.0,
                      fov_width=HALF_PI, max_range=6.0, ray_count=3)
    with pytest.raises(ValueError):
        cloud_to_rays(points, pose, fov_center_yaw=0.0, fov_width=0.0,
                      max_range=6.0, ray_count=3)
    with pytest.raises(ValueError):
        cloud_to_rays(points, pose, fov_center_yaw=0.0,
                      fov_width=2.0 * math.pi + 0.1, max_range=6.0, ray_count=3)
    with pytest.raises(ValueError):
        cloud_to_rays(points, pose, fov_center_yaw=0.0, fov_width=HALF_PI,
                      max_range=6.0, ray_count=0)


# --------------------------------------------------------------------------
# RateGate: per-source rate limiting
# --------------------------------------------------------------------------
def test_rate_gate_allows_first_call_then_enforces_period():
    gate = RateGate(200_000_000)  # 5 Hz
    assert gate.allow(1_000) is True
    assert gate.allow(1_000 + 100_000_000) is False
    assert gate.allow(1_000 + 199_999_999) is False
    assert gate.allow(1_000 + 200_000_000) is True
    assert gate.allow(1_000 + 399_999_999) is False


def test_rate_gate_rejects_nonpositive_period():
    with pytest.raises(ValueError):
        RateGate(0)
    with pytest.raises(ValueError):
        RateGate(-5)


# --------------------------------------------------------------------------
# Contract with OccupancyMapper: fused rays land on the grid
# --------------------------------------------------------------------------
def test_fused_scan_marks_wall_on_occupancy_grid():
    mapper = OccupancyMapper(0.5, 10.0, 10.0, 8.0, 0.1)
    pose = (1.0, 1.0, 0.0)
    angles = np.array([-0.05, 0.0, 0.05])
    ranges = np.array([3.0, 3.0, 3.0])
    rays = scan_to_rays(angles, ranges, pose_yaw=0.0, mount_yaw=0.0,
                        max_range=6.0, ray_count=3)
    mapper.update(pose, rays)
    assert mapper.grid[2, 8] == OCCUPIED       # dead-ahead endpoint (x=4)
    assert mapper.grid[2, 4] == 0              # free along the ray
