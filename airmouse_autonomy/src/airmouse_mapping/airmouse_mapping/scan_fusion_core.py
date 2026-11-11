"""Scan-fusion core: 2D LiDAR scans + front depth cloud -> mapper rays.

ROS-independent (numpy + stdlib only). The rclpy adapter
(``scan_fusion_node``) converts ROS messages, looks up TF and calls these
functions; everything else is testable offline.

Contract matches :class:`airmouse_mapping.occupancy_core.OccupancyMapper`:
metres, x right, y DOWN, rays are ``(angle, distance, hit)`` with ABSOLUTE
map-frame angles (theta 0 = +x, clockwise on screen). Every sensor sweep is
binned into a fixed number of sectors spanning that sensor's field of view;
a sector with returns reports the nearest one as a hit, an empty sector
reports a miss at ``max_range`` so free space gets carved. Nothing outside
the sensor FOV is ever claimed free.
"""
from __future__ import annotations

import math

import numpy as np

__all__ = ["RateGate", "cloud_to_rays", "optical_to_body", "scan_to_rays"]

TWO_PI = 2.0 * math.pi


def optical_to_body(points) -> np.ndarray:
    """Re-express an RGB-D optical-frame cloud in body axes.

    Camera/optical frames (e.g. ``camera_depth_optical_frame``) are
    x right, y DOWN, z forward; the mapper works in body axes x forward,
    y left, z up. Without this permutation the depth axis is mistaken for
    height, so a 3 m wall lands as noise at the drone instead of a hit.
    """
    arr = np.asarray(points, dtype=np.float64)
    if arr.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError("points must be an Nx3 array")
    out = np.empty_like(arr)
    out[:, 0] = arr[:, 2]      # forward
    out[:, 1] = -arr[:, 0]     # left
    out[:, 2] = -arr[:, 1]     # up
    return out


def _validated_range(max_range: float, min_range: float) -> tuple[float, float]:
    lo, hi = float(min_range), float(max_range)
    if not (math.isfinite(lo) and math.isfinite(hi)):
        raise ValueError("min_range/max_range must be finite")
    if lo < 0.0 or hi <= 0.0:
        raise ValueError("min_range must be >= 0 and max_range > 0")
    if lo >= hi:
        raise ValueError("min_range must be < max_range")
    return lo, hi


def _validated_ray_count(ray_count: int) -> int:
    count = int(ray_count)
    if count < 1:
        raise ValueError(f"ray_count must be >= 1, got {ray_count!r}")
    return count


def _ray(angle: float, sector_min: np.ndarray, index: int,
         max_range: float, offset: float) -> tuple[float, float, bool]:
    """One sector's ray: nearest hit if any return, else a max-range miss."""
    sector_angle = float(offset + angle)
    if np.isfinite(sector_min[index]):
        return (sector_angle, float(sector_min[index]), True)
    return (sector_angle, float(max_range), False)


def scan_to_rays(angles, ranges, *, pose_yaw: float, mount_yaw: float,
                 max_range: float, min_range: float = 0.05,
                 ray_count: int = 120) -> list[tuple[float, float, bool]]:
    """Decimate one LaserScan into ``ray_count`` absolute map-frame rays.

    ``angles`` are the scan's sample angles in its own frame (increasing,
    BeamScan convention) and ``ranges`` the matching distances. Invalid
    samples (0, NaN, inf, below ``min_range``) are dropped; finite ranges
    beyond ``max_range`` count as misses at the cap. Every sector of the
    scan FOV yields exactly one ray, so a silent LiDAR still carves free
    space ahead of the drone.
    """
    a = np.asarray(angles, dtype=np.float64).ravel()
    r = np.asarray(ranges, dtype=np.float64).ravel()
    if a.size != r.size:
        raise ValueError(f"angles/ranges length mismatch: {a.size} != {r.size}")
    if a.size < 2:
        raise ValueError("scan needs at least 2 samples")
    count = _validated_ray_count(ray_count)
    lo, hi = _validated_range(max_range, min_range)
    if not (math.isfinite(pose_yaw) and math.isfinite(mount_yaw)):
        raise ValueError("pose_yaw/mount_yaw must be finite")

    angle_min = float(a[0])
    angle_max = float(a[-1])
    if not (math.isfinite(angle_min) and math.isfinite(angle_max)
            and angle_max > angle_min):
        raise ValueError("scan angles must be finite and increasing")
    width = (angle_max - angle_min) / count

    usable = np.isfinite(a) & np.isfinite(r) & (r >= lo)
    distances = r[usable]
    sectors = np.clip(
        ((a[usable] - angle_min) / width).astype(np.int64), 0, count - 1
    )
    in_range = distances <= hi
    sector_min = np.full(count, np.inf, dtype=np.float64)
    np.minimum.at(sector_min, sectors[in_range], distances[in_range])

    offset = float(pose_yaw) + float(mount_yaw)
    return [
        _ray(angle_min + (index + 0.5) * width, sector_min, index, hi, offset)
        for index in range(count)
    ]


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % TWO_PI - math.pi


def cloud_to_rays(points, pose: tuple[float, float, float], *,
                  fov_center_yaw: float, fov_width: float,
                  max_range: float, min_range: float = 0.1,
                  ray_count: int = 90) -> list[tuple[float, float, bool]]:
    """Project a front depth cloud (map frame, height band already applied).

    Points are Nx3 world points; z is ignored (2D projection). Returns are
    binned into ``ray_count`` sectors spanning ``fov_width`` centred on
    ``fov_center_yaw`` (absolute map yaw); the nearest return per sector is
    the hit, empty sectors are max-range misses. Points behind the FOV,
    closer than ``min_range`` (self-noise), beyond ``max_range`` or
    non-finite are ignored.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        pts = np.empty((0, 3), dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points must be an Nx3 array")
    count = _validated_ray_count(ray_count)
    lo, hi = _validated_range(max_range, min_range)
    width_total = float(fov_width)
    if not (math.isfinite(width_total) and 0.0 < width_total <= TWO_PI + 1e-9):
        raise ValueError("fov_width must be in (0, 2*pi]")
    if not math.isfinite(fov_center_yaw):
        raise ValueError("fov_center_yaw must be finite")
    px, py = float(pose[0]), float(pose[1])

    dx = pts[:, 0] - px
    dy = pts[:, 1] - py
    distance = np.hypot(dx, dy)
    angle = np.arctan2(dy, dx)
    half = 0.5 * width_total
    relative = np.mod(angle - float(fov_center_yaw) + math.pi, TWO_PI) - math.pi

    usable = (np.isfinite(distance) & np.isfinite(angle)
              & (distance >= lo) & (np.abs(relative) <= half))
    width = width_total / count
    sectors = np.clip(
        ((relative[usable] + half) / width).astype(np.int64), 0, count - 1
    )
    distances = distance[usable]
    in_range = distances <= hi
    sector_min = np.full(count, np.inf, dtype=np.float64)
    np.minimum.at(sector_min, sectors[in_range], distances[in_range])

    return [
        _ray(-half + (index + 0.5) * width, sector_min, index, hi,
             float(fov_center_yaw))
        for index in range(count)
    ]


class RateGate:
    """Monotonic per-source rate limiter (first call always allowed)."""

    def __init__(self, period_ns: int) -> None:
        period = int(period_ns)
        if period <= 0:
            raise ValueError(f"period_ns must be > 0, got {period_ns!r}")
        self._period_ns = period
        self._last_ns: int | None = None

    def allow(self, now_ns: int) -> bool:
        now = int(now_ns)
        if self._last_ns is None or now - self._last_ns >= self._period_ns:
            self._last_ns = now
            return True
        return False
