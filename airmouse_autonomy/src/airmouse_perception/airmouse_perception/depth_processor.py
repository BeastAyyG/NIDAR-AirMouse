"""Depth image -> metric world points (ROS-independent core).

Camera model (per AirMouse design spec):
    * depth is an HxW metric depth image; 0 / NaN / inf are invalid.
    * pixel (u, v) back-projects in the camera frame to
      X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy, with Z the depth.
    * the camera looks along the drone +x axis, image y maps to world +y
      (down) and depth Z lies along +x. The remaining camera axis X becomes
      the point height z.
    * the drone-frame point is rotated by the drone heading theta
      (0 = +x, clockwise positive in the x-right / y-down world) and
      translated by the drone pose plus the camera offset.

Only numpy + stdlib are used; no rclpy import.
"""

from __future__ import annotations

import math

import numpy as np

__all__ = ["depth_to_points"]

DEFAULT_DEPTH_MIN = 0.2
DEFAULT_DEPTH_MAX = 8.0


def depth_to_points(
    depth,
    fx,
    fy,
    cx,
    cy,
    pose,
    cam_offset,
    depth_min: float = DEFAULT_DEPTH_MIN,
    depth_max: float = DEFAULT_DEPTH_MAX,
) -> np.ndarray:
    """Back-project a metric depth image into world-frame points.

    Args:
        depth: HxW array of metric depths. 0 / NaN / inf are invalid.
        fx, fy: focal lengths (pixels); must be non-zero.
        cx, cy: principal point (pixels).
        pose: (x, y, theta) drone pose in the world (x right, y down,
            theta 0 = +x, clockwise positive).
        cam_offset: (ox, oy) camera position relative to the drone.
        depth_min, depth_max: inclusive valid depth range (metres).

    Returns:
        Nx3 float array of world points (x, y, z). Invalid / out-of-range
        pixels are dropped. An empty image yields a (0, 3) array.

    Raises:
        ValueError: on malformed inputs (non-2D depth, zero focal length,
            wrong pose/offset length, or depth_min > depth_max).
    """
    depth_arr = np.asarray(depth, dtype=np.float64)
    if depth_arr.ndim != 2:
        raise ValueError("depth must be a 2D HxW array")
    if fx == 0 or fy == 0:
        raise ValueError("fx and fy must be non-zero")
    if depth_min > depth_max:
        raise ValueError("depth_min must be <= depth_max")

    pose_vals = np.asarray(pose, dtype=np.float64).ravel()
    offset_vals = np.asarray(cam_offset, dtype=np.float64).ravel()
    if pose_vals.size != 3:
        raise ValueError("pose must be (x, y, theta)")
    if offset_vals.size != 2:
        raise ValueError("cam_offset must be (ox, oy)")

    px, py, theta = (float(v) for v in pose_vals)
    ox, oy = (float(v) for v in offset_vals)

    valid = np.isfinite(depth_arr) & (depth_arr >= depth_min) & (depth_arr <= depth_max)
    rows, cols = np.nonzero(valid)  # rows = v, cols = u
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    z_depth = depth_arr[rows, cols]
    cam_x = (cols - cx) * z_depth / fx
    cam_y = (rows - cy) * z_depth / fy

    # Drone frame: depth along +x, image y along +y, image x is height z.
    drone_x = z_depth + ox
    drone_y = cam_y + oy
    height_z = cam_x

    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    world_x = px + cos_t * drone_x - sin_t * drone_y
    world_y = py + sin_t * drone_x + cos_t * drone_y

    return np.column_stack((world_x, world_y, height_z)).astype(np.float64, copy=False)