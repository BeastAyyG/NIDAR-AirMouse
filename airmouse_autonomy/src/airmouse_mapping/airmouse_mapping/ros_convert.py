"""ROS message <-> numpy conversions shared by the mapping adapters.

Pure conversion helpers only (no node logic): quaternion yaw, PointCloud2
xyz decode, yaw+translate transform to the map frame. numpy + stdlib + ROS
message types; the occupancy cores themselves stay ROS-free.
"""
from __future__ import annotations

import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

__all__ = ["pointcloud_to_xyz", "quaternion_to_yaw", "transform_xyz"]


def quaternion_to_yaw(q) -> float:
    """Yaw (rotation about z) of a geometry_msgs Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pointcloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Decode the x/y/z fields of a PointCloud2 into an Nx3 float64 array."""
    fields = {field.name: field for field in msg.fields}
    missing = [name for name in ("x", "y", "z") if name not in fields]
    if missing:
        raise ValueError(f"point cloud is missing fields: {missing}")

    endian = ">" if msg.is_bigendian else "<"
    names, formats, offsets = [], [], []
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype == PointField.FLOAT32:
            fmt = "f4"
        elif field.datatype == PointField.FLOAT64:
            fmt = "f8"
        else:
            raise ValueError(
                f"field {name!r} must be float32/float64, got datatype {field.datatype}"
            )
        names.append(name)
        formats.append(endian + fmt)
        offsets.append(int(field.offset))

    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": offsets,
        "itemsize": int(msg.point_step),
    })
    count = int(msg.width) * int(msg.height)
    available = len(msg.data) // max(1, int(msg.point_step))
    count = min(count, available)
    if count <= 0:
        return np.empty((0, 3), dtype=np.float64)

    raw = np.frombuffer(msg.data, dtype=dtype, count=count)
    return np.column_stack(
        (raw["x"], raw["y"], raw["z"])
    ).astype(np.float64, copy=False)


def transform_xyz(points: np.ndarray, translation_xyz: tuple[float, float, float],
                  yaw: float) -> np.ndarray:
    """Rotate (yaw) + translate map<-sensor points; z is only translated."""
    tx, ty, tz = translation_xyz
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    out = np.empty_like(points)
    out[:, 0] = tx + cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
    out[:, 1] = ty + sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
    out[:, 2] = tz + points[:, 2]
    return out
