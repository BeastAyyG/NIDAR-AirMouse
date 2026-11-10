"""rclpy adapter for the AirMouse occupancy mapper (mapping_node).

Thin ROS 2 shell around the ROS-independent cores: subscribe a depth
PointCloud2, transform it into the map frame with tf2, keep the flight-height
band, bin the points into polar range sectors around the drone pose, feed the
sweep to :class:`airmouse_mapping.occupancy_core.OccupancyMapper`, then
publish ``nav_msgs/OccupancyGrid`` on ``/map``. On shutdown the grid is
exported to PGM + YAML (nav2 map_server format).

All geometry/occupancy logic lives in the cores; this module only does ROS
message <-> numpy conversion, TF lookup and parameter wiring.
"""

from __future__ import annotations

import datetime
import math
import os

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField

from airmouse_perception.pointcloud_filter import filter_band

from .grid_manager import origin_for
from .map_exporter import save_pgm_yaml
from .occupancy_core import OccupancyMapper

CLOUD_TOPIC = "/camera/depth/color/points"
MAP_TOPIC = "/map"
MAP_FRAME = "map"
BASE_FRAME = "base_link"
TF_TIMEOUT_S = 0.2
TWO_PI = 2.0 * math.pi


def _quaternion_to_yaw(q) -> float:
    """Yaw (rotation about z) of a geometry_msgs Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
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


def _transform_xyz(points: np.ndarray, translation_xyz: tuple[float, float, float],
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


def _polar_rays(points: np.ndarray, pose: tuple[float, float, float],
                max_range: float, ray_count: int) -> list[tuple[float, float, bool]]:
    """Bin world points into ``ray_count`` polar sectors around the pose.

    Each sector reports the minimum point distance as a hit; sectors with no
    in-range points report ``(sector_angle, max_range, hit=False)``. Sector
    angles are absolute (theta 0 = +x, clockwise / y-down), matching
    ``OccupancyMapper.update`` ray expectations.
    """
    px, py, theta = pose
    sector_width = TWO_PI / ray_count
    base_angle = theta - math.pi
    min_distance = np.full(ray_count, np.inf, dtype=np.float64)

    if points.shape[0]:
        dx = points[:, 0] - px
        dy = points[:, 1] - py
        distance = np.hypot(dx, dy)
        in_range = distance <= max_range
        if np.any(in_range):
            angles = np.arctan2(dy[in_range], dx[in_range])
            relative = np.mod(angles - base_angle, TWO_PI)
            sectors = np.clip(
                np.floor(relative / sector_width).astype(np.int64),
                0, ray_count - 1,
            )
            np.minimum.at(min_distance, sectors, distance[in_range])

    rays: list[tuple[float, float, bool]] = []
    for index in range(ray_count):
        angle = base_angle + (index + 0.5) * sector_width
        sector_min = min_distance[index]
        if np.isfinite(sector_min):
            rays.append((float(angle), float(sector_min), True))
        else:
            rays.append((float(angle), float(max_range), False))
    return rays


class MappingNode(Node):
    """``mapping_node``: sensor sweep -> occupancy grid -> ``/map``."""

    def __init__(self) -> None:
        super().__init__("mapping_node")

        self.declare_parameter("cell_size", 0.10)
        self.declare_parameter("width_m", 15.0)
        self.declare_parameter("height_m", 15.0)
        self.declare_parameter("max_range_m", 6.0)
        self.declare_parameter("clearance_m", 0.10)
        self.declare_parameter("lidar_rays", 120)
        self.declare_parameter("save_dir", "maps")
        # Flight-height band (perception filters upstream of the 2D projection).
        self.declare_parameter("flight_band_zmin_m", 0.3)
        self.declare_parameter("flight_band_zmax_m", 2.0)
        # Frame + cadence wiring.
        self.declare_parameter("map_frame", MAP_FRAME)
        self.declare_parameter("base_frame", BASE_FRAME)
        self.declare_parameter("cloud_rate_hz", 5.0)
        self.declare_parameter("map_rate_hz", 2.0)
        self.declare_parameter("tf_timeout_s", TF_TIMEOUT_S)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._zmin = float(self.get_parameter("flight_band_zmin_m").value)
        self._zmax = float(self.get_parameter("flight_band_zmax_m").value)
        self._ray_count = int(self.get_parameter("lidar_rays").value)
        if self._ray_count < 1:
            raise ValueError(f"lidar_rays must be >= 1, got {self._ray_count}")
        self._save_dir = str(self.get_parameter("save_dir").value)
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter("tf_timeout_s").value)
        )

        self._mapper = OccupancyMapper(
            cell_size=float(self.get_parameter("cell_size").value),
            width_m=float(self.get_parameter("width_m").value),
            height_m=float(self.get_parameter("height_m").value),
            max_range=float(self.get_parameter("max_range_m").value),
            clearance=float(self.get_parameter("clearance_m").value),
        )
        # Informational hook (perception filters the band before projection).
        self._mapper.height_band = (self._zmin, self._zmax)

        cloud_rate = max(1e-3, float(self.get_parameter("cloud_rate_hz").value))
        self._cloud_period_ns = int(1e9 / cloud_rate)
        self._last_cloud_ns: int | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, MAP_TOPIC, map_qos)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._on_cloud, cloud_qos)

        map_rate = max(1e-3, float(self.get_parameter("map_rate_hz").value))
        self.create_timer(1.0 / map_rate, self._publish_map)

        self.get_logger().info(
            f"mapping_node ready: {MAP_TOPIC} @ {map_rate:.1f} Hz, "
            f"cloud {CLOUD_TOPIC} @ {cloud_rate:.1f} Hz, "
            f"grid {self._mapper.grid.shape[1]}x{self._mapper.grid.shape[0]} "
            f"@ {self._mapper.cell_size:.2f} m/cell"
        )

    # ------------------------------------------------------------------ input
    def _on_cloud(self, msg: PointCloud2) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_cloud_ns is not None:
            if now_ns - self._last_cloud_ns < self._cloud_period_ns:
                return
        self._last_cloud_ns = now_ns

        try:
            sensor_points = _pointcloud2_to_xyz(msg)
        except ValueError as exc:
            self.get_logger().error(f"cannot decode point cloud: {exc}")
            return

        try:
            cloud_tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                msg.header.frame_id or self._base_frame,
                msg.header.stamp,
                timeout=self._tf_timeout,
            )
            pose_tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                msg.header.stamp,
                timeout=self._tf_timeout,
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return

        translation = cloud_tf.transform.translation
        map_points = _transform_xyz(
            sensor_points,
            (translation.x, translation.y, translation.z),
            _quaternion_to_yaw(cloud_tf.transform.rotation),
        )
        map_points = filter_band(map_points, self._zmin, self._zmax)

        pose_translation = pose_tf.transform.translation
        pose = (
            float(pose_translation.x),
            float(pose_translation.y),
            _quaternion_to_yaw(pose_tf.transform.rotation),
        )
        rays = _polar_rays(
            map_points, pose, self._mapper.max_range, self._ray_count
        )
        self._mapper.update(pose, rays)

    # ----------------------------------------------------------------- output
    def _publish_map(self) -> None:
        if not rclpy.ok():
            return
        grid = self._mapper.grid
        rows, cols = grid.shape
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        message.info.resolution = float(self._mapper.cell_size)
        message.info.width = int(cols)
        message.info.height = int(rows)
        origin = origin_for(0.0, 0.0, 0.0)
        message.info.origin.position.x = origin[0]
        message.info.origin.position.y = origin[1]
        message.info.origin.position.z = 0.0
        message.info.origin.orientation.w = 1.0
        message.data = grid.astype(np.int8).ravel().tolist()
        self._map_pub.publish(message)

    def save_map(self) -> None:
        """Export the final grid as PGM + YAML under ``save_dir``."""
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self._save_dir, f"airmouse_map_{stamp}")
        try:
            pgm_path, yaml_path = save_pgm_yaml(
                self._mapper.grid,
                self._mapper.cell_size,
                origin_for(0.0, 0.0, 0.0),
                base,
            )
        except OSError as exc:
            self.get_logger().error(f"failed to save map to {self._save_dir}: {exc}")
            return
        self.get_logger().info(f"saved map: {pgm_path} + {yaml_path}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MappingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.save_map()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
