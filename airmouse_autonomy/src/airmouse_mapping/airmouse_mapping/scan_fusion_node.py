"""rclpy adapter for scan-fusion mapping (scan_fusion_node).

Fuses four 2D LiDAR ``sensor_msgs/LaserScan`` topics with the front 3D
sensor (``sensor_msgs/PointCloud2``) into one 2D occupancy grid, the
hardware counterpart of ``mapping_node``'s single-depth-camera path.

Per source: rate-gate the message, look up the drone pose in the map frame
via tf2, convert the sweep into absolute map-frame rays
(:mod:`airmouse_mapping.scan_fusion_core`) and integrate them with
:class:`airmouse_mapping.occupancy_core.OccupancyMapper`. The cloud is
transformed into the map frame and height-band filtered
(``flight_band_zmin_m``..``flight_band_zmax_m``) before projection. Publishes
``nav_msgs/OccupancyGrid`` on ``/map`` and exports PGM + YAML on shutdown.

Mount yaws (degrees, x-right / y-down frame) are parameters so a rig can be
commissioned without TF gymnastics while LiDAR mounts are being measured;
``use_scans`` / ``use_cloud`` let the node run on a camera-only bench.
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
from sensor_msgs.msg import LaserScan, PointCloud2

from airmouse_perception.pointcloud_filter import filter_band

from .grid_manager import origin_for
from .map_exporter import save_pgm_yaml
from .occupancy_core import OccupancyMapper
from .ros_convert import pointcloud_to_xyz, quaternion_to_yaw, transform_xyz
from .scan_fusion_core import RateGate, cloud_to_rays, optical_to_body, scan_to_rays

MAP_TOPIC = "/map"
MAP_FRAME = "map"
BASE_FRAME = "base_link"
TF_TIMEOUT_S = 0.2

# LiDAR name -> (default topic, default mount yaw in degrees, x-right/y-down).
SCAN_MOUNTS: dict[str, tuple[str, float]] = {
    "front": ("/scan_front", 0.0),
    "left": ("/scan_left", 90.0),
    "rear": ("/scan_rear", 180.0),
    "right": ("/scan_right", -90.0),
}


class ScanFusionNode(Node):
    """``scan_fusion_node``: 4 LiDARs + front cloud -> occupancy grid -> /map."""

    def __init__(self) -> None:
        super().__init__("scan_fusion_node")

        self.declare_parameter("cell_size", 0.10)
        self.declare_parameter("width_m", 15.0)
        self.declare_parameter("height_m", 15.0)
        self.declare_parameter("max_range_m", 6.0)
        self.declare_parameter("clearance_m", 0.10)
        self.declare_parameter("scan_rays", 120)
        self.declare_parameter("cloud_rays", 90)
        self.declare_parameter("scan_min_range_m", 0.05)
        self.declare_parameter("cloud_min_range_m", 0.10)
        self.declare_parameter("cloud_topic", "/camera/depth/color/points")
        self.declare_parameter("cloud_frame", "optical")
        self.declare_parameter("cloud_mount_yaw_deg", 0.0)
        self.declare_parameter("cloud_fov_deg", 86.0)
        self.declare_parameter("flight_band_zmin_m", 0.3)
        self.declare_parameter("flight_band_zmax_m", 2.0)
        self.declare_parameter("use_scans", True)
        self.declare_parameter("use_cloud", True)
        self.declare_parameter("update_rate_hz", 5.0)
        self.declare_parameter("map_frame", MAP_FRAME)
        self.declare_parameter("base_frame", BASE_FRAME)
        self.declare_parameter("tf_timeout_s", TF_TIMEOUT_S)
        self.declare_parameter("map_rate_hz", 2.0)
        self.declare_parameter("save_dir", "maps")
        for name in SCAN_MOUNTS:
            self.declare_parameter(f"scan_{name}_topic", SCAN_MOUNTS[name][0])
            self.declare_parameter(f"scan_{name}_mount_yaw_deg",
                                   SCAN_MOUNTS[name][1])

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._zmin = float(self.get_parameter("flight_band_zmin_m").value)
        self._zmax = float(self.get_parameter("flight_band_zmax_m").value)
        self._scan_rays = int(self.get_parameter("scan_rays").value)
        self._cloud_rays = int(self.get_parameter("cloud_rays").value)
        self._scan_min_range = float(self.get_parameter("scan_min_range_m").value)
        self._cloud_min_range = float(self.get_parameter("cloud_min_range_m").value)
        self._cloud_frame = str(self.get_parameter("cloud_frame").value)
        if self._cloud_frame not in ("optical", "body_z_up"):
            raise ValueError(
                "cloud_frame must be 'optical' (RGB-D, x right/y down/z forward) "
                f"or 'body_z_up', got {self._cloud_frame!r}"
            )
        self._cloud_mount_yaw = math.radians(
            float(self.get_parameter("cloud_mount_yaw_deg").value)
        )
        self._cloud_fov = math.radians(
            float(self.get_parameter("cloud_fov_deg").value)
        )
        self._use_scans = bool(self.get_parameter("use_scans").value)
        self._use_cloud = bool(self.get_parameter("use_cloud").value)
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
        self._mapper.height_band = (self._zmin, self._zmax)

        update_rate = max(1e-3, float(self.get_parameter("update_rate_hz").value))
        self._gates = {
            name: RateGate(int(1e9 / update_rate))
            for name in (*SCAN_MOUNTS.keys(), "cloud")
        }

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        scan_qos = QoSProfile(
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

        if self._use_scans:
            for name in SCAN_MOUNTS:
                topic = str(self.get_parameter(f"scan_{name}_topic").value)
                mount_yaw = math.radians(
                    float(self.get_parameter(f"scan_{name}_mount_yaw_deg").value)
                )
                self.create_subscription(
                    LaserScan, topic,
                    self._make_scan_callback(name, mount_yaw), scan_qos,
                )
        if self._use_cloud:
            cloud_topic = str(self.get_parameter("cloud_topic").value)
            self.create_subscription(
                PointCloud2, cloud_topic, self._on_cloud, scan_qos
            )

        map_rate = max(1e-3, float(self.get_parameter("map_rate_hz").value))
        self.create_timer(1.0 / map_rate, self._publish_map)

        self._updated: set[str] = set()
        self.get_logger().info(
            f"scan_fusion_node ready: {MAP_TOPIC} @ {map_rate:.1f} Hz, "
            f"fusing {update_rate:.1f} Hz sources "
            f"(scans={self._use_scans}, cloud={self._use_cloud}), "
            f"grid {self._mapper.grid.shape[1]}x{self._mapper.grid.shape[0]} "
            f"@ {self._mapper.cell_size:.2f} m/cell"
        )

    # ----------------------------------------------------------------- pose
    def _pose_at(self, stamp) -> tuple[float, float, float] | None:
        """Drone pose (x, y, yaw) in the map frame at the given stamp."""
        try:
            pose_tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, stamp,
                timeout=self._tf_timeout,
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"TF {self._map_frame}<-{self._base_frame} failed: {exc}"
            )
            return None
        translation = pose_tf.transform.translation
        return (
            float(translation.x),
            float(translation.y),
            quaternion_to_yaw(pose_tf.transform.rotation),
        )

    def _integrate(self, source: str, pose, rays) -> None:
        self._mapper.update(pose, rays)
        if source not in self._updated:
            self._updated.add(source)
            self.get_logger().info(
                f"first {source} sweep integrated: {len(rays)} rays"
            )

    # ----------------------------------------------------------------- input
    def _make_scan_callback(self, source: str, mount_yaw: float):
        def _on_scan(msg: LaserScan) -> None:
            if not self._gates[source].allow(
                    self.get_clock().now().nanoseconds):
                return
            ranges = np.asarray(msg.ranges, dtype=np.float64)
            if ranges.size < 2:
                return
            angles = (float(msg.angle_min)
                      + np.arange(ranges.size) * float(msg.angle_increment))
            pose = self._pose_at(msg.header.stamp)
            if pose is None:
                return
            try:
                rays = scan_to_rays(
                    angles, ranges,
                    pose_yaw=pose[2], mount_yaw=mount_yaw,
                    max_range=self._mapper.max_range,
                    min_range=self._scan_min_range,
                    ray_count=self._scan_rays,
                )
            except ValueError as exc:
                self.get_logger().error(f"cannot fuse {source} scan: {exc}")
                return
            self._integrate(source, pose, rays)

        return _on_scan

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not self._gates["cloud"].allow(self.get_clock().now().nanoseconds):
            return
        try:
            sensor_points = pointcloud_to_xyz(msg)
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
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"cloud TF lookup failed: {exc}")
            return
        pose = self._pose_at(msg.header.stamp)
        if pose is None:
            return

        translation = cloud_tf.transform.translation
        # Optical clouds carry an axis permutation, not a yaw: convert the
        # axes first, then rotate with the (yaw-only) drone pose.
        body_points = (optical_to_body(sensor_points)
                       if self._cloud_frame == "optical" else sensor_points)
        map_points = transform_xyz(
            body_points,
            (translation.x, translation.y, translation.z),
            pose[2],
        )
        map_points = filter_band(map_points, self._zmin, self._zmax)
        try:
            rays = cloud_to_rays(
                map_points, pose,
                fov_center_yaw=pose[2] + self._cloud_mount_yaw,
                fov_width=self._cloud_fov,
                max_range=self._mapper.max_range,
                min_range=self._cloud_min_range,
                ray_count=self._cloud_rays,
            )
        except ValueError as exc:
            self.get_logger().error(f"cannot fuse cloud: {exc}")
            return
        self._integrate("cloud", pose, rays)

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
        base = os.path.join(self._save_dir, f"airmouse_fusion_map_{stamp}")
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
        node = ScanFusionNode()
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
