"""rclpy adapter for the AirMouse perception cores (perception_node).

Thin ROS 2 shell: decode the depth image, back-project it with the shared
:func:`airmouse_perception.depth_processor.depth_to_points` camera model using
the drone pose from TF, keep the flight-height collision band, summarise
left / centre / right obstacle distances with
:func:`airmouse_perception.obstacle_detection.detect_regions`, and publish
``airmouse_interfaces/ObstacleRegions`` on ``/obstacles``.

No perception algorithm lives here; only message decoding, TF lookup and
parameter wiring.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from airmouse_interfaces.msg import ObstacleRegions

from .depth_processor import depth_to_points
from .obstacle_detection import detect_regions
from .pointcloud_filter import filter_band, filter_floor_ceiling

DEPTH_TOPIC = "/camera/depth/image_rect_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
OBSTACLE_TOPIC = "/obstacles"
MAP_FRAME = "map"
BASE_FRAME = "base_link"
TF_TIMEOUT_S = 0.2
REGION_KEYS = ("left", "centre", "right")
MESSAGE_REGION_COUNT = 3
# Empty regions are published as this "far away" range (inf is not useful to
# downstream planners / RViz).
NO_OBSTACLE_RANGE_M = 1e3


def _depth_to_meters(msg: Image) -> np.ndarray:
    """Decode a 32FC1 or 16UC1 (millimetre) depth image into float metres."""
    encoding = msg.encoding.lower()
    if encoding == "32fc1":
        dtype, scale = np.float32, 1.0
    elif encoding in ("16uc1", "mono16"):
        dtype, scale = np.uint16, 1e-3
    else:
        raise ValueError(f"unsupported depth encoding {msg.encoding!r}")
    if msg.height <= 0 or msg.width <= 0:
        return np.empty((0, 0), dtype=np.float64)
    itemsize = np.dtype(dtype).itemsize
    pixels = np.ndarray(
        shape=(msg.height, msg.width),
        dtype=dtype,
        buffer=msg.data,
        strides=(msg.step, itemsize),
    )
    return pixels.astype(np.float64) * scale


def _intrinsics_from_camera_info(msg: CameraInfo) -> tuple[float, float, float, float]:
    """Extract (fx, fy, cx, cy) from the row-major 3x3 K matrix."""
    k = msg.k
    return float(k[0]), float(k[4]), float(k[2]), float(k[5])


def _quaternion_to_yaw(q) -> float:
    """Yaw (rotation about z) of a geometry_msgs Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _ordered_region_keys(summary: dict) -> list[str]:
    """Exactly three summary keys in left/centre/right order (padded if needed)."""
    keys = [key for key in REGION_KEYS if key in summary]
    if len(keys) != MESSAGE_REGION_COUNT:
        keys = list(summary.keys())[:MESSAGE_REGION_COUNT]
    while len(keys) < MESSAGE_REGION_COUNT:
        keys.append("")
    return keys


def _distance(value: float) -> float:
    return float(value) if math.isfinite(value) else NO_OBSTACLE_RANGE_M


class PerceptionNode(Node):
    """``perception_node``: depth -> obstacle region summary -> ``/obstacles``."""

    def __init__(self) -> None:
        super().__init__("perception_node")

        self.declare_parameter("depth_min_m", 0.2)
        self.declare_parameter("depth_max_m", 8.0)
        self.declare_parameter("floor_z_m", 0.15)
        self.declare_parameter("ceil_z_m", 2.2)
        self.declare_parameter("flight_band_zmin_m", 0.3)
        self.declare_parameter("flight_band_zmax_m", 2.0)
        self.declare_parameter("hfov_rad", 1.2)
        self.declare_parameter("regions", 3)
        self.declare_parameter("representative_percentile", 20.0)
        self.declare_parameter("cam_offset_x", 0.0)
        self.declare_parameter("cam_offset_y", 0.0)
        self.declare_parameter("map_frame", MAP_FRAME)
        self.declare_parameter("base_frame", BASE_FRAME)
        self.declare_parameter("tf_timeout_s", TF_TIMEOUT_S)

        self._depth_min = float(self.get_parameter("depth_min_m").value)
        self._depth_max = float(self.get_parameter("depth_max_m").value)
        self._floor_z = float(self.get_parameter("floor_z_m").value)
        self._ceil_z = float(self.get_parameter("ceil_z_m").value)
        self._zmin = float(self.get_parameter("flight_band_zmin_m").value)
        self._zmax = float(self.get_parameter("flight_band_zmax_m").value)
        self._hfov = float(self.get_parameter("hfov_rad").value)
        self._regions = int(self.get_parameter("regions").value)
        self._percentile = float(self.get_parameter("representative_percentile").value)
        self._cam_offset = (
            float(self.get_parameter("cam_offset_x").value),
            float(self.get_parameter("cam_offset_y").value),
        )
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter("tf_timeout_s").value)
        )

        self._intrinsics: tuple[float, float, float, float] | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._obstacle_pub = self.create_publisher(
            ObstacleRegions, OBSTACLE_TOPIC, 10
        )
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC,
                                 self._on_camera_info, sensor_qos)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, sensor_qos)

        self.get_logger().info(
            f"perception_node ready: {DEPTH_TOPIC} -> {OBSTACLE_TOPIC} "
            f"(camera info {CAMERA_INFO_TOPIC})"
        )
        if self._regions != MESSAGE_REGION_COUNT:
            self.get_logger().warn(
                f"regions={self._regions} but ObstacleRegions carries "
                f"{MESSAGE_REGION_COUNT} slots (left/centre/right)"
            )

    # ------------------------------------------------------------------ input
    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._intrinsics = _intrinsics_from_camera_info(msg)

    def _on_depth(self, msg: Image) -> None:
        if self._intrinsics is None:
            self.get_logger().warn(
                f"no camera intrinsics yet (waiting on {CAMERA_INFO_TOPIC})",
                throttle_duration_sec=5.0,
            )
            return
        try:
            depth = _depth_to_meters(msg)
            pose_tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                msg.header.stamp,
                timeout=self._tf_timeout,
            )
        except (ValueError, tf2_ros.TransformException) as exc:
            self.get_logger().warn(
                f"perception input rejected: {exc}", throttle_duration_sec=2.0
            )
            return
        if depth.size == 0:
            return

        translation = pose_tf.transform.translation
        pose = (
            float(translation.x),
            float(translation.y),
            _quaternion_to_yaw(pose_tf.transform.rotation),
        )
        fx, fy, cx, cy = self._intrinsics

        try:
            points = depth_to_points(
                depth, fx, fy, cx, cy, pose, self._cam_offset,
                depth_min=self._depth_min, depth_max=self._depth_max,
            )
            points = filter_floor_ceiling(points, self._floor_z, self._ceil_z)
            points = filter_band(points, self._zmin, self._zmax)
            summary = detect_regions(
                points, self._hfov, self._regions, self._percentile
            )
        except ValueError as exc:
            self.get_logger().warn(
                f"perception core rejected frame: {exc}", throttle_duration_sec=2.0
            )
            return

        self._obstacle_pub.publish(self._obstacle_message(summary))

    # ----------------------------------------------------------------- output
    @staticmethod
    def _obstacle_message(summary: dict) -> ObstacleRegions:
        keys = _ordered_region_keys(summary)
        empty = {"min": math.inf, "representative": math.inf, "count": 0}
        message = ObstacleRegions()
        message.min_distance = [
            _distance(summary.get(key, empty)["min"]) for key in keys
        ]
        message.representative = [
            _distance(summary.get(key, empty)["representative"]) for key in keys
        ]
        message.count = [int(summary.get(key, empty)["count"]) for key in keys]
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
