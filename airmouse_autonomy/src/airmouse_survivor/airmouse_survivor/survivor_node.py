"""rclpy adapter for the AirMouse survivor detector + tracker (survivor_node).

Thin ROS 2 shell around the ROS-independent cores: decode the RGB + depth
images, back-project colour-blob detections with
:class:`airmouse_survivor.color_blob_detector.detect_survivors` using the drone
pose from TF, feed them to
:class:`airmouse_survivor.survivor_tracker.SurvivorTracker`, and republish the
tracker records as ``airmouse_interfaces/SurvivorTrack`` messages on
``/survivors`` at the configured cadence (one message per track, matching the
``/survivors`` consumers in exploration / mission).

No detection or tracking logic lives here; only message decoding, TF lookup
and parameter wiring.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_ros
from airmouse_interfaces.msg import SurvivorTrack
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from .color_blob_detector import detect_survivors
from .survivor_tracker import SurvivorTracker

NODE_NAME = "survivor_node"
COLOR_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
TRACKS_TOPIC = "/survivors"

MAP_FRAME = "map"
BASE_FRAME = "base_link"
TF_TIMEOUT_S = 0.2

# Encoding -> (channels, is_bgr) for the supported colour image encodings.
_COLOR_ENCODINGS = {
    "rgb8": (3, False),
    "rgba8": (4, False),
    "bgr8": (3, True),
    "bgra8": (4, True),
}


def _color_to_rgb(msg: Image) -> np.ndarray:
    """Decode an rgb8/rgba8/bgr8/bgra8 Image into an HxWx3 uint8 RGB array."""
    encoding = msg.encoding.lower()
    config = _COLOR_ENCODINGS.get(encoding)
    if config is None:
        raise ValueError(f"unsupported colour encoding {msg.encoding!r}")
    channels, is_bgr = config
    if msg.height <= 0 or msg.width <= 0:
        return np.empty((0, 0, 3), dtype=np.uint8)
    pixels = np.ndarray(
        shape=(msg.height, msg.width, channels),
        dtype=np.uint8,
        buffer=msg.data,
        strides=(msg.step, channels, 1),
    ).copy()
    rgb = pixels[:, :, :3]
    if is_bgr:
        rgb = rgb[:, :, ::-1]
    return np.ascontiguousarray(rgb)


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


def _quaternion_to_yaw(q) -> float:
    """Yaw (rotation about z) of a geometry_msgs Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def _stamp_s(msg) -> float:
    return int(msg.header.stamp.sec) + int(msg.header.stamp.nanosec) * 1e-9


class SurvivorNode(Node):
    """``survivor_node``: RGB-D images -> blob detections -> tracker -> ``/survivors``."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        # Detection parameters (config/survivor.yaml + perception depth band).
        self._detect_range_m = float(
            self.declare_parameter("detect_range_m", 5.0).value)
        self._depth_min_m = float(
            self.declare_parameter("depth_min_m", 0.2).value)
        self._min_pixels = int(self.declare_parameter("min_pixels", 60).value)
        self._max_sync_dt_ns = int(
            float(self.declare_parameter("max_sync_dt_s", 0.15).value) * 1e9)
        self._cam_offset = (
            float(self.declare_parameter("cam_offset_x", 0.0).value),
            float(self.declare_parameter("cam_offset_y", 0.0).value),
        )
        self._map_frame = str(self.declare_parameter("map_frame", MAP_FRAME).value)
        self._base_frame = str(self.declare_parameter("base_frame", BASE_FRAME).value)
        self._tf_timeout = Duration(
            seconds=float(self.declare_parameter("tf_timeout_s", TF_TIMEOUT_S).value))
        publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 2.0).value)

        # Tracker parameters (mirrors config/survivor.yaml).
        tracker_params = {
            "merge_radius_m": float(
                self.declare_parameter("merge_radius_m", 1.2).value),
            "confirm_confidence": float(
                self.declare_parameter("confirm_confidence", 0.85).value),
            "confirm_sightings": int(
                self.declare_parameter("confirm_sightings", 3).value),
            "decay_per_sec": float(
                self.declare_parameter("decay_per_sec", 0.02).value),
            "drop_confidence": float(
                self.declare_parameter("drop_confidence", 0.12).value),
        }
        self._tracker = SurvivorTracker(tracker_params)

        self._intrinsics: tuple[float, float, float, float] | None = None
        self._depth: np.ndarray | None = None
        self._depth_stamp_ns: int | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._track_pub = self.create_publisher(SurvivorTrack, TRACKS_TOPIC, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC,
                                 self._on_camera_info, sensor_qos)
        self.create_subscription(Image, COLOR_TOPIC, self._on_color, sensor_qos)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, sensor_qos)

        self.create_timer(1.0 / max(1e-3, publish_rate_hz), self._publish_tracks)
        self.get_logger().info(
            f"{NODE_NAME} ready: {COLOR_TOPIC} + {DEPTH_TOPIC} -> "
            f"{TRACKS_TOPIC} @ {publish_rate_hz:.1f} Hz"
        )

    # ------------------------------------------------------------------ input
    def _on_camera_info(self, msg: CameraInfo) -> None:
        k = msg.k
        self._intrinsics = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))

    def _on_depth(self, msg: Image) -> None:
        try:
            self._depth = _depth_to_meters(msg)
        except ValueError as exc:
            self.get_logger().warn(
                f"depth frame rejected: {exc}", throttle_duration_sec=2.0)
            return
        self._depth_stamp_ns = _stamp_ns(msg)

    def _on_color(self, msg: Image) -> None:
        if self._intrinsics is None:
            self.get_logger().warn(
                f"no camera intrinsics yet (waiting on {CAMERA_INFO_TOPIC})",
                throttle_duration_sec=5.0)
            return
        if self._depth is None or self._depth_stamp_ns is None:
            self.get_logger().warn(
                f"no depth frame yet (waiting on {DEPTH_TOPIC})",
                throttle_duration_sec=5.0)
            return
        if abs(_stamp_ns(msg) - self._depth_stamp_ns) > self._max_sync_dt_ns:
            self.get_logger().warn(
                "dropping colour frame: depth frame is stale",
                throttle_duration_sec=2.0)
            return

        try:
            rgb = _color_to_rgb(msg)
            pose_tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, msg.header.stamp,
                timeout=self._tf_timeout)
        except (ValueError, tf2_ros.TransformException) as exc:
            self.get_logger().warn(
                f"survivor input rejected: {exc}", throttle_duration_sec=2.0)
            return
        if rgb.size == 0:
            return

        translation = pose_tf.transform.translation
        pose = (
            float(translation.x),
            float(translation.y),
            _quaternion_to_yaw(pose_tf.transform.rotation),
        )
        fx, fy, cx, cy = self._intrinsics
        try:
            detections = detect_survivors(
                rgb, self._depth, fx, fy, cx, cy, pose, self._cam_offset,
                min_pixels=self._min_pixels,
                depth_min=self._depth_min_m,
                depth_max=self._detect_range_m,
            )
        except ValueError as exc:
            self.get_logger().warn(
                f"colour blob detector rejected frame: {exc}",
                throttle_duration_sec=2.0)
            return

        now = _stamp_s(msg)
        self._tracker.update(
            [(x, y, confidence, now) for x, y, confidence in detections], now)

    # ----------------------------------------------------------------- output
    def _publish_tracks(self) -> None:
        if not rclpy.ok():
            return
        for record in self._tracker.tracks:
            message = SurvivorTrack()
            message.survivor_id = int(record["survivor_id"])
            message.detected = bool(record["detected"])
            message.position_x = float(record["position_x"])
            message.position_y = float(record["position_y"])
            message.confidence = float(record["confidence"])
            message.timestamp = float(record["timestamp"])
            message.state = str(record["state"])
            message.sightings = int(record["sightings"])
            self._track_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SurvivorNode()
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
