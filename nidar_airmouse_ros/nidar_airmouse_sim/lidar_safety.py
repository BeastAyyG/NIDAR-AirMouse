"""Validate the LiDAR stream and publish a machine-readable safety state."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .safety import LidarAssessment, assess_scan


class LidarSafety(Node):
    """Turn LiDAR health and clearance into a conservative safety contract."""

    def __init__(self) -> None:
        super().__init__("nidar_airmouse_lidar_safety")
        self.declare_parameter("vehicle_envelope_radius", 0.32)
        self.declare_parameter("lidar_uncertainty_margin", 0.15)
        self.declare_parameter("lidar_minimum_clearance", 0.08)
        self.declare_parameter("lidar_stop_distance", 0.55)
        self.declare_parameter("lidar_emergency_distance", 0.40)
        self.declare_parameter("lidar_timeout_seconds", 0.5)
        self.declare_parameter("lidar_min_valid_beams", 20)

        envelope_radius = float(self.get_parameter("vehicle_envelope_radius").value)
        uncertainty_margin = float(
            self.get_parameter("lidar_uncertainty_margin").value
        )
        minimum_clearance = float(
            self.get_parameter("lidar_minimum_clearance").value
        )
        derived_stop_distance = envelope_radius + uncertainty_margin + minimum_clearance
        configured_stop_distance = float(
            self.get_parameter("lidar_stop_distance").value
        )
        self.stop_distance = max(configured_stop_distance, derived_stop_distance)
        self.emergency_distance = min(
            float(self.get_parameter("lidar_emergency_distance").value),
            self.stop_distance,
        )
        self.timeout_seconds = float(self.get_parameter("lidar_timeout_seconds").value)
        self.minimum_valid_beams = max(1, int(self.get_parameter("lidar_min_valid_beams").value))
        self.last_scan_at = None
        self.assessment = LidarAssessment(
            ready=False,
            safe=False,
            emergency=False,
            valid_beams=0,
            minimum_range=None,
            reason="waiting_for_scan",
        )

        self.status_pub = self.create_publisher(String, "/airmouse/safety/status", 10)
        self.ready_pub = self.create_publisher(Bool, "/airmouse/safety/lidar_ready", 10)
        self.safe_pub = self.create_publisher(Bool, "/airmouse/safety/lidar_safe", 10)
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.create_timer(0.1, self.publish_status)

    def scan_callback(self, message: LaserScan) -> None:
        self.last_scan_at = self.get_clock().now()
        self.assessment = assess_scan(
            message.ranges,
            message.range_min,
            message.range_max,
            self.stop_distance,
            self.emergency_distance,
            self.minimum_valid_beams,
        )

    def publish_status(self) -> None:
        assessment = self.assessment
        if self.last_scan_at is None:
            assessment = LidarAssessment(
                ready=False,
                safe=False,
                emergency=False,
                valid_beams=0,
                minimum_range=None,
                reason="waiting_for_scan",
            )
        else:
            age = (
                self.get_clock().now() - self.last_scan_at
            ).nanoseconds / 1e9
            if age > self.timeout_seconds:
                assessment = LidarAssessment(
                    ready=False,
                    safe=False,
                    emergency=False,
                    valid_beams=self.assessment.valid_beams,
                    minimum_range=self.assessment.minimum_range,
                    reason="stale_scan",
                )

        ready = Bool()
        ready.data = assessment.ready
        self.ready_pub.publish(ready)
        safe = Bool()
        safe.data = assessment.safe
        self.safe_pub.publish(safe)

        status = String()
        status.data = json.dumps(
            {
                "lidar_ready": assessment.ready,
                "lidar_safe": assessment.safe,
                "emergency": assessment.emergency,
                "valid_beams": assessment.valid_beams,
                "minimum_range": assessment.minimum_range,
                "stop_distance": self.stop_distance,
                "emergency_distance": self.emergency_distance,
                "reason": assessment.reason,
            },
            sort_keys=True,
        )
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
