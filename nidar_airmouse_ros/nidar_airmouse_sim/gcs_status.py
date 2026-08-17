"""Publish a compact GCS-facing status record for SITL monitoring."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

try:
    from px4_msgs.msg import VehicleStatus
except ImportError:  # Keep the node usable before px4_msgs is sourced.
    VehicleStatus = None


class GcsStatus(Node):
    """Combine the mission state with PX4 connection state for a simple dashboard."""

    def __init__(self) -> None:
        super().__init__("nidar_airmouse_gcs_status")
        self.mission_state = "BOOT"
        self.px4_connected = False
        self.status_pub = self.create_publisher(String, "/airmouse/gcs/status", 10)
        self.create_subscription(String, "/airmouse/mission_state", self.mission_callback, 10)
        if VehicleStatus is not None:
            self.create_subscription(
                VehicleStatus,
                "/fmu/out/vehicle_status_v1",
                self.px4_callback,
                qos_profile_sensor_data,
            )
        self.create_timer(1.0, self.publish_status)

    def mission_callback(self, message: String) -> None:
        self.mission_state = message.data

    def px4_callback(self, message) -> None:
        self.px4_connected = not bool(getattr(message, "gcs_connection_lost", True))

    def publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "mission_state": self.mission_state,
                "px4_topics_seen": self.px4_connected,
                "qgroundcontrol_link": self.px4_connected,
                "control_stack": "PX4",
                "simulation": True,
            },
            sort_keys=True,
        )
        self.status_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GcsStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
