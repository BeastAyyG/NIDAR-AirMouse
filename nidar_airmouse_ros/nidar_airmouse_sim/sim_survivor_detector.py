"""Synthetic detector for static Gazebo survivor markers.

This is deliberately simulation-only: it provides the RGB/depth detector
interface with known marker positions so the mission and grid pipeline can be
tested before the real OAK-D detector is connected.
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class SimSurvivorDetector(Node):
    """Emit repeated map detections when the simulated vehicle is nearby."""

    def __init__(self) -> None:
        super().__init__("nidar_airmouse_sim_survivor_detector")
        self.declare_parameter("detection_radius", 2.5)
        self.declare_parameter(
            "survivor_targets",
            [-12.0, -12.0, 0.5, -6.0, -7.0, 0.5, -12.0, 0.0, 0.5,
             -12.0, 12.0, 0.5, 12.0, -12.0, 0.5, 12.0, 12.0, 0.5],
        )
        flat_targets = [float(value) for value in self.get_parameter("survivor_targets").value]
        if len(flat_targets) % 3 != 0:
            raise ValueError("survivor_targets must contain x/y/z triples")
        self.targets = [
            tuple(flat_targets[index:index + 3])
            for index in range(0, len(flat_targets), 3)
        ]
        self.position = None
        self.publisher = self.create_publisher(PointStamped, "/airmouse/detections/map_point", 10)
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.5, self.detect)

    def position_callback(self, message: VehicleLocalPosition) -> None:
        self.position = message

    def detect(self) -> None:
        if self.position is None or not self.position.xy_valid:
            return
        radius = float(self.get_parameter("detection_radius").value)
        for target_x, target_y, target_z in self.targets:
            if math.hypot(float(self.position.x) - target_x, float(self.position.y) - target_y) <= radius:
                detection = PointStamped()
                detection.header.stamp = self.get_clock().now().to_msg()
                detection.header.frame_id = "map"
                detection.point.x = target_x
                detection.point.y = target_y
                detection.point.z = target_z
                self.publisher.publish(detection)
                self.get_logger().info(f"Simulated survivor detection at ({target_x:.1f}, {target_y:.1f})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimSurvivorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
