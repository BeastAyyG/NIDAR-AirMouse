"""Persist and de-duplicate simulated survivor detections in map/grid frames."""

import json
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String

from .grid import GridConfig, cell_key, world_to_grid


class SurvivorLocalizer(Node):
    """Convert map-frame detections to the book's machine-readable grid tags."""

    def __init__(self) -> None:
        super().__init__("nidar_airmouse_survivor_localizer")
        self.declare_parameter("map_origin_x", 0.0)
        self.declare_parameter("map_origin_y", 0.0)
        self.declare_parameter("grid_cell_size", 1.0)
        self.declare_parameter("min_observations", 2)
        self.config = GridConfig(
            origin_x=float(self.get_parameter("map_origin_x").value),
            origin_y=float(self.get_parameter("map_origin_y").value),
            cell_size=float(self.get_parameter("grid_cell_size").value),
        )
        self.min_observations = int(self.get_parameter("min_observations").value)
        self.observations: dict[str, dict] = {}
        self.tag_pub = self.create_publisher(String, "/airmouse/survivors/grid_tags", 10)
        self.create_subscription(PointStamped, "/airmouse/detections/map_point", self.detection_callback, 10)

    def detection_callback(self, message: PointStamped) -> None:
        if message.header.frame_id and message.header.frame_id != "map":
            self.get_logger().warning(f"Ignoring detection in frame {message.header.frame_id}; expected map")
            return
        grid_x, grid_y = world_to_grid(message.point.x, message.point.y, self.config)
        key = cell_key(grid_x, grid_y)
        now = datetime.now(timezone.utc).isoformat()
        record = self.observations.setdefault(
            key,
            {
                "id": f"survivor_{len(self.observations) + 1:03d}",
                "grid_x": grid_x,
                "grid_y": grid_y,
                "confidence": 0.0,
                "observations": 0,
                "first_seen": now,
            },
        )
        record["observations"] += 1
        record["confidence"] = min(1.0, 0.5 + 0.25 * record["observations"])
        record["last_seen"] = now
        record["position_map"] = {"x": message.point.x, "y": message.point.y, "z": message.point.z}
        record["validated"] = record["observations"] >= self.min_observations
        if record["validated"]:
            self.publish_tags()

    def publish_tags(self) -> None:
        message = String()
        message.data = json.dumps(
            {"frame": "map", "grid_cell_size": self.config.cell_size, "survivors": list(self.observations.values())},
            sort_keys=True,
        )
        self.tag_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SurvivorLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
