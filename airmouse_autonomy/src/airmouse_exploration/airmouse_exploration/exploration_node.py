"""rclpy adapter for the frontier exploration core.

Thin conversion layer only: every ``/map`` message is wrapped with
``airmouse_mapping.grid_manager.mapper_from_grid`` and every ``/survivors``
message refreshes a local track-dict cache. All goal selection, ranking,
blacklisting and replan cadence live in
``airmouse_exploration.frontier_explorer.FrontierExplorer``.

Topics
    in : ``/map``           nav_msgs/OccupancyGrid (transient_local)
         ``/survivors``     airmouse_interfaces/SurvivorTrack
         TF ``map`` -> ``base_link``
    out: ``/goal``          geometry_msgs/PoseStamped, frame ``map``
         ``/mission_complete`` std_msgs/Bool (transient_local, latched)

Goal publication happens when a new goal id becomes active, and is refreshed
every ``_GOAL_REPUBLISH_S`` seconds while that goal stays active so a late or
restarted navigation node can still pick it up without re-planning floods.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_ros
from airmouse_interfaces.msg import SurvivorTrack
from airmouse_mapping.grid_manager import mapper_from_grid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool

from airmouse_exploration.frontier_explorer import FrontierExplorer

NODE_NAME = "exploration_node"
MAP_TOPIC = "/map"
TRACKS_TOPIC = "/survivors"
GOAL_TOPIC = "/goal"
MISSION_COMPLETE_TOPIC = "/mission_complete"

MAP_FRAME = "map"
BASE_FRAME = "base_link"

TICK_PERIOD_S = 0.5  # 2 Hz decision cadence
_GOAL_REPUBLISH_S = 10.0
_GOAL_STATUS_ACTIVE = "active"
_TF_LOG_THROTTLE_S = 5.0


def _latched_qos() -> QoSProfile:
    """Reliable transient-local QoS for one-shot/state topics."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class ExplorationNode(Node):
    """Subscribes map + tracks, publishes exploration goals."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.cell_size = float(self.declare_parameter("cell_size", 0.10).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 6.0).value)
        self.clearance_m = float(self.declare_parameter("clearance_m", 0.10).value)
        self.entry = (
            float(self.declare_parameter("entry_x", 0.0).value),
            float(self.declare_parameter("entry_y", 0.0).value),
        )
        self._explorer_params = {
            "replan_interval_s": float(
                self.declare_parameter("replan_interval_s", 2.0).value),
            "goal_reach_radius_m": float(
                self.declare_parameter("goal_reach_radius_m", 0.4).value),
            "investigate_after_s": float(
                self.declare_parameter("investigate_after_s", 4.0).value),
            "max_investigate_attempts": int(
                self.declare_parameter("max_investigate_attempts", 2).value),
            "lidar_range_m": float(
                self.declare_parameter("lidar_range_m", 6.0).value),
        }
        self.reach_radius_m = self._explorer_params["goal_reach_radius_m"]

        self._tracks: dict[int, dict] = {}
        self._explorer: FrontierExplorer | None = None
        self._last_goal_id: int | None = None
        self._last_goal_publish_s: float | None = None
        self._mission_complete_sent = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, _latched_qos())
        self.create_subscription(SurvivorTrack, TRACKS_TOPIC, self._on_survivor, 10)
        self._goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
        self._complete_pub = self.create_publisher(
            Bool, MISSION_COMPLETE_TOPIC, _latched_qos())

        self.create_timer(TICK_PERIOD_S, self._tick)
        self.get_logger().info(
            f"{NODE_NAME} ready: entry={self.entry}, "
            f"replan={self._explorer_params['replan_interval_s']}s")

    # ------------------------------------------------------------- inputs

    def _on_map(self, msg: OccupancyGrid) -> None:
        width, height = int(msg.info.width), int(msg.info.height)
        if width <= 0 or height <= 0:
            self.get_logger().warn("ignoring /map with empty dimensions")
            return
        data = np.asarray(msg.data, dtype=np.int8)
        if data.size != width * height:
            self.get_logger().error(
                f"ignoring /map: {data.size} cells != {width}x{height}")
            return
        grid = data.reshape((height, width))
        try:
            mapper = mapper_from_grid(
                grid, self.cell_size, self.max_range_m, self.clearance_m)
        except ValueError as exc:
            self.get_logger().error(f"cannot build mapper view from /map: {exc}")
            return

        if self._explorer is None:
            self._explorer = FrontierExplorer(
                mapper, self._track_list, self.entry, self._explorer_params)
            self.get_logger().info("exploration planner initialised from first /map")
        else:
            self._explorer.mapper = mapper

    def _on_survivor(self, msg: SurvivorTrack) -> None:
        survivor_id = int(msg.survivor_id)
        previous = self._tracks.get(survivor_id)
        self._tracks[survivor_id] = {
            "survivor_id": survivor_id,
            "position_x": float(msg.position_x),
            "position_y": float(msg.position_y),
            "state": str(msg.state),
            "confidence": float(msg.confidence),
            "first_seen": (float(previous["first_seen"]) if previous is not None
                           else self._now_s()),
        }

    def _track_list(self) -> list[dict]:
        """tracks_fn for the explorer (fresh copies, never aliased)."""
        return [dict(track) for track in self._tracks.values()]

    # -------------------------------------------------------------- ticks

    def _tick(self) -> None:
        now = self._now_s()
        explorer = self._explorer
        if explorer is None:
            return
        if explorer.mission_complete:
            self._publish_mission_complete()
            return

        pose = self._lookup_pose()
        if pose is None:
            return
        explorer.set_pose(pose[0], pose[1])
        self._notify_reached_goal(explorer, pose)

        goal = explorer.decide(now)
        if goal is not None and goal["status"] == _GOAL_STATUS_ACTIVE:
            self._publish_goal(goal, now)
        if explorer.mission_complete:
            self._publish_mission_complete()

    def _notify_reached_goal(self, explorer: FrontierExplorer, pose) -> None:
        goal = explorer.current_goal
        if goal is None or goal["status"] != _GOAL_STATUS_ACTIVE:
            return
        target_x, target_y = goal["target"]
        if math.hypot(target_x - pose[0], target_y - pose[1]) > self.reach_radius_m:
            return
        self.get_logger().info(
            f"goal {goal['goal_id']} ({goal['kind']}) reached")
        explorer.notify_goal_reached(goal["goal_id"])
        self._last_goal_id = None

    # ------------------------------------------------------------ outputs

    def _publish_goal(self, goal: dict, now: float) -> None:
        same_goal = self._last_goal_id == goal["goal_id"]
        fresh = (self._last_goal_publish_s is None
                 or now - self._last_goal_publish_s >= _GOAL_REPUBLISH_S)
        if same_goal and not fresh:
            return

        target_x, target_y = goal["target"]
        msg = PoseStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(target_x)
        msg.pose.position.y = float(target_y)
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)

        self._last_goal_id = goal["goal_id"]
        self._last_goal_publish_s = now
        self.get_logger().info(
            f"published {goal['kind']} goal {goal['goal_id']} "
            f"at ({target_x:.2f}, {target_y:.2f})")

    def _publish_mission_complete(self) -> None:
        if self._mission_complete_sent:
            return
        self._complete_pub.publish(Bool(data=True))
        self._mission_complete_sent = True
        self.get_logger().info("exploration complete: published /mission_complete")

    # ------------------------------------------------------------ helpers

    def _lookup_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME, Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"TF {MAP_FRAME}->{BASE_FRAME} unavailable: {exc}",
                throttle_duration_sec=_TF_LOG_THROTTLE_S)
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        sin_yaw = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cos_yaw = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return (
            float(translation.x),
            float(translation.y),
            math.atan2(sin_yaw, cos_yaw),
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
