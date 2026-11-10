"""rclpy adapter for the navigation cores.

Thin conversion layer only: a ``/goal`` triggers one A* plan over the received
``/map`` (wrapped with ``mapper_from_grid``); a 20 Hz loop feeds the pose and
the mutable waypoint list to ``LocalPlanner.follow`` and publishes the body
velocity as ``TwistStamped``. All planning/following/stuck logic lives in the
navigation cores.

Topics
    in : ``/map``     nav_msgs/OccupancyGrid (transient_local)
         ``/goal``    geometry_msgs/PoseStamped, frame ``map``
         TF ``map`` -> ``base_link``
    out: ``/path``      nav_msgs/Path, frame ``map`` (empty = no active path)
         ``/cmd_vel``   geometry_msgs/TwistStamped, frame ``base_link``
         ``/nav_status`` std_msgs/String (``stuck``)

A goal that arrives before the first map (or before TF is up) is kept pending
and re-planned as soon as a map arrives; unreachable goals are logged and
simply not published so exploration re-plans.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_ros
from airmouse_mapping.grid_manager import mapper_from_grid
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String

from airmouse_navigation.astar_planner import plan
from airmouse_navigation.local_planner import LocalPlanner

NODE_NAME = "navigation_node"
MAP_TOPIC = "/map"
GOAL_TOPIC = "/goal"
PATH_TOPIC = "/path"
CMD_VEL_TOPIC = "/cmd_vel"
NAV_STATUS_TOPIC = "/nav_status"

MAP_FRAME = "map"
BASE_FRAME = "base_link"

FOLLOW_PERIOD_S = 0.05  # 20 Hz
STATUS_STUCK = "stuck"

_MAX_DT_S = 0.5
_MAX_ANGULAR_Z_RAD_S = 1.0
_MIN_FORWARD_SPEED_EPS = 1e-6
_TF_LOG_THROTTLE_S = 5.0


def _latched_qos() -> QoSProfile:
    """Reliable transient-local QoS for the map topic."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class NavigationNode(Node):
    """Turns exploration goals into paths and body-frame velocity commands."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.cell_size = float(self.declare_parameter("cell_size", 0.10).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 6.0).value)
        self.clearance_m = float(self.declare_parameter("clearance_m", 0.10).value)
        speed_mps = float(self.declare_parameter("speed_mps", 2.0).value)
        self._local = LocalPlanner({
            "speed_mps": speed_mps,
            "waypoint_pop_tolerance_m": float(
                self.declare_parameter("waypoint_pop_tolerance_m", 0.15).value),
            "stuck_watchdog_s": float(
                self.declare_parameter("stuck_watchdog_s", 3.0).value),
        })

        self._mapper = None
        self._path: list[tuple[float, float]] = []
        self._pending_goal: tuple[float, float] | None = None
        self._last_tick_s: float | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, _latched_qos())
        self.create_subscription(PoseStamped, GOAL_TOPIC, self._on_goal, 10)
        self._path_pub = self.create_publisher(Path, PATH_TOPIC, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        self._status_pub = self.create_publisher(String, NAV_STATUS_TOPIC, 10)

        self.create_timer(FOLLOW_PERIOD_S, self._follow_tick)
        self.get_logger().info(
            f"{NODE_NAME} ready: speed={speed_mps} m/s, "
            f"pop_tol={self._local.waypoint_pop_tolerance_m} m, "
            f"watchdog={self._local.stuck_watchdog_s} s")

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
            self._mapper = mapper_from_grid(
                grid, self.cell_size, self.max_range_m, self.clearance_m)
        except ValueError as exc:
            self.get_logger().error(f"cannot build mapper view from /map: {exc}")
            return
        if self._pending_goal is not None:
            self._try_plan()

    def _on_goal(self, msg: PoseStamped) -> None:
        self._pending_goal = (
            float(msg.pose.position.x), float(msg.pose.position.y))
        self._clear_path()  # the new goal supersedes whatever was active
        self._try_plan()

    # ------------------------------------------------------------ planning

    def _try_plan(self) -> None:
        if self._pending_goal is None:
            return
        if self._mapper is None:
            self.get_logger().warn(
                "goal received before first /map; keeping it pending",
                throttle_duration_sec=_TF_LOG_THROTTLE_S)
            return
        pose = self._lookup_pose()
        if pose is None:
            self.get_logger().warn(
                "goal received before TF is available; keeping it pending",
                throttle_duration_sec=_TF_LOG_THROTTLE_S)
            return

        goal = self._pending_goal
        path = plan(self._mapper, (pose[0], pose[1]), goal)
        if path is None:
            self.get_logger().warn(
                f"no path to goal ({goal[0]:.2f}, {goal[1]:.2f}); "
                "waiting for exploration to re-plan")
            self._pending_goal = None
            return

        self._pending_goal = None
        self._path = [(float(x), float(y)) for x, y in path]
        self._local.reset()
        self._publish_path(self._path)
        self.get_logger().info(
            f"planned {len(self._path)} waypoints to "
            f"({goal[0]:.2f}, {goal[1]:.2f})")

    # -------------------------------------------------------------- follow

    def _follow_tick(self) -> None:
        now = self._now_s()
        dt = self._dt(now)
        self._last_tick_s = now
        if not self._path:
            return
        pose = self._lookup_pose()
        if pose is None:
            return

        vx, vy, reached, stuck = self._local.follow(pose, self._path, dt)
        if reached:
            self.get_logger().info("path complete")
            self._publish_twist(0.0, 0.0)
            self._clear_path()
            return
        if stuck:
            self.get_logger().warn("local planner stuck: aborting path")
            self._publish_twist(0.0, 0.0)
            self._status_pub.publish(String(data=STATUS_STUCK))
            self._clear_path()
            return
        self._publish_twist(*self._body_velocity(pose, vx, vy))

    def _body_velocity(self, pose, vx: float, vy: float) -> tuple[float, float]:
        """Map-frame velocity -> (linear.x, clamped angular.z) in base_link."""
        theta = pose[2]
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        forward = vx * cos_t + vy * sin_t
        lateral = -vx * sin_t + vy * cos_t
        angular = math.atan2(lateral, max(forward, _MIN_FORWARD_SPEED_EPS))
        angular = max(-_MAX_ANGULAR_Z_RAD_S, min(_MAX_ANGULAR_Z_RAD_S, angular))
        return float(forward), float(angular)

    # ------------------------------------------------------------- outputs

    def _publish_path(self, waypoints: list[tuple[float, float]]) -> None:
        msg = Path()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = MAP_FRAME
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._path_pub.publish(msg)

    def _clear_path(self) -> None:
        self._path = []
        self._publish_path([])

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.frame_id = BASE_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    # ------------------------------------------------------------- helpers

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

    def _dt(self, now: float) -> float:
        if self._last_tick_s is None:
            return FOLLOW_PERIOD_S
        return max(1e-3, min(_MAX_DT_S, now - self._last_tick_s))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
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
