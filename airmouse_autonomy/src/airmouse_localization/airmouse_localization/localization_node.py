"""rclpy adapter: Gazebo ground-truth pose -> TF map->base_link (DEV BASELINE).

DEVELOPMENT BASELINE ONLY. This node wires the simulator's ground-truth pose
(from the ``ros_gz`` bridge) straight into the TF tree so mapping, exploration
and navigation can be exercised end-to-end before a real estimator exists. It
is NOT localization: it is a labeled crutch.

Goal-4 runtime follow-up (must replace this node): RGB-D / IMU VIO or SLAM
(e.g. ``rtabmap_ros``, ``slam_toolbox`` + VIO odometry) publishing
map -> odom -> base_link, with this node repurposed as an *evaluator* that
compares the estimate against ground truth and logs drift. Ground truth must
never enter the autonomy pipeline.

Behavior
--------
* Subscribes: ``/ground_truth/pose`` (geometry_msgs/PoseStamped) and ``/odom``
  (nav_msgs/Odometry, optional) for drift logging.
* Publishes: TF ``map -> base_link`` while ``use_truth`` is true.
* Periodically logs (throttled) the drift between the truth pose and the last
  odometry estimate so silent estimator regressions are visible in the log.

Parameters
----------
use_truth : bool (default True)
    Publish the truth TF at all. Set false once a real estimator owns TF.
truth_pose_topic : str (default "/ground_truth/pose")
odom_topic : str (default "/odom")
base_frame : str (default "base_link")
map_frame : str (default "map")
drift_log_period_s : float (default 5.0)
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .truth_baseline import format_truth_pose, validate_truth


def _yaw_from_quaternion(q) -> float:
    """Z-axis (yaw) of a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LocalizationNode(Node):
    """Truth-pose TF broadcaster with odometry drift logging (DEV baseline)."""

    def __init__(self) -> None:
        super().__init__("localization_node")
        self.declare_parameter("use_truth", True)
        self.declare_parameter("truth_pose_topic", "/ground_truth/pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("drift_log_period_s", 5.0)

        self._use_truth = bool(self.get_parameter("use_truth").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        period = float(self.get_parameter("drift_log_period_s").value)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError(f"drift_log_period_s must be > 0, got {period!r}")

        truth_topic = str(self.get_parameter("truth_pose_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)

        self._truth = None  # last valid truth pose dict
        self._odom_xy = None  # last odometry (x, y)
        self._last_drift_log = float("-inf")
        self._drift_log_period = period

        self._tf_broadcaster = TransformBroadcaster(self) if self._use_truth else None
        self.create_subscription(PoseStamped, truth_topic, self._on_truth, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        if not self._use_truth:
            self.get_logger().warn(
                "localization_node started with use_truth=False: no truth TF is "
                "published; a real VIO/SLAM stack must own map->base_link."
            )
        else:
            self.get_logger().warn(
                "localization_node is publishing a DEV-BASELINE TF map->base_link "
                "from Gazebo ground truth. Replace with VIO/SLAM (Goal 4) before "
                "any non-simulation use."
            )

    # ------------------------------------------------------------- callbacks
    def _on_truth(self, msg: PoseStamped) -> None:
        pose = format_truth_pose(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            _yaw_from_quaternion(msg.pose.orientation),
        )
        if not validate_truth(pose):
            self.get_logger().warning("dropping non-finite ground-truth pose sample")
            return
        self._truth = pose
        if self._tf_broadcaster is not None:
            self._broadcast(pose)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_xy = (float(msg.pose.pose.position.x),
                         float(msg.pose.pose.position.y))
        self._log_drift()

    # --------------------------------------------------------------- helpers
    def _broadcast(self, pose: dict) -> None:
        transform = TransformStamped()
        now = self.get_clock().now().to_msg()
        transform.header.stamp = now
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = pose["x"]
        transform.transform.translation.y = pose["y"]
        transform.transform.translation.z = pose["z"]
        yaw = pose["yaw"]
        transform.transform.rotation.z = math.sin(yaw * 0.5)
        transform.transform.rotation.w = math.cos(yaw * 0.5)
        self._tf_broadcaster.sendTransform(transform)

    def _log_drift(self) -> None:
        """Throttled drift log: truth vs odometry position error."""
        if self._truth is None or self._odom_xy is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_drift_log < self._drift_log_period:
            return
        self._last_drift_log = now
        drift = math.hypot(self._truth["x"] - self._odom_xy[0],
                           self._truth["y"] - self._odom_xy[1])
        self.get_logger().info(
            f"truth-vs-odom drift: {drift:.3f} m "
            f"(DEV BASELINE; ground truth is never used by autonomy)"
        )


def main(args=None) -> None:
    """Entry point (``ros2 run airmouse_localization localization_node``)."""
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
