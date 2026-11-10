"""PX4 offboard velocity bridge (Goal 7).

Turns autonomy commands into PX4 OFFBOARD actions:

  /cmd_vel (TwistStamped, body frame) + /mission_command (HOLD|LAND)
      -> OffboardSequencer (pure core, tested offline)
      -> /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint,
         /fmu/in/vehicle_command

Heartbeat: setpoints stream at ``rate_hz`` (10 Hz) whenever the sequencer
says so; a command timeout holds, and a sustained loss lands. Arming and
OFFBOARD are never requested until the autonomy stack has commanded motion.

This node complements, and does not replace, the existing waypoint-based
``nidar_airmouse_sim`` mission manager: use one or the other for PX4 control.
Requires ``px4_msgs`` (already part of the existing PX4 setup).

UNVERIFIED-OFFLINE: rclpy/px4_msgs runtime is not available on the authoring
machine; the sequencing logic itself is covered by unit tests.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from airmouse_mission.offboard_core import (
    OffboardParams,
    OffboardSequencer,
    Px4Status,
)

try:  # px4_msgs lives in the PX4 workspace; absent on a plain ROS install.
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleStatus,
    )
except ImportError:  # pragma: no cover - runtime-only path
    OffboardControlMode = TrajectorySetpoint = VehicleCommand = VehicleStatus = None

PX4_OFFBOARD_MAIN_MODE = 6
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_NAV_LAND = 21


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """ENU yaw from a quaternion (REP-103), 0 = +x (east)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class OffboardBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("offboard_bridge")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("prestream_s", 1.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("hold_timeout_s", 10.0)
        self.declare_parameter("max_speed_mps", 1.5)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        if VehicleStatus is None:
            self.get_logger().error(
                "px4_msgs not importable; source the PX4 workspace before "
                "running offboard_bridge")
            raise RuntimeError("px4_msgs missing")

        self.sequencer = OffboardSequencer(OffboardParams(
            rate_hz=float(self.get_parameter("rate_hz").value),
            prestream_s=float(self.get_parameter("prestream_s").value),
            cmd_timeout_s=float(self.get_parameter("cmd_timeout_s").value),
            hold_timeout_s=float(self.get_parameter("hold_timeout_s").value),
            max_speed_mps=float(self.get_parameter("max_speed_mps").value),
        ))
        self._vx = self._vy = 0.0
        self._yaw = 0.0
        self._hold = False
        self._land = False
        self._status = Px4Status()
        self._last_status_t = float("-inf")

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode",
            qos_profile_sensor_data)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint",
            qos_profile_sensor_data)
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, str(self.get_parameter("cmd_topic").value),
            self._on_cmd_vel, 10)
        self.create_subscription(
            String, "/mission_command", self._on_mission_command, 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, 10)
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._on_status,
            qos_profile_sensor_data)
        rate = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self._tick)

    # -- inputs -------------------------------------------------------------
    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        self._vx = float(msg.twist.linear.x)
        self._vy = float(msg.twist.linear.y)
        self.sequencer.note_command(self._now())

    def _on_mission_command(self, msg: String) -> None:
        command = msg.data.strip().upper()
        self._hold = command == "HOLD"
        self._land = command == "LAND"
        if command not in ("HOLD", "LAND", ""):
            self.get_logger().warn(f"unknown mission_command '{msg.data}'")

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_status(self, msg: VehicleStatus) -> None:
        self._last_status_t = self._now()
        self._status = Px4Status(
            connected=True,
            armed=msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            offboard=msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # -- output -------------------------------------------------------------
    def _tick(self) -> None:
        now = self._now()
        status = self._status
        if now - self._last_status_t > 2.0:
            status = Px4Status(connected=False)
        action = self.sequencer.step(
            now, status, self._vx, self._vy, 0.0, self._yaw,
            self._hold, self._land)
        if not action.stream:
            return
        mode = OffboardControlMode()
        mode.position = False
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = int(now * 1e6)
        self.offboard_pub.publish(mode)

        vn, ve, vd, yaw_rate = action.setpoint
        setpoint = TrajectorySetpoint()
        setpoint.position = [float("nan")] * 3
        setpoint.velocity = [float(vn), float(ve), float(vd)]
        setpoint.yaw = float("nan")
        setpoint.yawspeed = float(yaw_rate)
        setpoint.timestamp = int(now * 1e6)
        self.setpoint_pub.publish(setpoint)

        if action.request_offboard:
            self._command(VEHICLE_CMD_DO_SET_MODE,
                          float(PX4_OFFBOARD_MAIN_MODE), 1.0)
            self.get_logger().info("requested PX4 OFFBOARD mode")
        if action.request_arm:
            self._command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
            self.get_logger().info("requested arming")
        if action.request_land:
            self._command(VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            self.get_logger().warn("requested LAND (failsafe)")

    def _command(self, command_id: int, param1: float, param2: float) -> None:
        msg = VehicleCommand()
        msg.command = command_id
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self._now() * 1e6)
        self.command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()