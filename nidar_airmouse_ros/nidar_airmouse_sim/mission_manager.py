"""PX4 offboard mission manager for the AirMouse maze simulation."""

import json
import math
from enum import Enum

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MissionState(str, Enum):
    BOOT = "BOOT"
    PRECHECK = "PRECHECK"
    ARM_TAKEOFF = "ARM/TAKEOFF"
    HOME_ANCHOR = "HOME_ANCHOR"
    EXPLORE = "EXPLORE"
    ROOM_SCAN = "ROOM_SCAN/FRONTIER_SELECT"
    SURVIVOR_VERIFY = "SURVIVOR_VERIFY"
    MARK_GRID = "MARK_GRID"
    RETURN_HOME = "RETURN_HOME"
    LAND_COMPLETE = "LAND/COMPLETE"
    ABORT = "ABORT"


class MissionManager(Node):
    """Fly the configured safe waypoint route through the simulated maze."""

    def __init__(self) -> None:
        super().__init__("nidar_airmouse_mission")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("takeoff_altitude", 2.5)
        self.declare_parameter("waypoint_acceptance", 0.8)
        self.declare_parameter("precheck_seconds", 3.0)
        self.declare_parameter("hold_seconds", 2.0)
        self.declare_parameter("land_timeout_seconds", 15.0)
        self.declare_parameter("require_lidar", True)
        self.declare_parameter("safety_abort_timeout_seconds", 3.0)
        self.declare_parameter("geofence_x_min", -14.0)
        self.declare_parameter("geofence_x_max", 14.0)
        self.declare_parameter("geofence_y_min", -14.0)
        self.declare_parameter("geofence_y_max", 14.0)
        self.declare_parameter("geofence_z_min", -4.0)
        self.declare_parameter("geofence_z_max", 0.5)
        self.declare_parameter(
            "mission_waypoints",
            [-6.0, 0.0, -2.5, -6.0, -7.0, -2.5, -6.0, 0.0, -2.5,
             -12.0, 0.0, -2.5, 0.0, 0.0, -2.5],
        )

        self.home = (0.0, 0.0, -float(self.get_parameter("takeoff_altitude").value))
        flat_waypoints = [float(v) for v in self.get_parameter("mission_waypoints").value]
        if len(flat_waypoints) < 3 or len(flat_waypoints) % 3 != 0:
            raise ValueError("mission_waypoints must contain x/y/z triples")
        self.waypoints = [
            tuple(flat_waypoints[index:index + 3])
            for index in range(0, len(flat_waypoints), 3)
        ]

        self.state = MissionState.BOOT
        self.started = False
        self.state_entered_at = self.get_clock().now()
        self.position = None
        self.vehicle_status = None
        self.path_index = 0
        self.setpoint_count = 0
        self.survivor_count = 0
        self.land_requested = False
        self.disarm_requested = False
        self.abort_requested = False
        self.last_command_at = self.get_clock().now()
        self.lidar_ready = False
        self.lidar_safe = False
        self.safety_emergency = False
        self.safety_reason = "waiting_for_lidar"
        self.safety_blocked_at = None

        self.state_pub = self.create_publisher(String, "/airmouse/mission_state", 10)
        self.status_pub = self.create_publisher(String, "/airmouse/mission_status", 10)
        self.plan_pub = self.create_publisher(String, "/airmouse/mission_plan", 10)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile_sensor_data
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile_sensor_data
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile_sensor_data
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self.status_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, "/airmouse/survivors/grid_tags", self.survivor_callback, 10)
        self.create_subscription(String, "/airmouse/safety/status", self.safety_callback, 10)

        self.create_service(Trigger, "/airmouse/start", self.start_callback)
        self.create_service(Trigger, "/airmouse/abort", self.abort_callback)
        self.create_service(Trigger, "/airmouse/complete", self.complete_callback)
        self.create_timer(0.05, self.control_tick)
        self.create_timer(0.5, self.status_tick)
        self.publish_plan()
        self.publish_state()

        if bool(self.get_parameter("auto_start").value):
            self.started = True
            self.transition(MissionState.PRECHECK)

    def start_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.state in (MissionState.ABORT, MissionState.LAND_COMPLETE):
            response.success = False
            response.message = f"Mission is already {self.state.value}; restart the node for a new run"
            return response
        self.started = True
        self.abort_requested = False
        self.land_requested = False
        self.disarm_requested = False
        self.path_index = 0
        self.transition(MissionState.PRECHECK)
        response.success = True
        response.message = "PX4 offboard maze mission started"
        return response

    def abort_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.abort_requested = True
        self.transition(MissionState.ABORT)
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        response.success = True
        response.message = "Mission aborted; PX4 return-to-launch requested"
        return response

    def complete_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.state == MissionState.LAND_COMPLETE:
            response.success = True
            response.message = "Mission already complete"
        else:
            self.transition(MissionState.LAND_COMPLETE)
            response.success = True
            response.message = "Mission marked complete"
        return response

    def position_callback(self, message: VehicleLocalPosition) -> None:
        self.position = message

    def status_callback(self, message: VehicleStatus) -> None:
        self.vehicle_status = message

    def survivor_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.survivor_count = sum(1 for item in payload.get("survivors", []) if item.get("validated"))
        except (TypeError, ValueError, AttributeError):
            self.get_logger().warning("Ignoring malformed survivor tag payload")

    def safety_callback(self, message: String) -> None:
        """Update the mission gate from the LiDAR safety supervisor."""
        try:
            payload = json.loads(message.data)
            self.lidar_ready = bool(payload.get("lidar_ready", False))
            self.lidar_safe = bool(payload.get("lidar_safe", False))
            self.safety_emergency = bool(payload.get("emergency", False))
            self.safety_reason = str(payload.get("reason", "unknown"))
        except (TypeError, ValueError, AttributeError):
            self.lidar_ready = False
            self.lidar_safe = False
            self.safety_emergency = False
            self.safety_reason = "malformed_safety_status"

    def apply_safety_guard(self) -> None:
        """Hold on a bad LiDAR state, then request PX4 RTL if it persists."""
        if not bool(self.get_parameter("require_lidar").value):
            self.safety_blocked_at = None
            return
        if self.state in (
            MissionState.BOOT,
            MissionState.LAND_COMPLETE,
            MissionState.ABORT,
            MissionState.RETURN_HOME,
        ):
            self.safety_blocked_at = None
            return

        blocked = not self.lidar_ready or not self.lidar_safe
        if not blocked:
            self.safety_blocked_at = None
            return
        if self.safety_blocked_at is None:
            self.safety_blocked_at = self.get_clock().now()
        if not self.is_armed():
            return
        blocked_seconds = (
            self.get_clock().now() - self.safety_blocked_at
        ).nanoseconds / 1e9
        timeout = float(self.get_parameter("safety_abort_timeout_seconds").value)
        if self.safety_emergency or blocked_seconds >= timeout:
            self.request_abort(f"LiDAR safety condition: {self.safety_reason}")

    def control_tick(self) -> None:
        self.apply_safety_guard()
        self.publish_offboard_mode()
        self.publish_setpoint()

        if self.state in (MissionState.BOOT, MissionState.LAND_COMPLETE, MissionState.ABORT):
            return

        if self.position is not None and not self.inside_geofence():
            self.get_logger().error("PX4 position left the simulation geofence; aborting to RTL")
            self.request_abort("PX4 position left the simulation geofence")
            return

        if self.state == MissionState.PRECHECK:
            if self.elapsed_seconds() >= float(self.get_parameter("precheck_seconds").value):
                self.transition(MissionState.ARM_TAKEOFF)
            return

        if self.state == MissionState.ARM_TAKEOFF:
            if self.setpoint_count >= 20 and self.command_due() and self.ready_to_arm():
                self.send_offboard_and_arm()
            if self.at_target(self.home, 0.6) and self.is_armed():
                self.transition(MissionState.HOME_ANCHOR)
            return

        if self.state == MissionState.HOME_ANCHOR:
            if self.elapsed_seconds() >= float(self.get_parameter("hold_seconds").value):
                self.path_index = 0
                self.transition(MissionState.EXPLORE)
            return

        if self.state == MissionState.EXPLORE:
            if self.at_target(self.waypoints[self.path_index]):
                if self.path_index + 1 < len(self.waypoints):
                    self.path_index += 1
                else:
                    self.transition(MissionState.SURVIVOR_VERIFY)
            return

        if self.state == MissionState.SURVIVOR_VERIFY:
            if self.elapsed_seconds() >= float(self.get_parameter("hold_seconds").value):
                self.transition(MissionState.MARK_GRID)
            return

        if self.state == MissionState.MARK_GRID:
            if self.elapsed_seconds() >= 1.0:
                self.transition(MissionState.RETURN_HOME)
            return

        if self.state == MissionState.RETURN_HOME:
            horizontal_home = (
                self.position is not None
                and self.position.xy_valid
                and math.hypot(float(self.position.x) - self.home[0], float(self.position.y) - self.home[1]) <= 0.9
            )
            if self.at_target(self.home, 0.9) or (horizontal_home and self.position.z > -0.35):
                if not self.land_requested:
                    self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.land_requested = True
                    self.state_entered_at = self.get_clock().now()
                on_ground = self.position is not None and self.position.z > -0.35
                if on_ground and self.is_armed() and self.elapsed_seconds() >= 3.0 and not self.disarm_requested:
                    self.send_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        param1=0.0,
                        param2=21196.0 if bool(self.get_parameter("simulation_mode").value) else 0.0,
                    )
                    self.disarm_requested = True
                if self.elapsed_seconds() >= float(self.get_parameter("land_timeout_seconds").value) or (
                    on_ground and not self.is_armed()
                ):
                    self.transition(MissionState.LAND_COMPLETE)

    def status_tick(self) -> None:
        self.publish_state()

    def publish_offboard_mode(self) -> None:
        message = OffboardControlMode()
        message.timestamp = self.timestamp()
        message.position = self.state not in (MissionState.ABORT, MissionState.LAND_COMPLETE)
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False
        self.offboard_pub.publish(message)

    def publish_setpoint(self) -> None:
        message = TrajectorySetpoint()
        message.timestamp = self.timestamp()
        target = self.target_for_state()
        message.position = [float(target[0]), float(target[1]), float(target[2])]
        message.velocity = [math.nan, math.nan, math.nan]
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = 0.0
        message.yawspeed = math.nan
        self.setpoint_pub.publish(message)
        self.setpoint_count += 1

    def target_for_state(self) -> tuple[float, float, float]:
        if self.safety_hold_active():
            if self.position is not None and self.position.xy_valid and self.position.z_valid:
                return (float(self.position.x), float(self.position.y), float(self.position.z))
            return self.home
        if self.state == MissionState.EXPLORE:
            return self.waypoints[self.path_index]
        if self.state == MissionState.RETURN_HOME:
            return self.home
        if self.state == MissionState.LAND_COMPLETE:
            return (self.home[0], self.home[1], 0.0)
        return self.home

    def send_offboard_and_arm(self) -> None:
        self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            param2=0.0,
        )

    def ready_to_arm(self) -> bool:
        """Require a valid estimator and PX4 preflight checks before arming."""
        return (
            self.position is not None
            and self.position.xy_valid
            and self.position.z_valid
            and self.vehicle_status is not None
            and self.vehicle_status.pre_flight_checks_pass
            and (
                not bool(self.get_parameter("require_lidar").value)
                or (self.lidar_ready and self.lidar_safe and not self.safety_emergency)
            )
        )

    def safety_hold_active(self) -> bool:
        """Return whether the current setpoint should hold the vehicle in place."""
        if not bool(self.get_parameter("require_lidar").value):
            return False
        if self.state in (
            MissionState.BOOT,
            MissionState.PRECHECK,
            MissionState.RETURN_HOME,
            MissionState.LAND_COMPLETE,
            MissionState.ABORT,
        ):
            return False
        return not self.lidar_ready or not self.lidar_safe

    def send_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        message = VehicleCommand()
        message.timestamp = self.timestamp()
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.command = int(command)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.confirmation = 0
        message.from_external = True
        self.command_pub.publish(message)
        self.last_command_at = self.get_clock().now()

    def request_abort(self, reason: str) -> None:
        """Request the tested PX4 return-to-launch action once."""
        if self.abort_requested:
            return
        self.abort_requested = True
        self.get_logger().error(reason)
        self.transition(MissionState.ABORT)
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def command_due(self) -> bool:
        return (self.get_clock().now() - self.last_command_at).nanoseconds >= 1_000_000_000

    def at_target(self, target: tuple[float, float, float], acceptance: float | None = None) -> bool:
        if self.position is None or not self.position.xy_valid or not self.position.z_valid:
            return False
        radius = acceptance if acceptance is not None else float(self.get_parameter("waypoint_acceptance").value)
        horizontal = math.hypot(float(self.position.x) - target[0], float(self.position.y) - target[1])
        vertical = abs(float(self.position.z) - target[2])
        return horizontal <= radius and vertical <= max(0.5, radius)

    def is_armed(self) -> bool:
        return self.vehicle_status is not None and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def inside_geofence(self) -> bool:
        if self.position is None:
            return True
        return (
            float(self.get_parameter("geofence_x_min").value) <= float(self.position.x) <= float(self.get_parameter("geofence_x_max").value)
            and float(self.get_parameter("geofence_y_min").value) <= float(self.position.y) <= float(self.get_parameter("geofence_y_max").value)
            and float(self.get_parameter("geofence_z_min").value) <= float(self.position.z) <= float(self.get_parameter("geofence_z_max").value)
        )

    def transition(self, state: MissionState) -> None:
        self.state = state
        self.state_entered_at = self.get_clock().now()
        self.get_logger().info(f"AirMouse mission state: {state.value}")
        self.publish_state()

    def elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self.state_entered_at).nanoseconds / 1e9

    def timestamp(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def publish_plan(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "plan": "maze_corridor_room_loop",
                "frame": "px4_local_ned",
                "home": {"x": self.home[0], "y": self.home[1], "z": self.home[2]},
                "waypoints": [
                    {"index": index, "x": point[0], "y": point[1], "z": point[2]}
                    for index, point in enumerate(self.waypoints)
                ],
                "abort_action": "PX4_NAV_RETURN_TO_LAUNCH",
            },
            sort_keys=True,
        )
        self.plan_pub.publish(message)

    def publish_state(self) -> None:
        state_message = String()
        state_message.data = self.state.value
        self.state_pub.publish(state_message)
        status = String()
        status.data = json.dumps(
            {
                "mission_state": self.state.value,
                "started": self.started,
                "simulation_mode": bool(self.get_parameter("simulation_mode").value),
                "control_authority": "PX4_OFFBOARD",
                "abort_available": True,
                "armed": self.is_armed(),
                "position": self.position_dict(),
                "waypoint_index": self.path_index,
                "waypoint_count": len(self.waypoints),
                "validated_survivors": self.survivor_count,
                "lidar_ready": self.lidar_ready,
                "lidar_safe": self.lidar_safe,
                "safety_reason": self.safety_reason,
            },
            sort_keys=True,
        )
        self.status_pub.publish(status)

    def position_dict(self) -> dict | None:
        if self.position is None:
            return None
        return {"x": float(self.position.x), "y": float(self.position.y), "z": float(self.position.z)}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
