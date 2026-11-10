"""rclpy adapter for the mission monitor and failsafe FSM cores.

Thin conversion layer only: ``/map`` refreshes the mapper view used by
``MissionMonitor``, ``/survivors`` fills a lightweight tracker view, and a
1 Hz tick feeds ``MissionFSM.tick`` with recency/entry/complete inputs. All
coverage, region, findings, reporting and state-transition logic lives in
``airmouse_mission``.

Topics
    in : ``/map``              nav_msgs/OccupancyGrid (transient_local)
         ``/survivors``        airmouse_interfaces/SurvivorTrack
         ``/mission_complete`` std_msgs/Bool (transient_local)
         TF ``map`` -> ``base_link``
    out: ``/mission_status``   airmouse_interfaces/MissionStatus
         ``/mission_state``    std_msgs/String (state name)
         ``/mission_command``  std_msgs/String (``HOLD`` / ``LAND`` for the
                               PX4 bridge, published on change only)
         ``/goal``             geometry_msgs/PoseStamped to the entry while
                               the FSM is in RETURN

Command mapping: HOLD -> ``HOLD``, FAILSAFE -> ``LAND``, LAND -> ``LAND``.
The flight-time limit (``time_limit_sim_s``) is treated as mission complete so
the FSM returns home before the sim budget expires.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_ros
from airmouse_interfaces.msg import MissionStatus, SurvivorTrack
from airmouse_mapping.grid_manager import mapper_from_grid
from airmouse_mapping.occupancy_core import UNKNOWN
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String

from airmouse_mission.mission_fsm import FAILSAFE, HOLD, LAND, RETURN, MissionFSM
from airmouse_mission.mission_monitor import MissionMonitor

NODE_NAME = "mission_node"
MAP_TOPIC = "/map"
TRACKS_TOPIC = "/survivors"
MISSION_COMPLETE_TOPIC = "/mission_complete"
GOAL_TOPIC = "/goal"
STATUS_TOPIC = "/mission_status"
STATE_TOPIC = "/mission_state"
COMMAND_TOPIC = "/mission_command"

MAP_FRAME = "map"
BASE_FRAME = "base_link"

TICK_PERIOD_S = 1.0  # 1 Hz monitor/FSM cadence
ENTRY_RADIUS_M = 0.4
REGIONS_PER_AXIS = 3  # 15 m maze -> 3x3 approximate world regions
RETURN_GOAL_REPUBLISH_S = 10.0

COMMAND_HOLD = "HOLD"
COMMAND_LAND = "LAND"
_COMMAND_BY_STATE = {HOLD: COMMAND_HOLD, FAILSAFE: COMMAND_LAND, LAND: COMMAND_LAND}

_TF_LOG_THROTTLE_S = 5.0
_MAX_SURVIVORS_LOG_THROTTLE_S = 10.0


def _latched_qos() -> QoSProfile:
    """Reliable transient-local QoS for map and one-shot flag topics."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def build_world_regions(width_m: float, height_m: float, cell_size: float,
                        per_axis: int = REGIONS_PER_AXIS) -> list[dict]:
    """Split the world into a ``per_axis`` x ``per_axis`` region grid.

    ``bounds`` are INCLUSIVE grid-cell indices ``(x0, y0, x1, y1)`` with
    x=column and y=row, matching the workspace ``grid[row, col]`` convention
    expected by ``MissionMonitor``.
    """
    if cell_size <= 0.0:
        raise ValueError(f"cell_size must be positive, got {cell_size!r}")
    if width_m <= 0.0 or height_m <= 0.0:
        raise ValueError(
            f"world extent must be positive, got ({width_m!r}, {height_m!r})")
    if per_axis < 1:
        raise ValueError(f"per_axis must be >= 1, got {per_axis!r}")

    cols = int(width_m / cell_size)
    rows = int(height_m / cell_size)
    regions: list[dict] = []
    for row_index in range(per_axis):
        for col_index in range(per_axis):
            c0 = int(round(col_index * cols / per_axis))
            c1 = int(round((col_index + 1) * cols / per_axis)) - 1
            r0 = int(round(row_index * rows / per_axis))
            r1 = int(round((row_index + 1) * rows / per_axis)) - 1
            if c1 < c0 or r1 < r0:
                continue
            regions.append({
                "region_id": row_index * per_axis + col_index + 1,
                "center": ((c0 + c1 + 1) * 0.5 * cell_size,
                           (r0 + r1 + 1) * 0.5 * cell_size),
                "bounds": (c0, r0, c1, r1),
                "accessible": True,
            })
    return regions


class _TrackerView:
    """Minimal tracker duck-type: ``MissionMonitor`` only reads ``.tracks``."""

    def __init__(self) -> None:
        self.tracks: list[dict] = []


class _MapperView:
    """``MissionMonitor``-compatible view over a ``mapper_from_grid`` result.

    ``mapper_from_grid`` rebuilds the grid and planning state only, so the
    mapper's internal observation record stays empty. For a *received* map the
    observation record is implicit in the cell values, hence observed/explored
    are derived from ``grid != UNKNOWN`` here (planning queries still delegate
    to the wrapped mapper).
    """

    def __init__(self, mapper) -> None:
        self.grid = mapper.grid
        self._mapper = mapper
        self._observed = np.asarray(mapper.grid) != UNKNOWN

    def observed_mask(self) -> np.ndarray:
        return self._observed.copy()

    def explored_fraction(self) -> float:
        total = int(self._observed.size)
        if total == 0:
            return 0.0
        return float(np.count_nonzero(self._observed)) / total

    def reachable_free_mask(self) -> np.ndarray:
        return self._mapper.reachable_free_mask()

    def frontiers(self) -> list[tuple[float, float]]:
        return self._mapper.frontiers()


class MissionNode(Node):
    """Publishes mission status/state and acts on FSM fail-safe commands."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.entry = (
            float(self.declare_parameter("entry_x", 0.0).value),
            float(self.declare_parameter("entry_y", 0.0).value),
        )
        self.time_limit_sim_s = float(
            self.declare_parameter("time_limit_sim_s", 1800.0).value)
        self.cell_size = float(self.declare_parameter("cell_size", 0.10).value)
        self.stale_timeout_s = float(
            self.declare_parameter("input_stale_timeout_s", 2.0).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 6.0).value)
        self.clearance_m = float(self.declare_parameter("clearance_m", 0.10).value)
        self.max_survivors = int(self.declare_parameter("max_survivors", 6).value)

        fsm_params = {
            "sensor_loss_timeout_s": float(
                self.declare_parameter("sensor_loss_timeout_s", 2.0).value),
            "loc_loss_timeout_s": float(
                self.declare_parameter("loc_loss_timeout_s", 2.0).value),
            "comms_loss_timeout_s": float(
                self.declare_parameter("comms_loss_timeout_s", 1.0).value),
            "hold_timeout_s": float(
                self.declare_parameter("hold_timeout_s", 10.0).value),
        }
        self._fsm = MissionFSM(fsm_params)

        self._regions = build_world_regions(
            float(self.declare_parameter("maze_width_m", 15.0).value),
            float(self.declare_parameter("maze_height_m", 15.0).value),
            self.cell_size,
        )
        self._tracker_view = _TrackerView()
        self._tracks: dict[int, dict] = {}
        self._map_view: _MapperView | None = None
        self._monitor: MissionMonitor | None = None

        self._start_s: float | None = None
        self._last_map_s: float | None = None
        self._last_input_s: float | None = None
        self._last_tf_s: float | None = None
        self._mission_complete_signal = False
        self._last_command: str | None = None
        self._last_return_goal_s: float | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, _latched_qos())
        self.create_subscription(SurvivorTrack, TRACKS_TOPIC, self._on_survivor, 10)
        self.create_subscription(
            Bool, MISSION_COMPLETE_TOPIC, self._on_mission_complete, _latched_qos())
        self._status_pub = self.create_publisher(MissionStatus, STATUS_TOPIC, 10)
        self._state_pub = self.create_publisher(String, STATE_TOPIC, 10)
        self._command_pub = self.create_publisher(String, COMMAND_TOPIC, 10)
        self._goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)

        self.create_timer(TICK_PERIOD_S, self._tick)
        self.get_logger().info(
            f"{NODE_NAME} ready: {len(self._regions)} regions, entry={self.entry}, "
            f"time_limit={self.time_limit_sim_s} s")

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

        self._map_view = _MapperView(mapper)
        self._last_map_s = self._now_s()
        self._last_input_s = self._last_map_s
        if self._monitor is None:
            self._monitor = MissionMonitor(
                self._regions, self._map_view, self._tracker_view)
            self.get_logger().info("mission monitor initialised from first /map")
        else:
            self._monitor.mapper = self._map_view

    def _on_survivor(self, msg: SurvivorTrack) -> None:
        now = self._now_s()
        self._tracks[int(msg.survivor_id)] = {
            "survivor_id": int(msg.survivor_id),
            "position_x": float(msg.position_x),
            "position_y": float(msg.position_y),
            "confidence": float(msg.confidence),
            "state": str(msg.state),
            "timestamp": float(msg.timestamp),
        }
        self._tracker_view.tracks = list(self._tracks.values())
        self._last_input_s = now

    def _on_mission_complete(self, msg: Bool) -> None:
        if msg.data:
            self._mission_complete_signal = True
        self._last_input_s = self._now_s()

    # -------------------------------------------------------------- ticks

    def _tick(self) -> None:
        now = self._now_s()
        if self._start_s is None:
            self._start_s = now
        pose = self._lookup_pose()
        if pose is not None:
            self._last_tf_s = now

        state = self._fsm.tick(now, self._inputs(now, pose))
        report = self._monitor_report(now)
        self._publish_status(state, report, now)
        self._state_pub.publish(String(data=state))
        self._publish_command(state)
        self._publish_return_goal(state, now)

    def _inputs(self, now: float, pose) -> dict:
        return {
            "sensors_ok": self._map_view is not None
                          and self._recent(self._last_map_s, now),
            "loc_ok": self._recent(self._last_tf_s, now),
            "comms_ok": self._recent(self._last_input_s, now),
            "mission_complete": self._mission_complete(now),
            "at_entry": pose is not None
                        and math.hypot(pose[0] - self.entry[0],
                                       pose[1] - self.entry[1]) <= ENTRY_RADIUS_M,
        }

    def _monitor_report(self, now: float) -> dict | None:
        if self._monitor is None:
            return None
        self._monitor.update(now)
        return self._monitor.report()

    def _mission_complete(self, now: float) -> bool:
        timed_out = (self._start_s is not None
                     and now - self._start_s >= self.time_limit_sim_s)
        return self._mission_complete_signal or timed_out

    # ------------------------------------------------------------ outputs

    def _publish_status(self, state: str, report: dict | None, now: float) -> None:
        msg = MissionStatus()
        msg.state = state
        if report is not None:
            msg.explored_pct = float(report["explored_pct"])
            msg.accessible_pct = float(report["accessible_explored_pct"])
            survivor_count = int(report["survivor_count"])
        else:
            msg.explored_pct = 0.0
            msg.accessible_pct = 0.0
            survivor_count = 0
        msg.survivor_count = survivor_count
        msg.mission_complete = bool(self._mission_complete(now))
        msg.timestamp = float(now)
        self._status_pub.publish(msg)

        if survivor_count > self.max_survivors:
            self.get_logger().warn(
                f"estimated survivors ({survivor_count}) exceed configured "
                f"max_survivors ({self.max_survivors}); possible duplicate tracks",
                throttle_duration_sec=_MAX_SURVIVORS_LOG_THROTTLE_S)

    def _publish_command(self, state: str) -> None:
        command = _COMMAND_BY_STATE.get(state)
        if command is None:
            self._last_command = None  # re-arm after leaving the fail-safe states
            return
        if command == self._last_command:
            return
        self._command_pub.publish(String(data=command))
        self._last_command = command
        self.get_logger().warn(f"mission_state={state}: published /mission_command {command}")

    def _publish_return_goal(self, state: str, now: float) -> None:
        if state != RETURN:
            self._last_return_goal_s = None
            return
        if (self._last_return_goal_s is not None
                and now - self._last_return_goal_s < RETURN_GOAL_REPUBLISH_S):
            return

        msg = PoseStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.entry[0])
        msg.pose.position.y = float(self.entry[1])
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)
        self._last_return_goal_s = now
        self.get_logger().info(
            f"{state}: published entry goal ({self.entry[0]:.2f}, {self.entry[1]:.2f})")

    # ------------------------------------------------------------- helpers

    def _recent(self, last_s: float | None, now: float) -> bool:
        return last_s is not None and now - last_s <= self.stale_timeout_s

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
    node = MissionNode()
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
