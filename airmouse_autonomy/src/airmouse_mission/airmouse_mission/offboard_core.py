"""PX4 offboard sequencing core (ROS-independent, stdlib only).

The autonomy stack commands body-frame velocity (`/cmd_vel`, REP-103: x
forward, y left, z up). PX4 consumes NED local-frame velocity setpoints that
must stream continuously while in OFFBOARD mode. This core owns that contract
explicitly so it can be unit-tested without ROS:

* stream setpoints at ``rate_hz`` (>= 2 Hz required by PX4, 10 Hz default);
* request OFFBOARD only after ``prestream_s`` of continuous streaming;
* request arming only once PX4 reports OFFBOARD active;
* command loss (> ``cmd_timeout_s``) -> HOLD (zero velocity), and if it
  persists past ``hold_timeout_s`` -> LAND (fail-safe);
* explicit HOLD/LAND from the mission node override autonomy commands.

Frame note: PX4 local NED is x=north, y=east, z=down. Gazebo/ROS are ENU
(x=east, y=north, z=up). ``body_to_ned`` performs both rotations at once.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class OffboardParams:
    rate_hz: float = 10.0
    prestream_s: float = 1.0
    cmd_timeout_s: float = 0.5
    hold_timeout_s: float = 10.0
    max_speed_mps: float = 1.5
    yaw_rate_limit_rps: float = 1.0


@dataclass(frozen=True)
class Px4Status:
    connected: bool = False
    armed: bool = False
    offboard: bool = False


@dataclass(frozen=True)
class OffboardAction:
    """What the node should do on this tick (all flags are one-shot edges)."""

    state: str                    # IDLE / STREAMING / OFFBOARD / HOLD / LANDING
    stream: bool                  # publish a setpoint this tick
    setpoint: tuple[float, float, float, float]  # vn, ve, vd, yaw_rate (NED)
    request_offboard: bool
    request_arm: bool
    request_land: bool


def body_to_ned(vx_body: float, vy_body: float, vz_up: float,
                yaw_enu: float) -> tuple[float, float, float]:
    """Body-frame ENU velocity -> world NED velocity.

    ``yaw_enu`` = heading in radians, 0 = +x (east), increasing toward +y
    (north), matching the rest of the workspace (y DOWN in the map frame is a
    separate 2D convention; the world frame here is ENU).
    """
    cos_y, sin_y = math.cos(yaw_enu), math.sin(yaw_enu)
    v_east = vx_body * cos_y - vy_body * sin_y
    v_north = vx_body * sin_y + vy_body * cos_y
    return (v_north, v_east, -vz_up)


class OffboardSequencer:
    """Deterministic action sequencer for a PX4 offboard velocity bridge."""

    def __init__(self, params: OffboardParams | None = None) -> None:
        self.params = params or OffboardParams()
        if self.params.rate_hz < 2.0:
            raise ValueError("rate_hz must be >= 2.0 (PX4 requirement)")
        self.state = "IDLE"
        self._stream_since: float | None = None
        self._last_stream_t = float("-inf")
        self._last_cmd_t = float("-inf")
        self._land_latched = False
        self._prev_offboard_req = False
        self._prev_arm_req = False
        self._prev_land_req = False

    # -- inputs -------------------------------------------------------------
    def note_command(self, now: float) -> None:
        """Autonomy issued a velocity command (/cmd_vel) at ``now``."""
        self._last_cmd_t = now

    def request_land(self) -> None:
        self._land_latched = True

    def clear_land(self) -> None:
        self._land_latched = False

    # -- tick -----------------------------------------------------------------
    def step(self, now: float, status: Px4Status,
             vx_body: float = 0.0, vy_body: float = 0.0, vz_up: float = 0.0,
             yaw_enu: float = 0.0, yaw_rate: float = 0.0,
             hold: bool = False, land: bool = False) -> OffboardAction:
        p = self.params
        if land:
            self._land_latched = True
        has_cmd = self._last_cmd_t != float("-inf")
        command_fresh = has_cmd and (now - self._last_cmd_t) <= p.cmd_timeout_s
        command_lost = has_cmd and (now - self._last_cmd_t) > p.hold_timeout_s
        if command_lost:
            self._land_latched = True  # fail-safe landing is sticky

        if self._land_latched or command_lost:
            self.state = "LANDING"
            vx_body = vy_body = vz_up = yaw_rate = 0.0
        elif hold or (has_cmd and not command_fresh):
            self.state = "HOLD"
            vx_body = vy_body = vz_up = yaw_rate = 0.0
        elif status.offboard and status.armed:
            self.state = "OFFBOARD"
        else:
            self.state = "STREAMING"

        speed = math.hypot(vx_body, vy_body)
        if speed > p.max_speed_mps:
            scale = p.max_speed_mps / speed
            vx_body, vy_body = vx_body * scale, vy_body * scale
        yaw_rate = max(-p.yaw_rate_limit_rps, min(p.yaw_rate_limit_rps, yaw_rate))

        vn, ve, vd = body_to_ned(vx_body, vy_body, vz_up, yaw_enu)

        interval = 1.0 / p.rate_hz
        stream = (now - self._last_stream_t) >= interval - 1e-9
        if stream:
            self._last_stream_t = now
            if self._stream_since is None:
                self._stream_since = now

        prestreamed = (self._stream_since is not None
                       and now - self._stream_since >= p.prestream_s)
        # Never arm or switch to OFFBOARD until the autonomy stack has actually
        # commanded motion: a bridge that arms on startup is a safety hazard.
        request_offboard = (status.connected and not status.offboard
                            and prestreamed and (command_fresh or hold)
                            and not self._land_latched)
        request_arm = (status.connected and status.offboard
                       and not status.armed and not self._land_latched)
        request_land = self._land_latched and not self._prev_land_req

        action = OffboardAction(
            state=self.state,
            stream=stream,
            setpoint=(vn, ve, vd, yaw_rate),
            request_offboard=request_offboard and not self._prev_offboard_req,
            request_arm=request_arm and not self._prev_arm_req,
            request_land=request_land,
        )
        self._prev_offboard_req = request_offboard
        self._prev_arm_req = request_arm
        self._prev_land_req = self._land_latched
        return action