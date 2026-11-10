"""Tests for the PX4 offboard sequencing core (no ROS required)."""
import math

import pytest

from airmouse_mission.offboard_core import (
    OffboardParams,
    OffboardSequencer,
    Px4Status,
    body_to_ned,
)

DISCONNECTED = Px4Status(connected=False)
CONNECTED = Px4Status(connected=True)
OFFBOARD = Px4Status(connected=True, armed=False, offboard=True)
FLYING = Px4Status(connected=True, armed=True, offboard=True)


def test_body_to_ned_facing_east():
    vn, ve, vd = body_to_ned(1.0, 0.0, 0.0, yaw_enu=0.0)
    assert (round(vn, 6), round(ve, 6), round(vd, 6)) == (0.0, 1.0, -0.0)


def test_body_to_ned_facing_north_and_left_strafe():
    vn, ve, vd = body_to_ned(1.0, 0.0, 0.0, yaw_enu=math.pi / 2)
    assert (round(vn, 6), round(ve, 6)) == (1.0, 0.0)
    # Facing north, body-left is west (negative east).
    vn, ve, _ = body_to_ned(0.0, 1.0, 0.0, yaw_enu=math.pi / 2)
    assert (round(vn, 6), round(ve, 6)) == (0.0, -1.0)


def test_body_to_ned_up_is_down():
    assert body_to_ned(0.0, 0.0, 0.5, 0.0)[2] == pytest.approx(-0.5)


def test_no_arming_before_any_command():
    seq = OffboardSequencer(OffboardParams(rate_hz=10.0, prestream_s=1.0))
    actions = [seq.step(t, CONNECTED) for t in (0.0, 0.5, 1.5, 3.0)]
    assert not any(a.request_offboard for a in actions)
    assert not any(a.request_arm for a in actions)
    assert all(a.state == "STREAMING" for a in actions)


def test_prestream_then_offboard_then_arm_edges():
    seq = OffboardSequencer(OffboardParams(rate_hz=10.0, prestream_s=1.0,
                                           cmd_timeout_s=0.5))
    seq.note_command(0.0)
    first = seq.step(0.0, CONNECTED)
    assert not first.request_offboard
    offboard_reqs = []
    t = 0.0
    while t <= 1.2:
        seq.note_command(t)
        offboard_reqs.append(seq.step(t, CONNECTED).request_offboard)
        t += 0.1
    assert offboard_reqs.count(True) == 1  # one-shot edge after prestream
    seq.note_command(t)
    assert seq.step(t, OFFBOARD).request_arm is True
    seq.note_command(t + 0.1)
    assert seq.step(t + 0.1, OFFBOARD).request_arm is False  # edge only


def test_stream_rate_is_limited_to_rate_hz():
    seq = OffboardSequencer(OffboardParams(rate_hz=10.0))
    seq.note_command(0.0)
    streams = 0
    for i in range(101):  # 1 s at 100 Hz calls
        seq.note_command(i * 0.01)
        streams += seq.step(i * 0.01, FLYING).stream
    assert 9 <= streams <= 11


def test_command_timeout_holds_then_lands():
    seq = OffboardSequencer(OffboardParams(rate_hz=10.0, cmd_timeout_s=0.5,
                                           hold_timeout_s=2.0))
    seq.note_command(0.0)
    seq.step(0.0, FLYING, vx_body=1.0)
    hold = seq.step(0.6, FLYING)  # 0.6 > 0.5 -> stale
    assert hold.state == "HOLD"
    assert hold.setpoint[:3] == (0.0, 0.0, 0.0)
    land_actions = [seq.step(t, FLYING) for t in (2.5, 2.6, 2.7)]
    assert land_actions[0].state == "LANDING"
    assert land_actions[0].request_land is True
    assert not any(a.request_land for a in land_actions[1:])


def test_explicit_hold_and_land():
    seq = OffboardSequencer()
    seq.note_command(0.0)
    action = seq.step(0.0, FLYING, vx_body=1.0, hold=True)
    assert action.state == "HOLD"
    assert action.setpoint[:3] == (0.0, 0.0, 0.0)
    land = seq.step(0.1, FLYING, land=True)
    assert land.state == "LANDING" and land.request_land is True


def test_speed_and_yaw_rate_are_clamped():
    seq = OffboardSequencer(OffboardParams(rate_hz=10.0, max_speed_mps=1.5,
                                           yaw_rate_limit_rps=0.8))
    seq.note_command(0.0)
    action = seq.step(0.0, FLYING, vx_body=3.0, vy_body=4.0, yaw_rate=5.0)
    vn, ve, vd, yaw_rate = action.setpoint
    assert math.hypot(vn, ve) == pytest.approx(1.5)
    assert yaw_rate == pytest.approx(0.8)


def test_rate_below_px4_minimum_is_rejected():
    with pytest.raises(ValueError):
        OffboardSequencer(OffboardParams(rate_hz=1.0))