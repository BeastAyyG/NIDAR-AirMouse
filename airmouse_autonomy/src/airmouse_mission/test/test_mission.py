"""Unit tests for the mission lane cores (ROS-independent).

Cross-package dependencies (mapper, tracker) are STUBBED here; no other
airmouse package is imported.
"""
from __future__ import annotations

import numpy as np
import pytest

from airmouse_mission.mission_fsm import MissionFSM
from airmouse_mission.mission_monitor import MissionMonitor

FSM_PARAMS = {
    "sensor_loss_timeout_s": 2.0,
    "loc_loss_timeout_s": 2.0,
    "comms_loss_timeout_s": 1.0,
    "hold_timeout_s": 10.0,
}

INPUTS_OK = {
    "sensors_ok": True,
    "loc_ok": True,
    "comms_ok": True,
    "mission_complete": False,
    "at_entry": False,
}

REPORT_KEYS = {
    "explored_pct",
    "accessible_explored_pct",
    "survivor_count",
    "count_confidence",
    "regions",
    "findings",
    "tracks",
    "timestamp",
    "remaining_areas",
}


# ---------------------------------------------------------------- stubs


class StubMapper:
    """Minimal duck-typed mapper: grid + three mask/fraction methods."""

    def __init__(self, grid, observed, reachable, explored):
        self.grid = grid
        self._observed = observed
        self._reachable = reachable
        self._explored = explored

    def observed_mask(self):
        return self._observed

    def reachable_free_mask(self):
        return self._reachable

    def explored_fraction(self):
        return self._explored


class StubTracker:
    """Minimal duck-typed tracker: live list of track record dicts."""

    def __init__(self, tracks):
        self.tracks = tracks


def make_track(survivor_id, x, y, confidence, state, timestamp=0.0):
    return {
        "survivor_id": survivor_id,
        "detected": True,
        "position_x": x,
        "position_y": y,
        "confidence": confidence,
        "timestamp": timestamp,
        "state": state,
        "sightings": 3 if state == "confirmed" else 1,
    }


def make_regions():
    # bounds are inclusive grid-cell indices (x=col, y=row) on a 4x8 grid.
    return [
        {"region_id": 1, "center": (1.0, 1.0), "bounds": (0, 0, 3, 3), "accessible": True},
        {"region_id": 2, "center": (5.0, 1.0), "bounds": (4, 0, 7, 3), "accessible": False},
    ]


def make_monitor(tracks=(), observed=None, reachable=None, explored=0.5, regions=None):
    grid = np.full((4, 8), -1, dtype=np.int8)
    grid[:, :4] = 0  # region 1 cells are known free
    if observed is None:
        observed = np.zeros((4, 8), dtype=bool)
        observed[:, :4] = True  # region 1 fully observed
    if reachable is None:
        reachable = np.zeros((4, 8), dtype=bool)
        reachable[:, :4] = True
    mapper = StubMapper(grid, observed, reachable, explored)
    return MissionMonitor(
        make_regions() if regions is None else regions, mapper, StubTracker(list(tracks))
    )


# ---------------------------------------------------------------- monitor


def test_report_has_exact_schema():
    monitor = make_monitor()
    monitor.update(1.0)
    report = monitor.report()
    assert set(report.keys()) == REPORT_KEYS
    assert report["timestamp"] == pytest.approx(1.0)


def test_survivor_count_is_confirmed_plus_rounded_uncertain_mass():
    tracks = [
        make_track(0, 1.0, 1.0, 0.9, "confirmed"),
        make_track(1, 2.0, 2.0, 0.9, "confirmed"),
        make_track(2, 1.0, 2.0, 0.6, "uncertain"),
        make_track(3, 2.0, 1.0, 0.6, "uncertain"),
    ]
    monitor = make_monitor(tracks)
    monitor.update(1.0)
    report = monitor.report()
    # 2 confirmed + round(1.2) = 3; half-to-even rounding: round(0.5) == 0.
    assert report["survivor_count"] == 3
    assert 0.0 <= report["count_confidence"] <= 1.0

    half = make_monitor([make_track(0, 1.0, 1.0, 0.9, "confirmed"),
                         make_track(1, 2.0, 2.0, 0.5, "uncertain")])
    half.update(1.0)
    assert half.report()["survivor_count"] == 1  # 1 + round(0.5) = 1 + 0

    empty = make_monitor([])
    empty.update(1.0)
    assert empty.report()["survivor_count"] == 0
    assert empty.report()["count_confidence"] == 0.0


def test_region_status_explored_and_inaccessible():
    monitor = make_monitor()
    monitor.update(1.0)
    regions = {r["region_id"]: r for r in monitor.report()["regions"]}
    assert regions[1]["status"] == "explored"
    assert regions[1]["explored_frac"] == pytest.approx(1.0)
    assert regions[2]["status"] == "inaccessible"
    assert regions[2]["explored_frac"] == pytest.approx(0.0)


def test_region_partial_and_unexplored_thresholds():
    observed = np.zeros((4, 8), dtype=bool)
    observed[:2, :4] = True  # half of region 1 -> partial
    monitor = make_monitor(observed=observed)
    monitor.update(1.0)
    regions = {r["region_id"]: r for r in monitor.report()["regions"]}
    assert regions[1]["status"] == "partial"
    assert regions[1]["explored_frac"] == pytest.approx(0.5)

    unexplored = make_monitor(observed=np.zeros((4, 8), dtype=bool), explored=0.0)
    unexplored.update(1.0)
    regions = {r["region_id"]: r for r in unexplored.report()["regions"]}
    assert regions[1]["status"] == "unexplored"


def test_explored_pct_is_clamped_to_0_100():
    high = make_monitor(explored=1.5)
    high.update(1.0)
    assert high.report()["explored_pct"] == pytest.approx(100.0)
    low = make_monitor(explored=-0.2)
    low.update(1.0)
    assert low.report()["explored_pct"] == pytest.approx(0.0)


def test_accessible_explored_pct():
    monitor = make_monitor()
    monitor.update(1.0)
    # 16 observed free cells / 16 reachable free cells = 100%.
    assert monitor.report()["accessible_explored_pct"] == pytest.approx(100.0)
    none_reachable = make_monitor(reachable=np.zeros((4, 8), dtype=bool))
    none_reachable.update(1.0)
    assert none_reachable.report()["accessible_explored_pct"] == pytest.approx(0.0)


def test_findings_prioritized_with_stable_timestamps():
    tracks = [
        make_track(0, 1.0, 1.0, 0.95, "confirmed"),
        make_track(1, 2.0, 2.0, 0.4, "uncertain"),
    ]
    monitor = make_monitor(tracks)
    monitor.update(1.0)
    monitor.update(2.0)
    findings = monitor.report()["findings"]
    assert all(set(f.keys()) == {"priority", "text", "timestamp"} for f in findings)
    priorities = [f["priority"] for f in findings]
    assert priorities == sorted(priorities)
    confirmed = [f for f in findings if f["priority"] == 1]
    uncertain = [f for f in findings if f["priority"] == 2]
    assert len(confirmed) == 1 and len(uncertain) == 1
    # Identical finding text keeps its original timestamp across updates.
    assert confirmed[0]["timestamp"] == pytest.approx(1.0)
    assert uncertain[0]["timestamp"] == pytest.approx(1.0)


def test_remaining_areas_fallback_to_unexplored_region_centers():
    regions = make_regions() + [
        {"region_id": 3, "center": (6.0, 3.0), "bounds": (4, 2, 7, 3), "accessible": True},
    ]
    observed = np.zeros((4, 8), dtype=bool)
    observed[:, :4] = True  # region 1 fully observed -> explored
    observed[2, 4] = True  # region 3 barely observed (frac 0.125) -> unexplored
    monitor = make_monitor(observed=observed, regions=regions)
    monitor.update(1.0)
    report = monitor.report()
    statuses = {r["region_id"]: r["status"] for r in report["regions"]}
    assert statuses[3] == "unexplored"
    assert (6.0, 3.0) in report["remaining_areas"]
    assert (5.0, 1.0) not in report["remaining_areas"]  # inaccessible excluded


def test_report_tracks_mirror_tracker_records():
    tracks = [make_track(0, 1.0, 1.0, 0.9, "confirmed")]
    monitor = make_monitor(tracks)
    monitor.update(1.0)
    assert monitor.report()["tracks"] == tracks


def test_report_before_update_is_safe():
    report = make_monitor().report()
    assert set(report.keys()) == REPORT_KEYS
    assert report["timestamp"] == pytest.approx(0.0)
    assert report["explored_pct"] == pytest.approx(0.0)


def test_invalid_region_definition_raises():
    with pytest.raises(ValueError):
        MissionMonitor([{"region_id": 1}], StubMapper(None, None, None, 0.0), StubTracker([]))


# ---------------------------------------------------------------- FSM


def make_fsm(**overrides) -> MissionFSM:
    return MissionFSM({**FSM_PARAMS, **overrides})


def test_fsm_starts_in_explore():
    fsm = make_fsm()
    assert fsm.state == "EXPLORE"
    assert fsm.tick(0.0, dict(INPUTS_OK)) == "EXPLORE"


def test_sensor_loss_holds_then_failsafes_after_timeout():
    fsm = make_fsm()
    fsm.tick(0.0, dict(INPUTS_OK))
    fsm.on_sensor_loss()
    assert fsm.tick(0.5, {**INPUTS_OK, "sensors_ok": False}) == "HOLD"
    assert fsm.state == "HOLD"
    assert fsm.tick(1.9, {**INPUTS_OK, "sensors_ok": False}) == "HOLD"
    # Loss persisted >= sensor_loss_timeout_s (2.0) -> FAILSAFE.
    assert fsm.tick(2.0, {**INPUTS_OK, "sensors_ok": False}) == "FAILSAFE"
    assert fsm.state == "FAILSAFE"


@pytest.mark.parametrize(
    "loss_method,ok_key,timeout",
    [
        ("on_loc_loss", "loc_ok", 2.0),
        ("on_comms_loss", "comms_ok", 1.0),
    ],
)
def test_loc_and_comms_loss_hold_then_failsafe(loss_method, ok_key, timeout):
    fsm = make_fsm()
    fsm.tick(0.0, dict(INPUTS_OK))
    getattr(fsm, loss_method)()
    assert fsm.tick(0.25, {**INPUTS_OK, ok_key: False}) == "HOLD"
    assert fsm.tick(timeout + 0.1, {**INPUTS_OK, ok_key: False}) == "FAILSAFE"


def test_loss_recovery_resumes_explore():
    fsm = make_fsm()
    fsm.tick(0.0, dict(INPUTS_OK))
    fsm.on_sensor_loss()
    assert fsm.tick(0.5, {**INPUTS_OK, "sensors_ok": False}) == "HOLD"
    assert fsm.tick(1.0, dict(INPUTS_OK)) == "EXPLORE"
    assert fsm.state == "EXPLORE"


def test_mission_complete_return_land_done_sequence():
    fsm = make_fsm()
    fsm.tick(0.0, dict(INPUTS_OK))
    assert fsm.tick(1.0, {**INPUTS_OK, "mission_complete": True}) == "RETURN"
    assert fsm.state == "RETURN"
    assert fsm.tick(2.0, {**INPUTS_OK, "mission_complete": True, "at_entry": True}) == "LAND"
    assert fsm.state == "LAND"
    assert fsm.tick(3.0, {**INPUTS_OK, "mission_complete": True, "at_entry": True}) == "DONE"
    assert fsm.state == "DONE"
    assert fsm.tick(4.0, dict(INPUTS_OK)) == "DONE"  # terminal


def test_at_entry_without_mission_complete_does_not_land():
    fsm = make_fsm()
    assert fsm.tick(0.0, {**INPUTS_OK, "at_entry": True}) == "EXPLORE"


def test_hold_timeout_backstop_escalates_to_failsafe():
    fsm = make_fsm(sensor_loss_timeout_s=100.0, hold_timeout_s=2.0)
    fsm.tick(0.0, dict(INPUTS_OK))
    fsm.on_sensor_loss()
    assert fsm.tick(1.0, {**INPUTS_OK, "sensors_ok": False}) == "HOLD"
    # Per-loss timeout far away, but continuous HOLD exceeds hold_timeout_s.
    assert fsm.tick(3.5, {**INPUTS_OK, "sensors_ok": False}) == "FAILSAFE"


def test_failsafe_is_terminal_even_after_recovery():
    fsm = make_fsm()
    fsm.tick(0.0, dict(INPUTS_OK))
    fsm.on_sensor_loss()
    fsm.tick(3.0, {**INPUTS_OK, "sensors_ok": False})
    assert fsm.state == "FAILSAFE"
    assert fsm.tick(4.0, dict(INPUTS_OK)) == "FAILSAFE"


def test_loss_during_return_resumes_return():
    fsm = make_fsm()
    fsm.tick(0.0, dict(INPUTS_OK))
    fsm.tick(1.0, {**INPUTS_OK, "mission_complete": True})
    assert fsm.state == "RETURN"
    fsm.on_loc_loss()
    assert fsm.tick(1.5, {**INPUTS_OK, "mission_complete": True, "loc_ok": False}) == "HOLD"
    assert fsm.tick(2.0, {**INPUTS_OK, "mission_complete": True}) == "RETURN"


def test_tick_detects_loss_from_inputs_without_explicit_notification():
    fsm = make_fsm()
    fsm.tick(0.0, dict(INPUTS_OK))
    assert fsm.tick(0.5, {**INPUTS_OK, "sensors_ok": False}) == "HOLD"
    assert fsm.tick(1.0, {**INPUTS_OK, "sensors_ok": False}) == "HOLD"
    assert fsm.tick(3.0, {**INPUTS_OK, "sensors_ok": False}) == "FAILSAFE"


def test_missing_input_key_raises_value_error():
    fsm = make_fsm()
    with pytest.raises(ValueError):
        fsm.tick(0.0, {"sensors_ok": True})


@pytest.mark.parametrize("override", [
    {"sensor_loss_timeout_s": -1.0},
    {"loc_loss_timeout_s": -1.0},
    {"comms_loss_timeout_s": -1.0},
    {"hold_timeout_s": -1.0},
])
def test_invalid_fsm_params_raise(override):
    with pytest.raises(ValueError):
        make_fsm(**override)