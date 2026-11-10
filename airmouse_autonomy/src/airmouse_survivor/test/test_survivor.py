"""Unit tests for the survivor lane cores (ROS-independent, no cross-package imports).

Covers: spatial-gate dedup, UNCERTAIN->CONFIRMED promotion (confidence AND
sightings), decay/drop, record schema, sim target detector range/determinism,
and the YOLO placeholder contract.
"""
from __future__ import annotations

import math

import pytest

from airmouse_survivor.survivor_detector import Detector, SimTargetDetector, YoloDetector
from airmouse_survivor.survivor_tracker import SurvivorTracker

PARAMS = {
    "merge_radius_m": 1.2,
    "confirm_confidence": 0.85,
    "confirm_sightings": 3,
    "decay_per_sec": 0.02,
    "drop_confidence": 0.12,
}

RECORD_KEYS = {
    "survivor_id",
    "detected",
    "position_x",
    "position_y",
    "confidence",
    "timestamp",
    "state",
    "sightings",
}


def make_tracker(**overrides) -> SurvivorTracker:
    return SurvivorTracker({**PARAMS, **overrides})


# ---------------------------------------------------------------- tracker


def test_duplicate_detections_within_merge_radius_make_one_track():
    tracker = make_tracker()
    tracker.update([(1.0, 1.0, 0.5, 0.0), (1.4, 1.3, 0.5, 0.0)], now=0.0)
    assert len(tracker.tracks) == 1
    record = tracker.tracks[0]
    assert record["sightings"] == 2
    assert record["state"] == "uncertain"


def test_detections_outside_merge_radius_make_separate_tracks():
    tracker = make_tracker()
    tracker.update([(0.0, 0.0, 0.5, 0.0), (5.0, 5.0, 0.5, 0.0)], now=0.0)
    assert len(tracker.tracks) == 2
    assert {t["survivor_id"] for t in tracker.tracks} == {0, 1}


def test_three_sightings_with_high_confidence_confirms_track():
    tracker = make_tracker()
    for t in (0.0, 1.0, 2.0):
        tracker.update([(2.0, 2.0, 0.85, t)], now=t)
    assert len(tracker.tracks) == 1
    record = tracker.tracks[0]
    assert record["state"] == "confirmed"
    assert record["sightings"] == 3
    assert record["confidence"] >= PARAMS["confirm_confidence"]


def test_promotion_requires_both_confidence_and_sightings():
    tracker = make_tracker()
    # High confidence but a single sighting: not confirmed.
    tracker.update([(0.0, 0.0, 0.99, 0.0)], now=0.0)
    assert tracker.tracks[0]["state"] == "uncertain"
    # Three sightings but aggregated confidence below threshold: not confirmed.
    other = make_tracker()
    for t in (0.0, 1.0, 2.0):
        other.update([(9.0, 9.0, 0.3, t)], now=t)
    assert other.tracks[0]["sightings"] == 3
    assert other.tracks[0]["state"] == "uncertain"


def test_decay_drops_stale_uncertain_track():
    tracker = make_tracker()
    tracker.update([(0.0, 0.0, 0.2, 0.0)], now=0.0)
    # 0.2 - 0.02 * 10 = 0.0 < drop_confidence 0.12 -> dropped.
    tracker.update([], now=10.0)
    assert tracker.tracks == []


def test_decay_reduces_confidence_but_keeps_track_above_threshold():
    tracker = make_tracker()
    tracker.update([(0.0, 0.0, 0.5, 0.0)], now=0.0)
    tracker.update([], now=10.0)
    assert len(tracker.tracks) == 1
    record = tracker.tracks[0]
    assert record["confidence"] == pytest.approx(0.3)
    assert record["detected"] is False  # not observed in the latest update


def test_confirmed_tracks_persist_without_new_sightings():
    tracker = make_tracker()
    for t in (0.0, 1.0, 2.0):
        tracker.update([(2.0, 2.0, 0.85, t)], now=t)
    tracker.update([], now=1000.0)
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0]["state"] == "confirmed"


def test_track_record_has_exact_schema():
    tracker = make_tracker()
    tracker.update([(3.0, 4.0, 0.5, 0.0)], now=0.0)
    record = tracker.tracks[0]
    assert set(record.keys()) == RECORD_KEYS
    assert isinstance(record["survivor_id"], int)
    assert isinstance(record["detected"], bool)
    assert record["detected"] is True
    assert record["position_x"] == pytest.approx(3.0)
    assert record["position_y"] == pytest.approx(4.0)
    assert record["timestamp"] == pytest.approx(0.0)
    assert record["state"] in {"uncertain", "confirmed"}
    assert isinstance(record["sightings"], int)


def test_uncertain_tracks_returns_only_uncertain_records():
    tracker = make_tracker(decay_per_sec=0.0)
    tracker.update([(0.0, 0.0, 0.85, 0.0), (10.0, 10.0, 0.4, 0.0)], now=0.0)
    for t in (1.0, 2.0):
        tracker.update([(0.0, 0.0, 0.85, t)], now=t)
    uncertain = tracker.uncertain_tracks()
    assert len(uncertain) == 1
    assert uncertain[0]["state"] == "uncertain"
    assert set(uncertain[0].keys()) == RECORD_KEYS


def test_survivor_count_is_confirmed_plus_rounded_uncertain_mass():
    tracker = make_tracker(decay_per_sec=0.0)
    tracker.update(
        [(0.0, 0.0, 0.85, 0.0), (10.0, 10.0, 0.6, 0.0), (20.0, 20.0, 0.6, 0.0)],
        now=0.0,
    )
    for t in (1.0, 2.0):
        tracker.update([(0.0, 0.0, 0.85, t)], now=t)
    count, confidence = tracker.survivor_count()
    # 1 confirmed + round(0.6 + 0.6) = 1 + 1 = 2.
    assert count == 2
    assert 0.0 <= confidence <= 1.0


def test_survivor_count_empty_tracker():
    assert make_tracker().survivor_count() == (0, 0.0)


def test_backwards_time_does_not_decay():
    tracker = make_tracker()
    tracker.update([(0.0, 0.0, 0.5, 5.0)], now=5.0)
    tracker.update([], now=3.0)
    assert tracker.tracks[0]["confidence"] == pytest.approx(0.5)


def test_detection_confidence_is_clipped():
    tracker = make_tracker()
    tracker.update([(0.0, 0.0, 1.5, 0.0)], now=0.0)
    assert tracker.tracks[0]["confidence"] == pytest.approx(1.0)


@pytest.mark.parametrize(
    "override",
    [
        {"merge_radius_m": -1.0},
        {"confirm_confidence": 1.5},
        {"confirm_sightings": 0},
        {"decay_per_sec": -0.1},
        {"drop_confidence": -0.5},
    ],
)
def test_invalid_params_raise_value_error(override):
    with pytest.raises(ValueError):
        make_tracker(**override)


def test_malformed_detection_raises_value_error():
    tracker = make_tracker()
    with pytest.raises(ValueError):
        tracker.update([(1.0, 2.0, 0.5)], now=0.0)  # missing timestamp


# ---------------------------------------------------------------- detector


def test_sim_detector_returns_only_in_range_targets():
    detector = SimTargetDetector(detect_range_m=5.0, base_confidence=0.85)
    targets = [(1.0, 0.0), (6.0, 0.0), (0.0, 4.9), (5.0, 0.0)]
    detections = detector.detect(targets, pose=(0.0, 0.0), now=7.0)
    # (6, 0) is out of range; the boundary target at exactly 5.0 m is included.
    assert len(detections) == 3
    for x, y, confidence, timestamp in detections:
        assert timestamp == pytest.approx(7.0)
        assert 0.0 <= confidence <= 1.0
        assert abs(confidence - 0.85) <= 0.05  # small confidence noise
        assert math.hypot(x, y) <= 5.0 + 0.1  # small position noise


def test_sim_detector_is_deterministic():
    detector = SimTargetDetector()
    targets = [(1.0, 2.0), (3.0, -1.0)]
    first = detector.detect(targets, pose=(0.0, 0.0, 1.57), now=1.0)
    second = detector.detect(targets, pose=(0.0, 0.0, 1.57), now=1.0)
    assert first == second


def test_sim_detector_accepts_pose_with_heading():
    detector = SimTargetDetector()
    assert detector.detect([(1.0, 0.0)], (0.0, 0.0), 0.0) == detector.detect(
        [(1.0, 0.0)], (0.0, 0.0, 3.14), 0.0
    )


def test_sim_detector_line_of_sight_filter():
    detector = SimTargetDetector()

    def los_fn(x0, y0, x1, y1):
        return x1 < 2.0  # target at x=3 is occluded

    detections = detector.detect([(1.0, 0.0), (3.0, 0.0)], (0.0, 0.0), 0.0, los_fn=los_fn)
    assert len(detections) == 1


def test_sim_detector_empty_targets():
    assert SimTargetDetector().detect([], (0.0, 0.0), 0.0) == []


def test_detectors_satisfy_detector_protocol():
    assert isinstance(SimTargetDetector(), Detector)
    assert isinstance(YoloDetector(), Detector)


def test_yolo_placeholder_constructs_but_detect_raises():
    detector = YoloDetector(weights_path="best.pt")  # construction must be safe
    with pytest.raises(NotImplementedError):
        detector.detect(None, (0.0, 0.0, 0.0), 0.0)