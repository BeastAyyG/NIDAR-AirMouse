"""Pytest suite for the localization evaluator + truth-pose helpers.

Cores are ROS-independent; this suite imports only ``localization_eval`` and
``truth_baseline`` (never rclpy).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

# Standalone bootstrap (the repo-root conftest also adds src/* to sys.path).
_PKG_DIR = str(Path(__file__).resolve().parents[1])
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from airmouse_localization.localization_eval import compute_errors
from airmouse_localization.truth_baseline import format_truth_pose, validate_truth


# ---------------------------------------------------------------- compute_errors
def test_constant_offset_gives_unit_rmse_and_max():
    """A constant 5 m offset must report rmse == max == 5.0."""
    truth = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (2.0, 2.0, 0.0)]
    est = [(t, x + 5.0, y) for t, x, y in truth]

    errors = compute_errors(est, truth)

    assert errors["rmse"] == pytest.approx(5.0)
    assert errors["max"] == pytest.approx(5.0)
    assert errors["loop_closure_error"] == pytest.approx(5.0)


def test_rmse_is_rms_not_mean_of_distances():
    """Distances 3 and 4 must give rmse 3.535..., max 4, not mean 3.5."""
    truth = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    est = [(0.0, 3.0, 0.0), (1.0, 0.0, 4.0)]

    errors = compute_errors(est, truth)

    assert errors["rmse"] == pytest.approx(math.sqrt((9.0 + 16.0) / 2.0))
    assert errors["max"] == pytest.approx(4.0)
    assert errors["loop_closure_error"] == pytest.approx(4.0)  # last truth sample


def test_est_interpolated_onto_truth_timestamps():
    """A low-rate estimate is interpolated, so a perfect line reports zero."""
    truth = [(0.0, 0.0, 0.0), (1.0, 5.0, 0.0), (2.0, 10.0, 0.0)]
    est = [(0.0, 0.0, 0.0), (2.0, 10.0, 0.0)]  # every 2 s

    errors = compute_errors(est, truth)

    assert errors["rmse"] == pytest.approx(0.0, abs=1e-12)
    assert errors["max"] == pytest.approx(0.0, abs=1e-12)
    assert errors["loop_closure_error"] == pytest.approx(0.0, abs=1e-12)


def test_explicit_revisit_times_average_position_error():
    """revisit_times overrides the last-sample proxy with a mean error."""
    truth = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
    est = [(0.0, 0.0, 0.0), (10.0, 2.0, 0.0)]  # linear drift 0 -> 2 m

    errors = compute_errors(est, truth, revisit_times=[5.0, 10.0])

    assert errors["loop_closure_error"] == pytest.approx(1.5)  # mean(1.0, 2.0)
    assert errors["max"] == pytest.approx(2.0)


def test_rejects_too_few_samples():
    with pytest.raises(ValueError):
        compute_errors([(0.0, 0.0, 0.0)], [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])


def test_rejects_non_monotonic_timestamps():
    with pytest.raises(ValueError):
        compute_errors([(0.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
                       [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])


def test_rejects_malformed_samples():
    with pytest.raises(ValueError):
        compute_errors([(0.0, 0.0), (1.0, 0.0)], [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])


# ---------------------------------------------------------------- truth_baseline
def test_format_truth_pose_round_trip():
    pose = format_truth_pose(1.5, -2.25, 1.2, 0.5)

    assert pose == {"x": 1.5, "y": -2.25, "z": 1.2, "yaw": 0.5, "valid": True}
    assert validate_truth(pose) is True


def test_format_truth_pose_wraps_yaw():
    pose = format_truth_pose(0.0, 0.0, 0.0, 3.0 * math.pi)

    assert pose["yaw"] == pytest.approx(math.pi)
    assert pose["valid"] is True


def test_format_truth_pose_flags_non_finite():
    pose = format_truth_pose(float("nan"), 0.0, 0.0, 0.0)

    assert pose["valid"] is False
    assert validate_truth(pose) is False


def test_validate_truth_rejects_missing_and_bad_values():
    assert validate_truth({"x": 0.0, "y": 0.0, "z": 0.0}) is False
    assert validate_truth({"x": "a", "y": 0.0, "z": 0.0, "yaw": 0.0}) is False
    assert validate_truth({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0,
                           "valid": False}) is False
    assert validate_truth("not a dict") is False
