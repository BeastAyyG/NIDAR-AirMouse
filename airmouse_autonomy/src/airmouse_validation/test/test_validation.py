"""Pytest suite for the validation lane (metrics, replay harness, report).

Cores are ROS-independent; this suite imports only the validation + mapping
cores (never rclpy).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

# Standalone bootstrap (the repo-root conftest also adds src/* to sys.path).
_SRC_DIR = Path(__file__).resolve().parents[2]
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

from airmouse_mapping.occupancy_core import FREE, OCCUPIED, UNKNOWN
from airmouse_validation import replay, validation_report
from airmouse_validation.validation_metrics import (
    collisions_from_truth,
    map_f1,
    occupancy_counts,
    survivor_match_count,
)


# ---------------------------------------------------------------- map metrics
def test_map_f1_identical_grids_is_one():
    grid = np.full((10, 10), FREE, dtype=np.int8)
    grid[2, 2:8] = OCCUPIED

    assert map_f1(grid, grid.copy()) == pytest.approx(1.0)


def test_map_f1_degraded_by_false_positives_and_negatives():
    truth = np.full((10, 10), FREE, dtype=np.int8)
    truth[2, 2:8] = OCCUPIED  # 6 occupied cells

    degraded = truth.copy()
    degraded[2, 2:4] = FREE      # 2 false negatives
    degraded[7, 7] = OCCUPIED    # 1 false positive

    score = map_f1(degraded, truth)

    assert 0.0 < score < 1.0
    assert score == pytest.approx(2 * 4 / (2 * 4 + 1 + 2))


def test_map_f1_ignores_unknown_truth_cells():
    truth = np.full((8, 8), UNKNOWN, dtype=np.int8)
    truth[3, 3] = OCCUPIED
    grid = np.full((8, 8), OCCUPIED, dtype=np.int8)  # predictions everywhere

    counts = occupancy_counts(grid, truth)

    assert counts["tp"] == 1
    assert counts["fp"] == 0  # unknown truth cells are never scored
    assert map_f1(grid, truth) == pytest.approx(1.0)


def test_occupancy_counts_precision_and_recall():
    truth = np.zeros((4, 4), dtype=np.int8)
    truth[0, 0:2] = OCCUPIED
    grid = np.zeros((4, 4), dtype=np.int8)
    grid[0, 0] = OCCUPIED
    grid[1, 1] = OCCUPIED

    counts = occupancy_counts(grid, truth)

    assert (counts["tp"], counts["fp"], counts["fn"]) == (1, 1, 1)
    assert counts["precision"] == pytest.approx(0.5)
    assert counts["recall"] == pytest.approx(0.5)


def test_occupancy_counts_rejects_shape_mismatch():
    with pytest.raises(ValueError):
        occupancy_counts(np.zeros((4, 5)), np.zeros((5, 4)))


# ---------------------------------------------------------------- collisions
def test_collisions_detected_when_segment_crosses_wall():
    truth = np.full((20, 20), FREE, dtype=np.int8)
    truth[:, 10] = OCCUPIED  # wall column at x = 1.0 m

    collisions = collisions_from_truth(
        [((0.2, 1.0), (1.8, 1.0))], truth, radius=0.1, cell_size=0.1)

    assert collisions > 0


def test_no_collisions_on_clear_segment():
    truth = np.full((40, 40), FREE, dtype=np.int8)
    truth[:, 35] = OCCUPIED

    collisions = collisions_from_truth(
        [((0.5, 1.0), (2.0, 1.5)), ((2.0, 1.5), (3.0, 2.0))],
        truth, radius=0.1, cell_size=0.1)

    assert collisions == 0


def test_collisions_are_deduplicated_per_cell():
    truth = np.full((20, 20), FREE, dtype=np.int8)
    truth[5, 5] = OCCUPIED

    one_pass = collisions_from_truth(
        [((0.45, 0.55), (0.6, 0.55))], truth, radius=0.2, cell_size=0.1)
    two_passes = collisions_from_truth(
        [((0.45, 0.55), (0.6, 0.55)), ((0.45, 0.55), (0.6, 0.55))],
        truth, radius=0.2, cell_size=0.1)

    assert one_pass == 1
    assert two_passes == 2  # one event per (segment, cell) pair


# ---------------------------------------------------------------- survivors
def test_survivor_match_count_dedups_repeated_tracks():
    truth_positions = [(1.0, 1.0), (5.0, 5.0)]
    repeated = [
        {"position_x": 1.05, "position_y": 1.0},   # same survivor, 3 records
        {"position_x": 1.02, "position_y": 1.03},
        {"position_x": 0.98, "position_y": 0.97},
        {"position_x": 5.1, "position_y": 5.0},
    ]

    assert survivor_match_count(truth_positions, repeated, merge_radius=1.0) == 2
    assert survivor_match_count(truth_positions, repeated[:3], merge_radius=1.0) == 1


def test_survivor_match_count_respects_radius_and_one_to_one():
    truth_positions = [(0.0, 0.0), (0.2, 0.0)]
    tracks = [{"position_x": 0.1, "position_y": 0.0}]

    assert survivor_match_count(truth_positions, tracks, merge_radius=0.5) == 1
    assert survivor_match_count(truth_positions, tracks, merge_radius=0.01) == 0


# ------------------------------------------------------------------- replay
@pytest.fixture(scope="module")
def replay_result():
    return replay.run()


def test_replay_completes_default_mission(replay_result):
    result = replay_result

    assert result["mission_complete"] is True
    assert result["duration_s"] < 1800.0
    assert result["survivors_tracked"] >= 1
    assert result["map_f1"] > 0.5
    assert result["collisions"] == 0


def test_replay_reports_required_keys(replay_result):
    for key in ("mission_complete", "duration_s", "explored_pct", "map_f1",
                "collisions", "survivors_tracked", "truth_survivors",
                "track_records", "grid", "truth_grid", "path_history"):
        assert key in replay_result
    assert replay_result["truth_survivors"] == 3
    assert isinstance(replay_result["grid"], np.ndarray)
    assert isinstance(replay_result["truth_grid"], np.ndarray)
    assert len(replay_result["path_history"]) > 10


def test_replay_is_deterministic(replay_result):
    again = replay.run()

    assert again["duration_s"] == replay_result["duration_s"]
    assert again["map_f1"] == replay_result["map_f1"]
    assert again["collisions"] == replay_result["collisions"]
    assert np.array_equal(again["grid"], replay_result["grid"])


def test_replay_honours_step_cap_and_time_limit():
    result = replay.run({"mission": {"time_limit_s": 1.0, "max_steps": 20}})

    assert result["duration_s"] <= 1.0 + 1e-9
    assert result["steps"] <= 20
    assert result["mission_complete"] is False


# ------------------------------------------------------------------- report
def test_report_writes_markdown_json_and_png(replay_result, tmp_path):
    paths = validation_report.generate(replay_result, str(tmp_path))

    assert set(paths) == {"md", "png", "json"}
    for path in paths.values():
        assert Path(path).is_file()

    markdown = Path(paths["md"]).read_text(encoding="utf-8")
    assert "F1" in markdown
    assert "UNVERIFIED-OFFLINE" in markdown

    payload = json.loads(Path(paths["json"]).read_text(encoding="utf-8"))
    assert payload["mission_complete"] is True
    assert "grid" not in payload  # numpy arrays are excluded
    assert "truth_grid" not in payload
    assert isinstance(payload["track_records"], list)

    assert Path(paths["png"]).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_report_requires_grid():
    with pytest.raises(ValueError):
        validation_report.generate({"map_f1": 1.0}, "unused")
