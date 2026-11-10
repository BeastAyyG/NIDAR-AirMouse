"""Unit tests for the colour-blob survivor detector core (numpy only, no ROS).

Every case is synthetic: a dark RGB scene with solid-colour rectangles at
known pixels and a constant metric depth image, so the expected world point
follows directly from the shared depth_processor camera model.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from airmouse_survivor.color_blob_detector import (
    DEFAULT_COLOR_RANGES,
    TRACKER_RECORD_KEYS,
    detect_survivors,
)

H, W = 120, 160
FX = FY = 100.0
CX, CY = 80.0, 60.0
IDENTITY_POSE = (0.0, 0.0, 0.0)
ZERO_OFFSET = (0.0, 0.0)

DARK = (20, 20, 20)
RED = (240, 60, 60)


def make_scene(depth_m: float = 2.0) -> tuple[np.ndarray, np.ndarray]:
    rgb = np.full((H, W, 3), DARK, dtype=np.uint8)
    depth = np.full((H, W), depth_m, dtype=np.float64)
    return rgb, depth


def paint(rgb: np.ndarray, rows: range, cols: range, color=RED) -> None:
    rgb[rows.start:rows.stop, cols.start:cols.stop] = color


def detect(rgb, depth, pose=IDENTITY_POSE, offset=ZERO_OFFSET, **kwargs):
    return detect_survivors(
        rgb, depth, FX, FY, CX, CY, pose, offset, **kwargs
    )


def test_single_red_rectangle_localises_at_expected_world_point():
    rgb, depth = make_scene()
    # Rows 40..70 (mean 55), cols 60..78 (mean 69) -> 589-pixel blob.
    paint(rgb, range(40, 71), range(60, 79))

    detections = detect(rgb, depth)

    assert len(detections) == 1
    x, y, confidence = detections[0]
    expected_y = (55.0 - CY) * 2.0 / FY
    assert x == pytest.approx(2.0, abs=1e-3)
    assert y == pytest.approx(expected_y, abs=1e-3)  # -0.10 m
    assert y == pytest.approx(-0.10, abs=1e-3)
    assert confidence == pytest.approx(0.99, abs=1e-9)


def test_no_matching_pixels_yields_no_detections():
    rgb, depth = make_scene()
    paint(rgb, range(40, 71), range(60, 79), color=(30, 200, 60))  # green

    assert detect(rgb, depth) == []


def test_blob_below_min_pixels_is_rejected():
    rgb, depth = make_scene()
    paint(rgb, range(40, 45), range(60, 65))  # 25 pixels < 60

    assert detect(rgb, depth) == []


def test_confidence_scales_with_pixel_count():
    rgb, depth = make_scene()
    paint(rgb, range(40, 50), range(60, 70))  # 100 pixels

    detections = detect(rgb, depth, min_pixels=50)

    assert len(detections) == 1
    assert detections[0][2] == pytest.approx(0.6, abs=1e-9)


def test_blob_without_valid_depth_is_skipped():
    rgb, depth = make_scene()
    paint(rgb, range(40, 71), range(60, 79))
    depth[:, :] = 0.0  # all invalid

    assert detect(rgb, depth) == []


def test_depth_outside_configured_range_is_skipped():
    rgb, depth = make_scene(depth_m=9.0)  # beyond depth_max 8.0
    paint(rgb, range(40, 71), range(60, 79))

    assert detect(rgb, depth) == []


def test_two_separated_blobs_yield_two_detections():
    rgb, depth = make_scene()
    paint(rgb, range(40, 71), range(60, 79))
    paint(rgb, range(40, 71), range(100, 120))

    detections = detect(rgb, depth)

    assert len(detections) == 2
    for x, y, _confidence in detections:
        assert x == pytest.approx(2.0, abs=1e-3)
        assert y == pytest.approx(-0.10, abs=1e-3)


def test_pose_rotation_swings_world_point():
    rgb, depth = make_scene()
    paint(rgb, range(40, 71), range(60, 79))

    detections = detect(rgb, depth, pose=(0.0, 0.0, math.pi / 2.0))

    assert len(detections) == 1
    x, y, _confidence = detections[0]
    # Rotate (2.0, -0.1) by +90 deg (clockwise in the x-right / y-down frame).
    assert x == pytest.approx(0.1, abs=1e-3)
    assert y == pytest.approx(2.0, abs=1e-3)


def test_rgba_image_ignores_alpha_channel():
    rgb, depth = make_scene()
    paint(rgb, range(40, 71), range(60, 79))
    rgba = np.dstack([rgb, np.full((H, W), 255, dtype=np.uint8)])

    detections = detect(rgba, depth)

    assert len(detections) == 1


def test_custom_color_ranges_select_green_blob():
    rgb, depth = make_scene()
    green = (30, 220, 60)
    paint(rgb, range(40, 71), range(60, 79), color=green)

    detections = detect(
        rgb, depth, color_ranges=[((0, 180, 0), (80, 255, 100))]
    )

    assert len(detections) == 1


def test_empty_color_ranges_disable_detection():
    rgb, depth = make_scene()
    paint(rgb, range(40, 71), range(60, 79))

    assert detect(rgb, depth, color_ranges=[]) == []


def test_tracker_record_schema_note_matches_track_msg():
    assert TRACKER_RECORD_KEYS == (
        "survivor_id",
        "detected",
        "position_x",
        "position_y",
        "confidence",
        "timestamp",
        "state",
        "sightings",
    )
    assert len(TRACKER_RECORD_KEYS) == 8


def test_default_colour_ranges_match_red_and_orange_jackets():
    assert DEFAULT_COLOR_RANGES[0][0] == (200, 30, 30)
    assert DEFAULT_COLOR_RANGES[0][1] == (255, 110, 110)
    red = (240, 40, 40)
    orange = (255, 120, 40)
    for color in (red, orange):
        hit = any(
            all(lo[i] <= color[i] <= hi[i] for i in range(3))
            for lo, hi in DEFAULT_COLOR_RANGES
        )
        assert hit, f"{color} should match a default survivor colour"


def test_rejects_zero_focal_length():
    rgb, depth = make_scene()
    with pytest.raises(ValueError):
        detect_survivors(rgb, depth, 0.0, FY, CX, CY, IDENTITY_POSE, ZERO_OFFSET)


def test_rejects_depth_shape_mismatch():
    rgb, depth = make_scene()
    with pytest.raises(ValueError):
        detect_survivors(
            rgb, depth[:-1, :], FX, FY, CX, CY, IDENTITY_POSE, ZERO_OFFSET
        )


def test_rejects_malformed_colour_range():
    rgb, depth = make_scene()
    with pytest.raises(ValueError):
        detect(rgb, depth, color_ranges=[((255, 30, 30), (200, 110, 110))])
