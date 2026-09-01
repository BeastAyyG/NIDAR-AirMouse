from math import inf, nan

from nidar_airmouse_sim.safety import assess_scan


def check(ranges):
    return assess_scan(
        ranges=ranges,
        range_min=0.05,
        range_max=30.0,
        stop_distance=0.55,
        emergency_distance=0.40,
        minimum_valid_beams=3,
    )


def test_clear_scan_is_ready_and_safe():
    result = check([1.0, 2.0, 3.0, 4.0])
    assert result.ready is True
    assert result.safe is True
    assert result.emergency is False
    assert result.reason == "clear"


def test_stop_distance_blocks_without_emergency():
    result = check([0.60, 0.54, 2.0, 3.0])
    assert result.ready is True
    assert result.safe is False
    assert result.emergency is False
    assert result.reason == "obstacle_stop_distance"


def test_emergency_distance_is_immediate_abort_condition():
    result = check([0.8, 0.39, 2.0, 3.0])
    assert result.ready is True
    assert result.safe is False
    assert result.emergency is True
    assert result.reason == "obstacle_emergency_distance"


def test_invalid_ranges_do_not_make_a_scan_ready():
    result = check([nan, inf, 0.01, 31.0, 0.2])
    assert result.ready is False
    assert result.safe is False
    assert result.valid_beams == 1
    assert result.reason == "insufficient_valid_ranges"
