"""Tests for the mission-film clip contract and renderer (no ROS required)."""
import json
import shutil

import numpy as np
import pytest

from airmouse_validation.clip import (
    FORMAT,
    load_clip,
    make_synthetic_clip,
    save_clip,
    validate_manifest,
)
from airmouse_validation.video_render import (
    MissionFilm,
    RingAccumulator,
    colorize_depth,
    render_clip,
)


def test_validate_manifest_rejects_bad_input():
    with pytest.raises(ValueError):
        validate_manifest(None)
    with pytest.raises(ValueError):
        validate_manifest({"format": "wrong"})
    with pytest.raises(ValueError):
        validate_manifest({"format": FORMAT, "fps": 10, "width": 10,
                           "height": 10, "frames": []})
    a = {"format": FORMAT, "fps": 10, "width": 10, "height": 10,
         "frames": [{"t": 1.0}, {"t": 0.5}]}
    with pytest.raises(ValueError):
        validate_manifest(a)


def test_synthetic_clip_roundtrip(tmp_path):
    directory = make_synthetic_clip(tmp_path / "clip", frames=6,
                                    width=640, height=480)
    assert (directory / "clip.json").is_file()
    clip = load_clip(directory)
    assert len(clip["frames"]) == 6
    frame = clip["frames"][0]
    assert frame["rgb"].shape == (480, 640, 3)
    assert frame["rgb"].dtype == np.uint8
    assert frame["depth"].shape == (480, 640)
    assert np.isnan(frame["depth"]).any()  # invalid sample survives the roundtrip
    assert frame["map"].shape == (40, 40)  # every 4th frame carries the grid
    assert clip["frames"][1]["map"] is None


def test_save_clip_writes_manifest(tmp_path):
    manifest = {"format": FORMAT, "fps": 10.0, "width": 640, "height": 480,
                "frames": [{"t": 0.0}]}
    path = save_clip(tmp_path, manifest)
    assert json.loads(path.read_text())["format"] == FORMAT


def test_ring_accumulator_covers_more_headings_as_it_yaws():
    camera = {"fx": 320.0, "cx": 320.0, "cy": 240.0}
    depth = np.full((480, 640), 3.0, dtype=np.float32)
    ring = RingAccumulator(bins=120, max_range_m=8.0)
    ring.update([1.0, 1.0, 0.0], depth, camera)
    first = np.isfinite(ring.snapshot()).sum()
    ring.update([1.0, 1.0, np.pi / 2], depth, camera)
    second = np.isfinite(ring.snapshot()).sum()
    assert second > first
    assert np.nanmin(np.where(np.isfinite(ring.snapshot()),
                              ring.snapshot(), np.nan)) == pytest.approx(3.0)


def test_colorize_depth_marks_invalid_black():
    depth = np.array([[0.0, float("nan"), 1.0]], dtype=np.float32)
    rgb = colorize_depth(depth, vmax=8.0)
    assert tuple(rgb[0, 0]) == (0, 0, 0)
    assert tuple(rgb[0, 1]) == (0, 0, 0)
    assert tuple(rgb[0, 2]) != (0, 0, 0)


def test_frame_renders_with_only_scalars():
    clip = {
        "format": FORMAT, "fps": 10.0, "width": 640, "height": 480,
        "camera": {"fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0,
                   "depth_max_m": 8.0},
        "map_cell_m": 0.1, "map_origin": [0.0, 0.0],
        "frames": [{"t": 0.0, "pose": [1.0, 1.0, 0.0], "state": "EXPLORE",
                    "coverage_pct": 5.0, "survivors": 0}],
    }
    film = MissionFilm(clip)
    surface = film.render(0)
    assert surface.get_size() == (640, 480)


@pytest.mark.skipif(shutil.which("ffmpeg") is None,
                    reason="ffmpeg not on PATH")
def test_render_synthetic_clip_to_mp4(tmp_path):
    directory = make_synthetic_clip(tmp_path / "clip", frames=6,
                                    width=640, height=480)
    clip = load_clip(directory)
    out = tmp_path / "film.mp4"
    count = render_clip(clip, str(out))
    assert count == 6
    assert out.is_file() and out.stat().st_size > 10_000