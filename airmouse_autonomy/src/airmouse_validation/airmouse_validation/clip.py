"""Clip container for the AirMouse mission-film renderer.

A *clip* is a directory holding ``clip.json`` plus ``.npy`` arrays. It is
produced either by ``scripts/export_bag.py`` (on the ROS machine, from a
rosbag2 ``.db3``) or by :func:`make_synthetic_clip` (tests and demos, no ROS).

The renderer (``video_render.py``) reads clips with numpy only. Arrays are
``.npy`` on purpose: no image codecs, no ROS message dependencies, and float
depth/map data stays lossless.

Manifest (``clip.json``)::

    {
      "format": "airmouse-clip/1",
      "fps": 10.0,
      "width": 1280, "height": 800,
      "camera": {"fx": 554.3, "fy": 554.3, "cx": 320.0, "cy": 240.0,
                 "depth_max_m": 8.0},
      "map_cell_m": 0.10,
      "map_origin": [0.0, 0.0],
      "meta": {"world": "full_mission.sdf", "seed": 7},
      "frames": [
        {"t": 0.0,
         "rgb": "frames/000000_rgb.npy",      # (H,W,3) uint8, optional
         "depth": "frames/000000_depth.npy",  # (H,W) float32 m, optional
         "map": "maps/000000_map.npy",        # (rows,cols) int8, optional
         "pose": [x, y, yaw],                 # map frame, metres/rad
         "imu": [[ax,ay,az,gx,gy,gz], ...],   # m/s^2, rad/s
         "path": [[x, y], ...],               # planned path, map frame
         "goal": [x, y],
         "tracks": [{"id": 3, "x": 3.2, "y": 4.1,
                     "confidence": 0.94, "state": "confirmed"}],
         "coverage_pct": 12.3, "survivors": 1, "state": "EXPLORE"}
      ]
    }

Every frame field except ``t`` is optional; the renderer draws a labelled
placeholder for missing streams. Unknown cells are -1, free 0, occupied 100
(same convention as the workspace).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

FORMAT = "airmouse-clip/1"
STREAM_KEYS = ("rgb", "depth", "map")


def validate_manifest(manifest: dict) -> None:
    """Raise ValueError if the manifest violates the clip contract."""
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be a dict")
    if manifest.get("format") != FORMAT:
        raise ValueError(f"unsupported clip format {manifest.get('format')!r}")
    for key in ("fps", "width", "height", "frames"):
        if key not in manifest:
            raise ValueError(f"manifest missing required key {key!r}")
    if float(manifest["fps"]) <= 0.0:
        raise ValueError("fps must be > 0")
    if int(manifest["width"]) <= 0 or int(manifest["height"]) <= 0:
        raise ValueError("width/height must be > 0")
    frames = manifest["frames"]
    if not isinstance(frames, list):
        raise ValueError("frames must be a list")
    if not frames:
        raise ValueError("frames must be non-empty")
    last_t = float("-inf")
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ValueError(f"frame {index} must be a dict")
        t = float(frame.get("t", index / float(manifest["fps"])))
        if t < last_t:
            raise ValueError(f"frame {index} timestamp goes backwards")
        last_t = t


def save_clip(directory: str | Path, manifest: dict) -> Path:
    """Write ``clip.json`` (arrays are expected to be saved by the caller)."""
    validate_manifest(manifest)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "clip.json"
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=1)
    return path


def load_clip(directory: str | Path) -> dict:
    """Load a clip: manifest plus arrays resolved relative to the directory."""
    directory = Path(directory)
    with open(directory / "clip.json", encoding="utf-8") as handle:
        manifest = json.load(handle)
    validate_manifest(manifest)
    clip = dict(manifest)
    frames = []
    for frame in manifest["frames"]:
        loaded = dict(frame)
        for key in STREAM_KEYS:
            rel = loaded.get(key)
            loaded[key] = np.load(directory / rel) if rel else None
        frames.append(loaded)
    clip["frames"] = frames
    clip["directory"] = str(directory)
    return clip


# --------------------------------------------------------------------------
# Synthetic clip (development fixture: same contract, no ROS, no bag)
# --------------------------------------------------------------------------
def make_synthetic_clip(directory: str | Path, frames: int = 24,
                        width: int = 320, height: int = 240,
                        fps: float = 10.0, seed: int = 0) -> Path:
    """Write a deterministic fake mission clip (moving drone, wall, survivor).

    Deliberately small by default so tests stay fast; pass ``width/height``
    for a full-size demo. Returns the clip directory.
    """
    rng = np.random.default_rng(seed)
    directory = Path(directory)
    (directory / "frames").mkdir(parents=True, exist_ok=True)
    (directory / "maps").mkdir(parents=True, exist_ok=True)
    fx = fy = 0.8 * width
    cx, cy = width / 2.0, height / 2.0
    grid_rows, grid_cols = 40, 40
    manifest = {
        "format": FORMAT,
        "fps": float(fps),
        "width": int(width),
        "height": int(height),
        "camera": {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                   "depth_max_m": 8.0},
        "map_cell_m": 0.25,
        "map_origin": [0.0, 0.0],
        "meta": {"world": "synthetic", "seed": seed},
        "frames": [],
    }
    u = np.arange(width, dtype=np.float32)[None, :]
    v = np.arange(height, dtype=np.float32)[:, None]
    for index in range(frames):
        t = index / float(fps)
        phase = 2.0 * math.pi * index / max(frames - 1, 1)
        px = 5.0 + 1.8 * math.cos(phase)
        py = 5.0 + 1.8 * math.sin(phase)
        yaw = phase + math.pi / 2.0

        depth = (2.0 + 0.6 * np.sin(u / 23.0 + t)
                 + 0.4 * (v / height)).astype(np.float32)
        depth[:, :width // 12] = 0.0  # invalid border, like real depth
        depth[0, 0] = np.nan

        rgb = np.zeros((height, width, 3), dtype=np.uint8)
        rgb[:, :, 0] = (20 + 30 * (v / height)).astype(np.uint8)
        rgb[:, :, 1] = (28 + 20 * (u / width)).astype(np.uint8)
        blob_u = int(cx + 0.25 * width * math.sin(phase * 2.0))
        blob_v = int(cy - 0.05 * height)
        half = max(3, width // 60)
        rgb[blob_v - half:blob_v + half, blob_u - half:blob_u + half] = (224, 70, 56)

        np.save(directory / f"frames/{index:06d}_rgb.npy", rgb)
        np.save(directory / f"frames/{index:06d}_depth.npy", depth)

        imu = [[float(0.02 * math.sin(t * 5.0 + k * 0.3) + rng.normal(0, 0.01)),
                float(0.01 * math.cos(t * 4.0 + k * 0.2)),
                float(9.81 + rng.normal(0, 0.02)),
                float(0.05 * math.sin(t * 3.0 + k * 0.1)),
                float(0.03 * math.cos(t * 2.5 + k * 0.2)),
                float(0.02 * math.sin(t * 6.0 + k * 0.15))]
               for k in range(20)]
        frame = {
            "t": t,
            "rgb": f"frames/{index:06d}_rgb.npy",
            "depth": f"frames/{index:06d}_depth.npy",
            "pose": [px, py, yaw],
            "imu": imu,
            "path": [[5.0 + 1.8 * math.cos(a), 5.0 + 1.8 * math.sin(a)]
                     for a in np.linspace(0.0, phase, 6)],
            "goal": [5.0 + 1.8 * math.cos(phase + 0.6),
                     5.0 + 1.8 * math.sin(phase + 0.6)],
            "tracks": [
                {"id": 1, "x": 5.0 + 1.2 * math.cos(phase + 1.0),
                 "y": 5.0 + 1.2 * math.sin(phase + 1.0),
                 "confidence": 0.95, "state": "confirmed"},
                {"id": 2, "x": 5.0 + 1.5 * math.cos(phase + 2.4),
                 "y": 5.0 + 1.5 * math.sin(phase + 2.4),
                 "confidence": 0.42, "state": "uncertain"},
            ],
            "coverage_pct": round(8.0 + 70.0 * index / max(frames - 1, 1), 1),
            "survivors": 1,
            "state": "RETURN" if index >= frames - 3 else "EXPLORE",
        }
        if index % 4 == 0:
            grid = np.full((grid_rows, grid_cols), -1, dtype=np.int8)
            radius = 3.0 + 8.0 * index / max(frames - 1, 1)
            rr, cc = np.mgrid[0:grid_rows, 0:grid_cols]
            inside = ((rr - 20) ** 2 + (cc - 20) ** 2) <= radius ** 2
            grid[inside] = 0
            grid[0, :] = 100
            grid[-1, :] = 100
            grid[:, 0] = 100
            grid[:, -1] = 100
            grid[20, 26:33] = 100
            np.save(directory / f"maps/{index:06d}_map.npy", grid)
            frame["map"] = f"maps/{index:06d}_map.npy"
        manifest["frames"].append(frame)
    save_clip(directory, manifest)
    return directory