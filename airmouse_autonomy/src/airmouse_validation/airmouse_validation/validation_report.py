"""Markdown + PNG + JSON validation report generator.

Consumes a :func:`airmouse_validation.replay.run` result dict and writes:

* ``report.md``  - metrics tables plus a roadmap acceptance comparison that
  explicitly labels runtime-only items UNVERIFIED-OFFLINE,
* ``map.png``    - presentation map (occupancy + survivor tracks + room
  markers) via ``airmouse_mapping.map_exporter.save_png``,
* ``result.json`` - the same result without numpy arrays.

Everything here is stdlib + numpy; no rclpy imports.
"""
from __future__ import annotations

import json
import os

import numpy as np

from airmouse_mapping.map_exporter import save_png

from . import validation_metrics

UNVERIFIED = "UNVERIFIED-OFFLINE"
_SKIP = object()
_REPORT_NAME = "report.md"
_MAP_NAME = "map.png"
_RESULT_NAME = "result.json"


def _strip_numpy(value):
    """Recursively drop numpy arrays and convert numpy scalars to Python."""
    if isinstance(value, np.ndarray):
        return _SKIP
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            cleaned = _strip_numpy(item)
            if cleaned is not _SKIP:
                out[str(key)] = cleaned
        return out
    if isinstance(value, (list, tuple)):
        cleaned_items = []
        for item in value:
            cleaned = _strip_numpy(item)
            if cleaned is not _SKIP:
                cleaned_items.append(cleaned)
        return cleaned_items
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _table(header: tuple[str, ...], rows: list[tuple[str, ...]]) -> str:
    lines = ["| " + " | ".join(header) + " |",
             "|" + "|".join("---" for _ in header) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines)


def _pass_fail(measured: bool) -> str:
    return "PASS" if measured else "FAIL"


def _counts(result: dict) -> dict:
    counts = result.get("occupancy")
    if isinstance(counts, dict) and {"tp", "fp", "fn"} <= set(counts):
        return counts
    grid = result.get("grid")
    truth_grid = result.get("truth_grid")
    if grid is None or truth_grid is None:
        return {"tp": 0, "fp": 0, "fn": 0, "precision": 0.0, "recall": 0.0}
    return validation_metrics.occupancy_counts(grid, truth_grid)


def _acceptance_rows(result: dict) -> list[tuple[str, str, str, str]]:
    """Roadmap acceptance table: (criterion, target, measured, status)."""
    counts = _counts(result)
    f1 = float(result.get("map_f1", 0.0))
    collisions = int(result.get("collisions", 0))
    duration = float(result.get("duration_s", 0.0))
    tracked = int(result.get("survivors_tracked", 0))
    truth_survivors = int(result.get("truth_survivors", 0))
    complete = bool(result.get("mission_complete", False))

    return [
        ("Map F1 (OCCUPIED vs truth)", ">= 0.90", f"{f1:.3f}",
         _pass_fail(f1 >= 0.90)),
        ("Localization error", "<= 0.20 m", "truth baseline (0.000 m)",
         UNVERIFIED),
        ("Mission completion", ">= 95% of trials",
         f"{100 if complete else 0}% (1 offline replay)",
         _pass_fail(complete)),
        ("Collisions", "0", str(collisions), _pass_fail(collisions == 0)),
        ("Unique survivors", "<= 6",
         f"{tracked} tracked / {truth_survivors} in truth",
         _pass_fail(tracked <= 6)),
        ("Manual waypoints", "0", "0 (autonomous goal selection)",
         "PASS"),
        ("Mission duration", "< 1800 sim s", f"{duration:.1f} s",
         _pass_fail(duration < 1800.0)),
        ("Sensor availability", "100% nominal", "not measured offline",
         UNVERIFIED),
        ("Obstacle recall", ">= 95%",
         f"map recall proxy {counts['recall'] * 100:.1f}%", UNVERIFIED),
    ]


def _render_markdown(result: dict) -> str:
    counts = _counts(result)
    tracks = result.get("track_records") or []
    confirmed = [t for t in tracks if str(t.get("state", "")).lower() == "confirmed"]
    regions = result.get("regions") or []

    lines = [
        "# AirMouse autonomy replay validation report",
        "",
        "Offline replay of the ROS-independent autonomy cores (occupancy mapping, ",
        "frontier exploration, local planning, survivor tracking) against a ",
        "deterministic synthetic ground-truth world. Ground truth is used only to ",
        "synthesize sensor observations and to score the result; it is never fed ",
        "back into the autonomy pipeline.",
        "",
        "## Mission summary",
        "",
        _table(
            ("Metric", "Value"),
            [
                ("Mission complete", "yes" if result.get("mission_complete") else "no"),
                ("Duration (sim s)", f"{float(result.get('duration_s', 0.0)):.1f}"),
                ("Explored (% of grid observed)",
                 f"{float(result.get('explored_pct', 0.0)):.1f}%"),
                ("Steps", str(result.get("steps", "n/a"))),
                ("Wall-clock runtime (s)",
                 f"{float(result.get('wall_time_s', 0.0)):.2f}"),
            ],
        ),
        "",
        "## Map quality (ground-truth comparison)",
        "",
        _table(
            ("Metric", "Value"),
            [
                ("Map F1 (OCCUPIED positive class)", f"{float(result.get('map_f1', 0.0)):.3f}"),
                ("Precision", f"{counts['precision']:.3f}"),
                ("Recall", f"{counts['recall']:.3f}"),
                ("True positives", str(counts["tp"])),
                ("False positives", str(counts["fp"])),
                ("False negatives", str(counts["fn"])),
                ("Collisions (truth footprint)", str(result.get("collisions", 0))),
            ],
        ),
        "",
        "## Survivors",
        "",
        _table(
            ("Metric", "Value"),
            [
                ("Survivors in truth", str(result.get("truth_survivors", 0))),
                ("Tracks", str(len(tracks))),
                ("Confirmed tracks", str(len(confirmed))),
                ("Truth positions matched", str(result.get("survivor_matches", "n/a"))),
            ],
        ),
    ]

    if tracks:
        lines += [
            "",
            _table(
                ("Track", "x (m)", "y (m)", "State", "Confidence", "Sightings"),
                [
                    (
                        str(track.get("survivor_id", "?")),
                        f"{float(track.get('position_x', 0.0)):.2f}",
                        f"{float(track.get('position_y', 0.0)):.2f}",
                        str(track.get("state", "?")),
                        f"{float(track.get('confidence', 0.0)):.2f}",
                        str(track.get("sightings", "?")),
                    )
                    for track in tracks
                ],
            ),
        ]

    if regions:
        lines += [
            "",
            "## Room coverage",
            "",
            _table(
                ("Region", "Explored", "Status"),
                [
                    (
                        str(region.get("name", region.get("region_id", "?"))),
                        f"{float(region.get('explored_frac', 0.0)) * 100:.0f}%",
                        str(region.get("status", "?")),
                    )
                    for region in regions
                ],
            ),
        ]

    lines += [
        "",
        "## Roadmap acceptance comparison",
        "",
        _table(("Criterion", "Target", "Measured", "Status"),
               _acceptance_rows(result)),
        "",
        f"Items marked {UNVERIFIED} require ROS 2 / Gazebo / PX4 runtime hardware "
        "and are not fabricated offline.",
        "",
        "## UNVERIFIED-OFFLINE",
        "",
        "- Localization error target (<= 0.20 m): the replay localizes from truth "
        "(DEV baseline). VIO / RGB-D SLAM integration is a Goal-4 runtime follow-up.",
        "- Sensor availability (100% nominal): needs the Gazebo RGB-D + IMU bringup.",
        "- Obstacle recall (>= 95%): needs the depth -> obstacle pipeline in "
        "simulation; the map recall above is an offline occupancy proxy only.",
        "- colcon build, PX4 offboard/arming/landing, TF tree and rosbag capture: "
        "Goal-0 checklist on the Ubuntu 22.04 / ROS 2 Humble target machine.",
        "",
        "## Artifacts",
        "",
        f"- `{_MAP_NAME}` - occupancy map with survivor-track and room overlays",
        f"- `{_RESULT_NAME}` - full result (numpy arrays excluded)",
        "",
        "Simulation success is not proof of real-world flight safety.",
        "",
    ]
    return "\n".join(lines)


def generate(result: dict, out_dir: str) -> dict:
    """Write ``report.md``, ``map.png`` and ``result.json`` into ``out_dir``.

    Returns ``{"md": ..., "png": ..., "json": ...}`` absolute-ish paths of the
    created files. Raises ``ValueError`` when the result carries no grid (the
    PNG artifact is mandatory) or the directory cannot be used.
    """
    if not isinstance(result, dict):
        raise ValueError(f"result must be a dict, got {type(result).__name__}")
    grid = result.get("grid")
    if grid is None:
        raise ValueError("result must contain the occupancy 'grid' for the map PNG")
    cell_size = float(result.get("cell_size", 0.1))

    os.makedirs(out_dir, exist_ok=True)
    paths = {
        "md": os.path.join(out_dir, _REPORT_NAME),
        "png": os.path.join(out_dir, _MAP_NAME),
        "json": os.path.join(out_dir, _RESULT_NAME),
    }

    with open(paths["md"], "w", encoding="utf-8") as handle:
        handle.write(_render_markdown(result))

    payload = _strip_numpy(result)
    with open(paths["json"], "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")

    save_png(
        grid,
        list(result.get("track_records") or []),
        list(result.get("regions") or []),
        paths["png"],
        cell_size=cell_size,
        scale=4,
    )
    return paths
