#!/usr/bin/env python3
"""Validate the committed AirMouse maze geometry and nominal route.

This is a lightweight, dependency-free check for CI and review. It is not a
replacement for Gazebo physics, planner tests, or a physical arena survey.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


EXPECTED_INTERNAL_WALLS = {
    "wall_a", "wall_b", "wall_c", "wall_d", "wall_e", "wall_f", "wall_g",
    "wall_h1", "wall_h2", "wall_h3", "wall_h4", "wall_v1", "wall_v2",
}
EXPECTED_OUTER_WALLS = {
    "wall_outer_west", "wall_outer_east", "wall_outer_south", "wall_outer_north",
}
EXPECTED_SURVIVORS = {f"survivor_{index:02d}" for index in range(1, 7)}

# Gazebo ENU route corresponding to the mission_waypoints in config/airmouse.yaml.
ROUTE = ((0.0, 0.0), (0.0, -6.0), (-7.0, -6.0), (0.0, -6.0), (0.0, -12.0), (0.0, 0.0))


def floats(text: str | None) -> tuple[float, ...]:
    return tuple(float(value) for value in (text or "").split())


def model_pose(model: ET.Element) -> tuple[float, float, float]:
    pose = floats(model.findtext("pose"))
    if len(pose) < 3:
        raise ValueError(f"model {model.attrib.get('name')} has no xyz pose")
    return pose[:3]


def collision_box(model: ET.Element) -> tuple[float, float, float]:
    size = model.find("./link/collision/geometry/box/size")
    if size is None:
        raise ValueError(f"model {model.attrib.get('name')} has no collision box")
    values = floats(size.text)
    if len(values) != 3:
        raise ValueError(f"model {model.attrib.get('name')} has an invalid box size")
    return values


def rectangle(model: ET.Element) -> tuple[float, float, float, float]:
    x, y, _ = model_pose(model)
    sx, sy, _ = collision_box(model)
    return (x - sx / 2.0, x + sx / 2.0, y - sy / 2.0, y + sy / 2.0)


def segment_intersects_rect(
    start: tuple[float, float], end: tuple[float, float], rect: tuple[float, float, float, float]
) -> bool:
    min_x, max_x, min_y, max_y = rect
    seg_min_x, seg_max_x = sorted((start[0], end[0]))
    seg_min_y, seg_max_y = sorted((start[1], end[1]))
    return seg_max_x >= min_x and seg_min_x <= max_x and seg_max_y >= min_y and seg_min_y <= max_y


def segment_rect_distance(
    start: tuple[float, float], end: tuple[float, float], rect: tuple[float, float, float, float]
) -> float:
    if segment_intersects_rect(start, end, rect):
        return 0.0
    min_x, max_x, min_y, max_y = rect
    seg_min_x, seg_max_x = sorted((start[0], end[0]))
    seg_min_y, seg_max_y = sorted((start[1], end[1]))
    dx = max(min_x - seg_max_x, seg_min_x - max_x, 0.0)
    dy = max(min_y - seg_max_y, seg_min_y - max_y, 0.0)
    return math.hypot(dx, dy)


def validate(maze_path: Path, envelope_radius: float) -> int:
    root = ET.parse(maze_path).getroot()
    world = root.find("world")
    if world is None or world.attrib.get("name") != "maze":
        raise ValueError("expected an SDF world named 'maze'")

    models = {model.attrib["name"]: model for model in world.findall("model")}
    wall_names = {name for name in models if name.startswith("wall_")}
    survivor_names = {name for name in models if name.startswith("survivor_")}
    expected_walls = EXPECTED_INTERNAL_WALLS | EXPECTED_OUTER_WALLS
    if wall_names != expected_walls:
        raise ValueError(f"wall set mismatch: expected {sorted(expected_walls)}, got {sorted(wall_names)}")
    if survivor_names != EXPECTED_SURVIVORS:
        raise ValueError(f"survivor set mismatch: expected {sorted(EXPECTED_SURVIVORS)}, got {sorted(survivor_names)}")

    wall_rects = {name: rectangle(models[name]) for name in sorted(wall_names)}
    inflated_rects = {
        name: (rect[0] - envelope_radius, rect[1] + envelope_radius,
               rect[2] - envelope_radius, rect[3] + envelope_radius)
        for name, rect in wall_rects.items()
    }
    collisions = []
    minimum_gap = math.inf
    nearest_wall = ""
    for start, end in zip(ROUTE, ROUTE[1:]):
        for name, rect in wall_rects.items():
            gap = segment_rect_distance(start, end, rect)
            if gap < minimum_gap:
                minimum_gap = gap
                nearest_wall = name
            if segment_intersects_rect(start, end, inflated_rects[name]):
                collisions.append((start, end, name))

    if collisions:
        details = "; ".join(f"{start}->{end} intersects {name}" for start, end, name in collisions)
        raise ValueError(f"nominal route collides after {envelope_radius:.2f} m inflation: {details}")

    print(f"maze: PASS ({len(wall_names)} walls, {len(survivor_names)} survivor markers)")
    print(f"route: PASS ({len(ROUTE)} ENU points, envelope radius {envelope_radius:.2f} m)")
    print(f"minimum static wall gap: {minimum_gap:.2f} m near {nearest_wall}")
    print(f"estimated post-envelope gap: {minimum_gap - envelope_radius:.2f} m")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--maze", type=Path, default=Path(__file__).resolve().parents[1] / "PX4-gazebo-models" / "worlds" / "maze.sdf")
    parser.add_argument("--envelope-radius", type=float, default=0.32)
    args = parser.parse_args()
    try:
        return validate(args.maze, args.envelope_radius)
    except (ET.ParseError, OSError, ValueError) as error:
        print(f"maze: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
