"""Deterministic offline replay harness for the AirMouse autonomy stack.

Drives the REAL ROS-independent cores (occupancy mapper, frontier explorer,
local planner, survivor tracker) over a synthetic ground-truth world, so the
full perceive -> map -> explore -> navigate -> track -> report loop can be
validated on a machine without ROS, Gazebo or PX4. Ground truth is used ONLY
to synthesize sensor observations and to score the result (``validation_
metrics``); nothing derived from it is handed to the autonomy cores.

World
-----
A deterministic 15 m x 15 m floor plan at 0.10 m/cell: four rooms around a
cross-shaped corridor, thin interior walls with 1.0 m doorways, and three
survivors. Cells outside the modelled building stay UNKNOWN, so the map F1
metric scores exactly the modelled area.

Drone model
-----------
Truth 2D pose integrated toward the local planner's velocity commands, with a
footprint check against truth obstacles (axis-separated wall sliding, mirroring
the NIDAR SAR sim vehicle). A 120-ray lidar sweep is cast on the truth grid
(substepped march, Gaussian range noise) and integrated into the mapper; a
5 m line-of-sight survivor sensor emits tracker detections.

Result
------
Dict with ``mission_complete``, ``duration_s``, ``explored_pct``, ``map_f1``,
``collisions``, ``survivors_tracked``, ``truth_survivors``, ``track_records``,
``grid``, ``truth_grid``, ``path_history`` (plus a few report extras such as
``regions``, ``occupancy``, ``cell_size`` and ``wall_time_s``). numpy arrays
are returned as-is; ``validation_report.generate`` strips them for JSON.
"""
from __future__ import annotations

import copy
import math
import time

import numpy as np

from airmouse_exploration.frontier_explorer import FrontierExplorer
from airmouse_mapping.grid_manager import grid_shape, mapper_from_grid
from airmouse_mapping.occupancy_core import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    OccupancyMapper,
    supercover_line,
)
from airmouse_navigation.collision_checker import segment_clear
from airmouse_navigation.local_planner import LocalPlanner
from airmouse_survivor.survivor_tracker import SurvivorTracker

from . import validation_metrics

# --------------------------------------------------------------------------
# Deterministic default world + mission configuration.
# --------------------------------------------------------------------------
DEFAULT_CONFIG: dict = {
    "seed": 7,
    "world": {
        "width_m": 15.0,
        "height_m": 15.0,
        "cell_size": 0.10,
        "door_width_m": 1.0,
        "survivors": [(2.5, 3.0), (13.0, 3.0), (12.0, 12.0)],
        "entry": (0.6, 7.5),
    },
    "mapping": {
        "max_range_m": 6.0,
        "clearance_m": 0.20,  # planning margin > footprint + pop tolerance
    },
    "sensors": {
        "lidar_rays": 120,
        "lidar_range_m": 6.0,
        "lidar_noise_m": 0.02,
        "detect_range_m": 5.0,
        "detect_confidence": 0.85,
        "detect_confidence_std": 0.05,
        "position_noise_m": 0.25,
        "scan_interval_s": 0.50,
        "detect_interval_s": 1.00,
    },
    "drone": {
        "speed_mps": 2.0,
        "footprint_radius_m": 0.10,
        "waypoint_pop_tolerance_m": 0.05,
        "stuck_watchdog_s": 3.0,
    },
    "navigation": {
        # nav2-style costmap inflation (cells) for global path planning, and
        # the extra margin on the per-step known-map segment guard.
        "inflation_cells": 2,
        "segment_guard_margin_m": 0.02,
    },
    "tracker": {
        "merge_radius_m": 1.2,
        "confirm_confidence": 0.85,
        "confirm_sightings": 3,
        "decay_per_sec": 0.02,
        "drop_confidence": 0.12,
    },
    "planner": {
        "replan_interval_s": 2.0,
        "goal_reach_radius_m": 0.4,
        "investigate_after_s": 4.0,
        "max_investigate_attempts": 2,
        "lidar_range_m": 6.0,
    },
    "mission": {
        "dt": 0.05,
        "time_limit_s": 1800.0,
        "max_steps": 24000,
        "path_sample_steps": 1,  # dense trajectory: no chord artifacts in scoring
        "survivor_match_radius_m": 1.2,
    },
}

# Layout constants in 15 m reference units (scaled with the world extent).
# Walls are one cell thick (0.10 m at the roadmap resolution) so every
# OCCUPIED cell faces free space and is therefore observable by the lidar —
# a thick shell would leave permanently unobservable cells that only depress
# the map F1 score for no algorithmic reason.
_SHELL_THICKNESS_M = 0.1
_CORRIDOR_HALF_WIDTH_M = 0.4
_WALL_THICKNESS_M = 0.1
_ROOM_INSET_M = 0.1


# --------------------------------------------------------------------------
# Config handling
# --------------------------------------------------------------------------
def _merge_config(base: dict, override: dict | None) -> dict:
    """Recursively merge ``override`` into a deep copy of ``base``."""
    merged = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge_config(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _positive(value, name: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be finite and > 0, got {value!r}")
    return number


# --------------------------------------------------------------------------
# Synthetic ground-truth world
# --------------------------------------------------------------------------
def _fill(truth: np.ndarray, cs: float, rect, value: int) -> None:
    """Rasterize a world-meter rect (x0, y0, x1, y1) into ``truth``."""
    x0, y0, x1, y1 = rect
    rows, cols = truth.shape
    c0 = max(0, int(round(x0 / cs)))
    c1 = min(cols, int(round(x1 / cs)))
    r0 = max(0, int(round(y0 / cs)))
    r1 = min(rows, int(round(y1 / cs)))
    if c1 > c0 and r1 > r0:
        truth[r0:r1, c0:c1] = value


def build_truth_world(world_cfg: dict) -> dict:
    """Build the deterministic synthetic floor plan.

    Returns a dict with ``truth`` (int8 grid), ``cell_size``, ``width_m``,
    ``height_m``, ``entry`` (x, y, yaw), ``survivors`` (list of (x, y)),
    ``rooms`` (region dicts with world center and inclusive cell bounds) and
    ``is_free``/``line_of_sight`` helpers used by the drone model.
    """
    width = _positive(world_cfg["width_m"], "width_m")
    height = _positive(world_cfg["height_m"], "height_m")
    cs = _positive(world_cfg["cell_size"], "cell_size")
    door_w = _positive(world_cfg["door_width_m"], "door_width_m")
    rows, cols = grid_shape(width, height, cs)

    truth = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    scale = min(width, height) / 15.0
    shell = _SHELL_THICKNESS_M * scale
    half_corr = _CORRIDOR_HALF_WIDTH_M * scale
    wall = _WALL_THICKNESS_M * scale
    inset = _ROOM_INSET_M * scale
    cx, cy = width / 2.0, height / 2.0

    room_rects = {
        "NW": (inset, inset, cx - half_corr - wall, cy - half_corr - wall),
        "NE": (cx + half_corr + wall, inset, width - inset, cy - half_corr - wall),
        "SW": (inset, cy + half_corr + wall, cx - half_corr - wall, height - inset),
        "SE": (cx + half_corr + wall, cy + half_corr + wall, width - inset,
               height - inset),
    }
    corridor_rects = [
        (cx - half_corr, inset, cx + half_corr, height - inset),
        (inset, cy - half_corr, width - inset, cy + half_corr),
    ]

    # Rooms + corridors are open space.
    for rect in room_rects.values():
        _fill(truth, cs, rect, FREE)
    for rect in corridor_rects:
        _fill(truth, cs, rect, FREE)

    # Shell + interior wall bands (one cell thick, fully observable from inside).
    _fill(truth, cs, (0.0, 0.0, width, shell), OCCUPIED)
    _fill(truth, cs, (0.0, height - shell, width, height), OCCUPIED)
    _fill(truth, cs, (0.0, 0.0, shell, height), OCCUPIED)
    _fill(truth, cs, (width - shell, 0.0, width, height), OCCUPIED)
    left_wall = (cx - half_corr - wall, inset, cx - half_corr, height - inset)
    right_wall = (cx + half_corr, inset, cx + half_corr + wall, height - inset)
    top_wall = (inset, cy - half_corr - wall, width - inset, cy - half_corr)
    bottom_wall = (inset, cy + half_corr, width - inset, cy + half_corr + wall)
    for wall_rect in (left_wall, right_wall, top_wall, bottom_wall):
        _fill(truth, cs, wall_rect, OCCUPIED)

    # Doorways through the four wall bands (two per room).
    left_x = (cx - half_corr - wall, cx - half_corr)
    right_x = (cx + half_corr, cx + half_corr + wall)
    top_y = (cy - half_corr - wall, cy - half_corr)
    bottom_y = (cy + half_corr, cy + half_corr + wall)
    doors = [
        (left_x[0], 2.5, left_x[1], 2.5 + door_w),        # NW room
        (left_x[0], 11.0, left_x[1], 11.0 + door_w),      # SW room
        (right_x[0], 2.5, right_x[1], 2.5 + door_w),      # NE room
        (right_x[0], 11.0, right_x[1], 11.0 + door_w),    # SE room
        (2.5, top_y[0], 2.5 + door_w, top_y[1]),          # NW room
        (2.5, bottom_y[0], 2.5 + door_w, bottom_y[1]),    # SW room
        (11.5, top_y[0], 11.5 + door_w, top_y[1]),        # NE room
        (11.5, bottom_y[0], 11.5 + door_w, bottom_y[1]),  # SE room
    ]
    for rect in doors:
        _fill(truth, cs, rect, FREE)

    entry_x, entry_y = (float(v) for v in world_cfg["entry"])
    survivors = [(float(x), float(y)) for x, y in world_cfg["survivors"]]

    rooms = []
    for index, (name, rect) in enumerate(room_rects.items()):
        x0, y0, x1, y1 = rect
        rooms.append({
            "region_id": index,
            "name": name,
            "center": ((x0 + x1) / 2.0, (y0 + y1) / 2.0),
            "bounds": (int(round(x0 / cs)), int(round(y0 / cs)),
                       int(round(x1 / cs)) - 1, int(round(y1 / cs)) - 1),
        })

    return {
        "truth": truth,
        "cell_size": cs,
        "width_m": width,
        "height_m": height,
        "entry": (entry_x, entry_y, 0.0),
        "survivors": survivors,
        "rooms": rooms,
    }


# --------------------------------------------------------------------------
# Drone sensors / motion on the truth grid
# --------------------------------------------------------------------------
def _is_free(truth: np.ndarray, cs: float, x: float, y: float) -> bool:
    rows, cols = truth.shape
    if not (0.0 <= x < cols * cs and 0.0 <= y < rows * cs):
        return False
    return bool(truth[int(y / cs), int(x / cs)] == FREE)


def _line_of_sight(truth: np.ndarray, cs: float, x0: float, y0: float,
                   x1: float, y1: float) -> bool:
    """True when no non-FREE truth cell lies strictly between the endpoints."""
    rows, cols = truth.shape
    if not (0.0 <= x0 < cols * cs and 0.0 <= y0 < rows * cs):
        return False
    if not (0.0 <= x1 < cols * cs and 0.0 <= y1 < rows * cs):
        return False
    c0, r0 = int(x0 / cs), int(y0 / cs)
    c1, r1 = int(x1 / cs), int(y1 / cs)
    cells = supercover_line(r0, c0, r1, c1)
    for r, c in cells[1:-1]:
        if not (0 <= r < rows and 0 <= c < cols) or truth[r, c] != FREE:
            return False
    return True


def _scan(truth: np.ndarray, cs: float, x: float, y: float,
          sensors: dict, rng: np.random.Generator) -> list:
    """120-ray lidar sweep on the truth grid, substepped with range noise."""
    n_rays = int(sensors["lidar_rays"])
    max_range = float(sensors["lidar_range_m"])
    noise = float(sensors["lidar_noise_m"])
    angles = np.arange(n_rays, dtype=np.float64) * (2.0 * math.pi / n_rays)
    march_step = cs * 0.5
    n_steps = max(1, int(math.ceil(max_range / march_step)))
    distances_along = np.minimum(np.arange(1, n_steps + 1) * march_step, max_range)

    xs = x + np.cos(angles)[:, None] * distances_along[None, :]
    ys = y + np.sin(angles)[:, None] * distances_along[None, :]
    rows = np.floor(ys / cs).astype(np.int64)
    cols = np.floor(xs / cs).astype(np.int64)
    rows_grid, cols_grid = truth.shape
    in_bounds = ((rows >= 0) & (rows < rows_grid)
                 & (cols >= 0) & (cols < cols_grid))
    blocked = np.ones(xs.shape, dtype=bool)
    blocked[in_bounds] = truth[rows[in_bounds], cols[in_bounds]] != FREE

    hits = blocked.any(axis=1)
    first_blocked = blocked.argmax(axis=1)
    distances = np.where(hits, distances_along[first_blocked], max_range)
    n_hits = int(hits.sum())
    if n_hits:
        distances[hits] += rng.normal(0.0, noise, n_hits)
    np.clip(distances, 0.0, max_range, out=distances)
    return [
        (float(angle), float(distance), bool(hit))
        for angle, distance, hit in zip(angles, distances, hits)
    ]


def _detect(world: dict, x: float, y: float, now: float,
            sensors: dict, rng: np.random.Generator) -> list:
    """Line-of-sight survivor detections (SimTargetDetector-style)."""
    truth = world["truth"]
    cs = world["cell_size"]
    detect_range = float(sensors["detect_range_m"])
    base_conf = float(sensors["detect_confidence"])
    conf_std = float(sensors["detect_confidence_std"])
    pos_noise = float(sensors["position_noise_m"])
    detections = []
    for sx, sy in world["survivors"]:
        distance = math.hypot(sx - x, sy - y)
        if distance > detect_range:
            continue
        if not _line_of_sight(truth, cs, x, y, sx, sy):
            continue
        confidence = float(np.clip(rng.normal(base_conf, conf_std), 0.5, 0.99))
        detections.append((
            sx + float(rng.normal(0.0, pos_noise)),
            sy + float(rng.normal(0.0, pos_noise)),
            confidence,
            now,
        ))
    return detections


def _footprint_free(truth: np.ndarray, cs: float, x: float, y: float,
                    ux: float, uy: float, radius: float) -> bool:
    return (
        _is_free(truth, cs, x, y)
        and _is_free(truth, cs, x + ux * radius, y + uy * radius)
        and _is_free(truth, cs, x - uy * radius, y + ux * radius)
        and _is_free(truth, cs, x + uy * radius, y - ux * radius)
    )


class _InflatedPlanner:
    """Mapper view that plans with the drone's footprint clearance.

    Clearance now lives in ``OccupancyMapper.plan_path`` (cell-center disk
    checks), so this view only forwards a planner built with the footprint
    radius. It keeps the historical constructor shape used by ``run``.
    """

    def __init__(self, mapper: OccupancyMapper, footprint_radius_m: float,
                 inflation_cells: int = 0) -> None:
        self._mapper = mapper
        self._radius = _positive(footprint_radius_m, "footprint_radius_m")
        if int(inflation_cells) < 0:
            raise ValueError("inflation_cells must be >= 0")
        self._inflation_cells = int(inflation_cells)  # retained for API compat

    def frontiers(self) -> list:
        return self._mapper.frontiers()

    def is_known_free(self, x: float, y: float) -> bool:
        return self._mapper.is_known_free(x, y)

    def plan_path(self, start, goal):
        return self._mapper.plan_path(start, goal)


def _advance(truth: np.ndarray, cs: float, x: float, y: float,
             dx: float, dy: float, radius: float) -> tuple[float, float]:
    """Move with footprint collision + axis-separated sliding on the truth grid."""
    distance = math.hypot(dx, dy)
    if distance == 0.0:
        return x, y
    ux, uy = dx / distance, dy / distance
    n_chunks = max(1, int(math.ceil(distance / (cs * 0.5))))
    step_x, step_y = dx / n_chunks, dy / n_chunks
    for _ in range(n_chunks):
        if _footprint_free(truth, cs, x + step_x, y + step_y, ux, uy, radius):
            x, y = x + step_x, y + step_y
            continue
        if step_x != 0.0 and _footprint_free(
                truth, cs, x + step_x, y, math.copysign(1.0, step_x), 0.0, radius):
            x += step_x
        if step_y != 0.0 and _footprint_free(
                truth, cs, x, y + step_y, 0.0, math.copysign(1.0, step_y), radius):
            y += step_y
    return x, y


# --------------------------------------------------------------------------
# Replay
# --------------------------------------------------------------------------
def _room_regions(world: dict, observed: np.ndarray) -> list[dict]:
    """Per-room exploration status for the report/PNG overlay."""
    regions = []
    rows, cols = observed.shape
    for room in world["rooms"]:
        x0, y0, x1, y1 = room["bounds"]
        c0, r0 = max(0, x0), max(0, y0)
        c1, r1 = min(cols - 1, x1), min(rows - 1, y1)
        if r1 < r0 or c1 < c0:
            fraction = 0.0
        else:
            fraction = float(observed[r0:r1 + 1, c0:c1 + 1].mean())
        regions.append({
            "region_id": room["region_id"],
            "name": room["name"],
            "center": room["center"],
            "bounds": room["bounds"],
            "explored_frac": fraction,
            "status": "SEARCHED" if fraction >= 0.90 else "PENDING",
        })
    return regions


def _step_toward_goal(explorer, local_planner, mapper, truth: np.ndarray,
                      cs: float, pose: tuple[float, float, float], now: float,
                      dt: float, radius: float, guard_radius: float
                      ) -> tuple[float, float, float]:
    """One control step: decide -> follow -> integrate. Returns the new pose."""
    x, y, theta = pose
    explorer.set_pose(x, y)
    goal = explorer.decide(now)
    if goal is None:
        return x, y, theta
    # Drop already-reached waypoints FIRST: the planner prepends the exact pose,
    # so guarding pose->path[0] would test a zero-length segment and deadlock
    # whenever the drone happens to sit near a mapped wall.
    while explorer.path and math.hypot(explorer.path[0][0] - x,
                                       explorer.path[0][1] - y) < 0.15:
        explorer.path.pop(0)
    if not explorer.path:
        explorer.notify_goal_reached(goal["goal_id"])
        local_planner.reset()
        return x, y, theta
    # Known-map safety guard: a wall discovered after planning can invalidate
    # the next leg. Drop the goal instead of flying into it. Skip the guard
    # when the drone itself is nearer than its guard radius to a mapped wall
    # (it must be allowed to retreat); only a newly blocked CLEAR leg aborts.
    if (not segment_clear(mapper, (x, y), explorer.path[0], guard_radius)
            and segment_clear(mapper, (x, y), (x, y), guard_radius)):
        explorer.notify_goal_failed(goal["goal_id"])
        local_planner.reset()
        return x, y, theta

    vx, vy, reached, stuck = local_planner.follow((x, y, theta), explorer.path, dt)
    if reached:
        explorer.notify_goal_reached(goal["goal_id"])
        local_planner.reset()
    elif stuck:
        explorer.notify_goal_failed(goal["goal_id"])
        local_planner.reset()
    else:
        new_x, new_y = _advance(truth, cs, x, y, vx * dt, vy * dt, radius)
        if (new_x, new_y) != (x, y):
            theta = math.atan2(new_y - y, new_x - x)
            x, y = new_x, new_y
    return x, y, theta


def _summarise(cfg: dict, world: dict, mapper: OccupancyMapper,
               tracker: SurvivorTracker, explorer: FrontierExplorer,
               path_history: list, steps: int, now: float,
               started: float) -> dict:
    """Score the finished run against ground truth and build the result dict."""
    truth = world["truth"]
    cs = world["cell_size"]
    radius = float(cfg["drone"]["footprint_radius_m"])
    grid = mapper.grid.copy()
    tracks = tracker.tracks
    segments = [(path_history[i][1:], path_history[i + 1][1:])
                for i in range(len(path_history) - 1)]

    return {
        "mission_complete": bool(explorer.mission_complete),
        "duration_s": float(now),
        "explored_pct": float(mapper.explored_fraction() * 100.0),
        "map_f1": validation_metrics.map_f1(grid, truth),
        "collisions": validation_metrics.collisions_from_truth(
            segments, truth, radius, cs),
        "survivors_tracked": sum(1 for track in tracks
                                 if track["state"] == "confirmed"),
        "truth_survivors": len(world["survivors"]),
        "track_records": tracks,
        "grid": grid,
        "truth_grid": truth.copy(),
        "path_history": path_history,
        # Report extras.
        "cell_size": cs,
        "regions": _room_regions(world, mapper.observed_mask()),
        "occupancy": validation_metrics.occupancy_counts(grid, truth),
        "survivor_matches": validation_metrics.survivor_match_count(
            world["survivors"], tracks,
            float(cfg["mission"]["survivor_match_radius_m"])),
        "steps": steps,
        "wall_time_s": float(time.perf_counter() - started),
    }


def run(config_dict: dict | None = None) -> dict:
    """Execute one deterministic replay and return the metrics result dict."""
    started = time.perf_counter()
    cfg = _merge_config(DEFAULT_CONFIG, config_dict)

    world = build_truth_world(cfg["world"])
    truth = world["truth"]
    cs = world["cell_size"]

    mapper = OccupancyMapper(
        cs, world["width_m"], world["height_m"],
        cfg["mapping"]["max_range_m"], cfg["mapping"]["clearance_m"],
    )
    tracker = SurvivorTracker(cfg["tracker"])
    entry = (world["entry"][0], world["entry"][1])
    # The explorer's tracks contract wants ``first_seen`` (investigation age);
    # the tracker's public record schema exposes ``timestamp`` (last seen), so
    # the harness remembers each track's first appearance.
    first_seen: dict[int, float] = {}

    def tracks_view() -> list[dict]:
        records = []
        for track in tracker.tracks:
            survivor_id = track["survivor_id"]
            first_seen.setdefault(survivor_id, float(track["timestamp"]))
            records.append({**track, "first_seen": first_seen[survivor_id]})
        return records

    explorer = FrontierExplorer(
        _InflatedPlanner(mapper, cfg["drone"]["footprint_radius_m"],
                         cfg["navigation"]["inflation_cells"]),
        tracks_view, entry, cfg["planner"])
    local_planner = LocalPlanner(cfg["drone"])
    rng = np.random.default_rng(int(cfg["seed"]))

    dt = _positive(cfg["mission"]["dt"], "dt")
    time_limit = float(cfg["mission"]["time_limit_s"])
    max_steps = int(cfg["mission"]["max_steps"])
    sample_steps = max(1, int(cfg["mission"]["path_sample_steps"]))
    scan_interval = _positive(cfg["sensors"]["scan_interval_s"], "scan_interval_s")
    detect_interval = _positive(cfg["sensors"]["detect_interval_s"],
                                "detect_interval_s")
    radius = float(cfg["drone"]["footprint_radius_m"])
    guard_radius = radius + float(cfg["navigation"]["segment_guard_margin_m"])

    pose = (float(entry[0]), float(entry[1]), 0.0)
    now = 0.0
    path_history: list[tuple[float, float, float]] = [(now, pose[0], pose[1])]
    # Prime the mapper so the first planning cycle sees observed space (an
    # empty grid has no frontiers and no known-free start cell).
    mapper.update(pose, _scan(truth, cs, pose[0], pose[1], cfg["sensors"], rng))
    next_scan = scan_interval
    next_detect = 0.0
    steps = 0

    while steps < max_steps and now < time_limit and not explorer.mission_complete:
        steps += 1
        now += dt
        pose = _step_toward_goal(explorer, local_planner, mapper, truth, cs,
                                 pose, now, dt, radius, guard_radius)

        if now >= next_scan:
            next_scan = now + scan_interval
            mapper.update(pose, _scan(truth, cs, pose[0], pose[1],
                                      cfg["sensors"], rng))
        if now >= next_detect:
            next_detect = now + detect_interval
            tracker.update(_detect(world, pose[0], pose[1], now,
                                   cfg["sensors"], rng), now)
        if steps % sample_steps == 0:
            path_history.append((now, pose[0], pose[1]))

    path_history.append((now, pose[0], pose[1]))
    if len(path_history) > 1 and path_history[-1][1:] == path_history[-2][1:]:
        path_history.pop()

    return _summarise(cfg, world, mapper, tracker, explorer, path_history,
                      steps, now, started)
