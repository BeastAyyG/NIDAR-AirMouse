#!/usr/bin/env python3
"""Headless smoke test for the AirMouse mission pipeline.

This script is the no-WSL substitute for the live SITL/Gazebo/PX4 run. It uses:

* the committed ``PX4-gazebo-models/worlds/maze.sdf`` geometry,
* the real ``nidar_airmouse_sim.safety.assess_scan`` LiDAR classifier,
* the real ``nidar_airmouse_sim.grid`` world-to-grid conversion,
* the committed ``config/airmouse.yaml`` mission_waypoints and survivor targets,
* a synthetic 2D LiDAR scan computed by raycasting against the maze walls,
* a synthetic PX4 vehicle that follows the waypoint list.

It walks the mission state machine (``PRECHECK -> ARM/TAKEOFF ->
HOME_ANCHOR -> EXPLORE -> SURVIVOR_VERIFY -> MARK_GRID -> RETURN_HOME ->
LAND/COMPLETE``) and prints a detailed per-step trace so you can see the
drone moving, the LiDAR minimum range, which survivors enter the
detection radius, and the safety contract's verdict at every step.

Use ``--quiet`` to print only the summary and ``--verbose`` (default) for
the full step-by-step trace.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "nidar_airmouse_ros"))

from nidar_airmouse_sim.grid import GridConfig, cell_key, grid_to_world, world_to_grid
from nidar_airmouse_sim.safety import assess_scan


# ---------- Frame conversion ------------------------------------------------

def px4_to_gazebo(x: float, y: float, z: float) -> tuple[float, float, float]:
    """PX4 local NED (x=north, y=east, z=down) -> Gazebo ENU (x=east, y=north).

    Mirrors the convention documented in ``docs/MAZE-SPEC.md``::

        gazebo_x = px4_y_east
        gazebo_y = px4_x_north
        gazebo_z = -px4_z_down
    """
    return (y, x, -z)


def gazebo_to_px4(x: float, y: float, z: float) -> tuple[float, float, float]:
    return (y, x, -z)


# ---------- Maze geometry --------------------------------------------------

def _floats(text: str | None) -> tuple[float, ...]:
    return tuple(float(v) for v in (text or "").split())


def load_walls(maze_sdf: Path) -> list[tuple[str, tuple[float, float, float, float]]]:
    """Return [(name, (x_min, x_max, y_min, y_max))] for every wall model."""
    root = ET.parse(maze_sdf).getroot()
    world = root.find("world")
    if world is None or world.attrib.get("name") != "maze":
        raise SystemExit("maze.sdf is not a world named 'maze'")
    rects: list[tuple[str, tuple[float, float, float, float]]] = []
    for model in world.findall("model"):
        name = model.attrib.get("name", "")
        if not name.startswith("wall_"):
            continue
        pose = _floats(model.findtext("pose"))
        size = _floats(model.find("./link/collision/geometry/box/size").text)
        cx, cy = pose[0], pose[1]
        sx, sy = size[0], size[1]
        rects.append((name, (cx - sx / 2.0, cx + sx / 2.0, cy - sy / 2.0, cy + sy / 2.0)))
    return rects


def load_survivors_gz(maze_sdf: Path) -> dict[str, tuple[float, float, float]]:
    """Return {name: (gx, gy, gz)} for every survivor, in Gazebo ENU."""
    root = ET.parse(maze_sdf).getroot()
    world = root.find("world")
    out: dict[str, tuple[float, float, float]] = {}
    for model in world.findall("model"):
        name = model.attrib.get("name", "")
        if not name.startswith("survivor_"):
            continue
        pose = _floats(model.findtext("pose"))
        out[name] = (pose[0], pose[1], pose[2])
    return out


# ---------- Synthetic LiDAR ------------------------------------------------

def cast_scan(
    origin: tuple[float, float],
    walls: list[tuple[str, tuple[float, float, float, float]]],
    *,
    samples: int = 1080,
    angle_min: float = -math.pi,
    angle_max: float = math.pi,
    range_min: float = 0.05,
    range_max: float = 12.0,
) -> tuple[list[float], list[float]]:
    """Return (ranges, per-beam directions) by raycasting against walls.

    The second list is the corresponding unit direction vector for each
    range sample so callers can attribute the minimum range to a bearing.
    """
    ox, oy = origin
    ranges: list[float] = []
    bearings: list[float] = []
    for i in range(samples):
        theta = angle_min + (angle_max - angle_min) * (i / samples)
        dx, dy = math.cos(theta), math.sin(theta)
        best = range_max
        hit = False
        for _, (xmin, xmax, ymin, ymax) in walls:
            t_candidates: list[float] = []
            if abs(dx) > 1e-9:
                t_candidates.append((xmin - ox) / dx)
                t_candidates.append((xmax - ox) / dx)
            if abs(dy) > 1e-9:
                t_candidates.append((ymin - oy) / dy)
                t_candidates.append((ymax - oy) / dy)
            for t in t_candidates:
                if t <= 0 or t > best:
                    continue
                ix = ox + dx * t
                iy = oy + dy * t
                if xmin - 1e-9 <= ix <= xmax + 1e-9 and ymin - 1e-9 <= iy <= ymax + 1e-9:
                    best = t
                    hit = True
        clamped = max(range_min, min(range_max, best))
        ranges.append(clamped)
        bearings.append(theta)
    return ranges, bearings


def nearest_wall_in_direction(
    origin_gz: tuple[float, float],
    direction_deg: float,
    walls: list[tuple[str, tuple[float, float, float, float]]],
    range_max: float = 12.0,
) -> tuple[str, float]:
    """Return (wall_name, range) for the closest hit along the requested bearing."""
    theta = math.radians(direction_deg)
    dx, dy = math.cos(theta), math.sin(theta)
    ox, oy = origin_gz
    best = range_max
    best_name = ""
    for name, (xmin, xmax, ymin, ymax) in walls:
        t_candidates: list[float] = []
        if abs(dx) > 1e-9:
            t_candidates.append((xmin - ox) / dx)
            t_candidates.append((xmax - ox) / dx)
        if abs(dy) > 1e-9:
            t_candidates.append((ymin - oy) / dy)
            t_candidates.append((ymax - oy) / dy)
        for t in t_candidates:
            if t <= 0 or t > best:
                continue
            ix = ox + dx * t
            iy = oy + dy * t
            if xmin - 1e-9 <= ix <= xmax + 1e-9 and ymin - 1e-9 <= iy <= ymax + 1e-9:
                best = t
                best_name = name
    return best_name, best


# ---------- Mission walker --------------------------------------------------

@dataclass
class WalkerConfig:
    waypoints_px4: list[tuple[float, float, float]]
    home_px4: tuple[float, float, float]
    survivors_px4: list[tuple[str, tuple[float, float, float]]]
    detection_radius: float = 2.5
    grid: GridConfig = GridConfig()
    min_observations: int = 2


@dataclass
class MissionRecord:
    state: str
    position_px4: tuple[float, float, float]
    position_gz: tuple[float, float, float]
    lidar_min: float
    lidar_min_bearing_deg: float
    lidar_min_wall: str
    safety_ready: bool
    safety_safe: bool
    safety_reason: str
    validated_survivors: list[str]
    in_range_survivors: list[str]
    distance_to_target: float
    notes: str = ""
    step_index: int = 0


@dataclass
class Simulator:
    walls: list[tuple[str, tuple[float, float, float, float]]]
    cfg: WalkerConfig
    samples: int
    update_rate: float
    profile: str
    verbose: bool = True
    step_index: int = 0
    records: list[MissionRecord] = field(default_factory=list)
    observations: dict[str, dict] = field(default_factory=dict)
    validated: list[str] = field(default_factory=list)
    state: str = "BOOT"
    waypoint_index: int = 0
    heartbeat_tick: int = 0

    def detect(self, px4_xyz: tuple[float, float, float]) -> None:
        for sid, (x, y, z) in self.cfg.survivors_px4:
            if math.hypot(px4_xyz[0] - x, px4_xyz[1] - y) <= self.cfg.detection_radius:
                gx, gy = world_to_grid(x, y, self.cfg.grid)
                key = cell_key(gx, gy)
                rec = self.observations.setdefault(
                    key,
                    {"id": sid, "grid_x": gx, "grid_y": gy,
                     "x": x, "y": y, "z": z,
                     "observations": 0, "validated": False},
                )
                rec["observations"] += 1
                if rec["observations"] >= self.cfg.min_observations and not rec["validated"]:
                    rec["validated"] = True
                    if sid not in self.validated:
                        self.validated.append(sid)

    def cast_at_px4(self, px4_xyz: tuple[float, float, float]) -> tuple[float, float, str]:
        gx, gy, gz = px4_to_gazebo(*px4_xyz)
        ranges, bearings = cast_scan((gx, gy), self.walls, samples=self.samples, range_max=12.0)
        if not ranges:
            return (math.inf, 0.0, "")
        idx = min(range(len(ranges)), key=lambda i: ranges[i])
        bearing = math.degrees(bearings[idx])
        wall, _ = nearest_wall_in_direction((gx, gy), bearing, self.walls)
        return (ranges[idx], bearing, wall)

    def survivors_in_range(self, px4_xyz: tuple[float, float, float]) -> list[str]:
        return [
            sid
            for sid, (x, y, _z) in self.cfg.survivors_px4
            if math.hypot(px4_xyz[0] - x, px4_xyz[1] - y) <= self.cfg.detection_radius
        ]

    def record(self, *, state: str, px4_xyz: tuple[float, float, float], notes: str = "") -> MissionRecord:
        scan_min, scan_bearing_deg, scan_wall = self.cast_at_px4(px4_xyz)
        scan_ranges, _ = cast_scan(px4_to_gazebo(*px4_xyz)[:2], self.walls,
                                  samples=self.samples, range_max=12.0)
        assessment = assess_scan(scan_ranges, 0.05, 12.0, 0.55, 0.40, 20)
        in_range = self.survivors_in_range(px4_xyz)
        target = self.cfg.waypoints_px4[self.waypoint_index] if self.waypoint_index < len(self.cfg.waypoints_px4) else self.cfg.home_px4
        distance_to_target = math.hypot(px4_xyz[0] - target[0], px4_xyz[1] - target[1])
        rec = MissionRecord(
            state=state,
            position_px4=px4_xyz,
            position_gz=px4_to_gazebo(*px4_xyz),
            lidar_min=scan_min,
            lidar_min_bearing_deg=scan_bearing_deg,
            lidar_min_wall=scan_wall,
            safety_ready=assessment.ready,
            safety_safe=assessment.safe,
            safety_reason=assessment.reason,
            validated_survivors=list(self.validated),
            in_range_survivors=in_range,
            distance_to_target=distance_to_target,
            notes=notes,
            step_index=self.step_index,
        )
        self.records.append(rec)
        if self.verbose:
            print(self._format_step(rec))
        self.step_index += 1
        return rec

    @staticmethod
    def _format_step(rec: MissionRecord) -> str:
        px, py, pz = rec.position_px4
        gx, gy, gz = rec.position_gz
        marker = "OK" if rec.safety_safe else "!!"
        safe = f"{marker} safe={rec.safety_safe!s:5s} reason={rec.safety_reason:24s}"
        return (
            f"  step={rec.step_index:4d} state={rec.state:18s} "
            f"px4=({px:+6.2f},{py:+6.2f},{pz:+5.2f}) gz=({gx:+6.2f},{gy:+6.2f},{gz:+5.2f}) "
            f"lidar_min={rec.lidar_min:5.2f}m@{rec.lidar_min_bearing_deg:+6.1f}deg "
            f"-> {rec.lidar_min_wall or 'open':<18s} dst_target={rec.distance_to_target:5.2f}m "
            f"in_range={','.join(rec.in_range_survivors) or '-':<28s} "
            f"validated={','.join(rec.validated_survivors) or '-':<18s} "
            f"{safe} {rec.notes}"
        )

    def linear_path(self, a: tuple[float, float, float], b: tuple[float, float, float]):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        dist = math.hypot(dx, dy, dz)
        n = max(1, int(dist / 0.4))
        for i in range(n + 1):
            t = i / n
            yield (a[0] + dx * t, a[1] + dy * t, a[2] + dz * t)

    def run(self) -> tuple[str, list[MissionRecord]]:
        if self.verbose:
            print(f"=== profile={self.profile} samples={self.samples} "
                  f"update_rate={self.update_rate}Hz waypoints={len(self.cfg.waypoints_px4)} ===")
            print(f"home_px4={self.cfg.home_px4}  home_gz={px4_to_gazebo(*self.cfg.home_px4)}")

        # BOOT -> PRECHECK (3s wait, scanning at home pose)
        self.state = "PRECHECK"
        home = self.cfg.home_px4
        for tick in range(3):
            self.heartbeat_tick += 1
            self.record(state=self.state, px4_xyz=home, notes=f"preflight heartbeat {tick + 1}/3")

        # PRECHECK -> ARM/TAKEOFF (climb to takeoff altitude)
        self.state = "ARM/TAKEOFF"
        climb_target = (home[0], home[1], home[2] + 2.5)
        for pos in self.linear_path(home, climb_target):
            self.heartbeat_tick += 1
            self.record(state=self.state, px4_xyz=pos, notes=f"climb->{climb_target[2]:+.2f}m heartbeat={self.heartbeat_tick}")
        self.record(state=self.state, px4_xyz=climb_target, notes="offboard armed + takeoff complete")

        # ARM/TAKEOFF -> HOME_ANCHOR (settle at home altitude for 2 s)
        self.state = "HOME_ANCHOR"
        self.record(state=self.state, px4_xyz=climb_target, notes="home anchor hold")
        # Resolve real home after the climb
        real_home = (home[0], home[1], home[2] + 2.5)

        # HOME_ANCHOR -> EXPLORE (fly the route)
        self.state = "EXPLORE"
        previous = real_home
        for idx, waypoint in enumerate(self.cfg.waypoints_px4):
            self.waypoint_index = idx
            target_label = f"WP{idx + 1}/{len(self.cfg.waypoints_px4)} ({waypoint[0]:+.1f},{waypoint[1]:+.1f},{waypoint[2]:+.1f})"
            for pos in self.linear_path(previous, waypoint):
                self.heartbeat_tick += 1
                self.detect(pos)
                rec = self.record(state=self.state, px4_xyz=pos, notes=f"en-route {target_label} heartbeat={self.heartbeat_tick}")
                if not rec.safety_safe:
                    raise SystemExit(f"safety halted at {pos}: {rec.safety_reason}")
            previous = waypoint

        # EXPLORE -> SURVIVOR_VERIFY (dwell on the last waypoint for 2 s)
        self.state = "SURVIVOR_VERIFY"
        for dwell in range(2):
            self.heartbeat_tick += 1
            self.record(state=self.state, px4_xyz=previous, notes=f"verify dwell {dwell + 1}/2")

        # SURVIVOR_VERIFY -> MARK_GRID
        self.state = "MARK_GRID"
        for sid in self.validated:
            rec = next(v for k, v in self.observations.items() if v["id"] == sid)
            gx, gy = rec["grid_x"], rec["grid_y"]
            world_xy = grid_to_world(gx, gy, self.cfg.grid)
            self.record(state="MARK_GRID", px4_xyz=(world_xy[0], world_xy[1], 0.0),
                        notes=f"tagged {sid} at cell {gx}:{gy} -> {world_xy}")

        # MARK_GRID -> RETURN_HOME
        self.state = "RETURN_HOME"
        home_after = (0.0, 0.0, -2.5)
        for pos in self.linear_path(previous, home_after):
            self.heartbeat_tick += 1
            rec = self.record(state=self.state, px4_xyz=pos, notes=f"RTL heartbeat={self.heartbeat_tick}")
            if not rec.safety_safe:
                raise SystemExit(f"safety halted on RTL at {pos}: {rec.safety_reason}")
        self.record(state=self.state, px4_xyz=home_after, notes="RTL complete")

        # RETURN_HOME -> LAND/COMPLETE (descend to ground)
        self.state = "LAND/COMPLETE"
        ground = (home_after[0], home_after[1], 0.0)
        for pos in self.linear_path(home_after, ground):
            self.heartbeat_tick += 1
            self.record(state=self.state, px4_xyz=pos, notes=f"land heartbeat={self.heartbeat_tick}")
        self.record(state=self.state, px4_xyz=ground, notes="disarmed, mission complete")
        return self.state, self.records


# ---------- Entry point -----------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--maze", type=Path, default=REPO_ROOT / "PX4-gazebo-models" / "worlds" / "maze.sdf")
    parser.add_argument("--config", type=Path, default=REPO_ROOT / "nidar_airmouse_ros" / "config" / "airmouse.yaml")
    parser.add_argument("--profile", choices=["fast", "accurate"], default="accurate")
    parser.add_argument("--quiet", action="store_true", help="only print the summary")
    parser.add_argument("--limit", type=int, default=20, help="show at most this many sample steps in the summary")
    args = parser.parse_args()

    walls = load_walls(args.maze)
    expected_wall_names = {
        "wall_a", "wall_b", "wall_c", "wall_d", "wall_e", "wall_f", "wall_g",
        "wall_h1", "wall_h2", "wall_h3", "wall_h4", "wall_v1", "wall_v2",
        "wall_outer_west", "wall_outer_east", "wall_outer_south", "wall_outer_north",
    }
    wall_names = {name for name, _ in walls}
    if wall_names != expected_wall_names:
        raise SystemExit(f"wall set mismatch: {wall_names ^ expected_wall_names}")

    with open(args.config) as f:
        yaml_cfg = yaml.safe_load(f)
    raw_waypoints = yaml_cfg["nidar_airmouse_mission"]["ros__parameters"]["mission_waypoints"]
    waypoints = [tuple(raw_waypoints[i:i + 3]) for i in range(0, len(raw_waypoints), 3)]
    raw_targets = yaml_cfg["nidar_airmouse_sim_survivor_detector"]["ros__parameters"]["survivor_targets"]
    survivor_targets = [tuple(raw_targets[i:i + 3]) for i in range(0, len(raw_targets), 3)]
    survivor_records = [(f"survivor_{i + 1:02d}", survivor_targets[i]) for i in range(len(survivor_targets))]

    samples = 1080 if args.profile == "accurate" else 360
    update_rate = 30.0 if args.profile == "accurate" else 10.0

    cfg = WalkerConfig(
        waypoints_px4=waypoints,
        home_px4=(0.0, 0.0, -2.5),
        survivors_px4=survivor_records,
    )
    sim = Simulator(walls=walls, cfg=cfg, samples=samples, update_rate=update_rate,
                    profile=args.profile, verbose=not args.quiet)

    start = time.time()
    final_state, records = sim.run()
    elapsed = time.time() - start

    if args.quiet:
        print(f"profile={args.profile} steps={len(records)} final={final_state} wall_time={elapsed:.2f}s")

    print()
    print("=" * 84)
    print(f"Summary (profile={args.profile}, samples={samples} @ {update_rate:g} Hz)")
    print("=" * 84)
    print(f"  walls            : {len(walls)} ({sorted(wall_names)})")
    print(f"  survivor markers : {len(survivor_records)}")
    print(f"  mission waypoints: {len(waypoints)}")
    print(f"  steps simulated  : {len(records)}")
    print(f"  wall clock       : {elapsed:.2f}s")
    print(f"  final state      : {final_state}")
    final = records[-1]
    print(f"  final position   : PX4={tuple(round(v, 2) for v in final.position_px4)} GZ={tuple(round(v, 2) for v in final.position_gz)}")
    print(f"  validated        : {final.validated_survivors}")
    bad = [r for r in records if not r.safety_safe or not r.safety_ready]
    print(f"  safety pass rate : {len(records) - len(bad)}/{len(records)} ready+safe")
    if bad:
        print(f"  safety failures  :")
        for r in bad[: args.limit]:
            print(f"      step={r.step_index} {r.state:18s} px4=({r.position_px4[0]:+.2f},{r.position_px4[1]:+.2f},{r.position_px4[2]:+.2f}) reason={r.safety_reason}")
    print(f"  unique safety reasons: {sorted({r.safety_reason for r in records})}")

    # Show the LiDAR minimum range distribution and the bearing distribution.
    mins = sorted(r.lidar_min for r in records)
    print(f"  LiDAR min range  : min={mins[0]:.2f}m  median={mins[len(mins) // 2]:.2f}m  max={mins[-1]:.2f}m")
    tight = sorted(records, key=lambda r: r.lidar_min)[:5]
    print(f"  tightest 5 steps (lowest LiDAR min range):")
    for r in tight:
        print(f"      step={r.step_index:3d} {r.state:18s} px4=({r.position_px4[0]:+.2f},{r.position_px4[1]:+.2f},{r.position_px4[2]:+.2f}) "
              f"lidar={r.lidar_min:.2f}m@{r.lidar_min_bearing_deg:+.1f}deg wall={r.lidar_min_wall}")

    print(f"  walls ever seen as nearest:")
    wall_counts: dict[str, int] = {}
    for r in records:
        if r.lidar_min_wall:
            wall_counts[r.lidar_min_wall] = wall_counts.get(r.lidar_min_wall, 0) + 1
    for wall_name in sorted(wall_counts):
        print(f"      {wall_name:<22s} {wall_counts[wall_name]} steps")

    print(f"  survivor first-contact events:")
    for sid in sorted({r.in_range_survivors[0] for r in records if r.in_range_survivors}):
        first = next(r for r in records if sid in r.in_range_survivors)
        last = next(r for r in reversed(records) if sid in r.in_range_survivors)
        print(f"      {sid}: first step={first.step_index} px4=({first.position_px4[0]:+.2f},{first.position_px4[1]:+.2f}), "
              f"last step={last.step_index} px4=({last.position_px4[0]:+.2f},{last.position_px4[1]:+.2f})")

    # Hard assertions on the final outcome.
    if final_state != "LAND/COMPLETE":
        print(f"FAIL: final state {final_state} != LAND/COMPLETE")
        return 1
    home_after = (0.0, 0.0, -2.5)
    if math.hypot(final.position_px4[0] - home_after[0], final.position_px4[1] - home_after[1]) > 0.05:
        print(f"FAIL: did not return to home")
        return 1
    if "survivor_02" not in final.validated_survivors:
        print("FAIL: did not validate survivor_02")
        return 1
    if "survivor_03" not in final.validated_survivors:
        print("FAIL: did not validate survivor_03")
        return 1
    print()
    print("ALL ASSERTIONS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
