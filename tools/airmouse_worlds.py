"""Shared world, geometry and configuration helpers for the AirMouse tools.

Both ``tools/validate_maze.py`` (static geometry validation) and
``tools/headless_mission_smoke.py`` (mission walk) parse the same committed
SDF worlds and the same committed ROS 2 parameter files. Keeping the parsing
and the per-world expectations in one module means the two tools can never
disagree about what the arena looks like.
"""
from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
WORLDS_DIR = REPO_ROOT / "PX4-gazebo-models" / "worlds"
CONFIG_DIR = REPO_ROOT / "nidar_airmouse_ros" / "config"

# Make nidar_airmouse_sim importable when the tools are run from the repo root.
if str(REPO_ROOT / "nidar_airmouse_ros") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "nidar_airmouse_ros"))

# Resolution of the free-space flood fill used for reachability checks, in
# metres. 0.1 m resolves the narrowest doorway this project uses while keeping
# a 30 m arena under ~105k cells.
CONNECTIVITY_RESOLUTION = 0.10

# Radius of the survivor marker cylinder, used for the marker-clearance check.
SURVIVOR_MARKER_RADIUS = 0.28


@dataclass(frozen=True)
class WorldSpec:
    """One selectable arena: an SDF world plus its mission parameter file."""

    name: str
    sdf: Path
    config: Path
    expected_walls: frozenset[str]
    expected_survivors: frozenset[str]
    # Survivors that the committed mission walk is expected to validate.
    expected_validated: frozenset[str] = frozenset()
    # When True, every survivor marker must be reachable from home through the
    # inflated free space.
    require_reachable_survivors: bool = True

    @property
    def is_competition(self) -> bool:
        return self.name == "maze_compete"


WORLD_SPECS: dict[str, WorldSpec] = {
    "maze": WorldSpec(
        name="maze",
        sdf=WORLDS_DIR / "maze.sdf",
        config=CONFIG_DIR / "airmouse.yaml",
        expected_walls=frozenset({
            "wall_outer_west", "wall_outer_east", "wall_outer_south", "wall_outer_north",
            "wall_a", "wall_b", "wall_c", "wall_d", "wall_e", "wall_f", "wall_g",
            "wall_h1", "wall_h2", "wall_h3", "wall_h4", "wall_v1", "wall_v2",
        }),
        expected_survivors=frozenset(f"survivor_{i:02d}" for i in range(1, 7)),
        # Only these two are within the detection radius of the committed route.
        expected_validated=frozenset({"survivor_02", "survivor_03"}),
        # The practice course deliberately leaves some markers in sealed
        # pockets, so unreachable markers are reported but do not fail.
        require_reachable_survivors=False,
    ),
    "maze_compete": WorldSpec(
        name="maze_compete",
        sdf=WORLDS_DIR / "maze_compete.sdf",
        config=CONFIG_DIR / "airmouse_compete.yaml",
        expected_walls=frozenset({
            "wall_outer_west", "wall_outer_east", "wall_outer_south", "wall_outer_north",
            "wall_v1a", "wall_v1b", "wall_v2", "wall_v3a", "wall_v3b", "wall_v4",
        }),
        expected_survivors=frozenset(f"survivor_{i:02d}" for i in range(1, 7)),
        expected_validated=frozenset(f"survivor_{i:02d}" for i in range(1, 7)),
        require_reachable_survivors=True,
    ),
}

DEFAULT_WORLD = "maze"


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


# ---------- SDF geometry ----------------------------------------------------

def _floats(text: str | None) -> tuple[float, ...]:
    return tuple(float(value) for value in (text or "").split())


def load_walls(
    sdf_path: Path, world_name: str | None = None
) -> list[tuple[str, tuple[float, float, float, float]]]:
    """Return [(name, (x_min, x_max, y_min, y_max))] for every wall model."""
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{sdf_path.name} contains no <world> element")
    if world_name is not None and world.attrib.get("name") != world_name:
        raise ValueError(
            f"{sdf_path.name} is a world named '{world.attrib.get('name')}', expected '{world_name}'"
        )
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


def load_survivors_gz(sdf_path: Path) -> dict[str, tuple[float, float, float]]:
    """Return {name: (gx, gy, gz)} for every survivor, in Gazebo ENU."""
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{sdf_path.name} contains no <world> element")
    out: dict[str, tuple[float, float, float]] = {}
    for model in world.findall("model"):
        name = model.attrib.get("name", "")
        if not name.startswith("survivor_"):
            continue
        pose = _floats(model.findtext("pose"))
        out[name] = (pose[0], pose[1], pose[2])
    return out


def model_has_collision(sdf_path: Path, prefix: str) -> dict[str, bool]:
    """Return {model_name: has_collision} for models matching ``prefix``."""
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{sdf_path.name} contains no <world> element")
    out: dict[str, bool] = {}
    for model in world.findall("model"):
        name = model.attrib.get("name", "")
        if name.startswith(prefix):
            out[name] = model.find("./link/collision") is not None
    return out


# ---------- Axis-aligned rectangle predicates -------------------------------

Rect = tuple[float, float, float, float]  # (x_min, x_max, y_min, y_max)


def segment_intersects_rect(start, end, rect: Rect) -> bool:
    min_x, max_x, min_y, max_y = rect
    seg_min_x, seg_max_x = sorted((start[0], end[0]))
    seg_min_y, seg_max_y = sorted((start[1], end[1]))
    return seg_max_x >= min_x and seg_min_x <= max_x and seg_max_y >= min_y and seg_min_y <= max_y


def segment_rect_distance(start, end, rect: Rect) -> float:
    if segment_intersects_rect(start, end, rect):
        return 0.0
    min_x, max_x, min_y, max_y = rect
    seg_min_x, seg_max_x = sorted((start[0], end[0]))
    seg_min_y, seg_max_y = sorted((start[1], end[1]))
    dx = max(min_x - seg_max_x, seg_min_x - max_x, 0.0)
    dy = max(min_y - seg_max_y, seg_min_y - max_y, 0.0)
    return math.hypot(dx, dy)


def inflate(rect: Rect, radius: float) -> Rect:
    return (rect[0] - radius, rect[1] + radius, rect[2] - radius, rect[3] + radius)


# ---------- Free-space connectivity -----------------------------------------

class FreeSpace:
    """Occupancy grid over the free space of a world, inflated by a radius."""

    def __init__(
        self,
        wall_rects: list[tuple[str, Rect]],
        envelope_radius: float,
        resolution: float = CONNECTIVITY_RESOLUTION,
        pad: float = 1.0,
    ) -> None:
        inflated = [inflate(rect, envelope_radius) for _name, rect in wall_rects]
        self.min_x = min(rect[0] for rect in inflated) - pad
        self.max_x = max(rect[1] for rect in inflated) + pad
        self.min_y = min(rect[2] for rect in inflated) - pad
        self.max_y = max(rect[3] for rect in inflated) + pad
        self.resolution = resolution
        self.nx = int(math.ceil((self.max_x - self.min_x) / resolution))
        self.ny = int(math.ceil((self.max_y - self.min_y) / resolution))
        self.free = bytearray(self.nx * self.ny)
        for j in range(self.ny):
            py = self.min_y + (j + 0.5) * resolution
            row = j * self.nx
            for i in range(self.nx):
                px = self.min_x + (i + 0.5) * resolution
                if not any(segment_intersects_rect((px, py), (px, py), r) for r in inflated):
                    self.free[row + i] = 1

    def index_of(self, x: float, y: float) -> int:
        i = int((x - self.min_x) / self.resolution)
        j = int((y - self.min_y) / self.resolution)
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            raise ValueError(f"point ({x:.2f}, {y:.2f}) is outside the arena bounding box")
        return j * self.nx + i

    def is_free(self, x: float, y: float) -> bool:
        return bool(self.free[self.index_of(x, y)])

    def component_of(self, x: float, y: float) -> set[int]:
        """Flood fill the connected free component containing (x, y)."""
        start = self.index_of(x, y)
        if not self.free[start]:
            return set()
        seen = {start}
        queue = deque([start])
        while queue:
            cell = queue.popleft()
            i, j = cell % self.nx, cell // self.nx
            for ni, nj in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if not (0 <= ni < self.nx and 0 <= nj < self.ny):
                    continue
                nxt = nj * self.nx + ni
                if self.free[nxt] and nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        return seen


# ---------- Mission configuration -------------------------------------------

@dataclass
class MissionProfile:
    """The parts of a ROS 2 parameter file the offline tools need."""

    waypoints_px4: list[tuple[float, float, float]] = field(default_factory=list)
    survivors_px4: list[tuple[str, tuple[float, float, float]]] = field(default_factory=list)
    geofence: dict[str, float] = field(default_factory=dict)
    detection_radius: float = 2.5
    min_observations: int = 2
    takeoff_altitude: float = 2.5


def _triples(flat: list[float], label: str) -> list[tuple[float, float, float]]:
    if len(flat) % 3 != 0:
        raise ValueError(f"{label} must contain x/y/z triples, got {len(flat)} values")
    return [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]


def load_mission_profile(config_path: Path) -> MissionProfile:
    """Parse the committed ROS 2 parameter file into a MissionProfile."""
    with open(config_path) as handle:
        params = yaml.safe_load(handle)

    mission = params["nidar_airmouse_mission"]["ros__parameters"]
    waypoints = _triples([float(v) for v in mission["mission_waypoints"]], "mission_waypoints")
    geofence = {k: float(mission[k]) for k in
                ("geofence_x_min", "geofence_x_max", "geofence_y_min", "geofence_y_max")}

    detector = params.get("nidar_airmouse_sim_survivor_detector", {}).get("ros__parameters", {})
    targets = _triples([float(v) for v in detector.get("survivor_targets", [])], "survivor_targets")
    survivors = [(f"survivor_{i + 1:02d}", t) for i, t in enumerate(targets)]

    localizer = params.get("nidar_airmouse_survivor_localizer", {}).get("ros__parameters", {})

    return MissionProfile(
        waypoints_px4=waypoints,
        survivors_px4=survivors,
        geofence=geofence,
        detection_radius=float(detector.get("detection_radius", 2.5)),
        min_observations=int(localizer.get("min_observations", 2)),
        takeoff_altitude=float(mission.get("takeoff_altitude", 2.5)),
    )


def route_gz(profile: MissionProfile) -> list[tuple[float, float]]:
    """Return the whole route in Gazebo ENU, starting from the home origin."""
    points = [(0.0, 0.0, 0.0)] + list(profile.waypoints_px4)
    return [px4_to_gazebo(x, y, z)[:2] for x, y, z in points]
