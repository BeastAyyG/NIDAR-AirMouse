"""Live 2D occupancy mapper for the AirMouse drone (ROS-independent core).

Ported from the validated NIDAR SAR sim (``sar/mapping/occupancy.py``,
141 green upstream tests) with the SimConfig constructor replaced by explicit
parameters. numpy + stdlib only; NO rclpy/ROS imports.

Convention: meters, x right, y DOWN, ``grid[row, col]`` with
``row = int(y / cell_size)`` and ``col = int(x / cell_size)``.
Cell values: ``UNKNOWN = -1``, ``FREE = 0``, ``OCCUPIED = 100`` (int8 grid).

Design notes (unchanged from upstream)
--------------------------------------
* Ray casting uses a supercover grid traversal so every cell the ray touches
  is visited; walls stay leak-free (no diagonal gaps, nothing behind a hit is
  freed).
* Occupancy uses lightweight hit/miss counting (NOT sticky): each ray endpoint
  hit adds +2 (capped at +6), each free traversal subtracts 1 (floored at -4).
  A cell is OCCUPIED while its count is >= 1. This absorbs isolated noisy
  endpoint hits (a false wall clears after a couple of clean passes) while
  real walls stay solid, and keeps doorways from sealing permanently. The
  drone's own pose cell is force-cleared (the drone cannot be inside a wall).
* Frontier clustering and reachability use numpy + ``collections.deque`` only;
  no scipy dependency.
* Path smoothing checks the whole drone disk (fine-marched ``_line_free``),
  not just the centerline, so smoothed segments cannot clip wall corners.

Flight-height-band hook
-----------------------
``height_band`` is an informational no-op hook: the perception lane filters
the depth cloud to the flight band BEFORE projecting to 2D rays, so this
mapper never filters by height itself. Nodes may store ``(zmin, zmax)`` here
for introspection/logging; it does not affect mapping behavior.
"""
from __future__ import annotations

import heapq
import math
from collections import deque

import numpy as np

from .grid_manager import grid_shape, resized_copy

UNKNOWN = -1
FREE = 0
OCCUPIED = 100

SQRT2 = math.sqrt(2.0)
MIN_FRONTIER_CLUSTER = 3
GOAL_SEARCH_RADIUS = 3

# 8-connected neighbourhood (row, col) offsets; deterministic order.
NEIGHBORS8: tuple[tuple[int, int], ...] = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),           (0, 1),
    (1, -1),  (1, 0),  (1, 1),
)
# A* moves with traversal cost (diagonal = sqrt2).
MOVES: tuple[tuple[int, int, float], ...] = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
)


def supercover_line(r0: int, c0: int, r1: int, c1: int) -> list[tuple[int, int]]:
    """Return every grid cell the segment (r0,c0)->(r1,c1) touches.

    Supercover traversal: at an exact corner crossing both adjacent cells are
    visited, which keeps diagonal walls leak-free. Endpoints are included.
    """
    dx = c1 - c0
    dy = r1 - r0
    nx = abs(dx)
    ny = abs(dy)
    sx = 1 if dx > 0 else -1
    sy = 1 if dy > 0 else -1
    r, c = r0, c0
    points = [(r, c)]
    ix = iy = 0
    while ix < nx or iy < ny:
        if (1 + 2 * ix) * ny == (1 + 2 * iy) * nx:
            c += sx
            r += sy
            ix += 1
            iy += 1
        elif (1 + 2 * ix) * ny < (1 + 2 * iy) * nx:
            c += sx
            ix += 1
        else:
            r += sy
            iy += 1
        points.append((r, c))
    return points


def _octile(r: int, c: int, gr: int, gc: int) -> float:
    """Admissible heuristic for 8-connected movement with diagonal cost sqrt2."""
    dr = abs(r - gr)
    dc = abs(c - gc)
    return max(dr, dc) + (SQRT2 - 1.0) * min(dr, dc)


class OccupancyMapper:
    """Sensor-built occupancy grid with frontiers, A* planning and coverage.

    Parameters
    ----------
    cell_size : float
        Grid resolution in meters/cell (> 0).
    width_m, height_m : float
        World extent in meters (> 0); grid is ``int(size / cell_size)`` cells.
    max_range : float
        Sensor range cap in meters (> 0); ray distances are clamped to it.
    clearance : float
        Drone disk radius in meters (>= 0) used by LOS path smoothing.

    API: ``.grid``, ``.update(pose, rays)``, ``.frontiers()``,
    ``.plan_path(start, goal)``, ``.is_known_free(x, y)``, ``.observed_mask()``,
    ``.explored_fraction()``, ``.reachable_free_mask()``, ``.resize(w, h)``.
    ``pose = (x, y, theta)``; ``rays = list[(angle, distance, hit)]`` with
    absolute ray angles (theta 0 = +x, clockwise / y-down).
    """

    def __init__(self, cell_size: float, width_m: float, height_m: float,
                 max_range: float, clearance: float) -> None:
        self.cell_size = self._positive(cell_size, "cell_size")
        self.width_m = self._positive(width_m, "width_m")
        self.height_m = self._positive(height_m, "height_m")
        self.max_range = self._positive(max_range, "max_range")
        clr = float(clearance)
        if not math.isfinite(clr) or clr < 0.0:
            raise ValueError(f"clearance must be finite and >= 0, got {clearance!r}")
        self.clearance = clr
        # Informational flight-height-band hook (perception filters upstream).
        self.height_band: tuple[float, float] | None = None
        rows, cols = grid_shape(self.width_m, self.height_m, self.cell_size)
        self.grid = np.full((rows, cols), UNKNOWN, dtype=np.int8)
        self._counts = np.zeros((rows, cols), dtype=np.int16)
        self._observed = np.zeros((rows, cols), dtype=bool)
        self._rows = rows
        self._cols = cols
        self._seed: tuple[int, int] | None = None  # first cell ever marked FREE
        self._version = 0           # bumped whenever a grid value changes
        self._reach_cache: np.ndarray | None = None
        self._reach_version = -1
        self._clear_cache: np.ndarray | None = None
        self._clear_version = -1

    @staticmethod
    def _positive(value: float, name: str) -> float:
        v = float(value)
        if not math.isfinite(v) or v <= 0.0:
            raise ValueError(f"{name} must be finite and > 0, got {value!r}")
        return v

    # -- grid lifecycle ------------------------------------------------------
    def resize(self, width_m: float, height_m: float) -> None:
        """Resize the world extent, preserving the top-left overlap.

        New cells start UNKNOWN; counts/observed overlap is kept. The
        reachability seed is reset to the first preserved FREE cell (or None)
        and caches are invalidated.
        """
        width = self._positive(width_m, "width_m")
        height = self._positive(height_m, "height_m")
        rows, cols = grid_shape(width, height, self.cell_size)
        self.grid = resized_copy(self.grid, rows, cols, fill=UNKNOWN)
        self._counts = resized_copy(self._counts, rows, cols, fill=0)
        self._observed = resized_copy(self._observed, rows, cols, fill=False)
        self.width_m, self.height_m = width, height
        self._rows, self._cols = rows, cols
        free_cells = np.argwhere(self.grid == FREE)
        self._seed = ((int(free_cells[0][0]), int(free_cells[0][1]))
                      if free_cells.size else None)
        self._version += 1
        self._reach_cache = None
        self._reach_version = -1

    # -- coordinate helpers -------------------------------------------------
    def _in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self._rows and 0 <= c < self._cols

    def _clamp(self, r: int, c: int) -> tuple[int, int]:
        return (min(max(r, 0), self._rows - 1), min(max(c, 0), self._cols - 1))

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return int(y / self.cell_size), int(x / self.cell_size)

    def _cell_center(self, r: int, c: int) -> tuple[float, float]:
        return ((c + 0.5) * self.cell_size, (r + 0.5) * self.cell_size)

    # -- cell marking -------------------------------------------------------
    def _mark_free(self, r: int, c: int, force: bool = False) -> bool:
        """Count a free observation (or force-clear). Return True if changed."""
        if not self._in_bounds(r, c):
            return False
        if self._seed is None:
            self._seed = (r, c)
        cur = self.grid[r, c]
        if force:
            self._counts[r, c] = min(self._counts[r, c], 0)
            new = FREE
        else:
            self._counts[r, c] = max(self._counts[r, c] - 1, -4)
            new = OCCUPIED if self._counts[r, c] >= 1 else FREE
        if cur == new:
            return False
        self.grid[r, c] = new
        return True

    def _mark_occupied(self, r: int, c: int) -> bool:
        if not self._in_bounds(r, c):
            return False
        self._counts[r, c] = min(self._counts[r, c] + 2, 6)
        new = OCCUPIED if self._counts[r, c] >= 1 else FREE
        if self.grid[r, c] == new:
            return False
        self.grid[r, c] = new
        return True

    # -- sensor integration ---------------------------------------------------
    def update(self, pose: tuple[float, float, float],
               rays: list[tuple[float, float, bool]]) -> None:
        """Integrate a sensor sweep: free along rays, occupied at hit endpoints.

        ``pose = (x, y, theta)`` (theta unused: ray angles are absolute).
        ``rays`` items are ``(angle, distance, hit)``.
        """
        px, py = float(pose[0]), float(pose[1])
        r0, c0 = self._clamp(*self._world_to_cell(px, py))
        changed = self._mark_free(r0, c0, force=True)
        self._observed[r0, c0] = True
        for angle, distance, hit in rays:
            dist = max(0.0, min(float(distance), self.max_range))
            ex = px + dist * math.cos(angle)
            ey = py + dist * math.sin(angle)
            r1, c1 = self._clamp(*self._world_to_cell(ex, ey))
            cells = supercover_line(r0, c0, r1, c1)
            last = len(cells) - 1
            for idx, (r, c) in enumerate(cells):
                self._observed[r, c] = True
                if idx == last and hit:
                    changed |= self._mark_occupied(r, c)
                else:
                    changed |= self._mark_free(r, c)
        # The drone's own cell is always free (overrides a zero-range hit).
        changed |= self._mark_free(r0, c0, force=True)
        if changed:
            self._version += 1

    def _dilate8(self, mask: np.ndarray) -> np.ndarray:
        """Cells having at least one 8-adjacent True cell (vectorized)."""
        padded = np.pad(mask, 1, mode="constant", constant_values=False)
        out = np.zeros_like(mask)
        for dr in (0, 1, 2):
            for dc in (0, 1, 2):
                if dr == 1 and dc == 1:
                    continue
                out |= padded[dr:dr + self._rows, dc:dc + self._cols]
        return out

    def _frontier_mask(self) -> np.ndarray:
        free = self.grid == FREE
        unknown = self.grid == UNKNOWN
        return free & self._dilate8(unknown)

    def _bfs_component(self, mask: np.ndarray, visited: np.ndarray,
                       r: int, c: int) -> tuple[int, int, int]:
        """8-connected BFS over mask; return (size, sum_row, sum_col)."""
        visited[r, c] = True
        queue = deque([(r, c)])
        size = sum_r = sum_c = 0
        while queue:
            cr, cc = queue.popleft()
            size += 1
            sum_r += cr
            sum_c += cc
            for dr, dc in NEIGHBORS8:
                nr, nc = cr + dr, cc + dc
                if (self._in_bounds(nr, nc) and mask[nr, nc]
                        and not visited[nr, nc]):
                    visited[nr, nc] = True
                    queue.append((nr, nc))
        return size, sum_r, sum_c

    def frontiers(self) -> list[tuple[float, float]]:
        """World-meter centroids of frontier clusters, largest cluster first."""
        mask = self._frontier_mask()
        if not mask.any():
            return []
        visited = np.zeros_like(mask)
        clusters: list[tuple[int, tuple[float, float], int]] = []
        cs = self.cell_size
        for r, c in np.argwhere(mask):  # row-major -> deterministic seeds
            r, c = int(r), int(c)
            if visited[r, c]:
                continue
            size, sum_r, sum_c = self._bfs_component(mask, visited, r, c)
            if size >= MIN_FRONTIER_CLUSTER:
                cx = (sum_c / size + 0.5) * cs
                cy = (sum_r / size + 0.5) * cs
                clusters.append((size, (cx, cy), r * self._cols + c))
        clusters.sort(key=lambda item: (-item[0], item[2]))
        return [centroid for _, centroid, _ in clusters]

    def _resolve_goal(self, cell: tuple[int, int],
                      blocked: np.ndarray | None = None) -> tuple[int, int] | None:
        """Return a FREE goal cell (disk-clear when ``blocked`` given), ring search."""
        r, c = cell
        ok = (lambda rr, cc: self.grid[rr, cc] == FREE
              and (blocked is None or not blocked[rr, cc]))
        if self._in_bounds(r, c) and ok(r, c):
            return cell
        for radius in range(1, GOAL_SEARCH_RADIUS + 1):
            best: tuple[int, int] | None = None
            best_key: tuple[int, int, int] | None = None
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if max(abs(dr), abs(dc)) != radius:
                        continue
                    nr, nc = r + dr, c + dc
                    if not self._in_bounds(nr, nc) or not ok(nr, nc):
                        continue
                    key = (dr * dr + dc * dc, nr, nc)
                    if best_key is None or key < best_key:
                        best_key = key
                        best = (nr, nc)
            if best is not None:
                return best
        return None

    def _clearance_blocked(self) -> np.ndarray:
        """Cells whose CENTER disk overlaps a non-FREE cell (cached by version).

        A cell-center path must keep the drone footprint off obstacles; at fine
        resolutions every wall-adjacent cell is blocked, which is exactly the
        footprint constraint the vehicle has. Dilating obstacles by the true
        rectangle distance (not Chebyshev) preserves gaps wide enough to fly.
        """
        if self._clear_cache is not None and self._clear_version == self._version:
            return self._clear_cache
        blocked = np.zeros((self._rows, self._cols), dtype=bool)
        rad = self.clearance
        if rad > 0.0:
            cs = self.cell_size
            obstacles = self.grid == OCCUPIED
            k = int(math.ceil((rad + cs * 0.5) / cs))
            for dr in range(-k, k + 1):
                for dc in range(-k, k + 1):
                    gx = max(abs(dr) * cs - cs * 0.5, 0.0)
                    gy = max(abs(dc) * cs - cs * 0.5, 0.0)
                    if math.hypot(gx, gy) > rad + 1e-9:
                        continue
                    r_src0, r_src1 = max(0, -dr), min(self._rows, self._rows - dr)
                    c_src0, c_src1 = max(0, -dc), min(self._cols, self._cols - dc)
                    if r_src0 >= r_src1 or c_src0 >= c_src1:
                        continue
                    blocked[r_src0 + dr:r_src1 + dr, c_src0 + dc:c_src1 + dc] |= \
                        obstacles[r_src0:r_src1, c_src0:c_src1]
        self._clear_cache = blocked
        self._clear_version = self._version
        return blocked

    def _astar(self, start: tuple[int, int], goal: tuple[int, int],
               blocked: np.ndarray | None = None) -> list[tuple[int, int]] | None:
        if start == goal:
            return [start]
        rows, cols = self._rows, self._cols
        grid = self.grid
        g = np.full((rows, cols), math.inf, dtype=float)
        came = np.full((rows, cols), -1, dtype=np.int64)
        closed = np.zeros((rows, cols), dtype=bool)
        sr, sc = start
        gr, gc = goal
        g[sr, sc] = 0.0
        h0 = _octile(sr, sc, gr, gc)
        heap: list[tuple[float, float, int, int]] = [(h0, h0, sr, sc)]
        while heap:
            _, _, r, c = heapq.heappop(heap)
            if closed[r, c]:
                continue
            closed[r, c] = True
            if (r, c) == goal:
                return self._reconstruct(came, start, goal)
            for dr, dc, cost in MOVES:
                nr, nc = r + dr, c + dc
                if not self._in_bounds(nr, nc) or closed[nr, nc]:
                    continue
                if grid[nr, nc] != FREE:
                    continue
                if blocked is not None and blocked[nr, nc] and (nr, nc) != start:
                    continue
                if dr != 0 and dc != 0:
                    # No corner-cutting through occupied diagonals.
                    if grid[r + dr, c] != FREE or grid[r, c + dc] != FREE:
                        continue
                ng = g[r, c] + cost
                if ng < g[nr, nc]:
                    g[nr, nc] = ng
                    came[nr, nc] = r * cols + c
                    hh = _octile(nr, nc, gr, gc)
                    heapq.heappush(heap, (ng + hh, hh, nr, nc))
        return None

    def _reconstruct(self, came: np.ndarray, start: tuple[int, int],
                     goal: tuple[int, int]) -> list[tuple[int, int]]:
        path = [goal]
        cur = goal
        while cur != start:
            parent = int(came[cur])
            if parent < 0:
                return [goal]  # defensive; should not happen
            cur = (parent // self._cols, parent % self._cols)
            path.append(cur)
        path.reverse()
        return path

    def _line_free(self, p0: tuple[float, float], p1: tuple[float, float]) -> bool:
        """True if the drone disk stays on FREE cells along the whole segment.

        Fine-marches the true geometry (cell-index supercover misses corner
        clips near cell boundaries) and checks every cell the disk overlaps.
        """
        cs = self.cell_size
        r = self.clearance
        dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        steps = max(1, math.ceil(dist / (cs * 0.5)))
        for i in range(steps + 1):
            x = p0[0] + (p1[0] - p0[0]) * i / steps
            y = p0[1] + (p1[1] - p0[1]) * i / steps
            r0, r1 = math.floor((y - r) / cs), math.floor((y + r) / cs)
            c0, c1 = math.floor((x - r) / cs), math.floor((x + r) / cs)
            for rr in range(r0, r1 + 1):
                for cc in range(c0, c1 + 1):
                    if not self._in_bounds(rr, cc) or self.grid[rr, cc] != FREE:
                        return False
        return True

    def _smooth(self, waypoints: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """Greedy line-of-sight smoothing: skip to the farthest visible point."""
        n = len(waypoints)
        if n <= 2:
            return waypoints
        out = [waypoints[0]]
        i = 0
        while i < n - 1:
            j = n - 1
            while j > i + 1 and not self._line_free(waypoints[i], waypoints[j]):
                j -= 1
            out.append(waypoints[j])
            i = j
        return out

    def plan_path(self, start: tuple[float, float],
                  goal: tuple[float, float]) -> list[tuple[float, float]] | None:
        """Footprint-clear A* over known-FREE cells with LOS smoothing.

        Cells whose center disk overlaps an OCCUPIED cell are not expanded, so
        paths are flyable by the vehicle (not just by a point robot). If the
        drone currently sits closer to a wall than its radius (center cell
        blocked), planning starts from the nearest clear cell and the actual
        position is kept as the first waypoint.
        """
        start_cell = self._world_to_cell(*start)
        if not self._in_bounds(*start_cell) or self.grid[start_cell] != FREE:
            return None
        blocked = self._clearance_blocked()
        plan_start = start_cell
        if blocked[start_cell]:
            clear = self._resolve_goal(start_cell, blocked)
            if clear is None:
                return None
            plan_start = clear
        goal_cell_raw = self._world_to_cell(*goal)
        goal_cell = self._resolve_goal(goal_cell_raw, blocked)
        if goal_cell is None:
            return None
        cells = self._astar(plan_start, goal_cell, blocked)
        if cells is None:
            return None
        start_pt = (float(start[0]), float(start[1]))
        if (goal_cell == goal_cell_raw and not blocked[goal_cell]
                and self.grid[goal_cell] == FREE):
            goal_pt = (float(goal[0]), float(goal[1]))
        else:
            goal_pt = self._cell_center(*goal_cell)
        if len(cells) == 1:
            return self._smooth([start_pt, goal_pt])
        waypoints = [self._cell_center(r, c) for r, c in cells]
        waypoints[0] = start_pt
        waypoints[-1] = goal_pt
        return self._smooth(waypoints)

    def observed_mask(self) -> np.ndarray:
        return self._observed.copy()

    def explored_fraction(self) -> float:
        total = self._rows * self._cols
        if total == 0:
            return 0.0
        return float(np.count_nonzero(self._observed)) / total

    def is_known_free(self, x: float, y: float) -> bool:
        if x < 0.0 or y < 0.0 or x >= self.width_m or y >= self.height_m:
            return False
        r, c = self._world_to_cell(x, y)
        return self.grid[r, c] == FREE

    def _flood_reachable(self) -> np.ndarray:
        mask = np.zeros((self._rows, self._cols), dtype=bool)
        if self._seed is None:
            return mask
        r, c = self._seed
        if not self._in_bounds(r, c) or self.grid[r, c] != FREE:
            return mask
        mask[r, c] = True
        queue = deque([(r, c)])
        while queue:
            cr, cc = queue.popleft()
            for dr, dc in NEIGHBORS8:
                nr, nc = cr + dr, cc + dc
                if (self._in_bounds(nr, nc) and not mask[nr, nc]
                        and self.grid[nr, nc] == FREE):
                    mask[nr, nc] = True
                    queue.append((nr, nc))
        return mask

    def reachable_free_mask(self) -> np.ndarray:
        """FREE cells reachable from the spawn seed; cached until grid changes."""
        if self._reach_cache is not None and self._reach_version == self._version:
            return self._reach_cache.copy()
        mask = self._flood_reachable()
        self._reach_cache = mask
        self._reach_version = self._version
        return mask.copy()