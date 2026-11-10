"""Mission monitor core (ROS-independent: numpy + stdlib).

Ported from the validated NIDAR SAR sim ``sar/mission/monitor.py``
(commit c090950). ``update(now)`` runs at ~1 Hz and caches all expensive
state; ``report()`` is cheap and safe to call every tick. Findings are cached
by text so identical findings keep their original timestamp.

Duck-typed inputs (no cross-package imports):
- mapper: ``.grid`` (int8, UNKNOWN=-1/FREE=0/OCCUPIED=100),
  ``.explored_fraction()``, ``.observed_mask()``, ``.reachable_free_mask()``;
  optional ``.frontiers()`` is used for remaining areas when present.
- tracker: ``.tracks`` -> list of record dicts with keys
  ``position_x, position_y, confidence, state`` (state 'uncertain'/'confirmed').
- world_regions: list of dicts ``{region_id, center:(x,y),
  bounds:(x0,y0,x1,y1), accessible:bool}``. ``center`` is world meters;
  ``bounds`` are INCLUSIVE grid-cell indices with x=column, y=row, matching
  the workspace ``grid[row, col]`` convention.
"""
from __future__ import annotations

import math
from typing import Any, Protocol, Sequence

import numpy as np

FREE = 0  # occupancy cell value (workspace global constraint)

EXPLORED_THRESHOLD = 0.9
PARTIAL_THRESHOLD = 0.15
INACCESSIBLE_OBSERVED_MAX = 0.05
COVERAGE_MILESTONES = (25, 50, 75, 100)

STATUS_UNEXPLORED = "unexplored"
STATUS_PARTIAL = "partial"
STATUS_EXPLORED = "explored"
STATUS_INACCESSIBLE = "inaccessible"


class MapperLike(Protocol):
    """Minimal mapper contract consumed by the monitor."""

    grid: np.ndarray

    def explored_fraction(self) -> float: ...
    def observed_mask(self) -> np.ndarray: ...
    def reachable_free_mask(self) -> np.ndarray: ...


class TrackerLike(Protocol):
    """Minimal tracker contract consumed by the monitor."""

    tracks: list[dict[str, Any]]


class MissionMonitor:
    """Tracks region exploration status and produces priority-sorted findings."""

    def __init__(
        self,
        world_regions: Sequence[dict[str, Any]],
        mapper: MapperLike,
        tracker: TrackerLike,
    ) -> None:
        self._region_defs = [_validate_region(region) for region in world_regions]
        self.mapper = mapper
        self.tracker = tracker
        self._last_now = 0.0
        self._explored_pct = 0.0
        self._accessible_pct = 0.0
        self._remaining_areas: list[tuple[float, float]] = []
        self._findings: dict[str, dict[str, Any]] = {}  # text -> finding
        self._milestones_hit: set[int] = set()
        self._inaccessible_seen: set[int] = set()
        self._complete_announced = False
        self._region_reports = [
            {**region, "status": STATUS_UNEXPLORED, "explored_frac": 0.0}
            for region in self._region_defs
        ]

    # ------------------------------------------------------------------ API

    def update(self, now: float) -> None:
        """Recompute coverage, region status, remaining areas, and findings."""
        now = float(now)
        observed = np.asarray(self.mapper.observed_mask(), dtype=bool)
        reachable = np.asarray(self.mapper.reachable_free_mask(), dtype=bool)
        grid = np.asarray(self.mapper.grid)
        self._region_reports = self._compute_regions(observed, reachable)
        self._explored_pct = min(
            100.0, max(0.0, float(self.mapper.explored_fraction()) * 100.0)
        )
        self._accessible_pct = _accessible_pct(observed, reachable, grid)
        self._remaining_areas = self._compute_remaining_areas()
        self._refresh_findings(now)
        self._last_now = now

    def report(self) -> dict[str, Any]:
        """Cheap snapshot dict (exact report schema, fresh copies)."""
        tracks = [dict(track) for track in self.tracker.tracks]
        count, confidence = _survivor_count(tracks)
        findings = sorted(
            self._findings.values(), key=lambda f: (f["priority"], f["timestamp"])
        )
        return {
            "explored_pct": self._explored_pct,
            "accessible_explored_pct": self._accessible_pct,
            "survivor_count": count,
            "count_confidence": confidence,
            "regions": [dict(region) for region in self._region_reports],
            "findings": [dict(finding) for finding in findings],
            "tracks": tracks,
            "timestamp": self._last_now,
            "remaining_areas": list(self._remaining_areas),
        }

    # --------------------------------------------------------- region status

    def _compute_regions(
        self, observed: np.ndarray, reachable: np.ndarray
    ) -> list[dict[str, Any]]:
        rows, cols = observed.shape
        stats: list[tuple[float, bool]] = []
        for region in self._region_defs:
            x0, y0, x1, y1 = region["bounds"]
            c0, r0 = max(0, x0), max(0, y0)
            c1, r1 = min(cols - 1, x1), min(rows - 1, y1)
            if r1 < r0 or c1 < c0:
                stats.append((0.0, False))
                continue
            region_observed = observed[r0 : r1 + 1, c0 : c1 + 1]
            region_reachable = reachable[r0 : r1 + 1, c0 : c1 + 1]
            stats.append((float(region_observed.mean()), bool(region_reachable.any())))

        any_explored = any(frac >= EXPLORED_THRESHOLD for frac, _ in stats)
        updated: list[dict[str, Any]] = []
        for region, (frac, has_reachable) in zip(self._region_defs, stats):
            status = _coverage_status(frac)
            if not region["accessible"]:
                status = STATUS_INACCESSIBLE
            elif (
                status != STATUS_EXPLORED
                and frac < INACCESSIBLE_OBSERVED_MAX
                and not has_reachable
                and any_explored
            ):
                status = STATUS_INACCESSIBLE
            updated.append({**region, "status": status, "explored_frac": frac})
        return updated

    def _compute_remaining_areas(self) -> list[tuple[float, float]]:
        frontiers = getattr(self.mapper, "frontiers", None)
        if callable(frontiers):
            return [tuple(point) for point in frontiers()]
        # Minimal duck type has no frontiers(): approximate remaining areas by
        # the centers of accessible regions that are not yet explored.
        return [
            tuple(region["center"])
            for region in self._region_reports
            if region["accessible"]
            and region["status"] in (STATUS_UNEXPLORED, STATUS_PARTIAL)
        ]

    # -------------------------------------------------------------- findings

    def _refresh_findings(self, now: float) -> None:
        for track in self.tracker.tracks:
            x, y = float(track["position_x"]), float(track["position_y"])
            location = f"({x:.1f}, {y:.1f})"
            label = self._region_label(x, y)
            if track["state"] == "confirmed":
                self._add_finding(
                    1, f"Survivor confirmed in {label} {location} - high confidence", now
                )
            else:
                self._add_finding(
                    2,
                    f"Uncertain survivor indication in {label} {location} - needs verification",
                    now,
                )

        for region in self._region_reports:
            if (
                region["status"] == STATUS_INACCESSIBLE
                and region["region_id"] not in self._inaccessible_seen
            ):
                self._inaccessible_seen.add(region["region_id"])
                self._add_finding(
                    3,
                    f"Region {region['region_id']} is inaccessible - sealed off, drone cannot reach it",
                    now,
                )

        for milestone in COVERAGE_MILESTONES:
            if self._explored_pct >= milestone and milestone not in self._milestones_hit:
                self._milestones_hit.add(milestone)
                self._add_finding(
                    4, f"Coverage milestone: {milestone}% of the area mapped", now
                )

        if (
            not self._complete_announced
            and not self._remaining_areas
            and self._explored_pct > 0.0
        ):
            self._complete_announced = True
            count, _ = _survivor_count(self.tracker.tracks)
            confirmed = sum(1 for t in self.tracker.tracks if t["state"] == "confirmed")
            inaccessible = sum(
                1 for r in self._region_reports if r["status"] == STATUS_INACCESSIBLE
            )
            self._add_finding(
                5,
                f"Mission complete: {self._explored_pct:.0f}% of area mapped, "
                f"{count} survivor(s) estimated ({confirmed} confirmed), "
                f"{inaccessible} inaccessible region(s)",
                now,
            )

    def _add_finding(self, priority: int, text: str, now: float) -> None:
        if text not in self._findings:
            self._findings[text] = {"priority": priority, "text": text, "timestamp": now}

    def _region_label(self, x: float, y: float) -> str:
        if not self._region_defs:
            return "an unknown area"
        nearest = min(
            self._region_defs,
            key=lambda r: math.hypot(r["center"][0] - x, r["center"][1] - y),
        )
        return f"Region {nearest['region_id']}"


# ------------------------------------------------------------------ helpers


def _survivor_count(tracks: Sequence[dict[str, Any]]) -> tuple[int, float]:
    """count = confirmed + round(uncertain confidence mass); confidence 0..1."""
    if not tracks:
        return 0, 0.0
    confirmed = [t for t in tracks if t["state"] == "confirmed"]
    uncertain_mass = sum(
        float(t["confidence"]) for t in tracks if t["state"] == "uncertain"
    )
    count = len(confirmed) + int(round(uncertain_mass))
    mean_conf = sum(float(t["confidence"]) for t in tracks) / len(tracks)
    confidence = (len(confirmed) / len(tracks)) * mean_conf
    return count, min(1.0, max(0.0, confidence))


def _accessible_pct(
    observed: np.ndarray, reachable: np.ndarray, grid: np.ndarray
) -> float:
    observed_free = np.count_nonzero(observed & (grid == FREE))
    reachable_free = np.count_nonzero(reachable)
    if reachable_free == 0:
        return 0.0
    return min(100.0, observed_free / reachable_free * 100.0)


def _coverage_status(explored_frac: float) -> str:
    if explored_frac >= EXPLORED_THRESHOLD:
        return STATUS_EXPLORED
    if explored_frac >= PARTIAL_THRESHOLD:
        return STATUS_PARTIAL
    return STATUS_UNEXPLORED


def _validate_region(region: dict[str, Any]) -> dict[str, Any]:
    required = ("region_id", "center", "bounds", "accessible")
    if not isinstance(region, dict):
        raise ValueError(f"region must be a dict, got {region!r}")
    missing = [key for key in required if key not in region]
    if missing:
        raise ValueError(f"region {region.get('region_id')!r} missing keys: {missing}")
    center, bounds = region["center"], region["bounds"]
    if len(center) != 2 or len(bounds) != 4:
        raise ValueError(
            f"region {region['region_id']!r}: center must be (x, y) and bounds "
            f"(x0, y0, x1, y1), got {center!r}, {bounds!r}"
        )
    x0, y0, x1, y1 = (float(v) for v in bounds)
    if x1 < x0 or y1 < y0:
        raise ValueError(f"region {region['region_id']!r}: inverted bounds {bounds!r}")
    return {
        "region_id": region["region_id"],
        "center": (float(center[0]), float(center[1])),
        "bounds": (int(x0), int(y0), int(x1), int(y1)),
        "accessible": bool(region["accessible"]),
    }