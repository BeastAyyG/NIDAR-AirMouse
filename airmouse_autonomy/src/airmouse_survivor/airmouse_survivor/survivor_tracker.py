"""Survivor track management core (ROS-independent: stdlib only).

Ported from the validated NIDAR SAR sim ``sar/detection/tracker.py``
(commit c090950). Detections merge into the nearest track within
``merge_radius_m`` (spatial-gate dedup); otherwise they spawn an UNCERTAIN
track. Confidence aggregates as independent evidence:
``conf <- 1 - (1 - conf) * (1 - det_conf)``. A track is promoted to CONFIRMED
when ``confidence >= confirm_confidence`` AND ``sightings >= confirm_sightings``.
Unmatched UNCERTAIN tracks decay linearly (``decay_per_sec * dt``) and are
dropped below ``drop_confidence``; CONFIRMED tracks are permanent.

Public track records are fresh dicts with EXACTLY these keys:
``survivor_id, detected, position_x, position_y, confidence, timestamp,
state, sightings`` where ``state`` is ``'uncertain'`` or ``'confirmed'``.
``detected`` is True when the track was observed in the latest update.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Sequence

UNCERTAIN = "uncertain"
CONFIRMED = "confirmed"

# Defaults mirror airmouse_autonomy/config/survivor.yaml.
DEFAULT_PARAMS: dict[str, float] = {
    "merge_radius_m": 1.2,
    "confirm_confidence": 0.85,
    "confirm_sightings": 3,
    "decay_per_sec": 0.02,
    "drop_confidence": 0.12,
}

Detection = tuple[float, float, float, float]  # (x, y, confidence, timestamp)


@dataclass
class _Track:
    """Internal mutable track state; never exposed directly."""

    track_id: int
    x: float
    y: float
    confidence: float
    state: str
    sightings: int
    first_seen: float
    last_seen: float


class SurvivorTracker:
    """Greedy spatial-gating tracker.

    ``tracks`` and ``uncertain_tracks()`` return fresh record-dict snapshots,
    so callers cannot mutate internal state.
    """

    def __init__(self, params_dict: dict[str, Any] | None = None) -> None:
        params = {**DEFAULT_PARAMS, **(params_dict or {})}
        self._merge_radius = _non_negative_float(params, "merge_radius_m")
        self._confirm_confidence = _unit_float(params, "confirm_confidence")
        self._confirm_sightings = _positive_int(params, "confirm_sightings")
        self._decay_per_sec = _non_negative_float(params, "decay_per_sec")
        self._drop_confidence = _unit_float(params, "drop_confidence")
        self._track_list: list[_Track] = []
        self._next_id = 0
        self._last_update: float | None = None

    @property
    def tracks(self) -> list[dict[str, Any]]:
        """Live tracks as record-dict snapshots (exact roadmap schema)."""
        return [self._record(track) for track in self._track_list]

    def update(self, detections: Iterable[Sequence[float]], now: float) -> None:
        """Gate/merge detections, promote, then decay unmatched uncertain tracks.

        ``detections`` is a list of ``(x, y, confidence, timestamp)`` tuples in
        world meters; ``now`` is the authoritative update clock (seconds).
        """
        now = _as_float(now, "now")
        parsed = [_parse_detection(detection) for detection in detections]
        elapsed = (
            0.0 if self._last_update is None else max(0.0, now - self._last_update)
        )
        self._last_update = now
        matched_ids: set[int] = set()
        for x, y, confidence, _timestamp in parsed:
            track = self._nearest_within_gate(x, y)
            if track is None:
                track = self._spawn(x, y, confidence, now)
            else:
                self._merge(track, x, y, confidence, now)
            matched_ids.add(track.track_id)
            self._maybe_promote(track)
        self._decay_unmatched(elapsed, matched_ids)

    def uncertain_tracks(self) -> list[dict[str, Any]]:
        """Record-dict snapshots of UNCERTAIN tracks (verification candidates)."""
        return [self._record(t) for t in self._track_list if t.state == UNCERTAIN]

    def survivor_count(self) -> tuple[int, float]:
        """Best estimate ``(count, confidence)``.

        count      = n_confirmed + round(sum of UNCERTAIN confidences)
                     (built-in round, half-to-even)
        confidence = (n_confirmed / n_tracks) * mean(track.confidence),
                     clamped to [0, 1]; 0.0 when there are no tracks.
        """
        if not self._track_list:
            return 0, 0.0
        confirmed = [t for t in self._track_list if t.state == CONFIRMED]
        uncertain_mass = sum(
            t.confidence for t in self._track_list if t.state == UNCERTAIN
        )
        count = len(confirmed) + int(round(uncertain_mass))
        mean_conf = sum(t.confidence for t in self._track_list) / len(self._track_list)
        confidence = (len(confirmed) / len(self._track_list)) * mean_conf
        return count, min(1.0, max(0.0, confidence))

    # ------------------------------------------------------------- internals

    def _record(self, track: _Track) -> dict[str, Any]:
        return {
            "survivor_id": track.track_id,
            "detected": (
                self._last_update is not None and track.last_seen == self._last_update
            ),
            "position_x": track.x,
            "position_y": track.y,
            "confidence": track.confidence,
            "timestamp": track.last_seen,
            "state": track.state,
            "sightings": track.sightings,
        }

    def _nearest_within_gate(self, x: float, y: float) -> _Track | None:
        best: _Track | None = None
        best_dist = math.inf
        for track in self._track_list:
            dist = math.hypot(track.x - x, track.y - y)
            if dist <= self._merge_radius and dist < best_dist:
                best, best_dist = track, dist
        return best

    def _spawn(self, x: float, y: float, confidence: float, now: float) -> _Track:
        track = _Track(
            track_id=self._next_id,
            x=x,
            y=y,
            confidence=confidence,
            state=UNCERTAIN,
            sightings=1,
            first_seen=now,
            last_seen=now,
        )
        self._next_id += 1
        self._track_list.append(track)
        return track

    def _merge(
        self, track: _Track, x: float, y: float, confidence: float, now: float
    ) -> None:
        # Confidence-weighted running average; old weight = aggregate confidence.
        weight_sum = track.confidence + confidence
        if weight_sum > 0.0:
            track.x = (track.x * track.confidence + x * confidence) / weight_sum
            track.y = (track.y * track.confidence + y * confidence) / weight_sum
        track.confidence = 1.0 - (1.0 - track.confidence) * (1.0 - confidence)
        track.sightings += 1
        track.last_seen = now

    def _maybe_promote(self, track: _Track) -> None:
        if track.state == CONFIRMED:
            return
        if (
            track.confidence >= self._confirm_confidence
            and track.sightings >= self._confirm_sightings
        ):
            track.state = CONFIRMED

    def _decay_unmatched(self, elapsed: float, matched_ids: set[int]) -> None:
        if elapsed <= 0.0:
            return
        kept: list[_Track] = []
        for track in self._track_list:
            if track.state == UNCERTAIN and track.track_id not in matched_ids:
                track.confidence -= self._decay_per_sec * elapsed
                if track.confidence < self._drop_confidence:
                    continue
            kept.append(track)
        self._track_list = kept


# ------------------------------------------------------------------ helpers


def _parse_detection(detection: Sequence[float]) -> Detection:
    try:
        size = len(detection)
    except TypeError as exc:
        raise ValueError(f"detection must be a 4-element sequence, got {detection!r}") from exc
    if isinstance(detection, (str, bytes)) or size != 4:
        raise ValueError(
            f"detection must be a 4-element (x, y, confidence, timestamp) sequence, "
            f"got {detection!r}"
        )
    x, y, confidence, timestamp = (
        _as_float(value, "detection field") for value in detection
    )
    return x, y, _clip_confidence(confidence), timestamp


def _clip_confidence(confidence: float) -> float:
    return min(1.0, max(0.0, confidence))


def _as_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, got {value!r}")
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number, got {value!r}") from exc


def _non_negative_float(params: dict[str, Any], key: str) -> float:
    value = _as_float(params[key], key)
    if value < 0.0:
        raise ValueError(f"{key} must be >= 0, got {value}")
    return value


def _unit_float(params: dict[str, Any], key: str) -> float:
    value = _as_float(params[key], key)
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"{key} must be in [0, 1], got {value}")
    return value


def _positive_int(params: dict[str, Any], key: str) -> int:
    value = params[key]
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{key} must be an integer >= 1, got {value!r}")
    return value