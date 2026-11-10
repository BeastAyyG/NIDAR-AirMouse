"""Offline validation metrics for the AirMouse autonomy workspace.

numpy + stdlib only; NO rclpy/ROS imports. Every metric compares what the
autonomy produced (sensor-built occupancy grid, flown path, survivor tracks)
against ground truth, i.e. metrics and ground truth exist on the validation
side only — they are never fed back into the pipeline.

Grid convention (shared with airmouse_mapping): meters, x right, y DOWN,
``grid[row, col]`` with ``row = int(y / cell_size)`` and
``col = int(x / cell_size)``. Cell values: ``UNKNOWN = -1``, ``FREE = 0``,
``OCCUPIED = 100``.
"""
from __future__ import annotations

import math

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100

_MARCH_STEP_FRACTION = 0.5  # samples per cell along a collision-check segment


# ------------------------------------------------------------------ helpers
def _as_grid(grid, name: str) -> np.ndarray:
    arr = np.asarray(grid)
    if arr.ndim != 2 or arr.size == 0:
        raise ValueError(f"{name} must be a non-empty 2-D array, got shape {arr.shape}")
    return arr


def _as_grid_pair(grid, truth_grid) -> tuple[np.ndarray, np.ndarray]:
    pred = _as_grid(grid, "grid")
    truth = _as_grid(truth_grid, "truth_grid")
    if pred.shape != truth.shape:
        raise ValueError(
            f"grid shape {pred.shape} does not match truth_grid shape {truth.shape}"
        )
    return pred, truth


def _positive(value, name: str) -> float:
    v = float(value)
    if not math.isfinite(v) or v <= 0.0:
        raise ValueError(f"{name} must be finite and > 0, got {value!r}")
    return v


# ------------------------------------------------------- occupancy agreement
def occupancy_counts(grid, truth_grid) -> dict:
    """Confusion counts for the OCCUPIED positive class.

    Cells whose ground truth is UNKNOWN are excluded (never-observed / outside
    the world model); a predicted OCCUPIED cell there is not scored either way.

    Returns ``{"tp", "fp", "fn", "precision", "recall"}`` where ``tp`` means
    predicted OCCUPIED and truth OCCUPIED, ``fp`` predicted OCCUPIED but truth
    FREE, ``fn`` truth OCCUPIED but predicted FREE-or-UNKNOWN. Precision and
    recall are 0.0 when their denominator is 0.
    """
    pred, truth = _as_grid_pair(grid, truth_grid)
    known = truth != UNKNOWN
    predicted_occupied = (pred == OCCUPIED) & known
    actual_occupied = (truth == OCCUPIED) & known

    tp = int(np.count_nonzero(predicted_occupied & actual_occupied))
    fp = int(np.count_nonzero(predicted_occupied & ~actual_occupied))
    fn = int(np.count_nonzero(~predicted_occupied & actual_occupied))

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": float(precision),
        "recall": float(recall),
    }


def map_f1(grid, truth_grid) -> float:
    """F1 score of the mapped OCCUPIED class against ground truth.

    Positive class = OCCUPIED (100). Only cells where the truth is not UNKNOWN
    are evaluated, so unmodeled world regions cannot inflate or deflate the
    score. Returns 1.0 when nothing is occupied on either side (vacuous match);
    0.0 when there is nothing to score but one-sided predictions exist.
    """
    counts = occupancy_counts(grid, truth_grid)
    tp, fp, fn = counts["tp"], counts["fp"], counts["fn"]
    denominator = 2 * tp + fp + fn
    if denominator == 0:
        return 1.0
    return float(2 * tp / denominator)


# ------------------------------------------------------------------ collisions
def _segment(segment) -> tuple[float, float, float, float]:
    """Normalize a segment into (x0, y0, x1, y1)."""
    try:
        size = len(segment)
    except TypeError as exc:
        raise ValueError(f"segment must be a pair of points, got {segment!r}") from exc
    if isinstance(segment, (str, bytes)) or size != 2:
        raise ValueError(f"segment must be ((x0, y0), (x1, y1)), got {segment!r}")
    p0, p1 = segment
    if len(p0) != 2 or len(p1) != 2:
        raise ValueError(f"segment points must be (x, y), got {segment!r}")
    values = (float(p0[0]), float(p0[1]), float(p1[0]), float(p1[1]))
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"segment contains non-finite coordinates: {segment!r}")
    return values


def collisions_from_truth(segments, truth_grid, radius: float,
                          cell_size: float) -> int:
    """Count flown-segment intersections with ground-truth obstacles.

    Each segment is fine-marched at half-cell steps; every ground-truth
    OCCUPIED cell overlapped by the drone disk (``radius`` around the sample
    point) counts as one collision event. Events are de-duplicated per
    (segment, cell) pair so a slow pass along a wall is one incident per cell,
    not one per march sample.

    ``segments`` is an iterable of ``((x0, y0), (x1, y1))`` world-meter pairs
    (e.g. consecutive entries of a flown ``path_history``).
    """
    truth = _as_grid(truth_grid, "truth_grid")
    r = float(radius)
    if not math.isfinite(r) or r < 0.0:
        raise ValueError(f"radius must be finite and >= 0, got {radius!r}")
    cs = _positive(cell_size, "cell_size")
    rows, cols = truth.shape
    step = cs * _MARCH_STEP_FRACTION
    occupied = (truth == OCCUPIED).ravel()

    events: set[tuple[int, int, int]] = set()
    for seg_index, segment in enumerate(segments or []):
        x0, y0, x1, y1 = _segment(segment)
        distance = math.hypot(x1 - x0, y1 - y0)
        n_samples = max(1, int(math.ceil(distance / step)))
        fractions = np.linspace(0.0, 1.0, n_samples + 1)
        xs = x0 + (x1 - x0) * fractions
        ys = y0 + (y1 - y0) * fractions
        r0 = np.floor((ys - r) / cs).astype(np.int64)
        r1 = np.floor((ys + r) / cs).astype(np.int64)
        c0 = np.floor((xs - r) / cs).astype(np.int64)
        c1 = np.floor((xs + r) / cs).astype(np.int64)

        keys_parts = []
        for dr in range(int((r1 - r0).max()) + 1):
            cell_rows = r0 + dr
            in_rows = cell_rows <= r1
            for dc in range(int((c1 - c0).max()) + 1):
                cell_cols = c0 + dc
                keep = (in_rows & (cell_cols <= c1)
                        & (cell_rows >= 0) & (cell_rows < rows)
                        & (cell_cols >= 0) & (cell_cols < cols))
                if np.any(keep):
                    keys_parts.append(cell_rows[keep] * cols + cell_cols[keep])
        if not keys_parts:
            continue
        keys = np.unique(np.concatenate(keys_parts))
        for key in keys[occupied[keys]].tolist():
            events.add((seg_index, int(key) // cols, int(key) % cols))
    return len(events)


# -------------------------------------------------------------- survivors
def survivor_match_count(truth_positions, track_records, merge_radius: float) -> int:
    """Greedy nearest one-to-one matching of truth survivors to tracks.

    ``truth_positions`` is a sequence of ``(x, y)`` world-meter pairs;
    ``track_records`` is a sequence of track dicts with ``position_x`` and
    ``position_y``. Pair candidates within ``merge_radius`` are matched
    closest-first, and each truth position and each track may be used once —
    so repeated detections merged into one track cannot inflate the count.
    """
    radius = float(merge_radius)
    if not math.isfinite(radius) or radius < 0.0:
        raise ValueError(f"merge_radius must be finite and >= 0, got {merge_radius!r}")

    truths = []
    for index, position in enumerate(truth_positions or []):
        try:
            x, y = position[0], position[1]
        except (TypeError, IndexError) as exc:
            raise ValueError(
                f"truth_positions[{index}] must be (x, y), got {position!r}"
            ) from exc
        truths.append((float(x), float(y)))

    tracks = []
    for index, record in enumerate(track_records or []):
        try:
            tracks.append((float(record["position_x"]), float(record["position_y"])))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"track_records[{index}] must provide numeric position_x/position_y, "
                f"got {record!r}"
            ) from exc

    candidates = []
    for truth_index, (tx, ty) in enumerate(truths):
        for track_index, (px, py) in enumerate(tracks):
            distance = math.hypot(tx - px, ty - py)
            if distance <= radius:
                candidates.append((distance, truth_index, track_index))
    candidates.sort()

    used_truth: set[int] = set()
    used_tracks: set[int] = set()
    matched = 0
    for _, truth_index, track_index in candidates:
        if truth_index in used_truth or track_index in used_tracks:
            continue
        used_truth.add(truth_index)
        used_tracks.add(track_index)
        matched += 1
    return matched
