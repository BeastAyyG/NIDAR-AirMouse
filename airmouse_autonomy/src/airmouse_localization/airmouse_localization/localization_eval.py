"""Estimated-vs-truth localization error metrics (ROS-independent core).

numpy + stdlib only; NO rclpy/ROS imports. Ground truth is a DEVELOPMENT
reference: it is consumed only by this evaluator, never by the autonomy
pipeline (mapping / exploration / navigation / mission).

Series format: ``est`` and ``truth`` are lists of ``(t, x, y)`` samples in
seconds / world meters (the workspace y-DOWN frame). Timestamps must be
strictly increasing in both series. The estimate is linearly interpolated
onto the truth timestamps with :func:`numpy.interp` (values outside the
estimate's time span clamp to the nearest estimate endpoint, matching the
behavior of a stale/low-rate estimator at replay boundaries).
"""
from __future__ import annotations

import math

import numpy as np

_REQUIRED_FIELDS = 3  # (t, x, y)


def _as_series(samples, name: str) -> np.ndarray:
    """Validate a ``(t, x, y)`` sample sequence and return an (N, 3) array."""
    if samples is None:
        raise ValueError(f"{name} must be a sequence of (t, x, y) samples, got None")
    rows = []
    for index, sample in enumerate(samples):
        try:
            size = len(sample)
        except TypeError as exc:
            raise ValueError(
                f"{name}[{index}] must be a (t, x, y) sequence, got {sample!r}"
            ) from exc
        if isinstance(sample, (str, bytes)) or size != _REQUIRED_FIELDS:
            raise ValueError(
                f"{name}[{index}] must have exactly {_REQUIRED_FIELDS} fields "
                f"(t, x, y), got {sample!r}"
            )
        rows.append([float(value) for value in sample])
    if len(rows) < 2:
        raise ValueError(f"{name} needs at least 2 samples, got {len(rows)}")
    arr = np.asarray(rows, dtype=float)
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} contains non-finite values")
    times = arr[:, 0]
    if np.any(np.diff(times) <= 0.0):
        raise ValueError(f"{name} timestamps must be strictly increasing")
    return arr


def _interpolate(times: np.ndarray, series: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Linearly interpolate a (N, 3) series' x/y onto ``times``."""
    x = np.interp(times, series[:, 0], series[:, 1])
    y = np.interp(times, series[:, 0], series[:, 2])
    return x, y


def compute_errors(est, truth, revisit_times=None) -> dict:
    """Compare an estimated trajectory against ground truth.

    Parameters
    ----------
    est, truth : sequence of (t, x, y)
        Pose samples in seconds / world meters; strictly increasing times.
    revisit_times : sequence of float, optional
        Timestamps of loop-closure revisits (e.g. the second pass over the
        entry area). When omitted, the last truth sample is used as the
        revisit proxy; when given, the mean position error across the given
        revisit timestamps is reported.

    Returns
    -------
    dict with keys ``rmse`` (sqrt(mean(dx^2 + dy^2)) over all truth samples),
    ``max`` (largest sample distance) and ``loop_closure_error`` (position
    error at the revisit proxy / revisit times), all floats in meters.
    """
    est_arr = _as_series(est, "est")
    truth_arr = _as_series(truth, "truth")

    truth_times = truth_arr[:, 0]
    est_x, est_y = _interpolate(truth_times, est_arr)
    dx = est_x - truth_arr[:, 1]
    dy = est_y - truth_arr[:, 2]
    distances = np.hypot(dx, dy)

    if revisit_times is None:
        loop_closure_error = float(distances[-1])
    else:
        times = np.asarray([float(t) for t in revisit_times], dtype=float)
        if times.size == 0:
            raise ValueError("revisit_times must contain at least one timestamp")
        if not np.all(np.isfinite(times)):
            raise ValueError("revisit_times contains non-finite values")
        est_rx, est_ry = _interpolate(times, est_arr)
        truth_rx, truth_ry = _interpolate(times, truth_arr)
        revisit_errors = np.hypot(est_rx - truth_rx, est_ry - truth_ry)
        loop_closure_error = float(revisit_errors.mean())

    return {
        "rmse": float(math.sqrt(float(np.mean(distances ** 2)))),
        "max": float(distances.max()),
        "loop_closure_error": loop_closure_error,
    }
