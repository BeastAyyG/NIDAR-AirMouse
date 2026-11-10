"""Survivor detector interface + simulation target detector (ROS-independent).

``Detector`` is the structural contract every detector backend implements.
``SimTargetDetector`` detects clearly-identifiable simulation targets within
range, with small deterministic noise (same inputs -> same outputs, no RNG
state). ``YoloDetector`` is a documented placeholder for the real YOLO
integration: construction is safe, ``detect()`` raises NotImplementedError
ONLY when invoked (runtime follow-up on the target machine).
"""
from __future__ import annotations

import math
from typing import Any, Callable, Protocol, Sequence, runtime_checkable

Detection = tuple[float, float, float, float]  # (x, y, confidence, timestamp)
LosFn = Callable[[float, float, float, float], bool]

DEFAULT_DETECT_RANGE_M = 5.0
DEFAULT_BASE_CONFIDENCE = 0.85
DEFAULT_POSITION_NOISE_M = 0.05
DEFAULT_CONFIDENCE_NOISE = 0.02


@runtime_checkable
class Detector(Protocol):
    """Structural detector contract: ``detect(...) -> list[Detection]``."""

    def detect(self, *args: Any, **kwargs: Any) -> list[Detection]: ...


class SimTargetDetector:
    """Range-gated detector for clearly-identifiable sim targets.

    ``detect(targets, pose, now)`` returns ``(x, y, confidence, timestamp)``
    for every target within ``detect_range_m`` of ``pose`` (a 2- or 3-element
    ``(x, y[, theta])`` sequence; heading is ignored). Confidence is
    ``base_confidence`` plus small deterministic noise; positions carry small
    deterministic noise too. An optional ``los_fn(x0, y0, x1, y1) -> bool``
    filters occluded targets (line-of-sight hook for occupancy-aware callers).
    """

    def __init__(
        self,
        detect_range_m: float = DEFAULT_DETECT_RANGE_M,
        base_confidence: float = DEFAULT_BASE_CONFIDENCE,
        position_noise_m: float = DEFAULT_POSITION_NOISE_M,
        confidence_noise: float = DEFAULT_CONFIDENCE_NOISE,
    ) -> None:
        self.detect_range_m = _non_negative(detect_range_m, "detect_range_m")
        self.base_confidence = _unit_interval(base_confidence, "base_confidence")
        self.position_noise_m = _non_negative(position_noise_m, "position_noise_m")
        self.confidence_noise = _non_negative(confidence_noise, "confidence_noise")

    def detect(
        self,
        targets: Sequence[Sequence[float]],
        pose: Sequence[float],
        now: float,
        los_fn: LosFn | None = None,
    ) -> list[Detection]:
        px, py = _parse_pose(pose)
        now = float(now)
        detections: list[Detection] = []
        for target in targets:
            tx, ty = _parse_target(target)
            if math.hypot(tx - px, ty - py) > self.detect_range_m:
                continue
            if los_fn is not None and not los_fn(px, py, tx, ty):
                continue
            x = tx + _unit_noise(tx, ty, salt=1.0) * self.position_noise_m
            y = ty + _unit_noise(tx, ty, salt=2.0) * self.position_noise_m
            confidence = self.base_confidence + _unit_noise(tx, ty, salt=3.0) * self.confidence_noise
            detections.append((x, y, min(1.0, max(0.0, confidence)), now))
        return detections


class YoloDetector:
    """Placeholder for the YOLO survivor-detection backend.

    RUNTIME FOLLOW-UP (not fabricated offline): real weights, camera
    calibration, and inference wiring land with the perception runtime on the
    ROS 2 target machine. Construction stores configuration only and never
    raises; ``detect()`` raises NotImplementedError when explicitly invoked so
    misconfiguration fails loudly instead of silently returning no survivors.
    """

    def __init__(
        self,
        weights_path: str | None = None,
        confidence_threshold: float = 0.5,
    ) -> None:
        self.weights_path = weights_path
        self.confidence_threshold = confidence_threshold

    def detect(self, *args: Any, **kwargs: Any) -> list[Detection]:
        raise NotImplementedError(
            "YoloDetector is a runtime follow-up placeholder; use SimTargetDetector "
            "for simulation. Real YOLO inference is wired on the ROS 2 target."
        )


# ------------------------------------------------------------------ helpers


def _unit_noise(x: float, y: float, salt: float) -> float:
    """Deterministic pseudo-noise in [-1, 1) derived from coordinates.

    Hash-style sine scramble: identical inputs always produce identical
    output, with no RNG state to seed or leak across calls.
    """
    value = math.sin(x * 12.9898 + y * 78.233 + salt * 37.719) * 43758.5453
    return (value - math.floor(value)) * 2.0 - 1.0


def _parse_pose(pose: Sequence[float]) -> tuple[float, float]:
    if isinstance(pose, (str, bytes)) or len(pose) < 2:
        raise ValueError(f"pose must be (x, y) or (x, y, theta), got {pose!r}")
    return float(pose[0]), float(pose[1])


def _parse_target(target: Sequence[float]) -> tuple[float, float]:
    if isinstance(target, (str, bytes)) or len(target) != 2:
        raise ValueError(f"target must be an (x, y) pair, got {target!r}")
    return float(target[0]), float(target[1])


def _non_negative(value: float, name: str) -> float:
    value = float(value)
    if value < 0.0:
        raise ValueError(f"{name} must be >= 0, got {value}")
    return value


def _unit_interval(value: float, name: str) -> float:
    value = float(value)
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be in [0, 1], got {value}")
    return value