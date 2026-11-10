"""Regenerate the offline validation report (no ROS required).

Usage:
    python airmouse_autonomy/scripts/run_replay.py [--out DIR]

Runs the deterministic replay (occupancy mapping + frontier exploration + local
planning + survivor tracking against a synthetic truth world), scores it and
writes report.md / map.png / result.json. Prints the headline metrics and
exits non-zero unless the replay completed its mission.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for package in sorted((ROOT / "src").iterdir()):
    if package.is_dir() and (package / package.name).is_dir():
        sys.path.insert(0, str(package))

from airmouse_validation.replay import run            # noqa: E402
from airmouse_validation.validation_report import generate  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=str(ROOT / "reports"),
                        help="output directory (default: airmouse_autonomy/reports)")
    args = parser.parse_args()

    result = run()
    paths = generate(result, args.out)
    print(f"complete={result['mission_complete']} "
          f"sim={result['duration_s']:.0f}s "
          f"explored={result['explored_pct']:.1f}% "
          f"f1={result['map_f1']:.3f} "
          f"collisions={result['collisions']} "
          f"survivors={result['survivors_tracked']}/{result['truth_survivors']}")
    for name, path in paths.items():
        print(f"{name}: {path}")
    return 0 if result["mission_complete"] else 1


if __name__ == "__main__":
    sys.exit(main())