"""Render an AirMouse mission clip to MP4 without ROS.

See airmouse_autonomy/docs/VIDEO-PIPELINE.md for the workflow.

    python airmouse_autonomy/scripts/render_video.py --clip path/to/clip --out film.mp4
    python airmouse_autonomy/scripts/render_video.py --synthetic demo_clip --out demo.mp4
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for package in sorted((ROOT / "src").iterdir()):
    if package.is_dir() and (package / package.name).is_dir():
        sys.path.insert(0, str(package))

from airmouse_validation.video_render import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())