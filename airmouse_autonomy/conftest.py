"""Pytest path setup for the AirMouse workspace (self-contained).

Makes every ament_python package's inner module importable so cores can be
tested without ROS. Run from this directory:

    cd airmouse_autonomy && python -m pytest
"""
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent / "src"
if _SRC.is_dir():
    for pkg in sorted(_SRC.iterdir()):
        if pkg.is_dir() and (pkg / pkg.name).is_dir():
            sys.path.insert(0, str(pkg))