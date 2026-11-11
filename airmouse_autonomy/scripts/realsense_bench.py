#!/usr/bin/env python3
"""RealSense bench test: enumerate, stream, measure, save artifacts.

Runs on the desktop (Windows/Linux) or the Jetson, with no ROS required:
    pip install pyrealsense2 numpy
    python realsense_bench.py --seconds 5 --out realsense_bench

PASS = a device is found, depth frames arrive, and a sane fraction of the
depth image is valid. Writes depth.pgm / color.ppm previews, raw .npy
frames and a summary.json into --out. Exit code 0 on PASS, 1 on FAIL.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("FAIL: pyrealsense2 not installed (pip install pyrealsense2)")
    sys.exit(1)


def device_report():
    ctx = rs.context()
    devices = []
    for dev in ctx.query_devices():
        devices.append({
            "name": dev.get_info(rs.camera_info.name),
            "serial": dev.get_info(rs.camera_info.serial_number),
            "firmware": dev.get_info(rs.camera_info.firmware_version),
            "usb": dev.get_info(rs.camera_info.usb_type_descriptor),
        })
    return devices


def save_pgm(path, frame_mm):
    h, w = frame_mm.shape
    header = f"P5\n{w} {h}\n65535\n".encode("ascii")
    Path(path).write_bytes(header + frame_mm.astype("<u2").tobytes())


def save_ppm(path, rgb):
    h, w, _ = rgb.shape
    header = f"P6\n{w} {h}\n255\n".encode("ascii")
    Path(path).write_bytes(header + rgb.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--out", default="realsense_bench")
    args = ap.parse_args()

    devices = device_report()
    if not devices:
        print("FAIL: no RealSense device found. Check the USB cable/port "
              "(USB3 required for full modes); on Jetson also check power.")
        return 1
    print("Devices:")
    for d in devices:
        print(f"  {d['name']}  serial={d['serial']}  fw={d['firmware']}  "
              f"usb={d['usb']}")
    if any(d["usb"].startswith("2") for d in devices):
        print("WARN: device on USB2 — depth modes are limited; use a USB3 port.")

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    profile = pipeline.start(cfg)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    depth_frames = color_frames = 0
    last_depth = last_color = None
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
            d = frames.get_depth_frame()
            c = frames.get_color_frame()
            if d:
                depth_frames += 1
                last_depth = np.asanyarray(d.get_data())
            if c:
                color_frames += 1
                last_color = np.asanyarray(c.get_data())
    finally:
        pipeline.stop()
    elapsed = time.time() - t0

    if last_depth is None:
        print("FAIL: no depth frames received.")
        return 1

    depth_m = last_depth.astype(np.float32) * depth_scale
    valid = depth_m[depth_m > 0]
    valid_frac = float(valid.size / depth_m.size)
    stats = {
        "devices": devices,
        "seconds": round(elapsed, 2),
        "depth_frames": depth_frames,
        "color_frames": color_frames,
        "depth_fps": round(depth_frames / elapsed, 1),
        "color_fps": round(color_frames / elapsed, 1),
        "depth_scale_m": depth_scale,
        "valid_fraction": round(valid_frac, 3),
        "depth_min_m": round(float(valid.min()), 3) if valid.size else None,
        "depth_median_m": round(float(np.median(valid)), 3) if valid.size else None,
        "depth_max_m": round(float(valid.max()), 3) if valid.size else None,
    }

    save_pgm(out / "depth.pgm", last_depth)
    np.save(out / "depth.npy", last_depth)
    if last_color is not None:
        save_ppm(out / "color.ppm", last_color)
        np.save(out / "color.npy", last_color)
    (out / "summary.json").write_text(json.dumps(stats, indent=2))

    print(json.dumps(stats, indent=2))
    ok = depth_frames > 0 and valid_frac > 0.05 and stats["depth_fps"] > 5
    print(f"{'PASS' if ok else 'FAIL'}: artifacts in {out.resolve()}")
    if not ok:
        print("Check: lens cap off, scene not blank/reflective, USB3 port.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())