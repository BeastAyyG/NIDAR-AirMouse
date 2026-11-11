#!/usr/bin/env python3
"""Bench verifier for scan_fusion_node: watch /map and report.

Subscribes to ``nav_msgs/OccupancyGrid`` (transient-local) for a few seconds
and fails unless the fused map is actually being built: the grid must be
published repeatedly and contain both FREE and OCCUPIED cells carved by the
sensors. Prints a machine-readable summary and optionally saves a PGM.

Run on the machine hosting the map (needs ROS 2 Humble sourced):

    python3 scripts/verify_scan_fusion.py --seconds 12 --pgm /tmp/fusion_map.pgm

Exit code 0 = PASS, 1 = FAIL.
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

FREE = 0
OCCUPIED = 100
UNKNOWN = -1


class MapWatcher(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("verify_scan_fusion")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.messages = 0
        self.last: OccupancyGrid | None = None
        self.changed_cells = 0
        self._previous: np.ndarray | None = None
        self.create_subscription(OccupancyGrid, topic, self._on_map, qos)

    def _on_map(self, msg: OccupancyGrid) -> None:
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height,
                                                           msg.info.width)
        if self._previous is not None and self._previous.shape == grid.shape:
            self.changed_cells += int(np.count_nonzero(grid != self._previous))
        self._previous = grid
        self.messages += 1
        self.last = msg


def save_pgm(grid: np.ndarray, path: str) -> None:
    """Write the grid as an 8-bit PGM (free light, occupied dark)."""
    image = np.full(grid.shape, 205, dtype=np.uint8)   # unknown: mid grey
    image[grid == FREE] = 254
    image[grid == OCCUPIED] = 0
    with open(path, "wb") as handle:
        handle.write(f"P5\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode())
        handle.write(image.tobytes())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--min-messages", type=int, default=3)
    parser.add_argument("--min-free-cells", type=int, default=100)
    parser.add_argument("--min-occupied-cells", type=int, default=10)
    parser.add_argument("--min-explored-fraction", type=float, default=0.02)
    parser.add_argument("--pgm", default=None, help="optional PGM output path")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=["verify_scan_fusion"] + ros_args)
    node = MapWatcher(args.topic)
    deadline = Duration(seconds=args.seconds)
    start = node.get_clock().now()
    while (node.get_clock().now() - start) < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    summary = {"topic": args.topic, "messages": node.messages,
               "seconds": args.seconds, "changed_cells": node.changed_cells}
    reason = ""
    if node.last is None:
        reason = f"no {args.topic} message received"
    else:
        msg = node.last
        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        free = int(np.count_nonzero(grid == FREE))
        occupied = int(np.count_nonzero(grid == OCCUPIED))
        explored = (free + occupied) / grid.size
        summary.update({
            "frame_id": msg.header.frame_id,
            "resolution_m": float(msg.info.resolution),
            "width_cells": int(msg.info.width),
            "height_cells": int(msg.info.height),
            "free_cells": free,
            "occupied_cells": occupied,
            "unknown_cells": int(np.count_nonzero(grid == UNKNOWN)),
            "explored_fraction": round(explored, 4),
        })
        if node.messages < args.min_messages:
            reason = f"only {node.messages} map messages (need {args.min_messages})"
        elif free < args.min_free_cells:
            reason = f"only {free} FREE cells (need {args.min_free_cells})"
        elif occupied < args.min_occupied_cells:
            reason = f"only {occupied} OCCUPIED cells (need {args.min_occupied_cells})"
        elif explored < args.min_explored_fraction:
            reason = f"explored fraction {explored:.3f} < {args.min_explored_fraction}"
        if args.pgm and not reason:
            save_pgm(grid, args.pgm)
            summary["pgm"] = args.pgm

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    print(json.dumps(summary, indent=2))
    if reason:
        print(f"FAIL: {reason}")
        return 1
    print("PASS: scan-fusion map is live (free space carved, obstacles hit)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
