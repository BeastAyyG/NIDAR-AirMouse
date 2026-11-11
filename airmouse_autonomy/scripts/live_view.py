#!/usr/bin/env python3
"""Live browser view of the rig: camera video + distance + fused 2D map.

Runs on the Jetson next to ``scan_fusion_node`` and serves MJPEG over
HTTP, so any laptop on the tailnet can watch without ROS:

    open http://airmouse-jetson:8080/

    source /opt/ros/humble/setup.bash
    export PYTHONPATH=~/airmouse_autonomy/src/airmouse_validation:$PYTHONPATH
    python3 scripts/live_view.py --port 8080

Endpoints: ``/`` (page), ``/stream`` (MJPEG), ``/snapshot.jpg`` (one frame).
"""
from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

from airmouse_validation.live_view import compose_frame

_PAGE = b"""<!doctype html><html><head><title>AirMouse live map</title>
<style>body{background:#181818;color:#eee;font-family:monospace;text-align:center}
img{max-width:98%;border:1px solid #444}</style></head>
<body><h3>AirMouse &mdash; live camera + distance + fused 2D map</h3>
<img src="/stream" alt="live stream"></body></html>"""


class LiveViewNode(Node):
    """Latest colour frame, depth frame and fused grid, plus the bench pose."""

    def __init__(self) -> None:
        super().__init__("live_view")
        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.color: np.ndarray | None = None
        self.depth: np.ndarray | None = None
        self.grid: np.ndarray | None = None
        self.resolution = 0.1
        self.create_subscription(Image, "/camera/color/image_raw",
                                 self._on_color, sensor_qos)
        self.create_subscription(Image, "/camera/depth/image_rect_raw",
                                 self._on_depth, sensor_qos)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

    def _on_color(self, msg: Image) -> None:
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.color = frame

    def _on_depth(self, msg: Image) -> None:
        if msg.encoding not in ("16UC1", "mono16"):
            self.get_logger().warn(f"unexpected depth encoding {msg.encoding}")
            return
        self.depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self.resolution = float(msg.info.resolution)

    def pose_xy(self) -> tuple[float, float] | None:
        try:
            tf = self._tf.lookup_transform("map", "base_link",
                                           rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)


class FrameBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg: bytes = b""

    def set(self, jpeg: bytes) -> None:
        with self._lock:
            self._jpeg = jpeg

    def get(self) -> bytes:
        with self._lock:
            return self._jpeg


def _compose_loop(node: LiveViewNode, buffer: FrameBuffer,
                  rate: float, width: int) -> None:
    period = 1.0 / rate
    while rclpy.ok():
        started = time.monotonic()
        frame = compose_frame(node.color, node.depth, node.grid,
                              node.pose_xy(), resolution=node.resolution,
                              width=width)
        ok, jpeg = cv2.imencode(
            ".jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            buffer.set(jpeg.tobytes())
        time.sleep(max(0.0, period - (time.monotonic() - started)))


def _make_handler(buffer: FrameBuffer, rate: float):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args) -> None:
            pass

        def do_GET(self) -> None:
            if self.path == "/":
                self._send(200, "text/html", _PAGE)
            elif self.path == "/snapshot.jpg":
                self._send(200, "image/jpeg", buffer.get())
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        jpeg = buffer.get()
                        if jpeg:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                f"Content-Length: {len(jpeg)}\r\n\r\n"
                                .encode())
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                        time.sleep(1.0 / rate)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._send(404, "text/plain", b"not found")

        def _send(self, code: int, ctype: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=1280)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=["live_view"] + ros_args)
    node = LiveViewNode()
    buffer = FrameBuffer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=_compose_loop,
                     args=(node, buffer, args.rate, args.width),
                     daemon=True).start()
    server = ThreadingHTTPServer((args.bind, args.port),
                                 _make_handler(buffer, args.rate))
    node.get_logger().info(
        f"live view on http://{args.bind}:{args.port}/ "
        f"(stream {args.rate:.0f} fps, width {args.width})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())