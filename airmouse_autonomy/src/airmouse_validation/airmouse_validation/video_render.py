"""Render an AirMouse mission clip to MP4 (no ROS required).

Input is a *clip* directory (see ``clip.py``) produced either by
``scripts/export_bag.py`` on a ROS machine or by
``airmouse_validation.clip.make_synthetic_clip`` for development.

The frame is a fixed five-panel instrument view:
    RGB (detection) | depth (metres) | points (flight band) | IMU
    360-degree ring (accumulated while yawing) | occupancy map | HUD

Frames are piped to ffmpeg (libx264, yuv420p) exactly like the NIDAR mission
recorder, so the same clip always produces the same film.

CLI (via the workspace script):
    python scripts/render_video.py --clip path/to/clip --out film.mp4
    python scripts/render_video.py --synthetic demo_clip --out demo.mp4
"""
from __future__ import annotations

import math
import os
import subprocess

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import numpy as np  # noqa: E402
import pygame  # noqa: E402

from .clip import load_clip, make_synthetic_clip  # noqa: E402

BG = (10, 14, 19)
PANEL = (13, 20, 27)
BORDER = (27, 40, 51)
TEXT = (215, 227, 238)
MUTED = (123, 139, 156)
CYAN = (47, 212, 194)
GREEN = (70, 214, 138)
AMBER = (242, 176, 74)
RED = (255, 107, 107)
WALL = (190, 205, 215)
FREE = (54, 68, 80)
UNKNOWN = (14, 20, 26)

_GRID_LUT = np.zeros((256, 3), dtype=np.uint8)
_GRID_LUT[:] = UNKNOWN
_GRID_LUT[0] = FREE
_GRID_LUT[100] = WALL
_GRID_LUT[255] = UNKNOWN  # int8 -1 viewed as uint8


class RingAccumulator:
    """Per-heading minimum depth, accumulated as the drone yaws."""

    def __init__(self, bins: int = 120, max_range_m: float = 8.0) -> None:
        self.bins = int(bins)
        self.max_range_m = float(max_range_m)
        self._dist = np.full(self.bins, np.inf, dtype=np.float32)

    def update(self, pose, depth, camera) -> None:
        if pose is None or depth is None:
            return
        depth = np.asarray(depth, dtype=np.float32)
        if depth.ndim != 2 or depth.size == 0:
            return
        h, w = depth.shape
        band = depth[int(h * 0.30):int(h * 0.70), :]
        with np.errstate(invalid="ignore"):
            safe = np.where(band > 0, band, np.inf)
            d = np.nanmin(safe, axis=0) if band.size else np.full(w, np.inf)
        yaw = float(pose[2])
        u = np.arange(w, dtype=np.float32)
        angles = yaw + np.arctan2(u - float(camera["cx"]), float(camera["fx"]))
        valid = np.isfinite(d) & (d <= self.max_range_m)
        for angle, dist in zip(angles[valid], d[valid]):
            index = int((angle % (2.0 * math.pi)) / (2.0 * math.pi) * self.bins)
            index = min(max(index, 0), self.bins - 1)
            if dist < self._dist[index]:
                self._dist[index] = dist

    def snapshot(self) -> np.ndarray:
        return self._dist.copy()


def _to_surface(rgb: np.ndarray) -> pygame.Surface:
    arr = np.ascontiguousarray(rgb.transpose(1, 0, 2))
    return pygame.surfarray.make_surface(arr)


def colorize_depth(depth: np.ndarray, vmax: float) -> np.ndarray:
    d = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(d) & (d > 0) & (d <= vmax)
    t = np.where(valid, np.clip(d / max(vmax, 1e-6), 0.0, 1.0), 1.0)
    near = np.array([47, 212, 194], dtype=np.float32)
    far = np.array([10, 24, 36], dtype=np.float32)
    rgb = (near * (1.0 - t[..., None]) + far * t[..., None]).astype(np.uint8)
    rgb[~valid] = 0
    return rgb


class MissionFilm:
    """Draws one instrument frame per clip frame (sequential use)."""

    TOP_H = 44
    BOTTOM_H = 54
    GAP = 8
    MARGIN = 12

    def __init__(self, clip: dict) -> None:
        self.clip = clip
        self.width = int(clip["width"])
        self.height = int(clip["height"])
        camera = clip.get("camera") or {}
        self.camera = {
            "fx": float(camera.get("fx", self.width)),
            "fy": float(camera.get("fy", self.width)),
            "cx": float(camera.get("cx", self.width / 2.0)),
            "cy": float(camera.get("cy", self.height / 2.0)),
            "depth_max_m": float(camera.get("depth_max_m", 8.0)),
        }
        self.ring = RingAccumulator(max_range_m=self.camera["depth_max_m"])
        self._fonts: dict[int, pygame.font.Font] = {}
        self._state = {"path": [], "goal": None, "tracks": [], "state": "EXPLORE",
                       "coverage_pct": 0.0, "survivors": 0}
        pygame.font.init()

    # -- helpers ------------------------------------------------------------
    def _font(self, size: int) -> pygame.font.Font:
        if size not in self._fonts:
            self._fonts[size] = pygame.font.Font(None, size)
        return self._fonts[size]

    def _text(self, surface, text, x, y, size=14, color=TEXT) -> None:
        surface.blit(self._font(size).render(str(text), True, color), (x, y))

    def _panel(self, surface, rect, label, sub=None) -> None:
        pygame.draw.rect(surface, PANEL, rect)
        pygame.draw.rect(surface, BORDER, rect, 1)
        self._text(surface, label, rect.x + 8, rect.y + 6, 13, MUTED)
        if sub:
            self._text(surface, sub, rect.x + 8, rect.bottom - 18, 11, MUTED)

    @staticmethod
    def _draw_lines(surface, color, points, width=2) -> None:
        if len(points) >= 2:
            pygame.draw.lines(surface, color, False,
                              [(int(x), int(y)) for x, y in points], width)

    # -- panels ---------------------------------------------------------------
    def _rgb_panel(self, surface, rect, frame) -> None:
        rgb = frame.get("rgb")
        if rgb is None:
            self._text(surface, "NO RGB IN CLIP", rect.centerx - 60,
                       rect.centery, 14, MUTED)
            return
        image = _to_surface(np.asarray(rgb, dtype=np.uint8))
        inner = rect.inflate(-2, -2)
        surface.blit(pygame.transform.smoothscale(image, inner.size), inner.topleft)
        pose = frame.get("pose")
        for track in self._state["tracks"]:
            if pose is None:
                continue
            # projection is approximate on purpose (visual overlay only)
            dx = float(track["x"]) - float(pose[0])
            dy = float(track["y"]) - float(pose[1])
            yaw = float(pose[2])
            forward = dx * math.cos(yaw) + dy * math.sin(yaw)
            right = -dx * math.sin(yaw) + dy * math.cos(yaw)
            if forward <= 0.2:
                continue
            u = self.camera["cx"] + self.camera["fx"] * right / forward
            v = self.camera["cy"]
            sx = inner.x + u / np.asarray(rgb).shape[1] * inner.w
            sy = inner.y + v / np.asarray(rgb).shape[0] * inner.h
            color = GREEN if track.get("state") == "confirmed" else AMBER
            box = pygame.Rect(int(sx) - 22, int(sy) - 34, 44, 66)
            if not inner.contains(box):
                continue  # projected marker falls outside the image panel
            pygame.draw.rect(surface, color, box, 2)
            label = f"S{track.get('id')} {float(track.get('confidence', 0)):.2f}"
            self._text(surface, label, box.x, max(box.y - 14, inner.y), 11, color)

    def _depth_panel(self, surface, rect, frame) -> None:
        depth = frame.get("depth")
        if depth is None:
            self._text(surface, "NO DEPTH IN CLIP", rect.centerx - 60,
                       rect.centery, 14, MUTED)
            return
        rgb = colorize_depth(np.asarray(depth), self.camera["depth_max_m"])
        image = _to_surface(rgb)
        inner = rect.inflate(-2, -2)
        surface.blit(pygame.transform.smoothscale(image, inner.size), inner.topleft)

    def _cloud_panel(self, surface, rect, frame) -> None:
        depth = frame.get("depth")
        if depth is None:
            self._text(surface, "NO DEPTH IN CLIP", rect.centerx - 60,
                       rect.centery, 14, MUTED)
            return
        depth = np.asarray(depth, dtype=np.float32)
        h, w = depth.shape
        stride = max(1, w // 120)
        sub = depth[::stride, ::stride]
        v, u = np.mgrid[0:h:stride, 0:w:stride]
        valid = np.isfinite(sub) & (sub > 0) & (sub <= self.camera["depth_max_m"])
        angle = np.arctan2(u - self.camera["cx"], self.camera["fx"])
        forward = sub * np.cos(angle)
        lateral = sub * np.sin(angle)
        inner = rect.inflate(-8, -8)
        scale = min(inner.w, inner.h) / (2.0 * self.camera["depth_max_m"])
        cx, cy = inner.center
        for fx, ly in zip(forward[valid], lateral[valid]):
            sx = cx + float(ly) * scale
            sy = cy - float(fx) * scale
            surface.set_at((int(sx), int(sy)), CYAN)

    def _imu_panel(self, surface, rect, frame) -> None:
        samples = frame.get("imu") or []
        if not samples:
            self._text(surface, "NO IMU IN CLIP", rect.centerx - 55,
                       rect.centery, 14, MUTED)
            return
        inner = rect.inflate(-12, -24)
        n = len(samples)
        for key, color, row, label in ((0, AMBER, 0, "accel x"),
                                       (3, (106, 168, 255), 1, "gyro z")):
            values = [float(sample[key]) for sample in samples]
            lo, hi = min(values), max(values)
            span = max(hi - lo, 1e-6)
            y0 = inner.y + int(inner.h * (0.30 if row == 0 else 0.72))
            pts = [(inner.x + i / max(n - 1, 1) * inner.w,
                    y0 - (value - lo) / span * inner.h * 0.18)
                   for i, value in enumerate(values)]
            self._draw_lines(surface, color, pts, 2)
            self._text(surface, label, inner.x, y0 - int(inner.h * 0.20), 11, color)

    def _ring_panel(self, surface, rect) -> None:
        inner = rect.inflate(-10, -24)
        cx, cy = inner.center
        radius = min(inner.w, inner.h) // 2 - 2
        for ring_scale in (1.0, 0.66, 0.33):
            pygame.draw.circle(surface, (22, 34, 45),
                               (cx, cy), int(radius * ring_scale), 1)
        pygame.draw.line(surface, (30, 48, 60), (cx - radius, cy),
                         (cx + radius, cy), 1)
        pygame.draw.line(surface, (30, 48, 60), (cx, cy - radius),
                         (cx, cy + radius), 1)
        distances = self.ring.snapshot()
        max_range = self.ring.max_range_m
        for index, dist in enumerate(distances):
            if not np.isfinite(dist):
                continue
            angle = (index + 0.5) / self.ring.bins * 2.0 * math.pi
            px = cx + math.cos(angle) * dist / max_range * radius
            py = cy + math.sin(angle) * dist / max_range * radius
            pygame.draw.circle(surface, CYAN, (int(px), int(py)), 2)
        pygame.draw.polygon(surface, TEXT,
                            [(cx, cy - 7), (cx + 6, cy + 6), (cx - 6, cy + 6)])

    def _map_panel(self, surface, rect) -> None:
        grid = self._state.get("grid")
        if grid is None:
            self._text(surface, "NO MAP IN CLIP", rect.centerx - 55,
                       rect.centery, 14, MUTED)
            return
        grid = np.asarray(grid)
        rgb = _GRID_LUT[np.asarray(grid, dtype=np.uint8)]
        image = _to_surface(rgb)
        inner = rect.inflate(-10, -26)
        inner.y += 10
        scaled = pygame.transform.smoothscale(image, inner.size)
        surface.blit(scaled, inner.topleft)
        cell = float(self.clip.get("map_cell_m", 0.25))
        ox, oy = self.clip.get("map_origin", [0.0, 0.0])
        sx = inner.w / grid.shape[1]
        sy = inner.h / grid.shape[0]

        def to_px(x, y):
            return (inner.x + (float(x) - ox) / cell * sx,
                    inner.y + (float(y) - oy) / cell * sy)

        path = self._state.get("path") or []
        self._draw_lines(surface, CYAN, [to_px(x, y) for x, y in path], 2)
        goal = self._state.get("goal")
        if goal:
            gx, gy = to_px(*goal)
            pygame.draw.circle(surface, CYAN, (int(gx), int(gy)), 5, 2)
        for track in self._state["tracks"]:
            tx, ty = to_px(track["x"], track["y"])
            color = GREEN if track.get("state") == "confirmed" else AMBER
            pygame.draw.circle(surface, color, (int(tx), int(ty)), 5, 2)
        pose = self._state.get("pose")
        if pose:
            dx, dy = to_px(pose[0], pose[1])
            yaw = float(pose[2])
            points = [(dx + 8 * math.cos(yaw + a), dy + 8 * math.sin(yaw + a))
                      for a in (0.0, 2.6, -2.6)]
            pygame.draw.polygon(surface, TEXT, points)

    # -- frame composition -----------------------------------------------------
    def render(self, index: int) -> pygame.Surface:
        frame = self.clip["frames"][index]
        pose = frame.get("pose")
        self._state["pose"] = pose
        for key in ("path", "goal", "tracks"):
            if frame.get(key) is not None:
                self._state[key] = frame[key]
        for key, default in (("state", "EXPLORE"), ("coverage_pct", 0.0),
                             ("survivors", 0)):
            self._state[key] = frame.get(key, self._state.get(key, default))
        if frame.get("map") is not None:
            self._state["grid"] = frame["map"]
        self.ring.update(pose, frame.get("depth"), self.camera)

        surface = pygame.Surface((self.width, self.height))
        surface.fill(BG)
        margin, gap = self.MARGIN, self.GAP

        # title bar
        self._text(surface, "AIRMOUSE — MISSION FILM", margin, 14, 20, TEXT)
        meta = self.clip.get("meta") or {}
        info = f"world {meta.get('world', '?')} · seed {meta.get('seed', '?')}"
        self._text(surface, info, 330, 18, 13, MUTED)
        t = float(frame.get("t", index / float(self.clip["fps"])))
        self._text(surface, f"T+ {int(t) // 60:02d}:{int(t) % 60:02d}",
                   self.width - margin - 110, 16, 15, CYAN)
        pygame.draw.line(surface, BORDER, (0, self.TOP_H - 1),
                         (self.width, self.TOP_H - 1))

        # geometry
        right_w = max(280, int(self.width * 0.28))
        content_y = self.TOP_H + margin
        content_h = self.height - self.TOP_H - self.BOTTOM_H - 2 * margin
        left_w = self.width - 2 * margin - right_w - gap
        panel_w = (left_w - gap) // 2
        panel_h = (content_h - gap) // 2
        rects = {
            "rgb": pygame.Rect(margin, content_y, panel_w, panel_h),
            "depth": pygame.Rect(margin + panel_w + gap, content_y,
                                 panel_w, panel_h),
            "cloud": pygame.Rect(margin, content_y + panel_h + gap,
                                 panel_w, panel_h),
            "imu": pygame.Rect(margin + panel_w + gap,
                               content_y + panel_h + gap, panel_w, panel_h),
        }
        rx = margin + left_w + gap
        ring_h = int(content_h * 0.38)
        map_h = content_h - ring_h - gap
        rects["ring"] = pygame.Rect(rx, content_y, right_w, ring_h)
        rects["map"] = pygame.Rect(rx, content_y + ring_h + gap, right_w, map_h)

        for name, label in (("rgb", "RGB · DETECTION"), ("depth", "DEPTH · METRES"),
                            ("cloud", "POINTS · FLIGHT BAND"),
                            ("imu", "IMU · STABILITY"),
                            ("ring", "360° RING · ACCUMULATED"),
                            ("map", "MAP · DECISIONS")):
            self._panel(surface, rects[name], label)
        self._rgb_panel(surface, rects["rgb"], frame)
        self._depth_panel(surface, rects["depth"], frame)
        self._cloud_panel(surface, rects["cloud"], frame)
        self._imu_panel(surface, rects["imu"], frame)
        self._ring_panel(surface, rects["ring"])
        self._map_panel(surface, rects["map"])

        # bottom bar
        bar_y = self.height - self.BOTTOM_H
        pygame.draw.line(surface, BORDER, (0, bar_y), (self.width, bar_y))
        state = str(self._state.get("state", "EXPLORE"))
        color = CYAN if state == "EXPLORE" else (GREEN if state == "RETURN"
                                                 else AMBER)
        self._text(surface, state, margin, bar_y + 16, 18, color)
        coverage = float(self._state.get("coverage_pct", 0.0))
        bar = pygame.Rect(margin + 130, bar_y + 22, 220, 8)
        pygame.draw.rect(surface, (24, 36, 46), bar)
        pygame.draw.rect(surface, CYAN,
                         pygame.Rect(bar.x, bar.y, int(bar.w * min(coverage / 100.0, 1.0)),
                                     bar.h))
        self._text(surface, f"coverage {coverage:.0f}%", bar.x + bar.w + 10,
                   bar_y + 16, 14, TEXT)
        survivors = int(self._state.get("survivors", 0))
        self._text(surface, f"survivors {survivors}", bar.x + bar.w + 130,
                   bar_y + 16, 14, GREEN if survivors else TEXT)
        if self.width >= 900:
            self._text(surface, "evaluator view · ground truth never feeds autonomy",
                       self.width - margin - 340, bar_y + 18, 11, MUTED)
        return surface


class VideoWriter:
    """Pipes RGB frames to ffmpeg (libx264 / yuv420p)."""

    def __init__(self, path: str, width: int, height: int, fps: float) -> None:
        self.path = str(path)
        self.width, self.height, self.fps = int(width), int(height), float(fps)
        self._process = subprocess.Popen(
            ["ffmpeg", "-y", "-v", "error",
             "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{self.width}x{self.height}", "-r", f"{self.fps:g}",
             "-i", "-", "-c:v", "libx264", "-preset", "veryfast",
             "-crf", "20", "-pix_fmt", "yuv420p", self.path],
            stdin=subprocess.PIPE)

    def write(self, surface: pygame.Surface) -> None:
        self._process.stdin.write(pygame.image.tobytes(surface, "RGB"))

    def close(self) -> None:
        self._process.stdin.close()
        code = self._process.wait()
        if code != 0:
            raise RuntimeError(f"ffmpeg exited with {code} writing {self.path}")


def render_clip(clip: dict, out_path: str, limit: int | None = None,
                progress: bool = False) -> int:
    """Render every clip frame (or the first ``limit``) to ``out_path``."""
    film = MissionFilm(clip)
    writer = VideoWriter(out_path, film.width, film.height,
                         float(clip["fps"]))
    frames = clip["frames"]
    count = len(frames) if limit is None else min(int(limit), len(frames))
    try:
        for index in range(count):
            writer.write(film.render(index))
            if progress and (index + 1) % 50 == 0:
                print(f"  {index + 1}/{count} frames")
    finally:
        writer.close()
    return count


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Render an AirMouse clip to MP4")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--clip", help="clip directory (clip.json + .npy)")
    source.add_argument("--synthetic", metavar="DIR",
                        help="generate a synthetic clip at DIR, then render it")
    parser.add_argument("--out", default="mission_film.mp4")
    parser.add_argument("--limit", type=int, default=None,
                        help="render only the first N frames")
    parser.add_argument("--width", type=int, default=640,
                        help="synthetic clip width (default 640)")
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frames", type=int, default=60,
                        help="synthetic clip frame count (default 60)")
    args = parser.parse_args(argv)

    if args.synthetic:
        make_synthetic_clip(args.synthetic, frames=args.frames,
                            width=args.width, height=args.height)
        clip_dir = args.synthetic
    else:
        clip_dir = args.clip
    clip = load_clip(clip_dir)
    count = render_clip(clip, args.out, args.limit, progress=True)
    print(f"rendered {count} frames -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())