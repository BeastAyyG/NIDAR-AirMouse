# Mission film pipeline: bag → video

Turn one recorded mission into a reproducible instrument film: five sensor
views, the accumulated 360° ring, the live occupancy map, and the mission HUD.

The film is **derived data**, never a screen recording: record once, render any
number of times, same bag in → same film out.

```
rosbag2 (.db3)  ──export_bag.py──▶  clip/  ──render_video.py──▶  film.mp4
  ROS machine                       clip.json + .npy            any machine
```

- **Record** — `scripts/record_mission.sh` (ROS machine, during a mission)
- **Export** — `scripts/export_bag.py` (ROS machine; the only ROS-dependent step)
- **Render** — `scripts/render_video.py` (Windows/macOS/Linux; needs Python,
  numpy, pygame, ffmpeg — no ROS)

The clip format is documented in
`src/airmouse_validation/airmouse_validation/clip.py`. Arrays are `.npy`
(lossless depth/map, no image codecs); scalars and file paths are in
`clip.json`.

## Quick start (any machine, no ROS)

```bash
cd airmouse_autonomy
python scripts/render_video.py --synthetic demo_clip --out demo.mp4
```

This builds a fake 60-frame mission clip and renders it, so the whole path can
be exercised before any bag exists. Tests: `python -m pytest` (includes the
renderer suite; ffmpeg test skips if ffmpeg is absent).

## For Manoj's computer

**What his machine needs, exactly:**

| Requirement | Windows | Ubuntu |
|---|---|---|
| Python | 3.10+ on PATH | 3.10+ |
| `numpy` | `pip install numpy` | `python3-numpy` |
| `pygame` | `pip install pygame` | `pip install pygame` (or `python3-pygame`) |
| `ffmpeg` | install, then `ffmpeg -version` works in a fresh terminal | `sudo apt install ffmpeg` |
| ROS 2 | **not needed to render** | only for record/export (`source /opt/ros/humble/setup.bash`) |

Verify in one line:

```bash
python -c "import numpy, pygame; print('ok')" && ffmpeg -version | head -1
```

**Job split**

1. On the Ubuntu machine (ROS): fly a mission and record a bag
   (`scripts/record_mission.sh`), then export it to a clip
   (`scripts/export_bag.py`). Copy the *clip folder* over — not the bag —
   unless the bag itself is wanted for the acceptance matrix.
2. On Manoj's machine (no ROS): render the clip to MP4 and share it.

**Commands he will run**

```bash
# 0) sanity check the renderer end-to-end with fake data
cd airmouse_autonomy
python scripts/render_video.py --synthetic demo_clip --out demo.mp4
# expect: "rendered 60 frames -> demo.mp4", a playable 10 fps video

# 1) render the real clip (exported from the bag)
python scripts/render_video.py --clip /path/to/clip --out mission_film.mp4

# 2) optional: shorter cut or a still-preview from the first N frames
python scripts/render_video.py --clip /path/to/clip --out teaser.mp4 --limit 150
```

**What success looks like:** the MP4 plays at the clip's fps; the RGB, depth,
points, IMU, ring and map panels are populated; only intentionally missing
streams show `NO ... IN CLIP`; tracks appear as green (confirmed) / amber
(uncertain) boxes and markers.

**Clip folder layout**

```
clip/
├── clip.json
├── frames/000000_rgb.npy
├── frames/000000_depth.npy
└── maps/000000_map.npy
```

**Troubleshooting**

| Symptom | Cause / fix |
|---|---|
| `ffmpeg: command not found` | install ffmpeg; reopen the terminal |
| `ModuleNotFoundError: airmouse_validation` | run `python scripts/render_video.py` (the script adds `src/` to the path), not a bare `python -m ...` |
| `pygame` display error | the renderer sets SDL dummy drivers itself; make sure nothing imports pygame before it |
| All panels say `NO ... IN CLIP` | the export found no messages: check topic names/flags and `ros2 bag info` |
| `pose` panel missing | bag has no `/ground_truth/pose`; export with `--pose /odom` or accept the missing drone marker |
| MP4 larger than expected | lower `--fps` at export (5 fps) or record a shorter window |

**Disk guide (640×480, 10 Hz):** RGB ≈ 0.9 MB + depth ≈ 1.2 MB per frame pair,
so ≈ 21 MB/s ≈ **1.3 GB per minute** of recorded mission. Record at 5 Hz or
shorter windows if space is tight (the points topic is deliberately not
recorded; it is derived from depth).

## Work items (for whoever picks this up)

1. **Real-bag validation (Ubuntu).** Run one mission, export, render, attach
   the MP4 + clip metrics to a PR description (do not commit bags or MP4s).
   Files: `scripts/export_bag.py`, `docs/VIDEO-PIPELINE.md`.
2. **Harden the exporter.** Cover `16UC1` depth (already handled), compressed
   images (`CompressedImage`), and topic-name mismatches via the flags.
   File: `scripts/export_bag.py`.
3. **Scenario film set.** Capture the 12 worlds (corridor → full mission),
   render each, and add a contact-sheet index to `reports/`.
4. **Report integration.** Link the film from
   `airmouse_validation.validation_report.generate` (path + thumbnail).
   File: `src/airmouse_validation/airmouse_validation/validation_report.py`.
5. **Evaluator overlay (optional).** A truth-walls panel ala the NIDAR map —
   renderer/evaluator only, never the autonomy path.
6. **Ring and cloud tuning.** More bins, floor rejection by pitch, or a yaw
   panorama strip. Files: `video_render.py` (`RingAccumulator`, `_cloud_panel`).

## Limits, stated plainly

- The RGB-D camera sees ~90° forward; the 360° ring is **accumulated as the
  drone yaws**, which is what the mapper does too. It is not an instant lidar.
- The exporter is UNVERIFIED-OFFLINE: it needs a ROS 2 machine and a real bag.
  The renderer, clip contract and synthetic path are verified here (pytest).
- Ground truth (`/ground_truth/pose`) is used in the film for the drone marker
  and by the evaluator only; it never feeds the autonomy pipeline.
