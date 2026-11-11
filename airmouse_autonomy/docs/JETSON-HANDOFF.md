# Jetson handoff — start here

**Audience:** any AI agent or person picking up the Jetson side of AirMouse
cold. Everything needed to run, verify, and extend the rig is here or linked
from here. Last updated 2026-11-12.

**TL;DR state:** the rig is live-verified end to end for mapping. Camera →
ROS 2 → scan-fusion → 2D map → browser view all work on the real hardware.
A demo may be running right now (§4). No LiDARs mounted, no flight yet,
no real pose estimator — the map currently uses a static bench pose.
Remaining work is §8, each item with exact success criteria.

---

## 1. Rig facts (verified on the machine, not assumed)

| Item | Value |
|---|---|
| Board | Jetson Orin Nano **8 GB Super** dev kit |
| OS | Ubuntu 22.04.5 LTS, L4T **R36.4.7** (JetPack 6.2.x), kernel 5.15.148-tegra |
| Python | 3.10.12 (system) |
| ROS | ROS 2 **Humble** (apt), RMW Fast DDS default |
| Front 3D sensor | Intel **RealSense D435i**, fw 5.17.3.10, USB 3.2 (`8086:0b3a`) |
| LiDARs | **none mounted yet** (4 planned: front/left/rear/right) |
| Disk | 233 GB total, ~148 GB free |
| RAM | 7.4 GB total; GUI + browser eat ~2.2 GB — go headless before flight |
| Not installed | `colcon`, `tmux` (pure-Python packages run via PYTHONPATH, §5) |
| Pose source | **static bench TF** `map→base_link` at (7.5, 7.5, 1.0) — not a real estimator |

## 2. Access

- From the owner's desktop: `ssh airmouse-jetson` (Tailscale MagicDNS; the
  desktop `~/.ssh/config` maps this host to the rig, key auth already works,
  no password). Real hostname/IP/user live in the owner's local access file
  and in `~/airmouse_autonomy/HANDOFF.md` **on the rig** — deliberately not
  in this public repo.
- Tailscale carries **no multicast**: desktop-side ROS 2 needs unicast
  discovery (Fast DDS discovery server on the rig, `ROS_DISCOVERY_SERVER`).
  Everything in this handoff runs ON the rig, so you don't need that.
- Node key expiry is ON (6 months from 2026-11-11). If the rig drops off the
  tailnet months later, suspect key expiry first (admin console → disable).

## 3. What's on the rig

| Path | What |
|---|---|
| `~/airmouse_autonomy/` | this repo's workspace (sources only, no colcon build) — all 11 `src/airmouse_*` packages |
| `~/airmouse_autonomy/scripts/` | `run_live_demo.sh`, `live_view.py`, `verify_scan_fusion.py`, `realsense_bench.py`, … |
| `~/airmouse_logs/` | demo logs: `realsense.log`, `fusion.log`, `live_view.log`, `tf_map.log`, `tf_cam.log` |
| `~/airmouse_autonomy/maps/` | created on fusion-node shutdown: `airmouse_fusion_map_<stamp>.pgm` + `.yaml` (nav2 format) |
| `~/airmouse_autonomy/reports/` | bench evidence (`realsense_bench/`, `scan_fusion_bench/summary.json`) |
| everything else in `~/` | the owner's OTHER projects (PX4, yolo, person_detect, …) — **do not touch** |

## 4. Verified-working inventory (evidence, not claims)

| # | What | Evidence |
|---|---|---|
| 1 | Passwordless SSH desktop→rig over Tailscale | live since 2026-11-11 |
| 2 | D435i bench: 24.8 fps depth+color, sane stats | `reports/realsense_bench/`, PR #9 |
| 3 | ROS topics `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/depth/color/points` ~28 Hz | PR #10 |
| 4 | `scan_fusion_node` live: `/map` @ 2 Hz, 5214 FREE / 287 OCCUPIED, 24 % of 15×15 m explored | `reports/scan_fusion_bench/summary.json`, PR #11 |
| 5 | 4-LiDAR fusion path (synthetic scans): ring at 3.08 m median, 206/292 occupied cells outside camera ±43° FoV | same summary, PR #11 |
| 6 | Shutdown export: nav2 PGM+YAML | same summary |
| 7 | Live browser view: video + distance + map, pulled over the tailnet | PR #12 |
| 8 | Offline: **247 pytest**, **151/151** static checks, replay mission 87 s / 99.7 % explored / F1 0.982 / 0 collisions | `python -m pytest`, `tools/verify_workspace.py` |
| 9 | CPU budget: fusion 39.9 % of one core @ 480×270 cloud; driver 35.3 % | summary.json |

**Not verified (do not claim):** real LiDAR input, real pose estimation,
colcon build on the rig, Gazebo/PX4/TF/bags full stack, VIO-SLAM (Goal 4),
YOLO detection, any flight.

## 5. Run it (exact commands, on the rig)

```bash
# one command: driver (tuned) + bench TF + fusion + browser view
bash ~/airmouse_autonomy/scripts/run_live_demo.sh
# then open http://<jetson-host>:8080/ from any laptop on the tailnet
# stop everything:
bash ~/airmouse_autonomy/scripts/run_live_demo.sh --stop

# prove the map is being built (PASS = repeated /map, free+occupied, >2 %)
source /opt/ros/humble/setup.bash
for d in ~/airmouse_autonomy/src/*/; do export PYTHONPATH="$d:$PYTHONPATH"; done
python3 ~/airmouse_autonomy/scripts/verify_scan_fusion.py --seconds 15 --pgm /tmp/fusion_map.pgm

# camera-only bench (no ROS needed)
python3 ~/airmouse_autonomy/scripts/realsense_bench.py
```

Manual launch (what the script does): doc `JETSON-DEPLOYMENT.md` §6.1.
Fusion node alone: `python3 -m airmouse_mapping.scan_fusion_node --ros-args -p use_scans:=false`
(camera-only until LiDARs exist). Logs: `~/airmouse_logs/`. Processes are
`nohup`-ed so they survive SSH disconnect (no tmux on the rig).

## 6. Known quirks — read before debugging anything

1. **Namespace:** launch with `camera_namespace:=/` or topics land under
   `/camera/camera/...`.
2. **Pointcloud arg ignored** (realsense-ros 4.58.4): after launch run
   `ros2 param set /camera pointcloud__neon_.enable true`.
3. **Optical frame:** the cloud is `camera_depth_optical_frame` (x right,
   y down, **z forward**). `optical_to_body()` must permute axes before any
   height band — skipping it collapses the map to one cell. Regression-tested.
4. **PYTHONPATH: append, never clobber** — replacing it breaks Humble's own
   imports. The `for d in src/*/` loop above is the safe pattern.
5. **No colcon/tmux:** run pure-Python nodes via PYTHONPATH; persist with
   `nohup … &`. Installing colcon is remaining work §8.3.
6. **`set -u` breaks ROS `setup.bash`** (unbound `AMENT_TRACE_SETUP_FILES`).
   Use `set -eo pipefail` in scripts that source it.
7. **D435i IMU fails** with apt librealsense on the rig (depth/color fine).
   RSUSB source build is the fix when IMU is needed (doc §5).
8. **CPU limiter is cloud transport, not the mapper.** Halving the fusion
   rate saves ~2 %; dropping 848×480→480×270 saves ~18 %. Tune the driver
   profile first (`depth_module.depth_profile`, `rgb_camera.color_profile`).
9. **Bench pose must sit inside the map** — a corner pose clips half the
   FoV outside the grid and looks like a sensor bug.
10. **rclpy `LaserScan.ranges` is an array** — list slice-assign fails in
    synthetic publishers; assign via `np.asarray` / full replacement.
11. **Shell scripts must stay LF.** The Windows dev box has git autocrlf;
    a CRLF `.sh` dies on the rig. `scp` preserves bytes — don't re-save on
    Windows before shipping.
12. **GUI eats ~2.2 GB RAM.** `sudo systemctl set-default multi-user.target`
    before flight work.

## 7. Repo & publish workflow (how work lands here)

- Dev repo: `D:\AirMouse` on the owner's Windows desktop (local git, **no
  remote**). Push clone: `%TEMP%\nidar-airmouse-push` (origin =
  `github.com/BeastAyyG/NIDAR-AirMouse`).
- Flow: edit locally → `python -m pytest` (247) + `tools/verify_workspace.py`
  (151) → commit locally → copy changed files into the push clone → branch
  from `origin/main` → push → `gh pr create --head <branch>` → squash-merge.
- `gh` quirks: `pr create` needs explicit `--head`; `pr merge` needs the PR
  number when `--repo` is used.
- **Public-repo rules:** no camera frames / bags / MP4s (people in frames),
  no real hostnames/IPs/users/keys (placeholder `airmouse-jetson`), never
  touch `nidar_airmouse_ros/`, secret-scan every diff before pushing.
- Ship code to the rig with `scp` (sources are pure Python; no build step).
  Verify checksums after shipping (`md5sum` both sides).

## 8. Remaining work (priority order, with success criteria)

1. **Mount the 4 LiDARs** (hardware not received yet). Wire, measure each
   mount yaw, set `scan_*mount_yaw_deg` params, run with `use_scans:=true`.
   *Done when:* real scans produce occupied cells outside the camera's
   ±43° FoV (same test as the synthetic ring, summary.json case 3).
2. **Real pose source.** Replace the static bench TF with a localization
   estimator (VIO-SLAM, roadmap Goal 4 — currently UNVERIFIED-OFFLINE).
   *Done when:* `map→base_link` comes from the estimator and the map stays
   consistent while the rig is carried around the room.
3. **colcon build on the rig** (`sudo apt install python3-colcon-common-extensions`,
   `colcon build --symlink-install`). *Done when:* `ros2 run airmouse_mapping
   scan_fusion_node` works without PYTHONPATH tricks.
4. **Gravity-aligned slab in flight.** A 20° bank tilts the "2D" slice into
   the ground (doc rule 2). *Done when:* the height band is applied in a
   gravity-aligned frame, not the body frame, during a tilted hover.
5. **Tethered flight.** PX4 offboard + collision avoidance on the fused map.
   *Done when:* a tethered low hover completes with zero collisions and the
   map matches the room. Prerequisites: 2, 4, headless (§6.12).
6. **Full sim stack** (Gazebo Harmonic + PX4 SITL, Ubuntu desktop) — still
   UNVERIFIED-OFFLINE; Goal-0 checklist in `RUNTIME-HARDENING.md`.
7. **Mission layer on the real map:** frontier exploration + survivor
   detection (cores are ported and offline-verified; not yet run on rig data).
8. **Mission film:** owner **Manoj** — `VIDEO-PIPELINE.md` (record → export →
   render; render runs on any machine, no ROS).

## 9. Doc map

| Doc | What |
|---|---|
| `JETSON-DEPLOYMENT.md` | full deployment guide: flashing, ROS install, sensor quirks (§5), fusion bench (§6.1), checklist (§8) |
| `RUNTIME-HARDENING.md` | offboard bridge, camera TF, headless sim, Goal-0 runtime checklist |
| `VIDEO-PIPELINE.md` | mission-film handoff (Manoj owns render) |
| `reports/scan_fusion_bench/summary.json` | machine-readable live-verification evidence |
| this file | current state + how to pick up the work |