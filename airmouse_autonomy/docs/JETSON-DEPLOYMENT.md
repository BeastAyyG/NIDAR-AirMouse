# Jetson Orin Nano deployment: desktop control + 4-LiDAR / front-3D → 2D map

How to run this stack on a **Jetson Orin Nano** while developing from a laptop
or desktop, and how the 3D sensing becomes the 2D occupancy map the autonomy
uses.

Applies to: Orin Nano dev kit (4 GB or 8 GB), JetPack 6.2.x (Ubuntu 22.04),
ROS 2 Humble. Sensor set assumed: **four LiDARs** (2D `sensor_msgs/LaserScan`
expected) and **one front 3D sensor** (RGB-D or solid-state LiDAR producing a
point cloud). Exact sensor models still need to be confirmed — §9 lists what
is already answered by this repo and what only the hardware can answer.

The same split we already use for the mission film applies: the Jetson flies
and maps; the desktop displays, records and reviews.

```
Jetson (on the drone)                 Desktop (you)
sensor drivers ─┐                     RViz2  /map, /tf
mapping/autonomy ├─ ROS 2 over LAN ─▶ ros2 topic echo / hz
mission FSM      │                     ros2 bag record
PX4 offboard ────┘                     video render (no ROS)
```

> Version and parameter claims below were checked against NVIDIA, PX4 and
> upstream ROS documentation on 2026-11-11 (external handoff brief, same
> date). Items marked **[verify]** came from third-party BSP/vendor pages —
> confirm on NVIDIA's official JetPack page before flashing.

---

## 0. Identify the board first

"Jetson Nano Orion" is not a product name. The 2019 **Jetson Nano** (Maxwell,
JetPack 4.6, Ubuntu 18.04) cannot realistically run ROS 2 + five sensors +
mapping. Everything here assumes the **Jetson Orin Nano** (Ampere, JetPack
6.x). Confirm before doing anything else:

```bash
cat /etc/nv_tegra_release            # exact L4T / JetPack release
uname -a && lsb_release -a           # expect aarch64, Ubuntu 22.04
free -h && df -h                     # RAM variant, SD vs NVMe headroom
sudo pip3 install jetson-stats && sudo reboot
jtop                                 # board model, RAM, power mode, temps
```

Use `jtop` / `tegrastats` for monitoring — `nvidia-smi` is not the right tool
on Jetson. If the board turns out to be a 2019 Nano, stop and re-plan.

## 1. One-time Jetson setup

1. **Flash JetPack 6.2.x** (Ubuntu 22.04 → ROS 2 Humble). Do not pick
   JetPack 7.x just because it is newer: it pairs with Ubuntu 24.04 / ROS 2
   Jazzy, and Jetson package maturity matters more than newness here.
   Reported latest: 6.2.2 / r36.5.0 (Aug 2026) and 7.2 / r39.2.0 (Jul 2026)
   **[verify]**. Two flash paths:
   - **SD card image + balenaEtcher** — works from a Windows desktop, fine
     for initial bring-up, slow I/O.
   - **NVIDIA SDK Manager** (recommended) — choose **NVMe** as storage and
     **OEM Configuration → Pre-Config** to pre-set username/password/hostname
     so the board boots headless with SSH available. **SDK Manager needs an
     x86_64 Ubuntu host (22.04/24.04).** Our desktop is Windows: use a Linux
     VM with reliable USB passthrough, or boot an Ubuntu live USB.
   Keep a fallback: 3.3 V UART serial console (115200 8N1), and recovery
   mode = jumper **FC REC ↔ GND**, power on, USB to the host for reflash.
2. **Give it a fixed address.** DHCP reservation or static IP on the wired
   interface. Ethernet is strongly preferred over Wi-Fi for ROS 2 (multicast
   and latency). Direct desktop↔Jetson cable works with static IPs on both
   ends. mDNS gives `<hostname>.local` on most networks.
3. **Install ROS 2 Humble (arm64)** — official apt packages exist for 22.04:

   ```bash
   sudo apt update && sudo apt install -y software-properties-common curl
   sudo add-apt-repository universe
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
   sudo apt update && sudo apt install -y ros-humble-ros-base ros-dev-tools
   sudo apt install -y python3-numpy python3-pip
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

4. **Performance mode and headroom.** Orin Nano power modes: 8 GB →
   15 W / 25 W / MAXN SUPER; 4 GB → 10 W / 25 W / MAXN SUPER. Check which
   index is MAXN SUPER before selecting it, and make sure the power supply
   can deliver it:

   ```bash
   sudo nvpmodel -q --verbose        # find the MAXN SUPER index first
   sudo nvpmodel -m <index> && sudo jetson_clocks
   sudo systemctl set-default multi-user.target   # boot headless (no GUI)
   ```

   Gotcha: if an SD card previously ran JetPack 6.0/6.1, MAXN may not appear
   until `sudo rm -rf /etc/nvpmodel.conf` + reboot (NVIDIA's documented
   workaround). The desktop GUI eats ~1.5 GB of 8 GB — on a drone, keep it
   off.
5. **SSH keys** (from the desktop, one time):

   ```bash
   ssh-keygen -t ed25519 -C "you@desktop"       # if you don't have a key
   ssh-copy-id <user>@airmouse-jetson.local
   ssh <user>@airmouse-jetson.local 'hostname && uname -m'   # aarch64
   ```

6. **Clock sync** — timestamps from two machines end up in the same TF tree
   and bags:

   ```bash
   sudo apt install -y chrony
   timedatectl    # "System clock synchronized: yes" on both machines
   ```

7. **NVMe (recommended)** for the workspace and logs; keep bags on the desktop.

## 2. Updating JetPack later — three levels, pick the right one

**Level 1 — point release within the same L4T minor (safe, no reflash):**

```bash
sudo apt update && apt list --upgradable
sudo apt upgrade && sudo reboot
cat /etc/nv_tegra_release     # verify
```

**Level 2 — new L4T minor release via apt OTA:** edit BOTH lines of
`/etc/apt/sources.list.d/nvidia-l4t-apt-source.list` (e.g. `r36.4` → `r36.5`,
`common` and `som` repos), then:

```bash
sudo apt update && sudo apt dist-upgrade && sudo reboot
```

**Level 3 — new major JetPack / new Ubuntu / enable Super / move rootfs to
NVMe → reflash with SDK Manager** (§1.1). Back up the current SD/NVMe image
first and record working configs.

Cautions (NVIDIA-documented): never mix packages from different releases
(`nvidia-l4t-core` exists to block partial upgrades); OTA may update QSPI
firmware — power loss mid-update can brick the board, use a stable supply;
`do-release-upgrade` is disabled; **35.x → 36.x via apt is not supported**.
`apt upgrade` alone will never take JetPack 5 → 6 or 6 → 7.

Post-update check: `cat /etc/nv_tegra_release`, `jtop` (power mode, temps),
then a ROS talker/listener pair across the two machines (§4).

## 3. Controlling it from the desktop — exactly how

Three levels, use all three:

| Level | Tool | Command |
|---|---|---|
| Terminal | plain SSH (or `tmux` for long runs) | `ssh airmouse-jetson.local` then `tmux new -s mission` |
| Editor | VS Code Remote-SSH | install "Remote - SSH", connect to `airmouse-jetson.local`, open `~/airmouse_autonomy` — files, terminal and Python all run on the Jetson |
| ROS GUI | **RViz2 on the desktop**, not forwarded | see §4; `ssh -X` for RViz is slow and not recommended |

Run the stack on the Jetson inside `tmux` or as a systemd unit, so a dropped
SSH session does not kill the mission:

```bash
# on the Jetson
cd ~/airmouse_autonomy
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch airmouse_sim autonomy_bringup.launch.py start_offboard:=true
# detach with Ctrl-b d; reattach with: tmux attach -t mission
```

Copy the build to the Jetson once, then rebuild natively on it (arm64; do not
copy the laptop's `build/`/`install/` — they contain x86 binaries):

```bash
rsync -av --exclude build --exclude install --exclude log \
  ./airmouse_autonomy/ user@airmouse-jetson.local:~/airmouse_autonomy/
ssh airmouse-jetson.local 'cd ~/airmouse_autonomy && colcon build --symlink-install'
```

**Safety:** the companion link must never be the only control path. The RC
transmitter and flight-controller failsafes (RC loss, battery, geofence)
stay independent of SSH/ROS. Never stream raw `PointCloud2` over Wi-Fi
during flight — send downsampled clouds or derived scans to the desktop.

## 4. ROS 2 across the two machines

Both machines must agree on **domain id** and **middleware**:

```bash
# ~/.bashrc on BOTH machines
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # apt install ros-humble-rmw-cyclonedds-cpp
```

Open the firewall on the Jetson (UDP) if `ufw` is active:

```bash
sudo ufw allow 7400:7500/udp
```

Verify from the desktop, with the Jetson stack running:

```bash
ros2 multicast send          # on one machine
ros2 multicast receive       # on the other — must see packets
ros2 topic list
ros2 topic hz /map
ros2 run rviz2 rviz2        # add Map(/map), TF, Path(/path), PointCloud2
```

**If discovery is flaky (typical over Wi-Fi / AP isolation):** run a Fast DDS
discovery server on the Jetson and point the desktop at it:

```bash
# Jetson
sudo apt install -y ros-humble-rmw-fastrtps-cpp
fastdds discovery -i 0 -l 0.0.0.0 -p 11811
# Desktop
export ROS_DISCOVERY_SERVER=<jetson-ip>:11811
```

Record bags **on the desktop** so the Jetson's disk and CPU stay free (this is
also what feeds the video pipeline, see `docs/VIDEO-PIPELINE.md`):

```bash
bash airmouse_autonomy/scripts/record_mission.sh bags/jetson_run_01
```

## 5. Sensor bring-up on the Jetson

Install the vendor drivers for your exact models (RPLIDAR, Livox, RealSense,
OAK-D, ...). The stack expects these topics — remap the drivers if their
defaults differ:

| Sensor | Topic | Type |
|---|---|---|
| LiDAR front | `/scan_front` | `sensor_msgs/LaserScan` |
| LiDAR left | `/scan_left` | `sensor_msgs/LaserScan` |
| LiDAR rear | `/scan_rear` | `sensor_msgs/LaserScan` |
| LiDAR right | `/scan_right` | `sensor_msgs/LaserScan` |
| Front 3D | `/camera/depth/color/points` (or `/front/points`) | `sensor_msgs/PointCloud2` |
| IMU (if present) | `/camera/imu` | `sensor_msgs/Imu` |

**Confirm the LiDAR class before buying drivers:** 1D rangefinders
(TFmini/VL53L, single ray), 2D spinning LiDARs (RPLidar, `LaserScan`) and 3D
LiDARs (Livox Mid-360, `PointCloud2`) need completely different front-ends.
The table above assumes 2D. Ethernet LiDARs need per-device static IPs and
PTP/chrony time sync. Note: the Gazebo sim currently bridges only the RGB-D
camera + IMU (`airmouse_sensors/config/gz_bridge.yaml`); the four LiDARs are
a hardware-only part of the rig so far — adding four gz lidars to the x500
model is the sim-side equivalent of this bring-up.

Check every one before touching autonomy:

```bash
ros2 topic hz /scan_front          # expect the driver's rate (10-15 Hz)
ros2 topic echo --once /scan_left  # angle_min/max should span its real FOV
ros2 topic hz /front/points
```

**Frames and TF** are the usual failure point. Publish a URDF (or static
transforms) with `base_link` → each sensor, using the measured mount poses.
The map frame convention in this workspace is metres, x right, y DOWN; each
sensor's yaw offset is applied via TF, not baked into the driver.

**Time**: our mapper accumulates observations, so strict inter-sensor
synchronization is *not* required — feed each scan as it arrives with the
current pose. TF still needs sane timestamps, which §1.6 covers. For
Ethernet sensors sharing one clock, PTP via chrony is the robust option.

## 6. 3D → 2D mapping with 4 LiDARs + 1 front 3D sensor

The "3D → 2D" step is a **projection of a height slab**, not a rendering
task: keep only points within a Z band around the drone's collision volume
and collapse them into ranges or grid cells. The pipeline is already in the
workspace; only the sensor front-end changes:

```
/scan_left, /scan_right, /scan_front, /scan_rear  ──┐
                                                    ├─▶ scan fusion ──▶ OccupancyMapper.update(pose, rays)
/front/points ─▶ height-band filter ─▶ 2D projection ┘        (counting occupancy,
                                                              supercover rays,
                                                              footprints, A*)
```

Rules that make this work:

1. **One common frame.** Transform every scan/point into `base_link` (or
   `map`) with TF at the scan timestamp. The four LiDARs need only their mount
   yaw offsets if they are 2D; the front 3D cloud needs the full transform.
2. **Gravity alignment — the drone-specific killer.** The slab must be
   horizontal in the *world* frame, not the body frame: a 20° bank otherwise
   tilts the "2D" slice into the ground. Publish
   `base_link → base_link_stabilized` from the IMU attitude (remove
   roll/pitch, keep yaw) and filter/project in that frame. In simulation our
   perception node already filters world-Z (`floor_z_m` / `ceil_z_m`); on
   hardware the stabilized frame is the equivalent.
3. **Height band before projection.** Keep only points between the floor and
   ceiling of the *collision volume* (config `flight_band_zmin_m` /
   `flight_band_zmax_m`, default 0.30–2.00 m), tied to current altitude —
   not a fixed world Z. Floor hits are the main source of false walls; strip
   ground/ceiling returns before projection.
4. **Rays, not just points.** For each LiDAR scan, take its
   `(angle, range, hit=True)` triples directly. For the front cloud, project
   points to `(angle, distance, hit=True)` around the drone pose and add
   `(angle, max_range, hit=False)` rays for empty sectors, so free space gets
   carved. Reuse the mapper's counting occupancy — it is what stops noisy
   endpoints from sealing doorways.
5. **Resolution and decimation.** 0.10 m cells on the Orin Nano. Decimate each
   LiDAR scan to ~120–180 rays and update at 5–10 Hz. Budget: stay under
   ~8k rays/second total; the mapper's Python ray loop is the limiter, not the
   sensors. Voxel-downsample clouds (0.05 m leaf) and cap range (~20 m).
6. **Coverage gaps.** Four narrow-FoV LiDARs at 90° spacing leave blind
   sectors. Either overlap the FoVs, add a 360° unit, or accept that the
   planner must not route into unobserved directions (our frontier explorer
   already only routes through known-free space).
7. **Pose.** While testing take pose from the PX4/localization source; on the
   bench (no flight) publish a static `map -> base_link` and move the sensors,
   or use the truth pose in simulation. Do not rely on 2D scan matching alone
   on a drone — roll/pitch and altitude break its assumptions.

**Optional standard front-end:** if you want a plain `sensor_msgs/LaserScan`
from the front cloud (for RViz, slam_toolbox, or a scan merger),
`pointcloud_to_laserscan` is the upstream tool
(`sudo apt install ros-humble-pointcloud-to-laserscan`). Verified behavior:
it subscribes `cloud_in`, publishes `scan`, and **only processes input when
`scan` has a subscriber**. Starting config:

```yaml
pointcloud_to_laserscan_node:
  ros__parameters:
    target_frame: base_link_stabilized   # gravity-aligned, rule 2
    transform_tolerance: 0.01
    min_height: -0.30                    # slab half-height, metres
    max_height: 0.30
    angle_min: -3.14159
    angle_max: 3.14159
    angle_increment: 0.0174533           # 1 deg
    scan_time: 0.1                       # 10 Hz
    range_min: 0.20
    range_max: 20.0
    use_inf: true
```

Our mapper consumes rays directly, so this node is a convenience, not a
requirement.

**One integration piece not implemented yet:** the current
`mapping_node` subscribes to a single point cloud. With four `LaserScan`
inputs we need a small `scan_fusion` front-end (four subscriptions → one ray
list → `mapper.update`). The design is exactly rule 4; it is the first work
item for the hardware bring-up, tracked in the repo.

## 7. Optional later: reactive collision avoidance (PX4 CP)

**Design decision, explicit:** our 2D map serves *visualization and path
planning* — the mission FSM sends waypoints through the offboard bridge
(already built). PX4's Collision Prevention does **not** consume an
`OccupancyGrid`; it is a separate, reactive consumer. Only add it if you
want onboard reflex-level avoidance on top of the planner.

Verified against PX4 v1.17 docs: the companion streams MAVLink
`OBSTACLE_DISTANCE` (uORB `obstacle_distance`; fused output
`/fmu/out/obstacle_distance_fused`, `px4_msgs/msg/ObstacleDistance`). The
internal map uses 72 sectors; `65535` is the no-data sentinel. Key
parameters: `CP_DIST` (min allowed distance; negative disables CP),
`CP_DELAY` (tune to measured sensor-to-actuation latency), `CP_GUIDE_ANG`
(30° suggested, 0 disables guidance), `CP_GO_NO_DATA` (default 0 = refuse to
move into uncovered directions — with four narrow-FoV LiDARs this can freeze
the drone, see rule 6), and `MPC_POS_MODE` = Acceleration based (required).
Failsafe: no range data >0.5 s → movement blocked; >5 s → HOLD. The exact
`distances[]` array spec was not stated on that page — check the
`px4_msgs/ObstacleDistance` definition for your exact PX4 version before
coding. Target ≥10 Hz obstacle updates.

## 8. Bring-up checklist (in order)

1. [ ] Board identified (§0), current image backed up
2. [ ] Jetson boots headless, fixed IP, `ssh` works with keys
3. [ ] `ros2 multicast send/receive` + `ros2 topic list` between desktop and
       Jetson (domain id + firewall)
4. [ ] `chrony` synchronized on both machines
5. [ ] One LiDAR publishes `/scan_*` at the expected rate and FOV
6. [ ] All four LiDARs publish, TF mounts match the physical rig
7. [ ] Front 3D publishes points; gravity-aligned height band removes
       floor/ceiling
8. [ ] Bench test (props off): move a box in front of each sensor, watch
       `/map` update; no blind sectors at the bench
9. [ ] Mapping resolution/decimation tuned to keep CPU under ~50% one core
       (`jtop`)
10. [ ] Autonomy nodes run from the Jetson; RViz2 + recording on the desktop
11. [ ] One recorded bag → clip → film (`docs/VIDEO-PIPELINE.md`)
12. [ ] Flight progression: bench → tethered → low hover. Never test a first
        version in free flight.

## 9. Questions: answered by this repo vs still open

Resolved from the workspace and the original conversation (no need to ask
again):

| Question | Answer |
|---|---|
| Rest of the truncated request | Develop on the laptop now, transfer everything to the Jetson later — §3 rsync + native colcon is exactly that path |
| Desktop OS | Windows → SDK Manager needs an Ubuntu VM/live USB (§1.1); Etcher SD path works natively |
| Flight controller / bridge | PX4 v1.17.0, uXRCE-DDS with `px4_msgs` (offboard bridge already in `airmouse_mission`) |
| ROS version | ROS 2 Humble on both machines |
| Existing code to convert | This repo — occupancy/perception/exploration/mission cores are ported and tested offline; no legacy 3D codebase to migrate |
| Map purpose | Path planning + visualization for indoor SAR, flight band 0.30–2.00 m; reactive avoidance is optional later (§7) |
| Front 3D sensor in sim | Gazebo RGB-D → `/camera/depth/color/points` (`PointCloud2`) |

Still open — only the user/hardware can answer:

1. Orin Nano **4 GB or 8 GB**? Dev kit or third-party carrier? Current
   JetPack/L4T version, SD or NVMe boot? (§0 commands answer this in 1 min)
2. Exact models + interfaces of the four LiDARs, and whether they are 1D,
   2D or 3D (§5) — this decides the driver and fusion front-end.
3. Exact front 3D sensor model (depth camera vs 3D LiDAR), FoV, range, rate.
4. Power/weight budget and required obstacle-update rate for the airframe.

## 10. Honest limits

- CPU budget numbers above are estimates for an 8 GB Orin Nano; the 4 GB
  variant is tighter — measure with `jtop`/`tegrastats` on the real rig
  before trusting them. Keep one mapping node only; consider zram.
- The scan-fusion node is not written yet; the mapper, footprint planning,
  frontier exploration and mission logic it feeds are already tested offline.
- GPU acceleration is available on the Orin but unused; the stack is numpy/CPU.
- 2D maps on a flying vehicle are an approximation: height slab + tilt
  compensation + altitude tracking are mandatory, or obstacles at other
  altitudes are invisible or falsely inflated.
- Do not expose SSH to the internet; use keys, a firewall, and a VPN such as
  Tailscale if you need remote access.