# Jetson Orin Nano deployment: desktop control + 4-LiDAR / front-3D → 2D map

How to run this stack on a **Jetson Orin Nano** while developing from a laptop
or desktop, and how the 3D sensing becomes the 2D occupancy map the autonomy
uses.

Applies to: Orin Nano 8 GB dev kit, JetPack 6.x (Ubuntu 22.04), ROS 2 Humble.
Sensor set assumed: **four LiDARs** (2D `sensor_msgs/LaserScan` expected) and
**one front 3D sensor** (RGB-D or solid-state LiDAR producing a point cloud).
Exact sensor models still need to be confirmed — the steps below name the
places where that matters.

The same split we already use for the mission film applies: the Jetson flies
and maps; the desktop displays, records and reviews.

```
Jetson (on the drone)                 Desktop (you)
sensor drivers ─┐                     RViz2  /map, /tf
mapping/autonomy ├─ ROS 2 over LAN ─▶ ros2 topic echo / hz
mission FSM      │                     ros2 bag record
PX4 offboard ────┘                     video render (no ROS)
```

---

## 1. One-time Jetson setup

1. **Flash JetPack 6.x** (Ubuntu 22.04) with the NVIDIA SDK Manager or
   `balenaEtcher`. Boot, finish the first-run wizard, set a hostname you can
   remember, e.g. `airmouse-jetson`.
2. **Give it a fixed address.** Either a DHCP reservation in your router or a
   static IP on the wired interface. Ethernet is strongly preferred over
   Wi-Fi for ROS 2 (multicast and latency).
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

4. **Performance mode and headroom.** The desktop GUI eats ~1.5 GB of the
   8 GB — on a drone you don't want it:

   ```bash
   sudo nvpmodel -m 0 && sudo jetson_clocks     # max clocks
   sudo systemctl set-default multi-user.target # boot headless (no GUI)
   ```

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

## 2. Controlling it from the desktop — exactly how

Three levels, use all three:

| Level | Tool | Command |
|---|---|---|
| Terminal | plain SSH (or `tmux` for long runs) | `ssh airmouse-jetson.local` then `tmux new -s mission` |
| Editor | VS Code Remote-SSH | install "Remote - SSH", connect to `airmouse-jetson.local`, open `~/airmouse_autonomy` — files, terminal and Python all run on the Jetson |
| ROS GUI | **RViz2 on the desktop**, not forwarded | see §3; `ssh -X` for RViz is slow and not recommended |

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

## 3. ROS 2 across the two machines

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

## 4. Sensor bring-up on the Jetson

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

Check every one before touching autonomy:

```bash
ros2 topic hz /scan_front          # expect the driver's rate (10-15 Hz)
ros2 topic echo --once /scan_left  # angle_min/max should span its real FOV
ros2 topic hz /front/points
```

**Frames and TF** are the usual failure point. Publish a URDF (or static
transforms) with `base_link` → each sensor, using the measured mount poses. The
map frame convention in this workspace is metres, x right, y DOWN; each
sensor's yaw offset is applied via TF, not baked into the driver.

**Time**: our mapper accumulates observations, so strict inter-sensor
synchronization is *not* required — feed each scan as it arrives with the
current pose. TF still needs sane timestamps, which §1.6 covers.

## 5. 3D → 2D mapping with 4 LiDARs + 1 front 3D sensor

The pipeline is already in the workspace; only the sensor front-end changes:

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
2. **Height band before projection.** Keep only points between the floor and
   ceiling of the *collision volume* (config `flight_band_zmin_m` /
   `flight_band_zmax_m`, default 0.30–2.00 m). Floor hits are the main source
   of false walls.
3. **Rays, not just points.** For each LiDAR scan, take its
   `(angle, range, hit=True)` triples directly. For the front cloud, project
   points to `(angle, distance, hit=True)` around the drone pose and add
   `(angle, max_range, hit=False)` rays for empty sectors, so free space gets
   carved. Reuse the mapper's counting occupancy — it is what stops noisy
   endpoints from sealing doorways.
4. **Resolution and decimation.** 0.10 m cells on the Orin Nano. Decimate each
   LiDAR scan to ~120–180 rays and update at 5–10 Hz. Budget: stay under
   ~8k rays/second total; the mapper's Python ray loop is the limiter, not the
   sensors.
5. **Pose.** While testing take pose from the PX4/localization source; on the
   bench (no flight) publish a static `map -> base_link` and move the sensors,
   or use the truth pose in simulation.

**One integration piece not implemented yet:** the current
`mapping_node` subscribes to a single point cloud. With four `LaserScan`
inputs we need a small `scan_fusion` front-end (four subscriptions → one ray
list → `mapper.update`). The design is exactly rule 3; it is the first work
item for the hardware bring-up, tracked in the repo.

## 6. Bring-up checklist (in order)

1. [ ] Jetson boots headless, fixed IP, `ssh` works with keys
2. [ ] `ros2 topic list` between desktop and Jetson (domain id + firewall)
3. [ ] `chrony` synchronized on both machines
4. [ ] One LiDAR publishes `/scan_*` at the expected rate and FOV
5. [ ] All four LiDARs publish, TF mounts match the physical rig
6. [ ] Front 3D publishes points; height-band filter removes floor/ceiling
7. [ ] Bench test: move a box in front of each sensor, watch `/map` update
8. [ ] Mapping resolution/decimation tuned to keep CPU under ~50% one core
9. [ ] Autonomy nodes run from the Jetson; RViz2 + recording on the desktop
10. [ ] One recorded bag → clip → film (`docs/VIDEO-PIPELINE.md`)

## 7. Honest limits

- CPU budget numbers above are estimates for an 8 GB Orin Nano; measure with
  `top`/`tegrastats` on the real rig before trusting them.
- The scan-fusion node is not written yet; the mapper, footprint planning,
  frontier exploration and mission logic it feeds are already tested offline.
- GPU acceleration is available on the Orin but unused; the stack is numpy/CPU.
- Do not expose SSH to the internet; use keys, a firewall, and a VPN such as
  Tailscale if you need remote access.
