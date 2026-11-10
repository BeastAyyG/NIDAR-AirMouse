# AirMouse — Simulation-First Autonomy Workspace

ROS 2 Humble autonomy stack for the AirMouse X500 drone: perceives, builds a
2D occupancy map, explores without manual waypoints, detects and tracks
survivors, and completes a full mission in Gazebo Harmonic + PX4 SITL.

Implements the **AirMouse Simulation-First Development Roadmap (Goals 0–8)**.
Algorithm cores are ported from the validated NIDAR SAR simulation
(`D:\NIDAR HACKTRON`, 141 green tests, full mission completes) and are
ROS-independent (numpy + stdlib) so they can be tested without ROS.

> Simulation success is not proof of real-world flight safety. Hardware,
> sensor, localization, and flight behavior must be validated separately.

## Status: what is proven, and what is not

**Verified offline on this machine (Windows, no ROS):**

- 201 pytest tests across every algorithm core (`python -m pytest` at repo root).
- `python airmouse_autonomy/tools/verify_workspace.py` → **122/122 static
  checks** (package metadata, entry points, core purity, 12 SDF worlds parse,
  RGB-D model, YAML configs, launch compile, topic coverage).
- Offline mission replay (`airmouse_autonomy/reports/`): mission completes in
  **87 s sim**, **99.7% explored**, map **F1 0.982**, **0 collisions**,
  **3/3** survivors tracked in the deterministic test world.

**UNVERIFIED-OFFLINE — requires the Ubuntu 22.04 / ROS 2 Humble / Gazebo
Harmonic / PX4 v1.17.0 target machine:**

- colcon build and all runtime node behavior
- Gazebo world spawn, sensor topics, TF tree
- PX4 SITL offboard control, arming, takeoff, landing, failsafe
- rosbag capture and the 12-scenario simulation matrix
- VIO / RGB-D SLAM localization (Goal 4; truth baseline + evaluator shipped)

Nothing above is claimed as tested. Run the Goal-0 checklist first.

## Layout

```
airmouse_autonomy/
├── src/
│   ├── airmouse_interfaces/   # ObstacleRegions, SurvivorTrack, MissionStatus
│   ├── airmouse_sim/          # X500 RGB-D model, 12 worlds, launch files
│   ├── airmouse_sensors/      # ros_gz bridge config + sensors launch
│   ├── airmouse_perception/   # depth -> points, band filters, obstacle regions
│   ├── airmouse_localization/ # truth baseline + drift evaluator (VIO follow-up)
│   ├── airmouse_mapping/      # occupancy core, grid manager, map exporter
│   ├── airmouse_exploration/  # frontier goal selection
│   ├── airmouse_navigation/   # A* wrapper, local planner, collision checker
│   ├── airmouse_survivor/     # dedup tracker, color-blob detector
│   ├── airmouse_mission/      # coverage monitor + failsafe FSM
│   └── airmouse_validation/   # offline replay, metrics, report generator
├── config/                    # mission / mapping / perception / survivor YAML
├── reports/                   # sample validation report (generated)
├── tools/verify_workspace.py  # offline static verifier
└── scripts/check_env.sh       # Goal-0 environment diagnostics (Ubuntu)
```

Cores never import rclpy; each `*_node.py` is a thin adapter that converts ROS
messages and calls a core. Every package is a standard ament_python package
(interfaces and sim/sensors are ament_cmake).

## Build (Ubuntu 22.04 + ROS 2 Humble)

```bash
# PX4 SITL + Micro XRCE-DDS Agent are launched from the existing AirMouse setup
source /opt/ros/humble/setup.bash
cd AirMouse/airmouse_autonomy
colcon build --symlink-install
source install/setup.bash
```

Validate the environment before anything else:

```bash
bash scripts/check_env.sh
```

## Run

```bash
# Full mission: Gazebo world + RGB-D bridge + all autonomy nodes
ros2 launch airmouse_sim mission.launch.py world:=full_mission.sdf

# Simulation only
ros2 launch airmouse_sim sim_bringup.launch.py world:=full_maze_15x15.sdf

# Autonomy only (against an already-running simulation)
ros2 launch airmouse_sim autonomy_bringup.launch.py

# Individual scenario
ros2 launch airmouse_sim sim_bringup.launch.py world:=narrow_passage.sdf
```

Scenario worlds: straight corridor, L-corridor, T-junction, dead end, narrow
passage (0.60 m), multiple obstacles, multiple rooms, full 15×15 maze, moving
obstacles, noisy depth, loop closure, full mission (6 survivors + exit).

## Pipeline

| Node | Subscribes | Publishes |
|---|---|---|
| mapping_node | `/camera/depth/color/points`, TF | `/map` (OccupancyGrid, 2 Hz) |
| perception_node | `/camera/depth/image_rect_raw`, `/camera/color/camera_info`, TF | `/obstacles` |
| survivor_node | `/camera/color/image_raw`, depth, camera_info, TF | `/survivors` |
| exploration_node | `/map`, `/survivors`, TF | `/goal`, `/mission_complete` |
| navigation_node | `/map`, `/goal`, TF | `/path`, `/cmd_vel`, `/nav_status` |
| mission_node | `/map`, `/survivors`, `/mission_complete`, TF | `/mission_status`, `/mission_state`, `/mission_command` |
| localization_node | `/ground_truth/pose`, `/odom` | TF `map->base_link` (DEV baseline only) |

Ground truth is used **only** by `localization_node` (development baseline) and
`airmouse_validation` (scoring). It never enters the autonomy path.

## Algorithms (ported from the validated NIDAR sim)

- **Occupancy**: hit/miss counting (free −1 floor −4, hit +2 cap +6,
  OCCUPIED iff count ≥ 1) so noisy lidar endpoints cannot permanently seal
  doorways — the root cause of premature mission completion in the earlier sim.
- **Planning**: A* with no corner-cutting, expanding only cells whose drone
  disk is clear of OCCUPIED cells (footprint-clear paths, not point-robot
  paths); sub-footprint gaps are refused, flyable doors are preserved.
- **Smoothing**: fine-marched disk-clearance line-of-sight.
- **Exploration**: frontier clusters ranked by size proxy − distance;
  INVESTIGATE (uncertain survivor tracks) preempts EXPLORE; blacklist for
  unreachable frontiers; RETURN when no frontiers remain.
- **Survivor tracking**: spatial-gate merge dedup, 3-sighting + confidence
  promotion to CONFIRMED, confidence decay/drop.
- **Mission**: coverage/region status, prioritized findings, EXPLORE →
  RETURN → LAND state machine with HOLD/FAILSAFE on sensor/localization/comms
  loss.

## Configuration

`config/*.yaml` mirrors the defaults in code: 0.10 m/cell mapping, 6 m sensor
range, 0.10 m drone footprint, 2 s replan interval, 15×15 m mission world,
30 min mission cap. `autonomy_bringup.launch.py` flattens these into node
parameters.

## Validation

```bash
# Offline replay + metrics + report (no ROS needed)
python -m airmouse_validation.replay            # via PYTHONPATH or pytest
python -m pytest airmouse_autonomy/src -q       # core suites
python airmouse_autonomy/tools/verify_workspace.py
```

Sample output is committed in `airmouse_autonomy/reports/` (`report.md`,
`map.png`, `result.json`). On the Ubuntu machine, run each scenario and score
it with `airmouse_validation`; record bags under `bags/`.

### Acceptance targets (from the roadmap)

| Criterion | Target | Offline evidence |
|---|---|---|
| Sensor topics available | 100% nominal | UNVERIFIED-OFFLINE |
| Obstacle detection recall | ≥ 95% | UNVERIFIED-OFFLINE (map recall proxy 98.4%) |
| Occupancy map F1 | ≥ 0.90 | 0.982 (offline replay) |
| Localization error | ≤ 0.20 m | UNVERIFIED-OFFLINE (truth baseline 0.000 m) |
| Mission completion | ≥ 95% | 1/1 offline replay |
| Collisions | 0 | 0 (offline replay) |
| Survivor tracking | up to 6 unique | 3/3 in test world |
| Manual waypoint changes | 0 | 0 |
| Mission duration | < 30 sim-min | 87 s (offline world) |

## Goal-0 checklist (run on the Ubuntu machine)

Run `bash scripts/check_env.sh`; it checks versions and, once the stack is
launched, the expected topics. Then verify manually:

- [ ] `gz sim` launches the chosen world and the AirMouse spawns at a known pose
- [ ] PX4 SITL starts, connects, and `/fmu/out/vehicle_status` is published
- [ ] ROS 2 receives PX4 odometry and vehicle status
- [ ] All camera/depth/points/IMU topics publish at the expected rates
- [ ] TF tree connects map → odom → base_link → camera_link
- [ ] The simulation stops and restarts reliably

## Roadmap goal mapping

| Goal | Status |
|---|---|
| 0 Environment verification | diagnostics + checklist shipped; runtime UNVERIFIED |
| 1 Simulated RGB-D camera | model SDF + bridge config shipped; runtime UNVERIFIED |
| 2 Obstacle detection | core + node + tests shipped; runtime UNVERIFIED |
| 3 2D occupancy mapping | core + node + exporter + tests; replay-verified |
| 4 Localization / SLAM | truth baseline + drift evaluator shipped; VIO/SLAM runtime follow-up |
| 5 Exploration & navigation | cores + nodes + tests; replay-verified |
| 6 Survivor detection/tracking | color-blob detector + dedup tracker + node + tests; YOLO runtime follow-up |
| 7 PX4 integration & mission control | nodes + FSM + `/cmd_vel`/`/mission_command` shipped; offboard runtime UNVERIFIED |
| 8 Full mission validation | offline replay + report shipped; 12-scenario Gazebo matrix runs on Ubuntu |

## Provenance

Cores ported from the NIDAR SAR pygame simulation (`D:\NIDAR HACKTRON`,
commit c090950). Key lessons carried over: counting occupancy instead of sticky
walls; footprint-clear planning (a point-robot path wedges the vehicle at fine
resolution); waypoint pop tolerance below the clearance margin; planning from
the actual pose, never a stale goal target.
