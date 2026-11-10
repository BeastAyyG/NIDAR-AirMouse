# AirMouse Simulation-First Autonomy Workspace — Design

Date: 2026-11-11
Status: approved (design reviewed in chat; user directed full execution)
Source roadmap: `AirMouse — Simulation-First Development Roadmap` (Goals 0–8).
Provenance: algorithms ported from the validated NIDAR SAR pygame sim
(`D:\NIDAR HACKTRON\sar\`, commit c090950, 141 green tests, full mission completes).

## Objective

Prepare a complete, colcon-buildable ROS 2 Humble autonomy workspace for the
AirMouse X500 drone that perceives, maps, explores, detects survivors, and
completes a mission in Gazebo + PX4 SITL without manual waypoints. Everything
that can be validated offline (algorithm cores, static workspace checks) is
validated on this Windows box; everything requiring ROS/Gazebo/PX4 runtime is
shipped review-ready with an explicit Goal-0 checklist and labeled unverified.

Simulation success is not proof of real-world flight safety.

## Environment boundary (hard constraint)

This machine: Windows, Python 3.12, numpy, pytest. NO ROS 2 / Gazebo / PX4.
Target machine (not reachable this session): Ubuntu 22.04, ROS 2 Humble,
Gazebo Harmonic 8.15, PX4 v1.17.0 SITL, Micro XRCE-DDS Agent, RViz2.

Therefore:
- Algorithm cores are ROS-independent Python (numpy + stdlib only) and are
  unit-tested here with pytest.
- rclpy nodes, launch files, SDF models/worlds, package.xml/setup.py are
  authored to ROS 2 Humble conventions but NOT executed here.
- `tools/verify_workspace.py` performs static checks here (XML/SDF parse, YAML
  schema, topic-name consistency, core import graph, package metadata).
- `scripts/check_env.sh` + README Goal-0 checklist cover runtime verification
  on the Ubuntu box. Reports label runtime items UNVERIFIED-OFFLINE.

## Approach (chosen: A)

Port proven cores + thin rclpy adapters. Rejected: (B) rewrite ROS-native
(re-debug known bugs on unreachable machine), (C) wrap pygame sim as backend
(does not satisfy Gazebo/PX4 requirement).

## Repository layout

Repo root `D:\AirMouse\` (own git repo, local commits only, no push).

```
AirMouse/
  docs/superpowers/specs/2026-11-11-airmouse-autonomy-design.md
  airmouse_autonomy/
    src/
      airmouse_interfaces/   # msg: ObstacleRegions, SurvivorTrack, MissionStatus
      airmouse_sim/          # worlds/ models/ launch/  (X500 + RGB-D SDF)
      airmouse_sensors/      # sensor bringup / remap helpers
      airmouse_perception/   # depth_processor, pointcloud_filter, obstacle_detection
      airmouse_localization/ # truth baseline + TF + evaluator (VIO/SLAM = follow-up)
      airmouse_mapping/      # occupancy_core, grid_manager, map_exporter
      airmouse_exploration/  # frontier_explorer
      airmouse_navigation/   # astar_planner, local_planner, collision_checker
      airmouse_survivor/     # survivor_detector, survivor_tracker
      airmouse_mission/      # mission_monitor, mission_fsm
      airmouse_validation/   # replay harness, metrics, report generator
    config/  test_data/  maps/  bags/  reports/
    tools/verify_workspace.py
    scripts/check_env.sh
    README.md
```

Each `src/<pkg>/` is an ament_python package: `package.xml`, `setup.py`,
`setup.cfg`, `resource/<pkg>`, inner python dir `<pkg>/` holding pure cores
(`*_core.py` / logic modules, no rclpy import at module top) plus a separate
`*_node.py` rclpy adapter, and `test/` pytest suites importing only cores.
This adapts the roadmap's flat listing to a colcon-buildable layout.

## Core ports (provenance -> package)

- `airmouse_mapping/occupancy_core.py` <- `sar/mapping/occupancy.py`:
  hit/miss counting occupancy (NOT sticky), supercover ray traversal,
  frontier clustering, A* (8-connected, no corner-cut), footprint-aware LOS
  smoothing, reachable-free mask. `cell_size` default 0.10 m (roadmap).
  Adds flight-height-band filter hook for depth->2D projection (Goal 3).
- `airmouse_mapping/grid_manager.py`: grid lifecycle, resolution/world-frame
  transform helpers, origin handling.
- `airmouse_mapping/map_exporter.py`: OccupancyGrid -> PGM+YAML and PNG
  (reuse NIDAR presentation-map export style).
- `airmouse_exploration/frontier_explorer.py` <- `sar/planner/goals.py`:
  frontier ranking (size proxy - distance cost), blacklist, replan cadence,
  EXPLORE/INVESTIGATE/RETURN priority selection.
- `airmouse_navigation/astar_planner.py`: global planner delegating to
  `occupancy_core.plan_path` (single proven A* implementation).
- `airmouse_navigation/local_planner.py` <- `main.py:_follow`: waypoint
  follow, pop tolerance, stuck watchdog -> goal-failed signal.
- `airmouse_navigation/collision_checker.py` <- mapper disk-clearance
  (`_line_free`) + drone footprint check.
- `airmouse_survivor/survivor_tracker.py` <- `sar/detection/tracker.py`:
  spatial-gate merge dedup, UNCERTAIN->CONFIRMED promotion, decay/drop.
  Emits roadmap record schema {survivor_id, detected, position_x, position_y,
  confidence, timestamp}.
- `airmouse_survivor/survivor_detector.py`: detector interface + simulated
  clearly-identifiable-target detector; YOLO integration = runtime follow-up.
- `airmouse_mission/mission_monitor.py` <- `sar/mission/monitor.py`: coverage,
  region/room status, prioritized findings, mission report.
- `airmouse_mission/mission_fsm.py`: EXPLORE -> RETURN -> LAND; failsafe HOLD
  on sensor/localization/comms loss (Goal 7).

## New perception cores (unit-testable, synthetic depth)

- `depth_processor.py`: depth image -> metric points; reject invalid/NaN/
  out-of-range; camera->drone transform input.
- `pointcloud_filter.py`: floor/ceiling band separation; keep flight-path
  collision volume; voxel/downsample + noise filter.
- `obstacle_detection.py`: left/centre/right regions; min + representative
  (e.g. percentile) obstacle distance per region; structured output.

## Localization

- Truth-pose baseline node (Gazebo ground truth) as DEVELOPMENT reference only.
- TF wiring map->odom->base_link->camera.
- Evaluator core: estimated-vs-truth drift, loop-closure revisit error,
  map alignment metrics. Actual VIO/RGB-D SLAM integration = Goal-4 runtime
  follow-up (interface + evaluator shipped, not fabricated offline).
- Ground truth feeds ONLY the evaluator, never the autonomy pipeline.

## Interfaces / topics

Roadmap names: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`,
`/camera/color/camera_info`, `/camera/depth/camera_info`,
`/camera/depth/color/points`, `/camera/imu`, `/map` (nav_msgs/OccupancyGrid).
Added: `/obstacles` (airmouse_interfaces/ObstacleRegions), `/survivors`
(airmouse_interfaces/SurvivorTrack[]), `/goal` (geometry_msgs/PoseStamped),
`/path` (nav_msgs/Path), `/mission_status` (airmouse_interfaces/MissionStatus).

## Config

`config/` YAML per package + `mission.yaml`: cell_size 0.10, flight-height
band, merge_radius, confirm thresholds, replan_interval, drone footprint
radius vs doorway width (lesson: footprint must fit narrowest passage),
waypoint pop tolerance, stuck watchdog, failsafe timeouts.

## Sim package

- X500-based model SDF with a proper RGB-D sensor (fixes known survivor_cam
  RGB-only gap): color + depth + camera_info + points + IMU, realistic noise.
- 12 scenario worlds: straight corridor, L-corridor, T-junction, dead end,
  narrow passage, multiple static obstacles, multiple rooms, full 15x15 maze,
  moving obstacles, noisy-depth/dropped-frames, loop-closure course, full
  autonomous mission with survivors + exit.
- launch: sim bringup, autonomy bringup, full mission.

## Validation

`airmouse_validation`: offline replay harness (cores vs recorded/synthetic
observations, runnable here); metrics vs truth (map F1, localization error,
obstacle recall, collisions, survivor dedup count, mission completion,
duration); markdown + PNG report generator (NIDAR export style). Roadmap
acceptance table becomes report fields.

## Testing & verification

- pytest over every core on Windows (synthetic depth grids, ray fixtures,
  dedup fixtures, planner scenarios).
- `tools/verify_workspace.py`: SDF/launch XML parse, YAML schema, topic-name
  consistency across nodes, core import graph (no rclpy in cores), package
  metadata completeness.
- UNVERIFIED-OFFLINE (Goal-0 checklist + check_env.sh on Ubuntu): colcon build,
  Gazebo launch, PX4 offboard/arming/takeoff/landing, TF tree, ROS bags,
  uXRCE-DDS heartbeat/failsafe.

## Build waves (parallel lanes, central integration)

1. Skeleton + interfaces + config schema (main); cores+tests: mapping /
   navigation+exploration / survivor+mission (parallel lanes).
2. perception cores+tests; localization evaluator; validation harness (parallel).
3. rclpy nodes + launch + SDF/worlds + config + package metadata (parallel).
4. verify_workspace + sample report + README + Goal-0 checklist + commit (main).

## Acceptance targets (from roadmap; measured, not guaranteed)

sensor availability 100% nominal; obstacle recall >=95%; map F1 >=0.90;
localization error <=0.20 m; mission completion >=95%; collisions 0;
survivors <=6 unique; manual waypoints 0; duration <30 sim-minutes.