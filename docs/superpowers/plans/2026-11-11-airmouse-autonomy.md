# AirMouse Autonomy Workspace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a colcon-buildable ROS 2 Humble autonomy workspace whose algorithm cores are ported from the validated NIDAR SAR sim and unit-tested offline on Windows.

**Architecture:** ROS-independent Python cores (numpy+stdlib) per package + thin rclpy `*_node.py` adapters; ament_python package layout; ground truth only enters the evaluator. Cores tested here; ROS runtime shipped review-ready + Goal-0 checklist.

**Tech Stack:** Python 3.12, numpy, pytest (offline); ROS 2 Humble rclpy, sensor_msgs/nav_msgs/geometry_msgs, Gazebo Harmonic SDF, PX4 SITL (target only).

**Spec:** `docs/superpowers/specs/2026-11-11-airmouse-autonomy-design.md`

## Global Constraints

- Repo root `D:\AirMouse\`; workspace `airmouse_autonomy/`. Local commits only; NEVER push.
- Cores MUST NOT import rclpy/ROS at module top; only `*_node.py` may.
- Cell values: UNKNOWN=-1, FREE=0, OCCUPIED=100 (int8 grid).
- Coordinates: meters, x right, y DOWN, grid[row,col], row=int(y/cell), col=int(x/cell); theta 0=+x clockwise.
- Default mapping resolution 0.10 m/cell (config-overridable).
- Occupancy uses hit/miss counting (NOT sticky): free -1 floor -4, hit +2 cap +6, OCCUPIED iff count>=1.
- Path smoothing uses drone-disk clearance (fine-march), not centerline-only.
- Drone footprint radius must fit narrowest passage (config); default 0.10 m.
- No pip install; use preinstalled numpy/pytest. No hardcoded secrets.
- Proven port sources: `D:\NIDAR HACKTRON\sar\mapping\occupancy.py`, `sar\planner\goals.py`, `sar\detection\tracker.py`, `sar\mission\monitor.py`, `main.py:_follow`.
- Every core module gets a pytest suite under `src/<pkg>/test/`; run via repo-root conftest that adds `src/*` to sys.path.

## File Structure

Each `src/<pkg>/`: `package.xml`, `setup.py`, `setup.cfg`, `resource/<pkg>`, `<pkg>/__init__.py`, core modules, `<pkg>/<name>_node.py`, `test/test_*.py`.
Repo root: `conftest.py` (sys.path), `tools/verify_workspace.py`, `scripts/check_env.sh`, `config/*.yaml`, `README.md`.

Shared core signatures (lanes MUST match):
- `occupancy_core.OccupancyMapper(cell_size, width_m, height_m, max_range, clearance)`; `.grid`, `.update(pose, rays)`, `.frontiers()`, `.plan_path(start, goal)`, `.is_known_free(x,y)`, `.observed_mask()`, `.explored_fraction()`, `.reachable_free_mask()`. pose=(x,y,theta); rays=list[(angle,distance,hit)].
- `frontier_explorer.FrontierExplorer(mapper, tracks_fn, entry, params)`; `.set_pose(x,y)`, `.decide(now)->Goal|None`, `.notify_goal_reached(id)`, `.notify_goal_failed(id)`, `.mission_complete`, `.path`, `.current_goal`. Goal=(goal_id,kind,target,priority,status).
- `survivor_tracker.SurvivorTracker(params)`; `.update(detections, now)`, `.tracks`, `.uncertain_tracks()`. Detection=(x,y,confidence,timestamp). Track record dict keys: survivor_id,detected,position_x,position_y,confidence,timestamp,state,sightings.
- `mission_monitor.MissionMonitor(world_regions, mapper, tracker)`; `.update(now)`, `.report()->dict`.
- `mission_fsm.MissionFSM(params)`; `.state`, `.tick(now, inputs)->action`, `.on_sensor_loss()`, `.on_loc_loss()`, `.on_comms_loss()`.
- `depth_processor.depth_to_points(depth, fx,fy,cx,cy, pose, cam_offset)->Nx3 array`.
- `pointcloud_filter.filter_band(points, zmin, zmax)`, `.filter_floor_ceiling(points, floor_z, ceil_z)`.
- `obstacle_detection.detect_regions(points, hfov, regions=3)->dict{left,centre,right:{min,representative,count}}`.
- `localization_eval.compute_errors(est:list[(t,x,y)], truth:list[(t,x,y)])->dict{rmse,max,loop_closure_error}`.
- `validation_metrics.map_f1(grid, truth_grid)->float`; `validation_report.generate(result, out_dir)`.

---

### Task 1: Repo plumbing + interfaces + config (main)

**Files:** Create `conftest.py`, `src/airmouse_interfaces/msg/{ObstacleRegions,SurvivorTrack,MissionStatus}.msg`, `src/airmouse_interfaces/{package.xml,setup.py,setup.cfg,CMakeLists.txt}`, `config/mission.yaml`, `config/mapping.yaml`, `config/perception.yaml`, `config/survivor.yaml`.
**Interfaces:** Produces config keys + msg fields used by all lanes.

- [ ] Write `conftest.py` adding every `src/*` dir to sys.path.
- [ ] Write msg definitions (ObstacleRegions: float32[3] min_distance, float32[3] representative, int32[3] count; SurvivorTrack: int32 survivor_id, bool detected, float64 position_x, float64 position_y, float64 confidence, float64 timestamp, string state; MissionStatus: string state, float32 explored_pct, float32 accessible_pct, int32 survivor_count, bool mission_complete).
- [ ] Write ament_cmake package.xml/CMakeLists for interfaces (rosidl_generate_interfaces).
- [ ] Write config YAMLs with Global-Constraint defaults.
- [ ] Run `python -c "import yaml,sys;[yaml.safe_load(open(p)) for p in sys.argv[1:]]" config/*.yaml` (if pyyaml present) else static check.
- [ ] Commit.

### Task 2: mapping core (lane M)

**Files:** Create `src/airmouse_mapping/airmouse_mapping/{__init__,occupancy_core,grid_manager,map_exporter}.py`, `test/test_occupancy_core.py`, `test/test_map_exporter.py`, package metadata.
**Interfaces:** Consumes config cell_size/clearance. Produces OccupancyMapper signature above; `map_exporter.save_pgm_yaml(grid,cell_size,origin,path)`, `.save_png(grid,tracks,rooms,path)`.

- [ ] Port `occupancy.py` -> `occupancy_core.py`: replace SimConfig ctor with explicit params; keep counting occupancy, supercover, frontiers, A*, disk-clearance `_line_free`, reachable mask. Add `height_band` no-op hook doc (perception filters upstream).
- [ ] Write failing tests: noisy endpoint clears after 2 free passes; repeated hits keep wall; 1-cell door path keeps disk clearance; frontier at free/unknown boundary; A* None when walled off.
- [ ] Run tests (fail), implement, run (pass).
- [ ] `grid_manager.py`: world_to_cell/cell_center/origin helpers + resize.
- [ ] `map_exporter.py`: PGM+YAML (nav2 map_server format) + PNG via numpy (no pygame dependency; write PNG with zlib/struct or PIL if present else PGM only).
- [ ] Test exporter round-trip (read PGM back equals grid).
- [ ] Commit.

### Task 3: navigation + exploration (lane N)

**Files:** Create `src/airmouse_navigation/airmouse_navigation/{__init__,astar_planner,local_planner,collision_checker}.py`, `src/airmouse_exploration/airmouse_exploration/{__init__,frontier_explorer}.py`, tests, metadata.
**Interfaces:** Consumes OccupancyMapper. Produces signatures above.

- [ ] `collision_checker.py`: `disk_clear(mapper,x,y,r)`, `segment_clear(mapper,p0,p1,r)` (fine-march, reuse mapper grid).
- [ ] `astar_planner.py`: `plan(mapper,start,goal)` delegates to mapper.plan_path.
- [ ] `local_planner.py`: port `main._follow` logic -> `follow(pose,path,dt)->(vx,vy,reached,stuck)`; pop tolerance param; stuck watchdog param.
- [ ] `frontier_explorer.py`: port `goals.py` selection (ranking, blacklist, replan, EXPLORE/INVESTIGATE/RETURN, set_pose) but decouple from concrete tracker via `tracks_fn`.
- [ ] Failing tests: local planner pops waypoint & reports reached; stuck watchdog fires after timeout; explorer picks nearest-large frontier, blacklists unreachable, returns RETURN when no frontiers; INVESTIGATE preempts EXPLORE for aged uncertain track.
- [ ] Run fail, implement, run pass. Commit.

### Task 4: survivor + mission (lane S)

**Files:** Create `src/airmouse_survivor/airmouse_survivor/{__init__,survivor_tracker,survivor_detector}.py`, `src/airmouse_mission/airmouse_mission/{__init__,mission_monitor,mission_fsm}.py`, tests, metadata.
**Interfaces:** Consumes mapper+tracker. Produces signatures above.

- [ ] `survivor_tracker.py`: port `tracker.py` merge/confirm/decay; emit record dict schema; `uncertain_tracks()`.
- [ ] `survivor_detector.py`: `Detector` protocol + `SimTargetDetector.detect(targets,pose,now)` (LOS/range/confidence) for clearly-identifiable sim targets; YOLO = follow-up stub raising NotImplementedError only when explicitly invoked.
- [ ] `mission_monitor.py`: port `monitor.py` coverage/region status/findings/report dict.
- [ ] `mission_fsm.py`: states EXPLORE/RETURN/LAND/HOLD/FAILSAFE; tick transitions; on_*_loss -> HOLD/FAILSAFE.
- [ ] Failing tests: duplicate detections within merge_radius -> one track; 3 sightings+conf -> CONFIRMED; decay drops below threshold; FSM sensor loss -> HOLD then timeout -> FAILSAFE; monitor report counts confirmed+uncertain mass.
- [ ] Run fail, implement, run pass. Commit.

### Task 5: perception cores (lane P)

**Files:** Create `src/airmouse_perception/airmouse_perception/{__init__,depth_processor,pointcloud_filter,obstacle_detection}.py`, tests, metadata.
**Interfaces:** Produces signatures above.

- [ ] Failing tests: depth_to_points rejects NaN/0/out-of-range; back-projects a known pixel to known metric point; filter_band keeps z in [zmin,zmax]; filter_floor_ceiling removes floor/ceiling; detect_regions left/centre/right min+representative correct on synthetic cloud.
- [ ] Implement vectorized numpy. Run pass. Commit.

### Task 6: localization evaluator + validation harness (lane V)

**Files:** Create `src/airmouse_localization/airmouse_localization/{__init__,localization_eval,truth_baseline}.py`, `src/airmouse_validation/airmouse_validation/{__init__,replay,validation_metrics,validation_report}.py`, tests, metadata.
**Interfaces:** Consumes cores. Produces signatures above.

- [ ] Failing tests: compute_errors rmse/max on known offsets; loop_closure_error on revisit; map_f1 =1 identical, <1 with FP/FN; replay runs mapper+explorer over synthetic ray stream and completes; report writes markdown+png.
- [ ] Implement. `truth_baseline.py` pure helper formatting truth pose (node wires Gazebo). `replay.py` drives cores offline. `validation_report.generate` writes reports/ markdown + map PNG (reuse map_exporter).
- [ ] Run pass. Commit.

### Task 7: rclpy nodes + launch + SDF/worlds + sensors (lane R, after cores)

**Files:** Create `*_node.py` in each pkg; `src/airmouse_sim/{models/airmouse_x500_rgbd/model.sdf, worlds/*.sdf (12), launch/*.launch.py}`; `src/airmouse_sensors/...`; package metadata for all.
**Interfaces:** Consumes cores + config + msgs. Topics per spec.

- [ ] Nodes: thin subscribe->core->publish; no logic in nodes. mapping_node publishes /map; perception node /obstacles; survivor node /survivors; exploration node /goal; navigation node /path + offboard setpoint; mission node /mission_status; localization node TF + truth baseline.
- [ ] X500 SDF: add depth camera sensor alongside RGB (fix survivor_cam gap), camera_info, points, IMU; realistic noise.
- [ ] 12 worlds per spec list; 15x15 maze with <=6 survivors + exit.
- [ ] launch: sim_bringup, autonomy_bringup, mission.launch.py.
- [ ] Static check: `tools/verify_workspace.py` parses all SDF/launch XML/py, YAML, topic consistency, package metadata, core import graph.
- [ ] Commit.

### Task 8: verify + sample report + README + Goal-0 (main)

**Files:** Create `tools/verify_workspace.py`, `scripts/check_env.sh`, `README.md`, `reports/sample_*`.
- [ ] Run full pytest (all cores) green.
- [ ] Run verify_workspace green.
- [ ] Generate sample validation report + map PNG via replay (proof artifact).
- [ ] README: layout, build (colcon), run, Goal-0 checklist, UNVERIFIED-OFFLINE list, acceptance table, provenance.
- [ ] check_env.sh: ros2/gz/px4 version + topic checks.
- [ ] Commit.