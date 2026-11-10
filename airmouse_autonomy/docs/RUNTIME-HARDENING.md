# Runtime hardening addendum (2026-11-11)

Follow-up additions to `airmouse_autonomy/` made after the initial workspace
landed on `main`. The goal was the smallest set of changes that materially
improves the chance the stack actually flies on the Ubuntu target, without
touching existing `nidar_airmouse_ros/` code and without adding third-party
dependencies.

## What was added, and why

### 1. PX4 offboard velocity bridge (`airmouse_mission`)

**Gap:** the autonomy nodes publish `/cmd_vel` and `/mission_command`, but
nothing converted those into PX4 `OFFBOARD` setpoints. Goal 7 (offboard
heartbeat, mode/failsafe handling) was unimplemented on the new stack.

**Added:**
- `offboard_core.py` — pure, ROS-independent sequencer (stdlib only):
  - body-frame ENU velocity -> PX4 local NED (`body_to_ned`);
  - streams setpoints at 10 Hz (PX4 requires >= 2 Hz);
  - requests `OFFBOARD` only after 1 s of prestreaming **and** once the
    autonomy stack has actually commanded motion (never arms on startup);
  - requests arming only after PX4 reports `OFFBOARD` active;
  - command timeout (> 0.5 s) -> HOLD (zero velocity); sustained loss
    (> 10 s) -> latched LAND (sticky fail-safe);
  - explicit `HOLD` / `LAND` from `/mission_command` override autonomy;
  - speed and yaw-rate clamping.
- `offboard_bridge_node.py` — thin rclpy adapter: `/cmd_vel`, `/odom`,
  `/mission_command`, `/fmu/out/vehicle_status` in; `/fmu/in/offboard_control_mode`,
  `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` out.
- 10 unit tests covering frames, prestream/arm edges, rate limiting, timeout
  hold-then-land, explicit hold/land, clamping, and the PX4 >= 2 Hz guard.

**Relationship to existing code:** `nidar_airmouse_sim/mission_manager.py`
already implements a **waypoint** mission with PX4 offboard. The new bridge is
a **velocity** adapter for this workspace's autonomy pipeline. Use one or the
other for PX4 control; do not run both against the same vehicle.

**Status:** sequencing logic VERIFIED offline (unit tests); rclpy/px4_msgs
runtime UNVERIFIED-OFFLINE (no ROS on the authoring machine).

### 2. Camera TF and offboard toggle in `autonomy_bringup.launch.py`

**Gap:** nodes look up `map <- base_link`, but nothing published
`base_link <- camera_link`, so the TF tree was incomplete for RViz and any
camera-frame consumer.

**Added:** a `tf2_ros static_transform_publisher` (camera mount at
`0.10, 0.0, 0.05` from `base_link`) and an `offboard_bridge` node guarded by
the new `start_offboard` launch argument (set `false` when PX4 is not running).

### 3. Headless simulation option

**Gap:** `sim_bringup` always started the Gazebo GUI, which fails on a
display-less target.

**Added:** `headless:=true` runs `gz sim -s --headless-rendering` (server-only
with offscreen rendering so the RGB-D camera still publishes).

### 4. Stronger offline verification

**Added to `tools/verify_workspace.py`:**
- every `Node(package=..., executable=...)` in every launch file must resolve
  to a declared `console_scripts` entry (catches launch typos before colcon);
- expected-topic list extended with the PX4 bridge topics and `/odom`.

Checks grew from 122 to **144**; all pass.

### 5. One-command replay and smoke scripts

- `scripts/run_replay.py` — regenerates `reports/report.md`, `map.png`,
  `result.json` offline (`python airmouse_autonomy/scripts/run_replay.py`).
- `scripts/smoke_mission.sh` — on the Ubuntu box: `colcon build`, headless
  `mission.launch.py`, then asserts the expected topics exist, `/map` and the
  camera publish at a sane rate, `/mission_state` appears, and the launch log
  is free of fatal errors. This is the Goal-8 smoke gate, not the full
  12-scenario matrix.

## Constraints honored

- **No new third-party code or dependencies.** Everything is stdlib/numpy or
  first-party. `px4_msgs` is the message package already used by
  `nidar_airmouse_ros`; it is declared as an exec dependency, not vendored.
- **No changes to `nidar_airmouse_ros/`** or the existing mission manager.
- **No fabricated runtime claims.** Anything requiring ROS 2, Gazebo or PX4
  remains explicitly UNVERIFIED-OFFLINE.

## Evidence

- `cd airmouse_autonomy && python -m pytest`: **218 passed** (the workspace is
  self-contained; its own `conftest.py`/`pytest.ini` provide the import paths).
- `python airmouse_autonomy/tools/verify_workspace.py`: **144/144 checks**.
- Offline replay unchanged: mission complete, 99.7% explored, map F1 0.982,
  0 collisions, 3/3 survivors.

## Verify on the Ubuntu target (not yet done)

```bash
bash airmouse_autonomy/scripts/check_env.sh --topics   # after launching
bash airmouse_autonomy/scripts/smoke_mission.sh full_mission.sdf 40
# Expect: SMOKE PASS, arming only after /cmd_vel flows,
# HOLD on command loss, LAND on sustained loss.
```