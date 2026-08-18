# NIDAR AirMouse

PX4 + Gazebo + ROS 2 baseline for an indoor, GPS-denied AirMouse maze mission.

The current repository is a simulation and integration baseline. It proves
that PX4 SITL, Gazebo, ROS 2, the DDS bridge, QGroundControl, and the AirMouse
mission state machine can run together in the custom maze. It does not yet
prove physical-flight safety, real-camera survivor detection, SLAM-based
exploration, or competition readiness.

## Run the simulation

From PowerShell at the repository's parent directory:

```powershell
powershell -ExecutionPolicy Bypass -File .\work\Run-AirMouseSim.ps1
```

From Linux, after installing the prerequisites:

```bash
source /opt/ros/jazzy/setup.bash
bash ./run_airmouse_sim.sh
```

The launcher defaults to the `fast` development profile: 360 LiDAR samples at
10 Hz and shadows disabled in the copied runtime world. This keeps the same
real Gazebo/PX4 maze while making WSL2 development responsive. Use
`-Profile full` for the original 1080-sample, 30 Hz LiDAR and shadow settings.
Use `-Headless` for CI or machines without a usable Gazebo GUI. The detailed
setup is in [`AirMouse-Setup.md`](AirMouse-Setup.md).

For a clean machine, follow [`docs/SETUP-FROM-SCRATCH.md`](docs/SETUP-FROM-SCRATCH.md)
first. The one-line PowerShell command is the project launcher after the
upstream prerequisites are installed.

Validate the committed maze geometry and nominal route without starting
Gazebo:

```bash
python3 tools/validate_maze.py
```

Run the ROS package unit tests from WSL after the ROS workspace is installed:

```bash
PYTHONPATH=nidar_airmouse_ros python3 -m pytest nidar_airmouse_ros/tests -q
```

## What is working now

| Area | Current state |
| --- | --- |
| Flight stack | PX4 SITL, `gz_x500_lidar_2d`, offboard position setpoints |
| Simulator | Gazebo maze world with 17 wall models and six visual markers |
| ROS 2 | Jazzy package with mission, synthetic detector, localizer, and GCS status nodes |
| Sensors | Simulated 2D LiDAR bridged to `/scan`; odometry bridged to `/odom` |
| Mission | Takeoff, fixed safe route, simulated two-marker validation, RTL/land/disarm |
| Real autonomy | Not complete: no hardware driver/calibration, real detector, SLAM, or frontier planner |

## Repository layout

```text
.
├── AirMouse-Setup.md
├── Run-AirMouseSim.ps1
├── run_airmouse_sim.sh
├── airmouse_gui_topdown.config
├── PX4-gazebo-models/
│   ├── worlds/maze.sdf
│   └── models/x500_lidar_2d/
├── nidar_airmouse_ros/
│   ├── config/airmouse.yaml
│   ├── launch/airmouse_sim.launch.py
│   └── nidar_airmouse_sim/
└── docs/
    ├── Architecture.md
    ├── MAZE-SPEC.md
    ├── IRL-Transition-Plan.md
    └── Safety-and-Test-Plan.md
```

## Required baseline

The verified development target is Windows + WSL2 Ubuntu 24.04, ROS 2 Jazzy,
Gazebo Harmonic, PX4 v1.17.x, and optional QGroundControl on Windows. The
launcher expects PX4 at `$HOME/PX4-Autopilot` and Micro XRCE-DDS Agent at
`$HOME/px4_ros_uxrce_dds_ws/install/bin/MicroXRCEAgent`. The model repository
is bundled in this checkout. Override locations with
`AIR_MOUSE_PX4_ROOT`, `AIR_MOUSE_ROS_WS`, `AIR_MOUSE_PX4_MODELS` (optional
fallback), and `AIR_MOUSE_XRCE_AGENT`.

## Real-world status

The closest honest description is:

- The software topology is a useful starting point for a real vehicle.
- The current mission logic is a scripted demonstration, not unknown-maze autonomy.
- The simulated LiDAR and odometry interfaces are not substitutes for sensor mounting,
  calibration, time synchronization, vibration testing, and estimator validation.
- The physical aircraft must have independent pilot takeover, geofence, battery,
  link-loss, estimator, and propeller/arm safety procedures.

Read [`docs/IRL-Transition-Plan.md`](docs/IRL-Transition-Plan.md) before
connecting a real aircraft, and [`docs/Safety-and-Test-Plan.md`](docs/Safety-and-Test-Plan.md)
before any propeller-on test.

## Publishing to GitHub

Before publishing, choose one repository structure:

1. A single repository containing the project-owned maze, ROS package, scripts,
   and documentation; or
2. A top-level project repository with `PX4-gazebo-models` as a pinned submodule.

Do not commit WSL build/install/log folders, QGroundControl binaries, private
engineering documents, credentials, flight logs containing personal data, or
unlicensed third-party assets. Keep the current PX4-gazebo-models license and
attribution when redistributing that content.

## Upstream references

- [PX4 Gazebo simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [PX4 ROS 2 communication](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [PX4 ROS 2 offboard control](https://docs.px4.io/main/en/ros/ros2_offboard_control)
- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
- [QGroundControl documentation](https://docs.qgroundcontrol.com/master/en/)
