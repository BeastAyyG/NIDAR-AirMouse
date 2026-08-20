# NIDAR AirMouse setup handoff

This checkout is split into three parts:

- `PX4-gazebo-models/`: the Gazebo maze world and vehicle models.
- `nidar_airmouse_ros/`: the ROS 2 simulation package and mission interfaces.
- `Run-AirMouseSim.ps1` / `run_airmouse_sim.sh`: the cross-platform launcher.

The attached engineering book is a project reference, not the current
competition rulebook. Verify current rules, arena dimensions, survivor
definition, safety requirements, and telemetry restrictions with the event
organizers before freezing the real-aircraft design.

The repeatable PowerShell entry point is:

```powershell
powershell -ExecutionPolicy Bypass -File .\work\Run-AirMouseSim.ps1
```

It synchronizes the local ROS package and maze, builds the package, starts the
Micro XRCE-DDS agent, starts PX4 SITL with `gz_x500_lidar_2d` inside the
`maze` world, starts the Gazebo-to-ROS bridge for `/scan` and `/odom`, starts
the AirMouse ROS nodes, and opens QGroundControl if it is installed. Gazebo
opens in a visible top-down maze view by default and follows the drone with
Camera Tracking when the GUI is enabled. The default `fast` profile uses 360
LiDAR samples at 10 Hz and disables maze shadows in the runtime copy; the
repository source files remain unchanged. Use `-Profile accurate` (or
`-Profile full`) to restore 1080 LiDAR samples at 30 Hz and shadows. Use this
for a headless run:

```powershell
powershell -ExecutionPolicy Bypass -File .\work\Run-AirMouseSim.ps1 -Headless
```

To select a profile explicitly:

```powershell
powershell -ExecutionPolicy Bypass -File .\work\Run-AirMouseSim.ps1 -Profile fast
powershell -ExecutionPolicy Bypass -File .\work\Run-AirMouseSim.ps1 -Profile accurate
powershell -ExecutionPolicy Bypass -File .\work\Run-AirMouseSim.ps1 -Profile full  # alias
```

The launcher derives its checkout path from the script location, so it does
not depend on the original author's Windows username. The GitHub checkout
contains the model repository, so Linux users do not need a second
`PX4-gazebo-models` clone. If PX4, the ROS workspace, or the model repository
is in a different WSL location, set the relevant variables first:

```powershell
$env:AIR_MOUSE_PX4_ROOT = "/home/<your-linux-user>/PX4-Autopilot"
$env:AIR_MOUSE_XRCE_AGENT = "/home/<your-linux-user>/px4_ros_uxrce_dds_ws/install/bin/MicroXRCEAgent"
powershell -ExecutionPolicy Bypass -File .\work\Run-AirMouseSim.ps1
```

From Linux, after sourcing ROS 2, run the same shell launcher directly:

```bash
source /opt/ros/jazzy/setup.bash
bash ./run_airmouse_sim.sh
```

Linux profile selection:

```bash
AIR_MOUSE_PROFILE=fast bash ./run_airmouse_sim.sh
AIR_MOUSE_PROFILE=accurate bash ./run_airmouse_sim.sh
AIR_MOUSE_GUI=0 AIR_MOUSE_PROFILE=fast bash ./run_airmouse_sim.sh  # headless
```

The launcher copies only the selected fast-profile runtime changes into
`/tmp/airmouse_fast_models` and the PX4 runtime world. It does not modify the
committed model source. Accurate mode uses the source sensor and shadow
settings directly.

`AIR_MOUSE_PX4_MODELS` is optional and is only used as a fallback if the
bundled `PX4-gazebo-models/worlds/maze.sdf` is not present.

The AirMouse mission is a real PX4 offboard simulation, not only a state
publisher. It waits for valid PX4 preflight checks, arms, climbs to 2.5 m,
flies the collision-free south-corridor route, validates simulated survivor
markers, publishes grid tags, returns home, lands, and disarms. PX4 local NED
coordinates are converted from Gazebo ENU coordinates in the route
configuration; this is why the waypoint values may not look like the SDF
world coordinates.

The current maze contains six visual survivor markers. The tested mission
route visits two markers (`(-7,-6)` and `(0,-12)` in Gazebo ENU coordinates).
The other four markers are not yet covered by the mission. Some are placed in
narrow pockets, so a future planner must prove vehicle-envelope clearance
before attempting them; the current scripted route is not complete maze
exploration.

Useful checks:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash -lc "source /opt/ros/jazzy/setup.bash; source `$HOME/ros2_px4_ws/install/setup.bash; ros2 run nidar_airmouse_sim sim-smoke"
wsl.exe -d Ubuntu-24.04 -- bash -lc "source /opt/ros/jazzy/setup.bash; ros2 topic echo --once /scan sensor_msgs/msg/LaserScan"
wsl.exe -d Ubuntu-24.04 -- bash -lc "source /opt/ros/jazzy/setup.bash; ros2 service call /airmouse/start std_srvs/srv/Trigger {}"
wsl.exe -d Ubuntu-24.04 -- bash -lc "source /opt/ros/jazzy/setup.bash; source `$HOME/ros2_px4_ws/install/setup.bash; ros2 topic echo --once --full-length /airmouse/mission_status std_msgs/msg/String"
```

The engineering book's hardware items (RPLIDAR C1, OAK-D, optical flow,
real-aircraft failsafes, and competition validation) remain separate from this
simulation baseline. The current simulator uses PX4, as requested.

Final local verification target: `LAND/COMPLETE`, `armed:false`, two
validated survivors, home pose within approximately 0.1 m, and no mission
errors. This is a simulation acceptance result, not a flight-readiness claim.

For the detailed real-world transition plan, architecture, exact maze
dimensions, and safety gates, see:

- [`README.md`](README.md)
- [`docs/Architecture.md`](docs/Architecture.md)
- [`docs/MAZE-SPEC.md`](docs/MAZE-SPEC.md)
- [`docs/IRL-Transition-Plan.md`](docs/IRL-Transition-Plan.md)
- [`docs/Safety-and-Test-Plan.md`](docs/Safety-and-Test-Plan.md)
