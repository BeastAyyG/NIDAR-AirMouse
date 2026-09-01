# WSL/D:-drive live-simulation runbook

This is the step-by-step recipe for actually running the AirMouse simulation
on Windows + WSL2 with the project on the D: drive (which has more free space
than the C: drive).

If you only want the application-layer proof without Gazebo/PX4, run
``python tools/headless_mission_smoke.py --profile accurate`` on Windows —
that script runs the full mission state machine against a synthetic LiDAR
scan raycast from the real ``maze.sdf`` and prints a per-step trace of
position, LiDAR minimum range, nearest wall, distance to the next waypoint,
which survivors are inside the 2.5 m detection radius, and the runtime
safety contract verdict.

## 1. Pre-flight on Windows (one-time)

```powershell
# Confirm WSL2 is enabled and Ubuntu 24.04 is the default distro.
wsl --status
wsl --list --verbose

# Free space on D:
Get-PSDrive D | Format-Table Used,Free
```

If ``Ubuntu-24.04`` is not installed:

```powershell
wsl --install -d Ubuntu-24.04
# Restart Windows when prompted, open Ubuntu once, create the Linux user.
```

Move the project to D: (it already is at ``D:\setup\NIDAR-AirMouse``). From
PowerShell you can verify the path the launcher will use:

```powershell
powershell -ExecutionPolicy Bypass -File D:\setup\NIDAR-AirMouse\tools\check_powershell.ps1
```

This AST-parses ``Run-AirMouseSim.ps1`` and should report ``POWERSHELL-OK``
with zero parse errors.

## 2. Install the upstream stack inside WSL (one-time, ~20 min)

The launcher expects three things to already exist inside WSL:

| Path | Source | Used by |
| --- | --- | --- |
| ``/home/<user>/PX4-Autopilot`` (PX4 v1.17.0) | PX4 v1.17.0 git checkout | ``run_airmouse_sim.sh`` |
| ``/home/<user>/px4_ros_uxrce_dds_ws/install/bin/MicroXRCEAgent`` | Micro-XRCE-DDS-Agent v2.4.3 | XRCE-DDS agent |
| ``/home/<user>/ros2_px4_ws`` (built) | ROS 2 Jazzy + ``px4_msgs`` (release/1.17) | ``run_airmouse_sim.sh`` |

The repository includes an installer that does all three. From WSL:

```bash
git clone -b agent/fast-development-profile https://github.com/BeastAyyG/NIDAR-AirMouse.git /mnt/d/setup/NIDAR-AirMouse
cd /mnt/d/setup/NIDAR-AirMouse
bash ./setup_ubuntu.sh
```

If PX4 is already installed elsewhere, override the path with
``AIR_MOUSE_PX4_ROOT``; the same applies to ``AIR_MOUSE_ROS_WS``,
``AIR_MOUSE_XRCE_SOURCE``, and ``AIR_MOUSE_XRCE_WS``.

## 3. Run the simulation

Fast profile (recommended for day-to-day WSL2 development):

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/d/setup/NIDAR-AirMouse
AIR_MOUSE_PROFILE=fast AIR_MOUSE_GUI=1 bash ./run_airmouse_sim.sh
```

Accurate profile (1080-sample, 30 Hz LiDAR, shadows on):

```bash
AIR_MOUSE_PROFILE=accurate AIR_MOUSE_GUI=1 bash ./run_airmouse_sim.sh
```

Headless (no Gazebo GUI):

```bash
AIR_MOUSE_PROFILE=fast AIR_MOUSE_GUI=0 bash ./run_airmouse_sim.sh
```

From PowerShell (recommended):

```powershell
$env:AIR_MOUSE_PROFILE = "fast"
powershell -ExecutionPolicy Bypass -File D:\setup\NIDAR-AirMouse\Run-AirMouseSim.ps1
```

## 4. What to look at while it runs

Open a second WSL terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/ros2_px4_ws/install/setup.bash
ros2 topic hz /scan                              # should print ~30 Hz (accurate) or ~10 Hz (fast)
ros2 topic echo --once /airmouse/safety/status  # lidar_ready:true, lidar_safe:true, reason:"clear"
ros2 topic echo --once /airmouse/mission_state  # PRECHECK -> ARM/TAKEOFF -> ... -> LAND/COMPLETE
ros2 topic echo --once /airmouse/mission_status # JSON with armed, waypoint_index, validated_survivors
ros2 topic echo --once /airmouse/survivors/grid_tags # JSON, two survivors tagged
ros2 service call /airmouse/start std_srvs/srv/Trigger {}   # to re-arm
```

If ``/scan`` does not publish, the bridge log is at
``/tmp/airmouse_bridge.log``. If the world appears paused,
``/tmp/airmouse_gz_unpause.log`` shows the unpause attempt. PX4 logs land in
``/tmp/airmouse_px4.log``, the ROS launch in ``/tmp/airmouse_ros.log``, the
XRCE agent in ``/tmp/airmouse_xrce.log``, and the Gazebo GUI in
``/tmp/airmouse_gz_gui.log``.

## 5. Offline / no-WSL validation

Run from any Python 3.10+ on Windows:

```powershell
cd D:\setup\NIDAR-AirMouse
python tools\validate_maze.py
$env:PYTHONPATH = "nidar_airmouse_ros"
python -m pytest nidar_airmouse_ros\tests -q
python -m nidar_airmouse_sim.sim_smoke
python tools\headless_mission_smoke.py --profile accurate
python tools\headless_mission_smoke.py --profile fast
powershell -ExecutionPolicy Bypass -File tools\check_powershell.ps1
```

Expected:

* ``validate_maze.py``: ``maze: PASS (17 walls, 6 survivor markers)`` and
  ``route: PASS (6 ENU points, envelope radius 0.32 m)``.
* ``pytest``: ``5 passed``.
* ``sim_smoke``: prints a JSON ``{"status": "PASS", ...}``.
* ``headless_mission_smoke``: prints ``ALL ASSERTIONS PASSED`` and a
  127-step per-step trace ending at ``LAND/COMPLETE``.
* ``check_powershell.ps1``: ``POWERSHELL-OK``.

## 6. Tearing down

```bash
pkill -f [M]icroXRCEAgent
pkill -f [g]z_x500_lidar_2d
pkill -f [g]z sim
pkill -f [r]os_gz_bridge
pkill -f '/nidar_airmouse_sim/lib/nidar_airmouse_sim/[m]ission-manager'
pkill -f '/nidar_airmouse_sim/lib/nidar_airmouse_sim/[s]im-survivor-detector'
```

The next ``run_airmouse_sim.sh`` call is idempotent — it skips already-running
processes and resumes the Gazebo world clock before starting the mission.

## 7. Free-space budget on D:

The full PX4 + ROS 2 Jazzy + colcon build consumes roughly 12 GB on the
Linux side (PX4 ~5 GB, ROS 2 Jazzy ~3 GB, colcon build/install/log ~3 GB,
PX4-gazebo-models and toolchain the rest). On a D: drive with at least
25 GB of free space, the project has comfortable headroom for incremental
builds and Gazebo logs.
