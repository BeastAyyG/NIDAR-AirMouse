# From-scratch setup for a new contributor

This document describes the supported development shape. The PowerShell
launcher is one line only after the prerequisites are installed; it is not a
replacement for installing WSL, ROS 2, PX4, Gazebo, and the DDS agent.

## 1. Windows and WSL2

Open an elevated PowerShell and install the pinned Ubuntu distribution:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart Windows if requested, open Ubuntu once, and create the Linux user.
Keep the project in a path accessible to WSL2. For Windows/QGroundControl
operation, a Windows checkout under `C:\src\AirMouse` is convenient; the
launcher converts its own location automatically.

## 2. Install the upstream dependencies

Inside Ubuntu, follow the official installation instructions for:

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- [PX4 Ubuntu development setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- [PX4 Gazebo simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [PX4 ROS 2 communication and XRCE-DDS](https://docs.px4.io/main/en/ros/ros2_comm.html)

The project is tested against Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and
PX4 v1.17.x. Pin the PX4 and `px4_msgs` versions together in the contributor's
setup notes; do not mix arbitrary branches.

The expected WSL locations are:

```text
$HOME/PX4-Autopilot
$HOME/px4_ros_uxrce_dds_ws/install/bin/MicroXRCEAgent
```

The repository contains its own `PX4-gazebo-models` directory, including the
maze. A separate `$HOME/PX4-gazebo-models` clone is optional. If you use
different locations, set the `AIR_MOUSE_PX4_ROOT`, `AIR_MOUSE_ROS_WS`,
`AIR_MOUSE_PX4_MODELS` (optional fallback), and `AIR_MOUSE_XRCE_AGENT` variables described in
[`AirMouse-Setup.md`](../AirMouse-Setup.md).

## 3. Assemble the project checkout

When publishing this staging directory to GitHub, publish its contents as the
repository root so `Run-AirMouseSim.ps1`, `nidar_airmouse_ros`,
`PX4-gazebo-models`, and `docs` are siblings. Then clone it, for example:

```bash
git clone <your-airmouse-repository-url> AirMouse
cd AirMouse
```

From Linux, source ROS 2 and launch directly from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
bash ./run_airmouse_sim.sh
```

If `PX4-gazebo-models` is kept as a submodule instead of copied into the
project, initialize it before running the launcher:

```bash
git submodule update --init --recursive
```

## 4. Run the one-line launcher

From PowerShell in the parent directory of the checkout:

```powershell
powershell -ExecutionPolicy Bypass -File .\AirMouse\Run-AirMouseSim.ps1
```

For the current staging layout, the equivalent command is:

```powershell
powershell -ExecutionPolicy Bypass -File .\work\Run-AirMouseSim.ps1
```

The first run builds the ROS package and may take several minutes. It writes
runtime logs under `/tmp` in WSL and opens QGroundControl if installed.

## 5. Validate without flying

From WSL at the repository root:

```bash
python3 tools/validate_maze.py
PYTHONPATH=nidar_airmouse_ros python3 -m pytest nidar_airmouse_ros/tests -q
```

Then run the simulation and check:

```powershell
powershell -ExecutionPolicy Bypass -File .\AirMouse\Run-AirMouseSim.ps1 -Headless
```

The expected simulation terminal evidence is `LAND/COMPLETE`, `armed:false`,
and the configured two-marker synthetic validation. It is not physical-flight
approval; use [`Safety-and-Test-Plan.md`](Safety-and-Test-Plan.md) for that.
