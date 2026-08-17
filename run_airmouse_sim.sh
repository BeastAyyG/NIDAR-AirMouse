#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${AIR_MOUSE_REPO_ROOT:-$SCRIPT_DIR}"
LINUX_USER="${USER:-$(id -un)}"
ROS_WS="${AIR_MOUSE_ROS_WS:-/home/${LINUX_USER}/ros2_px4_ws}"
PX4_ROOT="${AIR_MOUSE_PX4_ROOT:-/home/${LINUX_USER}/PX4-Autopilot}"
PX4_MODELS="${AIR_MOUSE_PX4_MODELS:-/home/${LINUX_USER}/PX4-gazebo-models}"
XRCE_AGENT="${AIR_MOUSE_XRCE_AGENT:-/home/${LINUX_USER}/px4_ros_uxrce_dds_ws/install/bin/MicroXRCEAgent}"

PROJECT_MODELS="$REPO_ROOT/PX4-gazebo-models"
if [ -f "$PROJECT_MODELS/worlds/maze.sdf" ]; then
  MAZE_WORLD="$PROJECT_MODELS/worlds/maze.sdf"
elif [ -f "$PX4_MODELS/worlds/maze.sdf" ]; then
  MAZE_WORLD="$PX4_MODELS/worlds/maze.sdf"
else
  echo "Could not find maze.sdf in the project or optional external model checkout." >&2
  echo "Expected: $PROJECT_MODELS/worlds/maze.sdf" >&2
  echo "Or set AIR_MOUSE_PX4_MODELS to an external PX4-gazebo-models path." >&2
  exit 1
fi

for required_path in \
  "$REPO_ROOT/nidar_airmouse_ros" \
  "$REPO_ROOT/airmouse_gui_topdown.config" \
  "$PX4_ROOT" \
  "$PX4_ROOT/Tools/simulation/gz/worlds"; do
  if [ ! -e "$required_path" ]; then
    echo "Missing required path: $required_path" >&2
    echo "Set AIR_MOUSE_REPO_ROOT or AIR_MOUSE_PX4_ROOT/AIR_MOUSE_PX4_MODELS if your checkout differs." >&2
    exit 1
  fi
done

mkdir -p "$ROS_WS/src/nidar_airmouse_sim"
cp -a "$REPO_ROOT/nidar_airmouse_ros/." "$ROS_WS/src/nidar_airmouse_sim/"
cp "$MAZE_WORLD" "$PX4_ROOT/Tools/simulation/gz/worlds/maze.sdf"

source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash" 2>/dev/null || true
colcon build --base-paths "$ROS_WS/src" --build-base "$ROS_WS/build" --install-base "$ROS_WS/install" --packages-select nidar_airmouse_sim >/tmp/airmouse_build.log 2>&1
source "$ROS_WS/install/setup.bash"

if [ ! -x "$XRCE_AGENT" ]; then
  echo "Missing MicroXRCEAgent at $XRCE_AGENT" >&2
  exit 1
fi

if ! pgrep -f '[M]icroXRCEAgent udp4 -p 8888' >/dev/null; then
  XRCE_LIB_DIR="$(dirname "$(dirname "$XRCE_AGENT")")/lib"
  nohup env LD_LIBRARY_PATH="$XRCE_LIB_DIR:/opt/ros/jazzy/lib" "$XRCE_AGENT" udp4 -p 8888 >/tmp/airmouse_xrce.log 2>&1 &
fi

if ! pgrep -f '[g]z_x500_lidar_2d' >/dev/null; then
  px4_gz_start='PX4_GZ_WORLD=maze HEADLESS=1 make px4_sitl gz_x500_lidar_2d'
  nohup bash -lc "cd '$PX4_ROOT'; source /opt/ros/jazzy/setup.bash; export PATH=/usr/bin:/opt/ros/jazzy/bin:/usr/local/bin; unset GZ_CONFIG_PATH; ${px4_gz_start}" >/tmp/airmouse_px4.log 2>&1 &
fi

if [ "${AIR_MOUSE_GUI:-1}" = "1" ]; then
  gui_config="$REPO_ROOT/airmouse_gui_topdown.config"
  if ! pgrep -f '[g]z sim -g --gui-config' >/dev/null; then
    nohup bash -lc "export PATH=/usr/bin:/opt/ros/jazzy/bin:/usr/local/bin; unset GZ_CONFIG_PATH; gz sim -g --gui-config '${gui_config}'" >/tmp/airmouse_gz_gui.log 2>&1 &
  fi
fi

if ! pgrep -f '[r]os_gz_bridge parameter_bridge' >/dev/null; then
  nohup bash -lc 'source /opt/ros/jazzy/setup.bash; export PATH=/usr/bin:/opt/ros/jazzy/bin:/usr/local/bin; ros2 run ros_gz_bridge parameter_bridge "/world/maze/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" "/model/x500_lidar_2d_0/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance" --ros-args -r /world/maze/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/scan -r /model/x500_lidar_2d_0/odometry_with_covariance:=/odom' >/tmp/airmouse_bridge.log 2>&1 &
fi

if ! pgrep -f '/nidar_airmouse_sim/lib/nidar_airmouse_sim/[m]ission-manager' >/dev/null || \
   ! pgrep -f '/nidar_airmouse_sim/lib/nidar_airmouse_sim/[s]im-survivor-detector' >/dev/null; then
  nohup bash -lc "source /opt/ros/jazzy/setup.bash; source '$ROS_WS/install/setup.bash'; ros2 launch nidar_airmouse_sim airmouse_sim.launch.py auto_start:=true" >/tmp/airmouse_ros.log 2>&1 &
fi

echo "AirMouse simulation services requested"
echo "PX4 log: /tmp/airmouse_px4.log"
echo "ROS log: /tmp/airmouse_ros.log"
echo "Bridge log: /tmp/airmouse_bridge.log"
