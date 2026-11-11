#!/usr/bin/env bash
# Start the live demo on the Jetson: RealSense driver + bench TF +
# scan-fusion mapping + browser view. Then open http://<jetson-host>:8080/
# from any laptop on the tailnet. Logs: ~/airmouse_logs/. Stop: run with --stop.
set -eo pipefail  # no -u: ROS setup.bash references unbound vars
source /opt/ros/humble/setup.bash
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOGS="$HOME/airmouse_logs"

stop_all() {
  pkill -f "rs_launch.py" 2>/dev/null || true
  pkill -f "realsense2_camera_node" 2>/dev/null || true
  pkill -f "scan_fusion_node" 2>/dev/null || true
  pkill -f "live_view.py" 2>/dev/null || true
  pkill -f "static_transform_publisher" 2>/dev/null || true
}
if [ "${1:-}" = "--stop" ]; then stop_all; echo "stopped"; exit 0; fi

stop_all
sleep 1
mkdir -p "$LOGS"
for d in "$ROOT"/src/*/; do export PYTHONPATH="$d${PYTHONPATH:+:$PYTHONPATH}"; done

# tuned profile: cloud transport is the Orin Nano bottleneck (doc §6.1)
nohup ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=/ pointcloud.enable:=true \
  depth_module.depth_profile:=480x270x30 rgb_camera.color_profile:=640x480x30 \
  > "$LOGS/realsense.log" 2>&1 &

for _ in $(seq 1 30); do
  ros2 node list 2>/dev/null | grep -q "^/camera$" && break
  sleep 1
done
# 4.58.4 quirk: the pointcloud launch arg never reaches the node (doc §5)
ros2 param set /camera pointcloud__neon_.enable true

# bench pose: drone 1 m up, centred in the 15x15 m map; camera on the nose
nohup ros2 run tf2_ros static_transform_publisher 7.5 7.5 1.0 0 0 0 1 \
  map base_link > "$LOGS/tf_map.log" 2>&1 &
nohup ros2 run tf2_ros static_transform_publisher 0.1 0 0.05 0 0 0 1 \
  base_link camera_link > "$LOGS/tf_cam.log" 2>&1 &

cd "$ROOT"
nohup python3 -m airmouse_mapping.scan_fusion_node --ros-args -p use_scans:=false \
  > "$LOGS/fusion.log" 2>&1 &
nohup python3 scripts/live_view.py --port 8080 > "$LOGS/live_view.log" 2>&1 &

echo "live demo starting — open http://$(hostname):8080/ (logs: $LOGS)"