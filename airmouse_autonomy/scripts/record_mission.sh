#!/usr/bin/env bash
# Record one AirMouse mission to a rosbag2 folder (run on the ROS machine while
# the mission is flying).
#
#   bash airmouse_autonomy/scripts/record_mission.sh [OUT_DIR]
#
# Deliberately does NOT record /camera/depth/color/points: the film renderer
# derives the point view from depth + camera_info, which keeps the bag small.
#
# Disk guide (640x480): RGB+depth at 10 Hz ~= 21 MB/s -> ~1.3 GB/minute.
# For long runs record at 5 Hz or use a shorter capture window.
set -u

OUT="${1:-bags/mission_$(date +%Y%m%d_%H%M%S)}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found; source /opt/ros/humble/setup.bash first" >&2
  exit 2
fi

mkdir -p "$(dirname "$OUT")"
echo "recording to $OUT  (Ctrl-C to stop)"
ros2 bag record -o "$OUT" \
  /camera/color/image_raw \
  /camera/depth/image_rect_raw \
  /camera/color/camera_info \
  /camera/depth/camera_info \
  /camera/imu \
  /map \
  /survivors \
  /mission_state \
  /mission_status \
  /goal \
  /path \
  /ground_truth/pose \
  /clock
echo "bag written to $OUT"
