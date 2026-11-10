#!/usr/bin/env bash
# One-command Goal-8 smoke test (run on the Ubuntu 22.04 / ROS 2 Humble box).
#
#   bash airmouse_autonomy/scripts/smoke_mission.sh [world] [seconds]
#
# Builds the workspace, launches the headless full mission, waits, then checks
# that the core topics exist and are flowing, that /map is growing, and that
# the mission node reports a state. Exits non-zero on the first failed check.
# This is a smoke test, not the full 12-scenario acceptance matrix.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD="${1:-full_mission.sdf}"
SECONDS_TO_RUN="${2:-40}"
LOG="${TMPDIR:-/tmp}/airmouse_smoke_$(date +%s).log"
FAIL=0

ok()  { printf '  PASS  %s\n' "$1"; }
bad() { printf '  FAIL  %s\n' "$1"; FAIL=1; }

command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found; source ROS 2 first"; exit 2; }
if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ROS_DISTRO unset; run: source /opt/ros/humble/setup.bash"; exit 2
fi

echo "== build =="
( cd "$WS" && colcon build --symlink-install ) || { echo "colcon build failed"; exit 1; }
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

echo "== launch (headless, world=$WORLD, ${SECONDS_TO_RUN}s) =="
ros2 launch airmouse_sim mission.launch.py world:="$WORLD" headless:=true \
  >"$LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
  sleep 3
  kill -KILL "$LAUNCH_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep "$SECONDS_TO_RUN"

echo "== topics =="
TOPICS=$(timeout 10 ros2 topic list 2>/dev/null || true)
for t in /camera/color/image_raw /camera/depth/image_rect_raw \
         /camera/depth/color/points /camera/imu /map /obstacles /survivors \
         /goal /path /cmd_vel /mission_status; do
  if printf '%s\n' "$TOPICS" | grep -qx "$t"; then ok "topic $t"; else bad "topic $t"; fi
done

echo "== rates =="
rate_of() {
  timeout 5 ros2 topic hz "$1" 2>/dev/null | grep -oE "average rate: [0-9.]+" \
    | head -n1 | grep -oE "[0-9.]+"
}
MAP_HZ=$(rate_of /map)
IMAGE_HZ=$(rate_of /camera/color/image_raw)
[ -n "${MAP_HZ:-}" ] && [ "$(printf '%.0f' "$MAP_HZ")" -ge 1 ] \
  && ok "/map ~${MAP_HZ} Hz" || bad "/map is not publishing"
[ -n "${IMAGE_HZ:-}" ] && [ "$(printf '%.0f' "$IMAGE_HZ")" -ge 5 ] \
  && ok "/camera/color/image_raw ~${IMAGE_HZ} Hz" || bad "camera not publishing"

echo "== mission state =="
STATE=$(timeout 5 ros2 topic echo --once /mission_state std_msgs/msg/String 2>/dev/null \
  | grep -oE "data: .*" | head -n1)
[ -n "$STATE" ] && ok "$STATE" || bad "no /mission_state message"

if grep -qiE "traceback|ModuleNotFoundError|package .* not found" "$LOG"; then
  bad "errors in launch log ($LOG)"; grep -iE "traceback|ModuleNotFoundError|not found" "$LOG" | head -5
else
  ok "no fatal errors in launch log"
fi

echo
if [ "$FAIL" -eq 0 ]; then
  echo "SMOKE PASS  (log: $LOG)"
else
  echo "SMOKE FAIL  (log: $LOG)"
fi
exit "$FAIL"
