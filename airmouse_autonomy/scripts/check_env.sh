#!/usr/bin/env bash
# AirMouse Goal-0 environment diagnostics (run on the Ubuntu 22.04 target box).
#
# Usage:
#   bash scripts/check_env.sh            # tool versions only
#   bash scripts/check_env.sh --topics   # also check topics of a RUNNING stack
#
# Exit code 0 only if every required tool is present (and, with --topics,
# every expected topic is being published). Runtime behavior itself is still
# verified by actually flying the 12 scenarios; this script only proves the
# environment is wired.
set -u

PASS=0
FAIL=0

ok()   { printf '  PASS  %s\n' "$1"; PASS=$((PASS + 1)); }
bad()  { printf '  FAIL  %s\n' "$1"; FAIL=$((FAIL + 1)); }
note() { printf '  NOTE  %s\n' "$1"; }

echo "== AirMouse environment diagnostics =="
echo

echo "-- OS --"
if command -v lsb_release >/dev/null 2>&1; then
  VERSION=$(lsb_release -rs 2>/dev/null || echo unknown)
  if [ "$VERSION" = "22.04" ]; then ok "Ubuntu 22.04"; else bad "expected Ubuntu 22.04, got $VERSION"; fi
else
  bad "lsb_release not found (cannot confirm Ubuntu 22.04)"
fi

echo "-- ROS 2 --"
if [ "${ROS_DISTRO:-}" = "humble" ]; then ok "ROS_DISTRO=humble"; else
  bad "ROS_DISTRO is '${ROS_DISTRO:-unset}' (expected humble; source /opt/ros/humble/setup.bash)"
fi
command -v ros2 >/dev/null 2>&1 && ok "ros2 CLI present" || bad "ros2 CLI not found"
command -v colcon >/dev/null 2>&1 && ok "colcon present" || bad "colcon not found"

echo "-- Gazebo --"
if command -v gz >/dev/null 2>&1; then
  GZ_V=$(gz sim --versions 2>/dev/null | head -n1 || true)
  ok "gz CLI present${GZ_V:+ ($GZ_V)}"
else
  bad "gz CLI not found (Gazebo Harmonic)"
fi

echo "-- PX4 + XRCE-DDS --"
if command -v MicroXRCEAgent >/dev/null 2>&1; then ok "MicroXRCEAgent present"; else
  bad "MicroXRCEAgent not found (uXRCE-DDS agent)"
fi
if [ -d "$HOME/PX4-Autopilot" ] || [ -n "${PX4_DIR:-}" ]; then
  ok "PX4-Autopilot tree found"
else
  note "PX4-Autopilot not found at \$HOME/PX4-Autopilot or \$PX4_DIR (verify existing setup)"
fi

echo "-- Workspace --"
if [ -f "$(dirname "$0")/../src/airmouse_interfaces/package.xml" ]; then
  ok "airmouse workspace layout intact"
else
  bad "cannot see src/airmouse_interfaces/package.xml (run from the repo)"
fi

if [ "${1:-}" = "--topics" ]; then
  echo "-- Topics (requires a RUNNING mission stack) --"
  if ! command -v ros2 >/dev/null 2>&1; then
    bad "ros2 not available; cannot check topics"
  else
    TOPICS=$(timeout 10 ros2 topic list 2>/dev/null || true)
    for t in /camera/color/image_raw /camera/depth/image_rect_raw \
             /camera/depth/color/points /camera/imu /map /obstacles \
             /survivors /goal /path /cmd_vel /mission_status; do
      if printf '%s\n' "$TOPICS" | grep -qx "$t"; then ok "topic $t"; else bad "topic $t missing"; fi
    done
    echo "-- Rates (2 s sample) --"
    for t in /map /camera/color/image_raw /camera/depth/image_rect_raw; do
      note "$t: $(timeout 4 ros2 topic hz "$t" 2>/dev/null | head -n2 | tr '\n' ' ')"
    done
  fi
fi

echo
printf '== %d passed, %d failed ==\n' "$PASS" "$FAIL"
[ "$FAIL" -eq 0 ] || exit 1
