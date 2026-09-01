#!/usr/bin/env bash
set -Eeuo pipefail

# Reproduce the verified AirMouse simulation on Ubuntu 24.04.
# This script installs development dependencies, PX4 v1.17.0, ROS 2 Jazzy,
# Gazebo dependencies, px4_msgs, and the Micro XRCE-DDS Agent, then builds the
# project workspace. It does not install or configure physical-flight hardware.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LINUX_USER="${USER:-$(id -un)}"
PX4_ROOT="${AIR_MOUSE_PX4_ROOT:-/home/${LINUX_USER}/PX4-Autopilot}"
ROS_WS="${AIR_MOUSE_ROS_WS:-/home/${LINUX_USER}/ros2_px4_ws}"
AGENT_SRC="${AIR_MOUSE_XRCE_SOURCE:-/home/${LINUX_USER}/Micro-XRCE-DDS-Agent}"
AGENT_WS="${AIR_MOUSE_XRCE_WS:-/home/${LINUX_USER}/px4_ros_uxrce_dds_ws}"
PX4_VERSION="${PX4_VERSION:-v1.17.0}"
XRCE_VERSION="${XRCE_VERSION:-v2.4.3}"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

[[ "${EUID}" -ne 0 ]] || die "Run this script as your normal user, not with sudo. It asks for sudo when needed."
[[ -f /etc/os-release ]] || die "Cannot identify the operating system."
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == "ubuntu" ]] || die "This installer supports Ubuntu 24.04 only. Detected: ${ID:-unknown}."
[[ "${VERSION_ID:-}" == "24.04" ]] || die "Use Ubuntu 24.04 for the verified Jazzy setup. Detected: ${VERSION_ID:-unknown}."

require_command sudo
require_command curl
require_command git

echo "Installing base tools..."
sudo apt-get update
sudo apt-get install -y \
  ca-certificates \
  build-essential \
  cmake \
  curl \
  git \
  gnupg \
  lsb-release \
  ninja-build \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-pytest \
  python3-rosdep \
  python3-vcstool \
  python3-venv \
  software-properties-common \
  unzip \
  wget \
  zip \
  libasio-dev \
  libfastcdr-dev \
  libfoonathan-memory-dev \
  libtinyxml2-dev \
  libyaml-cpp-dev \
  libssl-dev

sudo add-apt-repository --yes universe

echo "Configuring the ROS 2 apt repository..."
ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | \
  sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p' | head -n 1)"
[[ -n "${ROS_APT_SOURCE_VERSION}" ]] || die "Could not determine the latest ros-apt-source release."
ROS_APT_DEB="/tmp/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.deb"
curl -fL -o "${ROS_APT_DEB}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i "${ROS_APT_DEB}"
sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz-bridge

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "Cloning or updating PX4 ${PX4_VERSION}..."
if [[ -d "${PX4_ROOT}/.git" ]]; then
  [[ -z "$(git -C "${PX4_ROOT}" status --porcelain)" ]] || \
    die "PX4 checkout has local changes: ${PX4_ROOT}"
  git -C "${PX4_ROOT}" fetch --tags --prune origin
  git -C "${PX4_ROOT}" checkout "${PX4_VERSION}"
  git -C "${PX4_ROOT}" submodule update --init --recursive
elif [[ ! -e "${PX4_ROOT}" ]]; then
  git clone --branch "${PX4_VERSION}" --recursive https://github.com/PX4/PX4-Autopilot.git "${PX4_ROOT}"
else
  die "PX4 path exists but is not a Git checkout: ${PX4_ROOT}"
fi

echo "Installing the PX4 Ubuntu toolchain and Gazebo dependencies..."
bash "${PX4_ROOT}/Tools/setup/ubuntu.sh"

echo "Cloning or updating Micro XRCE-DDS Agent ${XRCE_VERSION}..."
if [[ -d "${AGENT_SRC}/.git" ]]; then
  [[ -z "$(git -C "${AGENT_SRC}" status --porcelain)" ]] || \
    die "Micro XRCE-DDS Agent checkout has local changes: ${AGENT_SRC}"
  git -C "${AGENT_SRC}" fetch --tags --prune origin
  git -C "${AGENT_SRC}" checkout "${XRCE_VERSION}"
elif [[ ! -e "${AGENT_SRC}" ]]; then
  git clone --branch "${XRCE_VERSION}" --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "${AGENT_SRC}"
else
  die "Agent path exists but is not a Git checkout: ${AGENT_SRC}"
fi

cmake -S "${AGENT_SRC}" -B "${AGENT_WS}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${AGENT_WS}/install"
cmake --build "${AGENT_WS}/build" --parallel "$(nproc)"
cmake --install "${AGENT_WS}/build"

echo "Preparing the ROS 2 workspace..."
mkdir -p "${ROS_WS}/src"
PX4_MSGS_ROOT="${ROS_WS}/src/px4_msgs"
if [[ -d "${PX4_MSGS_ROOT}/.git" ]]; then
  [[ -z "$(git -C "${PX4_MSGS_ROOT}" status --porcelain)" ]] || \
    die "px4_msgs checkout has local changes: ${PX4_MSGS_ROOT}"
  git -C "${PX4_MSGS_ROOT}" fetch --prune origin
  git -C "${PX4_MSGS_ROOT}" checkout release/1.17
  git -C "${PX4_MSGS_ROOT}" pull --ff-only origin release/1.17
elif [[ ! -e "${PX4_MSGS_ROOT}" ]]; then
  git clone --branch release/1.17 --depth 1 https://github.com/PX4/px4_msgs.git "${PX4_MSGS_ROOT}"
else
  die "px4_msgs path exists but is not a Git checkout: ${PX4_MSGS_ROOT}"
fi

mkdir -p "${ROS_WS}/src/nidar_airmouse_sim"
cp -a "${SCRIPT_DIR}/nidar_airmouse_ros/." "${ROS_WS}/src/nidar_airmouse_sim/"

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths "${ROS_WS}/src" --ignore-src --rosdistro jazzy -r -y
colcon build \
  --base-paths "${ROS_WS}/src" \
  --build-base "${ROS_WS}/build" \
  --install-base "${ROS_WS}/install"

if ! grep -Fq "# NIDAR AirMouse ROS 2" "${HOME}/.bashrc" 2>/dev/null; then
  {
    echo
    echo "# NIDAR AirMouse ROS 2"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "source ${ROS_WS}/install/setup.bash"
  } >> "${HOME}/.bashrc"
fi

echo
echo "Ubuntu AirMouse setup completed."
echo "PX4:        ${PX4_ROOT} (${PX4_VERSION})"
echo "ROS 2:      ${ROS_WS}"
echo "XRCE Agent: ${AGENT_WS}/install/bin/MicroXRCEAgent (${XRCE_VERSION})"
echo
echo "Validate the maze:"
echo "  cd ${SCRIPT_DIR} && python3 tools/validate_maze.py"
echo "Run the simulation:"
echo "  cd ${SCRIPT_DIR} && AIR_MOUSE_PROFILE=fast AIR_MOUSE_GUI=1 bash ./run_airmouse_sim.sh"
echo "Run accurate mode:"
echo "  cd ${SCRIPT_DIR} && AIR_MOUSE_PROFILE=accurate AIR_MOUSE_GUI=1 bash ./run_airmouse_sim.sh"
