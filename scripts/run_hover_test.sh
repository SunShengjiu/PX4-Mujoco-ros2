#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/px4_mujoco_ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"

if [[ -n "${PX4_MUJOCO_ROS2_SETUP}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${PX4_MUJOCO_ROS2_SETUP}"
  set -u
elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/humble/setup.bash"
  set -u
elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/jazzy/setup.bash"
  set -u
else
  echo "Error: ROS 2 setup file not found." >&2
  echo "Set PX4_MUJOCO_ROS2_SETUP=/opt/ros/<distro>/setup.bash in configs/project.env." >&2
  exit 1
fi

if [[ ! -f "${PX4_MUJOCO_ROS2_WS_ABS}/install/setup.bash" ]]; then
  echo "Error: ROS 2 workspace is not built yet." >&2
  echo "Run ./scripts/build_ros2.sh first." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source "${PX4_MUJOCO_ROS2_WS_ABS}/install/setup.bash"
set -u

exec ros2 run uav_control hover_test "$@"
