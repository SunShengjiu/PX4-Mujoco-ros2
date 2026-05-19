#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

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

require_command colcon

cd "${PX4_MUJOCO_ROS2_WS_ABS}"

PX4_MSGS_PKG_DIR="${PX4_MUJOCO_ROS2_WS_ABS}/src/px4_msgs"
PX4_MSG_SRC_DIR="${PX4_MUJOCO_PX4_DIR_ABS}/msg"
PX4_SRV_SRC_DIR="${PX4_MUJOCO_PX4_DIR_ABS}/srv"

if [[ -d "${PX4_MSGS_PKG_DIR}" && -d "${PX4_MSG_SRC_DIR}" && -d "${PX4_SRV_SRC_DIR}" ]]; then
  echo "[ros2-build] syncing px4_msgs definitions from ${PX4_MUJOCO_PX4_DIR_ABS}"
  rm -f "${PX4_MSGS_PKG_DIR}/msg/"*.msg
  rm -f "${PX4_MSGS_PKG_DIR}/srv/"*.srv
  cp "${PX4_MSG_SRC_DIR}/"*.msg "${PX4_MSGS_PKG_DIR}/msg/"
  cp "${PX4_SRV_SRC_DIR}/"*.srv "${PX4_MSGS_PKG_DIR}/srv/"
fi

echo "[ros2-build] clearing stale px4_msgs and control-package build artifacts"
rm -rf \
  "${PX4_MUJOCO_ROS2_WS_ABS}/build/px4_msgs" \
  "${PX4_MUJOCO_ROS2_WS_ABS}/build/px4_mujoco_ros2_control" \
  "${PX4_MUJOCO_ROS2_WS_ABS}/install/px4_msgs" \
  "${PX4_MUJOCO_ROS2_WS_ABS}/install/px4_mujoco_ros2_control"

colcon build \
  --symlink-install \
  --packages-select px4_msgs px4_mujoco_ros2_control \
  --cmake-force-configure
