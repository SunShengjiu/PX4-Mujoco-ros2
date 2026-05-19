#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

START_QGC=1
OFFBOARD_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-qgc)
      START_QGC=0
      shift
      ;;
    --)
      shift
      OFFBOARD_ARGS+=("$@")
      break
      ;;
    *)
      OFFBOARD_ARGS+=("$1")
      shift
      ;;
  esac
done

source_ros2_env() {
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

  if [[ -f "${PX4_MUJOCO_ROS2_WS_ABS}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${PX4_MUJOCO_ROS2_WS_ABS}/install/setup.bash"
    set -u
  fi
}

px4_module_cmd() {
  local module="${1}"
  shift
  "${PX4_MUJOCO_PX4_BUILD_DIR_ABS}/bin/px4-${module}" --instance 0 "$@"
}

wait_for_ros2_topic() {
  local topic="${1}"
  local timeout_seconds="${2:-15}"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    if ros2 topic info "${topic}" -v --no-daemon >/tmp/px4_ros2_topic_info.txt 2>/dev/null; then
      if grep -Eq '^Publisher count: [1-9][0-9]*$' /tmp/px4_ros2_topic_info.txt; then
        return 0
      fi
    fi
    sleep 1
  done

  return 1
}

log_px4_process_state() {
  if kill -0 "${px4_pid}" 2>/dev/null; then
    echo "[stack-ros2] PX4 process is still alive (pid=${px4_pid})" >&2
    return 0
  fi

  echo "[stack-ros2] PX4 process exited unexpectedly." >&2
  if [[ -f /tmp/px4_stack_ros2.log ]]; then
    echo "---- PX4 log tail ----" >&2
    tail -n 120 /tmp/px4_stack_ros2.log >&2 || true
  fi
}

log_px4_uxrce_diagnostics() {
  log_px4_process_state
  echo "---- uxrce_dds_client status ----" >&2
  px4_module_cmd uxrce_dds_client status >&2 || true
  echo "---- PX4 vehicle_status listener ----" >&2
  px4_module_cmd listener vehicle_status 1 >&2 || true
  echo "---- PX4 timesync_status listener ----" >&2
  px4_module_cmd listener timesync_status 1 >&2 || true
  echo "---- PX4 vehicle_local_position listener ----" >&2
  px4_module_cmd listener vehicle_local_position 1 >&2 || true
  echo "---- PX4 estimator_status_flags listener ----" >&2
  px4_module_cmd listener estimator_status_flags 1 >&2 || true
  echo "---- ROS 2 topic list ----" >&2
  ros2 topic list --no-daemon >&2 || true
}

ensure_px4_uxrce_client() {
  if px4_module_cmd uxrce_dds_client status >/tmp/px4_uxrce_status.log 2>&1; then
    echo "[stack-ros2] PX4 uXRCE-DDS client is running"
    return 0
  fi

  echo "[stack-ros2] PX4 uXRCE-DDS client not running, starting it explicitly"
  px4_module_cmd uxrce_dds_client start -t udp -h 127.0.0.1 -p "${PX4_MUJOCO_UXRCE_DDS_PORT}" >/tmp/px4_uxrce_start.log 2>&1 || true
  sleep 2

  if px4_module_cmd uxrce_dds_client status >/tmp/px4_uxrce_status.log 2>&1; then
    echo "[stack-ros2] PX4 uXRCE-DDS client started successfully"
    return 0
  fi

  echo "Error: PX4 uXRCE-DDS client is still unavailable." >&2
  echo "---- uxrce_dds_client status ----" >&2
  cat /tmp/px4_uxrce_status.log >&2 || true
  echo "---- uxrce_dds_client start ----" >&2
  cat /tmp/px4_uxrce_start.log >&2 || true
  log_px4_uxrce_diagnostics
  return 1
}

agent_pid=0
bridge_pid=0
offboard_pid=0
qgc_pid=0

cleanup() {
  for pid in "${offboard_pid}" "${bridge_pid}" "${qgc_pid}" "${agent_pid}"; do
    if [[ "${pid}" -gt 0 ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

if ! command_exists "${PX4_MUJOCO_UXRCE_DDS_AGENT}"; then
  echo "Error: required command not found: ${PX4_MUJOCO_UXRCE_DDS_AGENT}" >&2
  echo "Install Micro XRCE-DDS Agent or set PX4_MUJOCO_UXRCE_DDS_AGENT to the correct executable." >&2
  exit 1
fi

if px4_running; then
  echo "[stack-ros2] stopping existing PX4 SITL instance"
  "${REPO_ROOT}/scripts/stop_px4.sh"
fi

if ros2_agent_running; then
  echo "[stack-ros2] stopping existing Micro XRCE-DDS Agent"
  pkill -TERM -f "${PX4_MUJOCO_UXRCE_DDS_AGENT}" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "${PX4_MUJOCO_UXRCE_DDS_AGENT}" 2>/dev/null || true
fi

rm -f "${PX4_MUJOCO_PX4_ROOTFS}/parameters.bson" "${PX4_MUJOCO_PX4_ROOTFS}/parameters_backup.bson"

echo "[stack-ros2] starting Micro XRCE-DDS Agent"
"${REPO_ROOT}/scripts/run_ros2_agent.sh" &
agent_pid=$!
sleep 1

if ! kill -0 "${agent_pid}" 2>/dev/null; then
  echo "Error: Micro XRCE-DDS Agent exited during startup." >&2
  exit 1
fi

if [[ "${START_QGC}" -eq 1 ]]; then
  if [[ -n "${PX4_MUJOCO_QGC_APP_ABS}" && -f "${PX4_MUJOCO_QGC_APP_ABS}" ]]; then
    echo "[stack-ros2] starting QGroundControl"
    "${REPO_ROOT}/scripts/run_qgc.sh" &
    qgc_pid=$!
    sleep 2
  else
    echo "[stack-ros2] QGroundControl skipped because PX4_MUJOCO_QGC_APP is unset or missing"
  fi
fi

echo "[stack-ros2] starting Python MuJoCo bridge"
rm -f "${PX4_MUJOCO_BRIDGE_READY_FILE}"
rm -f "${PX4_MUJOCO_BRIDGE_CONNECTED_FILE}"
source_ros2_env
# Keep PX4 state estimation on the proven MAVLink odometry path and use ROS 2
# only for Offboard control. This avoids duplicate EV feeds racing each other.
"${REPO_ROOT}/scripts/run_bridge.sh" --no-local-hover &
bridge_pid=$!

echo "[stack-ros2] waiting for bridge listener"
bridge_ready_file="${PX4_MUJOCO_BRIDGE_READY_FILE}"
if ! wait_for_file "${bridge_ready_file}" 30; then
  echo "Error: bridge did not enter listening state: ${bridge_ready_file}" >&2
  exit 1
fi

echo "[stack-ros2] starting PX4 SITL"
rm -f /tmp/px4_stack_ros2.log
"${REPO_ROOT}/scripts/run_px4.sh" > /tmp/px4_stack_ros2.log 2>&1 &
px4_pid=$!

sleep 1
if ! kill -0 "${px4_pid}" 2>/dev/null; then
  echo "Error: PX4 SITL exited during startup." >&2
  wait "${px4_pid}" || true
  exit 1
fi

echo "[stack-ros2] waiting for PX4 to connect to bridge"
bridge_connected_file="${PX4_MUJOCO_BRIDGE_CONNECTED_FILE}"
if ! wait_for_file "${bridge_connected_file}" 30; then
  echo "Error: PX4 did not connect to bridge within 30s: ${bridge_connected_file}" >&2
  exit 1
fi

if ! ensure_px4_uxrce_client; then
  exit 1
fi

echo "[stack-ros2] starting ROS 2 offboard control node"
"${REPO_ROOT}/scripts/run_offboard_hold.sh" "${OFFBOARD_ARGS[@]}" &
offboard_pid=$!

echo "[stack-ros2] waiting for ROS 2 PX4 status topics"
if ! wait_for_ros2_topic "/fmu/out/vehicle_status" 15; then
  echo "Warning: ROS 2 topic /fmu/out/vehicle_status did not appear after starting offboard_control." >&2
  log_px4_uxrce_diagnostics
else
  echo "[stack-ros2] ROS 2 topic /fmu/out/vehicle_status is visible"
fi

if ! wait_for_ros2_topic "/fmu/out/timesync_status" 15; then
  echo "Warning: ROS 2 topic /fmu/out/timesync_status did not appear after starting offboard_control." >&2
  log_px4_uxrce_diagnostics
else
  echo "[stack-ros2] ROS 2 topic /fmu/out/timesync_status is visible"
fi

if ! wait_for_ros2_topic "/fmu/out/vehicle_local_position" 15; then
  echo "Warning: ROS 2 topic /fmu/out/vehicle_local_position did not appear after starting offboard_control." >&2
  log_px4_uxrce_diagnostics
else
  echo "[stack-ros2] ROS 2 topic /fmu/out/vehicle_local_position is visible"
fi

if ! wait_for_ros2_topic "/fmu/out/vehicle_odometry" 15; then
  echo "Warning: ROS 2 topic /fmu/out/vehicle_odometry did not appear after starting offboard_control." >&2
  log_px4_uxrce_diagnostics
else
  echo "[stack-ros2] ROS 2 topic /fmu/out/vehicle_odometry is visible"
fi

wait "${px4_pid}"
