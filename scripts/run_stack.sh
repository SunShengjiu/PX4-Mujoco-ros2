#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

START_QGC=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-qgc)
      START_QGC=0
      shift
      ;;
    *)
      echo "usage: $0 [--no-qgc]" >&2
      exit 1
      ;;
  esac
done

bridge_pid=0
qgc_pid=0

cleanup() {
  if [[ "${bridge_pid}" -gt 0 ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
    kill "${bridge_pid}" 2>/dev/null || true
    wait "${bridge_pid}" 2>/dev/null || true
  fi

  if [[ "${qgc_pid}" -gt 0 ]] && kill -0 "${qgc_pid}" 2>/dev/null; then
    kill "${qgc_pid}" 2>/dev/null || true
    wait "${qgc_pid}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if px4_running; then
  echo "[stack] stopping existing PX4 SITL instance"
  "${REPO_ROOT}/scripts/stop_px4.sh"
fi

if [[ "${START_QGC}" -eq 1 ]]; then
  if [[ -n "${PX4_MUJOCO_QGC_APP_ABS}" && -f "${PX4_MUJOCO_QGC_APP_ABS}" ]]; then
    echo "[stack] starting QGroundControl"
    "${REPO_ROOT}/scripts/run_qgc.sh" &
    qgc_pid=$!
    sleep 2
  else
    echo "[stack] QGroundControl skipped because PX4_MUJOCO_QGC_APP is unset or missing"
  fi
fi

echo "[stack] starting Python MuJoCo bridge"
rm -f "${PX4_MUJOCO_BRIDGE_READY_FILE}"
rm -f "${PX4_MUJOCO_BRIDGE_CONNECTED_FILE}"
"${REPO_ROOT}/scripts/run_bridge.sh" --no-local-hover &
bridge_pid=$!

echo "[stack] waiting for bridge listener"
bridge_ready_file="${PX4_MUJOCO_BRIDGE_READY_FILE}"
if ! wait_for_file "${bridge_ready_file}" 30; then
  echo "Error: bridge did not enter listening state: ${bridge_ready_file}" >&2
  exit 1
fi

echo "[stack] starting PX4 SITL"
"${REPO_ROOT}/scripts/run_px4.sh" &
px4_pid=$!

echo "[stack] waiting for PX4 to connect to bridge"
bridge_connected_file="${PX4_MUJOCO_BRIDGE_CONNECTED_FILE}"
if ! wait_for_file "${bridge_connected_file}" 30; then
  echo "Error: PX4 did not connect to bridge within 30s: ${bridge_connected_file}" >&2
  exit 1
fi

wait "${px4_pid}"
