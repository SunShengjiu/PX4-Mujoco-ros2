#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

lock_file="/tmp/px4_lock-0"
socket_file="/tmp/px4-sock-0"

echo "[px4-stop] stopping PX4 SITL processes for this workspace"
pkill -TERM -f "${PX4_MUJOCO_PX4_BIN}" 2>/dev/null || true
sleep 1
pkill -KILL -f "${PX4_MUJOCO_PX4_BIN}" 2>/dev/null || true

if ! pgrep -f "${PX4_MUJOCO_PX4_BIN}" >/dev/null 2>&1; then
  rm -f "${lock_file}" "${socket_file}"
  echo "[px4-stop] cleared ${lock_file} and ${socket_file}"
else
  echo "Warning: PX4 process still appears to be running." >&2
  echo "Check with: pgrep -af '${PX4_MUJOCO_PX4_BIN}'" >&2
  exit 1
fi
