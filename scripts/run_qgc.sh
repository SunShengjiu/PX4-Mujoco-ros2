#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

if [[ -z "${PX4_MUJOCO_QGC_APP_ABS}" ]]; then
  echo "Error: PX4_MUJOCO_QGC_APP is not set." >&2
  exit 1
fi

if [[ ! -f "${PX4_MUJOCO_QGC_APP_ABS}" ]]; then
  echo "Error: QGroundControl AppImage not found: ${PX4_MUJOCO_QGC_APP_ABS}" >&2
  exit 1
fi

if [[ ! -x "${PX4_MUJOCO_QGC_APP_ABS}" ]]; then
  chmod +x "${PX4_MUJOCO_QGC_APP_ABS}"
fi

exec "${PX4_MUJOCO_QGC_APP_ABS}" "$@"
