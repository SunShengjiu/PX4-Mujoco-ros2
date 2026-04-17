#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env
activate_conda_if_configured

PYTHON_BIN="${PX4_MUJOCO_PYTHON}"

HAS_MODEL_ARG=0
HAS_HOST_ARG=0
HAS_PORT_ARG=0
HAS_HOVER_ARG=0
for arg in "$@"; do
  if [[ "${arg}" == "--model" ]] || [[ "${arg}" == --model=* ]]; then
    HAS_MODEL_ARG=1
  elif [[ "${arg}" == "--mavlink-host" ]] || [[ "${arg}" == --mavlink-host=* ]]; then
    HAS_HOST_ARG=1
  elif [[ "${arg}" == "--mavlink-port" ]] || [[ "${arg}" == --mavlink-port=* ]]; then
    HAS_PORT_ARG=1
  elif [[ "${arg}" == "--px4-hover-thrust" ]] || [[ "${arg}" == --px4-hover-thrust=* ]]; then
    HAS_HOVER_ARG=1
  fi
done

ARGS=("$@")

if [[ "${HAS_MODEL_ARG}" -eq 0 ]]; then
  ARGS+=(--model "${PX4_MUJOCO_MODEL_ABS}")
fi

if [[ "${HAS_HOST_ARG}" -eq 0 ]]; then
  ARGS+=(--mavlink-host "${PX4_MUJOCO_MAVLINK_HOST}")
fi

if [[ "${HAS_PORT_ARG}" -eq 0 ]]; then
  ARGS+=(--mavlink-port "${PX4_MUJOCO_TCP_PORT}")
fi

if [[ "${HAS_HOVER_ARG}" -eq 0 ]]; then
  ARGS+=(--px4-hover-thrust "${PX4_MUJOCO_HOVER_THRUST}")
fi

exec "${PYTHON_BIN}" "${REPO_ROOT}/bridge.py" "${ARGS[@]}"
