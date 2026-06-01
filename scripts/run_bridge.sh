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
HAS_ACTUATOR_PORT_ARG=0
HAS_HOVER_ARG=0
HAS_PHYSICS_SUBSTEPS_ARG=0
HAS_LOCAL_HOVER_ARG=0
HAS_NO_LOCAL_HOVER_ARG=0
HAS_LOCAL_HOVER_TARGET_Z_ARG=0
HAS_LOCAL_HOVER_RAMP_SECONDS_ARG=0
for arg in "$@"; do
  if [[ "${arg}" == "--model" ]] || [[ "${arg}" == --model=* ]]; then
    HAS_MODEL_ARG=1
  elif [[ "${arg}" == "--mavlink-host" ]] || [[ "${arg}" == --mavlink-host=* ]]; then
    HAS_HOST_ARG=1
  elif [[ "${arg}" == "--mavlink-port" ]] || [[ "${arg}" == --mavlink-port=* ]]; then
    HAS_PORT_ARG=1
  elif [[ "${arg}" == "--actuator-mavlink-port" ]] || [[ "${arg}" == --actuator-mavlink-port=* ]]; then
    HAS_ACTUATOR_PORT_ARG=1
  elif [[ "${arg}" == "--px4-hover-thrust" ]] || [[ "${arg}" == --px4-hover-thrust=* ]]; then
    HAS_HOVER_ARG=1
  elif [[ "${arg}" == "--physics-substeps-per-sensor" ]] || [[ "${arg}" == --physics-substeps-per-sensor=* ]]; then
    HAS_PHYSICS_SUBSTEPS_ARG=1
  elif [[ "${arg}" == "--local-hover" ]]; then
    HAS_LOCAL_HOVER_ARG=1
  elif [[ "${arg}" == "--no-local-hover" ]]; then
    HAS_NO_LOCAL_HOVER_ARG=1
  elif [[ "${arg}" == "--local-hover-target-z" ]] || [[ "${arg}" == --local-hover-target-z=* ]]; then
    HAS_LOCAL_HOVER_TARGET_Z_ARG=1
  elif [[ "${arg}" == "--local-hover-ramp-seconds" ]] || [[ "${arg}" == --local-hover-ramp-seconds=* ]]; then
    HAS_LOCAL_HOVER_RAMP_SECONDS_ARG=1
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

if [[ "${HAS_ACTUATOR_PORT_ARG}" -eq 0 ]]; then
  ARGS+=(--actuator-mavlink-port "${PX4_MUJOCO_ACTUATOR_MAVLINK_PORT}")
fi

if [[ "${HAS_HOVER_ARG}" -eq 0 ]]; then
  ARGS+=(--px4-hover-thrust "${PX4_MUJOCO_HOVER_THRUST}")
fi

if [[ "${HAS_PHYSICS_SUBSTEPS_ARG}" -eq 0 ]]; then
  ARGS+=(--physics-substeps-per-sensor "${PX4_MUJOCO_PHYSICS_SUBSTEPS_PER_SENSOR:-10}")
fi

if [[ "${HAS_LOCAL_HOVER_ARG}" -eq 0 && "${HAS_NO_LOCAL_HOVER_ARG}" -eq 0 && "${PX4_MUJOCO_LOCAL_HOVER:-0}" == "1" ]]; then
  ARGS+=(--local-hover)
fi

if [[ "${HAS_LOCAL_HOVER_TARGET_Z_ARG}" -eq 0 && -n "${PX4_MUJOCO_LOCAL_HOVER_TARGET_Z:-}" ]]; then
  ARGS+=(--local-hover-target-z "${PX4_MUJOCO_LOCAL_HOVER_TARGET_Z}")
fi

if [[ "${HAS_LOCAL_HOVER_RAMP_SECONDS_ARG}" -eq 0 && -n "${PX4_MUJOCO_LOCAL_HOVER_RAMP_SECONDS:-}" ]]; then
  ARGS+=(--local-hover-ramp-seconds "${PX4_MUJOCO_LOCAL_HOVER_RAMP_SECONDS}")
fi

if [[ -n "${PX4_MUJOCO_BRIDGE_READY_FILE:-}" ]]; then
  ARGS+=(--ready-file "${PX4_MUJOCO_BRIDGE_READY_FILE}")
fi

if [[ -n "${PX4_MUJOCO_BRIDGE_CONNECTED_FILE:-}" ]]; then
  ARGS+=(--connected-file "${PX4_MUJOCO_BRIDGE_CONNECTED_FILE}")
fi

if [[ -n "${PX4_MUJOCO_BRIDGE_EXTRA_ARGS:-}" ]]; then
  # Intentionally split on shell words so debug flags can be injected from env.
  # shellcheck disable=SC2206
  EXTRA_ARGS=(${PX4_MUJOCO_BRIDGE_EXTRA_ARGS})
  ARGS+=("${EXTRA_ARGS[@]}")
fi

exec "${PYTHON_BIN}" "${REPO_ROOT}/bridge.py" "${ARGS[@]}"
