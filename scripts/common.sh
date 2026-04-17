#!/usr/bin/env bash

COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${COMMON_DIR}/.." && pwd)"
CONFIG_FILE="${REPO_ROOT}/configs/project.env"

resolve_repo_path() {
  local raw_path="${1:-}"
  if [[ -z "${raw_path}" ]]; then
    return 0
  fi

  if [[ "${raw_path}" = /* ]]; then
    printf '%s\n' "${raw_path}"
  else
    printf '%s\n' "${REPO_ROOT}/${raw_path}"
  fi
}

load_project_env() {
  if [[ -f "${CONFIG_FILE}" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "${CONFIG_FILE}"
    set +a
  fi

  export PX4_MUJOCO_PYTHON="${PX4_MUJOCO_PYTHON:-python3}"
  export PX4_MUJOCO_CONDA_ENV="${PX4_MUJOCO_CONDA_ENV:-px4-mujoco}"
  export PX4_MUJOCO_MODEL="${PX4_MUJOCO_MODEL:-UAV/scene_uav_delta.xml}"
  export PX4_MUJOCO_PX4_DIR="${PX4_MUJOCO_PX4_DIR:-external/PX4-Autopilot}"
  export PX4_MUJOCO_PX4_BRANCH="${PX4_MUJOCO_PX4_BRANCH:-release/1.15}"
  export PX4_MUJOCO_PX4_BUILD_DIR="${PX4_MUJOCO_PX4_BUILD_DIR:-build/px4_sitl_default}"
  export PX4_MUJOCO_PX4_AUTOSTART="${PX4_MUJOCO_PX4_AUTOSTART:-22002}"
  export PX4_MUJOCO_PX4_SIM_MODEL="${PX4_MUJOCO_PX4_SIM_MODEL:-mujoco_delta}"
  export PX4_MUJOCO_MAVLINK_HOST="${PX4_MUJOCO_MAVLINK_HOST:-0.0.0.0}"
  export PX4_MUJOCO_TCP_PORT="${PX4_MUJOCO_TCP_PORT:-4560}"
  export PX4_MUJOCO_HOVER_THRUST="${PX4_MUJOCO_HOVER_THRUST:-0.60}"
  export PX4_MUJOCO_QGC_APP="${PX4_MUJOCO_QGC_APP:-}"
  export PX4_MUJOCO_QGC_UDP_PORT="${PX4_MUJOCO_QGC_UDP_PORT:-14550}"
  export PX4_MUJOCO_ROS2_SETUP="${PX4_MUJOCO_ROS2_SETUP:-}"

  export PX4_MUJOCO_MODEL_ABS="$(resolve_repo_path "${PX4_MUJOCO_MODEL}")"
  export PX4_MUJOCO_PX4_DIR_ABS="$(resolve_repo_path "${PX4_MUJOCO_PX4_DIR}")"

  if [[ "${PX4_MUJOCO_PX4_BUILD_DIR}" = /* ]]; then
    export PX4_MUJOCO_PX4_BUILD_DIR_ABS="${PX4_MUJOCO_PX4_BUILD_DIR}"
  else
    export PX4_MUJOCO_PX4_BUILD_DIR_ABS="${PX4_MUJOCO_PX4_DIR_ABS}/${PX4_MUJOCO_PX4_BUILD_DIR}"
  fi

  if [[ -n "${PX4_MUJOCO_QGC_APP}" ]]; then
    export PX4_MUJOCO_QGC_APP_ABS="$(resolve_repo_path "${PX4_MUJOCO_QGC_APP}")"
  else
    export PX4_MUJOCO_QGC_APP_ABS=""
  fi

  export PX4_MUJOCO_PATCH_FILE="${REPO_ROOT}/integrations/px4/patches/release-1.15-mujoco-delta.patch"
  export PX4_MUJOCO_PX4_AIRFRAME_FILE="${PX4_MUJOCO_PX4_DIR_ABS}/ROMFS/px4fmu_common/init.d-posix/airframes/${PX4_MUJOCO_PX4_AUTOSTART}_mujoco_delta"
  export PX4_MUJOCO_PX4_AIRFRAME_LIST="${PX4_MUJOCO_PX4_DIR_ABS}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
  export PX4_MUJOCO_PX4_GCS_FILE="${PX4_MUJOCO_PX4_DIR_ABS}/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"
  export PX4_MUJOCO_PX4_BIN="${PX4_MUJOCO_PX4_BUILD_DIR_ABS}/bin/px4"
  export PX4_MUJOCO_PX4_ETC_DIR="${PX4_MUJOCO_PX4_BUILD_DIR_ABS}/etc"
  export PX4_MUJOCO_PX4_ROOTFS="${PX4_MUJOCO_PX4_BUILD_DIR_ABS}/rootfs"
}

require_command() {
  local cmd="${1}"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "Error: required command not found: ${cmd}" >&2
    exit 1
  fi
}

activate_conda_if_configured() {
  if [[ -z "${PX4_MUJOCO_CONDA_ENV:-}" ]]; then
    return 0
  fi

  if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck disable=SC1091
    source "${HOME}/miniconda3/etc/profile.d/conda.sh"
    conda activate "${PX4_MUJOCO_CONDA_ENV}"
  fi
}

px4_repo_exists() {
  [[ -d "${PX4_MUJOCO_PX4_DIR_ABS}/.git" ]]
}

px4_patch_applied() {
  [[ -f "${PX4_MUJOCO_PX4_AIRFRAME_FILE}" ]] || return 1
  grep -q '22002_mujoco_delta' "${PX4_MUJOCO_PX4_AIRFRAME_LIST}" || return 1
  grep -q -- '-o 14550 -t 127.0.0.1' "${PX4_MUJOCO_PX4_GCS_FILE}" || return 1
}

px4_build_ready() {
  [[ -x "${PX4_MUJOCO_PX4_BIN}" ]] || return 1
  [[ -d "${PX4_MUJOCO_PX4_ETC_DIR}" ]] || return 1
  [[ -d "${PX4_MUJOCO_PX4_ROOTFS}" ]] || return 1
}

wait_for_tcp_port() {
  local host="${1}"
  local port="${2}"
  local timeout_seconds="${3:-20}"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    if (echo >/dev/tcp/"${host}"/"${port}") >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done

  return 1
}
