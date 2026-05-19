#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

if ! px4_patch_applied; then
  echo "Error: PX4 patch is not applied yet." >&2
  echo "Run ./scripts/apply_px4_patch.sh first." >&2
  exit 1
fi

if ! px4_build_ready; then
  echo "Error: PX4 SITL build is missing." >&2
  echo "Run ./scripts/build_px4.sh first." >&2
  exit 1
fi

mkdir -p "${PX4_MUJOCO_PX4_ROOTFS}"

PX4_ARGS=()
if [[ -n "${NO_PXH:-}" ]]; then
  PX4_ARGS+=("-d")
fi

pushd "${PX4_MUJOCO_PX4_ROOTFS}" >/dev/null
export PX4_SYS_AUTOSTART="${PX4_MUJOCO_PX4_AUTOSTART}"
export PX4_SIMULATOR="mujoco"
export PX4_SIM_MODEL="${PX4_MUJOCO_PX4_SIM_MODEL}"
export PX4_UXRCE_DDS_PORT="${PX4_MUJOCO_UXRCE_DDS_PORT}"
export PX4_MUJOCO_ACTUATOR_MAVLINK_PORT="${PX4_MUJOCO_ACTUATOR_MAVLINK_PORT}"
exec "${PX4_MUJOCO_PX4_BIN}" "${PX4_ARGS[@]}" "$@" "${PX4_MUJOCO_PX4_ETC_DIR}"
