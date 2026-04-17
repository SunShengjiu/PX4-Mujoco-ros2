#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

require_command make

if ! px4_repo_exists; then
  echo "Error: PX4 repository not found: ${PX4_MUJOCO_PX4_DIR_ABS}" >&2
  echo "Run ./scripts/bootstrap_workspace.sh first." >&2
  exit 1
fi

if ! px4_patch_applied; then
  echo "Error: PX4 patch is not applied yet." >&2
  echo "Run ./scripts/apply_px4_patch.sh first." >&2
  exit 1
fi

echo "[px4-build] building px4_sitl_default"
make -C "${PX4_MUJOCO_PX4_DIR_ABS}" px4_sitl_default

if ! px4_build_ready; then
  echo "Error: PX4 build finished without expected SITL outputs." >&2
  exit 1
fi

echo "[px4-build] build artifacts ready at ${PX4_MUJOCO_PX4_BUILD_DIR_ABS}"
