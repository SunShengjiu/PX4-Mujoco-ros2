#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

WORKSPACE_DIR="${REPO_ROOT}/.workspace"

mkdir -p "${REPO_ROOT}/external"
mkdir -p "${WORKSPACE_DIR}/downloads"
mkdir -p "${WORKSPACE_DIR}/logs"

echo "[workspace] repo root: ${REPO_ROOT}"
echo "[workspace] px4 dir:   ${PX4_MUJOCO_PX4_DIR_ABS}"
echo "[workspace] px4 branch:${PX4_MUJOCO_PX4_BRANCH}"

if [[ -d "${PX4_MUJOCO_PX4_DIR_ABS}/.git" ]]; then
  echo "[px4] existing repository detected, skip clone"
else
  echo "[px4] cloning PX4-Autopilot"
  git clone --recursive --branch "${PX4_MUJOCO_PX4_BRANCH}" https://github.com/PX4/PX4-Autopilot.git "${PX4_MUJOCO_PX4_DIR_ABS}"
fi

echo
echo "Bootstrap completed."
echo "Next checks:"
echo "  1. ./scripts/install_ubuntu_deps.sh"
echo "  2. ./scripts/apply_px4_patch.sh"
echo "  3. ./scripts/build_px4.sh"
echo "  4. ./scripts/doctor.py"
