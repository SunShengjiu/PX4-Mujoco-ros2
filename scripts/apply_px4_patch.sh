#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

CHECK_ONLY=0
if [[ "${1:-}" == "--check" ]]; then
  CHECK_ONLY=1
elif [[ $# -gt 0 ]]; then
  echo "usage: $0 [--check]" >&2
  exit 1
fi

require_command git

if ! px4_repo_exists; then
  echo "Error: PX4 repository not found: ${PX4_MUJOCO_PX4_DIR_ABS}" >&2
  echo "Run ./scripts/bootstrap_workspace.sh first." >&2
  exit 1
fi

if [[ ! -f "${PX4_MUJOCO_PATCH_FILE}" ]]; then
  echo "Error: patch file not found: ${PX4_MUJOCO_PATCH_FILE}" >&2
  exit 1
fi

CURRENT_BRANCH="$(git -C "${PX4_MUJOCO_PX4_DIR_ABS}" branch --show-current)"
if [[ "${CURRENT_BRANCH}" != "${PX4_MUJOCO_PX4_BRANCH}" ]]; then
  echo "Error: PX4 branch mismatch." >&2
  echo "Expected: ${PX4_MUJOCO_PX4_BRANCH}" >&2
  echo "Current:  ${CURRENT_BRANCH:-detached}" >&2
  exit 1
fi

if px4_patch_applied; then
  echo "[px4-patch] already applied"
  exit 0
fi

if ! git -C "${PX4_MUJOCO_PX4_DIR_ABS}" diff --quiet || ! git -C "${PX4_MUJOCO_PX4_DIR_ABS}" diff --cached --quiet; then
  echo "Error: PX4 working tree is not clean. Commit or stash changes before patching." >&2
  exit 1
fi

if ! git -C "${PX4_MUJOCO_PX4_DIR_ABS}" apply --check "${PX4_MUJOCO_PATCH_FILE}" >/dev/null 2>&1; then
  echo "Error: patch cannot be applied cleanly to ${PX4_MUJOCO_PX4_DIR_ABS}" >&2
  echo "Make sure you are using a clean ${PX4_MUJOCO_PX4_BRANCH} clone." >&2
  exit 1
fi

if [[ "${CHECK_ONLY}" -eq 1 ]]; then
  echo "[px4-patch] patch is applicable"
  exit 0
fi

git -C "${PX4_MUJOCO_PX4_DIR_ABS}" apply "${PX4_MUJOCO_PATCH_FILE}"

if ! px4_patch_applied; then
  echo "Error: patch post-check failed." >&2
  exit 1
fi

echo "[px4-patch] applied successfully"
