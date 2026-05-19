#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PX4_MUJOCO_MODEL="UAV/scene.xml"
export PX4_MUJOCO_PX4_AUTOSTART="22003"
export PX4_MUJOCO_HOVER_THRUST="0.45"

exec "${SCRIPT_DIR}/run_stack.sh" "$@"
