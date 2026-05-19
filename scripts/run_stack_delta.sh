#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PX4_MUJOCO_MODEL="UAV/scene_uav_delta.xml"

exec "${SCRIPT_DIR}/run_stack.sh" "$@"
