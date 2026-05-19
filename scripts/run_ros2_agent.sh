#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

require_command "${PX4_MUJOCO_UXRCE_DDS_AGENT}"

exec "${PX4_MUJOCO_UXRCE_DDS_AGENT}" udp4 -p "${PX4_MUJOCO_UXRCE_DDS_PORT}"
