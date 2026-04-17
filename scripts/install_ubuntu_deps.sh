#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_project_env

require_command git
require_command bash
require_command python3

if ! px4_repo_exists; then
  echo "Error: PX4 repository not found: ${PX4_MUJOCO_PX4_DIR_ABS}" >&2
  echo "Run ./scripts/bootstrap_workspace.sh first." >&2
  exit 1
fi

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "Error: this helper currently supports Ubuntu Linux only." >&2
  exit 1
fi

echo "[deps] installing Ubuntu packages for Python MuJoCo runtime"
sudo apt-get update -y
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  libgl1 \
  libglew2.2 \
  libglfw3 \
  libxcursor1 \
  libxinerama1 \
  libxrandr2 \
  libxi6 \
  python3-pip \
  python3-venv

echo "[deps] installing PX4 common build dependencies"
bash "${PX4_MUJOCO_PX4_DIR_ABS}/Tools/setup/ubuntu.sh" --no-nuttx --no-sim-tools

echo "[deps] installing Python dependencies for this workspace"
activate_conda_if_configured
"${PX4_MUJOCO_PYTHON}" -m pip install -r "${REPO_ROOT}/requirements.txt"

echo
echo "Dependency setup completed."
