# Current Status

## Already Implemented

- `bridge.py` defaults to the repository-local model `UAV/scene_uav_delta.xml`
- `UAV/UAV.xml` and `UAV/UAV_Delta.xml` include the required HIL sensors:
  - `body_gyro`
  - `body_linacc`
  - `body_mag`
- Flight actuators are explicitly named `motor_1..motor_4`
- Flight motor order is aligned with PX4 Quad X:
  - front-left
  - rear-right
  - front-right
  - rear-left
- Delta manipulator actuators remain in the model, but PX4 output is mapped only to the flight motors
- The repository includes a complete first-release script set:
  - `bootstrap_workspace.sh`
  - `install_ubuntu_deps.sh`
  - `apply_px4_patch.sh`
  - `build_px4.sh`
  - `run_bridge.sh`
  - `run_px4.sh`
  - `run_qgc.sh`
  - `run_stack.sh`
- The official PX4 patch is stored in:
  - `integrations/px4/patches/release-1.15-mujoco-delta.patch`

## Current Validation Level

These parts have been validated locally:

- shell script syntax
- Python syntax compilation
- model contract checks
- local bridge smoke run with `--no-mavlink`
- PX4 patch applicability with `git apply --check`

## Still Pending

The remaining work is not repository structure anymore. It is end-to-end tuning and verification:

1. validate the full workflow on a clean Ubuntu machine
2. tune hover thrust, rotor geometry, and reaction torque
3. confirm stable arm, takeoff, hover, and landing through QGroundControl
4. only after that, extend the stack toward ROS 2 mission logic
