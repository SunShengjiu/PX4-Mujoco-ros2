# Current Status

## Already Implemented

- `bridge.py` defaults to the repository-local model `UAV/scene_uav_delta.xml`
- `bridge.py` also includes a bridge-local takeoff/hover mode used by `make run-local`
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
  - `build_ros2.sh`
  - `run_bridge.sh`
  - `run_px4.sh`
  - `run_qgc.sh`
  - `run_stack.sh`
  - `run_stack_ros2.sh`
  - `run_ros2_agent.sh`
  - `run_offboard_hold.sh`
- The repository includes a ROS 2 workspace and package for PX4 Offboard control:
  - `ros2_ws/src/px4_mujoco_ros2_control`
- `run-stack` and `run-stack-ros2` explicitly start the bridge with `--no-local-hover` so PX4 remains the only flight controller in those paths
- The official PX4 patch is stored in:
  - `integrations/px4/patches/release-1.15-mujoco-delta.patch`

## Current Validation Level

These parts have been validated locally:

- shell script syntax
- Python syntax compilation
- model contract checks
- local bridge smoke run with `--no-mavlink`
- direct local MuJoCo auto takeoff/hover visual path through `make run-local`
- PX4 patch applicability with `git apply --check`
- ROS 2 workspace build
- PX4 ROS 2 message-chain visibility up to topics such as:
  - `/fmu/out/vehicle_status`
  - `/fmu/out/timesync_status`
  - `/fmu/out/vehicle_local_position`
  - `/fmu/out/vehicle_odometry`
- PX4 Offboard requests reaching PX4 through the ROS 2 pipeline, with the hover-fallback and external-command interface implemented in `offboard_control.py`

## Still Pending

The remaining work is not basic repository structure anymore. It is end-to-end stabilization and verification:

1. stabilize PX4 estimator initialization in `make run-stack-ros2`
2. confirm stable PX4 arm, takeoff, hover, and hold in MuJoCo through the Offboard path
3. verify the delta-arm scene and plain UAV scene both behave correctly under the PX4-controlled stack
4. harden startup cleanup around stale PX4 and Micro XRCE-DDS Agent processes where needed
