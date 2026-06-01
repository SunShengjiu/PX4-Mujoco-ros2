# Architecture

The repository currently exposes three runtime paths:

1. local MuJoCo hover demo
2. PX4 + MuJoCo stack without the manipulator arm
3. PX4 + MuJoCo stack with the delta arm scene

## Data Flows

### Local visual hover

```text
MuJoCo <- Python bridge local-hover controller
```

This path is launched by `make run-local`. It does not use PX4.

### PX4 stack

```text
QGroundControl <- UDP 14550 <- PX4 SITL <- TCP 4560 -> Python bridge <-> MuJoCo
```

This is the control path used by `make run-stack-*`.

### PX4 + ROS 2 Offboard stack

```text
QGroundControl <- UDP 14550 <- PX4 SITL <- TCP 4560 -> Python bridge <-> MuJoCo
uav_control -> ROS 2 cmd_pose/cmd_twist -> offboard_control -> px4_msgs -> Micro XRCE-DDS Agent <- UDP 8888 <- PX4 SITL
```

This is the message chain used by the separated ROS 2 startup flow: Agent, bridge, PX4, `offboard_control`, then an upper-layer `uav_control` node.

## Responsibilities

- `bridge.py`
  - drives MuJoCo
  - converts MuJoCo state into PX4-facing MAVLink HIL messages
  - applies PX4 actuator outputs back into MuJoCo
  - can also run a bridge-local hover controller for the non-PX4 demo path

- `UAV/`
  - stores the MuJoCo scenes and model assets
  - currently includes both the plain UAV scene and the delta-arm scene

- `scripts/`
  - owns setup, patching, build, validation, and launch orchestration
  - provides separate local, PX4, Agent, bridge, and ROS 2 node startup entry points

- `ros2_ws/`
  - stores `px4_msgs`, the Offboard gateway package, the separated bringup package, and `uav_control`
  - keeps PX4-facing gateway code in `px4_mujoco_ros2_control`
  - keeps launch files in `px4_mujoco_ros2_bringup`
  - keeps hover, waypoint cruise, and future planning nodes in `uav_control`

- `integrations/px4/`
  - stores the minimal PX4 patch required by this repository

## Control Ownership Rules

1. `make run-local` uses the bridge-local hover controller.
2. PX4 and ROS 2 Offboard runs start the bridge with `--no-local-hover`.
3. In PX4 paths, PX4 remains the only flight controller in the loop.
4. `offboard_control` only forwards active external setpoints and does not own mission behavior.
5. `uav_control` owns mission behavior and publishes generic ROS 2 commands.

## Why This Shape

The goal is to keep one codebase but make the three user-facing paths obvious:

- local mode for quick visual confirmation
- plain UAV mode for simpler PX4 stack validation
- delta-arm mode for the final integrated scene
