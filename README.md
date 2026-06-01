# PX4 + MuJoCo + ROS 2

This repository packages a PX4 SITL + MuJoCo simulation stack with:

- a Python HIL bridge between PX4 and MuJoCo
- a plain UAV scene and a delta-arm scene
- a ROS 2 Offboard gateway plus separate upper-layer control nodes
- reproducible scripts for build, launch, and validation

The recommended ROS 2 hover path is started from several terminals so each layer can be inspected independently.

## What Works

- `make run-local` gives a quick MuJoCo-only hover demo
- `make run-stack-uav` runs the plain UAV with PX4 in control
- manual ROS 2 startup adds Offboard control with separate Agent, bridge, PX4, gateway, and upper-layer control terminals
- `make run-stack-delta` runs the arm scene; ROS 2 Offboard can also be started manually with the same terminal split

## Quick Start

1. Set up your PX4 tree and QGroundControl.
2. Copy `configs/project.env.example` to `configs/project.env`.
3. Point `PX4_MUJOCO_PX4_DIR` at your PX4 checkout.
4. Install deps with `make deps-ubuntu`.
5. Apply the PX4 patch with `make px4-patch`.
6. Build ROS 2 with `make ros2-build`.
7. Build PX4 with `make px4-build`.
8. Validate with `make doctor`.

Typical first checks:

```bash
make run-local
make run-stack-uav
```

## Run Modes

### Local hover

```bash
make run-local
```

MuJoCo only. No PX4. Good for fast visual checks.

### Plain UAV with PX4

```bash
make run-stack-uav
```

PX4 owns flight control. No manipulator arm.

### Plain UAV with ROS 2 Offboard

Use five separate terminals for ROS 2 Offboard so DDS, MuJoCo bridge, PX4, the Offboard gateway, and the upper-layer controller can be debugged independently.

The five terminals are intentionally split:

- Terminal 1 is the PX4 <-> ROS 2 DDS transport.
- Terminal 2 is the MuJoCo physics bridge.
- Terminal 3 is PX4 SITL, the flight controller.
- Terminal 4 is the generic ROS 2 Offboard gateway.
- Terminal 5 is your upper-layer control node, such as hover or waypoint cruise.

Terminal 1, Micro XRCE-DDS Agent:

```bash
make run-ros2-agent
```

Terminal 2, MuJoCo bridge:

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
PX4_MUJOCO_BRIDGE_EXTRA_ARGS="--px4-alignment-hold-seconds 2.0 --debug-hil-rate" \
./scripts/run_bridge.sh --no-local-hover
```

Terminal 3, PX4 SITL:

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
./scripts/run_px4.sh
```

Terminal 4, ROS 2 Offboard gateway:

```bash
./scripts/run_offboard_control.sh
```

Terminal 5, upper-layer controller:

```bash
./scripts/run_hover_test.sh --ros-args -p z:=-1.0
```

For waypoint cruise, use this as terminal 5 instead:

```bash
./scripts/run_waypoint_cruise.sh
```

Or pass custom waypoints:

```bash
./scripts/run_waypoint_cruise.sh --ros-args \
  -p waypoints:="[0.0, 0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 1.0, -1.0, 0.0, 1.0, -1.0]" \
  -p arrival_radius:=0.25 \
  -p hold_time_s:=2.0
```

Terminal 5 decides where to fly, terminal 4 forwards commands into PX4, terminal 3 runs PX4 control, terminal 2 applies actuator outputs to MuJoCo, and terminal 1 carries PX4/ROS 2 DDS traffic.

### Delta-arm scene

```bash
make run-stack-delta
```

Use this when you want the integrated arm model. For ROS 2 Offboard with the delta-arm scene, use the same five-terminal flow and set `PX4_MUJOCO_MODEL=UAV/scene_uav_delta.xml`.

## ROS 2 Offboard Control

The ROS 2 node is [`offboard_control`](ros2_ws/src/px4_mujoco_ros2_control/px4_mujoco_ros2_control/offboard_control.py).

It:

- forwards fresh external setpoints into PX4 Offboard topics
- stays idle when no upper-layer setpoint is active
- accepts external `TrajectorySetpoint` input on `~/trajectory_setpoint`
- accepts generic ROS 2 position commands on `~/cmd_pose` with `geometry_msgs/msg/PoseStamped`
- accepts generic ROS 2 velocity commands on `~/cmd_twist` with `geometry_msgs/msg/TwistStamped`
- republishes PX4 local state as generic ROS 2 odometry on `~/odom`
- accepts raw `VehicleCommand` input on `~/vehicle_command`

Example PX4-native position command:

```bash
ros2 topic pub /offboard_control/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  "{position: [1.0, 0.0, -2.0], velocity: [.nan, .nan, .nan], acceleration: [.nan, .nan, .nan], yaw: 0.0, yawspeed: .nan}"
```

Example generic ROS 2 position command:

```bash
ros2 topic pub /offboard_control/cmd_pose geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 1.0, y: 0.0, z: -2.0}, orientation: {w: 1.0}}}"
```

If no external command arrives, the gateway does not publish trajectory setpoints and does not auto-arm.

Upper-layer control nodes live in [`uav_control`](ros2_ws/src/uav_control). Current examples are:

- `hover_test`: publishes one fixed `PoseStamped` target
- `waypoint_cruise`: cycles through fixed waypoints using `/offboard_control/odom` for arrival checks

## Project Layout

- [`bridge.py`](bridge.py): PX4 <-> MuJoCo HIL bridge
- [`UAV/`](UAV): MuJoCo scenes and assets
- [`ros2_ws/`](ros2_ws): ROS 2 workspace with `px4_msgs`, `px4_mujoco_ros2_control`, `px4_mujoco_ros2_bringup`, and `uav_control`
- [`scripts/`](scripts): build and launch helpers
- [`docs/`](docs): scenario and reference docs

## PX4 Settings

- default no-arm airframe: `22003_mujoco_delta`
- arm scene airframe: `22002_mujoco_delta`
- PX4 branch: `release/1.15`

## Notes

- The plain UAV path is the cleanest hover target.
- The delta-arm path is the integrated scene.
- If ROS 2 is enabled, PX4 remains the flight controller.

## Troubleshooting

- If manual ROS 2 Offboard does not arm, check `/fmu/out/vehicle_status`, `/fmu/out/vehicle_control_mode`, and `/fmu/out/vehicle_local_position`.
- If `MicroXRCEAgent` is missing, verify `PX4_MUJOCO_UXRCE_DDS_AGENT` in `configs/project.env`.

## Documentation

- [Scenario overview](docs/scenarios.md)
- [Docs index](docs/README.md)
# PX4-Mujoco-ros2
