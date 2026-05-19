# PX4 + MuJoCo + ROS 2

This repository packages a PX4 SITL + MuJoCo simulation stack with:

- a Python HIL bridge between PX4 and MuJoCo
- a plain UAV scene and a delta-arm scene
- a ROS 2 Offboard controller for PX4
- reproducible scripts for build, launch, and validation

The recommended hover path for the no-arm aircraft is `make run-stack-ros2-uav`.

## What Works

- `make run-local` gives a quick MuJoCo-only hover demo
- `make run-stack-uav` runs the plain UAV with PX4 in control
- `make run-stack-ros2-uav` adds ROS 2 Offboard hover control
- `make run-stack-delta` and `make run-stack-ros2-delta` run the arm scene

## Quick Start

1. Set up your PX4 tree and QGroundControl.
2. Copy `configs/project.env.example` to `configs/project.env`.
3. Point `PX4_MUJOCO_PX4_DIR` at your PX4 checkout.
4. Install deps with `make deps-ubuntu`.
5. Apply the PX4 patch with `make px4-patch`.
6. Build ROS 2 with `make ros2-build`.
7. Build PX4 with `make px4-build`.
8. Validate with `make doctor`.

Typical first run:

```bash
make run-local
make run-stack-uav
make run-stack-ros2-uav
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

```bash
make run-stack-ros2-uav
```

This is the best starting point for stable hover on the no-arm aircraft.

### Delta-arm scene

```bash
make run-stack-delta
make run-stack-ros2-delta
```

Use these when you want the integrated arm model.

## ROS 2 Offboard Control

The ROS 2 node is [`offboard_control`](ros2_ws/src/px4_mujoco_ros2_control/px4_mujoco_ros2_control/offboard_control.py).

It:

- publishes PX4 Offboard setpoints
- defaults to hover fallback at NED `x=0, y=0, z=-2, yaw=0`
- accepts external `TrajectorySetpoint` input on `~/trajectory_setpoint`
- accepts raw `VehicleCommand` input on `~/vehicle_command`

Example position command:

```bash
ros2 topic pub /offboard_control/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  "{position: [1.0, 0.0, -2.0], velocity: [.nan, .nan, .nan], acceleration: [.nan, .nan, .nan], yaw: 0.0, yawspeed: .nan}"
```

If no external command arrives, the node stays in hover fallback mode.

## Project Layout

- [`bridge.py`](bridge.py): PX4 <-> MuJoCo HIL bridge
- [`UAV/`](UAV): MuJoCo scenes and assets
- [`ros2_ws/`](ros2_ws): ROS 2 workspace and Offboard package
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

- If `make run-stack-ros2-uav` does not arm, check `/fmu/out/vehicle_local_position` and `/fmu/out/vehicle_odometry`.
- If you explicitly enable estimator gating, also check `/fmu/out/estimator_status_flags`.
- If `MicroXRCEAgent` is missing, verify `PX4_MUJOCO_UXRCE_DDS_AGENT` in `configs/project.env`.

## Documentation

- [Scenario overview](docs/scenarios.md)
- [Docs index](docs/README.md)
