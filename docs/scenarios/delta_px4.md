# PX4 Delta Arm Scenario

## Purpose

Use this path when you want the integrated scene with the manipulator arm present in MuJoCo.

## Commands

Stack only:

```bash
make run-stack-delta
```

PX4 + ROS 2 Offboard path:

Use five terminals. The fifth terminal can be `hover_test`, `waypoint_cruise`, or another upper-layer node from `uav_control`.

```bash
make run-ros2-agent
```

```bash
PX4_MUJOCO_MODEL=UAV/scene_uav_delta.xml \
PX4_MUJOCO_PX4_AUTOSTART=22002 \
./scripts/run_bridge.sh --no-local-hover
```

```bash
PX4_MUJOCO_MODEL=UAV/scene_uav_delta.xml \
PX4_MUJOCO_PX4_AUTOSTART=22002 \
./scripts/run_px4.sh
```

```bash
./scripts/run_offboard_control.sh
```

```bash
./scripts/run_hover_test.sh --ros-args -p z:=-1.0
```

For waypoint cruise, use this as terminal 5 instead:

```bash
./scripts/run_waypoint_cruise.sh
```

## Scene and Airframe

- MuJoCo scene: `UAV/scene_uav_delta.xml`
- PX4 autostart: `22002`

## Control Ownership

- the bridge is started with `--no-local-hover`
- PX4 owns the flight control loop
- `px4_mujoco_ros2_control` remains only the Offboard gateway
- hover, waypoint cruise, and later mission logic live in `uav_control`
- the manipulator actuators remain in the model, but current PX4 flight output is still mapped to the four flight motors only

## When To Use It

- when you want to validate the final integrated visual scene
- when you want the same scene that the repository currently treats as the default PX4-oriented model
