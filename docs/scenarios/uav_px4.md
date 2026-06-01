# PX4 Plain UAV Scenario

## Purpose

Use this path when you want the simpler PX4-controlled scene without the manipulator arm.

## Commands

Stack only:

```bash
make run-stack-uav
```

PX4 + ROS 2 Offboard path:

Use five terminals. The split is Agent, MuJoCo bridge, PX4 SITL, Offboard gateway, then upper-layer control.

```bash
make run-ros2-agent
```

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
PX4_MUJOCO_BRIDGE_EXTRA_ARGS="--px4-alignment-hold-seconds 2.0 --debug-hil-rate" \
./scripts/run_bridge.sh --no-local-hover
```

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
./scripts/run_px4.sh
```

```bash
./scripts/run_offboard_control.sh
```

```bash
./scripts/run_hover_test.sh --ros-args -p z:=-1.0
```

For waypoint cruise, use this as the fifth terminal instead:

```bash
./scripts/run_waypoint_cruise.sh
```

Custom waypoint example:

```bash
./scripts/run_waypoint_cruise.sh --ros-args \
  -p waypoints:="[0.0, 0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 1.0, -1.0, 0.0, 1.0, -1.0]" \
  -p arrival_radius:=0.25 \
  -p hold_time_s:=2.0
```

The Offboard gateway does not contain hover or planning logic. Upper-layer nodes in `uav_control` decide where to fly.

## Scene and Airframe

- MuJoCo scene: `UAV/scene.xml`
- PX4 autostart: `22003`

## Control Ownership

- the bridge is started with `--no-local-hover`
- PX4 owns the flight control loop
- if ROS 2 is used, the `offboard_control` node publishes Offboard setpoints into PX4 rather than controlling MuJoCo directly
- `offboard_control` waits for fresh upper-layer setpoints and stays idle without them
- hover and waypoint cruise live in `uav_control`
- upper-layer nodes can publish `px4_msgs/msg/TrajectorySetpoint` to `~/trajectory_setpoint`, or use the generic ROS 2 topics `~/cmd_pose` and `~/cmd_twist`

## When To Use It

- when you want a cleaner PX4 validation target before adding the arm scene
- when you want to compare PX4 behavior against the delta-arm scene with fewer model variables
