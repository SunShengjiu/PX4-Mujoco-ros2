# PX4 Plain UAV Scenario

## Purpose

Use this path when you want the simpler PX4-controlled scene without the manipulator arm.

## Commands

Stack only:

```bash
make run-stack-uav
```

PX4 + ROS 2 Offboard path:

```bash
make run-stack-ros2-uav
```

## Scene and Airframe

- MuJoCo scene: `UAV/scene.xml`
- PX4 autostart: `22003`

## Control Ownership

- the bridge is started with `--no-local-hover`
- PX4 owns the flight control loop
- if ROS 2 is used, the `offboard_control` node publishes Offboard setpoints into PX4 rather than controlling MuJoCo directly
- the default ROS 2 behavior is still hover hold, but you can override it by publishing `px4_msgs/msg/TrajectorySetpoint` to `~/trajectory_setpoint`

## When To Use It

- when you want a cleaner PX4 validation target before adding the arm scene
- when you want to compare PX4 behavior against the delta-arm scene with fewer model variables
