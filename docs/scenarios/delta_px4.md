# PX4 Delta Arm Scenario

## Purpose

Use this path when you want the integrated scene with the manipulator arm present in MuJoCo.

## Commands

Stack only:

```bash
make run-stack-delta
```

PX4 + ROS 2 Offboard path:

```bash
make run-stack-ros2-delta
```

## Scene and Airframe

- MuJoCo scene: `UAV/scene_uav_delta.xml`
- PX4 autostart: `22002`

## Control Ownership

- the bridge is started with `--no-local-hover`
- PX4 owns the flight control loop
- the manipulator actuators remain in the model, but current PX4 flight output is still mapped to the four flight motors only

## When To Use It

- when you want to validate the final integrated visual scene
- when you want the same scene that the repository currently treats as the default PX4-oriented model
