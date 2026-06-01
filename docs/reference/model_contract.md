# MuJoCo Model Contract

For PX4 SITL and MuJoCo to run in a stable closed loop, the MuJoCo model must satisfy a small but explicit contract.

This contract applies to both repository scenes:

- `UAV/scene.xml`
- `UAV/scene_uav_delta.xml`

## Minimum Requirements

- a root free joint: `<freejoint />`
- actuators that the bridge can write to
- IMU-related sensors that the bridge can read from

The bridge currently expects these sensor names:

- required: `body_gyro`
- required: `body_linacc`
- optional: `body_mag`

## Recommended Model Shape

- the root body should represent the flying vehicle
- IMU sensors should be attached to the main vehicle body
- flight actuators should be named clearly and independently
- if the delta manipulator remains in the same model, the flight channels and manipulator channels should stay clearly separated

## Current Repository Convention

- flight channels are named:
  - `motor_1`
  - `motor_2`
  - `motor_3`
  - `motor_4`
- Quad X order is fixed as:
  - front-left
  - rear-right
  - front-right
  - rear-left
- `UAV/UAV_Delta.xml` keeps the manipulator actuators in the same file, but PX4 flight output is still mapped only to the four flight motors

## Scope Notes

- `make run-local` uses the same scene/model assets, but does not depend on PX4.
- PX4 stack and ROS 2 Offboard startup flows depend on this contract for PX4-facing HIL behavior.
