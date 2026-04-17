# MuJoCo Model Contract

For PX4 SITL and MuJoCo to run in a stable closed loop, the MuJoCo model must satisfy a small but explicit contract.

## Minimum Requirements

- A root free joint: `<freejoint />`
- Actuators that the bridge can write to
- IMU-related sensors that the bridge can read from

The bridge currently expects these sensor names:

- required: `body_gyro`
- required: `body_linacc`
- optional: `body_mag`

## Recommended Model Shape

- The root body should represent the flying vehicle
- IMU sensors should be attached to the main vehicle body
- Flight actuators should be named clearly and independently
- If the Delta manipulator remains in the same model, the flight control channels and manipulator channels should stay clearly separated

## Current Repository Convention

- Flight channels are named:
  - `motor_1`
  - `motor_2`
  - `motor_3`
  - `motor_4`
- Quad X order is fixed as:
  - front-left
  - rear-right
  - front-right
  - rear-left
- `UAV/UAV_Delta.xml` also keeps three Delta manipulator actuators beyond the four flight motors

## Recommended Next Steps

1. Use the current `motor_1..motor_4` layout to validate PX4 arming, takeoff, and attitude control
2. Tune rotor positions, thrust scaling, and reaction torque
3. Add ROS 2 control for the Delta manipulator only after the PX4 flight loop is stable
