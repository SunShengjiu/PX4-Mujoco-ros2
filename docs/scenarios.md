# Scenario Layout

The documentation is now organized around three startup paths.

## 1. Local visual hover

- purpose: quickly confirm that MuJoCo opens and the aircraft visibly takes off and hovers
- command: `make run-local`
- guide: [docs/scenarios/local_hover.md](/home/sun/PX4-MuJoCo-ROS2/docs/scenarios/local_hover.md)
- control owner: bridge-local hover controller
- PX4 required: no
- manipulator arm: depends on the selected scene, but the default local path uses the configured default scene

## 2. PX4 stack without the manipulator arm

- purpose: validate the simpler plain-UAV PX4 path first
- command: `make run-stack-uav`
- optional ROS 2 path: start five terminals for Agent, bridge, PX4, Offboard gateway, and an upper-layer node such as `./scripts/run_hover_test.sh` or `./scripts/run_waypoint_cruise.sh`
- guide: [docs/scenarios/uav_px4.md](/home/sun/PX4-MuJoCo-ROS2/docs/scenarios/uav_px4.md)
- control owner: PX4
- PX4 required: yes
- manipulator arm: no

## 3. PX4 stack with the delta arm scene

- purpose: validate the final integrated scene with the arm model present
- command: `make run-stack-delta`
- optional ROS 2 path: use the same separated ROS 2 terminal flow with `PX4_MUJOCO_MODEL=UAV/scene_uav_delta.xml`
- guide: [docs/scenarios/delta_px4.md](/home/sun/PX4-MuJoCo-ROS2/docs/scenarios/delta_px4.md)
- control owner: PX4
- PX4 required: yes
- manipulator arm: yes

## Shared Rules

- ROS 2 Offboard runs should start the bridge with `--no-local-hover`.
- The local visual path is not equivalent to PX4 Offboard hover.
- `px4_mujoco_ros2_control` is the generic PX4 Offboard gateway.
- `uav_control` owns hover tests, waypoint cruise, and future planning nodes.
- The plain UAV and delta-arm scenarios share the same bridge and ROS 2 package code, but use different scenes and PX4 autostart IDs.
