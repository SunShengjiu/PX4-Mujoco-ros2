# px4_mujoco_ros2_control

Generic ROS 2 gateway between upper-layer UAV control nodes and PX4 Offboard topics.

This package should stay close to PX4 communication. Do not put hover, path planning, waypoint logic, or replanning policy here. Those belong in packages such as `uav_control`.

## Inputs

- `trajectory_setpoint_topic`, default `~/trajectory_setpoint`: `px4_msgs/msg/TrajectorySetpoint`
- `cmd_pose_topic`, default `~/cmd_pose`: `geometry_msgs/msg/PoseStamped`
- `cmd_twist_topic`, default `~/cmd_twist`: `geometry_msgs/msg/TwistStamped`
- `vehicle_command_topic`, default `~/vehicle_command`: optional forwarded `px4_msgs/msg/VehicleCommand`

## PX4 Topics

- `px4_input_prefix`, default `/fmu/in`
- `px4_output_prefix`, default `/fmu/out`

For a namespaced or multi-vehicle setup, launch the same gateway with different prefixes and node namespaces.

## Safety And Mode Parameters

- `auto_request_offboard`, default `true`: request PX4 Offboard mode after warmup setpoints.
- `auto_arm`, default `true`: arm PX4 after warmup setpoints.
- `auto_request_offboard_and_arm`, default `true`: legacy combined switch. If this is `false`, both automatic actions are disabled.
- `require_local_position_for_offboard`, default `true`: wait for valid PX4 local position before requesting Offboard/Arm.
- `command_timeout_s`, default `0.5`: stop forwarding stale upper-layer setpoints.
- `warmup_setpoints`, default `20`: number of forwarded setpoints before requesting Offboard/Arm.

## Outputs

- `odom_topic`, default `~/odom`: generic `nav_msgs/msg/Odometry` view of PX4 local position.

If no external setpoint is active, the gateway does not publish PX4 trajectory setpoints and does not auto-arm.

## Five-Terminal ROS 2 Flow

Run ROS 2 Offboard in separate terminals so each layer is easy to debug.

Terminal 1 starts the Micro XRCE-DDS Agent:

```bash
make run-ros2-agent
```

This exposes PX4 uORB topics to ROS 2 through DDS.

Terminal 2 starts the MuJoCo bridge:

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
PX4_MUJOCO_BRIDGE_EXTRA_ARGS="--px4-alignment-hold-seconds 2.0 --debug-hil-rate" \
./scripts/run_bridge.sh --no-local-hover
```

This sends MuJoCo sensor data to PX4 and writes PX4 actuator outputs back into MuJoCo.

Terminal 3 starts PX4 SITL:

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
./scripts/run_px4.sh
```

PX4 owns arming, Offboard mode, attitude control, position control, and actuator allocation.

Terminal 4 starts this Offboard gateway:

```bash
./scripts/run_offboard_control.sh
```

This converts upper-layer ROS 2 commands into PX4 Offboard inputs.

Terminal 5 starts an upper-layer control node from `uav_control`, for example:

```bash
./scripts/run_hover_test.sh --ros-args -p z:=-1.0
```

or:

```bash
./scripts/run_waypoint_cruise.sh
```

The split is:

- Terminal 1 carries PX4/ROS 2 DDS traffic.
- Terminal 2 applies PX4 outputs to MuJoCo physics.
- Terminal 3 runs the PX4 flight controller.
- Terminal 4 converts generic ROS 2 commands into PX4 Offboard inputs.
- Terminal 5 decides where to fly.

Keep new hover, cruise, trajectory, or replanning behavior in `uav_control`; keep this package as the reusable PX4-facing gateway.
