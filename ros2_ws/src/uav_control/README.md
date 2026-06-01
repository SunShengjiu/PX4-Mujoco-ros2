# uav_control

Upper-layer UAV control, planning, and test nodes for the PX4 MuJoCo ROS 2 stack.

This package should decide where the aircraft should fly. It should not publish directly to PX4 `/fmu/in/*` topics. Instead, it publishes generic commands to the Offboard gateway in `px4_mujoco_ros2_control`.

## Interfaces

- command output: `/offboard_control/cmd_pose`, `geometry_msgs/msg/PoseStamped`
- optional state input: `/offboard_control/odom`, `nav_msgs/msg/Odometry`
- default frame: `map_ned`
- height convention: PX4 NED, so flying upward uses negative `z`, for example `z=-1.0`

## Nodes

### hover_test

Publishes one fixed pose continuously. This is only a link test and a simple hover target.

```bash
./scripts/run_hover_test.sh --ros-args -p z:=-1.0
```

Parameters:

- `x`, default `0.0`
- `y`, default `0.0`
- `z`, default `-1.0`
- `yaw`, default `0.0`
- `rate_hz`, default `20.0`

### waypoint_cruise

Cycles through fixed 3D waypoints. It publishes the current target to `/offboard_control/cmd_pose` and watches `/offboard_control/odom` to decide when the target has been reached.

```bash
./scripts/run_waypoint_cruise.sh --ros-args \
  -p waypoints:="[0.0, 0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 1.0, -1.0, 0.0, 1.0, -1.0]" \
  -p arrival_radius:=0.25 \
  -p hold_time_s:=2.0
```

Parameters:

- `waypoints`, flat list of `x, y, z` triples
- `yaw`, default `0.0`
- `rate_hz`, default `20.0`
- `arrival_radius`, default `0.25`
- `hold_time_s`, default `2.0`
- `cmd_pose_topic`, default `/offboard_control/cmd_pose`
- `odom_topic`, default `/offboard_control/odom`
- `frame_id`, default `map_ned`
- `loop`, default `true`

## Adding New Control Nodes

Put new mission logic here, for example circle flight, waypoint files, trajectory smoothing, or replanning.

The normal pattern is:

1. Subscribe to `/offboard_control/odom` if the node needs vehicle state.
2. Publish `PoseStamped` to `/offboard_control/cmd_pose` or `TwistStamped` to `/offboard_control/cmd_twist`.
3. Add a console entry in `setup.py`.
4. Keep PX4-specific topic names and arming logic out of this package unless the node is explicitly testing a PX4 command path.
