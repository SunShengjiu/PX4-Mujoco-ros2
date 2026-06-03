# uav_control

这是 PX4 MuJoCo ROS 2 工程里的上层 UAV 控制、规划和测试节点包。

这个包负责决定飞机应该飞到哪里。它不应该直接发布 PX4 `/fmu/in/*` 话题，而是应该把通用 ROS 2 命令发布给 `px4_mujoco_ros2_control` 里的 Offboard 网关。

## 接口

- 控制输出：`/offboard_control/cmd_pose`，类型 `geometry_msgs/msg/PoseStamped`
- 可选状态输入：`/offboard_control/odom`，类型 `nav_msgs/msg/Odometry`
- 默认坐标系：`map_ned`
- 高度约定：PX4 使用 NED 坐标，向上是负数，例如 `z=-1.0`

## 节点

### hover_test

持续发布一个固定位置。它只是链路测试节点，也可以作为最简单的悬停目标。

```bash
./scripts/run_hover_test.sh --ros-args -p z:=-1.0
```

参数：

- `x`，默认 `0.0`
- `y`，默认 `0.0`
- `z`，默认 `-1.0`
- `yaw`，默认 `0.0`
- `rate_hz`，默认 `20.0`

### waypoint_cruise

从 `/offboard_control/odom` 锁定当前 XY 位置，先到目标 NED 高度 `z` 并保持一小段时间，然后发布一圈平滑圆轨迹，最后回到起点并持续发布起点位置，让飞机稳定悬停。

```bash
./scripts/run_waypoint_cruise.sh --ros-args \
  -p z:=-1.0 \
  -p radius:=0.8 \
  -p period_s:=24.0 \
  -p pre_circle_hold_s:=5.0
```

参数：

- `center_x`，默认 `0.0`，仅在 `use_current_position=false` 时使用
- `center_y`，默认 `0.0`，仅在 `use_current_position=false` 时使用
- `z`，默认 `-1.0`
- `radius`，默认 `0.8`
- `period_s`，默认 `24.0`
- `pre_circle_hold_s`，默认 `5.0`
- `hold_time_s`，默认 `999999.0`
- `yaw`，默认 `0.0`
- `rate_hz`，默认 `20.0`
- `cmd_pose_topic`，默认 `/offboard_control/cmd_pose`
- `odom_topic`，默认 `/offboard_control/odom`
- `frame_id`，默认 `map_ned`
- `clockwise`，默认 `false`
- `use_current_position`，默认 `true`

## 新增控制节点

新的任务逻辑应该放在这里，例如画圈、航点文件、轨迹平滑、重规划等。

完整步骤请阅读：

```text
ros2_ws/src/uav_control/ADDING_FEATURES.md
```

常规模式是：

1. 如果节点需要飞机状态，订阅 `/offboard_control/odom`。
2. 发布 `PoseStamped` 到 `/offboard_control/cmd_pose`，或者发布 `TwistStamped` 到 `/offboard_control/cmd_twist`。
3. 在 `setup.py` 里添加 console entry。
4. 不要在这个包里写 PX4 专用话题名和解锁逻辑，除非该节点就是为了测试 PX4 命令通道。
