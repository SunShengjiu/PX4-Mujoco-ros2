# px4_mujoco_ros2_control

这是上层 UAV 控制节点和 PX4 Offboard 话题之间的通用 ROS 2 网关。

这个包应该尽量只负责 PX4 通信。不要把悬停、路径规划、航点巡航、重规划策略写在这里。这些任务逻辑应该放到 `uav_control` 这样的上层控制包里。

## 输入接口

- `trajectory_setpoint_topic`，默认 `~/trajectory_setpoint`：`px4_msgs/msg/TrajectorySetpoint`
- `cmd_pose_topic`，默认 `~/cmd_pose`：`geometry_msgs/msg/PoseStamped`
- `cmd_twist_topic`，默认 `~/cmd_twist`：`geometry_msgs/msg/TwistStamped`
- `vehicle_command_topic`，默认 `~/vehicle_command`：可选转发的 `px4_msgs/msg/VehicleCommand`

## PX4 话题前缀

- `px4_input_prefix`，默认 `/fmu/in`
- `px4_output_prefix`，默认 `/fmu/out`

如果后续要做命名空间或多机仿真，可以用不同的话题前缀和节点命名空间启动多个网关。

## 安全和模式参数

- `auto_request_offboard`，默认 `true`：收到足够的 warmup setpoint 后请求 PX4 进入 Offboard 模式。
- `auto_arm`，默认 `true`：收到足够的 warmup setpoint 后请求 PX4 解锁。
- `auto_request_offboard_and_arm`，默认 `true`：兼容旧用法的总开关。如果它是 `false`，自动 Offboard 和自动解锁都会关闭。
- `require_local_position_for_offboard`，默认 `true`：请求 Offboard/Arm 前等待 PX4 本地位置有效。
- `command_timeout_s`，默认 `0.5`：上层 setpoint 超时后停止转发。
- `warmup_setpoints`，默认 `20`：请求 Offboard/Arm 前先连续转发的 setpoint 数量。

## 输出接口

- `odom_topic`，默认 `~/odom`：把 PX4 本地位置转成通用 `nav_msgs/msg/Odometry`。

如果没有活跃的外部 setpoint，网关不会发布 PX4 trajectory setpoint，也不会自动解锁。

## 五终端 ROS 2 启动流程

ROS 2 Offboard 建议分多个终端启动，这样每一层都容易调试。

终端 1 启动 Micro XRCE-DDS Agent：

```bash
make run-ros2-agent
```

它负责把 PX4 uORB 话题通过 DDS 暴露给 ROS 2。

终端 2 启动 MuJoCo bridge：

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
PX4_MUJOCO_BRIDGE_EXTRA_ARGS="--px4-alignment-hold-seconds 2.0 --debug-hil-rate" \
./scripts/run_bridge.sh --no-local-hover
```

它负责把 MuJoCo 传感器数据发给 PX4，并把 PX4 电机输出写回 MuJoCo。

终端 3 启动 PX4 SITL：

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
./scripts/run_px4.sh
```

PX4 负责解锁、Offboard 模式、姿态控制、位置控制和电机分配。

终端 4 启动本包的 Offboard 网关：

```bash
./scripts/run_offboard_control.sh
```

它负责把上层 ROS 2 控制命令转换为 PX4 Offboard 输入。

终端 5 启动 `uav_control` 里的上层控制节点，例如：

```bash
./scripts/run_hover_test.sh --ros-args -p z:=-1.0
```

或者：

```bash
./scripts/run_waypoint_cruise.sh
```

整体分工是：

- 终端 1：承载 PX4 和 ROS 2 的 DDS 通信。
- 终端 2：把 PX4 输出作用到 MuJoCo 物理仿真。
- 终端 3：运行 PX4 飞控。
- 终端 4：把通用 ROS 2 命令转换成 PX4 Offboard 输入。
- 终端 5：决定飞机应该飞到哪里。

新增悬停、巡航、轨迹或重规划功能时，请放到 `uav_control`。本包应该保持为可复用的 PX4 网关。
