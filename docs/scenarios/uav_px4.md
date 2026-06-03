# PX4 普通无人机场景

## 目的

当你想使用不带机械臂的、更简单的 PX4 控制场景时，使用这条路径。

## 命令

只运行 PX4 栈：

```bash
make run-stack-uav
```

PX4 + ROS 2 Offboard 路径：

使用五个终端。分工是 Agent、MuJoCo bridge、PX4 SITL、Offboard 网关、上层控制节点。

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

如果要测试“画一圈后悬停”，把第五个终端换成：

```bash
./scripts/run_waypoint_cruise.sh
```

自定义画圈参数示例：

```bash
./scripts/run_waypoint_cruise.sh --ros-args \
  -p z:=-1.0 \
  -p radius:=0.8 \
  -p period_s:=24.0 \
  -p pre_circle_hold_s:=5.0
```

Offboard 网关不包含悬停或规划逻辑。上层节点在 `uav_control` 中决定飞机飞到哪里。

## 场景和 Airframe

- MuJoCo 场景：`UAV/scene.xml`
- PX4 autostart：`22003`

## 控制权

- bridge 使用 `--no-local-hover` 启动。
- PX4 负责飞控闭环。
- 如果使用 ROS 2，`offboard_control` 节点把 Offboard setpoint 发布给 PX4，而不是直接控制 MuJoCo。
- `offboard_control` 等待新鲜上层 setpoint；没有 setpoint 时保持空闲。
- 悬停和画圈测试都放在 `uav_control`。
- 上层节点可以发布 `px4_msgs/msg/TrajectorySetpoint` 到 `~/trajectory_setpoint`，也可以使用通用 ROS 2 话题 `~/cmd_pose` 和 `~/cmd_twist`。

## 什么时候使用

- 在加入机械臂场景前，先验证更干净的 PX4 目标。
- 当你想用更少模型变量对比 PX4 行为和 Delta 机械臂场景时。
