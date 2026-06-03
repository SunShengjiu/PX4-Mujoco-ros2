# PX4 Delta 机械臂场景

## 目的

当你想使用 MuJoCo 中包含机械臂的集成场景时，使用这条路径。

## 命令

只运行 PX4 栈：

```bash
make run-stack-delta
```

PX4 + ROS 2 Offboard 路径：

使用五个终端。第五个终端可以启动 `hover_test`、`waypoint_cruise`，或 `uav_control` 中的其他上层节点。

```bash
make run-ros2-agent
```

```bash
PX4_MUJOCO_MODEL=UAV/scene_uav_delta.xml \
PX4_MUJOCO_PX4_AUTOSTART=22002 \
./scripts/run_bridge.sh --no-local-hover
```

```bash
PX4_MUJOCO_MODEL=UAV/scene_uav_delta.xml \
PX4_MUJOCO_PX4_AUTOSTART=22002 \
./scripts/run_px4.sh
```

```bash
./scripts/run_offboard_control.sh
```

```bash
./scripts/run_hover_test.sh --ros-args -p z:=-1.0
```

如果要测试画圈节点，把第五个终端换成：

```bash
./scripts/run_waypoint_cruise.sh
```

## 场景和 Airframe

- MuJoCo 场景：`UAV/scene_uav_delta.xml`
- PX4 autostart：`22002`

## 控制权

- bridge 使用 `--no-local-hover` 启动。
- PX4 负责飞控闭环。
- `px4_mujoco_ros2_control` 只作为 Offboard 网关。
- 悬停、画圈和后续任务逻辑都放在 `uav_control`。
- 机械臂动作器保留在模型中，但当前 PX4 飞行输出仍然只映射到四个飞行电机。

## 什么时候使用

- 当你想验证最终集成可视化场景时。
- 当你想使用仓库当前默认的 PX4-oriented 集成模型时。
