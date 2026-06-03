# PX4 + MuJoCo + ROS 2

这个仓库把 PX4 SITL、MuJoCo 和 ROS 2 组织成一套可运行、可扩展的无人机仿真工程。

它包含：

- Python HIL 桥接程序，用来连接 PX4 和 MuJoCo。
- 无机械臂的普通无人机场景，以及带 Delta 机械臂的场景。
- ROS 2 Offboard 网关，以及独立的上层控制节点。
- 用于依赖安装、构建、启动和验证的脚本。

推荐的 ROS 2 Offboard 路径采用多个终端分别启动。这样 DDS、MuJoCo bridge、PX4、Offboard 网关和上层控制节点都能单独观察和调试。

## 当前可用功能

- `make run-local`：快速运行 MuJoCo 本地悬停演示，不启动 PX4。
- `make run-stack-uav`：运行无机械臂无人机场景，由 PX4 负责飞控。
- 手动 ROS 2 启动：按 Agent、bridge、PX4、Offboard 网关、上层控制节点分开启动。
- `make run-stack-delta`：运行带 Delta 机械臂的场景；ROS 2 Offboard 也可以按同样的五终端方式手动启动。

## 快速开始

1. 准备 PX4 源码目录和 QGroundControl。
2. 把 `configs/project.env.example` 复制为 `configs/project.env`。
3. 在 `configs/project.env` 里把 `PX4_MUJOCO_PX4_DIR` 指向你的 PX4 源码目录。
4. 安装依赖：

```bash
make deps-ubuntu
```

5. 应用 PX4 patch：

```bash
make px4-patch
```

6. 构建 ROS 2：

```bash
make ros2-build
```

7. 构建 PX4：

```bash
make px4-build
```

8. 运行检查：

```bash
make doctor
```

建议第一次先跑：

```bash
make run-local
make run-stack-uav
```

## 运行模式

### 本地悬停

```bash
make run-local
```

只启动 MuJoCo，不启动 PX4。适合快速确认模型、旋翼和可视化是否正常。

### 无机械臂 PX4 场景

```bash
make run-stack-uav
```

PX4 负责飞行控制，不包含机械臂。

### 无机械臂 ROS 2 Offboard 场景

ROS 2 Offboard 建议使用五个终端分别启动，便于调试每一层。

五个终端的分工如下：

- 终端 1：PX4 和 ROS 2 之间的 DDS 通信。
- 终端 2：MuJoCo 物理仿真 bridge。
- 终端 3：PX4 SITL 飞控。
- 终端 4：通用 ROS 2 Offboard 网关。
- 终端 5：上层控制节点，例如悬停测试或画圈测试。

终端 1，Micro XRCE-DDS Agent：

```bash
make run-ros2-agent
```

终端 2，MuJoCo bridge：

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
PX4_MUJOCO_BRIDGE_EXTRA_ARGS="--px4-alignment-hold-seconds 2.0 --debug-hil-rate" \
./scripts/run_bridge.sh --no-local-hover
```

终端 3，PX4 SITL：

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
./scripts/run_px4.sh
```

终端 4，ROS 2 Offboard 网关：

```bash
./scripts/run_offboard_control.sh
```

终端 5，上层控制节点。悬停测试：

```bash
./scripts/run_hover_test.sh --ros-args -p z:=-1.0
```

如果要做“画一圈后回到起点悬停”的测试，用：

```bash
./scripts/run_waypoint_cruise.sh
```

也可以传入自定义参数：

```bash
./scripts/run_waypoint_cruise.sh --ros-args \
  -p z:=-1.0 \
  -p radius:=0.8 \
  -p period_s:=24.0
```

终端 5 决定飞机飞到哪里，终端 4 把上层控制命令转发给 PX4，终端 3 运行 PX4 控制器，终端 2 把 PX4 输出作用到 MuJoCo，终端 1 负责 PX4/ROS 2 DDS 通信。

### Delta 机械臂场景

```bash
make run-stack-delta
```

当你需要带机械臂的集成场景时使用这个模式。如果要在机械臂场景中使用 ROS 2 Offboard，也按同样的五终端流程启动，并设置：

```bash
PX4_MUJOCO_MODEL=UAV/scene_uav_delta.xml
```

## ROS 2 Offboard 控制

ROS 2 Offboard 网关节点是：

[offboard_control.py](ros2_ws/src/px4_mujoco_ros2_control/px4_mujoco_ros2_control/offboard_control.py)

它负责：

- 把外部新鲜 setpoint 转发到 PX4 Offboard 话题。
- 没有上层 setpoint 时保持空闲，不自己悬停，也不自动发布轨迹点。
- 接收 `~/trajectory_setpoint` 上的 PX4 原生 `TrajectorySetpoint`。
- 接收 `~/cmd_pose` 上的通用 ROS 2 位置命令，类型是 `geometry_msgs/msg/PoseStamped`。
- 接收 `~/cmd_twist` 上的通用 ROS 2 速度命令，类型是 `geometry_msgs/msg/TwistStamped`。
- 把 PX4 本地位置重新发布成通用 ROS 2 里程计 `~/odom`。
- 接收 `~/vehicle_command` 上的原始 `VehicleCommand`。

PX4 原生位置 setpoint 示例：

```bash
ros2 topic pub /offboard_control/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  "{position: [1.0, 0.0, -2.0], velocity: [.nan, .nan, .nan], acceleration: [.nan, .nan, .nan], yaw: 0.0, yawspeed: .nan}"
```

通用 ROS 2 位置命令示例：

```bash
ros2 topic pub /offboard_control/cmd_pose geometry_msgs/msg/PoseStamped \
  "{pose: {position: {x: 1.0, y: 0.0, z: -2.0}, orientation: {w: 1.0}}}"
```

如果没有外部命令到达，Offboard 网关不会发布 PX4 trajectory setpoint，也不会自动解锁。

上层控制节点放在：

[uav_control](ros2_ws/src/uav_control)

当前已有示例：

- `hover_test`：持续发布一个固定的 `PoseStamped` 目标点。
- `waypoint_cruise`：锁定当前位置，飞一圈，然后回到起点并保持悬停。

## 项目结构

- [bridge.py](bridge.py)：PX4 和 MuJoCo 之间的 HIL bridge。
- [UAV/](UAV)：MuJoCo 场景和模型资源。
- [ros2_ws/](ros2_ws)：ROS 2 工作空间，包含 `px4_msgs`、`px4_mujoco_ros2_control`、`px4_mujoco_ros2_bringup` 和 `uav_control`。
- [scripts/](scripts)：构建和启动脚本。
- [docs/](docs)：场景文档和参考文档。

## PX4 设置

- 无机械臂默认 airframe：`22003_mujoco_delta`
- 带机械臂场景 airframe：`22002_mujoco_delta`
- PX4 分支：`release/1.15`

## 注意事项

- 无机械臂场景是最干净的 PX4 悬停验证目标。
- Delta 机械臂场景是最终集成场景。
- 启用 ROS 2 时，PX4 仍然是唯一飞控，ROS 2 只给 PX4 Offboard setpoint。

## 故障排查

- 如果手动 ROS 2 Offboard 不能解锁，检查 `/fmu/out/vehicle_status`、`/fmu/out/vehicle_control_mode` 和 `/fmu/out/vehicle_local_position`。
- 如果找不到 `MicroXRCEAgent`，检查 `configs/project.env` 里的 `PX4_MUJOCO_UXRCE_DDS_AGENT`。

## 文档入口

- [场景总览](docs/scenarios.md)
- [文档索引](docs/README.md)
