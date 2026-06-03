# 架构说明

当前仓库提供三条运行路径：

1. 本地 MuJoCo 悬停演示
2. 不带机械臂的 PX4 + MuJoCo 场景
3. 带 Delta 机械臂的 PX4 + MuJoCo 场景

## 数据流

### 本地可视化悬停

```text
MuJoCo <- Python bridge local-hover controller
```

通过 `make run-local` 启动，不使用 PX4。

### PX4 场景

```text
QGroundControl <- UDP 14550 <- PX4 SITL <- TCP 4560 -> Python bridge <-> MuJoCo
```

这是 `make run-stack-*` 使用的控制链路。

### PX4 + ROS 2 Offboard 场景

```text
QGroundControl <- UDP 14550 <- PX4 SITL <- TCP 4560 -> Python bridge <-> MuJoCo
uav_control -> ROS 2 cmd_pose/cmd_twist -> offboard_control -> px4_msgs -> Micro XRCE-DDS Agent <- UDP 8888 <- PX4 SITL
```

这是分终端 ROS 2 启动流程使用的消息链路：Agent、bridge、PX4、`offboard_control`，最后是上层 `uav_control` 节点。

## 职责划分

- `bridge.py`
  - 驱动 MuJoCo。
  - 把 MuJoCo 状态转换为 PX4 需要的 MAVLink HIL 消息。
  - 把 PX4 执行器输出作用回 MuJoCo。
  - 在非 PX4 演示路径中，也可以运行 bridge 本地悬停控制器。

- `UAV/`
  - 存放 MuJoCo 场景和模型资源。
  - 当前包含普通无人机场景和 Delta 机械臂场景。

- `scripts/`
  - 负责环境准备、patch、构建、验证和启动编排。
  - 提供本地、PX4、Agent、bridge 和 ROS 2 节点的独立启动入口。

- `ros2_ws/`
  - 存放 `px4_msgs`、Offboard 网关包、bringup 包和 `uav_control`。
  - PX4 网关代码放在 `px4_mujoco_ros2_control`。
  - launch 文件放在 `px4_mujoco_ros2_bringup`。
  - 悬停、画圈、巡航和后续规划节点放在 `uav_control`。

- `integrations/px4/`
  - 存放本仓库需要的最小 PX4 patch。

## 控制权规则

1. `make run-local` 使用 bridge 本地悬停控制器。
2. PX4 和 ROS 2 Offboard 运行时，bridge 使用 `--no-local-hover`。
3. 在 PX4 路径中，PX4 始终是唯一飞控。
4. `offboard_control` 只转发活跃的外部 setpoint，不负责具体任务行为。
5. `uav_control` 负责具体任务行为，并发布通用 ROS 2 命令。

## 为什么这样组织

目标是在一个代码库中清晰表达三条用户可见路径：

- 本地模式：快速做可视化确认。
- 普通无人机场景：更简单的 PX4 栈验证目标。
- Delta 机械臂场景：最终集成场景。
