# 场景总览

当前文档按三条主要启动路径组织。

## 1. 本地可视化悬停

- 目的：快速确认 MuJoCo 能打开，并且飞机能起飞和悬停。
- 命令：`make run-local`
- 指南：[docs/scenarios/local_hover.md](/home/sun/PX4-MuJoCo-ROS2/docs/scenarios/local_hover.md)
- 控制方：bridge 本地悬停控制器
- 是否需要 PX4：不需要
- 是否包含机械臂：取决于所选场景，默认本地路径使用配置里的默认场景

## 2. 不带机械臂的 PX4 场景

- 目的：优先验证更简单的普通无人机 PX4 链路。
- 命令：`make run-stack-uav`
- 可选 ROS 2 路径：使用五个终端分别启动 Agent、bridge、PX4、Offboard 网关，以及 `./scripts/run_hover_test.sh` 或 `./scripts/run_waypoint_cruise.sh` 这样的上层节点。
- 指南：[docs/scenarios/uav_px4.md](/home/sun/PX4-MuJoCo-ROS2/docs/scenarios/uav_px4.md)
- 控制方：PX4
- 是否需要 PX4：需要
- 是否包含机械臂：不包含

## 3. 带 Delta 机械臂的 PX4 场景

- 目的：验证带机械臂模型的最终集成场景。
- 命令：`make run-stack-delta`
- 可选 ROS 2 路径：使用同样的分终端 ROS 2 启动流程，并设置 `PX4_MUJOCO_MODEL=UAV/scene_uav_delta.xml`。
- 指南：[docs/scenarios/delta_px4.md](/home/sun/PX4-MuJoCo-ROS2/docs/scenarios/delta_px4.md)
- 控制方：PX4
- 是否需要 PX4：需要
- 是否包含机械臂：包含

## 共同规则

- ROS 2 Offboard 运行时，bridge 应该加 `--no-local-hover`。
- 本地可视化悬停不等同于 PX4 Offboard 悬停。
- `px4_mujoco_ros2_control` 是通用 PX4 Offboard 网关。
- `uav_control` 负责悬停测试、画圈测试和后续规划节点。
- 普通无人机场景和 Delta 机械臂场景共用同一套 bridge 和 ROS 2 包代码，但使用不同场景和 PX4 autostart ID。
