# 文档索引

这个仓库的文档分为两类：

- 按运行场景组织的启动指南
- 架构、模型约束和当前状态等参考说明

## 从这里开始

根据你要运行的内容选择文档：

1. 本地可视化悬停演示
   - 命令：`make run-local`
   - 指南：[docs/scenarios/local_hover.md](/home/sun/PX4-MuJoCo-ROS2/docs/scenarios/local_hover.md)

2. 不带机械臂的 PX4 场景
   - 命令：`make run-stack-uav`
   - ROS 2 Offboard：分别启动 Agent、bridge、PX4、Offboard 网关和 `uav_control` 节点
   - 指南：[docs/scenarios/uav_px4.md](/home/sun/PX4-MuJoCo-ROS2/docs/scenarios/uav_px4.md)

3. 带 Delta 机械臂的 PX4 场景
   - 命令：`make run-stack-delta`
   - 指南：[docs/scenarios/delta_px4.md](/home/sun/PX4-MuJoCo-ROS2/docs/scenarios/delta_px4.md)

如果想快速比较所有路径，请看：

[docs/scenarios.md](/home/sun/PX4-MuJoCo-ROS2/docs/scenarios.md)

## 参考文档

- [架构说明](/home/sun/PX4-MuJoCo-ROS2/docs/reference/architecture.md)
- [模型约束](/home/sun/PX4-MuJoCo-ROS2/docs/reference/model_contract.md)
- [当前状态](/home/sun/PX4-MuJoCo-ROS2/docs/reference/current_status.md)
