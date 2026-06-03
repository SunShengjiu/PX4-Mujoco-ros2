# MuJoCo 模型约束

为了让 PX4 SITL 和 MuJoCo 稳定闭环运行，MuJoCo 模型需要满足一组明确的最小约束。

这份约束适用于仓库中的两个场景：

- `UAV/scene.xml`
- `UAV/scene_uav_delta.xml`

## 最小要求

- 根 body 需要有自由关节：`<freejoint />`
- 模型需要有 bridge 可以写入的 actuators
- 模型需要有 bridge 可以读取的 IMU 相关 sensors

bridge 当前期望这些传感器名称：

- 必需：`body_gyro`
- 必需：`body_linacc`
- 可选：`body_mag`

## 推荐模型结构

- 根 body 应该代表飞行器本体。
- IMU 传感器应该挂在主飞行器 body 上。
- 飞行动作器应该命名清楚，并和其他动作器分开。
- 如果 Delta 机械臂保留在同一个模型里，飞行通道和机械臂通道应该保持清晰分离。

## 当前仓库约定

- 飞行通道命名为：
  - `motor_1`
  - `motor_2`
  - `motor_3`
  - `motor_4`
- Quad X 顺序固定为：
  - 左前
  - 右后
  - 右前
  - 左后
- `UAV/UAV_Delta.xml` 把机械臂动作器保留在同一个文件里，但 PX4 飞行输出仍然只映射到四个飞行电机。

## 范围说明

- `make run-local` 使用同一套场景和模型资源，但不依赖 PX4。
- PX4 栈和 ROS 2 Offboard 启动流程依赖这份约束来保证 PX4-facing HIL 行为正确。
