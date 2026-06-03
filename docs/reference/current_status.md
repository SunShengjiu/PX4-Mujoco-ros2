# 当前状态

## 已实现内容

- `bridge.py` 默认使用仓库内模型 `UAV/scene_uav_delta.xml`。
- `bridge.py` 包含 bridge 本地起飞/悬停模式，供 `make run-local` 使用。
- `UAV/UAV.xml` 和 `UAV/UAV_Delta.xml` 包含 HIL 所需传感器：
  - `body_gyro`
  - `body_linacc`
  - `body_mag`
- 飞行动作器显式命名为 `motor_1..motor_4`。
- 飞行电机顺序已经对齐 PX4 Quad X：
  - 左前
  - 右后
  - 右前
  - 左后
- Delta 机械臂动作器保留在模型中，但 PX4 输出只映射到四个飞行电机。
- 仓库包含第一版完整脚本集合：
  - `bootstrap_workspace.sh`
  - `install_ubuntu_deps.sh`
  - `apply_px4_patch.sh`
  - `build_px4.sh`
  - `build_ros2.sh`
  - `run_bridge.sh`
  - `run_px4.sh`
  - `run_qgc.sh`
  - `run_stack.sh`
  - `run_ros2_agent.sh`
  - `run_offboard_control.sh`
- 仓库包含 ROS 2 工作空间和 PX4 Offboard 相关包：
  - `ros2_ws/src/px4_mujoco_ros2_control`
  - `ros2_ws/src/px4_mujoco_ros2_bringup`
  - `ros2_ws/src/uav_control`
- PX4 和 ROS 2 Offboard 运行时使用 `--no-local-hover`，确保 PX4 是唯一飞控。
- `px4_mujoco_ros2_control` 已经整理为通用 Offboard 网关，悬停和画圈测试放在 `uav_control`。
- 官方 PX4 patch 存放在：
  - `integrations/px4/patches/release-1.15-mujoco-delta.patch`

## 当前验证水平

以下内容已经在本地验证过：

- shell 脚本语法
- Python 语法编译
- 模型约束检查
- bridge 本地 smoke run，使用 `--no-mavlink`
- `make run-local` 直接运行 MuJoCo 自动起飞/悬停可视化路径
- PX4 patch 可应用性，使用 `git apply --check`
- ROS 2 工作空间构建
- PX4 ROS 2 消息链路可见，例如：
  - `/fmu/out/vehicle_status`
  - `/fmu/out/vehicle_local_position`
- PX4 Offboard 请求可以通过 ROS 2 链路到达 PX4
- `hover_test` 和 `waypoint_cruise` 可以作为 `uav_control` 上层节点运行

## 后续待做

剩余工作不再是基础项目结构，而是端到端稳定性和验证：

1. 稳定分终端 ROS 2 Offboard 启动流程中的 PX4 本地位置 readiness。
2. 确认 Offboard 路径下 PX4 解锁、起飞、悬停和保持稳定。
3. 验证 Delta 机械臂场景和普通无人机场景在 PX4 控制下都能稳定运行。
4. 加强旧 PX4 和 Micro XRCE-DDS Agent 进程清理逻辑。
5. 增加更丰富的 `uav_control` 示例，例如圆轨迹、平滑轨迹和重规划。
