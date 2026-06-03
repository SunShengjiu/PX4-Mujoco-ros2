# 如何新增 UAV 控制功能

这份文档说明如何在当前 ROS 2 工作空间里新增一个上层 UAV 控制功能。

核心原则：

```text
任务逻辑写在 uav_control。
不要把悬停、航点、画圈、轨迹生成、规划、重规划逻辑写进 px4_mujoco_ros2_control。
```

## 包职责

```text
px4_mujoco_ros2_control
  只作为 PX4 Offboard 网关。
  把通用 ROS 2 命令转换成 PX4 话题。

uav_control
  存放上层控制、规划和测试节点。
  新的飞行行为优先写在这里。

px4_mujoco_ros2_bringup
  只存放 launch 文件。

px4_msgs
  只提供 PX4 ROS 2 消息定义。
```

## 常规数据流

你的新节点通常走这条链路：

```text
uav_control node
  -> /offboard_control/cmd_pose
  -> px4_mujoco_ros2_control
  -> /fmu/in/trajectory_setpoint
  -> PX4
```

如果节点需要读取飞机状态：

```text
PX4
  -> /fmu/out/vehicle_local_position
  -> px4_mujoco_ros2_control
  -> /offboard_control/odom
  -> uav_control node
```

## 第 1 步：创建新节点文件

在下面目录中新建 Python 文件：

```text
ros2_ws/src/uav_control/uav_control/
```

例如：

```text
ros2_ws/src/uav_control/uav_control/my_task.py
```

文件名建议直接描述行为：

```text
circle_hold.py
figure_eight.py
path_follow.py
replan_test.py
```

## 第 2 步：发布通用控制命令

如果是位置控制，发布：

```text
topic: /offboard_control/cmd_pose
type:  geometry_msgs/msg/PoseStamped
```

如果是速度控制，发布：

```text
topic: /offboard_control/cmd_twist
type:  geometry_msgs/msg/TwistStamped
```

第一次写新功能时，建议优先使用 `cmd_pose`，因为位置 setpoint 更容易调试。

## 第 3 步：需要状态时订阅 odom

如果你的节点需要当前飞机位置，订阅：

```text
topic: /offboard_control/odom
type:  nav_msgs/msg/Odometry
```

常见用途：

- 锁定当前起飞点
- 判断什么时候开始轨迹
- 判断是否到达目标点
- 从当前位置重新规划

## 第 4 步：最小节点模板

下面这个例子会发布一个简单移动目标。

```python
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class MyTaskNode(Node):
    def __init__(self) -> None:
        super().__init__("my_task")
        self.publisher = self.create_publisher(PoseStamped, "/offboard_control/cmd_pose", 10)
        self.start_time_s: float | None = None
        self.create_timer(0.05, self.on_timer)

    def on_timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time_s is None:
            self.start_time_s = now_s

        elapsed_s = now_s - self.start_time_s

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map_ned"
        msg.pose.position.x = math.sin(elapsed_s * 0.2)
        msg.pose.position.y = 0.0
        msg.pose.position.z = -1.0
        msg.pose.orientation.w = 1.0
        self.publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MyTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

## 第 5 步：注册可执行入口

编辑：

```text
ros2_ws/src/uav_control/setup.py
```

在 `console_scripts` 里添加你的节点：

```python
entry_points={
    "console_scripts": [
        "hover_test = uav_control.hover_test:main",
        "waypoint_cruise = uav_control.waypoint_cruise:main",
        "my_task = uav_control.my_task:main",
    ],
},
```

之后 ROS 2 就可以这样运行：

```bash
ros2 run uav_control my_task
```

## 第 6 步：构建

从仓库根目录运行：

```bash
./scripts/build_ros2.sh
```

或者只构建上层控制包：

```bash
cd ros2_ws
colcon build --symlink-install --packages-select uav_control
source install/setup.bash
```

检查 ROS 2 是否能找到你的可执行入口：

```bash
ros2 pkg executables uav_control
```

## 第 7 步：运行完整链路

先启动 PX4 + MuJoCo + ROS 2 Offboard 基础链路。

终端 1：

```bash
make run-ros2-agent
```

终端 2：

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
PX4_MUJOCO_BRIDGE_EXTRA_ARGS="--px4-alignment-hold-seconds 2.0 --debug-hil-rate" \
./scripts/run_bridge.sh --no-local-hover --physics-substeps-per-sensor 10
```

终端 3：

```bash
PX4_MUJOCO_MODEL=UAV/scene.xml \
PX4_MUJOCO_PX4_AUTOSTART=22003 \
PX4_MUJOCO_HOVER_THRUST=0.343 \
./scripts/run_px4.sh
```

终端 4：

```bash
./scripts/run_offboard_control.sh
```

终端 5：

```bash
ros2 run uav_control my_task
```

## 第 8 步：可选运行脚本

如果你希望用脚本快速启动，可以新建：

```text
scripts/run_my_task.sh
```

参考下面两个脚本：

```text
scripts/run_hover_test.sh
scripts/run_waypoint_cruise.sh
```

脚本里应该 source ROS 2、source `ros2_ws/install/setup.bash`，然后调用：

```bash
ros2 run uav_control my_task "$@"
```

## 第 9 步：可选 launch 文件

如果你希望支持 launch，在下面目录添加 launch 文件：

```text
ros2_ws/src/px4_mujoco_ros2_bringup/launch/
```

例如：

```text
my_task.launch.py
offboard_my_task.launch.py
```

然后在这里注册：

```text
ros2_ws/src/px4_mujoco_ros2_bringup/setup.py
```

## 什么时候需要修改 px4_mujoco_ros2_control

只有当你要修改“通用 PX4 接口本身”时，才应该改 `px4_mujoco_ros2_control`。

合理原因：

- 新增一个通用命令话题
- 把新的 PX4 状态话题暴露成通用 ROS 2 状态
- 支持多机命名空间
- 修改 Offboard 解锁或模式请求策略

不合理原因：

- 添加悬停
- 添加画圈
- 添加航点巡航
- 添加轨迹生成
- 添加避障
- 添加重规划

这些都应该放在 `uav_control`。

## 快速检查清单

测试新功能前先运行：

```bash
python3 -m py_compile ros2_ws/src/uav_control/uav_control/*.py
./scripts/build_ros2.sh
source ros2_ws/install/setup.bash
ros2 pkg executables uav_control
```

飞行测试时建议观察：

```bash
ros2 topic echo /offboard_control/cmd_pose
ros2 topic echo /offboard_control/odom
ros2 topic echo /fmu/in/trajectory_setpoint
```

如果飞机运动异常，优先检查：

- 节点发布的是不是 NED 坐标？
- `z` 是否为负数表示向上？
- 目标点是相对当前位置，还是绝对地图点？
- 目标点变化速度是否过快？
- Offboard 网关是否收到了 `cmd_pose`？
