import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from rclpy.node import Node


class HoverTestNode(Node):
    """Publish a simple fixed pose command for testing the Offboard gateway."""

    def __init__(self) -> None:
        super().__init__("hover_test")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", -1.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("rate_hz", 20.0)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.yaw = float(self.get_parameter("yaw").value)
        rate_hz = max(float(self.get_parameter("rate_hz").value), 1.0)

        self.publisher = self.create_publisher(PoseStamped, "/offboard_control/cmd_pose", 10)
        self.create_timer(1.0 / rate_hz, self.publish_setpoint)

        self.get_logger().info(
            "Hover test publishing /offboard_control/cmd_pose: "
            f"x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, yaw={self.yaw:.2f}"
        )

    def publish_setpoint(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map_ned"
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        msg.pose.orientation = self.quaternion_from_yaw(self.yaw)
        self.publisher.publish(msg)

    def quaternion_from_yaw(self, yaw: float) -> Quaternion:
        quaternion = Quaternion()
        quaternion.z = math.sin(yaw * 0.5)
        quaternion.w = math.cos(yaw * 0.5)
        return quaternion


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HoverTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
