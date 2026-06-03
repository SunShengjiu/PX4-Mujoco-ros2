import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WaypointCruiseNode(Node):
    """Hold near the current position, fly one smooth circle, then hold again."""

    def __init__(self) -> None:
        super().__init__("waypoint_cruise")

        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("z", -1.0)
        self.declare_parameter("radius", 0.8)
        self.declare_parameter("period_s", 24.0)
        self.declare_parameter("pre_circle_hold_s", 5.0)
        self.declare_parameter("hold_time_s", 999999.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("cmd_pose_topic", "/offboard_control/cmd_pose")
        self.declare_parameter("odom_topic", "/offboard_control/odom")
        self.declare_parameter("frame_id", "map_ned")
        self.declare_parameter("clockwise", False)
        self.declare_parameter("use_current_position", True)

        self.center_x = float(self.get_parameter("center_x").value)
        self.center_y = float(self.get_parameter("center_y").value)
        self.z = float(self.get_parameter("z").value)
        self.radius = max(float(self.get_parameter("radius").value), 0.01)
        self.period_s = max(float(self.get_parameter("period_s").value), 1.0)
        self.pre_circle_hold_s = max(float(self.get_parameter("pre_circle_hold_s").value), 0.0)
        self.hold_time_s = max(float(self.get_parameter("hold_time_s").value), 0.0)
        self.yaw = float(self.get_parameter("yaw").value)
        self.rate_hz = max(float(self.get_parameter("rate_hz").value), 1.0)
        self.cmd_pose_topic = str(self.get_parameter("cmd_pose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.clockwise = self.parameter_bool("clockwise")
        self.use_current_position = self.parameter_bool("use_current_position")

        self.start_time_s: float | None = None
        self.circle_start_s: float | None = None
        self.start_x: float | None = None
        self.start_y: float | None = None
        self.completed_circle = False
        self.logged_hold = False
        self.waiting_for_odom_logged = False

        self.publisher = self.create_publisher(PoseStamped, self.cmd_pose_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_timer(1.0 / self.rate_hz, self.on_timer)

        direction = "clockwise" if self.clockwise else "counter-clockwise"
        self.get_logger().info(
            "Circle cruise ready: "
            f"z={self.z:.2f}, radius={self.radius:.2f}, "
            f"pre_circle_hold_s={self.pre_circle_hold_s:.2f}, period_s={self.period_s:.2f}, "
            f"direction={direction}, use_current_position={int(self.use_current_position)}, "
            f"cmd_pose_topic={self.cmd_pose_topic}, odom_topic={self.odom_topic}"
        )

        if not self.use_current_position:
            self.set_start_from_config()

    def on_odom(self, msg: Odometry) -> None:
        if self.start_x is not None and self.start_y is not None:
            return

        if not self.use_current_position:
            return

        self.start_x = float(msg.pose.pose.position.x)
        self.start_y = float(msg.pose.pose.position.y)
        self.set_circle_center_from_start()
        self.get_logger().info(
            "Circle start locked from odom: "
            f"start_x={self.start_x:.2f}, start_y={self.start_y:.2f}, "
            f"center_x={self.center_x:.2f}, center_y={self.center_y:.2f}, z={self.z:.2f}"
        )

    def on_timer(self) -> None:
        if self.start_x is None or self.start_y is None:
            if not self.waiting_for_odom_logged:
                self.get_logger().info("Waiting for odometry before publishing circle setpoints")
                self.waiting_for_odom_logged = True
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time_s is None:
            self.start_time_s = now_s

        elapsed_s = now_s - self.start_time_s
        if elapsed_s < self.pre_circle_hold_s:
            self.publish_pose(self.start_x, self.start_y, self.z)
            return

        if self.circle_start_s is None:
            self.circle_start_s = now_s
            self.get_logger().info("Starting one circle from the current hold point")

        circle_elapsed_s = now_s - self.circle_start_s
        if circle_elapsed_s < self.period_s:
            progress = circle_elapsed_s / self.period_s
            angle = 2.0 * math.pi * progress
            if self.clockwise:
                angle = -angle
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)
        else:
            x = self.start_x
            y = self.start_y
            if not self.completed_circle:
                self.completed_circle = True
                self.get_logger().info(
                    "Circle complete; holding final point "
                    f"x={x:.2f}, y={y:.2f}, z={self.z:.2f}"
                )

        if circle_elapsed_s > self.period_s + self.hold_time_s and not self.logged_hold:
            self.logged_hold = True
            self.get_logger().info("Hold time elapsed; continuing to publish final hold point")

        self.publish_pose(x, y, self.z)

    def set_start_from_config(self) -> None:
        self.start_x = self.center_x + self.radius
        self.start_y = self.center_y

    def set_circle_center_from_start(self) -> None:
        if self.start_x is None or self.start_y is None:
            return
        self.center_x = self.start_x - self.radius
        self.center_y = self.start_y

    def publish_pose(self, x: float, y: float, z: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation = self.quaternion_from_yaw(self.yaw)
        self.publisher.publish(msg)

    def parameter_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"1", "true", "yes", "on"}:
                return True
            if normalized in {"0", "false", "no", "off"}:
                return False
        return bool(value)

    def quaternion_from_yaw(self, yaw: float) -> Quaternion:
        quaternion = Quaternion()
        quaternion.z = math.sin(yaw * 0.5)
        quaternion.w = math.cos(yaw * 0.5)
        return quaternion


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaypointCruiseNode()
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
