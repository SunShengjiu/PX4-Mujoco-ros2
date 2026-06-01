import math
from ast import literal_eval
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


DEFAULT_WAYPOINTS = [
    0.0,
    0.0,
    -1.0,
    1.0,
    0.0,
    -1.0,
    1.0,
    1.0,
    -1.0,
    0.0,
    1.0,
    -1.0,
]


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float


class WaypointCruiseNode(Node):
    """Cycle through fixed NED waypoints using the generic Offboard pose interface."""

    def __init__(self) -> None:
        super().__init__("waypoint_cruise")

        self.declare_parameter("waypoints", DEFAULT_WAYPOINTS)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("arrival_radius", 0.25)
        self.declare_parameter("hold_time_s", 2.0)
        self.declare_parameter("cmd_pose_topic", "/offboard_control/cmd_pose")
        self.declare_parameter("odom_topic", "/offboard_control/odom")
        self.declare_parameter("frame_id", "map_ned")
        self.declare_parameter("loop", True)

        self.waypoints = self.parse_waypoints(self.get_parameter("waypoints").value)
        self.yaw = float(self.get_parameter("yaw").value)
        rate_hz = max(float(self.get_parameter("rate_hz").value), 1.0)
        self.arrival_radius = max(float(self.get_parameter("arrival_radius").value), 0.01)
        self.hold_time_s = max(float(self.get_parameter("hold_time_s").value), 0.0)
        self.cmd_pose_topic = str(self.get_parameter("cmd_pose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.loop = self.parameter_bool("loop")

        self.current_index = 0
        self.arrival_started_s: float | None = None
        self.latest_odom: Odometry | None = None

        self.publisher = self.create_publisher(PoseStamped, self.cmd_pose_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            "Waypoint cruise ready: "
            f"count={len(self.waypoints)}, loop={int(self.loop)}, "
            f"arrival_radius={self.arrival_radius:.2f}, hold_time_s={self.hold_time_s:.2f}, "
            f"cmd_pose_topic={self.cmd_pose_topic}, odom_topic={self.odom_topic}"
        )
        self.log_current_waypoint()

    def parse_waypoints(self, values) -> list[Waypoint]:
        if isinstance(values, str):
            try:
                values = literal_eval(values)
            except (SyntaxError, ValueError) as exc:
                raise ValueError(
                    "waypoints must be a list of numbers or a string like "
                    "'[0.0, 0.0, -1.0]'"
                ) from exc

        numbers = [float(value) for value in list(values)]
        if not numbers or len(numbers) % 3 != 0:
            raise ValueError("waypoints must contain one or more x,y,z triples")

        return [
            Waypoint(numbers[index], numbers[index + 1], numbers[index + 2])
            for index in range(0, len(numbers), 3)
        ]

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def on_timer(self) -> None:
        self.publish_current_waypoint()
        if self.latest_odom is None:
            return

        distance = self.distance_to_current_waypoint(self.latest_odom)
        now_s = self.get_clock().now().nanoseconds * 1e-9

        if distance > self.arrival_radius:
            self.arrival_started_s = None
            return

        if self.arrival_started_s is None:
            self.arrival_started_s = now_s
            self.get_logger().info(
                f"Arrived near waypoint {self.current_index}: distance={distance:.2f} m"
            )
            return

        if now_s - self.arrival_started_s >= self.hold_time_s:
            self.advance_waypoint()

    def publish_current_waypoint(self) -> None:
        waypoint = self.waypoints[self.current_index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = waypoint.x
        msg.pose.position.y = waypoint.y
        msg.pose.position.z = waypoint.z
        msg.pose.orientation = self.quaternion_from_yaw(self.yaw)
        self.publisher.publish(msg)

    def distance_to_current_waypoint(self, odom: Odometry) -> float:
        waypoint = self.waypoints[self.current_index]
        dx = float(odom.pose.pose.position.x) - waypoint.x
        dy = float(odom.pose.pose.position.y) - waypoint.y
        dz = float(odom.pose.pose.position.z) - waypoint.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def advance_waypoint(self) -> None:
        if self.current_index == len(self.waypoints) - 1 and not self.loop:
            self.arrival_started_s = None
            return

        self.current_index = (self.current_index + 1) % len(self.waypoints)
        self.arrival_started_s = None
        self.log_current_waypoint()

    def log_current_waypoint(self) -> None:
        waypoint = self.waypoints[self.current_index]
        self.get_logger().info(
            f"Target waypoint {self.current_index}: "
            f"x={waypoint.x:.2f}, y={waypoint.y:.2f}, z={waypoint.z:.2f}, yaw={self.yaw:.2f}"
        )

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
    try:
        node = WaypointCruiseNode()
    except ValueError as exc:
        fallback = Node("waypoint_cruise_config_error")
        fallback.get_logger().error(str(exc))
        fallback.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

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
