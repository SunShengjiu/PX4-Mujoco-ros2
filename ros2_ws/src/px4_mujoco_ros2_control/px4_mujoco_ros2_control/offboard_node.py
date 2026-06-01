import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node

from .command_inputs import SetpointMux
from .command_inputs import setpoint_from_pose
from .command_inputs import setpoint_from_px4
from .command_inputs import setpoint_from_twist
from .px4_interface import Px4OffboardInterface
from .px4_interface import px4_qos
from .px4_interface import topic_from_prefix
from .setpoint_types import ManagedSetpoint
from .vehicle_state import VehicleState


class OffboardControlNode(Node):
    """Small PX4 Offboard node with generic ROS 2 command inputs."""

    def __init__(self) -> None:
        super().__init__("offboard_control")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("warmup_setpoints", 20)
        self.declare_parameter("mode_request_interval", 1.0)
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("auto_request_offboard_and_arm", True)
        self.declare_parameter("auto_request_offboard", True)
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("require_local_position_for_offboard", True)
        self.declare_parameter("accept_external_vehicle_commands", True)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("px4_input_prefix", "/fmu/in")
        self.declare_parameter("px4_output_prefix", "/fmu/out")
        self.declare_parameter("trajectory_setpoint_topic", "~/trajectory_setpoint")
        self.declare_parameter("cmd_pose_topic", "~/cmd_pose")
        self.declare_parameter("cmd_twist_topic", "~/cmd_twist")
        self.declare_parameter("vehicle_command_topic", "~/vehicle_command")
        self.declare_parameter("odom_topic", "~/odom")
        self.declare_parameter("odom_frame_id", "map_ned")
        self.declare_parameter("base_frame_id", "base_link")

        target_system = int(self.get_parameter("target_system").value)
        target_component = int(self.get_parameter("target_component").value)
        command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        rate_hz = max(float(self.get_parameter("rate_hz").value), 2.0)
        px4_input_prefix = str(self.get_parameter("px4_input_prefix").value)
        self.px4_output_prefix = str(self.get_parameter("px4_output_prefix").value)
        self.trajectory_setpoint_topic = str(
            self.get_parameter("trajectory_setpoint_topic").value
        )
        self.cmd_pose_topic = str(self.get_parameter("cmd_pose_topic").value)
        self.cmd_twist_topic = str(self.get_parameter("cmd_twist_topic").value)
        self.vehicle_command_topic = str(self.get_parameter("vehicle_command_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.state = VehicleState()
        self.setpoints = SetpointMux(command_timeout_s)
        self.px4 = Px4OffboardInterface(
            self,
            target_system,
            target_component,
            input_prefix=px4_input_prefix,
        )

        self.warmup_setpoints = int(self.get_parameter("warmup_setpoints").value)
        self.mode_request_interval_s = max(
            float(self.get_parameter("mode_request_interval").value),
            0.2,
        )
        legacy_auto_mode = self.parameter_bool("auto_request_offboard_and_arm")
        self.auto_request_offboard = legacy_auto_mode and self.parameter_bool(
            "auto_request_offboard"
        )
        self.auto_arm = legacy_auto_mode and self.parameter_bool("auto_arm")
        self.require_local_position_for_offboard = self.parameter_bool(
            "require_local_position_for_offboard"
        )
        self.accept_external_vehicle_commands = self.parameter_bool(
            "accept_external_vehicle_commands"
        )
        self.publish_odom = self.parameter_bool("publish_odom")
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)

        self.setpoint_count = 0
        self.last_mode_request_s = 0.0
        self.last_status_log_s = 0.0
        self.last_active_source = ""

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._create_px4_subscriptions()
        self._create_command_subscriptions()

        self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(
            "Offboard control ready: "
            f"inputs={self.trajectory_setpoint_topic},{self.cmd_pose_topic},{self.cmd_twist_topic}, "
            "no built-in hover fallback, "
            f"state={self.odom_topic}"
        )

    def _create_px4_subscriptions(self) -> None:
        qos = px4_qos()
        self.create_subscription(
            VehicleStatus,
            topic_from_prefix(self.px4_output_prefix, "vehicle_status"),
            self.on_vehicle_status,
            qos,
        )
        self.create_subscription(
            VehicleControlMode,
            topic_from_prefix(self.px4_output_prefix, "vehicle_control_mode"),
            self.on_vehicle_control_mode,
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            topic_from_prefix(self.px4_output_prefix, "vehicle_local_position"),
            self.on_vehicle_local_position,
            qos,
        )

    def _create_command_subscriptions(self) -> None:
        self.create_subscription(
            TrajectorySetpoint,
            self.trajectory_setpoint_topic,
            self.on_px4_setpoint,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.cmd_pose_topic,
            self.on_cmd_pose,
            10,
        )
        self.create_subscription(
            TwistStamped,
            self.cmd_twist_topic,
            self.on_cmd_twist,
            10,
        )
        self.create_subscription(
            VehicleCommand,
            self.vehicle_command_topic,
            self.on_vehicle_command,
            10,
        )

    def on_vehicle_status(self, msg: VehicleStatus) -> None:
        self.state.update_status(msg)

    def on_vehicle_control_mode(self, msg: VehicleControlMode) -> None:
        self.state.update_control_mode(msg)

    def on_vehicle_local_position(self, msg: VehicleLocalPosition) -> None:
        self.state.update_local_position(msg)
        self.publish_odometry()

    def on_px4_setpoint(self, msg: TrajectorySetpoint) -> None:
        self.register_setpoint(setpoint_from_px4(msg))

    def on_cmd_pose(self, msg: PoseStamped) -> None:
        self.register_setpoint(setpoint_from_pose(msg))

    def on_cmd_twist(self, msg: TwistStamped) -> None:
        self.register_setpoint(setpoint_from_twist(msg))

    def register_setpoint(self, setpoint: ManagedSetpoint) -> None:
        if not self.setpoints.update(setpoint):
            self.get_logger().warn("Ignoring external setpoint because it has no active control fields")

    def on_vehicle_command(self, msg: VehicleCommand) -> None:
        if not self.accept_external_vehicle_commands:
            return

        self.px4.publish_vehicle_command(
            int(msg.command),
            param1=float(msg.param1),
            param2=float(msg.param2),
            param3=float(msg.param3),
            param4=float(msg.param4),
            param5=float(msg.param5),
            param6=float(msg.param6),
            param7=float(msg.param7),
            target_system=int(msg.target_system) if int(msg.target_system) > 0 else None,
            target_component=int(msg.target_component) if int(msg.target_component) > 0 else None,
            source_system=int(msg.source_system) if int(msg.source_system) > 0 else 1,
            source_component=int(msg.source_component) if int(msg.source_component) > 0 else 1,
        )

    def on_timer(self) -> None:
        setpoint = self.active_setpoint()
        if setpoint is None:
            self.setpoint_count = 0
            self.log_status(None)
            return

        self.px4.publish_setpoint(setpoint)

        should_request_mode = self.auto_request_offboard or self.auto_arm
        if should_request_mode and self.setpoint_count >= self.warmup_setpoints:
            self.ensure_offboard_and_arm()

        self.log_status(setpoint)
        self.setpoint_count += 1

    def active_setpoint(self) -> ManagedSetpoint | None:
        setpoint = self.setpoints.active()
        if setpoint is None:
            if self.last_active_source != "none":
                self.get_logger().info("No active external setpoint; waiting for command input")
                self.last_active_source = "none"
            return None

        if setpoint.source != self.last_active_source:
            self.get_logger().info(f"Using setpoint source: {setpoint.source}")
            self.last_active_source = setpoint.source

        return setpoint

    def ensure_offboard_and_arm(self) -> None:
        now_s = time.monotonic()
        if now_s - self.last_mode_request_s < self.mode_request_interval_s:
            return

        if not self.state.is_ready_for_offboard(self.require_local_position_for_offboard):
            self.get_logger().warn(
                "Waiting for PX4 status/control mode"
                + (
                    " and valid local position before requesting Offboard/Arm"
                    if self.require_local_position_for_offboard
                    else " before requesting Offboard/Arm"
                )
            )
            self.last_mode_request_s = now_s
            return

        if self.auto_request_offboard and not self.state.in_offboard:
            self.px4.request_offboard_mode()

        if self.auto_arm and not self.state.armed:
            if not self.state.preflight_ok:
                self.get_logger().warn("PX4 preflight checks are not ready yet")
            self.px4.arm()

        self.last_mode_request_s = now_s

    def publish_odometry(self) -> None:
        if not self.publish_odom:
            return

        pose = self.state.local_position
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = pose.z
        msg.pose.pose.orientation = self.quaternion_from_yaw(pose.heading)
        msg.twist.twist.linear.x = pose.vx
        msg.twist.twist.linear.y = pose.vy
        msg.twist.twist.linear.z = pose.vz
        self.odom_pub.publish(msg)

    def log_status(self, setpoint: ManagedSetpoint | None) -> None:
        now_s = time.monotonic()
        if now_s - self.last_status_log_s < 2.0:
            return

        pose = self.state.local_position
        if setpoint is None:
            setpoint_status = "source=none,mode=waiting,pos=[],vel=[]"
        else:
            setpoint_status = (
                f"source={setpoint.source},"
                f"mode={setpoint.mode_name()},"
                f"pos={self.format_vector(setpoint.position)},"
                f"vel={self.format_vector(setpoint.velocity)}"
            )

        self.get_logger().info(
            "status="
            f"status_rx={int(self.state.status_received)},"
            f"control_rx={int(self.state.control_mode_received)},"
            f"local_ready={int(pose.ready)},"
            f"offboard={int(self.state.in_offboard)},"
            f"armed={int(self.state.armed)},"
            f"nav={self.state.nav_state} "
            f"setpoint={setpoint_status} "
            "local="
            f"x={pose.x:.2f},y={pose.y:.2f},z={pose.z:.2f},"
            f"vx={pose.vx:.2f},vy={pose.vy:.2f},vz={pose.vz:.2f}"
        )
        self.last_status_log_s = now_s

    def format_vector(self, values: list[float]) -> list[float | str]:
        return [round(value, 3) if math.isfinite(value) else "nan" for value in values]

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
    node = OffboardControlNode()
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
