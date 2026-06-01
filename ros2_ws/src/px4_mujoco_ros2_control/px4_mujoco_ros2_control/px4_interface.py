import time
from typing import Optional

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from .setpoint_types import ManagedSetpoint


def px4_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class Px4OffboardInterface:
    """PX4 input publishers used by the Offboard node."""

    def __init__(
        self,
        node,
        target_system: int,
        target_component: int,
        input_prefix: str = "/fmu/in",
    ) -> None:
        self._node = node
        self.target_system = target_system
        self.target_component = target_component
        self.input_prefix = normalize_topic_prefix(input_prefix)
        qos = px4_qos()

        self.offboard_mode_pub = node.create_publisher(
            OffboardControlMode,
            topic_from_prefix(self.input_prefix, "offboard_control_mode"),
            qos,
        )
        self.trajectory_pub = node.create_publisher(
            TrajectorySetpoint,
            topic_from_prefix(self.input_prefix, "trajectory_setpoint"),
            qos,
        )
        self.command_pub = node.create_publisher(
            VehicleCommand,
            topic_from_prefix(self.input_prefix, "vehicle_command"),
            qos,
        )

    def timestamp_us(self) -> int:
        return int(time.monotonic_ns() / 1000)

    def publish_setpoint(self, setpoint: ManagedSetpoint) -> None:
        mode = OffboardControlMode()
        mode.timestamp = self.timestamp_us()
        setpoint.fill_offboard_control_mode(mode)
        self.offboard_mode_pub.publish(mode)

        trajectory = TrajectorySetpoint()
        trajectory.timestamp = mode.timestamp
        trajectory.position = list(setpoint.position)
        trajectory.velocity = list(setpoint.velocity)
        trajectory.acceleration = list(setpoint.acceleration)
        trajectory.yaw = setpoint.yaw
        trajectory.yawspeed = setpoint.yawspeed
        self.trajectory_pub.publish(trajectory)

    def publish_vehicle_command(
        self,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        target_system: Optional[int] = None,
        target_component: Optional[int] = None,
        source_system: int = 1,
        source_component: int = 1,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.target_system = self.target_system if target_system is None else int(target_system)
        msg.target_component = self.target_component if target_component is None else int(target_component)
        msg.source_system = int(source_system)
        msg.source_component = int(source_component)
        msg.from_external = True
        self.command_pub.publish(msg)

    def request_offboard_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def arm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )


def normalize_topic_prefix(prefix: str) -> str:
    normalized = str(prefix).strip()
    if not normalized:
        return ""
    return normalized.rstrip("/")


def topic_from_prefix(prefix: str, name: str) -> str:
    normalized = normalize_topic_prefix(prefix)
    if not normalized:
        return name
    return f"{normalized}/{name.lstrip('/')}"
