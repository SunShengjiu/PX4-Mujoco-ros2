import math
import time

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from px4_msgs.msg import TrajectorySetpoint

from .setpoint_types import ManagedSetpoint
from .setpoint_types import copy_scalar
from .setpoint_types import copy_xyz


class SetpointMux:
    """Keeps the newest external setpoint while it is fresh."""

    def __init__(self, timeout_s: float) -> None:
        self.timeout_us = int(max(timeout_s, 0.1) * 1e6)
        self.setpoint: ManagedSetpoint | None = None
        self.arrival_us = 0
        self.timeout_logged = False

    def update(self, setpoint: ManagedSetpoint) -> bool:
        if setpoint.mode_name() == "invalid":
            return False

        self.setpoint = setpoint
        self.arrival_us = int(time.monotonic_ns() / 1000)
        self.timeout_logged = False
        return True

    def active(self) -> ManagedSetpoint | None:
        if self.setpoint is None:
            return None

        now_us = int(time.monotonic_ns() / 1000)
        if now_us - self.arrival_us <= self.timeout_us:
            return self.setpoint

        return None


def setpoint_from_px4(msg: TrajectorySetpoint) -> ManagedSetpoint:
    return ManagedSetpoint(
        position=copy_xyz(getattr(msg, "position", [math.nan, math.nan, math.nan])),
        velocity=copy_xyz(getattr(msg, "velocity", [math.nan, math.nan, math.nan])),
        acceleration=copy_xyz(getattr(msg, "acceleration", [math.nan, math.nan, math.nan])),
        yaw=copy_scalar(getattr(msg, "yaw", math.nan)),
        yawspeed=copy_scalar(getattr(msg, "yawspeed", math.nan)),
        source="trajectory_setpoint",
    )


def setpoint_from_pose(msg: PoseStamped) -> ManagedSetpoint:
    return ManagedSetpoint(
        position=[
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ],
        velocity=[math.nan, math.nan, math.nan],
        acceleration=[math.nan, math.nan, math.nan],
        yaw=yaw_from_quaternion(msg.pose.orientation),
        yawspeed=math.nan,
        source="cmd_pose",
    )


def setpoint_from_twist(msg: TwistStamped) -> ManagedSetpoint:
    return ManagedSetpoint(
        position=[math.nan, math.nan, math.nan],
        velocity=[
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
        ],
        acceleration=[math.nan, math.nan, math.nan],
        yaw=math.nan,
        yawspeed=float(msg.twist.angular.z),
        source="cmd_twist",
    )


def yaw_from_quaternion(quaternion: Quaternion) -> float:
    siny_cosp = 2.0 * (
        float(quaternion.w) * float(quaternion.z) + float(quaternion.x) * float(quaternion.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        float(quaternion.y) * float(quaternion.y) + float(quaternion.z) * float(quaternion.z)
    )
    return math.atan2(siny_cosp, cosy_cosp)
