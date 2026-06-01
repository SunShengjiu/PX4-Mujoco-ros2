import math
from dataclasses import dataclass

from px4_msgs.msg import OffboardControlMode


@dataclass
class ManagedSetpoint:
    position: list[float]
    velocity: list[float]
    acceleration: list[float]
    yaw: float
    yawspeed: float
    source: str

    def mode_name(self) -> str:
        if any(math.isfinite(value) for value in self.position):
            return "position"
        if any(math.isfinite(value) for value in self.velocity):
            return "velocity"
        if any(math.isfinite(value) for value in self.acceleration):
            return "acceleration"
        return "invalid"

    def fill_offboard_control_mode(self, msg: OffboardControlMode) -> None:
        mode_name = self.mode_name()
        msg.position = mode_name == "position"
        msg.velocity = mode_name == "velocity"
        msg.acceleration = mode_name == "acceleration"
        msg.attitude = False
        msg.body_rate = False


def copy_scalar(value: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def copy_xyz(values: list[float]) -> list[float]:
    copied = [math.nan, math.nan, math.nan]
    for index, value in enumerate(list(values)[:3]):
        copied[index] = copy_scalar(value)
    return copied
