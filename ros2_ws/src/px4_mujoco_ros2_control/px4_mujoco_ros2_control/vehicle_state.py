from dataclasses import dataclass
from dataclasses import field

from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


@dataclass
class LocalPosition:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    heading: float = 0.0
    xy_valid: bool = False
    z_valid: bool = False
    v_xy_valid: bool = False
    v_z_valid: bool = False
    heading_good_for_control: bool = False

    @property
    def ready(self) -> bool:
        return (
            self.xy_valid
            and self.z_valid
            and self.v_xy_valid
            and self.v_z_valid
            and self.heading_good_for_control
        )

    @classmethod
    def from_msg(cls, msg: VehicleLocalPosition) -> "LocalPosition":
        return cls(
            x=float(getattr(msg, "x", 0.0)),
            y=float(getattr(msg, "y", 0.0)),
            z=float(getattr(msg, "z", 0.0)),
            vx=float(getattr(msg, "vx", 0.0)),
            vy=float(getattr(msg, "vy", 0.0)),
            vz=float(getattr(msg, "vz", 0.0)),
            heading=float(getattr(msg, "heading", 0.0)),
            xy_valid=bool(getattr(msg, "xy_valid", False)),
            z_valid=bool(getattr(msg, "z_valid", False)),
            v_xy_valid=bool(getattr(msg, "v_xy_valid", False)),
            v_z_valid=bool(getattr(msg, "v_z_valid", False)),
            heading_good_for_control=bool(getattr(msg, "heading_good_for_control", False)),
        )


@dataclass
class VehicleState:
    status_received: bool = False
    control_mode_received: bool = False
    nav_state: int = VehicleStatus.NAVIGATION_STATE_MANUAL
    arming_state: int = VehicleStatus.ARMING_STATE_DISARMED
    preflight_ok: bool = False
    offboard_enabled: bool = False
    local_position: LocalPosition = field(default_factory=LocalPosition)

    @property
    def ready_for_offboard(self) -> bool:
        return self.status_received and self.control_mode_received and self.local_position.ready

    def is_ready_for_offboard(self, require_local_position: bool = True) -> bool:
        if not self.status_received or not self.control_mode_received:
            return False
        if require_local_position and not self.local_position.ready:
            return False
        return True

    @property
    def armed(self) -> bool:
        return self.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def in_offboard(self) -> bool:
        return self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.offboard_enabled

    def update_status(self, msg: VehicleStatus) -> bool:
        was_armed = self.armed
        self.status_received = True
        self.nav_state = int(msg.nav_state)
        self.arming_state = int(msg.arming_state)
        self.preflight_ok = bool(msg.pre_flight_checks_pass)
        return was_armed and not self.armed

    def update_control_mode(self, msg: VehicleControlMode) -> None:
        self.control_mode_received = True
        self.offboard_enabled = bool(msg.flag_control_offboard_enabled)

    def update_local_position(self, msg: VehicleLocalPosition) -> None:
        self.local_position = LocalPosition.from_msg(msg)
