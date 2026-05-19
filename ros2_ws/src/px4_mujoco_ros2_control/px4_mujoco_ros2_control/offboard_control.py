import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from px4_msgs.msg import EstimatorStatusFlags
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleStatus


UINT64_MAX = (1 << 64) - 1
MAX_REASONABLE_TIMESYNC_OFFSET_US = 10_000_000
MAX_REASONABLE_TIMESYNC_RTT_US = 1_000_000


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


class OffboardControl(Node):
    """PX4 Offboard helper with hover fallback and ROS 2 command inputs."""

    def __init__(self) -> None:
        super().__init__("offboard_control")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", -2.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("warmup_setpoints", 20)
        self.declare_parameter("mode_request_interval", 1.0)
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("auto_request_offboard_and_arm", True)
        self.declare_parameter("use_estimator_flags_gate", False)
        self.declare_parameter("require_ev_vel", False)
        self.declare_parameter("accept_external_vehicle_commands", True)

        self.hover_setpoint = ManagedSetpoint(
            position=[
                float(self.get_parameter("x").value),
                float(self.get_parameter("y").value),
                float(self.get_parameter("z").value),
            ],
            velocity=[math.nan, math.nan, math.nan],
            acceleration=[math.nan, math.nan, math.nan],
            yaw=float(self.get_parameter("yaw").value),
            yawspeed=math.nan,
            source="hover_fallback",
        )
        self.warmup_setpoints = int(self.get_parameter("warmup_setpoints").value)
        self.mode_request_interval = max(float(self.get_parameter("mode_request_interval").value), 0.2)
        self.target_system = int(self.get_parameter("target_system").value)
        self.target_component = int(self.get_parameter("target_component").value)
        self.command_timeout_us = int(max(float(self.get_parameter("command_timeout_s").value), 0.1) * 1e6)
        self.auto_request_offboard_and_arm = bool(self.get_parameter("auto_request_offboard_and_arm").value)
        self.use_estimator_flags_gate = bool(self.get_parameter("use_estimator_flags_gate").value)
        self.require_ev_vel = bool(self.get_parameter("require_ev_vel").value)
        self.accept_external_vehicle_commands = bool(
            self.get_parameter("accept_external_vehicle_commands").value
        )

        rate_hz = max(float(self.get_parameter("rate_hz").value), 2.0)
        px4_out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        px4_in_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            px4_in_qos,
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            px4_in_qos,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            px4_in_qos,
        )

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            px4_out_qos,
        )
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode,
            "/fmu/out/vehicle_control_mode",
            self.vehicle_control_mode_callback,
            px4_out_qos,
        )
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            px4_out_qos,
        )
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.vehicle_odometry_callback,
            px4_out_qos,
        )
        self.timesync_status_sub = self.create_subscription(
            TimesyncStatus,
            "/fmu/out/timesync_status",
            self.timesync_status_callback,
            px4_out_qos,
        )

        self.estimator_status_flags_sub = None
        if self.use_estimator_flags_gate:
            self.estimator_status_flags_sub = self.create_subscription(
                EstimatorStatusFlags,
                "/fmu/out/estimator_status_flags",
                self.estimator_status_flags_callback,
                px4_out_qos,
            )

        self.external_trajectory_sub = self.create_subscription(
            TrajectorySetpoint,
            "~/trajectory_setpoint",
            self.external_trajectory_callback,
            10,
        )
        self.external_vehicle_command_sub = self.create_subscription(
            VehicleCommand,
            "~/vehicle_command",
            self.external_vehicle_command_callback,
            10,
        )

        self.setpoint_counter = 0
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.pre_flight_checks_pass = False
        self.accepts_offboard_setpoints = False
        self.offboard_enabled = False
        self.last_request_time_us = 0
        self.arm_request_logged = False
        self.waiting_for_health_logged = False
        self.last_debug_log_us = 0
        self.last_waiting_for_status_log_us = 0
        self.last_timesync_warning_log_us = 0
        self.start_time_us = int(time.monotonic_ns() / 1000)
        self.vehicle_status_received = False
        self.vehicle_control_mode_received = False
        self.timesync_received = False
        self.estimator_flags = None
        self.local_position_state = {
            "xy_valid": False,
            "z_valid": False,
            "v_xy_valid": False,
            "v_z_valid": False,
            "heading_good_for_control": False,
        }
        self.vehicle_odometry_received = False
        self.timesync_offset_us = 0
        self.timesync_rtt_us = 0
        self.latest_external_setpoint: Optional[ManagedSetpoint] = None
        self.latest_external_setpoint_arrival_us = 0
        self.active_setpoint_source = ""
        self.external_setpoint_timeout_logged = False

        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.get_logger().info(
            "ROS 2 Offboard controller ready. Hover fallback is NED "
            f"x={self.hover_setpoint.position[0]:.2f}, "
            f"y={self.hover_setpoint.position[1]:.2f}, "
            f"z={self.hover_setpoint.position[2]:.2f}, "
            f"yaw={self.hover_setpoint.yaw:.2f}. "
            "External setpoint topic: ~/trajectory_setpoint. "
            "External vehicle-command topic: ~/vehicle_command."
        )

    def timestamp_us(self) -> int:
        base_timestamp_us = int(time.monotonic_ns() / 1000)
        if not self.timesync_received:
            return base_timestamp_us

        if (
            abs(self.timesync_offset_us) > MAX_REASONABLE_TIMESYNC_OFFSET_US
            or self.timesync_rtt_us < 0
            or self.timesync_rtt_us > MAX_REASONABLE_TIMESYNC_RTT_US
        ):
            if base_timestamp_us - self.last_timesync_warning_log_us > 2_000_000:
                self.get_logger().warn(
                    "Ignoring implausible PX4 timesync data and falling back to monotonic time. "
                    f"offset_us={self.timesync_offset_us}, rtt_us={self.timesync_rtt_us}"
                )
                self.last_timesync_warning_log_us = base_timestamp_us
            return base_timestamp_us

        adjusted_timestamp_us = base_timestamp_us + self.timesync_offset_us
        if adjusted_timestamp_us < 0 or adjusted_timestamp_us > UINT64_MAX:
            if base_timestamp_us - self.last_timesync_warning_log_us > 2_000_000:
                self.get_logger().warn(
                    "Ignoring overflowing PX4 timesync adjustment and falling back to monotonic time. "
                    f"offset_us={self.timesync_offset_us}"
                )
                self.last_timesync_warning_log_us = base_timestamp_us
            return base_timestamp_us

        return adjusted_timestamp_us

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status_received = True
        self.nav_state = int(msg.nav_state)
        self.arming_state = int(msg.arming_state)
        self.pre_flight_checks_pass = bool(msg.pre_flight_checks_pass)
        self.accepts_offboard_setpoints = bool(
            getattr(msg, "accepts_offboard_setpoints", self.accepts_offboard_setpoints)
        )

    def vehicle_control_mode_callback(self, msg: VehicleControlMode) -> None:
        self.vehicle_control_mode_received = True
        self.offboard_enabled = bool(msg.flag_control_offboard_enabled)

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.local_position_state = {
            "xy_valid": bool(getattr(msg, "xy_valid", False)),
            "z_valid": bool(getattr(msg, "z_valid", False)),
            "v_xy_valid": bool(getattr(msg, "v_xy_valid", False)),
            "v_z_valid": bool(getattr(msg, "v_z_valid", False)),
            "heading_good_for_control": bool(getattr(msg, "heading_good_for_control", False)),
        }

    def vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        self.vehicle_odometry_received = bool(msg.timestamp > 0)

    def timesync_status_callback(self, msg: TimesyncStatus) -> None:
        self.timesync_received = True
        self.timesync_offset_us = int(msg.estimated_offset)
        self.timesync_rtt_us = int(msg.round_trip_time)

    def estimator_status_flags_callback(self, msg: EstimatorStatusFlags) -> None:
        self.estimator_flags = {
            "tilt": bool(getattr(msg, "cs_tilt_align", False)),
            "yaw": bool(getattr(msg, "cs_yaw_align", False)),
            "ev_pos": bool(getattr(msg, "cs_ev_pos", False)),
            "ev_vel": bool(getattr(msg, "cs_ev_vel", False)),
            "ev_yaw": bool(getattr(msg, "cs_ev_yaw", False)),
            "ev_hgt": bool(getattr(msg, "cs_ev_hgt", False)),
            "vehicle_at_rest": bool(getattr(msg, "cs_vehicle_at_rest", False)),
        }

    def external_trajectory_callback(self, msg: TrajectorySetpoint) -> None:
        managed = ManagedSetpoint(
            position=self._copy_xyz(getattr(msg, "position", [math.nan, math.nan, math.nan])),
            velocity=self._copy_xyz(getattr(msg, "velocity", [math.nan, math.nan, math.nan])),
            acceleration=self._copy_xyz(getattr(msg, "acceleration", [math.nan, math.nan, math.nan])),
            yaw=self._copy_scalar(getattr(msg, "yaw", math.nan)),
            yawspeed=self._copy_scalar(getattr(msg, "yawspeed", math.nan)),
            source="external_ros2",
        )
        if managed.mode_name() == "invalid":
            self.get_logger().warn(
                "Ignoring external trajectory setpoint because all position, velocity, and acceleration "
                "components are NaN."
            )
            return

        self.latest_external_setpoint = managed
        self.latest_external_setpoint_arrival_us = int(time.monotonic_ns() / 1000)
        self.external_setpoint_timeout_logged = False

    def external_vehicle_command_callback(self, msg: VehicleCommand) -> None:
        if not self.accept_external_vehicle_commands:
            return

        self.publish_vehicle_command(
            int(msg.command),
            param1=float(msg.param1),
            param2=float(msg.param2),
            param3=float(msg.param3),
            param4=float(msg.param4),
            param5=float(msg.param5),
            param6=float(msg.param6),
            param7=float(msg.param7),
            target_system=int(msg.target_system) if int(msg.target_system) > 0 else self.target_system,
            target_component=(
                int(msg.target_component) if int(msg.target_component) > 0 else self.target_component
            ),
            source_system=int(msg.source_system) if int(msg.source_system) > 0 else 1,
            source_component=int(msg.source_component) if int(msg.source_component) > 0 else 1,
        )

    def timer_callback(self) -> None:
        active_setpoint = self.active_setpoint()
        self.publish_offboard_control_mode(active_setpoint)
        self.publish_trajectory_setpoint(active_setpoint)

        if self.auto_request_offboard_and_arm and self.setpoint_counter >= self.warmup_setpoints:
            self.ensure_offboard_and_arm()

        self.maybe_log_debug_state(active_setpoint)
        self.setpoint_counter += 1

    def active_setpoint(self) -> ManagedSetpoint:
        now_us = int(time.monotonic_ns() / 1000)
        if self.latest_external_setpoint is not None:
            age_us = now_us - self.latest_external_setpoint_arrival_us
            if age_us <= self.command_timeout_us:
                if self.active_setpoint_source != self.latest_external_setpoint.source:
                    self.get_logger().info("Using external ROS 2 trajectory setpoints")
                self.active_setpoint_source = self.latest_external_setpoint.source
                return self.latest_external_setpoint

            if not self.external_setpoint_timeout_logged:
                self.get_logger().warn(
                    "External ROS 2 trajectory setpoints timed out; falling back to hover hold."
                )
                self.external_setpoint_timeout_logged = True

        if self.active_setpoint_source != self.hover_setpoint.source:
            self.get_logger().info("Using hover fallback setpoint")
        self.active_setpoint_source = self.hover_setpoint.source
        return self.hover_setpoint

    def local_position_ready(self) -> bool:
        return (
            self.local_position_state["xy_valid"]
            and self.local_position_state["z_valid"]
            and self.local_position_state["v_xy_valid"]
            and self.local_position_state["v_z_valid"]
            and self.local_position_state["heading_good_for_control"]
        )

    def estimator_ready(self) -> bool:
        if not self.use_estimator_flags_gate:
            return True

        if self.estimator_flags is None:
            return False

        return (
            self.estimator_flags["tilt"]
            and self.estimator_flags["yaw"]
            and self.estimator_flags["ev_pos"]
            and (not self.require_ev_vel or self.estimator_flags["ev_vel"])
            and (self.estimator_flags["ev_hgt"] or self.local_position_state["z_valid"])
            and (self.estimator_flags["ev_yaw"] or self.local_position_state["heading_good_for_control"])
        )

    def ensure_offboard_and_arm(self) -> None:
        now_us = self.timestamp_us()
        if not self.vehicle_status_received or not self.vehicle_control_mode_received:
            if (
                now_us - self.start_time_us > 5_000_000
                and now_us - self.last_waiting_for_status_log_us > 2_000_000
            ):
                self.get_logger().warn(
                    "Still waiting for PX4 /fmu/out status topics over ROS 2. "
                    "Offboard arming is paused until DDS status is visible."
                )
                self.last_waiting_for_status_log_us = now_us
            return

        if not self.local_position_ready():
            if now_us - self.last_waiting_for_status_log_us > 2_000_000:
                self.get_logger().warn(
                    "Waiting for PX4 local position to become valid before Offboard arming. "
                    "Need xy/z/velocity validity plus heading_good_for_control."
                )
                self.last_waiting_for_status_log_us = now_us
            return

        if not self.estimator_ready():
            if now_us - self.last_waiting_for_status_log_us > 2_000_000:
                self.get_logger().warn(
                    "Waiting for PX4 estimator readiness before Offboard arming."
                )
                self.last_waiting_for_status_log_us = now_us
            return

        if now_us - self.last_request_time_us < int(self.mode_request_interval * 1e6):
            return

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or not self.offboard_enabled:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )
            self.last_request_time_us = now_us

        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self.last_request_time_us = now_us
            if not self.arm_request_logged:
                self.get_logger().info("Requesting Offboard mode and arm")
                self.arm_request_logged = True

            if not self.pre_flight_checks_pass and not self.waiting_for_health_logged:
                self.get_logger().warn("PX4 preflight checks not ready yet, waiting before arming")
                self.waiting_for_health_logged = True
        else:
            self.waiting_for_health_logged = False

    def maybe_log_debug_state(self, active_setpoint: ManagedSetpoint) -> None:
        now_us = self.timestamp_us()
        if now_us - self.last_debug_log_us < 2_000_000:
            return

        self.last_debug_log_us = now_us
        estimator_text = "estimator_gate=disabled"
        if self.use_estimator_flags_gate:
            estimator_text = "estimator_flags=waiting"
            if self.estimator_flags is not None:
                estimator_text = (
                    "estimator_flags="
                    f"tilt={int(self.estimator_flags['tilt'])},"
                    f"yaw={int(self.estimator_flags['yaw'])},"
                    f"ev_pos={int(self.estimator_flags['ev_pos'])},"
                    f"ev_vel={int(self.estimator_flags['ev_vel'])},"
                    f"ev_yaw={int(self.estimator_flags['ev_yaw'])},"
                    f"ev_hgt={int(self.estimator_flags['ev_hgt'])},"
                    f"rest={int(self.estimator_flags['vehicle_at_rest'])}"
                )

        local_position_text = (
            "lpos="
            f"xy={int(self.local_position_state['xy_valid'])},"
            f"z={int(self.local_position_state['z_valid'])},"
            f"vxy={int(self.local_position_state['v_xy_valid'])},"
            f"vz={int(self.local_position_state['v_z_valid'])},"
            f"heading={int(self.local_position_state['heading_good_for_control'])}"
        )
        status_text = (
            "status="
            f"rx_status={int(self.vehicle_status_received)},"
            f"rx_ctrl_mode={int(self.vehicle_control_mode_received)},"
            f"rx_timesync={int(self.timesync_received)},"
            f"preflight={int(self.pre_flight_checks_pass)},"
            f"offboard_accept={int(self.accepts_offboard_setpoints)},"
            f"offboard_enabled={int(self.offboard_enabled)},"
            f"nav={self.nav_state},"
            f"arm={self.arming_state},"
            f"lpos_ready={int(self.local_position_ready())},"
            f"est_ready={int(self.estimator_ready())},"
            f"vehicle_odom={int(self.vehicle_odometry_received)}"
        )
        active_setpoint_text = (
            "setpoint="
            f"source={active_setpoint.source},"
            f"mode={active_setpoint.mode_name()},"
            f"pos={[round(value, 3) if math.isfinite(value) else 'nan' for value in active_setpoint.position]},"
            f"vel={[round(value, 3) if math.isfinite(value) else 'nan' for value in active_setpoint.velocity]},"
            f"yaw={round(active_setpoint.yaw, 3) if math.isfinite(active_setpoint.yaw) else 'nan'}"
        )
        sync_text = f"timesync=offset_us={self.timesync_offset_us},rtt_us={self.timesync_rtt_us}"
        self.get_logger().info(
            f"{status_text} {local_position_text} {estimator_text} {active_setpoint_text} {sync_text}"
        )

    def publish_offboard_control_mode(self, active_setpoint: ManagedSetpoint) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        active_setpoint.fill_offboard_control_mode(msg)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, active_setpoint: ManagedSetpoint) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        msg.position = list(active_setpoint.position)
        msg.velocity = list(active_setpoint.velocity)
        msg.acceleration = list(active_setpoint.acceleration)
        msg.yaw = active_setpoint.yaw
        msg.yawspeed = active_setpoint.yawspeed
        self.trajectory_setpoint_pub.publish(msg)

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
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = self.target_system if target_system is None else int(target_system)
        msg.target_component = self.target_component if target_component is None else int(target_component)
        msg.source_system = int(source_system)
        msg.source_component = int(source_component)
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _copy_xyz(self, values: list[float]) -> list[float]:
        copied = [math.nan, math.nan, math.nan]
        for index, value in enumerate(list(values)[:3]):
            copied[index] = self._copy_scalar(value)
        return copied

    def _copy_scalar(self, value: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return math.nan


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = OffboardControl()
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


if __name__ == "__main__":
    main()
