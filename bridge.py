"""Minimal but structured Python bridge between PX4 SITL and MuJoCo.

This module intentionally mirrors the high-level dataflow of the C++ MuJoCo
bridge inside PX4:

1. Read MuJoCo state and sensors.
2. Convert FLU/world-up quantities into the PX4-facing FRD/NED convention.
3. Send MAVLink HIL_SENSOR, HIL_GPS and ODOMETRY messages to PX4.
4. Receive HIL_ACTUATOR_CONTROLS from PX4.
5. Write actuator commands back into MuJoCo and step physics forward.

Compared with the C++ bridge, this file trades UI completeness for easier
iteration and hackability in Python. The recent refactors therefore prioritize:

- explicit type sanitation before MAVLink packing;
- clearer control flow around paused/running simulation states;
- small helper functions with single, readable responsibilities.
"""

from __future__ import annotations

import argparse
import math
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np

from frames import flu_to_frd, mujoco_quat_to_px4_frd_to_ned, mujoco_world_to_ned, quat_wxyz_to_rotmat

try:
    import mujoco.viewer
except Exception:
    mujoco.viewer = None

try:
    import glfw
except Exception:
    glfw = None

try:
    import rclpy
    from rclpy.node import Node as RosNode
    from rclpy.qos import DurabilityPolicy as RosDurabilityPolicy
    from rclpy.qos import HistoryPolicy as RosHistoryPolicy
    from rclpy.qos import QoSProfile as RosQoSProfile
    from rclpy.qos import ReliabilityPolicy as RosReliabilityPolicy
    from px4_msgs.msg import VehicleOdometry as RosVehicleOdometry
except Exception:
    rclpy = None
    RosNode = None
    RosDurabilityPolicy = None
    RosHistoryPolicy = None
    RosQoSProfile = None
    RosReliabilityPolicy = None
    RosVehicleOdometry = None

try:
    os.environ.setdefault("MAVLINK20", "1")
    from pymavlink import mavutil
    mavutil.set_dialect("common")
except Exception:
    mavutil = None


REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL = Path(
    os.environ.get(
        "PX4_MUJOCO_MODEL",
        str(REPO_ROOT / "UAV" / "scene_uav_delta.xml"),
    )
).expanduser()
ALL_HIL_SENSOR_FIELDS = 8191
HOME_LAT_DEG = 47.6061
HOME_LON_DEG = -122.3328
HOME_ALT_M = 488.0
LAT_DEG_PER_M = 1.0 / 111111.0
LON_DEG_PER_M = 1.0 / (111111.0 * math.cos(math.radians(HOME_LAT_DEG)))
FLOAT32_MAX = float(np.finfo(np.float32).max)
KEY_TOGGLE_PAUSE = getattr(glfw, "KEY_SPACE", 32) if glfw is not None else 32
KEY_STEP_ONCE = getattr(glfw, "KEY_PERIOD", 46) if glfw is not None else 46
REQUIRED_HIL_SENSORS = ("body_gyro", "body_linacc")
OPTIONAL_HIL_SENSORS = ("body_mag",)
FLIGHT_ACTUATOR_NAMES = ("motor_1", "motor_2", "motor_3", "motor_4")
FLIGHT_ACTUATOR_SITE_NAMES = {
    "motor_1": "motor_4_site",
    "motor_2": "motor_2_site",
    "motor_3": "motor_1_site",
    "motor_4": "motor_3_site",
}
ACCEL_NOISE_STDDEV = 0.03
GYRO_NOISE_STDDEV = 0.002
MAG_NOISE_STDDEV = 0.0005
BARO_PRESSURE_NOISE_STDDEV = 0.03
BARO_ALT_NOISE_STDDEV = 0.03
BARO_TEMP_NOISE_STDDEV = 0.02
GRAVITY_WORLD = np.array([0.0, 0.0, -9.81], dtype=float)
PRESETTLE_DURATION_SECONDS = 1.5
PX4_ALIGNMENT_HOLD_SECONDS = 2.0


@dataclass
class BridgeConfig:
    """Runtime configuration for the Python bridge."""

    model: Path = DEFAULT_MODEL
    mavlink_host: str = "0.0.0.0"
    mavlink_port: int = 4560
    actuator_mavlink_port: Optional[int] = None
    no_mavlink: bool = False
    headless: bool = False
    steps: Optional[int] = None
    px4_hover_thrust: float = 0.60
    real_time_factor: float = 1.0
    physics_substeps_per_sensor: int = 1
    send_hil_gps: bool = False
    local_hover: bool = False
    local_hover_target_z: float = 2.0
    local_hover_ramp_seconds: float = 3.0
    publish_visual_odometry_ros2: bool = False
    publish_debug_truth_ros2: bool = False
    presettle_duration_seconds: float = PRESETTLE_DURATION_SECONDS
    px4_alignment_hold_seconds: float = PX4_ALIGNMENT_HOLD_SECONDS
    ready_file: Optional[Path] = None
    connected_file: Optional[Path] = None
    debug_controls: bool = False
    debug_hil_rate: bool = False


class MuJoCoSim:
    """Thin MuJoCo wrapper for viewer management, state extraction and controls."""

    def __init__(self, model_path: Path) -> None:
        if not model_path.exists():
            raise FileNotFoundError(f"MuJoCo 模型不存在: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.model_path = model_path
        self._default_ctrl = np.zeros(int(self.model.nu), dtype=float)
        if self.model.nkey > 0 and int(self.model.nu) > 0:
            self._default_ctrl[:] = np.array(self.model.key_ctrl[0, :], dtype=float)
        self._actuator_name_to_id = {}
        for actuator_id in range(int(self.model.nu)):
            actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            if actuator_name:
                self._actuator_name_to_id[actuator_name] = actuator_id
        self._flight_actuator_ids = [
            self._actuator_name_to_id[name]
            for name in FLIGHT_ACTUATOR_NAMES
            if name in self._actuator_name_to_id
        ]
        self._has_named_flight_actuators = len(self._flight_actuator_ids) == len(FLIGHT_ACTUATOR_NAMES)
        self._flight_actuator_wrench_matrix = self._build_flight_actuator_wrench_matrix()
        self._sensor_cache = {}
        self._viewer = None
        self._paused = False
        self._step_once_requested = False

    def launch_viewer(self) -> None:
        if mujoco.viewer is None:
            raise RuntimeError("mujoco.viewer 不可用")
        self._viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self._handle_key_event,
        )

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    def _handle_key_event(self, keycode: int) -> None:
        """Handle lightweight bridge-local keyboard controls.

        Space toggles paused/running state.
        Period steps the simulation forward by exactly one MuJoCo step while paused.
        """

        if keycode == KEY_TOGGLE_PAUSE:
            self._paused = not self._paused
            if not self._paused:
                self._step_once_requested = False
        elif keycode == KEY_STEP_ONCE:
            self._step_once_requested = True

    def _viewer_overlay(self) -> tuple[int, int, str, str] | None:
        """Build a short overlay describing bridge-local controls and state."""

        if self._viewer is None:
            return None

        status = "Paused" if self._paused else "Running"
        sim_time = f"{float(self.data.time):.3f}s"
        return (
            mujoco.mjtFontScale.mjFONTSCALE_150,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            "Python Bridge\nSpace: pause/resume\n.: single-step while paused",
            f"{status}\nSim time: {sim_time}",
        )

    def sync_viewer(self) -> None:
        if self._viewer is not None and self._viewer.is_running():
            overlay = self._viewer_overlay()
            if overlay is not None:
                self._viewer.set_texts(overlay)
            self._viewer.sync()

    def viewer_running(self) -> bool:
        return self._viewer is None or self._viewer.is_running()

    def should_step(self) -> bool:
        """Return whether one physics step should be executed this loop."""

        if not self._paused:
            return True

        if self._step_once_requested:
            self._step_once_requested = False
            return True

        return False

    def refresh_paused_state(self) -> None:
        """Recompute derived MuJoCo quantities while paused for stable rendering."""

        mujoco.mj_forward(self.model, self.data)

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def zero_ctrl(self) -> None:
        self.data.ctrl[:] = 0.0

    def _reset_ctrl_for_px4(self) -> None:
        if self.model.nkey > 0 and int(self.model.nu) > 0:
            self.data.ctrl[:] = self._default_ctrl
        else:
            self.zero_ctrl()

    def _actuator_site_id(self, actuator_name: str, actuator_id: int) -> int:
        site_name = FLIGHT_ACTUATOR_SITE_NAMES.get(actuator_name)
        if site_name:
            site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            if site_id >= 0:
                return site_id

        transmission_site_id = int(self.model.actuator_trnid[actuator_id, 0])
        if 0 <= transmission_site_id < int(self.model.nsite):
            return transmission_site_id

        return -1

    def _build_flight_actuator_wrench_matrix(self) -> np.ndarray:
        if not self._has_named_flight_actuators:
            return np.zeros((4, 0), dtype=float)

        matrix = np.zeros((4, len(self._flight_actuator_ids)), dtype=float)
        for column, actuator_id in enumerate(self._flight_actuator_ids):
            actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            if not actuator_name:
                continue

            site_id = self._actuator_site_id(actuator_name, actuator_id)
            if site_id < 0:
                continue

            position_body = np.array(self.model.site_pos[site_id], dtype=float)
            gear = np.array(self.model.actuator_gear[actuator_id], dtype=float)
            force_body = gear[:3]
            torque_body = np.cross(position_body, force_body) + gear[3:6]
            matrix[:, column] = np.array(
                [
                    force_body[2],
                    torque_body[0],
                    torque_body[1],
                    torque_body[2],
                ],
                dtype=float,
            )

        return matrix

    def flight_actuator_wrench_matrix(self) -> np.ndarray:
        return np.array(self._flight_actuator_wrench_matrix, dtype=float)

    def flight_actuator_ctrl_limits(self) -> tuple[np.ndarray, np.ndarray]:
        if not self._flight_actuator_ids:
            return np.zeros(0, dtype=float), np.zeros(0, dtype=float)

        ctrl_min = np.array(
            [self.model.actuator_ctrlrange[actuator_id, 0] for actuator_id in self._flight_actuator_ids],
            dtype=float,
        )
        ctrl_max = np.array(
            [self.model.actuator_ctrlrange[actuator_id, 1] for actuator_id in self._flight_actuator_ids],
            dtype=float,
        )
        return ctrl_min, ctrl_max

    def write_direct_flight_controls(self, controls: np.ndarray) -> None:
        if not self._has_named_flight_actuators:
            return

        self._reset_ctrl_for_px4()
        ctrl_values = sanitize_vector(controls, len(self._flight_actuator_ids))
        for control_value, actuator_id in zip(ctrl_values, self._flight_actuator_ids):
            ctrl_min = float(self.model.actuator_ctrlrange[actuator_id, 0])
            ctrl_max = float(self.model.actuator_ctrlrange[actuator_id, 1])
            self.data.ctrl[actuator_id] = float(np.clip(control_value, ctrl_min, ctrl_max))

    def write_controls(self, controls: np.ndarray, armed: bool, hover_thrust: float) -> None:
        actuator_count = int(self.model.nu)
        if actuator_count == 0:
            return

        if self._has_named_flight_actuators:
            target_actuator_ids = self._flight_actuator_ids
            self._reset_ctrl_for_px4()
        else:
            target_actuator_ids = list(range(min(actuator_count, controls.shape[0])))
            if not armed:
                self.zero_ctrl()
                return

        if not armed:
            for actuator_id in target_actuator_ids:
                self.data.ctrl[actuator_id] = 0.0
            return

        for control_index, actuator_id in enumerate(target_actuator_ids[: controls.shape[0]]):
            normalized = float(np.clip(controls[control_index], 0.0, 1.0))
            ctrl_min = float(self.model.actuator_ctrlrange[actuator_id, 0])
            ctrl_max = float(self.model.actuator_ctrlrange[actuator_id, 1])
            hover_ctrl = ctrl_max
            if self.model.nkey > 0:
                hover_ctrl = float(self.model.key_ctrl[0, actuator_id])
            hover_thrust_safe = max(hover_thrust, 1e-6)
            scaled = (normalized / hover_thrust_safe) * hover_ctrl
            self.data.ctrl[actuator_id] = float(np.clip(scaled, ctrl_min, ctrl_max))

    def has_sensor(self, name: str) -> bool:
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        return sensor_id >= 0

    def validate_px4_hil_model_contract(self) -> None:
        missing_required = [name for name in REQUIRED_HIL_SENSORS if not self.has_sensor(name)]
        if missing_required:
            missing_text = ", ".join(missing_required)
            raise RuntimeError(
                "当前 MuJoCo 模型不满足 PX4 HIL bridge 的最小契约，缺少必需传感器: "
                f"{missing_text}。"
                "请先在 XML 中补齐对应 sensor，或先使用 --no-mavlink 做本地 MuJoCo 调试。"
            )

        if int(self.model.nu) == 0:
            raise RuntimeError(
                "当前 MuJoCo 模型没有 actuator，PX4 无法把控制量写回仿真。"
            )

        if not self._has_named_flight_actuators and int(self.model.nu) != 4:
            raise RuntimeError(
                "当前 MuJoCo 模型没有完整的命名飞行 actuator "
                f"{', '.join(FLIGHT_ACTUATOR_NAMES)}，且 actuator 总数不等于 4，"
                "PX4 电机输出无法安全映射。"
            )

        if not self._has_named_flight_actuators and int(self.model.nu) == 4:
            print(
                "Warning: 当前模型未命名飞行 actuator，bridge 将回退到前 4 路 actuator 顺序映射。"
            )

        missing_optional = [name for name in OPTIONAL_HIL_SENSORS if not self.has_sensor(name)]
        if missing_optional:
            optional_text = ", ".join(missing_optional)
            print(
                "Warning: 当前模型缺少可选传感器 "
                f"{optional_text}，bridge 将回退为零磁场数据。"
            )

    def _sensor_slice(self, name: str) -> np.ndarray:
        """Return a cached slice view into MuJoCo sensordata by sensor name."""

        if name not in self._sensor_cache:
            sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sensor_id < 0:
                raise KeyError(f"未找到传感器: {name}")
            start = int(self.model.sensor_adr[sensor_id])
            dim = int(self.model.sensor_dim[sensor_id])
            self._sensor_cache[name] = slice(start, start + dim)
        return self.data.sensordata[self._sensor_cache[name]]

    def imu_frd(self) -> tuple[np.ndarray, np.ndarray]:
        gyro_flu = np.array(self._sensor_slice("body_gyro"), dtype=float)
        accel_world = np.array(self.data.qacc[0:3], dtype=float)
        rotation_world_from_body_flu = quat_wxyz_to_rotmat(
            sanitize_quaternion(np.array(self.data.qpos[3:7], dtype=float))
        )
        rotation_body_flu_from_world = rotation_world_from_body_flu.T
        # PX4 expects body-frame specific force, not gravity-free linear acceleration.
        specific_force_body_flu = rotation_body_flu_from_world @ (accel_world - GRAVITY_WORLD)
        return flu_to_frd(gyro_flu), flu_to_frd(specific_force_body_flu)

    def mag_frd(self) -> np.ndarray:
        try:
            return flu_to_frd(np.array(self._sensor_slice("body_mag"), dtype=float))
        except KeyError:
            return np.zeros(3, dtype=float)

    def state_ned(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        position_ned = mujoco_world_to_ned(np.array(self.data.qpos[0:3], dtype=float))
        velocity_ned = mujoco_world_to_ned(np.array(self.data.qvel[0:3], dtype=float))
        quat_frd_to_ned = mujoco_quat_to_px4_frd_to_ned(np.array(self.data.qpos[3:7], dtype=float))
        angular_velocity_frd = flu_to_frd(np.array(self.data.qvel[3:6], dtype=float))
        return position_ned, velocity_ned, quat_frd_to_ned, angular_velocity_frd

    def state_world(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        position_world = np.array(self.data.qpos[0:3], dtype=float)
        velocity_world = np.array(self.data.qvel[0:3], dtype=float)
        quat_world_from_body = sanitize_quaternion(np.array(self.data.qpos[3:7], dtype=float))
        angular_velocity_body = np.array(self.data.qvel[3:6], dtype=float)
        return position_world, velocity_world, quat_world_from_body, angular_velocity_body


class LocalHoverController:
    """Simple onboard-free MuJoCo hover controller for visual takeoff demos."""

    def __init__(self, sim: MuJoCoSim, target_z: float, ramp_seconds: float) -> None:
        self._target_xy = np.array(sim.data.qpos[0:2], dtype=float)
        self._initial_z = float(sim.data.qpos[2])
        self._target_z = max(float(target_z), self._initial_z + 0.2)
        self._ramp_seconds = max(float(ramp_seconds), 0.5)
        self._mass = float(np.sum(sim.model.body_mass[1:]))
        self._gravity = float(np.linalg.norm(np.array(sim.model.opt.gravity, dtype=float)))
        self._wrench_matrix = sim.flight_actuator_wrench_matrix()
        self._ctrl_min, self._ctrl_max = sim.flight_actuator_ctrl_limits()

        if self._wrench_matrix.shape != (4, 4):
            raise RuntimeError("本地悬停模式要求 4 个已命名飞行电机。")

        if self._ctrl_min.shape[0] != 4 or self._ctrl_max.shape[0] != 4:
            raise RuntimeError("本地悬停模式未能读取电机控制范围。")

    def _target_position(self, sim_time: float) -> np.ndarray:
        alpha = float(np.clip(sim_time / self._ramp_seconds, 0.0, 1.0))
        z = self._initial_z + alpha * (self._target_z - self._initial_z)
        return np.array([self._target_xy[0], self._target_xy[1], z], dtype=float)

    def compute_controls(
        self,
        position_world: np.ndarray,
        velocity_world: np.ndarray,
        quat_world_from_body: np.ndarray,
        angular_velocity_body: np.ndarray,
        sim_time: float,
    ) -> np.ndarray:
        target_position = self._target_position(sim_time)
        position_error = target_position - sanitize_vector(position_world, 3)
        velocity_error = -sanitize_vector(velocity_world, 3)

        kp_pos = np.array([1.4, 1.4, 2.8], dtype=float)
        kd_vel = np.array([1.6, 1.6, 2.0], dtype=float)
        desired_accel_world = kp_pos * position_error + kd_vel * velocity_error
        desired_accel_world[0:2] = np.clip(desired_accel_world[0:2], -2.0, 2.0)
        desired_accel_world[2] = float(np.clip(desired_accel_world[2], -2.0, 4.0))
        desired_force_world = self._mass * (desired_accel_world + np.array([0.0, 0.0, self._gravity], dtype=float))

        force_norm = float(np.linalg.norm(desired_force_world))
        if force_norm < 1e-6:
            desired_force_world = np.array([0.0, 0.0, self._mass * self._gravity], dtype=float)
            force_norm = float(np.linalg.norm(desired_force_world))

        desired_body_z_world = desired_force_world / force_norm
        desired_yaw = 0.0
        desired_heading_world = np.array([math.cos(desired_yaw), math.sin(desired_yaw), 0.0], dtype=float)
        desired_body_y_world = np.cross(desired_body_z_world, desired_heading_world)
        if np.linalg.norm(desired_body_y_world) < 1e-6:
            desired_body_y_world = np.array([0.0, 1.0, 0.0], dtype=float)
        desired_body_y_world /= np.linalg.norm(desired_body_y_world)
        desired_body_x_world = np.cross(desired_body_y_world, desired_body_z_world)
        desired_body_x_world /= max(float(np.linalg.norm(desired_body_x_world)), 1e-6)
        desired_rotation = np.column_stack((desired_body_x_world, desired_body_y_world, desired_body_z_world))

        current_rotation = quat_wxyz_to_rotmat(sanitize_quaternion(quat_world_from_body))
        rotation_error_matrix = 0.5 * (desired_rotation.T @ current_rotation - current_rotation.T @ desired_rotation)
        rotation_error = np.array(
            [
                rotation_error_matrix[2, 1],
                rotation_error_matrix[0, 2],
                rotation_error_matrix[1, 0],
            ],
            dtype=float,
        )
        angular_velocity = sanitize_vector(angular_velocity_body, 3)
        k_rot = np.array([7.0, 7.0, 2.5], dtype=float)
        k_rate = np.array([2.4, 2.4, 0.9], dtype=float)
        desired_torque_body = -k_rot * rotation_error - k_rate * angular_velocity

        total_thrust = float(np.dot(desired_force_world, current_rotation[:, 2]))
        total_thrust = max(total_thrust, 0.0)
        desired_wrench = np.array(
            [
                total_thrust,
                desired_torque_body[0],
                desired_torque_body[1],
                desired_torque_body[2],
            ],
            dtype=float,
        )

        controls, *_ = np.linalg.lstsq(self._wrench_matrix, desired_wrench, rcond=None)
        controls = np.clip(controls, self._ctrl_min, self._ctrl_max)
        return controls


class Px4MavlinkIo:
    """Encapsulates the MAVLink-side IO contract with PX4 SITL."""

    def __init__(self, host: str, port: int, actuator_port: Optional[int], enabled: bool) -> None:
        self.enabled = enabled and mavutil is not None
        self.host = host
        self.port = port
        self.actuator_port = actuator_port
        self.master = None
        self.actuator_master = None
        self.armed = False
        self._last_heartbeat = 0.0
        self._received_first_actuator = False
        self._was_connected = False
        self.last_actuator_controls: Optional[np.ndarray] = None
        self._pending_actuator_controls: Optional[np.ndarray] = None
        self._reader_lock = threading.Lock()
        self._reader_stop = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None

    def connect(self) -> None:
        if not self.enabled:
            return
        endpoint = f"tcpin:{self.host}:{self.port}"
        self.master = mavutil.mavlink_connection(endpoint, source_system=1, source_component=200)
        print(f"Listening for PX4 on {endpoint}")

        if self.actuator_port is not None:
            actuator_endpoint = f"udpin:0.0.0.0:{self.actuator_port}"
            self.actuator_master = mavutil.mavlink_connection(
                actuator_endpoint,
                source_system=1,
                source_component=201,
            )
            print(f"Listening for PX4 actuator MAVLink on {actuator_endpoint}")

        self._start_reader_thread()

    def close(self) -> None:
        self._reader_stop.set()
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)

    def _start_reader_thread(self) -> None:
        if self.master is None or self._reader_thread is not None:
            return

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="px4-mavlink-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def _handle_timesync_message(self, reader, msg) -> None:
        if reader is None or not hasattr(reader, "mav"):
            return

        tc1 = int(getattr(msg, "tc1", 0))
        ts1 = int(getattr(msg, "ts1", 0))

        if tc1 != 0:
            return

        try:
            reader.mav.timesync_send(time.monotonic_ns(), ts1)
        except Exception:
            return

    def _handle_incoming_message(self, reader, msg) -> None:
        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT":
            self._last_heartbeat = time.time()
            return

        if msg_type == "TIMESYNC":
            self._handle_timesync_message(reader, msg)
            return

        if msg_type == "HIL_ACTUATOR_CONTROLS":
            controls = np.array(msg.controls, dtype=float)
            armed = bool(msg.mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            with self._reader_lock:
                self.armed = armed
                self._received_first_actuator = True
                self.last_actuator_controls = controls
                self._pending_actuator_controls = controls

    def _reader_loop(self) -> None:
        while not self._reader_stop.is_set():
            readers = [self.master]
            if self.actuator_master is not None and self.actuator_master is not self.master:
                readers.append(self.actuator_master)

            saw_message = False

            for reader in readers:
                if reader is None:
                    continue

                try:
                    msg = reader.recv_match(blocking=False)
                except Exception:
                    continue

                if msg is None:
                    continue

                saw_message = True
                self._handle_incoming_message(reader, msg)

            if not saw_message:
                time.sleep(0.002)

    def connected(self) -> bool:
        return bool(self.enabled and self.master is not None and getattr(self.master, "port", None) is not None)

    def service_connection(self) -> None:
        """Drive pymavlink's lazy tcpin accept path even before sensor IO starts."""

        if not self.enabled or self.master is None:
            return

        if getattr(self.master, "port", None) is not None:
            return

        try:
            # mavtcpin only accepts an inbound TCP connection when recv() is touched.
            # Poll it explicitly so the bridge can acknowledge PX4 quickly even if the
            # simulation is paused or has not yet reached the actuator polling phase.
            self.master.recv(1)
        except Exception:
            return

    def connection_state_changed(self) -> Optional[bool]:
        connected = self.connected()
        if connected == self._was_connected:
            return None
        self._was_connected = connected
        return connected

    def poll_actuator_controls(self, wait: bool) -> Optional[np.ndarray]:
        if not self.enabled or self.master is None:
            return None

        deadline = time.monotonic() + 0.02 if wait else time.monotonic()

        while True:
            with self._reader_lock:
                if self._pending_actuator_controls is not None:
                    controls = np.array(self._pending_actuator_controls, dtype=float)
                    self._pending_actuator_controls = None
                    return controls

            if not wait or time.monotonic() >= deadline:
                return None

            time.sleep(0.001)

    def send_heartbeat(self) -> None:
        if not self.enabled or self.master is None:
            return
        now = time.time()
        if now - self._last_heartbeat < 1.0:
            return
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        self._last_heartbeat = now

    def send_hil_sensor(
        self,
        timestamp_us: int,
        accel_frd: np.ndarray,
        gyro_frd: np.ndarray,
        mag_frd: np.ndarray,
        pressure_hpa: float,
        pressure_alt_m: float,
        temperature_c: float = 20.0,
    ) -> None:
        if not self.enabled or self.master is None:
            return
        accel = sanitize_vector(accel_frd, 3)
        gyro = sanitize_vector(gyro_frd, 3)
        mag = sanitize_vector(mag_frd, 3)
        self.master.mav.hil_sensor_send(
            clamp_uint64(timestamp_us),
            sanitize_float(accel[0]),
            sanitize_float(accel[1]),
            sanitize_float(accel[2]),
            sanitize_float(gyro[0]),
            sanitize_float(gyro[1]),
            sanitize_float(gyro[2]),
            sanitize_float(mag[0]),
            sanitize_float(mag[1]),
            sanitize_float(mag[2]),
            sanitize_float(pressure_hpa, min_value=0.0),
            sanitize_float(0.0, min_value=0.0),
            sanitize_float(pressure_alt_m),
            sanitize_float(temperature_c),
            clamp_uint32(ALL_HIL_SENSOR_FIELDS),
            clamp_uint8(0),
        )

    def send_hil_gps(self, timestamp_us: int, position_ned: np.ndarray, velocity_ned: np.ndarray) -> None:
        if not self.enabled or self.master is None:
            return
        position = sanitize_vector(position_ned, 3)
        velocity = sanitize_vector(velocity_ned, 3)
        lat = clamp_int32((HOME_LAT_DEG + position[0] * LAT_DEG_PER_M) * 1e7)
        lon = clamp_int32((HOME_LON_DEG + position[1] * LON_DEG_PER_M) * 1e7)
        alt_mm = clamp_int32((HOME_ALT_M - position[2]) * 1000.0)
        vel_cm_s = sanitize_float(np.linalg.norm(velocity) * 100.0, min_value=0.0)
        vn_cm_s = sanitize_float(velocity[0] * 100.0)
        ve_cm_s = sanitize_float(velocity[1] * 100.0)
        vd_cm_s = sanitize_float(velocity[2] * 100.0)

        cog = 0
        if vel_cm_s > 1.0:
            course_deg = (math.degrees(math.atan2(velocity[1], velocity[0])) + 360.0) % 360.0
            cog = clamp_uint16(course_deg * 100.0)

        self.master.mav.hil_gps_send(
            clamp_uint64(timestamp_us),
            clamp_uint8(3),
            lat,
            lon,
            alt_mm,
            clamp_uint16(100.0),
            clamp_uint16(100.0),
            clamp_uint16(vel_cm_s),
            clamp_int16(vn_cm_s),
            clamp_int16(ve_cm_s),
            clamp_int16(vd_cm_s),
            cog,
            clamp_uint8(10),
        )

    def send_visual_odometry(
        self,
        timestamp_us: int,
        position_ned: np.ndarray,
        velocity_ned: np.ndarray,
        quat_frd_to_ned: np.ndarray,
        angular_velocity_frd: np.ndarray,
    ) -> None:
        if not self.enabled or self.master is None:
            return
        position = sanitize_vector(position_ned, 3)
        velocity_world = sanitize_vector(velocity_ned, 3)
        orientation = sanitize_quaternion(quat_frd_to_ned)
        angular_velocity = sanitize_vector(angular_velocity_frd, 3)
        pose_cov = sanitize_covariance([1e-4] * 21)
        vel_cov = sanitize_covariance([1e-4] * 21)
        self.master.mav.odometry_send(
            clamp_uint64(timestamp_us),
            clamp_uint8(mavutil.mavlink.MAV_FRAME_LOCAL_NED),
            clamp_uint8(mavutil.mavlink.MAV_FRAME_LOCAL_NED),
            sanitize_float(position[0]),
            sanitize_float(position[1]),
            sanitize_float(position[2]),
            orientation.tolist(),
            sanitize_float(velocity_world[0]),
            sanitize_float(velocity_world[1]),
            sanitize_float(velocity_world[2]),
            sanitize_float(angular_velocity[0]),
            sanitize_float(angular_velocity[1]),
            sanitize_float(angular_velocity[2]),
            pose_cov,
            vel_cov,
            clamp_uint8(0),
            clamp_uint8(mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION),
            clamp_uint8(100),
        )


class RosVehicleOdometryPublisher:
    """Publish NED odometry over ROS 2 using px4_msgs/VehicleOdometry."""

    def __init__(self, topic_name: str, node_name: str) -> None:
        if rclpy is None or RosNode is None or RosVehicleOdometry is None:
            raise RuntimeError(
                "ROS 2 visual odometry mode requested, but rclpy/px4_msgs is unavailable. "
                "Please source ROS 2 before running the bridge."
            )

        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = RosNode(node_name)
        qos = RosQoSProfile(
            reliability=RosReliabilityPolicy.BEST_EFFORT,
            durability=RosDurabilityPolicy.VOLATILE,
            history=RosHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = self._node.create_publisher(
            RosVehicleOdometry,
            topic_name,
            qos,
        )
        self._closed = False

    def publish(
        self,
        timestamp_us: int,
        position_ned: np.ndarray,
        velocity_ned: np.ndarray,
        quat_frd_to_ned: np.ndarray,
        angular_velocity_frd: np.ndarray,
    ) -> None:
        if self._closed or self._node is None or self._publisher is None:
            return
        if rclpy is None or not rclpy.ok():
            return

        msg = RosVehicleOdometry()
        msg.timestamp = clamp_uint64(timestamp_us)
        msg.timestamp_sample = clamp_uint64(timestamp_us)
        msg.pose_frame = RosVehicleOdometry.POSE_FRAME_NED
        msg.position = sanitize_vector(position_ned, 3).tolist()
        msg.q = sanitize_quaternion(quat_frd_to_ned).tolist()
        msg.velocity_frame = RosVehicleOdometry.VELOCITY_FRAME_NED
        msg.velocity = sanitize_vector(velocity_ned, 3).tolist()
        msg.angular_velocity = sanitize_vector(angular_velocity_frd, 3).tolist()
        msg.position_variance = [1e-4, 1e-4, 1e-4]
        msg.orientation_variance = [1e-4, 1e-4, 1e-4]
        msg.velocity_variance = [1e-4, 1e-4, 1e-4]
        msg.quality = 100
        try:
            self._publisher.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.0)
        except Exception:
            self._closed = True

    def close(self) -> None:
        if rclpy is None or self._node is None:
            return

        self._closed = True
        self._node.destroy_node()
        self._node = None
        self._publisher = None


def rotate_vector_by_quat_conjugate(quat_wxyz: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate a vector by the quaternion conjugate.

    The bridge uses this to convert world-frame velocity into body-frame velocity
    before emitting MAVLink ODOMETRY.
    """

    w, x, y, z = quat_wxyz
    q = np.array([w, -x, -y, -z], dtype=float)
    vx, vy, vz = vector
    v_quat = np.array([0.0, vx, vy, vz], dtype=float)
    return quat_multiply(quat_multiply(q, v_quat), quat_inverse(q))[1:]


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Quaternion multiplication in wxyz convention."""

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_inverse(quat_wxyz: np.ndarray) -> np.ndarray:
    """Return a numerically safe quaternion inverse in wxyz convention."""

    quat = np.array(quat_wxyz, dtype=float)
    norm_sq = float(np.dot(quat, quat))
    if norm_sq < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    quat[1:] *= -1.0
    return quat / norm_sq


def pressure_hpa_from_altitude(altitude_asl_m: float) -> float:
    """Approximate barometric pressure from altitude above sea level."""

    sea_level_pressure_hpa = 1013.25
    sea_level_temp_k = 288.15
    lapse_rate_k_per_m = 0.0065
    exponent = 5.2558797
    ratio = max(0.0, 1.0 - lapse_rate_k_per_m * altitude_asl_m / sea_level_temp_k)
    return sea_level_pressure_hpa * math.pow(ratio, exponent)


def clamp_uint16(value: float) -> int:
    """Clamp a numeric value into MAVLink uint16 range."""

    return int(np.clip(round(value), 0, 65535))


def clamp_int16(value: float) -> int:
    """Clamp a numeric value into MAVLink int16 range."""

    return int(np.clip(round(value), -32768, 32767))


def clamp_uint8(value: float) -> int:
    """Clamp a numeric value into MAVLink uint8 range."""

    return int(np.clip(round(value), 0, 255))


def clamp_uint32(value: float) -> int:
    """Clamp a numeric value into MAVLink uint32 range."""

    return int(np.clip(round(value), 0, 4294967295))


def clamp_uint64(value: float) -> int:
    """Clamp a numeric value into MAVLink uint64 range."""

    return int(np.clip(round(value), 0, 18446744073709551615))


def clamp_int32(value: float) -> int:
    """Clamp a numeric value into MAVLink int32 range."""

    return int(np.clip(round(value), -2147483648, 2147483647))


def sanitize_float(value: float, default: float = 0.0, min_value: float | None = None, max_value: float | None = None) -> float:
    """Convert to a finite float32-safe scalar with optional lower/upper bounds."""

    try:
        scalar = float(value)
    except (TypeError, ValueError):
        scalar = default

    if not np.isfinite(scalar):
        scalar = default

    if min_value is not None:
        scalar = max(scalar, min_value)

    if max_value is not None:
        scalar = min(scalar, max_value)

    scalar = float(np.clip(scalar, -FLOAT32_MAX, FLOAT32_MAX))
    return scalar


def sanitize_vector(values: np.ndarray | list[float], size: int, default: float = 0.0) -> np.ndarray:
    """Normalize arbitrary input into a fixed-size finite float vector."""

    array = np.asarray(values, dtype=float).reshape(-1)
    result = np.full(size, default, dtype=float)
    copy_count = min(size, array.shape[0])
    if copy_count > 0:
        result[:copy_count] = array[:copy_count]
    return np.array([sanitize_float(v, default=default) for v in result], dtype=float)


def sanitize_quaternion(quat_wxyz: np.ndarray | list[float]) -> np.ndarray:
    """Return a normalized quaternion or identity when the input is degenerate."""

    quat = sanitize_vector(quat_wxyz, 4, default=0.0)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return quat / norm


def sanitize_covariance(values: np.ndarray | list[float], default: float = 1e-4) -> list[float]:
    """Return a 21-element non-negative covariance vector."""

    covariance = sanitize_vector(values, 21, default=default)
    return [sanitize_float(v, default=default, min_value=0.0) for v in covariance]


def add_sensor_noise(rng: np.random.Generator, values: np.ndarray, stddev: float) -> np.ndarray:
    """Add small deterministic simulation noise so PX4 does not flag perfectly static sensors as stale."""

    if stddev <= 0.0:
        return np.array(values, dtype=float)

    return np.array(values, dtype=float) + rng.normal(0.0, stddev, size=np.shape(values))


def pace_realtime(start_wall_time: float, start_sim_time: float, current_sim_time: float, real_time_factor: float) -> None:
    """Throttle simulation so MuJoCo time advances at a chosen real-time factor."""

    if real_time_factor <= 0.0:
        return

    target_wall_time = start_wall_time + (current_sim_time - start_sim_time) / real_time_factor
    sleep_duration = target_wall_time - time.monotonic()
    if sleep_duration > 0.0:
        time.sleep(sleep_duration)


def presettle_simulation(sim: MuJoCoSim, duration_seconds: float) -> None:
    """Let the free-body model settle onto contact surfaces before PX4 starts sampling it."""

    if duration_seconds <= 0.0:
        return

    target_time = float(sim.data.time) + duration_seconds
    while float(sim.data.time) < target_time:
        sim.zero_ctrl()
        sim.step()

    # Remove residual drop energy so PX4 sees a quiet vehicle when it starts
    # its first IMU/EV alignment pass.
    sim.data.qvel[:] = 0.0
    mujoco.mj_forward(sim.model, sim.data)


def run(config: BridgeConfig) -> None:
    """Run the Python PX4-MuJoCo bridge until the viewer closes or steps are exhausted."""

    sim = MuJoCoSim(config.model)
    if not config.headless:
        sim.launch_viewer()

    io = Px4MavlinkIo(
        config.mavlink_host,
        config.mavlink_port,
        config.actuator_mavlink_port,
        not config.no_mavlink,
    )
    if io.enabled:
        sim.validate_px4_hil_model_contract()
    mujoco.mj_forward(sim.model, sim.data)
    presettle_simulation(sim, config.presettle_duration_seconds)
    settled_qpos = np.array(sim.data.qpos, dtype=float)
    io.connect()
    local_hover_controller = None
    visual_odometry_ros_publisher = None
    debug_truth_ros_publisher = None
    if config.local_hover:
        local_hover_controller = LocalHoverController(
            sim,
            target_z=config.local_hover_target_z,
            ramp_seconds=config.local_hover_ramp_seconds,
        )
        print(
            "Local hover enabled: "
            f"take off from z={float(sim.data.qpos[2]):.2f} m to z={config.local_hover_target_z:.2f} m",
            flush=True,
        )
    if config.publish_visual_odometry_ros2:
        visual_odometry_ros_publisher = RosVehicleOdometryPublisher(
            "/fmu/in/vehicle_visual_odometry",
            "mujoco_visual_odometry_bridge",
        )
        print("ROS 2 visual odometry publishing enabled on /fmu/in/vehicle_visual_odometry", flush=True)
    if config.publish_debug_truth_ros2:
        debug_truth_ros_publisher = RosVehicleOdometryPublisher(
            "/px4_mujoco/debug_truth_odometry",
            "mujoco_debug_truth_bridge",
        )
        print("ROS 2 debug truth publishing enabled on /px4_mujoco/debug_truth_odometry", flush=True)
    if config.ready_file is not None:
        config.ready_file.parent.mkdir(parents=True, exist_ok=True)
        config.ready_file.write_text("ready\n", encoding="utf-8")
    if config.connected_file is not None:
        config.connected_file.parent.mkdir(parents=True, exist_ok=True)
        try:
            config.connected_file.unlink()
        except FileNotFoundError:
            pass
    start_wall_time = time.monotonic()
    start_sim_time = float(sim.data.time)
    rng = np.random.default_rng(7)
    wall_time_origin_us = time.monotonic_ns() // 1000
    last_debug_controls_log_time = -1.0
    px4_alignment_hold_started_at: float | None = None

    step_count = 0
    hil_rate_window_started_at = time.monotonic()
    hil_rate_last_sim_time = float(sim.data.time)
    hil_rate_steps = 0
    hil_rate_sensor_messages = 0
    hil_rate_actuator_messages = 0
    try:
        while sim.viewer_running():
            if io.enabled:
                io.service_connection()
                connection_changed = io.connection_state_changed()
                if connection_changed is True:
                    print(f"PX4 connected on tcp:{config.mavlink_host}:{config.mavlink_port}", flush=True)
                    px4_alignment_hold_started_at = time.monotonic()
                    if config.connected_file is not None:
                        config.connected_file.write_text("connected\n", encoding="utf-8")
                elif connection_changed is False:
                    print(f"PX4 disconnected from tcp:{config.mavlink_host}:{config.mavlink_port}", flush=True)
                    px4_alignment_hold_started_at = None
                    if config.connected_file is not None:
                        try:
                            config.connected_file.unlink()
                        except FileNotFoundError:
                            pass

            if not sim.should_step():
                sim.refresh_paused_state()
                sim.sync_viewer()
                time.sleep(0.01)
                continue

            in_px4_alignment_hold = (
                px4_alignment_hold_started_at is not None
                and (time.monotonic() - px4_alignment_hold_started_at) < config.px4_alignment_hold_seconds
            )

            if in_px4_alignment_hold:
                sim.data.qpos[:] = settled_qpos
                sim.data.qvel[:] = 0.0
                sim.zero_ctrl()
                mujoco.mj_forward(sim.model, sim.data)

            elapsed_wall_time = time.monotonic() - start_wall_time
            timestamp_us = int(wall_time_origin_us + elapsed_wall_time * 1e6)
            position_world, velocity_world, quat_world_from_body, angular_velocity_body = sim.state_world()
            position_ned, velocity_ned, quat_frd_to_ned, angular_velocity_frd = sim.state_ned()
            if io.enabled:
                gyro_frd, accel_frd = sim.imu_frd()
                gyro_frd = add_sensor_noise(rng, gyro_frd, GYRO_NOISE_STDDEV)
                accel_frd = add_sensor_noise(rng, accel_frd, ACCEL_NOISE_STDDEV)
                mag_frd = add_sensor_noise(rng, sim.mag_frd(), MAG_NOISE_STDDEV)
                pressure_hpa = pressure_hpa_from_altitude(HOME_ALT_M - position_ned[2])
                pressure_hpa = sanitize_float(pressure_hpa + float(rng.normal(0.0, BARO_PRESSURE_NOISE_STDDEV)), min_value=0.0)
                pressure_alt_m = float(HOME_ALT_M - position_ned[2] + rng.normal(0.0, BARO_ALT_NOISE_STDDEV))
                temperature_c = float(20.0 + rng.normal(0.0, BARO_TEMP_NOISE_STDDEV))

                io.send_heartbeat()
                io.send_hil_sensor(timestamp_us, accel_frd, gyro_frd, mag_frd, pressure_hpa, pressure_alt_m, temperature_c)
                hil_rate_sensor_messages += 1
                if config.send_hil_gps:
                    io.send_hil_gps(timestamp_us, position_ned, velocity_ned)
                if visual_odometry_ros_publisher is None:
                    io.send_visual_odometry(timestamp_us, position_ned, velocity_ned, quat_frd_to_ned, angular_velocity_frd)

            if visual_odometry_ros_publisher is not None:
                visual_odometry_ros_publisher.publish(
                    timestamp_us,
                    position_ned,
                    velocity_ned,
                    quat_frd_to_ned,
                    angular_velocity_frd,
                )
            if debug_truth_ros_publisher is not None:
                debug_truth_ros_publisher.publish(
                    timestamp_us,
                    position_ned,
                    velocity_ned,
                    quat_frd_to_ned,
                    angular_velocity_frd,
                )

            if in_px4_alignment_hold:
                sim.zero_ctrl()
            elif local_hover_controller is not None:
                sim.write_direct_flight_controls(
                    local_hover_controller.compute_controls(
                        position_world,
                        velocity_world,
                        quat_world_from_body,
                        angular_velocity_body,
                        float(sim.data.time),
                    )
                )
            else:
                controls = io.poll_actuator_controls(wait=io._received_first_actuator)
                if controls is not None:
                    hil_rate_actuator_messages += 1
                    sim.write_controls(controls, io.armed, config.px4_hover_thrust)
                elif config.no_mavlink or not io.enabled:
                    sim.zero_ctrl()
            if local_hover_controller is None and (config.no_mavlink or not io.enabled):
                sim.zero_ctrl()

            if config.debug_controls and float(sim.data.time) - last_debug_controls_log_time >= 1.0:
                last_debug_controls_log_time = float(sim.data.time)
                mav_controls = None
                if io.last_actuator_controls is not None:
                    mav_controls = np.array(io.last_actuator_controls[:4], dtype=float).tolist()
                ctrl_snapshot = np.array(sim.data.ctrl[: min(4, int(sim.model.nu))], dtype=float).tolist()
                print(
                    "bridge-debug "
                    f"t={float(sim.data.time):.2f}s "
                    f"armed={int(io.armed)} "
                    f"qpos_x={float(sim.data.qpos[0]):.3f} "
                    f"qpos_y={float(sim.data.qpos[1]):.3f} "
                    f"qpos_z={float(sim.data.qpos[2]):.3f} "
                    f"qvel_x={float(sim.data.qvel[0]):.3f} "
                    f"qvel_y={float(sim.data.qvel[1]):.3f} "
                    f"qvel_z={float(sim.data.qvel[2]):.3f} "
                    f"mav_controls={mav_controls} "
                    f"ctrl={ctrl_snapshot}",
                    flush=True,
                )

            if not in_px4_alignment_hold:
                for _ in range(config.physics_substeps_per_sensor):
                    sim.step()
                    hil_rate_steps += 1
            pace_realtime(start_wall_time, start_sim_time, float(sim.data.time), config.real_time_factor)

            if config.debug_hil_rate:
                now_s = time.monotonic()
                elapsed_s = now_s - hil_rate_window_started_at
                if elapsed_s >= 1.0:
                    sim_elapsed_s = float(sim.data.time) - hil_rate_last_sim_time
                    print(
                        "bridge-hil-rate "
                        f"sensor_hz={hil_rate_sensor_messages / elapsed_s:.1f} "
                        f"step_hz={hil_rate_steps / elapsed_s:.1f} "
                        f"actuator_hz={hil_rate_actuator_messages / elapsed_s:.1f} "
                        f"real_time={sim_elapsed_s / elapsed_s:.2f} "
                        f"connected={int(io.connected())} "
                        f"armed={int(io.armed)}",
                        flush=True,
                    )
                    hil_rate_window_started_at = now_s
                    hil_rate_last_sim_time = float(sim.data.time)
                    hil_rate_steps = 0
                    hil_rate_sensor_messages = 0
                    hil_rate_actuator_messages = 0

            sim.sync_viewer()
            step_count += 1
            if config.steps is not None and step_count >= config.steps:
                break
    finally:
        io.close()
        if visual_odometry_ros_publisher is not None:
            visual_odometry_ros_publisher.close()
        if debug_truth_ros_publisher is not None:
            debug_truth_ros_publisher.close()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        sim.close()
        if config.ready_file is not None:
            try:
                config.ready_file.unlink()
            except FileNotFoundError:
                pass
        if config.connected_file is not None:
            try:
                config.connected_file.unlink()
            except FileNotFoundError:
                pass


def parse_args() -> BridgeConfig:
    """Parse command-line arguments into a BridgeConfig."""

    parser = argparse.ArgumentParser(description="Minimal Python PX4 + MuJoCo bridge")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--mavlink-host", default="0.0.0.0")
    parser.add_argument("--mavlink-port", type=int, default=4560)
    parser.add_argument("--actuator-mavlink-port", type=int, default=None)
    parser.add_argument("--no-mavlink", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--px4-hover-thrust", type=float, default=0.60)
    parser.add_argument("--real-time-factor", type=float, default=1.0)
    parser.add_argument("--physics-substeps-per-sensor", type=int, default=1)
    parser.add_argument("--send-hil-gps", action="store_true")
    local_hover_group = parser.add_mutually_exclusive_group()
    local_hover_group.add_argument("--local-hover", dest="local_hover", action="store_true")
    local_hover_group.add_argument("--no-local-hover", dest="local_hover", action="store_false")
    parser.set_defaults(local_hover=False)
    parser.add_argument("--local-hover-target-z", type=float, default=2.0)
    parser.add_argument("--local-hover-ramp-seconds", type=float, default=3.0)
    parser.add_argument("--publish-visual-odometry-ros2", action="store_true")
    parser.add_argument("--publish-debug-truth-ros2", action="store_true")
    parser.add_argument("--presettle-duration-seconds", type=float, default=PRESETTLE_DURATION_SECONDS)
    parser.add_argument("--px4-alignment-hold-seconds", type=float, default=PX4_ALIGNMENT_HOLD_SECONDS)
    parser.add_argument("--ready-file", type=Path, default=None)
    parser.add_argument("--connected-file", type=Path, default=None)
    parser.add_argument("--debug-controls", action="store_true")
    parser.add_argument("--debug-hil-rate", action="store_true")
    args = parser.parse_args()
    model_path = args.model.expanduser()
    if not model_path.is_absolute():
        model_path = (REPO_ROOT / model_path).resolve()
    return BridgeConfig(
        model=model_path,
        mavlink_host=args.mavlink_host,
        mavlink_port=args.mavlink_port,
        actuator_mavlink_port=args.actuator_mavlink_port,
        no_mavlink=args.no_mavlink or mavutil is None,
        headless=args.headless,
        steps=args.steps,
        px4_hover_thrust=args.px4_hover_thrust,
        real_time_factor=args.real_time_factor,
        physics_substeps_per_sensor=max(args.physics_substeps_per_sensor, 1),
        send_hil_gps=args.send_hil_gps,
        local_hover=args.local_hover,
        local_hover_target_z=args.local_hover_target_z,
        local_hover_ramp_seconds=args.local_hover_ramp_seconds,
        publish_visual_odometry_ros2=args.publish_visual_odometry_ros2,
        publish_debug_truth_ros2=args.publish_debug_truth_ros2,
        presettle_duration_seconds=max(args.presettle_duration_seconds, 0.0),
        px4_alignment_hold_seconds=max(args.px4_alignment_hold_seconds, 0.0),
        ready_file=args.ready_file,
        connected_file=args.connected_file,
        debug_controls=args.debug_controls,
        debug_hil_rate=args.debug_hil_rate,
    )


if __name__ == "__main__":
    run(parse_args())
