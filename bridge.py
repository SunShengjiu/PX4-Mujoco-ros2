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
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np

from frames import flu_to_frd, mujoco_quat_to_px4_frd_to_ned, mujoco_world_to_ned

try:
    import mujoco.viewer
except Exception:
    mujoco.viewer = None

try:
    import glfw
except Exception:
    glfw = None

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


@dataclass
class BridgeConfig:
    """Runtime configuration for the Python bridge."""

    model: Path = DEFAULT_MODEL
    mavlink_host: str = "0.0.0.0"
    mavlink_port: int = 4560
    no_mavlink: bool = False
    headless: bool = False
    steps: Optional[int] = None
    px4_hover_thrust: float = 0.60
    real_time_factor: float = 1.0


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
        gyro = flu_to_frd(np.array(self._sensor_slice("body_gyro"), dtype=float))
        accel = flu_to_frd(np.array(self._sensor_slice("body_linacc"), dtype=float))
        return gyro, accel

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


class Px4MavlinkIo:
    """Encapsulates the MAVLink-side IO contract with PX4 SITL."""

    def __init__(self, host: str, port: int, enabled: bool) -> None:
        self.enabled = enabled and mavutil is not None
        self.host = host
        self.port = port
        self.master = None
        self.armed = False
        self._last_heartbeat = 0.0
        self._received_first_actuator = False

    def connect(self) -> None:
        if not self.enabled:
            return
        endpoint = f"tcpin:{self.host}:{self.port}"
        self.master = mavutil.mavlink_connection(endpoint, source_system=1, source_component=200)
        print(f"Listening for PX4 on {endpoint}")

    def poll_actuator_controls(self, wait: bool) -> Optional[np.ndarray]:
        if not self.enabled or self.master is None:
            return None

        deadline = time.monotonic() + 0.02 if wait else time.monotonic()

        while True:
            timeout = max(0.0, deadline - time.monotonic()) if wait else 0.0
            if wait and timeout <= 0.0:
                return None

            msg = self.master.recv_match(blocking=wait, timeout=timeout)
            if msg is None:
                return None

            msg_type = msg.get_type()
            if msg_type == "HEARTBEAT":
                self._last_heartbeat = time.time()
            elif msg_type == "HIL_ACTUATOR_CONTROLS":
                self.armed = bool(msg.mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._received_first_actuator = True
                return np.array(msg.controls, dtype=float)

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
        velocity_body_frd = sanitize_vector(rotate_vector_by_quat_conjugate(orientation, velocity_world), 3)
        pose_cov = sanitize_covariance([1e-4] * 21)
        vel_cov = sanitize_covariance([1e-4] * 21)
        self.master.mav.odometry_send(
            clamp_uint64(timestamp_us),
            clamp_uint8(mavutil.mavlink.MAV_FRAME_LOCAL_NED),
            clamp_uint8(mavutil.mavlink.MAV_FRAME_BODY_FRD),
            sanitize_float(position[0]),
            sanitize_float(position[1]),
            sanitize_float(position[2]),
            orientation.tolist(),
            sanitize_float(velocity_body_frd[0]),
            sanitize_float(velocity_body_frd[1]),
            sanitize_float(velocity_body_frd[2]),
            sanitize_float(angular_velocity[0]),
            sanitize_float(angular_velocity[1]),
            sanitize_float(angular_velocity[2]),
            pose_cov,
            vel_cov,
            clamp_uint8(0),
            clamp_uint8(mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION),
            clamp_uint8(100),
        )


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


def pace_realtime(start_wall_time: float, start_sim_time: float, current_sim_time: float, real_time_factor: float) -> None:
    """Throttle simulation so MuJoCo time advances at a chosen real-time factor."""

    if real_time_factor <= 0.0:
        return

    target_wall_time = start_wall_time + (current_sim_time - start_sim_time) / real_time_factor
    sleep_duration = target_wall_time - time.monotonic()
    if sleep_duration > 0.0:
        time.sleep(sleep_duration)


def run(config: BridgeConfig) -> None:
    """Run the Python PX4-MuJoCo bridge until the viewer closes or steps are exhausted."""

    sim = MuJoCoSim(config.model)
    if not config.headless:
        sim.launch_viewer()

    io = Px4MavlinkIo(config.mavlink_host, config.mavlink_port, not config.no_mavlink)
    if io.enabled:
        sim.validate_px4_hil_model_contract()
    io.connect()
    mujoco.mj_forward(sim.model, sim.data)
    start_wall_time = time.monotonic()
    start_sim_time = float(sim.data.time)

    step_count = 0
    try:
        while sim.viewer_running():
            if not sim.should_step():
                sim.refresh_paused_state()
                sim.sync_viewer()
                time.sleep(0.01)
                continue

            timestamp_us = int(sim.data.time * 1e6)
            position_ned, velocity_ned, quat_frd_to_ned, angular_velocity_frd = sim.state_ned()
            if io.enabled:
                gyro_frd, accel_frd = sim.imu_frd()
                mag_frd = sim.mag_frd()
                pressure_hpa = pressure_hpa_from_altitude(HOME_ALT_M - position_ned[2])
                pressure_alt_m = float(HOME_ALT_M - position_ned[2])

                io.send_heartbeat()
                io.send_hil_sensor(timestamp_us, accel_frd, gyro_frd, mag_frd, pressure_hpa, pressure_alt_m)
                io.send_hil_gps(timestamp_us, position_ned, velocity_ned)
                io.send_visual_odometry(timestamp_us, position_ned, velocity_ned, quat_frd_to_ned, angular_velocity_frd)

            controls = io.poll_actuator_controls(wait=io._received_first_actuator)
            if controls is not None:
                sim.write_controls(controls, io.armed, config.px4_hover_thrust)
            elif config.no_mavlink or not io.enabled:
                sim.zero_ctrl()

            sim.step()
            pace_realtime(start_wall_time, start_sim_time, float(sim.data.time), config.real_time_factor)

            sim.sync_viewer()
            step_count += 1
            if config.steps is not None and step_count >= config.steps:
                break
    finally:
        sim.close()


def parse_args() -> BridgeConfig:
    """Parse command-line arguments into a BridgeConfig."""

    parser = argparse.ArgumentParser(description="Minimal Python PX4 + MuJoCo bridge")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--mavlink-host", default="0.0.0.0")
    parser.add_argument("--mavlink-port", type=int, default=4560)
    parser.add_argument("--no-mavlink", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--px4-hover-thrust", type=float, default=0.60)
    parser.add_argument("--real-time-factor", type=float, default=1.0)
    args = parser.parse_args()
    model_path = args.model.expanduser()
    if not model_path.is_absolute():
        model_path = (REPO_ROOT / model_path).resolve()
    return BridgeConfig(
        model=model_path,
        mavlink_host=args.mavlink_host,
        mavlink_port=args.mavlink_port,
        no_mavlink=args.no_mavlink or mavutil is None,
        headless=args.headless,
        steps=args.steps,
        px4_hover_thrust=args.px4_hover_thrust,
        real_time_factor=args.real_time_factor,
    )


if __name__ == "__main__":
    run(parse_args())
