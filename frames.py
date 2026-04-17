from __future__ import annotations

import numpy as np


def flu_to_frd(vector: np.ndarray) -> np.ndarray:
    return np.array([vector[0], -vector[1], -vector[2]], dtype=float)


def mujoco_world_to_ned(vector: np.ndarray) -> np.ndarray:
    return np.array([vector[0], -vector[1], -vector[2]], dtype=float)


def quat_wxyz_to_rotmat(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotmat_to_quat_wxyz(rotation: np.ndarray) -> np.ndarray:
    trace = np.trace(rotation)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation[2, 1] - rotation[1, 2]) * s
        y = (rotation[0, 2] - rotation[2, 0]) * s
        z = (rotation[1, 0] - rotation[0, 1]) * s
    else:
        idx = int(np.argmax(np.diag(rotation)))
        if idx == 0:
            s = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            w = (rotation[2, 1] - rotation[1, 2]) / s
            x = 0.25 * s
            y = (rotation[0, 1] + rotation[1, 0]) / s
            z = (rotation[0, 2] + rotation[2, 0]) / s
        elif idx == 1:
            s = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            w = (rotation[0, 2] - rotation[2, 0]) / s
            x = (rotation[0, 1] + rotation[1, 0]) / s
            y = 0.25 * s
            z = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            w = (rotation[1, 0] - rotation[0, 1]) / s
            x = (rotation[0, 2] + rotation[2, 0]) / s
            y = (rotation[1, 2] + rotation[2, 1]) / s
            z = 0.25 * s

    quat = np.array([w, x, y, z], dtype=float)
    return quat / np.linalg.norm(quat)


def mujoco_quat_to_px4_frd_to_ned(quat_world_from_body_flu: np.ndarray) -> np.ndarray:
    flip = np.diag([1.0, -1.0, -1.0])
    rotation_world_from_body_flu = quat_wxyz_to_rotmat(quat_world_from_body_flu)
    rotation_ned_from_frd = flip @ rotation_world_from_body_flu @ flip
    return rotmat_to_quat_wxyz(rotation_ned_from_frd)
