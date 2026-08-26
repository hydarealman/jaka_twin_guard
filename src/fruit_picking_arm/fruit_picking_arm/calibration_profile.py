"""Strict per-robot kinematic profile loading for ROS launch assembly.

The calibration algorithm and profile writer are C++17.  This module only
converts a validated YAML profile into xacro/ROS parameter values at launch.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Any

import yaml


NOMINAL_MODEL_HASH = "fruit-arm-kinematics-v1"
_SERIAL_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")

_JOINT_RPY = (
    (0.0, 0.0, 0.0),
    (0.0, -1.137899491, 0.0),
    (0.0, 1.011527890, 0.0),
    (3.141592654, 0.0, 0.0),
    (0.0, 0.583631552, 0.0),
    (0.0, 0.0, 0.0),
)
_JOINT_AXES = (
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (-0.834467, 0.0, -0.551058),
)
_J3_XYZ = (0.399412, -0.015781, 0.184581)
_J5_XYZ = (-0.372174, -0.034450, -0.004890)


@dataclass(frozen=True)
class CalibrationSelection:
    mode: str
    robot_serial: str
    profile_path: str
    xacro_mappings: dict[str, str]
    calibrated_camera_parameters: dict[str, Any] | None


def _matrix_multiply(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)] for r in range(3)]


def _rpy_matrix(rpy: tuple[float, float, float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _axis_matrix(axis: tuple[float, float, float], angle: float) -> list[list[float]]:
    norm = math.sqrt(sum(value * value for value in axis))
    x, y, z = (value / norm for value in axis)
    c, s, one = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [c + x * x * one, x * y * one - z * s, x * z * one + y * s],
        [y * x * one + z * s, c + y * y * one, y * z * one - x * s],
        [z * x * one - y * s, z * y * one + x * s, c + z * z * one],
    ]


def _matrix_rpy(matrix: list[list[float]]) -> tuple[float, float, float]:
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    if abs(math.cos(pitch)) > 1.0e-9:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def _corrected_rpy(index: int, offset: float) -> tuple[float, float, float]:
    return _matrix_rpy(_matrix_multiply(_rpy_matrix(_JOINT_RPY[index]), _axis_matrix(_JOINT_AXES[index], offset)))


def _corrected_vector(nominal: tuple[float, float, float], correction: float) -> tuple[float, float, float]:
    norm = math.sqrt(sum(value * value for value in nominal))
    return tuple(value + correction * value / norm for value in nominal)


def _angle_axis_matrix(values: list[float]) -> list[list[float]]:
    angle = math.sqrt(sum(value * value for value in values))
    if angle < 1.0e-15:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    return _axis_matrix(tuple(value / angle for value in values), angle)


def _quaternion_xyzw(matrix: list[list[float]]) -> list[float]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = [
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
            0.25 * scale,
        ]
    else:
        index = max(range(3), key=lambda item: matrix[item][item])
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
            quaternion = [0.25 * scale, (matrix[0][1] + matrix[1][0]) / scale,
                          (matrix[0][2] + matrix[2][0]) / scale, (matrix[2][1] - matrix[1][2]) / scale]
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
            quaternion = [(matrix[0][1] + matrix[1][0]) / scale, 0.25 * scale,
                          (matrix[1][2] + matrix[2][1]) / scale, (matrix[0][2] - matrix[2][0]) / scale]
        else:
            scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
            quaternion = [(matrix[0][2] + matrix[2][0]) / scale,
                          (matrix[1][2] + matrix[2][1]) / scale, 0.25 * scale,
                          (matrix[1][0] - matrix[0][1]) / scale]
    norm = math.sqrt(sum(value * value for value in quaternion))
    return [value / norm for value in quaternion]


def _vector(values: Any, count: int, name: str) -> list[float]:
    if not isinstance(values, list) or len(values) != count:
        raise RuntimeError(f"{name} must contain {count} values")
    converted = [float(value) for value in values]
    if not all(math.isfinite(value) for value in converted):
        raise RuntimeError(f"{name} contains a non-finite value")
    return converted


def default_profile_path(robot_serial: str) -> Path:
    return Path.home() / ".config" / "jaka_twin_guard" / "robots" / robot_serial / "kinematics.yaml"


def load_calibration_selection(
    mode: str, robot_serial: str, calibration_file: str = ""
) -> CalibrationSelection:
    normalized_mode = mode.strip().lower()
    if normalized_mode not in {"nominal", "calibrated"}:
        raise RuntimeError("kinematics_mode must be 'nominal' or 'calibrated'")
    if not _SERIAL_PATTERN.fullmatch(robot_serial):
        raise RuntimeError("robot_serial is required and may contain only letters, digits, '.', '_' and '-'")

    offsets = [0.0] * 6
    link_j3 = 0.0
    link_j5 = 0.0
    tcp_translation = [0.0, 0.0, -0.086]
    tcp_rpy = (0.0, 0.0, 0.0)
    camera_parameters = None
    profile_path = Path(calibration_file).expanduser() if calibration_file else default_profile_path(robot_serial)

    if normalized_mode == "calibrated":
        if not profile_path.is_file():
            raise RuntimeError(f"calibrated mode requires profile: {profile_path}")
        profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        if not isinstance(profile, dict):
            raise RuntimeError("kinematic profile root must be a mapping")
        if profile.get("schema_version") != 1:
            raise RuntimeError("unsupported kinematic profile schema_version")
        if profile.get("robot_serial") != robot_serial:
            raise RuntimeError("kinematic profile robot_serial does not match launch argument")
        if profile.get("nominal_model_hash") != NOMINAL_MODEL_HASH:
            raise RuntimeError("kinematic profile was generated for a different nominal model")
        if profile.get("validated") is not True or profile.get("observable") is not True:
            raise RuntimeError("kinematic profile is not validated and observable")
        offsets = _vector(profile.get("joint_offsets_rad"), 6, "joint_offsets_rad")
        if abs(offsets[0]) > 1.0e-12 or abs(offsets[5]) > 1.0e-12:
            raise RuntimeError("D455 profile must keep unobservable J1/J6 offsets at zero")
        link_j3 = float(profile.get("j2_j3_length_correction_m", 0.0))
        link_j5 = float(profile.get("j4_j5_length_correction_m", 0.0))
        if abs(link_j3) > 0.03 or abs(link_j5) > 0.03:
            raise RuntimeError("link correction exceeds the 30 mm safety bound")
        flange_tcp = profile.get("flange_T_tcp", {})
        tcp_translation = _vector(flange_tcp.get("translation_m"), 3, "flange_T_tcp.translation_m")
        tcp_angle_axis = _vector(flange_tcp.get("angle_axis"), 3, "flange_T_tcp.angle_axis")
        tcp_rpy = _matrix_rpy(_angle_axis_matrix(tcp_angle_axis))
        base_camera = profile.get("base_T_camera", {})
        camera_translation = _vector(base_camera.get("translation_m"), 3, "base_T_camera.translation_m")
        camera_angle_axis = _vector(base_camera.get("angle_axis"), 3, "base_T_camera.angle_axis")
        camera_parameters = {
            "calibrated": True,
            "parent_frame": "base_link",
            "child_frame": "camera_link",
            "translation_m": camera_translation,
            "quaternion_xyzw": _quaternion_xyzw(_angle_axis_matrix(camera_angle_axis)),
        }

    mappings: dict[str, str] = {}
    for index, offset in enumerate(offsets):
        mappings[f"joint_{index + 1}_rpy"] = " ".join(f"{value:.15g}" for value in _corrected_rpy(index, offset))
    mappings["joint_3_xyz"] = " ".join(f"{value:.15g}" for value in _corrected_vector(_J3_XYZ, link_j3))
    mappings["joint_5_xyz"] = " ".join(f"{value:.15g}" for value in _corrected_vector(_J5_XYZ, link_j5))
    mappings["tcp_xyz"] = " ".join(f"{value:.15g}" for value in tcp_translation)
    mappings["tcp_rpy"] = " ".join(f"{value:.15g}" for value in tcp_rpy)
    return CalibrationSelection(
        normalized_mode, robot_serial, str(profile_path), mappings, camera_parameters
    )
