"""Keep URDF, MoveIt, runtime and serial safety limits synchronized."""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from fruit_picking_arm.communication.trajectory_validation import (
    DEFAULT_LOWER_LIMITS,
    DEFAULT_UPPER_LIMITS,
)


REPO = Path(__file__).resolve().parents[3]
ROBOT_CONFIG = (
    REPO
    / "src"
    / "moveit_resources-ros2"
    / "fruit_arm_moveit_config"
    / "config"
)
APP_CONFIG = REPO / "src" / "fruit_picking_arm" / "config"
ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]
XACRO_NAMESPACE = "http://www.ros.org/wiki/xacro"


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def urdf_limits() -> dict[str, tuple[float, float]]:
    root = ET.parse(ROBOT_CONFIG / "fruit_arm_macro.xacro").getroot()
    limits = {}
    for joint in root.findall(".//joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name in ARM_JOINTS and limit is not None:
            limits[name] = (
                float(limit.get("lower")),
                float(limit.get("upper")),
            )
    return limits


def nominal_xacro_defaults() -> dict[str, str]:
    root = ET.parse(ROBOT_CONFIG / "fruit_picking_arm.urdf.xacro").getroot()
    return {
        argument.get("name"): argument.get("default")
        for argument in root.findall(f".//{{{XACRO_NAMESPACE}}}arg")
    }


def vector(text: str) -> tuple[float, ...]:
    return tuple(float(value) for value in text.split())


def test_all_arm_limit_layers_are_consistent():
    hard = urdf_limits()
    runtime = load_yaml(APP_CONFIG / "safety_params.yaml")["joint_limits"]
    moveit = load_yaml(ROBOT_CONFIG / "joint_limits.yaml")["joint_limits"]
    margin = float(runtime["margin"])

    assert set(hard) == set(ARM_JOINTS)
    for index, name in enumerate(ARM_JOINTS):
        lower, upper = hard[name]
        assert math.isclose(runtime["lower"][index], lower, abs_tol=1e-9)
        assert math.isclose(runtime["upper"][index], upper, abs_tol=1e-9)
        assert math.isclose(
            moveit[name]["min_position"], lower + margin, abs_tol=1e-9
        )
        assert math.isclose(
            moveit[name]["max_position"], upper - margin, abs_tol=1e-9
        )
        assert math.isclose(
            DEFAULT_LOWER_LIMITS[index],
            moveit[name]["min_position"],
            abs_tol=1e-9,
        )
        assert math.isclose(
            DEFAULT_UPPER_LIMITS[index],
            moveit[name]["max_position"],
            abs_tol=1e-9,
        )


def test_nominal_j2_j3_zero_geometry_matches_mechanical_convention():
    defaults = nominal_xacro_defaults()
    joint_2_pitch = vector(defaults["joint_2_rpy"])[1]
    joint_3_pitch = vector(defaults["joint_3_rpy"])[1]
    joint_3_xyz = vector(defaults["joint_3_xyz"])

    # Real-arm datum: at the two indicated hard stops, J2=144 deg and
    # J3=-90 deg make the straight upper/forearm assembly horizontal.
    assert math.isclose(
        joint_2_pitch
        + joint_3_pitch
        + math.radians(144.0)
        - math.radians(90.0),
        0.0,
        abs_tol=1e-9,
    )

    # Consequently the upper arm is vertical at J2=54 deg.
    assert math.isclose(
        joint_2_pitch + math.radians(54.0),
        -joint_3_pitch,
        abs_tol=1e-9,
    )

    # At J3=0 the child X axis (the forearm/roll-axis direction) must be
    # perpendicular to the upper-arm centreline from J2 to J3.
    upper_arm_x = joint_3_xyz[0]
    upper_arm_z = joint_3_xyz[2]
    forearm_x = math.cos(joint_3_pitch)
    forearm_z = -math.sin(joint_3_pitch)
    dot_product = upper_arm_x * forearm_x + upper_arm_z * forearm_z
    assert math.isclose(dot_product, 0.0, abs_tol=1e-9)


def test_gripper_geometry_has_exactly_100_mm_clear_opening():
    root = ET.parse(ROBOT_CONFIG / "gripper.xacro").getroot()
    limits = {
        joint.get("name"): joint.find("limit")
        for joint in root.findall(".//joint")
        if joint.get("name") in ("left_finger_joint", "right_finger_joint")
    }
    left_center = float(limits["left_finger_joint"].get("upper"))
    right_center = float(limits["right_finger_joint"].get("lower"))
    finger_half_thickness = 0.012 / 2.0
    clear_opening = (
        left_center - finger_half_thickness
    ) - (
        right_center + finger_half_thickness
    )
    assert math.isclose(clear_opening, 0.100, abs_tol=1e-12)
