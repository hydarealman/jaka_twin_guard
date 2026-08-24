"""Keep URDF, MoveIt, runtime and serial safety limits synchronized."""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from jaka_single_arm.communication.trajectory_validation import (
    DEFAULT_LOWER_LIMITS,
    DEFAULT_UPPER_LIMITS,
)


REPO = Path(__file__).resolve().parents[3]
ROBOT_CONFIG = (
    REPO
    / "src"
    / "moveit_resources-ros2"
    / "single_arm_jaka_c5_pick_place"
    / "config"
)
APP_CONFIG = REPO / "src" / "jaka_single_arm" / "config"
ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]


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
