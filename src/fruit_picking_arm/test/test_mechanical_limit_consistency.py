"""Keep URDF, MoveIt, runtime and serial safety limits synchronized."""

from __future__ import annotations

import json
import math
from pathlib import Path
import struct
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
COMMON_SH = REPO / "scripts" / "single_arm" / "common.sh"
SERIAL_CONTROLLER = (
    REPO / "src" / "fruit_picking_arm" / "src" / "serial_trajectory_controller.cpp"
)
PRODUCTION_MESHES = (
    REPO
    / "src"
    / "moveit_resources-ros2"
    / "fruit_arm_description"
    / "meshes"
    / "production_current"
)
GRIPPER_MESHES = PRODUCTION_MESHES.parent / "gripper_current"
ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]
SERIAL_JOINT_MAX_VELOCITIES = [0.42, 0.49, 0.49, 1.29, 1.50, 1.50]
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


def binary_stl_bounds(path: Path) -> tuple[tuple[float, ...], tuple[float, ...], int]:
    data = path.read_bytes()
    assert len(data) >= 84
    triangles = struct.unpack_from("<I", data, 80)[0]
    assert triangles > 0
    assert len(data) == 84 + triangles * 50
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    for triangle in range(triangles):
        facet = 84 + triangle * 50
        for vertex in range(3):
            xyz = struct.unpack_from("<3f", data, facet + 12 + vertex * 12)
            for axis, value in enumerate(xyz):
                minimum[axis] = min(minimum[axis], value)
                maximum[axis] = max(maximum[axis], value)
    return tuple(minimum), tuple(maximum), triangles


def test_all_arm_limit_layers_are_consistent():
    hard = urdf_limits()
    runtime = load_yaml(APP_CONFIG / "safety_params.yaml")["joint_limits"]
    moveit = load_yaml(ROBOT_CONFIG / "joint_limits.yaml")["joint_limits"]
    serial = load_yaml(APP_CONFIG / "architecture_a_serial.yaml")[
        "serial_trajectory_controller"
    ]["ros__parameters"]
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
        assert math.isclose(
            serial["arm_lower_limits"][index],
            moveit[name]["min_position"],
            abs_tol=1e-9,
        )
        assert math.isclose(
            serial["arm_upper_limits"][index],
            moveit[name]["max_position"],
            abs_tol=1e-9,
        )
        assert math.isclose(
            serial["arm_feedback_lower_limits"][index],
            lower,
            abs_tol=1e-9,
        )
        assert math.isclose(
            serial["arm_feedback_upper_limits"][index],
            upper,
            abs_tol=1e-9,
        )

    assert serial["arm_joint_names"] == ARM_JOINTS


def test_serial_velocity_limits_match_firmware_contract_and_cpp_defaults():
    serial = load_yaml(APP_CONFIG / "architecture_a_serial.yaml")[
        "serial_trajectory_controller"
    ]["ros__parameters"]
    controller = SERIAL_CONTROLLER.read_text(encoding="utf-8")

    assert serial["arm_max_velocities"] == SERIAL_JOINT_MAX_VELOCITIES
    assert (
        '"arm_max_velocities", {0.42, 0.49, 0.49, 1.29, 1.50, 1.50}'
        in controller
    )

    # MoveIt may plan faster in other modes, but Architecture A's real serial
    # gate must remain no higher than the configured planning-model maximum.
    moveit = load_yaml(ROBOT_CONFIG / "joint_limits.yaml")["joint_limits"]
    for name, serial_limit in zip(ARM_JOINTS, SERIAL_JOINT_MAX_VELOCITIES):
        assert serial_limit <= moveit[name]["max_velocity"]


def test_real_robot_state_readiness_uses_validated_serial_marker():
    common = COMMON_SH.read_text(encoding="utf-8")
    controller = SERIAL_CONTROLLER.read_text(encoding="utf-8")
    readiness = common.split("require_real_robot_state()", 1)[1].split(
        "wait_for_log_pattern()", 1
    )[0]

    assert "REAL_ROBOT_STATE_READY:" in controller
    assert "REAL_ROBOT_STATE_READY:" in readiness
    assert "ros2 topic echo" not in readiness
    assert "process_or_group_is_alive" in readiness


def test_nominal_joint_frames_match_latest_solidworks_export():
    defaults = nominal_xacro_defaults()
    assert vector(defaults["joint_1_rpy"]) == (0.0, 0.0, 1.5707963267949)
    for name in (
        "joint_2_rpy", "joint_3_rpy", "joint_4_rpy",
        "joint_5_rpy", "joint_6_rpy",
    ):
        assert vector(defaults[name]) == (0.0, 0.0, 0.0)
    assert vector(defaults["joint_3_xyz"]) == (
        -0.435124907351151, 0.0, 0.0653170345518886,
    )
    assert vector(defaults["joint_5_xyz"]) == (
        0.35399, 0.0195, 0.0030892,
    )

    root = ET.parse(ROBOT_CONFIG / "fruit_arm_macro.xacro").getroot()
    joints = {
        joint.get("name"): joint
        for joint in root.findall(".//joint")
        if joint.get("name") in ARM_JOINTS
    }
    expected_axes = {
        "joint_1": (0.0, 0.0, -1.0),
        "joint_2": (0.0, 1.0, 0.0),
        "joint_3": (0.0, 1.0, 0.0),
        "joint_4": (0.99996, 0.0, 0.0087265),
        "joint_5": (0.0, 1.0, 0.0),
        "joint_6": (1.0, 0.0, 0.0),
    }
    for name, expected in expected_axes.items():
        assert vector(joints[name].find("axis").get("xyz")) == expected


def test_long_forearm_mesh_is_upstream_of_joint_5():
    """Prevent the SW exporter rigid-group regression reported on 2026-08-27."""
    link4_min, link4_max, link4_triangles = binary_stl_bounds(
        PRODUCTION_MESHES / "Link4.STL"
    )
    link5_min, link5_max, link5_triangles = binary_stl_bounds(
        PRODUCTION_MESHES / "Link5.STL"
    )
    joint_5_x = vector(nominal_xacro_defaults()["joint_5_xyz"])[0]

    # Link4 owns the approximately 354 mm J4-to-J5 forearm and reaches the
    # downstream side of the J5 axis in its q=0 local frame.
    assert link4_max[0] > joint_5_x
    assert link4_max[0] - link4_min[0] > 0.40

    # Link5 is only the short J5-to-J6 wrist.  The broken exporter output had
    # an approximately 0.32 m backward span here, causing J5 to swing the arm.
    assert link5_max[0] - link5_min[0] < 0.10
    assert link5_min[0] > -0.06
    assert link4_triangles > 1000
    assert link5_triangles > 1000


def test_link6_is_a_pure_arm_rigid_body_without_baked_in_gripper():
    minimum, maximum, triangles = binary_stl_bounds(PRODUCTION_MESHES / "Link6.STL")

    # The broken classifier baked the fixed gripper mechanism into Link6 and
    # extended it to x=0.1187 m.  The corrected J6 body ends near the measured
    # mount plane at x=0.0422 m.
    assert maximum[0] < 0.060
    assert minimum[0] > -0.060
    assert triangles > 1000


def test_actual_four_finger_gripper_endpoint_contract():
    manifest = json.loads((GRIPPER_MESHES / "KINEMATICS.json").read_text(encoding="utf-8"))
    assert manifest["units"] == "metres"
    assert manifest["mesh_pose"] == "closed endpoint (q=0)"
    assert manifest["leaf_counts"] == {
        "fixed": 3,
        "slider": 1,
        "drive_links": 4,
        "finger_parts": 8,
    }
    assert manifest["j6_to_gripper_mount"]["xyz_m"] == [0.0422, 0.0, 0.0]
    assert manifest["gripper_mount_to_tcp"]["xyz_m"] == [0.0, 0.0, 0.124]
    assert max(
        float(joint["endpoint_residual_mm"])
        for joint in manifest["joints"].values()
    ) < 1e-9

    expected_meshes = {
        "gripper_base", "center_slider",
        "drive_neg_x", "drive_neg_y", "drive_pos_x", "drive_pos_y",
        "finger_neg_x", "finger_neg_y", "finger_pos_x", "finger_pos_y",
    }
    assert set(manifest["exports"]) == expected_meshes
    for name in expected_meshes:
        _, _, triangles = binary_stl_bounds(GRIPPER_MESHES / f"{name}.STL")
        assert triangles > 0


def test_gripper_urdf_mimic_endpoints_match_step_endpoints():
    root = ET.parse(ROBOT_CONFIG / "gripper.xacro").getroot()
    limits = {
        joint.get("name"): joint.find("limit")
        for joint in root.findall(".//joint")
        if joint.get("name") in ("left_finger_joint", "right_finger_joint")
    }
    master_travel = float(limits["left_finger_joint"].get("upper"))
    assert master_travel == 0.056
    assert float(limits["right_finger_joint"].get("lower")) == -0.056

    calls = root.findall(f".//{{{XACRO_NAMESPACE}}}gripper_moving_link")
    assert len(calls) == 9
    for call in calls:
        travel = float(call.get("travel"))
        multiplier = float(call.get("multiplier"))
        assert math.isclose(master_travel * multiplier, travel, abs_tol=1e-13)
