"""Protect the public TF and Cartesian-target contract from regression."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


REPO = Path(__file__).resolve().parents[3]
MOVEIT_CONFIG = (
    REPO / "src" / "moveit_resources-ros2" / "fruit_arm_moveit_config" / "config"
)
APP = REPO / "src" / "fruit_picking_arm"


def test_public_base_and_cad_adapter_are_separate_frames():
    top = ET.parse(MOVEIT_CONFIG / "fruit_picking_arm.urdf.xacro").getroot()
    assert top.find("./link[@name='world']") is not None
    assert top.find("./link[@name='base_link']") is not None

    mount = top.find("./joint[@name='world_to_base']")
    assert mount is not None
    assert mount.find("parent").get("link") == "world"
    assert mount.find("child").get("link") == "base_link"

    arm = ET.parse(MOVEIT_CONFIG / "fruit_arm_macro.xacro").getroot()
    adapter = arm.find(".//joint[@name='base_link_to_cad_base']")
    assert adapter is not None
    assert adapter.find("child").get("link") == "cad_base_link"
    assert adapter.find("origin").get("rpy") == "0 0 3.141592654"

    ros_link_names = {link.get("name") for link in arm.findall(".//link")}
    assert "cad_base_link" in ros_link_names
    assert {f"arm_link_{index}" for index in range(1, 7)} <= ros_link_names
    assert not any(name and name.startswith("Link_") for name in ros_link_names)


def test_moveit_targets_the_explicit_gripper_tcp():
    gripper = ET.parse(MOVEIT_CONFIG / "gripper.xacro").getroot()
    tcp_joint = gripper.find(".//joint[@name='gripper_tcp_joint']")
    assert tcp_joint is not None
    assert tcp_joint.find("parent").get("link") == "${parent}"
    assert tcp_joint.find("child").get("link") == "gripper_tcp"
    assert tcp_joint.find("origin").get("xyz") == "0 0 -0.086"

    srdf = ET.parse(MOVEIT_CONFIG / "fruit_picking_arm.srdf").getroot()
    chain = srdf.find("./group[@name='arm']/chain")
    assert chain.get("base_link") == "base_link"
    assert chain.get("tip_link") == "gripper_tcp"

    robot = yaml.safe_load((APP / "config" / "robot_params.yaml").read_text())
    assert robot["ik_link"] == "gripper_tcp"


def test_camera_tf_has_one_owner_and_architecture_b_uses_base_link():
    hand_eye = yaml.safe_load(
        (APP / "config" / "hand_eye_params.yaml").read_text()
    )["hand_eye_static_tf"]["ros__parameters"]
    assert hand_eye["parent_frame"] == "base_link"
    assert hand_eye["child_frame"] == "camera_link"

    calibration = yaml.safe_load(
        (APP / "config" / "eye_to_hand_calibration.yaml").read_text()
    )["eye_to_hand_calibrator"]["ros__parameters"]
    assert calibration["base_frame"] == "base_link"
    assert calibration["camera_frame"] == "camera_color_optical_frame"
    assert calibration["camera_root_frame"] == "camera_link"

    architecture_b = (
        APP / "launch" / "architecture_b_target_serial.launch.py"
    ).read_text()
    assert '"output_frame": "base_link"' in architecture_b
    architecture_b_sim = (
        APP / "launch" / "architecture_b_sim.launch.py"
    ).read_text()
    assert '"output_frame": "base_link"' in architecture_b_sim


def test_cartesian_skills_do_not_duplicate_tcp_offset():
    for name in ("approach.py", "grasp.py", "lift.py", "place.py"):
        source = (APP / "fruit_picking_arm" / "skills" / name).read_text()
        assert "0.086" not in source
