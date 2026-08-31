"""Protect the public TF and Cartesian-target contract from regression."""

from __future__ import annotations

from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest
import yaml

from fruit_picking_arm.calibration_profile import load_calibration_selection


REPO = Path(__file__).resolve().parents[3]
MOVEIT_CONFIG = (
    REPO / "src" / "moveit_resources-ros2" / "fruit_arm_moveit_config" / "config"
)
APP = REPO / "src" / "fruit_picking_arm"
XACRO_NAMESPACE = "http://www.ros.org/wiki/xacro"


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
    # Public +X is the physical workcell front/J1=0 plane, not the raw
    # SolidWorks assembly axis. The old 180-degree adapter is forbidden.
    assert adapter.find("origin").get("rpy") == "0 0 -1.5707963267949"
    assert adapter.find("origin").get("xyz") == (
        "0.269625725668887 0.0397188996984261 0.190511595840763"
    )

    # In the public frame the 400x400 mm base footprint is centred on X/Y,
    # its bottom face is z=0, and the J1 axis remains at its delivered CAD
    # location (-5 mm, +17 mm, +82 mm) relative to that geometric centre.
    adapter_xyz = [float(v) for v in adapter.find("origin").get("xyz").split()]
    joint_1 = arm.find(".//joint[@name='joint_1']")
    joint_1_xyz = [float(v) for v in joint_1.find("origin").get("xyz").split()]
    # Rz(-90deg) * [x, y, z] = [y, -x, z].
    rotated_joint_1 = [joint_1_xyz[1], -joint_1_xyz[0], joint_1_xyz[2]]
    composed = [a + b for a, b in zip(adapter_xyz, rotated_joint_1)]
    assert composed == pytest.approx([-0.005, 0.017, 0.082], abs=1.0e-9)

    # The raw J1 frame has +90deg yaw and the public CAD adapter has -90deg;
    # they cancel. Consequently J2/J3 motion stays in public X-Z at J1=0.
    joint_1_rpy = top.find(
        f".//{{{XACRO_NAMESPACE}}}arg[@name='joint_1_rpy']"
    ).get("default")
    joint_1_yaw = float(joint_1_rpy.split()[2])
    adapter_yaw = float(adapter.find("origin").get("rpy").split()[2])
    assert adapter_yaw + joint_1_yaw == pytest.approx(0.0, abs=1.0e-12)
    assert joint_1.find("axis").get("xyz") == "0 0 -1"

    ros_link_names = {
        call.get("name")
        for call in arm.findall(f".//{{{XACRO_NAMESPACE}}}cad_link")
    }
    assert "cad_base_link" in ros_link_names
    assert {f"arm_link_{index}" for index in range(1, 7)} <= ros_link_names
    assert not any(name and name.startswith("Link_") for name in ros_link_names)


def test_rebased_base_mesh_is_centred_and_sits_on_z_zero():
    mesh = (
        REPO
        / "src"
        / "moveit_resources-ros2"
        / "fruit_arm_description"
        / "meshes"
        / "production_current"
        / "base_link.STL"
    ).read_bytes()
    triangle_count = struct.unpack_from("<I", mesh, 80)[0]
    assert len(mesh) == 84 + triangle_count * 50

    vertices = []
    for triangle_index in range(triangle_count):
        vertex_offset = 84 + triangle_index * 50 + 12
        for vertex_index in range(3):
            vertices.append(
                struct.unpack_from(
                    "<fff", mesh, vertex_offset + vertex_index * 12
                )
            )

    adapter = (0.269625725668887, 0.0397188996984261, 0.190511595840763)
    transformed = [
        (
            adapter[0] + vertex[1],
            adapter[1] - vertex[0],
            adapter[2] + vertex[2],
        )
        for vertex in vertices
    ]
    minimum = [min(vertex[axis] for vertex in transformed) for axis in range(3)]
    maximum = [max(vertex[axis] for vertex in transformed) for axis in range(3)]
    assert minimum == pytest.approx([-0.2, -0.2, 0.0], abs=2.0e-7)
    assert maximum == pytest.approx([0.2, 0.2, 0.09925], abs=2.0e-7)


def test_moveit_targets_the_explicit_gripper_tcp():
    gripper = ET.parse(MOVEIT_CONFIG / "gripper.xacro").getroot()
    tcp_joint = gripper.find(".//joint[@name='gripper_tcp_joint']")
    assert tcp_joint is not None
    assert tcp_joint.find("parent").get("link") == "gripper_base"
    assert tcp_joint.find("child").get("link") == "gripper_tcp"
    assert tcp_joint.find("origin").get("xyz") == "${tcp_xyz}"
    assert tcp_joint.find("origin").get("rpy") == "${tcp_rpy}"

    top = ET.parse(MOVEIT_CONFIG / "fruit_picking_arm.urdf.xacro").getroot()
    arguments = {
        argument.get("name"): argument.get("default")
        for argument in top.findall(f".//{{{XACRO_NAMESPACE}}}arg")
    }
    assert arguments["tcp_xyz"] == "0 0 0.124"
    assert arguments["tcp_rpy"] == "0 0 0"

    arm = ET.parse(MOVEIT_CONFIG / "fruit_arm_macro.xacro").getroot()
    flange = arm.find(".//joint[@name='tool_flange_joint']")
    assert flange is not None
    assert flange.find("origin").get("xyz") == "0.0422 0 0"
    assert flange.find("origin").get("rpy") == "0 1.5707963267949 0"

    srdf = ET.parse(MOVEIT_CONFIG / "fruit_picking_arm.srdf").getroot()
    chain = srdf.find("./group[@name='arm']/chain")
    assert chain.get("base_link") == "base_link"
    assert chain.get("tip_link") == "gripper_tcp"

    robot = yaml.safe_load((APP / "config" / "robot_params.yaml").read_text())
    assert robot["ik_link"] == "gripper_tcp"


def test_nominal_launch_keeps_current_four_finger_tcp_measurement():
    selection = load_calibration_selection("nominal", "fruit-arm-primary")
    assert selection.xacro_mappings["tcp_xyz"] == "0 0 0.124"
    assert selection.xacro_mappings["tcp_rpy"] == "0 0 0"


def test_camera_tf_has_one_owner_and_architecture_b_uses_base_link():
    hand_eye = yaml.safe_load(
        (APP / "config" / "hand_eye_params.yaml").read_text()
    )["hand_eye_static_tf"]["ros__parameters"]
    assert hand_eye["parent_frame"] == "base_link"
    assert hand_eye["child_frame"] == "camera_link"
    assert hand_eye["translation_m"] == pytest.approx(
        [0.7646083400444, 0.2480702804854, 0.6010795563502],
        abs=1.0e-12,
    )
    assert hand_eye["quaternion_xyzw"] == pytest.approx(
        [0.5663754069818, 0.0095076947572, -0.8233852680552, 0.0341350619759],
        abs=1.0e-12,
    )

    # Undo the new base origin and -90deg yaw. The recovered camera pose must
    # equal the original hand-eye result, proving that only coordinates moved.
    new_t = hand_eye["translation_m"]
    adapter_t = [0.269625725668887, 0.0397188996984261, 0.190511595840763]
    delta = [new_t[index] - adapter_t[index] for index in range(3)]
    recovered_old_t = [-delta[1], delta[0], delta[2]]  # Rz(+90deg)
    assert recovered_old_t == pytest.approx(
        [-0.2083513807870, 0.4949826143755, 0.4105679605094],
        abs=2.0e-13,
    )

    def quaternion_multiply(a, b):
        x1, y1, z1, w1 = a
        x2, y2, z2, w2 = b
        return [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]

    root_half = 2.0 ** -0.5
    recovered_old_q = quaternion_multiply(
        [0.0, 0.0, root_half, root_half], hand_eye["quaternion_xyzw"]
    )
    assert recovered_old_q == pytest.approx(
        [0.3937649355379, 0.4072108464104, -0.5580841727716, 0.6063584403703],
        abs=2.0e-13,
    )

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


def test_hand_eye_sessions_are_bound_to_the_rebased_geometry_contract():
    source = (APP / "src" / "eye_to_hand_calibrator.cpp").read_text()
    assert (
        'kBaseFrameContract[] = "base_bottom_center_x_forward_v1"'
        in source
    )
    assert 'storage << "format_version" << 2' in source
    assert source.count(
        'storage << "base_frame_contract" << kBaseFrameContract'
    ) == 2
    assert 'stored_base_contract != kBaseFrameContract' in source
    assert "legacy or unknown base_link geometry" in source


def test_real_table_roi_and_architecture_b_share_x_forward_contract():
    perception = yaml.safe_load(
        (APP / "config" / "perception_params.yaml").read_text()
    )
    table = perception["table_perception"]
    assert table["world_roi_min"] == [-0.03, -0.41, -0.11]
    assert table["world_roi_max"] == [0.97, 0.89, 0.14]

    target_source = (
        APP / "fruit_picking_arm" / "perception" / "fruit_target_node.py"
    ).read_text()
    assert '"world_roi_min": [-0.03, -0.41, -0.11]' in target_source
    assert '"world_roi_max": [0.97, 0.89, 0.14]' in target_source

    architecture_b = yaml.safe_load(
        (APP / "config" / "architecture_b_serial.yaml").read_text()
    )["serial_fruit_target_bridge"]["ros__parameters"]
    assert architecture_b["workspace_min_mm"] == [200, -600, 0]
    assert architecture_b["workspace_max_mm"] == [900, 600, 1000]


def test_cartesian_skills_do_not_duplicate_tcp_offset():
    for name in ("approach.py", "grasp.py", "lift.py", "place.py"):
        source = (APP / "fruit_picking_arm" / "skills" / name).read_text()
        assert "0.086" not in source
