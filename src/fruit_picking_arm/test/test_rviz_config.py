from pathlib import Path

import yaml


def test_pick_place_rviz_is_valid_and_contains_real_debug_displays():
    config = (
        Path(__file__).resolve().parents[2]
        / "moveit_resources-ros2"
        / "fruit_arm_moveit_config"
        / "config"
        / "fruit_picking_arm.rviz"
    )
    data = yaml.safe_load(config.read_text(encoding="utf-8"))
    manager = data["Visualization Manager"]
    displays = {item["Name"]: item for item in manager["Displays"]}

    assert manager["Global Options"]["Fixed Frame"] == "world"
    assert displays["Grid"]["Enabled"] is True
    assert displays["RobotModel"]["Enabled"] is True
    assert "MotionPlanning" not in displays
    assert (
        displays["Real D455 Raw Depth (camera frame)"]["Topic"]["Value"]
        == "/perception/debug/camera_cloud"
    )


def test_real_plan_execute_rviz_uses_arm_group_and_conservative_scaling():
    config = (
        Path(__file__).resolve().parents[2]
        / "moveit_resources-ros2"
        / "fruit_arm_moveit_config"
        / "config"
        / "fruit_picking_arm_plan_execute.rviz"
    )
    data = yaml.safe_load(config.read_text(encoding="utf-8"))
    displays = {item["Name"]: item for item in data["Visualization Manager"]["Displays"]}
    motion = displays["MotionPlanning"]

    assert motion["Enabled"] is True
    assert displays["TF"]["Enabled"] is False
    assert displays["TF"]["Show Names"] is False
    assert displays["Current Real Robot (joint feedback)"]["Enabled"] is True
    assert motion["Planning Request"]["Planning Group"] == "arm"
    assert motion["Planning Request"]["Goal State Alpha"] == 0.35
    assert motion["Velocity_Scaling_Factor"] == 0.30
    assert motion["Acceleration_Scaling_Factor"] == 0.24
    assert motion["Planning Scene Topic"] == "/monitored_planning_scene"
    assert motion["MoveIt_Allow_External_Program"] is True
    assert (
        displays["Fruit Plan Markers"]["Topic"]["Value"]
        == "/perception/fruit_plan_markers"
    )
    assert displays["Perception Markers"]["Enabled"] is False


def test_real_plan_execute_script_cannot_start_the_automatic_pick_task():
    script_path = (
        Path(__file__).resolve().parents[3]
        / "scripts"
        / "single_arm"
        / "start_architecture_a_real_plan_execute.sh"
    )
    script = script_path.read_text(encoding="utf-8")

    assert '"start_robot_stack:=true"' in script
    assert '"run_task:=false"' in script
    assert '"start_rviz:=true"' in script
    assert '"rviz_config:=fruit_picking_arm_plan_execute.rviz"' in script


def test_fruit_assisted_rviz_script_requires_native_plan_then_execute():
    script_path = (
        Path(__file__).resolve().parents[3]
        / "scripts"
        / "single_arm"
        / "start_architecture_a_real_fruit_plan_execute.sh"
    )
    script = script_path.read_text(encoding="utf-8")
    common = (script_path.parent / "common.sh").read_text(encoding="utf-8")

    assert '"start_robot_stack:=true"' in script
    assert '"start_perception:=true"' in script
    assert '"start_fruit_goal_bridge:=true"' in script
    assert '"perception_output_frame:=world"' in script
    assert "configure_validated_field_perception" in script
    assert "d455_apple_detector_v1.pt" in common
    assert 'FRUIT_ARM_DETECTOR_CONFIDENCE:-0.25' in common
    assert 'FRUIT_ARM_STABLE_DETECTION_CONFIDENCE:-0.10' in common
    assert 'FRUIT_ARM_STABLE_MIN_FRAMES:-5' in common
    assert "sha256sum --check --status" in common
    assert 'FRUIT_ARM_DETECTION_ROI_MIN_Z:--0.10' in script
    assert 'FRUIT_ARM_DETECTION_ROI_MAX_Z:-1.20' in script
    assert '"run_task:=false"' in script
    assert '"run_task:=true"' not in script
    assert "request_light_vision_reset" in script
    assert "wait_for_real_robot_control_ready" in script
    assert "after the final HOME Execute" in script
    assert "reset_vision.sh" in script


def test_manual_validation_is_rviz_only_and_production_uses_the_behavior_tree():
    scripts = Path(__file__).resolve().parents[3] / "scripts" / "single_arm"
    manual = (scripts / "start_architecture_a_real_pick_task_test.sh").read_text(
        encoding="utf-8"
    )
    automatic = (scripts / "start_architecture_a_real_run.sh").read_text(
        encoding="utf-8"
    )

    assert "start_architecture_a_real_fruit_plan_execute.sh" in manual
    assert "native RViz Plan and Execute" in manual
    assert "dashboard" in manual
    assert '"start_robot_stack:=true"' in automatic
    assert '"start_perception:=true"' in automatic
    assert '"enable_table_perception:=true"' in automatic
    assert '"start_fruit_goal_bridge:=false"' in automatic
    assert '"run_task:=true"' in automatic
    assert "configure_validated_field_perception" in automatic
    assert '"stable_min_frames:=${STABLE_MIN_FRAMES}"' in automatic
    assert '"start_rviz:=true"' in automatic
    assert '"start_debug_view:=true"' in automatic
    assert '"require_auto_start_signal:=true"' in automatic
    assert '"continuous_auto_task:=true"' in automatic
    assert 'start_launch "a_real_auto"' in automatic
    assert "request_auto_task_start" in automatic
    assert "require_real_robot_control_ready" in automatic
    assert "request_light_vision_reset" in automatic
    assert "automatically after each completed HOME return" in automatic
    assert 'FRUIT_ARM_DETECTION_ROI_MIN_Z:--0.10' in automatic
    assert 'FRUIT_ARM_DETECTION_ROI_MAX_Z:-1.20' in automatic

    fruit_debug = (
        scripts / "start_architecture_a_real_fruit_plan_execute.sh"
    ).read_text(encoding="utf-8")
    assert 'start_launch "a_real_fruit_debug"' in fruit_debug


def test_every_physical_pick_place_stage_is_explicit_in_the_shared_tree():
    tree_path = (
        Path(__file__).resolve().parents[1]
        / "fruit_picking_arm"
        / "behavior"
        / "trees"
        / "pick_place_task.xml"
    )
    tree = tree_path.read_text(encoding="utf-8")
    for stage in (
        'name="ExecApproach"',
        'name="OpenGripperBeforeDescent"',
        'name="ExecGrasp"',
        'name="CloseGripper"',
        'name="ExecLift"',
        'name="ExecPlaceHover"',
        'name="ExecPlaceDrop"',
        'name="OpenGripper"',
        'name="ExecPlaceRetract"',
        'name="ExecRetreat"',
    ):
        assert stage in tree

    runner = (
        Path(__file__).resolve().parents[1]
        / "fruit_picking_arm"
        / "behavior"
        / "bt_runner.py"
    ).read_text(encoding="utf-8")
    assert '"fruit_pregrasp_target"' in runner
    assert '"fruit_place_release_target"' in runner

    import xml.etree.ElementTree as ET
    root = ET.parse(tree_path).getroot()
    setup = root.find("./BehaviorTree[@ID='SetupTree']/Sequence")
    assert setup is not None
    assert [child.tag for child in setup] == ["SetupScene", "WaitServices"]
    sense = root.find("./BehaviorTree[@ID='SenseTree']/Sequence")
    assert sense is not None
    assert [child.tag for child in sense] == ["RetryUntilSuccessful"]

    assert not (
        Path(__file__).resolve().parents[1] / "scripts" / "pick_task_dashboard"
    ).exists()
    assert not (
        Path(__file__).resolve().parents[1]
        / "fruit_picking_arm"
        / "behavior"
        / "operator_approval.py"
    ).exists()


def test_real_and_sim_scene_profiles_are_explicitly_separated():
    package_root = Path(__file__).resolve().parents[1]
    config_dir = package_root / "config"
    real = yaml.safe_load(
        (config_dir / "scene_params_real.yaml").read_text(encoding="utf-8")
    )
    simulation = yaml.safe_load(
        (config_dir / "scene_params_sim.yaml").read_text(encoding="utf-8")
    )

    assert not (config_dir / "scene_params.yaml").exists()
    assert real["profile"] == "real"
    assert real["workcell_surveyed"] is True
    assert real["objects"] == []
    assert real["table"] == {}
    assert set(real["bins"]) == {"healthy", "unhealthy"}
    assert real["bins"]["healthy"]["center"] == {
        "x": 0.05, "y": 0.45, "z": 0.115,
    }
    assert real["bins"]["healthy"]["size"] == {
        "x": 0.40, "y": 0.20, "z": 0.23,
    }
    assert real["bins"]["unhealthy"]["center"] == {
        "x": 0.06, "y": -0.73, "z": 0.11,
    }
    assert real["bins"]["unhealthy"]["size"] == {
        "x": 0.34, "y": 0.25, "z": 0.22,
    }
    # Real HOME is the electrical controller's custom-control initial pose.
    # It must remain independent of the simulation's mid-range seed.
    assert real["home_pose"] == [0.0, 0.2, -0.2, 0.0, 0.0, 0.0]
    assert simulation["profile"] == "simulation"
    assert len(simulation["objects"]) == 4
    assert simulation["home_pose"] == [
        0.0, 1.265363708, -1.570796327, 0.0, 0.0, 0.0,
    ]
    assert real["home_pose"] != simulation["home_pose"]

    real_launch = (
        package_root / "launch" / "architecture_a_moveit_serial.launch.py"
    ).read_text(encoding="utf-8")
    sim_launch = (
        package_root / "launch" / "sim_gazebo.launch.py"
    ).read_text(encoding="utf-8")
    assert '"scene_params_real.yaml"' in real_launch
    assert '"scene_params_sim.yaml"' in sim_launch


def test_fruit_goal_bridge_keeps_arm_motion_operator_gated():
    source_path = (
        Path(__file__).resolve().parents[1]
        / "fruit_picking_arm"
        / "perception"
        / "fruit_rviz_goal_bridge.py"
    )
    source = source_path.read_text(encoding="utf-8")

    assert '"/rviz/moveit/update_custom_goal_state"' in source
    assert '"/apply_planning_scene"' in source
    assert '"/compute_ik"' in source
    assert '"require_perceived_table"' in source
    assert '"/perception/table_surface"' in source
    assert '"top_down_yaw"' in source
    assert '"+X FRONT / J1=0"' in source
    assert '"+Y LEFT"' in source
    assert '"+Z UP"' in source
    assert "top_down_quaternion" in source
    assert "symmetric_yaw_candidates" in source
    assert "no RViz goal was" in source
    assert '"fruit_task_target"' in source
    assert '"pregrasp"' in source
    assert '"place_retract"' in source
    assert '"retreat_home"' in source
    # The helper observes this status topic but has no FollowJointTrajectory
    # action client, so only RViz Execute can send arm motion.
    assert '"/arm_controller/follow_joint_trajectory/_action/status"' in source
    assert "FollowJointTrajectory" not in source
    assert "ActionClient" in source  # binary GripperCommand only
    assert 'GripperCommand, "/gripper_controller/gripper_cmd"' in source
    assert 'self._send_gripper("pregrasp_open")' in source
    assert "rviz_goal.joint_state.header.stamp.sec = 0" in source
    assert "rviz_goal.joint_state.header.stamp.nanosec = 0" in source
    assert 'if operation == "pregrasp_open"' in source
    assert "FRUIT_RVIZ_QUEUE" in source
    assert "remove released fruit before collision-free retract" in source


def test_multi_fruit_selection_keeps_current_track_then_chooses_next():
    from fruit_picking_arm.perception.fruit_rviz_goal_bridge import (
        select_task_target,
    )

    fruits = [
        {"source_id": "fruit_a", "confidence": 0.70, "x": 0.5, "y": 0.1},
        {"source_id": "fruit_b", "confidence": 0.95, "x": 0.6, "y": -0.1},
    ]
    assert select_task_target(fruits)["source_id"] == "fruit_b"
    # A later score inversion must not make the pregrasp jump to fruit_a.
    fruits[0]["confidence"] = 0.99
    assert select_task_target(fruits, "fruit_b")["source_id"] == "fruit_b"
    # After fruit_b has been completed/filtered, fruit_a becomes the next task.
    assert select_task_target([fruits[0]], "fruit_b")["source_id"] == "fruit_a"


def test_grasp_stage_tries_all_eight_vertical_wrist_yaws():
    from types import SimpleNamespace
    from fruit_picking_arm.perception.fruit_rviz_goal_bridge import (
        FruitRvizGoalBridge,
    )

    bridge = object.__new__(FruitRvizGoalBridge)
    bridge._selected = {"x": 0.5, "y": 0.1, "z": 0.06}
    bridge._stage = "grasp"
    bridge._goal_yaw = 3.141592654
    bridge._pose = lambda x, y, z, yaw: SimpleNamespace(
        x=x, y=y, z=z, yaw=yaw
    )

    bridge._build_ik_candidates()

    assert len(bridge._ik_candidates) == 8
    assert len({round(item["yaw"], 9) for item in bridge._ik_candidates}) == 8


def test_unreachable_fruit_exclusion_is_geometric_not_ephemeral_track_id():
    from fruit_picking_arm.perception.fruit_rviz_goal_bridge import (
        FruitRvizGoalBridge,
    )

    bridge = object.__new__(FruitRvizGoalBridge)
    bridge._unreachable_target_exclusion_radius = 0.06
    bridge._unreachable_positions = [(0.50, 0.10, 0.06)]

    assert bridge._target_is_unreachable(0.51, 0.11, 0.06)
    assert not bridge._target_is_unreachable(0.58, 0.10, 0.06)


def test_fruit_goal_bridge_carves_fixed_base_out_of_perceived_table():
    from fruit_picking_arm.perception.fruit_rviz_goal_bridge import (
        FruitRvizGoalBridge,
    )

    bridge = object.__new__(FruitRvizGoalBridge)
    bridge._table_base_cutout = (-0.22, 0.22, -0.22, 0.22)
    rectangles = bridge._subtract_base_cutout(-0.24, 0.49, 1.129, 0.722)

    assert len(rectangles) == 3
    for center_x, center_y, size_x, size_y in rectangles:
        min_x, max_x = center_x - size_x / 2.0, center_x + size_x / 2.0
        min_y, max_y = center_y - size_y / 2.0, center_y + size_y / 2.0
        epsilon = 1.0e-9
        overlaps_cutout = (
            min_x < 0.22 - epsilon
            and max_x > -0.22 + epsilon
            and min_y < 0.22 - epsilon
            and max_y > -0.22 + epsilon
        )
        assert not overlaps_cutout


def test_fruit_goal_hysteresis_ignores_small_jitter_but_accepts_real_motion():
    from fruit_picking_arm.perception.fruit_rviz_goal_bridge import (
        target_geometry_changed,
    )

    previous = {
        "source_id": "fruit_track_00001",
        "x": 0.10,
        "y": 0.20,
        "z": 0.30,
        "radius": 0.04,
    }
    jitter = dict(previous, x=0.102, y=0.198, z=0.301)
    moved = dict(previous, x=0.106)
    replacement = dict(previous, source_id="fruit_track_00002")

    assert not target_geometry_changed(previous, jitter, 0.005)
    assert target_geometry_changed(previous, moved, 0.005)
    assert target_geometry_changed(previous, replacement, 0.005)
