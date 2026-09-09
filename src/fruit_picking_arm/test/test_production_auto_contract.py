import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import yaml

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from fruit_picking_arm.behavior.bt_runner import PickPlaceRunner
from fruit_picking_arm.planner.planner_server import SingleArmPlannerServer
from fruit_picking_arm.perception.fruit_target_node import FruitTargetNode
from fruit_picking_arm.perception.fruit_rviz_goal_bridge import FruitRvizGoalBridge
from fruit_picking_arm.scene.scene_manager import SceneManager
from fruit_picking_arm.skills.approach import ApproachSkill
from fruit_picking_arm.skills.grasp import GraspSkill


REPO = Path(__file__).resolve().parents[3]
APP = REPO / "src" / "fruit_picking_arm"


def test_debug_empty_reset_does_not_poison_camera_timestamp():
    from vision_msgs.msg import Detection3DArray
    calls = []
    bridge = SimpleNamespace(
        _vision_reset_pending=False, _task_active=False,
        _last_target_stamp_s=100.0,
        _invalidate_goal=lambda: calls.append("invalidated"),
    )
    empty = Detection3DArray()
    empty.header.stamp.sec = 1900000000
    FruitRvizGoalBridge._on_targets(bridge, empty)
    assert calls == ["invalidated"]
    assert bridge._last_target_stamp_s == 100.0


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass

    def warning(self, _message):
        pass


class _Clock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time())


class _Node:
    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return _Clock()


def test_automatic_approach_reuses_all_eight_validated_yaw_candidates():
    successful = JointTrajectory()

    class Planner:
        def __init__(self):
            self.calls = []

        def plan_pose_target(self, pose, cartesian=False):
            self.calls.append((pose, cartesian))
            return successful if len(self.calls) == 8 else None

    planner = Planner()
    blackboard = {
        "target_object": SimpleNamespace(
            centroid=(0.5, 0.0, 0.35), radius=0.04
        ),
        "simulation_mode": False,
    }
    skill = ApproachSkill()
    skill.configure(
        _Node(),
        planner,
        {
            "top_down_yaw": 3.141592654,
            "finger_tip_beyond_tcp": 0.037,
            "clearance_above_fruit": 0.050,
            "cartesian": False,
        },
        blackboard,
    )

    assert skill.plan() is successful
    assert len(planner.calls) == 8
    assert all(cartesian is False for _pose, cartesian in planner.calls)
    assert "top_down_yaw" in blackboard


def test_automatic_grasp_reuses_the_same_eight_rviz_yaw_candidates():
    successful = JointTrajectory()

    class Planner:
        def __init__(self):
            self.calls = []

        def plan_pose_target(self, pose, cartesian=False):
            self.calls.append((pose, cartesian))
            return successful if len(self.calls) == 8 else None

    planner = Planner()
    blackboard = {
        "target_object": SimpleNamespace(
            centroid=(0.5, 0.0, 0.35), radius=0.04
        ),
        "top_down_yaw": 3.141592654,
        "simulation_mode": False,
    }
    skill = GraspSkill()
    skill.configure(
        _Node(), planner, {"cartesian": True}, blackboard
    )

    assert skill.plan() is successful
    assert len(planner.calls) == 8
    assert all(cartesian is True for _pose, cartesian in planner.calls)


def test_automatic_pose_planning_matches_validated_rviz_joint_target_workflow():
    harness = SimpleNamespace(
        solve_ik=lambda _pose: [0.1] * 6,
        plan_joint_target=lambda joints: ("planned", joints),
        _logger=_Logger(),
    )
    assert SingleArmPlannerServer.plan_pose_target(
        harness, PoseStamped(), cartesian=True
    ) == ("planned", [0.1] * 6)


def test_real_execution_keeps_arm_only_and_preserves_velocities():
    class Future:
        def __init__(self, value):
            self._value = value

        def result(self):
            return self._value

    class Handle:
        accepted = True

        def get_result_async(self):
            result = SimpleNamespace(
                error_code=FollowJointTrajectory.Result.SUCCESSFUL,
                error_string="",
            )
            return Future(SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED, result=result
            ))

    class Client:
        def __init__(self):
            self.goal = None

        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, goal):
            self.goal = goal
            return Future(Handle())

    client = Client()
    harness = SimpleNamespace(
        _arm_client=client,
        _real_mode=True,
        _arm_joints=[f"joint_{index}" for index in range(1, 7)],
        _gripper_joints=["left_finger_joint", "right_finger_joint"],
        _all_joints=[f"joint_{index}" for index in range(1, 7)]
        + ["left_finger_joint", "right_finger_joint"],
        _goal_time_tol=0.5,
        _exec_timeout=5.0,
        _logger=_Logger(),
        _spin_both=lambda _future, timeout_sec: None,
        _set_active_goal=lambda _handle: None,
        _clear_active_goal=lambda _handle: None,
        _send_guarded_goal=lambda client, goal, timeout: client.send_goal_async(goal).result(),
        _await_result=lambda handle, timeout: handle.get_result_async().result(),
        wait_until_stopped=lambda: True,
        get_current_gripper_positions=lambda: [0.056, -0.056],
    )
    trajectory = JointTrajectory()
    trajectory.joint_names = list(harness._arm_joints)
    point = JointTrajectoryPoint()
    point.positions = [0.1] * 6
    point.velocities = [0.2] * 6
    point.time_from_start.sec = 1
    trajectory.points = [point]

    assert SingleArmPlannerServer.execute(harness, trajectory)
    sent = client.goal.trajectory
    assert sent.joint_names == harness._arm_joints
    assert list(sent.points[0].velocities) == [0.2] * 6


def test_continuous_runner_selects_exactly_one_highest_confidence_target():
    objects = [
        SimpleNamespace(id="low", health="healthy", confidence=0.60),
        SimpleNamespace(id="unknown", health="unknown", confidence=0.99),
        SimpleNamespace(id="high", health="unhealthy", confidence=0.90),
    ]
    selected = PickPlaceRunner._select_one_target(objects)
    assert selected.id == "high"


def test_external_freshness_uses_raw_rgbd_not_static_table_publish_rate():
    now = time.monotonic()
    runner = SimpleNamespace(
        _real_mode=True,
        _use_external_perception=True,
        _external_target_arrival=now,
        _external_table_arrival=now - 30.0,
        _external_rgb_arrival=now,
        _external_depth_arrival=now,
        _perceived_table={"top_z": 0.0},
        _data_timeout_s=1.0,
    )
    runner._camera_stream_is_fresh = lambda sample_now=None: (
        PickPlaceRunner._camera_stream_is_fresh(runner, sample_now)
    )

    assert PickPlaceRunner._perception_is_fresh(runner)
    runner._external_rgb_arrival = now - 2.0
    assert not PickPlaceRunner._perception_is_fresh(runner)


def test_motion_watchdog_does_not_require_continuous_target_updates():
    class Planner:
        cancelled = False

        def cancel_active_goal(self):
            self.cancelled = True

    planner = Planner()
    runner = SimpleNamespace(
        _real_mode=True,
        _perception_watchdog_enabled=True,
        _perception_fault=False,
        _camera_stream_is_fresh=lambda: True,
        _planner=planner,
        _safety=SimpleNamespace(check=lambda: None),
    )

    PickPlaceRunner._perception_watchdog(runner)
    assert not runner._perception_fault
    assert not planner.cancelled


def test_external_freshness_diagnostic_identifies_missing_heartbeat():
    now = time.monotonic()
    runner = SimpleNamespace(
        _use_external_perception=True,
        _external_target_arrival=now,
        _external_rgb_arrival=0.0,
        _external_depth_arrival=now,
        _perceived_table={"top_z": 0.0},
        _data_timeout_s=1.0,
    )
    runner._arrival_age = PickPlaceRunner._arrival_age

    detail = PickPlaceRunner._external_freshness_diagnostic(runner)
    assert "rgb_age=infs" in detail
    assert "depth_age=" in detail
    assert "table=ready" in detail


def test_runner_drains_a_bounded_callback_burst_before_decision():
    class Executor:
        calls = 0

        def spin_once(self, timeout_sec):
            assert timeout_sec == 0.0
            self.calls += 1

    executor = Executor()
    PickPlaceRunner._drain_ready_callbacks(executor, max_callbacks=7)
    assert executor.calls == 7


def test_auto_attach_matches_manual_bridge_after_grasp_collision_removal():
    captured = {}
    manager = object.__new__(SceneManager)
    manager._world_frame = "world"
    manager._detected_objects = {
        "fruit_01": (
            "perceived_fruit_01",
            {"id": "perceived_fruit_01", "radius": 0.035},
        )
    }

    def apply_scene(scene, description):
        captured["scene"] = scene
        captured["description"] = description
        return True

    manager._apply_scene = apply_scene
    assert manager.attach_detected_object(SimpleNamespace(id="fruit_01"))

    scene = captured["scene"]
    assert not scene.world.collision_objects
    assert len(scene.robot_state.attached_collision_objects) == 1
    attached = scene.robot_state.attached_collision_objects[0]
    assert attached.object.id == "perceived_fruit_01"
    assert attached.link_name == "gripper_tcp"


def test_light_vision_reset_clears_trackers_and_markers_but_keeps_table():
    calls = []

    class Publisher:
        def publish(self, _message):
            calls.append("projection")

    class Event:
        def set(self):
            calls.append("wake")

    table = object()
    harness = SimpleNamespace(
        _vision_reset_lock=threading.Lock(),
        _invalidate_health_jobs=lambda: calls.append("invalidate_health"),
        _reset_tracking=lambda: calls.append("reset_trackers"),
        _detector=SimpleNamespace(
            clear_markers=lambda: calls.append("clear_markers")
        ),
        _publish_empty=lambda: calls.append("publish_empty"),
        _kf_projection_pub=Publisher(),
        _vision_reset_generation=0,
        _last_cloud_stamp=(100, 0),
        _last_rgbd_stamp=(100, 1),
        _process_event=Event(),
        _table_estimator=table,
        get_clock=lambda: _Clock(),
        get_logger=lambda: _Logger(),
    )
    response = SimpleNamespace(success=False, message="")

    returned = FruitTargetNode._on_reset_vision(harness, None, response)

    assert returned is response
    assert response.success
    assert "fresh 5-frame target" in response.message
    assert calls == [
        "invalidate_health",
        "reset_trackers",
        "clear_markers",
        "publish_empty",
        "projection",
        "wake",
    ]
    assert harness._table_estimator is table
    assert harness._last_cloud_stamp == (100, 0)
    assert harness._last_rgbd_stamp == (100, 1)


def test_auto_light_reset_clears_scene_and_gates_on_new_detection_sequence():
    class Future:
        def done(self):
            return True

        def result(self):
            return SimpleNamespace(success=True, message="reset")

    class Client:
        def service_is_ready(self):
            return True

        def call_async(self, _request):
            return Future()

    scene = SimpleNamespace(cleared=False)

    def clear_scene():
        scene.cleared = True
        return True

    blackboard = {
        "external_detection_sequence": 17,
        "external_detected_objects": [object()],
        "detected_objects": [object()],
        "detection_count": 1,
    }
    harness = SimpleNamespace(
        _use_external_perception=True,
        _task_active=False,
        _scene_mgr=SimpleNamespace(clear_detected_objects=clear_scene),
        _vision_reset_client=Client(),
        _drain_ready_callbacks=lambda _executor: None,
        _external_objects=[object()],
        _blackboard=blackboard,
        get_logger=lambda: _Logger(),
    )

    assert PickPlaceRunner._light_vision_reset(
        harness,
        SimpleNamespace(spin_once=lambda timeout_sec: None),
        0.01,
        completed_target_id="fruit_01",
    )
    assert scene.cleared
    assert harness._external_objects == []
    assert blackboard["external_detected_objects"] == []
    assert blackboard["minimum_external_detection_sequence"] == 17


def test_scene_light_reset_removes_only_detected_fruit_collisions():
    captured = {}

    def apply_scene(scene, description):
        captured["scene"] = scene
        captured["description"] = description
        return True

    manager = SimpleNamespace(
        _world_frame="world",
        _detected_objects={
            "fruit_a": ("perceived_fruit_a", {"radius": 0.03}),
            "fruit_b": ("perceived_fruit_b", {"radius": 0.04}),
        },
        _apply_scene=apply_scene,
        get_logger=lambda: _Logger(),
    )
    assert SceneManager.clear_detected_objects(manager)
    assert manager._detected_objects == {}
    assert captured["description"] == "clear perceived fruit snapshot"
    assert {
        item.id for item in captured["scene"].world.collision_objects
    } == {"perceived_fruit_a", "perceived_fruit_b"}


def test_real_launch_exposes_explicit_start_and_continuous_parameters():
    launch = (APP / "launch" / "architecture_a_moveit_serial.launch.py").read_text(
        encoding="utf-8"
    )
    runner = (APP / "fruit_picking_arm" / "behavior" / "bt_runner.py").read_text(
        encoding="utf-8"
    )
    bridge = (APP / "fruit_picking_arm" / "perception" / "fruit_rviz_goal_bridge.py").read_text(encoding="utf-8")
    assert '"require_auto_start_signal"' in launch
    assert '"continuous_auto_task"' in launch
    assert '"/fruit_picking/start_auto_task"' in runner
    assert '"SenseTree"' in runner
    assert 'self.declare_parameter("lift_height", 0.150)' in bridge
    assert "top_down_tilt" not in bridge


def test_automatic_motion_parameters_match_the_validated_manual_bridge():
    skill_cfg = yaml.safe_load(
        (APP / "config" / "skill_params.yaml").read_text(encoding="utf-8")
    )
    bridge = (
        APP / "fruit_picking_arm" / "perception" / "fruit_rviz_goal_bridge.py"
    ).read_text(encoding="utf-8")
    approach = (APP / "fruit_picking_arm" / "skills" / "approach.py").read_text(
        encoding="utf-8"
    )
    grasp = (APP / "fruit_picking_arm" / "skills" / "grasp.py").read_text(
        encoding="utf-8"
    )

    assert math.isclose(skill_cfg["approach"]["top_down_yaw"], math.pi)
    assert skill_cfg["approach"]["finger_tip_beyond_tcp"] == 0.037
    assert skill_cfg["approach"]["clearance_above_fruit"] == 0.050
    assert skill_cfg["lift"]["height"] == 0.15
    assert skill_cfg["place"]["approach_height"] == 0.12
    assert skill_cfg["place"]["release_clearance"] == 0.015
    assert skill_cfg["place"]["drop_wall_clearance"] == 0.010
    planner_cfg = yaml.safe_load(
        (APP / "config" / "planner_params.yaml").read_text(encoding="utf-8")
    )
    assert planner_cfg["max_velocity_scaling_factor"] == 0.30
    assert planner_cfg["max_acceleration_scaling_factor"] == 0.24
    assert 'self.declare_parameter("finger_tip_beyond_tcp", 0.037)' in bridge
    assert 'self.declare_parameter("clearance_above_fruit", 0.050)' in bridge
    assert 'self.declare_parameter("place_approach_height", 0.120)' in bridge
    assert 'self.declare_parameter("place_release_clearance", 0.015)' in bridge
    assert 'self.declare_parameter("place_wall_clearance", 0.010)' in bridge
    assert "symmetric_yaw_candidates" in approach
    assert "symmetric_yaw_candidates" in grasp


def test_automatic_cycle_keeps_the_same_frozen_target_through_arm_occlusion():
    runner = (APP / "fruit_picking_arm" / "behavior" / "bt_runner.py").read_text(
        encoding="utf-8"
    )
    nodes = (
        APP / "fruit_picking_arm" / "behavior" / "bt_nodes" / "pick_place_nodes.py"
    ).read_text(encoding="utf-8")
    bridge = (
        APP / "fruit_picking_arm" / "perception" / "fruit_rviz_goal_bridge.py"
    ).read_text(encoding="utf-8")

    # The accepted manual workflow freezes its selected target once execution
    # starts. Automatic execution must not add a visibility/re-aim gate between
    # pregrasp and descent because the eye-to-hand view is normally occluded.
    assert "if self._task_active:" in bridge
    assert "use the frozen inspected target" in bridge
    assert "validate_grasp_target" not in runner
    assert "pre_execute_check" not in nodes
    assert "PRE_DESCENT_REJECTED" not in runner


def test_real_serial_controller_filters_only_near_zero_duplicate_motion():
    serial_cfg = yaml.safe_load(
        (APP / "config" / "architecture_a_serial.yaml").read_text(
            encoding="utf-8"
        )
    )["serial_trajectory_controller"]["ros__parameters"]
    controller = (APP / "src" / "serial_trajectory_controller.cpp").read_text(
        encoding="utf-8"
    )

    assert serial_cfg["no_op_position_tolerance_rad"] == 0.0025
    assert serial_cfg["no_op_velocity_tolerance_rad_s"] == 0.01
    assert "trajectory_is_no_op" in controller
    assert "Skipping near-zero real trajectory" in controller
