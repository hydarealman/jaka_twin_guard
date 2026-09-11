import threading
from types import SimpleNamespace

from trajectory_msgs.msg import JointTrajectory

from fruit_picking_arm.behavior.bt_node_base import NodeStatus
from fruit_picking_arm.behavior.bt_nodes.pick_place_nodes import SetupScene
from fruit_picking_arm.communication.protocol import (
    FruitClass,
    MotionResult,
    ResultCode,
)
from fruit_picking_arm.communication.serial_fruit_target_bridge import (
    SerialFruitTargetBridge,
)
from fruit_picking_arm.control.safety_monitor import SafetyLevel, SafetyMonitor
from fruit_picking_arm.scene.scene_manager import SceneManager
from fruit_picking_arm.skills.grasp import GraspSkill
from fruit_picking_arm.skills.place import PlaceSkill


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    warn = warning

    def error(self, _message):
        pass


class _Node:
    def __init__(self):
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


def test_setup_scene_fails_when_a_required_collision_object_fails():
    manager = SimpleNamespace(
        register_table=lambda: True,
        register_bin=lambda: False,
        register_all_objects=lambda: 0,
    )
    condition = SetupScene()
    condition.blackboard = {
        "node": _Node(),
        "scene_manager": manager,
        "scene_config": {"objects": []},
        "simulation_mode": False,
    }
    assert condition.execute() == NodeStatus.FAILURE


def test_missing_gripper_controller_is_not_assumed_successful():
    skill = GraspSkill()
    planner = SimpleNamespace(execute=lambda _trajectory: True)
    skill.configure(_Node(), planner, {}, {})
    assert not skill.execute(JointTrajectory())


def test_place_requires_exact_health_bin_without_fallback():
    target = SimpleNamespace(health="Unhealthy", radius=0.03)
    skill = PlaceSkill()
    skill.configure(
        _Node(),
        SimpleNamespace(),
        {},
        {
            "target_object": target,
            "scene_config": {
                "bins": {"healthy": {"center": {"x": 0.5, "y": 0.4}}}
            },
        },
    )
    assert skill.plan() is None


def test_perceived_collision_registration_rejects_partial_failure():
    class Harness:
        def __init__(self):
            self._detected_objects = {}
            self.calls = 0

        def get_logger(self):
            return _Logger()

        def register_object(self, _config):
            self.calls += 1
            return self.calls == 1

        def _fetch_scene_inventory(self):
            return set(), set()

        def purge_unknown_perceived_objects(self, _keep_source_ids):
            return True

    objects = [
        SimpleNamespace(id="one", centroid=(0.5, 0.0, 0.35), radius=0.03),
        SimpleNamespace(id="two", centroid=(0.6, 0.0, 0.35), radius=0.03),
    ]
    harness = Harness()
    assert not SceneManager.register_detected_objects(harness, objects)


def test_architecture_b_retries_then_dead_letters_failed_target():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message.data)

    class Link:
        def send_fruit_target(self, *_args, **_kwargs):
            return MotionResult(1, 1, ResultCode.UNREACHABLE, 42)

    class Harness:
        _send_worker = SerialFruitTargetBridge._send_worker

        def __init__(self):
            self._link = Link()
            self._max_target_attempts = 3
            self._retry_delay_s = 0.0
            self._lock = threading.Lock()
            self._completed_tracks = set()
            self._failed_tracks = set()
            self._retry_not_before = {}
            self._active_track = "track"
            self._result_pub = Publisher()

        def get_logger(self):
            return _Logger()

    target = SimpleNamespace(target_id=1, fruit_class=FruitClass.HEALTHY)
    harness = Harness()
    harness._send_worker("track", target, 1)
    assert "track" not in harness._failed_tracks
    assert "track" in harness._retry_not_before

    harness._active_track = "track"
    harness._send_worker("track", target, 3)
    assert "track" in harness._failed_tracks
    assert "track" not in harness._completed_tracks


def test_missing_joint_velocity_feedback_is_a_halt_not_zero_velocity():
    monitor = object.__new__(SafetyMonitor)
    monitor._arm_joints = ["joint_1", "joint_2"]
    monitor._joint_velocities = {"joint_1": 0.0}
    monitor._limits = SimpleNamespace(
        require_velocity_feedback=True,
        max_joint_velocity=2.0,
        velocity_halt_ratio=0.95,
        velocity_warn_ratio=0.85,
    )
    violations, level = SafetyMonitor._check_joint_velocities(monitor)
    assert level == SafetyLevel.HALT
    assert "joint_2" in violations[0]
