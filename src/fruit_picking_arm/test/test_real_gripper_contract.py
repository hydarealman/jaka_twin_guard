from pathlib import Path

import yaml

from fruit_picking_arm.control.gripper_controller import GripperController


REPO = Path(__file__).resolve().parents[3]
APP = REPO / "src" / "fruit_picking_arm"
MOVEIT = REPO / "src" / "moveit_resources-ros2" / "fruit_arm_moveit_config"


def load(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_real_moveit_splits_arm_and_binary_gripper_controllers():
    real = load(MOVEIT / "config" / "moveit_controllers_real.yaml")
    manager = real["moveit_simple_controller_manager"]
    assert manager["controller_names"] == ["arm_controller", "gripper_controller"]
    assert manager["arm_controller"]["type"] == "FollowJointTrajectory"
    assert manager["arm_controller"]["joints"] == [f"joint_{i}" for i in range(1, 7)]
    assert manager["gripper_controller"]["type"] == "GripperCommand"
    assert manager["gripper_controller"]["action_ns"] == "gripper_cmd"
    assert manager["gripper_controller"]["command_joint"] == "left_finger_joint"


def test_simulation_controller_configuration_is_still_combined():
    simulation = load(MOVEIT / "config" / "moveit_controllers.yaml")
    arm = simulation["moveit_simple_controller_manager"]["arm_controller"]
    assert arm["type"] == "FollowJointTrajectory"
    assert arm["joints"][-2:] == ["left_finger_joint", "right_finger_joint"]


def test_real_serial_gripper_accepts_only_confirmed_endpoints():
    params = load(APP / "config" / "architecture_a_serial.yaml")[
        "serial_trajectory_controller"
    ]["ros__parameters"]
    gripper = load(APP / "config" / "gripper_params.yaml")
    assert params["gripper_action_name"] == "/gripper_controller/gripper_cmd"
    assert params["gripper_endpoint_tolerance_m"] == 0.002
    assert gripper["open"] == [0.056, -0.056]
    assert gripper["closed"] == [0.0, 0.0]


def test_real_launch_uses_real_controller_map():
    launch = (APP / "launch" / "architecture_a_moveit_serial.launch.py").read_text(
        encoding="utf-8"
    )
    assert "moveit_controllers_real.yaml" in launch


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class _Node:
    def get_logger(self):
        return _Logger()


class _Planner:
    def __init__(self):
        self.binary = []
        self.simulated = []

    def send_binary_gripper_command(self, position, effort):
        self.binary.append((position, effort))
        return True

    def send_gripper_command(self, positions, duration):
        self.simulated.append((positions, duration))
        return True


def test_real_gripper_wrapper_routes_only_binary_endpoints():
    planner = _Planner()
    config = {
        "open": [0.056, -0.056],
        "closed": [0.0, 0.0],
        "max_effort": 50.0,
    }
    gripper = GripperController(_Node(), planner, config, real_mode=True)
    assert gripper.open()
    assert gripper.close_for_radius(0.03)
    assert not gripper.move([0.028, -0.028])
    assert planner.binary == [(0.056, 50.0), (0.0, 50.0)]
    assert planner.simulated == []


def test_simulation_gripper_keeps_existing_trajectory_path():
    planner = _Planner()
    config = {"open": [0.056, -0.056], "closed": [0.0, 0.0]}
    gripper = GripperController(_Node(), planner, config, real_mode=False)
    assert gripper.open()
    assert planner.simulated == [([0.056, -0.056], 1.0)]
    assert planner.binary == []
