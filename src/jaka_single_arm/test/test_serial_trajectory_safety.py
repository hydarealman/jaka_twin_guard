from types import SimpleNamespace

from jaka_single_arm.communication.serial_trajectory_controller import (
    DEFAULT_JOINT_NAMES,
    DEFAULT_LOWER_LIMITS,
    DEFAULT_UPPER_LIMITS,
    validate_trajectory_positions,
)


LIMITS = dict(zip(
    DEFAULT_JOINT_NAMES,
    zip(DEFAULT_LOWER_LIMITS, DEFAULT_UPPER_LIMITS),
))


def trajectory(names, points):
    return SimpleNamespace(
        joint_names=names,
        points=[
            SimpleNamespace(positions=positions)
            for positions in points
        ],
    )


def test_serial_gate_accepts_values_on_software_boundaries():
    candidate = trajectory(
        DEFAULT_JOINT_NAMES,
        [DEFAULT_LOWER_LIMITS, DEFAULT_UPPER_LIMITS],
    )
    assert validate_trajectory_positions(candidate, LIMITS) is None


def test_serial_gate_rejects_arm_joint_beyond_soft_limit():
    positions = [0.0] * len(DEFAULT_JOINT_NAMES)
    positions[1] = DEFAULT_UPPER_LIMITS[1] + 0.001
    reason = validate_trajectory_positions(
        trajectory(DEFAULT_JOINT_NAMES, [positions]),
        LIMITS,
    )
    assert "joint_2" in reason
    assert "outside software limits" in reason


def test_serial_gate_rejects_gripper_overtravel_and_nonfinite_values():
    overtravel = trajectory(
        ["left_finger_joint"],
        [[DEFAULT_UPPER_LIMITS[6] + 0.001]],
    )
    assert "left_finger_joint" in validate_trajectory_positions(
        overtravel, LIMITS
    )

    nonfinite = trajectory(["joint_1"], [[float("nan")]])
    assert "not finite" in validate_trajectory_positions(nonfinite, LIMITS)


def test_serial_gate_rejects_malformed_point():
    malformed = trajectory(["joint_1", "joint_2"], [[0.0]])
    assert "position count" in validate_trajectory_positions(
        malformed, LIMITS
    )
