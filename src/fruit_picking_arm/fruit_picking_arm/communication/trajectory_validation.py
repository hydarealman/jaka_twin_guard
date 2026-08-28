"""ROS-free validation shared by the serial trajectory adapter and tests."""

from __future__ import annotations

import math


DEFAULT_JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
    "left_finger_joint", "right_finger_joint",
]

DEFAULT_LOWER_LIMITS = [
    -1.483529864, 0.087266463, -3.054326191,
    -2.792526803, -1.483529864, -3.054326191,
    0.0, -0.056,
]

DEFAULT_UPPER_LIMITS = [
    1.483529864, 2.443460952, -0.087266463,
    2.792526803, 1.483529864, 3.054326191,
    0.056, 0.0,
]


def validate_trajectory_positions(trajectory, limits: dict[str, tuple[float, float]]) -> str | None:
    """Return a rejection reason for values that can reach the wire encoder."""
    for point_index, point in enumerate(trajectory.points):
        if len(point.positions) != len(trajectory.joint_names):
            return f"point {point_index}: position count does not match joint_names"
        velocities = getattr(point, "velocities", ())
        if len(velocities) != len(trajectory.joint_names):
            return f"point {point_index}: velocity count does not match joint_names"
        for name, value in zip(trajectory.joint_names, point.positions):
            if not math.isfinite(value):
                return f"point {point_index}: {name} position is not finite"
            lower, upper = limits[name]
            if value < lower or value > upper:
                return (
                    f"point {point_index}: {name}={value:.6f} outside "
                    f"software limits [{lower:.6f}, {upper:.6f}]"
                )
        for value in velocities:
            if not math.isfinite(value):
                return f"point {point_index}: velocity is not finite"
    return None
