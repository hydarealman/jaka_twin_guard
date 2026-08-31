"""Shared pose contract for vertical four-finger fruit manipulation."""

from __future__ import annotations

import math

from geometry_msgs.msg import Quaternion


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def top_down_quaternion(yaw: float) -> Quaternion:
    """Return Rz(yaw)*Rx(pi): gripper TCP local +Z points world -Z."""
    half = 0.5 * float(yaw)
    q = Quaternion()
    q.x = math.cos(half)
    q.y = math.sin(half)
    q.z = 0.0
    q.w = 0.0
    return q


def symmetric_yaw_candidates(preferred_yaw: float) -> list[float]:
    """Try wrist rotations without changing the vertical approach axis."""
    offsets = (0.0, math.pi / 2.0, -math.pi / 2.0, math.pi,
               math.pi / 4.0, -math.pi / 4.0, 3.0 * math.pi / 4.0,
               -3.0 * math.pi / 4.0)
    result: list[float] = []
    for offset in offsets:
        candidate = normalize_angle(preferred_yaw + offset)
        if not any(abs(normalize_angle(candidate - item)) < 1.0e-9 for item in result):
            result.append(candidate)
    return result


def pregrasp_tcp_z(
    fruit_center_z: float,
    fruit_radius: float,
    finger_tip_beyond_tcp: float,
    clearance_above_fruit: float,
) -> float:
    """Keep the CAD finger tips above the fruit top by the requested clearance."""
    return (
        float(fruit_center_z)
        + max(0.0, float(fruit_radius))
        + max(0.0, float(finger_tip_beyond_tcp))
        + max(0.0, float(clearance_above_fruit))
    )
