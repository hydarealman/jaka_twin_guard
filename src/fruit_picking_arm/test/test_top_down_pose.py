import math

import numpy as np

from fruit_picking_arm.skills.top_down_pose import (
    pregrasp_tcp_z,
    symmetric_yaw_candidates,
    top_down_quaternion,
)


def _rotate_local_z(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [2.0 * (x * z + w * y), 2.0 * (y * z - w * x),
         1.0 - 2.0 * (x * x + y * y)]
    )


def test_top_down_quaternion_points_tcp_positive_z_world_down():
    for yaw in (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0):
        direction = _rotate_local_z(top_down_quaternion(yaw))
        assert np.allclose(direction, [0.0, 0.0, -1.0], atol=1.0e-9)


def test_pregrasp_height_accounts_for_fruit_and_cad_finger_overhang():
    assert math.isclose(
        pregrasp_tcp_z(-0.135, 0.045, 0.037, 0.050), -0.003,
        abs_tol=1.0e-9,
    )


def test_four_finger_wrist_candidates_preserve_unique_yaws():
    candidates = symmetric_yaw_candidates(math.pi)
    assert len(candidates) == 8
    assert math.isclose(abs(candidates[0]), math.pi, abs_tol=1.0e-9)
