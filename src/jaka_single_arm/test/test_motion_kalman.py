import numpy as np

from jaka_single_arm.perception.motion_kalman import ConstantVelocityKalman3D


def test_constant_velocity_filter_estimates_motion_from_source_timestamps():
    tracker = ConstantVelocityKalman3D(
        measurement_std_m=0.003,
        acceleration_std_mps2=0.5,
        gate_sigma=5.0,
    )
    for index in range(8):
        timestamp = index * 0.1
        position = (0.4 + 0.2 * timestamp, 0.1, 0.34)
        assert tracker.update(position, timestamp)

    assert abs(tracker.position[0] - 0.54) < 0.004
    assert abs(tracker.velocity[0] - 0.2) < 0.04


def test_mahalanobis_gate_rejects_background_jump_without_corrupting_track():
    tracker = ConstantVelocityKalman3D(
        measurement_std_m=0.003,
        acceleration_std_mps2=0.4,
        gate_sigma=4.0,
    )
    for index in range(5):
        assert tracker.update((0.5 + index * 0.005, 0.1, 0.34), index * 0.1)
    before = np.asarray(tracker.position)

    assert not tracker.update((1.2, -0.5, 2.0), 0.5)

    assert np.linalg.norm(np.asarray(tracker.position) - before) < 1.0e-12


def test_out_of_order_measurement_is_rejected():
    tracker = ConstantVelocityKalman3D()
    assert tracker.update((0.5, 0.1, 0.34), 2.0)
    assert not tracker.update((0.51, 0.1, 0.34), 1.9)
