"""Timestamp-driven constant-velocity Kalman filter for 3-D fruit motion."""

from __future__ import annotations

import numpy as np


class ConstantVelocityKalman3D:
    """Linear KF with Cartesian position measurements and innovation gating."""

    def __init__(
        self,
        measurement_std_m: float = 0.008,
        acceleration_std_mps2: float = 1.5,
        gate_sigma: float = 4.0,
    ) -> None:
        self.measurement_variance = max(1.0e-8, float(measurement_std_m) ** 2)
        self.acceleration_variance = max(
            1.0e-8, float(acceleration_std_mps2) ** 2
        )
        self.gate_squared = max(1.0, float(gate_sigma) ** 2)
        self.initialized = False
        self.timestamp = 0.0
        self.state = np.zeros(6, dtype=np.float64)
        self.covariance = np.eye(6, dtype=np.float64)

    def initialize(self, position, timestamp: float) -> None:
        self.state.fill(0.0)
        self.state[:3] = np.asarray(position, dtype=np.float64)
        self.covariance = np.diag(
            [self.measurement_variance] * 3 + [1.0] * 3
        ).astype(np.float64)
        self.timestamp = float(timestamp)
        self.initialized = True

    def projected(self, timestamp: float):
        if not self.initialized:
            return self.state.copy(), self.covariance.copy()
        dt = max(0.0, float(timestamp) - self.timestamp)
        transition, process_noise = self._model(dt)
        return (
            transition @ self.state,
            transition @ self.covariance @ transition.T + process_noise,
        )

    def update(self, position, timestamp: float) -> bool:
        measurement = np.asarray(position, dtype=np.float64)
        if not self.initialized:
            self.initialize(measurement, timestamp)
            return True
        timestamp = float(timestamp)
        if timestamp + 1.0e-9 < self.timestamp:
            return False
        predicted_state, predicted_covariance = self.projected(timestamp)
        observation = np.zeros((3, 6), dtype=np.float64)
        observation[:, :3] = np.eye(3)
        measurement_noise = np.eye(3) * self.measurement_variance
        innovation = measurement - observation @ predicted_state
        innovation_covariance = (
            observation @ predicted_covariance @ observation.T + measurement_noise
        )
        mahalanobis_squared = float(
            innovation.T @ np.linalg.solve(innovation_covariance, innovation)
        )
        if mahalanobis_squared > self.gate_squared:
            return False
        gain = (
            predicted_covariance
            @ observation.T
            @ np.linalg.inv(innovation_covariance)
        )
        self.state = predicted_state + gain @ innovation
        identity = np.eye(6)
        residual = identity - gain @ observation
        # Joseph form keeps covariance symmetric and positive semi-definite.
        self.covariance = (
            residual @ predicted_covariance @ residual.T
            + gain @ measurement_noise @ gain.T
        )
        self.timestamp = timestamp
        return True

    @property
    def position(self) -> tuple[float, float, float]:
        return tuple(float(value) for value in self.state[:3])

    @property
    def velocity(self) -> tuple[float, float, float]:
        return tuple(float(value) for value in self.state[3:])

    @property
    def position_std(self) -> float:
        return float(np.sqrt(max(0.0, np.max(np.diag(self.covariance)[:3]))))

    def _model(self, dt: float):
        transition = np.eye(6, dtype=np.float64)
        transition[0, 3] = transition[1, 4] = transition[2, 5] = dt
        process_noise = np.zeros((6, 6), dtype=np.float64)
        q11 = 0.25 * dt ** 4 * self.acceleration_variance
        q12 = 0.5 * dt ** 3 * self.acceleration_variance
        q22 = dt ** 2 * self.acceleration_variance
        for position_index, velocity_index in ((0, 3), (1, 4), (2, 5)):
            process_noise[position_index, position_index] = q11
            process_noise[position_index, velocity_index] = q12
            process_noise[velocity_index, position_index] = q12
            process_noise[velocity_index, velocity_index] = q22
        return transition, process_noise
