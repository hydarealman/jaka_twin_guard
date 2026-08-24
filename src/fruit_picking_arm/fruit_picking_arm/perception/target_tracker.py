"""ROS-independent multi-frame stabilisation for detected fruits."""

from __future__ import annotations

import math
import statistics
from collections import Counter, deque
from dataclasses import dataclass, field
from enum import Enum

from fruit_picking_arm.perception.motion_kalman import ConstantVelocityKalman3D


@dataclass(frozen=True)
class FruitObservation:
    x: float
    y: float
    z: float
    radius: float
    health: str
    detection_confidence: float
    health_confidence: float
    # Difference between the winning and runner-up quality scores.  A
    # classifier that does not expose a margin may leave the backwards
    # compatible default of 1.0; production backends should always provide it.
    health_margin: float = 1.0


@dataclass(frozen=True)
class StableFruitTarget:
    track_id: str
    centroid: tuple[float, float, float]
    radius: float
    health: str
    confidence: float
    position_std: float
    sample_count: int
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    predicted: bool = False
    measurement_age: float = 0.0
    phase: str = "tracking"


class TrackPhase(str, Enum):
    """Lifecycle shared by measured and short-horizon predicted targets."""

    DETECTING = "detecting"
    TRACKING = "tracking"
    COASTING = "coasting"
    LOST = "lost"


@dataclass
class _Track:
    track_id: str
    samples: deque[tuple[float, FruitObservation]] = field(default_factory=deque)
    last_seen: float = 0.0
    motion_filter: ConstantVelocityKalman3D | None = None
    missed_frames: int = 0
    phase: TrackPhase = TrackPhase.DETECTING
    confirmed: bool = False


class FruitTargetTracker:
    """Nearest-neighbour tracking followed by temporal quality gating."""

    def __init__(
        self,
        min_frames: int = 5,
        window_size: int = 7,
        association_distance: float = 0.06,
        max_position_std: float = 0.005,
        class_majority: float = 0.7,
        min_confidence: float = 0.55,
        min_detection_confidence: float | None = None,
        min_health_margin: float = 0.0,
        min_known_ratio: float = 0.8,
        stale_after: float = 0.35,
        max_missed_frames: int = 2,
        kalman_measurement_std_m: float = 0.008,
        kalman_acceleration_std_mps2: float = 1.5,
        kalman_gate_sigma: float = 4.0,
        publish_kalman_predictions: bool = False,
        max_prediction_age_s: float = 0.18,
        require_known_health: bool = True,
    ):
        self.min_frames = max(1, int(min_frames))
        self.window_size = max(self.min_frames, int(window_size))
        self.association_distance = float(association_distance)
        self.max_position_std = float(max_position_std)
        self.class_majority = float(class_majority)
        self.min_confidence = float(min_confidence)
        # Detector confidence and health-classifier confidence are different
        # quantities. Small-object/tiled detectors can have a low calibrated
        # box score while repeated depth + health observations are strong.
        # Keep the old coupled behavior by default, but allow a debug/field
        # configuration to set an independent detector floor.
        self.min_detection_confidence = (
            self.min_confidence
            if min_detection_confidence is None
            else float(min_detection_confidence)
        )
        self.min_health_margin = max(0.0, float(min_health_margin))
        self.min_known_ratio = min(1.0, max(0.0, float(min_known_ratio)))
        self.stale_after = float(stale_after)
        self.max_missed_frames = max(0, int(max_missed_frames))
        self.kalman_measurement_std_m = max(
            0.0001, float(kalman_measurement_std_m)
        )
        self.kalman_acceleration_std_mps2 = max(
            0.001, float(kalman_acceleration_std_mps2)
        )
        self.kalman_gate_sigma = max(1.0, float(kalman_gate_sigma))
        self.publish_kalman_predictions = bool(publish_kalman_predictions)
        self.require_known_health = bool(require_known_health)
        self.max_prediction_age_s = min(
            self.stale_after, max(0.0, float(max_prediction_age_s))
        )
        self._tracks: dict[str, _Track] = {}
        self._next_id = 0
        self._last_timestamp: float | None = None
        self.last_rejection_reason = "no_track"

    def reset(self) -> None:
        self._tracks.clear()
        self._last_timestamp = None

    @property
    def track_phases(self) -> dict[str, TrackPhase]:
        """Return a read-only snapshot for diagnostics and state-machine tests."""
        return {
            track_id: track.phase
            for track_id, track in self._tracks.items()
        }

    def update(
        self, observations: list[FruitObservation], timestamp: float
    ) -> list[StableFruitTarget]:
        timestamp = float(timestamp)
        if (
            not math.isfinite(timestamp)
            or timestamp < 0.0
            or (
                self._last_timestamp is not None
                and timestamp + 1.0e-9 < self._last_timestamp
            )
        ):
            # Source time is part of the motion model.  Continuing after a
            # clock jump would turn an old velocity into a plausible-looking
            # but unsafe pick coordinate, so mirror the self-aim reset rule.
            self.reset()
            self.last_rejection_reason = "invalid_or_backward_timestamp"
            return []
        self._last_timestamp = timestamp
        unmatched_tracks = set(self._tracks)
        seen_tracks: set[str] = set()

        # Associate the most confident observations first.
        ordered = sorted(
            observations,
            key=lambda o: min(o.detection_confidence, o.health_confidence),
            reverse=True,
        )
        for observation in ordered:
            match = self._nearest_track(observation, unmatched_tracks, timestamp)
            if match is None:
                match = self._new_track()
                self._update_motion_filter(match, observation, timestamp)
            else:
                matched_track = match
                if self._update_motion_filter(match, observation, timestamp):
                    unmatched_tracks.discard(match.track_id)
                else:
                    # A spatially close but statistically implausible jump is
                    # not allowed to corrupt the established motion model.
                    match = self._new_track()
                    self._update_motion_filter(match, observation, timestamp)
                    self.last_rejection_reason = (
                        "kalman_innovation_rejected=%s" % matched_track.track_id
                    )
            match.samples.append((timestamp, observation))
            while len(match.samples) > self.window_size:
                match.samples.popleft()
            match.last_seen = timestamp
            match.missed_frames = 0
            match.phase = (
                TrackPhase.TRACKING
                if match.confirmed
                else TrackPhase.DETECTING
            )
            seen_tracks.add(match.track_id)

        for track_id in unmatched_tracks:
            track = self._tracks[track_id]
            track.missed_frames += 1
            if track.confirmed:
                track.phase = TrackPhase.COASTING

        expired = [
            track_id for track_id, track in self._tracks.items()
            if (
                track.missed_frames > self.max_missed_frames
                or timestamp - track.last_seen > self.stale_after
            )
        ]
        for track_id in expired:
            self._tracks[track_id].phase = TrackPhase.LOST
            del self._tracks[track_id]

        stable: list[StableFruitTarget] = []
        self.last_rejection_reason = "no_track"
        publishable_tracks = set(seen_tracks)
        if self.publish_kalman_predictions:
            publishable_tracks.update(
                track_id
                for track_id, track in self._tracks.items()
                if track_id not in seen_tracks
                and track.confirmed
                and track.phase == TrackPhase.COASTING
                and track.missed_frames <= self.max_missed_frames
                and timestamp - track.last_seen <= self.max_prediction_age_s
            )
        for track_id in publishable_tracks:
            track = self._tracks[track_id]
            predicted = track_id not in seen_tracks
            target = self._stable_target(track, timestamp, predicted)
            if target is not None:
                stable.append(target)
        if stable:
            self.last_rejection_reason = "accepted"
        return sorted(stable, key=lambda target: (-target.confidence, target.track_id))

    def _nearest_track(
        self, observation: FruitObservation, candidates: set[str], timestamp: float
    ) -> _Track | None:
        best: _Track | None = None
        best_distance = self.association_distance
        nearest_distance = None
        for track_id in candidates:
            track = self._tracks[track_id]
            if track.motion_filter is not None:
                predicted_state, _ = track.motion_filter.projected(timestamp)
                predicted = tuple(float(value) for value in predicted_state[:3])
            else:
                previous = track.samples[-1][1]
                predicted = (previous.x, previous.y, previous.z)
            distance = math.dist(
                (observation.x, observation.y, observation.z),
                predicted,
            )
            nearest_distance = (
                distance
                if nearest_distance is None
                else min(nearest_distance, distance)
            )
            if distance < best_distance:
                best = track
                best_distance = distance
        if best is None and nearest_distance is not None:
            self.last_rejection_reason = (
                "association_distance=%.3fm>%.3fm"
                % (nearest_distance, self.association_distance)
            )
        return best

    def _update_motion_filter(
        self, track: _Track, observation: FruitObservation, timestamp: float
    ) -> bool:
        if track.motion_filter is None:
            track.motion_filter = ConstantVelocityKalman3D(
                measurement_std_m=self.kalman_measurement_std_m,
                acceleration_std_mps2=self.kalman_acceleration_std_mps2,
                gate_sigma=self.kalman_gate_sigma,
            )
        return track.motion_filter.update(
            (observation.x, observation.y, observation.z), timestamp
        )

    def _new_track(self) -> _Track:
        self._next_id += 1
        track_id = f"fruit_track_{self._next_id:05d}"
        track = _Track(track_id)
        self._tracks[track_id] = track
        return track

    def _stable_target(
        self, track: _Track, output_timestamp: float, predicted: bool
    ) -> StableFruitTarget | None:
        observations = [sample for _, sample in track.samples]
        if len(observations) < self.min_frames:
            self.last_rejection_reason = (
                "samples=%d<min_frames=%d" % (len(observations), self.min_frames)
            )
            return None
        labels = [self._normalise_health(o.health) for o in observations]
        if self.require_known_health:
            known = [label for label in labels if label != "Unknown"]
            if not known or len(known) / len(labels) < self.min_known_ratio:
                self.last_rejection_reason = "known_ratio=%.2f<%.2f" % (
                    len(known) / max(1.0, len(labels)), self.min_known_ratio
                )
                return None
            health, count = Counter(known).most_common(1)[0]
            # Unknown observations count against the majority.  This prevents
            # a track with a few confident frames and many failed
            # classifications from becoming a valid pick target.
            if count / len(observations) < self.class_majority:
                self.last_rejection_reason = "class_majority=%.2f<%.2f" % (
                    count / len(observations), self.class_majority
                )
                return None
        else:
            # Coordinate-only debug tracking deliberately reports Unknown;
            # it proves YOLO+depth+KF independently from MobileNet and is
            # never connected to the grasp target publisher.
            health = "Unknown"
            count = len(observations)

        timestamps = [timestamp for timestamp, _ in track.samples]
        xs = [o.x for o in observations]
        ys = [o.y for o in observations]
        zs = [o.z for o in observations]
        axis_std = [
            self._linear_residual_std(timestamps, axis)
            for axis in (xs, ys, zs)
        ]
        position_std = max(axis_std)
        if position_std > self.max_position_std:
            self.last_rejection_reason = "position_std=%.4fm>%.4fm" % (
                position_std, self.max_position_std
            )
            return None

        winning = (
            [
                o for o in observations
                if self._normalise_health(o.health) == health
                and float(o.health_margin) >= self.min_health_margin
            ]
            if self.require_known_health
            else observations
        )
        if len(winning) < count:
            self.last_rejection_reason = "health_margin=%d<winning=%d" % (
                len(winning), count
            )
            return None
        detection_confidence = statistics.mean(
            float(o.detection_confidence) for o in winning
        )
        health_confidence = statistics.mean(
            float(o.health_confidence) for o in winning
        )
        if (
            detection_confidence < self.min_detection_confidence
            or health_confidence < self.min_confidence
        ):
            self.last_rejection_reason = (
                "confidence=det%.3f/%.3f health%.3f/%.3f"
                % (
                    detection_confidence,
                    self.min_detection_confidence,
                    health_confidence,
                    self.min_confidence,
                )
            )
            return None
        confidence = min(detection_confidence, health_confidence)
        measurement_age = max(0.0, float(output_timestamp) - track.last_seen)
        if track.motion_filter is not None and predicted:
            projected_state, _ = track.motion_filter.projected(output_timestamp)
            filtered_position = tuple(float(value) for value in projected_state[:3])
        else:
            filtered_position = (
                track.motion_filter.position
                if track.motion_filter is not None
                else (observations[-1].x, observations[-1].y, observations[-1].z)
            )
        filtered_velocity = (
            track.motion_filter.velocity
            if track.motion_filter is not None
            else (0.0, 0.0, 0.0)
        )
        if predicted and self.max_prediction_age_s > 0.0:
            confidence *= max(
                0.35, 1.0 - 0.65 * measurement_age / self.max_prediction_age_s
            )
        if predicted:
            track.phase = TrackPhase.COASTING
        else:
            # Confirmation happens only after depth, geometry, health and all
            # temporal gates pass.  A tentative detection can never coast into
            # a published robot target.
            track.confirmed = True
            track.phase = TrackPhase.TRACKING
        return StableFruitTarget(
            track_id=track.track_id,
            # A moving target must be published at its current position.  The
            # residual gate above rejects jitter but does not add historical
            # latency to an otherwise coherent trajectory.
            centroid=filtered_position,
            radius=statistics.median(o.radius for o in observations),
            health=health,
            confidence=confidence,
            position_std=position_std,
            sample_count=len(observations),
            velocity=filtered_velocity,
            predicted=predicted,
            measurement_age=measurement_age,
            phase=track.phase.value,
        )

    @staticmethod
    def _linear_residual_std(timestamps, values) -> float:
        """Measure jitter around a constant-velocity trajectory."""
        if len(values) < 2:
            return 0.0
        t0 = float(timestamps[0])
        times = [float(value) - t0 for value in timestamps]
        mean_t = statistics.mean(times)
        mean_value = statistics.mean(values)
        denominator = sum((value - mean_t) ** 2 for value in times)
        if denominator <= 1.0e-12:
            return statistics.pstdev(values)
        slope = sum(
            (time_value - mean_t) * (value - mean_value)
            for time_value, value in zip(times, values)
        ) / denominator
        intercept = mean_value - slope * mean_t
        residuals = [
            value - (intercept + slope * time_value)
            for time_value, value in zip(times, values)
        ]
        return statistics.pstdev(residuals)

    @staticmethod
    def _normalise_health(value: str) -> str:
        value = str(value).strip().lower()
        if value in ("healthy", "good", "0"):
            return "Healthy"
        if value in ("unhealthy", "bad", "1"):
            return "Unhealthy"
        return "Unknown"
