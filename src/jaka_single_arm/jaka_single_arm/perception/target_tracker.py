"""ROS-independent multi-frame stabilisation for detected fruits."""

from __future__ import annotations

import math
import statistics
from collections import Counter, deque
from dataclasses import dataclass, field


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


@dataclass
class _Track:
    track_id: str
    samples: deque[tuple[float, FruitObservation]] = field(default_factory=deque)
    last_seen: float = 0.0
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    missed_frames: int = 0


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
        self._tracks: dict[str, _Track] = {}
        self._next_id = 0
        self.last_rejection_reason = "no_track"

    def reset(self) -> None:
        self._tracks.clear()

    def update(self, observations: list[FruitObservation], timestamp: float) -> list[StableFruitTarget]:
        timestamp = float(timestamp)
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
            else:
                unmatched_tracks.discard(match.track_id)
                self._update_velocity(match, observation, timestamp)
            match.samples.append((timestamp, observation))
            while len(match.samples) > self.window_size:
                match.samples.popleft()
            match.last_seen = timestamp
            match.missed_frames = 0
            seen_tracks.add(match.track_id)

        for track_id in unmatched_tracks:
            self._tracks[track_id].missed_frames += 1

        expired = [
            track_id for track_id, track in self._tracks.items()
            if (
                track.missed_frames > self.max_missed_frames
                or timestamp - track.last_seen > self.stale_after
            )
        ]
        for track_id in expired:
            del self._tracks[track_id]

        stable: list[StableFruitTarget] = []
        self.last_rejection_reason = "no_track"
        for track_id in seen_tracks:
            target = self._stable_target(self._tracks[track_id])
            if target is not None:
                stable.append(target)
        return sorted(stable, key=lambda target: (-target.confidence, target.track_id))

    def _nearest_track(
        self, observation: FruitObservation, candidates: set[str], timestamp: float
    ) -> _Track | None:
        best: _Track | None = None
        best_distance = self.association_distance
        nearest_distance = None
        for track_id in candidates:
            track = self._tracks[track_id]
            previous = track.samples[-1][1]
            elapsed = max(0.0, float(timestamp) - track.last_seen)
            predicted = tuple(
                value + speed * min(elapsed, self.stale_after)
                for value, speed in zip(
                    (previous.x, previous.y, previous.z), track.velocity
                )
            )
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

    def _update_velocity(
        self, track: _Track, observation: FruitObservation, timestamp: float
    ) -> None:
        if not track.samples:
            return
        previous_timestamp, previous = track.samples[-1]
        elapsed = float(timestamp) - float(previous_timestamp)
        if elapsed <= 1.0e-6:
            return
        measured = tuple(
            (current - old) / elapsed
            for current, old in zip(
                (observation.x, observation.y, observation.z),
                (previous.x, previous.y, previous.z),
            )
        )
        # A small exponential smoother keeps the prediction useful while
        # preventing one noisy depth sample from moving the association gate.
        alpha = 0.5
        track.velocity = tuple(
            (1.0 - alpha) * old + alpha * new
            for old, new in zip(track.velocity, measured)
        )

    def _new_track(self) -> _Track:
        self._next_id += 1
        track_id = f"fruit_track_{self._next_id:05d}"
        track = _Track(track_id)
        self._tracks[track_id] = track
        return track

    def _stable_target(self, track: _Track) -> StableFruitTarget | None:
        observations = [sample for _, sample in track.samples]
        if len(observations) < self.min_frames:
            self.last_rejection_reason = (
                "samples=%d<min_frames=%d" % (len(observations), self.min_frames)
            )
            return None
        labels = [self._normalise_health(o.health) for o in observations]
        known = [label for label in labels if label != "Unknown"]
        if not known or len(known) / len(labels) < self.min_known_ratio:
            self.last_rejection_reason = "known_ratio=%.2f<%.2f" % (
                len(known) / max(1.0, len(labels)), self.min_known_ratio
            )
            return None
        health, count = Counter(known).most_common(1)[0]
        # Unknown observations count against the majority.  This prevents a
        # track with a few confident frames and many failed classifications
        # from becoming a valid pick target.
        if count / len(observations) < self.class_majority:
            self.last_rejection_reason = "class_majority=%.2f<%.2f" % (
                count / len(observations), self.class_majority
            )
            return None

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

        winning = [
            o for o in observations
            if self._normalise_health(o.health) == health
            and float(o.health_margin) >= self.min_health_margin
        ]
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
        return StableFruitTarget(
            track_id=track.track_id,
            # A moving target must be published at its current position.  The
            # residual gate above rejects jitter but does not add historical
            # latency to an otherwise coherent trajectory.
            centroid=(observations[-1].x, observations[-1].y, observations[-1].z),
            radius=statistics.median(o.radius for o in observations),
            health=health,
            confidence=confidence,
            position_std=position_std,
            sample_count=len(observations),
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
