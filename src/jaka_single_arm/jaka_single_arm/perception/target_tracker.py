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
        min_health_margin: float = 0.0,
        min_known_ratio: float = 0.8,
        stale_after: float = 1.0,
    ):
        self.min_frames = max(1, int(min_frames))
        self.window_size = max(self.min_frames, int(window_size))
        self.association_distance = float(association_distance)
        self.max_position_std = float(max_position_std)
        self.class_majority = float(class_majority)
        self.min_confidence = float(min_confidence)
        self.min_health_margin = max(0.0, float(min_health_margin))
        self.min_known_ratio = min(1.0, max(0.0, float(min_known_ratio)))
        self.stale_after = float(stale_after)
        self._tracks: dict[str, _Track] = {}
        self._next_id = 0

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
            match = self._nearest_track(observation, unmatched_tracks)
            if match is None:
                match = self._new_track()
            else:
                unmatched_tracks.discard(match.track_id)
            match.samples.append((timestamp, observation))
            while len(match.samples) > self.window_size:
                match.samples.popleft()
            match.last_seen = timestamp
            seen_tracks.add(match.track_id)

        expired = [
            track_id for track_id, track in self._tracks.items()
            if timestamp - track.last_seen > self.stale_after
        ]
        for track_id in expired:
            del self._tracks[track_id]

        stable: list[StableFruitTarget] = []
        for track_id in seen_tracks:
            target = self._stable_target(self._tracks[track_id])
            if target is not None:
                stable.append(target)
        return sorted(stable, key=lambda target: (-target.confidence, target.track_id))

    def _nearest_track(self, observation: FruitObservation, candidates: set[str]) -> _Track | None:
        best: _Track | None = None
        best_distance = self.association_distance
        for track_id in candidates:
            track = self._tracks[track_id]
            previous = track.samples[-1][1]
            distance = math.dist(
                (observation.x, observation.y, observation.z),
                (previous.x, previous.y, previous.z),
            )
            if distance < best_distance:
                best = track
                best_distance = distance
        return best

    def _new_track(self) -> _Track:
        self._next_id += 1
        track_id = f"fruit_track_{self._next_id:05d}"
        track = _Track(track_id)
        self._tracks[track_id] = track
        return track

    def _stable_target(self, track: _Track) -> StableFruitTarget | None:
        observations = [sample for _, sample in track.samples]
        if len(observations) < self.min_frames:
            return None
        labels = [self._normalise_health(o.health) for o in observations]
        known = [label for label in labels if label != "Unknown"]
        if not known or len(known) / len(labels) < self.min_known_ratio:
            return None
        health, count = Counter(known).most_common(1)[0]
        # Unknown observations count against the majority.  This prevents a
        # track with a few confident frames and many failed classifications
        # from becoming a valid pick target.
        if count / len(observations) < self.class_majority:
            return None

        xs = [o.x for o in observations]
        ys = [o.y for o in observations]
        zs = [o.z for o in observations]
        axis_std = [statistics.pstdev(axis) for axis in (xs, ys, zs)]
        position_std = max(axis_std)
        if position_std > self.max_position_std:
            return None

        winning = [
            o for o in observations
            if self._normalise_health(o.health) == health
            and float(o.health_margin) >= self.min_health_margin
        ]
        if len(winning) < count:
            return None
        confidence = statistics.mean(
            min(o.detection_confidence, o.health_confidence) for o in winning
        )
        if confidence < self.min_confidence:
            return None
        return StableFruitTarget(
            track_id=track.track_id,
            centroid=(statistics.median(xs), statistics.median(ys), statistics.median(zs)),
            radius=statistics.median(o.radius for o in observations),
            health=health,
            confidence=confidence,
            position_std=position_std,
            sample_count=len(observations),
        )

    @staticmethod
    def _normalise_health(value: str) -> str:
        value = str(value).strip().lower()
        if value in ("healthy", "good", "0"):
            return "Healthy"
        if value in ("unhealthy", "bad", "1"):
            return "Unhealthy"
        return "Unknown"
