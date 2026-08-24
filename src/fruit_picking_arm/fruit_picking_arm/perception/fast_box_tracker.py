"""Short-horizon 2-D box continuity for debug and downstream association only."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DisplayDetection:
    box: np.ndarray
    score: float
    predicted: bool = False
    confirmed: bool = False


class FastBoxTracker:
    """Bridge very short detector gaps without manufacturing grasp measurements."""

    def __init__(
        self,
        confirm_hits: int = 2,
        max_misses: int = 2,
        max_age_s: float = 0.18,
        max_center_motion_ratio: float = 0.35,
        velocity_alpha: float = 0.65,
    ) -> None:
        self._confirm_hits = max(2, int(confirm_hits))
        self._max_misses = max(0, int(max_misses))
        self._max_age_s = max(0.0, float(max_age_s))
        self._max_center_motion_ratio = max(0.01, float(max_center_motion_ratio))
        self._velocity_alpha = min(1.0, max(0.0, float(velocity_alpha)))
        self.reset()

    def reset(self) -> None:
        self._box = None
        self._score = 0.0
        self._stamp_s = 0.0
        self._velocity = np.zeros(4, dtype=np.float32)
        self._hits = 0
        self._misses = 0

    def update(self, detections, stamp_s: float, image_shape) -> list[DisplayDetection]:
        height, width = int(image_shape[0]), int(image_shape[1])
        stamp_s = float(stamp_s)
        if detections:
            box, score = max(detections, key=lambda item: float(item[1]))
            box = self._clip(np.asarray(box, dtype=np.float32), width, height)
            associated = self._associated(box, width, height)
            if associated and stamp_s > self._stamp_s:
                dt = stamp_s - self._stamp_s
                measured_velocity = (box - self._box) / dt
                alpha = self._velocity_alpha
                self._velocity = (
                    alpha * measured_velocity + (1.0 - alpha) * self._velocity
                ).astype(np.float32)
                self._hits += 1
            else:
                self._velocity.fill(0.0)
                self._hits = 1
            self._box = box
            self._score = float(score)
            self._stamp_s = stamp_s
            self._misses = 0
            return [
                DisplayDetection(
                    box.copy(),
                    self._score,
                    predicted=False,
                    confirmed=self._hits >= self._confirm_hits,
                )
            ]

        self._misses += 1
        age_s = stamp_s - self._stamp_s
        if (
            self._box is None
            or self._hits < self._confirm_hits
            or self._misses > self._max_misses
            or age_s < 0.0
            or age_s > self._max_age_s
        ):
            if age_s > self._max_age_s or self._misses > self._max_misses:
                self.reset()
            return []
        predicted = self._clip(
            self._box + self._velocity * age_s, width, height
        )
        score = self._score * (0.72 ** self._misses)
        return [DisplayDetection(predicted, score, predicted=True, confirmed=True)]

    def _associated(self, box: np.ndarray, width: int, height: int) -> bool:
        if self._box is None:
            return False
        old_center = (self._box[:2] + self._box[2:]) * 0.5
        new_center = (box[:2] + box[2:]) * 0.5
        distance = float(np.linalg.norm(new_center - old_center))
        diagonal = max(1.0, float(np.hypot(width, height)))
        return distance <= self._max_center_motion_ratio * diagonal

    @staticmethod
    def _clip(box: np.ndarray, width: int, height: int) -> np.ndarray:
        clipped = box.copy()
        clipped[[0, 2]] = np.clip(clipped[[0, 2]], 0.0, max(0, width - 1))
        clipped[[1, 3]] = np.clip(clipped[[1, 3]], 0.0, max(0, height - 1))
        if clipped[2] < clipped[0]:
            clipped[0], clipped[2] = clipped[2], clipped[0]
        if clipped[3] < clipped[1]:
            clipped[1], clipped[3] = clipped[3], clipped[1]
        return clipped
