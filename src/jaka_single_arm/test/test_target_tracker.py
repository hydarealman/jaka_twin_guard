from jaka_single_arm.perception.target_tracker import (
    FruitObservation,
    FruitTargetTracker,
)


def observation(x=0.5, health="Healthy", confidence=0.9):
    return FruitObservation(
        x=x,
        y=0.1,
        z=0.34,
        radius=0.035,
        health=health,
        detection_confidence=confidence,
        health_confidence=confidence,
    )


def test_target_becomes_stable_after_required_frames():
    tracker = FruitTargetTracker(min_frames=5, window_size=7, max_position_std=0.005)
    for index in range(4):
        assert tracker.update([observation(x=0.5 + index * 0.0005)], index * 0.1) == []
    stable = tracker.update([observation(x=0.502)], 0.4)
    assert len(stable) == 1
    assert stable[0].health == "Healthy"
    assert stable[0].position_std < 0.005


def test_unstable_position_is_rejected():
    tracker = FruitTargetTracker(min_frames=5, window_size=5, max_position_std=0.003)
    stable = []
    for index, x in enumerate((0.50, 0.51, 0.49, 0.515, 0.485)):
        stable = tracker.update([observation(x=x)], index * 0.1)
    assert stable == []


def test_unknown_or_split_class_is_rejected():
    tracker = FruitTargetTracker(min_frames=5, window_size=5, class_majority=0.8)
    stable = []
    labels = ("Healthy", "Unhealthy", "Healthy", "Unhealthy", "Healthy")
    for index, label in enumerate(labels):
        stable = tracker.update([observation(health=label)], index * 0.1)
    assert stable == []


def test_unknown_frames_count_against_majority():
    tracker = FruitTargetTracker(
        min_frames=5, window_size=5, class_majority=0.75, min_known_ratio=0.8
    )
    labels = ("Healthy", "Healthy", "Healthy", "Unknown", "Unknown")
    stable = []
    for index, label in enumerate(labels):
        stable = tracker.update([observation(health=label)], index * 0.1)
    assert stable == []


def test_ambiguous_quality_margin_is_rejected():
    tracker = FruitTargetTracker(
        min_frames=5, window_size=5, min_health_margin=0.20
    )
    stable = []
    for index in range(5):
        stable = tracker.update(
            [
                FruitObservation(
                    x=0.5, y=0.1, z=0.34, radius=0.035,
                    health="Healthy", detection_confidence=0.9,
                    health_confidence=0.8, health_margin=0.10,
                )
            ],
            index * 0.1,
        )
    assert stable == []
