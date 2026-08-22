from jaka_single_arm.perception.target_tracker import (
    FruitObservation,
    FruitTargetTracker,
    TrackPhase,
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


def test_linear_motion_is_stable_and_published_at_latest_position():
    tracker = FruitTargetTracker(
        min_frames=3,
        window_size=5,
        association_distance=0.08,
        max_position_std=0.002,
    )
    stable = []
    for index in range(5):
        stable = tracker.update(
            [observation(x=0.40 + 0.025 * index)], index * 0.1
        )
    assert len(stable) == 1
    assert abs(stable[0].centroid[0] - 0.50) < 0.005
    assert stable[0].velocity[0] > 0.15
    assert stable[0].position_std < 0.002


def test_tracker_allows_two_short_misses_but_not_a_long_gap():
    tracker = FruitTargetTracker(
        min_frames=3,
        window_size=5,
        stale_after=0.35,
        max_missed_frames=2,
    )
    for index in range(3):
        tracker.update([observation()], index * 0.1)
    assert tracker.update([], 0.3) == []
    assert tracker.update([], 0.4) == []
    stable = tracker.update([observation(x=0.501)], 0.5)
    assert len(stable) == 1
    assert stable[0].sample_count == 4


def test_kalman_prediction_bridges_only_two_fresh_misses():
    tracker = FruitTargetTracker(
        min_frames=3,
        window_size=5,
        stale_after=0.25,
        max_missed_frames=2,
        publish_kalman_predictions=True,
        max_prediction_age_s=0.18,
    )
    for index in range(3):
        stable = tracker.update(
            [observation(x=0.50 + 0.01 * index)], index * 0.05
        )
    measured_confidence = stable[0].confidence

    first = tracker.update([], 0.15)
    second = tracker.update([], 0.20)
    expired = tracker.update([], 0.25)

    assert first[0].predicted and second[0].predicted
    assert first[0].phase == TrackPhase.COASTING.value
    assert first[0].centroid[0] > stable[0].centroid[0]
    assert abs(first[0].measurement_age - 0.05) < 1.0e-9
    assert first[0].confidence < measured_confidence
    assert expired == []


def test_detector_floor_can_be_separate_from_health_floor():
    tracker = FruitTargetTracker(
        min_frames=3,
        window_size=5,
        min_confidence=0.60,
        min_detection_confidence=0.10,
    )
    for index in range(3):
        stable = tracker.update(
            [FruitObservation(
                x=0.5, y=0.1, z=0.34, radius=0.035,
                health="Healthy", detection_confidence=0.13,
                health_confidence=0.90, health_margin=0.40,
            )],
            index * 0.1,
        )
    assert len(stable) == 1
    assert stable[0].confidence == 0.13


def test_large_position_jump_starts_a_new_unstable_track():
    tracker = FruitTargetTracker(
        min_frames=3, window_size=5, association_distance=0.05
    )
    for index in range(3):
        tracker.update([observation(x=0.50)], index * 0.1)
    assert tracker.update([observation(x=0.70)], 0.3) == []


def test_explicit_state_machine_confirms_coasts_and_reacquires_same_track():
    tracker = FruitTargetTracker(
        min_frames=3,
        window_size=5,
        stale_after=0.30,
        max_missed_frames=3,
        publish_kalman_predictions=True,
        max_prediction_age_s=0.18,
    )
    first_id = None
    for index in range(3):
        stable = tracker.update(
            [observation(x=0.50 + 0.005 * index)], index * 0.05
        )
        if index == 0:
            first_id = next(iter(tracker.track_phases))
            assert tracker.track_phases[first_id] == TrackPhase.DETECTING

    assert stable[0].track_id == first_id
    assert stable[0].phase == TrackPhase.TRACKING.value
    assert tracker.track_phases[first_id] == TrackPhase.TRACKING

    coasted = tracker.update([], 0.15)
    assert coasted[0].track_id == first_id
    assert tracker.track_phases[first_id] == TrackPhase.COASTING

    reacquired = tracker.update([observation(x=0.515)], 0.20)
    assert reacquired[0].track_id == first_id
    assert not reacquired[0].predicted
    assert tracker.track_phases[first_id] == TrackPhase.TRACKING


def test_unconfirmed_track_never_publishes_a_prediction():
    tracker = FruitTargetTracker(
        min_frames=3,
        publish_kalman_predictions=True,
        max_prediction_age_s=0.18,
    )
    tracker.update([observation()], 0.0)
    track_id = next(iter(tracker.track_phases))
    assert tracker.update([], 0.05) == []
    assert tracker.track_phases[track_id] == TrackPhase.DETECTING


def test_different_target_does_not_hijack_confirmed_track():
    tracker = FruitTargetTracker(
        min_frames=3,
        association_distance=0.05,
        publish_kalman_predictions=True,
        max_prediction_age_s=0.18,
    )
    for index in range(3):
        stable = tracker.update([observation(x=0.50)], index * 0.05)
    locked_id = stable[0].track_id

    output = tracker.update([observation(x=0.70)], 0.15)
    assert len(tracker.track_phases) == 2
    assert output[0].track_id == locked_id
    assert output[0].predicted
    assert tracker.track_phases[locked_id] == TrackPhase.COASTING


def test_backward_or_invalid_source_time_forces_lost_reset():
    tracker = FruitTargetTracker(min_frames=2)
    tracker.update([observation()], 1.0)
    stable = tracker.update([observation()], 1.1)
    assert stable

    assert tracker.update([observation()], 1.05) == []
    assert tracker.track_phases == {}
    assert tracker.last_rejection_reason == "invalid_or_backward_timestamp"

    assert tracker.update([observation()], float("nan")) == []
    assert tracker.track_phases == {}


def test_coordinate_only_tracker_does_not_wait_for_health_classification():
    tracker = FruitTargetTracker(
        min_frames=2,
        window_size=5,
        min_confidence=0.0,
        min_detection_confidence=0.1,
        require_known_health=False,
    )
    unknown = observation(health="Unknown", confidence=0.8)

    assert tracker.update([unknown], 0.0) == []
    stable = tracker.update([unknown], 0.05)

    assert len(stable) == 1
    assert stable[0].health == "Unknown"
    assert stable[0].phase == TrackPhase.TRACKING.value
