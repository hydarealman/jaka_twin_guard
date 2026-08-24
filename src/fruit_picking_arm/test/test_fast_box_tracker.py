import numpy as np

from fruit_picking_arm.perception.fast_box_tracker import FastBoxTracker


def detection(x1, x2, score=0.9):
    return [(np.array([x1, 20.0, x2, 40.0], dtype=np.float32), score)]


def test_single_false_detection_is_never_extended():
    tracker = FastBoxTracker(confirm_hits=2, max_misses=2, max_age_s=0.18)

    first = tracker.update(detection(10.0, 30.0), 1.0, (80, 100, 3))
    missed = tracker.update([], 1.06, (80, 100, 3))

    assert len(first) == 1
    assert not first[0].confirmed
    assert missed == []


def test_confirmed_moving_box_is_predicted_for_two_short_misses():
    tracker = FastBoxTracker(confirm_hits=2, max_misses=2, max_age_s=0.18)
    tracker.update(detection(10.0, 30.0), 1.0, (80, 100, 3))
    confirmed = tracker.update(detection(20.0, 40.0), 1.1, (80, 100, 3))

    first_gap = tracker.update([], 1.15, (80, 100, 3))
    second_gap = tracker.update([], 1.18, (80, 100, 3))

    assert confirmed[0].confirmed
    assert first_gap[0].predicted
    assert second_gap[0].predicted
    assert first_gap[0].box[0] > 20.0
    assert first_gap[0].score < confirmed[0].score


def test_prediction_expires_and_does_not_restart_without_real_hits():
    tracker = FastBoxTracker(confirm_hits=2, max_misses=2, max_age_s=0.18)
    tracker.update(detection(10.0, 30.0), 1.0, (80, 100, 3))
    tracker.update(detection(20.0, 40.0), 1.1, (80, 100, 3))

    assert tracker.update([], 1.30, (80, 100, 3)) == []
    assert tracker.update([], 1.31, (80, 100, 3)) == []
