import numpy as np

from sensor_msgs.msg import Image

from jaka_single_arm.perception.fruit_debug_viewer import FruitDebugViewer


def test_missing_camera_card_is_visible_but_not_sensor_data():
    image = FruitDebugViewer._diagnostic_card(
        "WAITING FOR D455 RGB IMAGE",
        ["NO SIMULATION DATA IS BEING USED"],
    )

    assert image.shape == (480, 848, 3)
    assert np.any(image != 0)
    assert np.any(image[:, :, 0] != image[:, :, 1])


def test_z16_depth_is_rendered_as_non_black_bgr_image():
    values = np.array([[500, 1000], [1500, 2500]], dtype="<u2")
    msg = Image()
    msg.height = 2
    msg.width = 2
    msg.encoding = "16UC1"
    msg.step = values.strides[0]
    msg.data = values.tobytes()

    image, minimum, maximum = FruitDebugViewer._to_depth_bgr(msg)

    assert image is not None
    assert image.shape == (2, 2, 3)
    assert minimum > 0.0
    assert maximum > minimum
    assert np.any(image != 0)


def test_invalid_depth_returns_no_visualization():
    values = np.zeros((2, 2), dtype="<u2")
    msg = Image()
    msg.height = 2
    msg.width = 2
    msg.encoding = "16UC1"
    msg.step = values.strides[0]
    msg.data = values.tobytes()

    image, minimum, maximum = FruitDebugViewer._to_depth_bgr(msg)

    assert image is None
    assert minimum == 0.0
    assert maximum == 0.0


def test_depth_display_uses_fixed_working_distance_range():
    values = np.array([[500, 1000], [1500, 2500]], dtype="<u2")
    msg = Image()
    msg.height = 2
    msg.width = 2
    msg.encoding = "16UC1"
    msg.step = values.strides[0]
    msg.data = values.tobytes()

    image, minimum, maximum = FruitDebugViewer._to_depth_bgr(msg, 0.2, 2.0)

    assert image is not None
    assert minimum == 0.2
    assert maximum == 2.0
    assert np.any(image[0, 0] != image[1, 1])
    assert FruitDebugViewer._depth_valid_ratio_from_msg(msg) == 1.0


def test_stream_state_distinguishes_startup_reconnect_and_offline():
    viewer = FruitDebugViewer.__new__(FruitDebugViewer)
    viewer._started_at = 100.0
    viewer._camera_reconnect_grace_s = 30.0

    assert viewer._stream_state(0.0, 1.0, 105.0) == "WAITING"
    assert viewer._stream_state(109.5, 1.0, 110.0) == "ONLINE"
    assert viewer._stream_state(100.0, 1.0, 110.0) == "RECONNECTING"
    assert viewer._stream_state(100.0, 1.0, 131.0) == "OFFLINE"


def test_current_rate_recovers_immediately_after_a_long_gap():
    rate, stamp = FruitDebugViewer._update_rate(10.0, 15.0, 13.0)
    assert rate == 0.0
    assert stamp == 13.0

    rate, stamp = FruitDebugViewer._update_rate(stamp, rate, 13.1)
    assert 9.9 < rate < 10.1
    assert stamp == 13.1


def test_status_overlay_does_not_grey_the_camera_image_below_header():
    source = np.full((240, 424, 3), (80, 140, 220), dtype=np.uint8)
    result = FruitDebugViewer._apply_status_overlay(source, 58)

    assert np.any(result[:58] != source[:58])
    assert np.array_equal(result[58:], source[58:])


def test_annotation_must_match_latest_raw_source_stamp_within_100ms():
    raw = Image()
    raw.header.stamp.sec = 10
    annotated = Image()
    annotated.header.stamp.sec = 9
    annotated.header.stamp.nanosec = 950_000_000

    assert FruitDebugViewer._annotation_matches_raw(raw, annotated, 0.1)

    annotated.header.stamp.nanosec = 800_000_000
    assert not FruitDebugViewer._annotation_matches_raw(raw, annotated, 0.1)


def test_zero_source_stamp_is_never_treated_as_current_annotation():
    assert not FruitDebugViewer._annotation_matches_raw(Image(), Image(), 0.1)
