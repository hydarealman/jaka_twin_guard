import numpy as np
import time
from types import SimpleNamespace

from sensor_msgs.msg import CameraInfo, Image

from jaka_single_arm.perception.realsense_camera import RealSenseCamera
from jaka_single_arm.perception.fruit_target_node import FruitTargetNode
from jaka_single_arm.perception.target_tracker import StableFruitTarget
from jaka_single_arm.perception.yolo_depth_localizer import YoloDepthLocalizer


def _stamp(msg, seconds):
    msg.header.stamp.sec = int(seconds)
    msg.header.stamp.nanosec = int((seconds - int(seconds)) * 1.0e9)
    return msg


def _rgb(width=4, height=4, seconds=1.0):
    msg = Image()
    msg.width, msg.height = width, height
    msg.encoding = "rgb8"
    msg.step = width * 3
    msg.data = np.zeros((height, width, 3), dtype=np.uint8).tobytes()
    return _stamp(msg, seconds)


def _depth(values, seconds=1.0):
    values = np.asarray(values, dtype="<u2")
    msg = Image()
    msg.width, msg.height = values.shape[1], values.shape[0]
    msg.encoding = "16UC1"
    msg.step = values.strides[0]
    msg.data = values.tobytes()
    return _stamp(msg, seconds)


def _fake_localizer():
    localizer = object.__new__(YoloDepthLocalizer)
    localizer._min_depth = 0.1
    localizer._max_depth = 5.0
    localizer._min_coverage = 0.2
    localizer._min_radius = 0.001
    localizer._max_radius = 1.0
    localizer._confidence = 0.1
    localizer._tracking_confidence = 0.05
    localizer._iou = 0.5
    localizer._image_size = 32
    localizer._max_detections = 5
    localizer._device = "cpu"
    localizer._inference_threads = 1
    localizer._small_object_tiles = False
    localizer._tile_rows = 2
    localizer._tile_cols = 2
    localizer._tile_overlap = 0.25
    localizer._local_tile_width_ratio = 0.70
    localizer._local_tile_height_ratio = 0.75
    localizer._max_local_tile_misses = 2
    localizer._edge_recovery = False
    localizer._edge_shift_ratio = 0.18
    localizer._edge_batch_warmed = False
    localizer._tile_cursor = 0
    localizer._last_bbox = None
    localizer._last_tile_bounds = None
    localizer._last_prediction_bounds = None
    localizer._local_tile_misses = 0
    localizer._apple_class_id = 47
    localizer._output_frame = "camera_color_optical_frame"
    localizer._camera_frame = "camera_color_optical_frame"
    localizer._roi_min = None
    localizer._roi_max = None
    localizer._transform_point = lambda point, header, frame: point
    localizer._load_model = lambda: None
    localizer._last_stats_log = time.monotonic()
    localizer._stats = {
        "frames": 0.0,
        "raw_boxes": 0.0,
        "no_box_frames": 0.0,
        "accepted": 0.0,
        "depth_rejected": 0.0,
        "radius_rejected": 0.0,
        "tf_rejected": 0.0,
        "roi_rejected": 0.0,
        "inference_ms": 0.0,
    }
    return localizer


class _TensorLike:
    def __init__(self, value):
        self.value = np.asarray(value, dtype=np.float32)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.value


class _Boxes:
    def __init__(self):
        self.xyxy = _TensorLike([[0.0, 0.0, 4.0, 4.0]])
        self.conf = _TensorLike([0.9])

    def __len__(self):
        return 1


class _Result:
    boxes = _Boxes()


class _Model:
    names = {47: "apple"}

    def predict(self, **kwargs):
        return [_Result()]


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FullMissThenTileHitModel:
    names = {47: "apple"}

    def __init__(self):
        self.calls = 0

    def predict(self, **kwargs):
        self.calls += 1
        return [type("EmptyResult", (), {"boxes": None})()] if self.calls == 1 else [_Result()]


def test_registered_depth_deprojects_apple_box_center():
    localizer = _fake_localizer()
    localizer._model = _Model()
    rgb = _rgb()
    depth = _depth(np.full((4, 4), 1000, dtype="<u2"))
    info = CameraInfo()
    info.k = [100.0, 0.0, 2.0, 0.0, 100.0, 2.0, 0.0, 0.0, 1.0]

    objects = localizer.process(rgb, depth, info)

    assert len(objects) == 1
    assert np.allclose(objects[0].centroid, (0.0, 0.0, 1.0), atol=1.0e-6)
    assert objects[0].bbox2d == (0.0, 0.0, 4.0, 4.0)
    assert objects[0].depth_coverage == 1.0


def test_fast_annotation_is_published_before_quality_pipeline_with_source_stamp():
    localizer = _fake_localizer()
    localizer._model = _Model()
    localizer._fast_annotated_pub = _CapturePublisher()
    rgb = _rgb(seconds=12.25)
    depth = _depth(np.full((4, 4), 1000, dtype="<u2"), seconds=12.25)
    info = CameraInfo()
    info.k = [100.0, 0.0, 2.0, 0.0, 100.0, 2.0, 0.0, 0.0, 1.0]

    localizer.process(rgb, depth, info)

    assert len(localizer._fast_annotated_pub.messages) == 1
    annotated = localizer._fast_annotated_pub.messages[0]
    assert annotated.header.stamp.sec == 12
    assert annotated.header.stamp.nanosec == 250_000_000
    assert annotated.width == rgb.width
    assert annotated.height == rgb.height


def test_fast_detection_message_preserves_stamp_and_box_geometry():
    localizer = _fake_localizer()
    localizer._model = _Model()
    localizer._fast_detections_pub = _CapturePublisher()
    rgb = _rgb(seconds=12.25)
    depth = _depth(np.full((4, 4), 1000, dtype="<u2"), seconds=12.25)
    info = CameraInfo()
    info.k = [100.0, 0.0, 2.0, 0.0, 100.0, 2.0, 0.0, 0.0, 1.0]

    localizer.process(rgb, depth, info)

    output = localizer._fast_detections_pub.messages[0]
    assert output.header.stamp.sec == 12
    assert output.header.stamp.nanosec == 250_000_000
    assert len(output.detections) == 1
    assert output.detections[0].bbox.center.position.x == 2.0
    assert output.detections[0].bbox.center.position.y == 2.0
    assert output.detections[0].bbox.size_x == 4.0
    assert output.detections[0].results[0].hypothesis.class_id == "apple"


def test_edge_detail_tile_runs_immediately_after_full_frame_miss():
    localizer = _fake_localizer()
    localizer._small_object_tiles = True
    localizer._model = _FullMissThenTileHitModel()
    image = np.zeros((80, 100, 3), dtype=np.uint8)

    predictions, _ = localizer._predict_boxes(image)

    assert localizer._model.calls == 2
    assert len(predictions) == 1
    assert localizer._last_prediction_mode == "grid_after_full_miss"


def test_tracking_confidence_cannot_acquire_but_can_continue_associated_box():
    localizer = _fake_localizer()
    localizer._confidence = 0.25
    localizer._tracking_confidence = 0.12
    weak = (np.asarray([30, 20, 60, 50], np.float32), 0.18)

    assert localizer._select_boxes([weak], 100, 80) == []

    localizer._last_bbox = (28, 19, 59, 49)
    selected = localizer._select_boxes([weak], 100, 80)
    assert len(selected) == 1
    assert selected[0][1] == 0.18


def test_local_tracking_crop_recenters_on_latest_box():
    localizer = _fake_localizer()
    localizer._local_tile_width_ratio = 0.40
    localizer._local_tile_height_ratio = 0.50
    localizer._last_bbox = (10, 20, 30, 40)
    first = localizer._local_tile_bounds(200, 100)
    localizer._last_bbox = (150, 60, 180, 90)
    second = localizer._local_tile_bounds(200, 100)

    assert second[0] > first[0]
    assert second[1] > first[1]


def test_depth_sampling_rejects_invalid_and_far_outlier_pixels():
    localizer = _fake_localizer()
    values = np.full((10, 10), 1.0, dtype=np.float32)
    values[5, 5] = 4.0
    values[0, 0] = 0

    result = localizer._sample_depth(values, 0, 0, 10, 10)

    assert result is not None
    assert abs(result[0] - 1.0) < 1.0e-6
    assert result[1] > 0.9


def test_rgbd_sync_rejects_pair_over_33ms():
    camera = object.__new__(RealSenseCamera)
    camera._rgb_frames = [_rgb(seconds=1.000)]
    camera._aligned_depth_frames = [_depth(np.ones((2, 2)) * 1000, seconds=1.040)]
    camera._latest_color_camera_info = CameraInfo()

    assert camera.get_synced_rgbd(0.033) is None
    camera._aligned_depth_frames.append(
        _depth(np.ones((2, 2)) * 1000, seconds=1.020)
    )
    pair = camera.get_synced_rgbd(0.033)
    assert pair is not None
    assert pair[3] < 0.033


def test_rgbd_sync_selects_newest_valid_pair_instead_of_oldest_tie():
    camera = object.__new__(RealSenseCamera)
    camera._rgb_frames = [_rgb(seconds=1.0), _rgb(seconds=2.0)]
    camera._aligned_depth_frames = [
        _depth(np.ones((2, 2)) * 1000, seconds=1.0),
        _depth(np.ones((2, 2)) * 1000, seconds=2.0),
    ]
    camera._latest_color_camera_info = CameraInfo()

    rgb, depth, _, delta = camera.get_synced_rgbd(0.033)

    assert rgb.header.stamp.sec == 2
    assert depth.header.stamp.sec == 2
    assert delta == 0.0


def test_kf_projection_draws_filtered_position_and_velocity_arrow():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    info = CameraInfo()
    info.header.frame_id = "camera_color_optical_frame"
    info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
    target = StableFruitTarget(
        track_id="fruit_track_00001",
        centroid=(0.0, 0.0, 1.0),
        radius=0.05,
        health="Healthy",
        confidence=0.9,
        position_std=0.001,
        sample_count=5,
        velocity=(0.2, 0.0, 0.0),
        phase="tracking",
    )

    annotated = FruitTargetNode._draw_kf_projection(
        image, [target], info, "camera_color_optical_frame"
    )

    assert np.any(annotated[48:53, 48:53] != 0)
    assert np.any(annotated[48:53, 53:57] != 0)


def test_kf_projection_refuses_to_fake_a_cross_frame_projection():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    info = CameraInfo()
    info.header.frame_id = "camera_color_optical_frame"
    info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
    target = SimpleNamespace(
        centroid=(0.0, 0.0, 1.0),
        radius=0.05,
        phase="tracking",
        predicted=False,
        track_id="fruit_track_00001",
        velocity=(0.0, 0.0, 0.0),
        measurement_age=0.0,
    )

    annotated = FruitTargetNode._draw_kf_projection(
        image, [target], info, "Link_00"
    )

    assert not np.any(annotated[45:56, 45:56])
    assert np.any(annotated != 0)
