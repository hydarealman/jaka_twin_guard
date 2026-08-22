import numpy as np
import time

from sensor_msgs.msg import CameraInfo, Image

from jaka_single_arm.perception.realsense_camera import RealSenseCamera
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
