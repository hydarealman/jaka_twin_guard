import threading
import time
from types import SimpleNamespace

from fruit_picking_arm.perception.fruit_target_node import FruitTargetNode
from std_msgs.msg import Header


def _pipeline_shell():
    node = FruitTargetNode.__new__(FruitTargetNode)
    node._health_lock = threading.Lock()
    node._health_event = threading.Event()
    node._health_generation = 0
    node._health_job = None
    return node


def test_health_queue_keeps_only_the_newest_detection_frame():
    node = _pipeline_shell()

    node._enqueue_health_job(["old"], "rgb-old", "depth-old", 0.01)
    node._enqueue_health_job(["new"], "rgb-new", "depth-new", 0.02)

    generation, objects, rgb, depth_stamp, sync_delta, queued_at = node._health_job
    assert generation == 0
    assert queued_at > 0
    assert objects == ["new"]
    assert rgb == "rgb-new"
    assert depth_stamp == "depth-new"
    assert sync_delta == 0.02


def test_invalidating_pipeline_discards_queued_classification():
    node = _pipeline_shell()
    node._enqueue_health_job(["old"], "rgb-old", "depth-old", 0.01)

    node._invalidate_health_jobs()

    assert node._health_generation == 1
    assert node._health_job is None


def test_completed_stale_classification_cannot_reach_tracker():
    node = _pipeline_shell()
    node._health_generation = 1
    node._tracker_lock = threading.Lock()

    class InvalidatingFusion:
        def fuse(self, *args, **kwargs):
            node._invalidate_health_jobs()

    class TrackerMustNotRun:
        def update(self, *args, **kwargs):
            raise AssertionError("stale health result reached tracker")

    node._fusion = InvalidatingFusion()
    node._tracker = TrackerMustNotRun()

    node._process_health_job(1, [], object(), object(), 0.0)


def test_rgbd_marker_header_labels_transformed_world_coordinates():
    source = Header()
    source.frame_id = "camera_color_optical_frame"
    source.stamp.sec = 123
    source.stamp.nanosec = 456

    output = FruitTargetNode._marker_header(source, "world")

    assert output.frame_id == "world"
    assert output.stamp == source.stamp


def test_newer_detection_does_not_starve_inflight_health_result():
    node = _pipeline_shell()
    node._vision_reset_lock = threading.Lock()
    node._data_timeout_s = 1.0
    committed = []
    node._fusion = SimpleNamespace(fuse=lambda *a, **k: node._enqueue_health_job(
        ["new"], "next-rgb", "next-depth", 0.0))
    node._commit_health_result = lambda *a: committed.append(a)
    node._process_health_job(0, ["old"], object(), object(), 0.0)
    assert len(committed) == 1


def test_timed_out_health_result_cannot_publish():
    node = _pipeline_shell()
    node._vision_reset_lock = threading.Lock()
    node._data_timeout_s = 1.0
    committed = []
    node._fusion = SimpleNamespace(fuse=lambda *a, **k: None)
    node._commit_health_result = lambda *a: committed.append(a)
    node._process_health_job(0, [], object(), object(), 0.0, time.monotonic() - 2)
    assert committed == []
