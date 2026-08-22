import threading

from jaka_single_arm.perception.fruit_target_node import FruitTargetNode


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

    generation, objects, rgb, depth_stamp, sync_delta, color_info = node._health_job
    assert generation == 2
    assert objects == ["new"]
    assert rgb == "rgb-new"
    assert depth_stamp == "depth-new"
    assert sync_delta == 0.02
    assert color_info is None


def test_invalidating_pipeline_discards_queued_classification():
    node = _pipeline_shell()
    node._enqueue_health_job(["old"], "rgb-old", "depth-old", 0.01)

    node._invalidate_health_jobs()

    assert node._health_generation == 2
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
