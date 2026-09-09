from types import SimpleNamespace

from fruit_picking_arm.perception.source_freshness import SourceProgress


def stamp(sec):
    return SimpleNamespace(sec=int(sec), nanosec=int((sec - int(sec)) * 1e9))


def test_duplicate_and_replayed_frames_cannot_refresh_heartbeat():
    progress = SourceProgress()
    assert progress.observe(stamp(50), 10)
    assert not progress.observe(stamp(50), 11)
    assert not progress.observe(stamp(49), 12)
    assert not progress.fresh(12, 1)
    assert progress.observe(stamp(51), 12)
    assert progress.fresh(12, 1)


def test_target_must_belong_to_recent_camera_source_window():
    progress = SourceProgress()
    progress.observe(stamp(100), 1)
    assert progress.contains(stamp(99.5), 1)
    assert not progress.contains(stamp(98), 1)
    assert not progress.contains(stamp(102), 1)
    assert not progress.observe(stamp(0), 2)
