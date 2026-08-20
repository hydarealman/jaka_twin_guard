from types import SimpleNamespace

from jaka_single_arm.perception.health_fusion import HealthFusion


class _Logger:
    def info(self, _message):
        pass


def test_scene_fallback_is_allowed_only_for_simulated_cameras():
    assert HealthFusion._scene_fallback_allowed("mock", True)
    assert HealthFusion._scene_fallback_allowed("gazebo", True)
    assert HealthFusion._scene_fallback_allowed(" GAZEBO ", True)

    assert not HealthFusion._scene_fallback_allowed("realsense", True)
    assert not HealthFusion._scene_fallback_allowed("unknown", True)
    assert not HealthFusion._scene_fallback_allowed("gazebo", False)


def test_fuse_keeps_unknown_when_scene_fallback_is_disabled():
    fusion = HealthFusion.__new__(HealthFusion)
    fusion._enabled = False
    fusion._allow_scene_fallback = False
    obj = SimpleNamespace(
        health="unknown",
        health_confidence=0.0,
        health_margin=0.0,
        fruit_type="unknown",
        class_id=-1,
    )

    result = fusion.fuse([obj])

    assert result == [obj]
    assert obj.health == "unknown"
    assert obj.class_id == -1


def test_fuse_applies_scene_hint_when_simulation_fallback_is_enabled():
    fusion = HealthFusion.__new__(HealthFusion)
    fusion._enabled = False
    fusion._allow_scene_fallback = True
    fusion._fallback_confidence = 1.0
    fusion._logger = _Logger()
    fusion._scene_objects = [
        {"position": {"x": 0.4, "y": 0.1}, "health": "Unhealthy"}
    ]
    obj = SimpleNamespace(
        centroid=(0.4, 0.1, 0.3),
        health="unknown",
        health_confidence=0.0,
        health_margin=0.0,
        fruit_type="unknown",
        class_id=-1,
    )

    fusion.fuse([obj])

    assert obj.health == "Unhealthy"
    assert obj.health_confidence == 1.0
    assert obj.class_id == 1
