from types import SimpleNamespace

from sensor_msgs.msg import CameraInfo, Image

from jaka_single_arm.skills.detect_objects import DetectObjectsSkill


class _Camera:
    def __init__(self, pair):
        self.pair = pair

    def get_synced_rgbd(self, _delta):
        return self.pair

    def get_rgb_image_age_s(self):
        return 0.01

    def get_aligned_depth_image_age_s(self):
        return 0.01


class _Detector:
    def __init__(self):
        self.published = []

    def publish_objects(self, objects, _header):
        self.published = list(objects)

    def clear_markers(self, _header):
        self.published = []

    def republish_latest_markers(self):
        pass


def _skill(object_health="Healthy"):
    rgb, depth, info = Image(), Image(), CameraInfo()
    obj = SimpleNamespace(health=object_health)
    detector = _Detector()
    skill = DetectObjectsSkill()
    skill._blackboard = {
        "perception_config": {
            "localizer_backend": "yolo_depth",
            "rgbd_sync_tolerance_s": 0.033,
            "data_timeout_s": 1.0,
        },
        "rgbd_localizer": SimpleNamespace(
            process=lambda *_args: [obj]
        ),
        "health_fusion": SimpleNamespace(
            fuse=lambda *_args, **_kwargs: None
        ),
    }
    skill._detector = detector
    skill._get_param = lambda _name, default: default
    skill._log = lambda _message: None
    return skill, _Camera((rgb, depth, info, 0.0)), detector


def test_rgbd_skill_uses_registered_pair_and_keeps_classified_target():
    skill, camera, detector = _skill("Healthy")

    trajectory = skill._plan_rgbd(camera)

    assert trajectory is not None
    assert skill._blackboard["detection_count"] == 1
    assert len(detector.published) == 1


def test_rgbd_skill_fails_closed_for_unknown_health():
    skill, camera, _ = _skill("Unknown")

    assert skill._plan_rgbd(camera) is None
    assert skill._blackboard["detected_objects"] == []
