import pytest

from jaka_single_arm.perception.real_mode import validate_real_perception_config


def test_real_mode_accepts_only_real_camera_without_fallbacks():
    validate_real_perception_config(
        camera_type="realsense",
        allow_scene_fallback=False,
        force_table_center_z=False,
        enable_table_z_fallback=False,
    )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"camera_type": "mock"},
        {"camera_type": "gazebo"},
        {"allow_scene_fallback": True},
        {"force_table_center_z": True},
        {"enable_table_z_fallback": True},
    ],
)
def test_real_mode_rejects_simulation_inputs(kwargs):
    config = {
        "camera_type": "realsense",
        "allow_scene_fallback": False,
        "force_table_center_z": False,
        "enable_table_z_fallback": False,
    }
    config.update(kwargs)
    with pytest.raises(ValueError, match="physical mode rejects"):
        validate_real_perception_config(**config)

