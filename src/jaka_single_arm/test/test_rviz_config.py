from pathlib import Path

import yaml


def test_pick_place_rviz_is_valid_and_contains_real_debug_displays():
    config = (
        Path(__file__).resolve().parents[2]
        / "moveit_resources-ros2"
        / "single_arm_jaka_c5_pick_place"
        / "config"
        / "pick_place.rviz"
    )
    data = yaml.safe_load(config.read_text(encoding="utf-8"))
    manager = data["Visualization Manager"]
    displays = {item["Name"]: item for item in manager["Displays"]}

    assert manager["Global Options"]["Fixed Frame"] == "Link_00"
    assert displays["Grid"]["Enabled"] is True
    assert displays["RobotModel"]["Enabled"] is True
    assert (
        displays["Real D455 Raw Depth (camera frame)"]["Topic"]["Value"]
        == "/perception/debug/camera_cloud"
    )
