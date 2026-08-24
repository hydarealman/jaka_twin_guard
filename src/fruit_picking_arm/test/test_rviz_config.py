from pathlib import Path

import yaml


def test_pick_place_rviz_is_valid_and_contains_real_debug_displays():
    config = (
        Path(__file__).resolve().parents[2]
        / "moveit_resources-ros2"
        / "fruit_arm_moveit_config"
        / "config"
        / "fruit_picking_arm.rviz"
    )
    data = yaml.safe_load(config.read_text(encoding="utf-8"))
    manager = data["Visualization Manager"]
    displays = {item["Name"]: item for item in manager["Displays"]}

    assert manager["Global Options"]["Fixed Frame"] == "world"
    assert displays["Grid"]["Enabled"] is True
    assert displays["RobotModel"]["Enabled"] is True
    assert (
        displays["Real D455 Raw Depth (camera frame)"]["Topic"]["Value"]
        == "/perception/debug/camera_cloud"
    )
