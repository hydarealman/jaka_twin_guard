import math

import pytest

from fruit_picking_arm.scene.bin_geometry import (
    bin_drop_candidates,
    bin_wall_boxes,
    normalized_bin_config,
)


def _bin():
    return {
        "center": {"x": 0.50, "y": -0.25, "z": 0.325},
        "size": {"x": 0.30, "y": 0.20, "z": 0.15},
        "wall_thickness": 0.01,
    }


def test_bin_yaml_size_is_exact_outer_envelope():
    boxes = bin_wall_boxes(_bin())
    min_x = min(box.center[0] - box.size[0] / 2.0 for box in boxes)
    max_x = max(box.center[0] + box.size[0] / 2.0 for box in boxes)
    min_y = min(box.center[1] - box.size[1] / 2.0 for box in boxes)
    max_y = max(box.center[1] + box.size[1] / 2.0 for box in boxes)
    min_z = min(box.center[2] - box.size[2] / 2.0 for box in boxes)
    max_z = max(box.center[2] + box.size[2] / 2.0 for box in boxes)

    assert math.isclose(min_x, 0.35)
    assert math.isclose(max_x, 0.65)
    assert math.isclose(min_y, -0.35)
    assert math.isclose(max_y, -0.15)
    assert math.isclose(min_z, 0.25)
    assert math.isclose(max_z, 0.40)


def test_bin_validation_exposes_inner_opening():
    cfg = normalized_bin_config(_bin())
    assert math.isclose(cfg["inner_size"]["x"], 0.28)
    assert math.isclose(cfg["inner_size"]["y"], 0.18)
    assert math.isclose(cfg["bottom_z"], 0.25)
    assert math.isclose(cfg["top_z"], 0.40)


@pytest.mark.parametrize(
    "change",
    [
        {"size": {"x": 0.01, "y": 0.20, "z": 0.15}},
        {"size": {"x": 0.30, "y": 0.20, "z": 0.005}},
        {"center": {"x": 0.50, "y": -0.25, "z": float("nan")}},
    ],
)
def test_impossible_bin_geometry_fails_closed(change):
    config = _bin()
    config.update(change)
    with pytest.raises(ValueError):
        normalized_bin_config(config)


def test_drop_candidates_stay_inside_opening_and_favour_base_side():
    config = {
        "center": {"x": 0.06, "y": -0.73, "z": 0.11},
        "size": {"x": 0.34, "y": 0.25, "z": 0.22},
        "wall_thickness": 0.01,
    }
    candidates = bin_drop_candidates(config, fruit_radius=0.03, wall_clearance=0.01)
    assert candidates[0] == pytest.approx((0.06, -0.73))
    assert candidates[1] == pytest.approx((0.06, -0.655))
    assert candidates[3] == pytest.approx((0.18, -0.655))
    for x, y in candidates:
        assert -0.060000001 <= x <= 0.180000001
        assert -0.805000001 <= y <= -0.654999999


def test_drop_candidates_reject_fruit_that_cannot_fit():
    config = {
        "center": {"x": 0.0, "y": 0.0, "z": 0.1},
        "size": {"x": 0.10, "y": 0.10, "z": 0.20},
        "wall_thickness": 0.01,
    }
    with pytest.raises(ValueError, match="does not fit"):
        bin_drop_candidates(config, fruit_radius=0.05, wall_clearance=0.01)
