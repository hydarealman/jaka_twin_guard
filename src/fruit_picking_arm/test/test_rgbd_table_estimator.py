import numpy as np

from fruit_picking_arm.perception.rgbd_table_estimator import RgbdTableEstimator


def test_horizontal_depth_plane_becomes_stable_after_five_frames():
    estimator = RgbdTableEstimator(
        stride=4, min_inliers=100, min_stable_frames=5
    )
    depth = np.full((120, 160), 0.60, dtype=np.float32)
    k = np.asarray([[140.0, 0.0, 80.0], [0.0, 140.0, 60.0], [0.0, 0.0, 1.0]])

    surface = None
    for index in range(5):
        noisy = depth + np.float32(index * 0.0002)
        surface = estimator.update(noisy, k, lambda points: points)

    assert surface is not None
    assert abs(surface.top_z - 0.6004) < 0.002
    assert surface.size_x > 0.5
    assert surface.size_y > 0.35
    assert surface.tilt_deg < 0.1


def test_world_roi_rejects_a_larger_horizontal_plane_outside_workcell():
    estimator = RgbdTableEstimator(
        stride=4,
        min_inliers=100,
        min_stable_frames=2,
        world_roi_min=(-0.5, -0.5, -0.3),
        world_roi_max=(0.5, 0.5, 0.0),
    )
    depth = np.full((240, 424), 0.8, dtype=np.float32)
    camera_matrix = np.array(
        [[300.0, 0.0, 211.5], [0.0, 300.0, 119.5], [0.0, 0.0, 1.0]]
    )

    def outside_floor(points):
        transformed = points.copy()
        transformed[:, 2] = -0.8
        return transformed

    assert estimator.update(depth, camera_matrix, outside_floor) is None
    assert estimator.last_reason == "too_few_world_roi_points"


def test_small_depth_patch_is_not_accepted_as_a_table():
    estimator = RgbdTableEstimator(stride=4, min_inliers=100)
    depth = np.zeros((120, 160), dtype=np.float32)
    depth[50:60, 70:80] = 0.60
    k = np.asarray([[140.0, 0.0, 80.0], [0.0, 140.0, 60.0], [0.0, 0.0, 1.0]])

    assert estimator.update(depth, k, lambda points: points) is None
