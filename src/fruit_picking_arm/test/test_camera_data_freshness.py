import math
import time

from fruit_picking_arm.perception.realsense_camera import RealSenseCamera


def test_realsense_reports_missing_streams_as_infinite_age():
    camera = object.__new__(RealSenseCamera)
    camera._latest_cloud = None
    camera._latest_rgb = None
    camera._latest_cloud_time = 0.0
    camera._latest_rgb_time = 0.0

    assert math.isinf(camera.get_point_cloud_age_s())
    assert math.isinf(camera.get_rgb_image_age_s())


def test_realsense_reports_received_streams_as_fresh_then_old():
    camera = object.__new__(RealSenseCamera)
    camera._latest_cloud = object()
    camera._latest_rgb = object()
    camera._latest_cloud_time = time.monotonic()
    camera._latest_rgb_time = time.monotonic()

    assert camera.get_point_cloud_age_s() < 1.0
    assert camera.get_rgb_image_age_s() < 1.0

    camera._latest_cloud_time -= 2.0
    camera._latest_rgb_time -= 2.0
    assert camera.get_point_cloud_age_s() > 1.0
    assert camera.get_rgb_image_age_s() > 1.0

