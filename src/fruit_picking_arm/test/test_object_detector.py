import math
import struct
import builtins

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from fruit_picking_arm.perception.object_detector import (
    ObjectDetector,
    resolve_table_fallback_policy,
)


class _Logger:
    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class _DetectorStub:
    _logger = _Logger()


class _VoxelDetectorStub:
    _voxel_size = 0.01


def test_organized_point_cloud_decodes_all_rows_and_skips_nan():
    msg = PointCloud2()
    msg.height = 2
    msg.width = 2
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 40  # Includes 8 bytes of padding at the end of each row.
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    data = bytearray(msg.row_step * msg.height)
    values = [
        (0.1, 0.2, 0.3),
        (0.4, 0.5, 0.6),
        (0.7, 0.8, 0.9),
        (math.nan, 1.1, 1.2),
    ]
    for index, value in enumerate(values):
        row, col = divmod(index, msg.width)
        struct.pack_into("<fff", data, row * msg.row_step + col * msg.point_step, *value)
    msg.data = bytes(data)

    result = ObjectDetector._cloud_to_array(_DetectorStub(), msg)

    assert result.shape == (3, 3)
    np.testing.assert_allclose(
        result,
        np.array(values[:3], dtype=np.float32),
    )


def test_voxel_filter_returns_centroid_and_skips_non_finite_points():
    points = np.array(
        [
            [0.001, 0.002, 0.003],
            [0.005, 0.006, 0.007],
            [0.012, 0.013, 0.014],
            [0.018, 0.017, 0.016],
            [math.nan, 0.0, 0.0],
            [0.0, math.inf, 0.0],
        ],
        dtype=np.float32,
    )

    result = ObjectDetector._voxel_filter(_VoxelDetectorStub(), points)

    np.testing.assert_allclose(
        result,
        np.array(
            [
                [0.003, 0.004, 0.005],
                [0.015, 0.015, 0.015],
            ],
            dtype=np.float32,
        ),
        atol=1e-7,
    )


def test_voxel_filter_is_independent_of_input_order():
    points = np.array(
        [
            [-0.019, 0.001, 0.001],
            [-0.011, 0.005, 0.005],
            [0.001, 0.002, 0.003],
            [0.009, 0.008, 0.007],
        ],
        dtype=np.float32,
    )

    forward = ObjectDetector._voxel_filter(_VoxelDetectorStub(), points)
    reversed_result = ObjectDetector._voxel_filter(
        _VoxelDetectorStub(), points[::-1]
    )

    np.testing.assert_allclose(forward, reversed_result, atol=1e-7)


def test_real_camera_cannot_enable_fixed_table_fallbacks():
    assert resolve_table_fallback_policy("realsense", True, True) == (
        False,
        False,
    )
    assert resolve_table_fallback_policy("gazebo", True, True) == (True, True)


def test_missing_scipy_rejects_frame_instead_of_merging_all_points(monkeypatch):
    original_import = builtins.__import__

    def reject_scipy(name, *args, **kwargs):
        if name.startswith("scipy"):
            raise ImportError("test: scipy unavailable")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", reject_scipy)
    points = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    assert ObjectDetector._euclidean_cluster(_DetectorStub(), points) == []
