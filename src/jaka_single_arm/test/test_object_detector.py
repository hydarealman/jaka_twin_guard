import math
import struct

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from jaka_single_arm.perception.object_detector import ObjectDetector


class _Logger:
    def warning(self, _message):
        pass


class _DetectorStub:
    _logger = _Logger()


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
