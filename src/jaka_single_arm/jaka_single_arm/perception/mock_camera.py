#!/usr/bin/env python3
"""Mock Camera — synthetic point cloud for development without hardware.

Generates realistic point cloud data from scene YAML config.
Port of the original simulated_camera.py, integrated into the
CameraInterface abstraction.
"""

from __future__ import annotations

import math
import random
import struct

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image

from jaka_single_arm.perception.camera_interface import CameraInterface


class MockCamera(CameraInterface):
    """Generates synthetic point clouds from scene YAML config.

    Objects appear as spherical point clusters on a flat table plane.
    Gaussian noise simulates real sensor characteristics.
    """

    def __init__(self, node: Node, config: dict, scene_cfg: dict = None):
        super().__init__(node, config)
        self._scene_cfg = scene_cfg or {}
        self._mock_cfg = config.get("mock_camera", {})

        self._publish_rate = self._mock_cfg.get("publish_rate", 10.0)
        self._table_points = self._mock_cfg.get("table_points", 2000)
        self._points_per_object = self._mock_cfg.get("points_per_object", 400)
        self._noise_std = self._mock_cfg.get("noise_std", 0.003)

        # Compute table bounds from scene config
        table = self._scene_cfg.get("table", {})
        self._table_top_z = table.get("top_z", 0.30)
        tc = table.get("center", {"x": 0.70, "y": 0.0})
        ts = table.get("size", {"x": 0.75, "y": 0.70, "z": 0.04})
        self._table_cx = tc["x"]
        self._table_cy = tc["y"]
        self._table_hx = ts["x"] / 2.0
        self._table_hy = ts["y"] / 2.0

        self._objects = self._scene_cfg.get("objects", [])

        # Publishers
        self._cloud_pub = None
        self._timer = None
        self._latest_cloud: PointCloud2 | None = None
        self._latest_stamp = None

    def connect(self) -> bool:
        if self._connected:
            return True

        self._cloud_pub = self._node.create_publisher(
            PointCloud2, self._config.get("point_cloud_topic", "/perception/point_cloud"), 10
        )
        period = 1.0 / max(self._publish_rate, 1.0)
        self._timer = self._node.create_timer(period, self._publish_cloud)

        self._connected = True
        self._logger.info(
            f"MockCamera connected (rate={self._publish_rate}Hz, "
            f"objects={len(self._objects)})"
        )
        return True

    def disconnect(self) -> bool:
        self._connected = False
        if self._timer:
            self._node.destroy_timer(self._timer)
            self._timer = None
        if self._cloud_pub:
            self._node.destroy_publisher(self._cloud_pub)
            self._cloud_pub = None
        return True

    def get_point_cloud(self) -> PointCloud2 | None:
        return self._latest_cloud

    def get_depth_image(self) -> Image | None:
        return None  # Mock camera doesn't generate depth images

    def get_rgb_image(self) -> Image | None:
        return None  # Mock camera doesn't generate RGB images

    # ── Internal point generation ────────────────────────────

    def _generate_table_points(self) -> list[tuple[float, float, float]]:
        points = []
        for _ in range(self._table_points):
            x = random.uniform(self._table_cx - self._table_hx,
                               self._table_cx + self._table_hx)
            y = random.uniform(self._table_cy - self._table_hy,
                               self._table_cy + self._table_hy)
            z = self._table_top_z + random.gauss(0, self._noise_std)
            points.append((x, y, z))
        return points

    def _generate_object_points(self, obj: dict) -> list[tuple[float, float, float]]:
        points = []
        pos = obj.get("position", {"x": 0.0, "y": 0.0})
        cx, cy = pos["x"], pos["y"]
        radius = obj.get("radius", 0.03)
        cz = self._table_top_z + radius

        for _ in range(self._points_per_object):
            theta = random.uniform(0, 2.0 * math.pi)
            phi = random.uniform(0, math.pi / 2.0 + 0.3)  # bias toward top

            r = radius + random.gauss(0, self._noise_std)
            r = max(0.0, r)

            x = cx + r * math.sin(phi) * math.cos(theta)
            y = cy + r * math.sin(phi) * math.sin(theta)
            z = cz + r * math.cos(phi)
            points.append((x, y, z))
        return points

    def _publish_cloud(self) -> None:
        now = self._node.get_clock().now().to_msg()

        all_points = list(self._generate_table_points())
        for obj in self._objects:
            all_points.extend(self._generate_object_points(obj))

        num_points = len(all_points)

        msg = PointCloud2()
        msg.header.stamp = now
        # Mock camera generates points directly in world coordinates.
        # Use "world" as the frame so RViz can display without needing a TF.
        msg.header.frame_id = "world"  # self._camera_frame → no TF available
        msg.height = 1
        msg.width = num_points
        msg.is_bigendian = False
        msg.is_dense = True
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * num_points

        data = bytearray()
        for x, y, z in all_points:
            data.extend(struct.pack("<fff", x, y, z))
        msg.data = bytes(data)

        self._cloud_pub.publish(msg)
        self._latest_cloud = msg
        self._latest_stamp = now
