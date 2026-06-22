#!/usr/bin/env python3
"""Simulated depth camera node for pick-and-place demo.

Publishes synthetic point cloud data representing the scene as seen by an
overhead Odin1-class depth camera. Fruits appear as spherical clusters.

This node can run alongside pick_place_demo.py or be replaced by a real
camera driver later — the topic interface is the same.
"""

from __future__ import annotations

import math
import random
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


# ---------------------------------------------------------------------------
# Scene knowledge (must match pick_place_demo.py FRUITS)
# ---------------------------------------------------------------------------

TABLE_TOP_Z = 0.30
CAMERA_Z = 0.86  # height of camera_depth_frame above ground

FRUITS = [
    {"x": 0.50, "y": -0.20, "radius": 0.035},
    {"x": 0.55, "y": -0.05, "radius": 0.040},
    {"x": 0.40, "y": 0.15, "radius": 0.030},
    {"x": 0.60, "y": -0.08, "radius": 0.028},
]

POINTS_PER_FRUIT = 400
TABLE_POINTS = 2000
NOISE_STD = 0.003  # 3mm Gaussian noise


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class SimulatedCamera(Node):
    def __init__(self) -> None:
        super().__init__("simulated_camera")

        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("frame_id", "camera_depth_frame")
        self.declare_parameter("table_top_z", TABLE_TOP_Z)
        self.declare_parameter("camera_z", CAMERA_Z)

        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.cloud_pub = self.create_publisher(
            PointCloud2, "/camera/depth/points", 10,
        )
        self.timer = self.create_timer(1.0 / rate, self._publish_cloud)

        # Precompute table plane points (random scatter on table top)
        self.table_points = self._generate_table_points()

        self.get_logger().info(
            f"Simulated Odin1 depth camera publishing on /camera/depth/points "
            f"at {rate} Hz (frame: {self.frame_id})"
        )

    def _generate_table_points(self) -> list[tuple[float, float, float]]:
        """Generate random points on the table surface within its bounds."""
        points = []
        for _ in range(TABLE_POINTS):
            x = random.uniform(0.35, 0.95)
            y = random.uniform(-0.35, 0.35)
            z = TABLE_TOP_Z + random.gauss(0, NOISE_STD)
            points.append((x, y, z))
        return points

    def _generate_fruit_points(
        self, fruit: dict,
    ) -> list[tuple[float, float, float]]:
        """Generate points on the visible hemisphere of a fruit."""
        points = []
        cx, cy = fruit["x"], fruit["y"]
        cz = TABLE_TOP_Z + fruit["radius"]

        for _ in range(POINTS_PER_FRUIT):
            # Uniform random direction on hemisphere (visible from above)
            theta = random.uniform(0, 2.0 * math.pi)
            # bias toward top of sphere (camera looks down)
            phi = random.uniform(0, math.pi / 2.0 + 0.3)

            r = fruit["radius"] + random.gauss(0, NOISE_STD)
            r = max(0.0, r)

            x = cx + r * math.sin(phi) * math.cos(theta)
            y = cy + r * math.sin(phi) * math.sin(theta)
            z = cz + r * math.cos(phi)

            points.append((x, y, z))

        return points

    def _publish_cloud(self) -> None:
        now = self.get_clock().now().to_msg()

        # Gather all points
        all_points = list(self.table_points)
        for fruit in FRUITS:
            all_points.extend(self._generate_fruit_points(fruit))

        num_points = len(all_points)

        # Build PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        msg.height = 1
        msg.width = num_points
        msg.is_bigendian = False
        msg.is_dense = True

        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12  # 3 × float32
        msg.row_step = msg.point_step * num_points

        # Pack binary data
        data = bytearray()
        for x, y, z in all_points:
            data.extend(struct.pack("<fff", x, y, z))
        msg.data = bytes(data)

        self.cloud_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    rclpy.init()
    node = SimulatedCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
