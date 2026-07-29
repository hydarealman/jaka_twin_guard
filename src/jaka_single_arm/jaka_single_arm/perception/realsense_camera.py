#!/usr/bin/env python3
"""RealSense Camera — interfaces with Intel RealSense D400 series.

Subscribes to standard realsense2_camera ROS2 driver topics.
For future deployment with physical RealSense D435/D455 cameras.

RealSense ROS2 topics (configurable via perception_params.yaml):
  /camera/camera/depth/color/points  — PointCloud2 (aligned)
  /camera/camera/depth/image_rect_raw — depth Image
  /camera/camera/color/image_raw      — color Image
"""

from __future__ import annotations

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

from jaka_single_arm.perception.camera_interface import CameraInterface


class RealSenseCamera(CameraInterface):
    """Bridges Intel RealSense D400 camera topics into CameraInterface.

    Requires the realsense2_camera ROS2 driver running separately:
        ros2 launch realsense2_camera rs_launch.py depth_module.profile:=848x480x30

    Usage:
        camera = RealSenseCamera(node, perception_cfg)
        if camera.connect():
            cloud = camera.get_point_cloud()
    """

    def __init__(self, node: Node, config: dict, scene_cfg: dict = None):
        super().__init__(node, config)
        rs_cfg = config.get("realsense_camera", {})

        self._depth_topic = rs_cfg.get("depth_topic", "/camera/camera/depth/color/points")
        self._depth_image_topic = rs_cfg.get(
            "depth_image_topic", "/camera/camera/depth/image_rect_raw"
        )
        self._color_image_topic = rs_cfg.get(
            "color_image_topic", "/camera/camera/color/image_raw"
        )

        # Latest data buffers
        self._latest_cloud: PointCloud2 | None = None
        self._latest_depth: Image | None = None
        self._latest_rgb: Image | None = None

        # Subscribers
        self._cloud_sub = None
        self._depth_sub = None
        self._rgb_sub = None

    def connect(self) -> bool:
        if self._connected:
            return True

        self._cloud_sub = self._node.create_subscription(
            PointCloud2, self._depth_topic, self._on_cloud,
            qos_profile_sensor_data,
        )
        self._depth_sub = self._node.create_subscription(
            Image, self._depth_image_topic, self._on_depth,
            qos_profile_sensor_data,
        )
        self._rgb_sub = self._node.create_subscription(
            Image, self._color_image_topic, self._on_rgb,
            qos_profile_sensor_data,
        )

        self._connected = True
        self._logger.info(
            f"RealSenseCamera connected: cloud={self._depth_topic}, "
            f"depth={self._depth_image_topic}, rgb={self._color_image_topic}"
        )
        return True

    def disconnect(self) -> bool:
        self._connected = False
        for sub in (self._cloud_sub, self._depth_sub, self._rgb_sub):
            if sub:
                self._node.destroy_subscription(sub)
        self._cloud_sub = self._depth_sub = self._rgb_sub = None
        return True

    def get_point_cloud(self) -> PointCloud2 | None:
        return self._latest_cloud

    def get_depth_image(self) -> Image | None:
        return self._latest_depth

    def get_rgb_image(self) -> Image | None:
        return self._latest_rgb

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth = msg

    def _on_rgb(self, msg: Image) -> None:
        self._latest_rgb = msg
