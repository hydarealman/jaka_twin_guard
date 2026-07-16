#!/usr/bin/env python3
"""Gazebo Camera — interfaces with Gazebo RGBD sensor plugin.

Subscribes to standard Gazebo ROS2 RGBD plugin topics and bridges
them into the CameraInterface abstraction.

The simulated topics intentionally match the RealSense ROS 2 defaults:
  /camera/camera/depth/color/points    — PointCloud2
  /camera/camera/depth/image_rect_raw  — depth Image
  /camera/camera/color/image_raw       — color Image
"""

from __future__ import annotations

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

from jaka_single_arm.perception.camera_interface import CameraInterface


class GazeboCamera(CameraInterface):
    """Bridges Gazebo RGBD sensor plugin topics into CameraInterface.

    Usage:
        camera = GazeboCamera(node, perception_cfg)
        camera.connect()
        cloud = camera.get_point_cloud()  # latest frame
    """

    def __init__(self, node: Node, config: dict, scene_cfg: dict = None):
        super().__init__(node, config)
        gz_cfg = config.get("gazebo_camera", {})

        self._depth_topic = gz_cfg.get(
            "depth_topic", "/camera/camera/depth/color/points"
        )
        self._depth_image_topic = gz_cfg.get(
            "depth_image_topic", "/camera/camera/depth/image_rect_raw"
        )
        self._color_image_topic = gz_cfg.get(
            "color_image_topic", "/camera/camera/color/image_raw"
        )

        # Latest data buffers
        self._latest_cloud: PointCloud2 | None = None
        self._latest_depth: Image | None = None
        self._latest_rgb: Image | None = None

        # Subscribers (created in connect())
        self._cloud_sub = None
        self._depth_sub = None
        self._rgb_sub = None
        self._bridge_pub = None  # re-publish to /perception/point_cloud

    def connect(self) -> bool:
        if self._connected:
            return True

        self._cloud_sub = self._node.create_subscription(
            PointCloud2, self._depth_topic, self._on_cloud, 10
        )
        self._depth_sub = self._node.create_subscription(
            Image, self._depth_image_topic, self._on_depth, 10
        )
        self._rgb_sub = self._node.create_subscription(
            Image, self._color_image_topic, self._on_rgb, 10
        )

        # Bridge publisher — re-publish Gazebo cloud to the standard perception topic
        # so RViz can show it without needing to know which camera backend is active.
        self._bridge_pub = self._node.create_publisher(
            PointCloud2,
            self._config.get("point_cloud_topic", "/perception/point_cloud"),
            10,
        )

        self._connected = True
        self._logger.info(
            f"GazeboCamera connected: cloud={self._depth_topic}, "
            f"depth={self._depth_image_topic}, rgb={self._color_image_topic}"
        )
        return True

    def disconnect(self) -> bool:
        self._connected = False
        for sub in (self._cloud_sub, self._depth_sub, self._rgb_sub):
            if sub:
                self._node.destroy_subscription(sub)
        self._cloud_sub = self._depth_sub = self._rgb_sub = None
        if self._bridge_pub:
            self._node.destroy_publisher(self._bridge_pub)
            self._bridge_pub = None
        return True

    def get_point_cloud(self) -> PointCloud2 | None:
        return self._latest_cloud

    def get_depth_image(self) -> Image | None:
        return self._latest_depth

    def get_rgb_image(self) -> Image | None:
        return self._latest_rgb

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg
        # Re-publish to standard perception topic for RViz
        if self._bridge_pub:
            self._bridge_pub.publish(msg)

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth = msg

    def _on_rgb(self, msg: Image) -> None:
        self._latest_rgb = msg
