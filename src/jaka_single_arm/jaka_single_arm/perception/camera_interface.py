#!/usr/bin/env python3
"""Camera Interface — abstract base class for swappable depth cameras.

Supports: MockCamera, GazeboCamera, RealSenseCamera, and future additions.
Camera type selected via perception_params.yaml → camera_type field.

Reference:
  - ros2_moveit2_ur5e_grasp — modular vision package design
  - librealsense2 ROS2 driver — standard RealSense topic conventions
  - Gazebo ROS2 RGBD plugin — standard Gazebo camera topic conventions
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2


class CameraInterface(ABC):
    """Abstract depth camera interface.

    All camera implementations publish standardized topics and expose
    the same API. Swap cameras by changing camera_type in YAML config.

    Standardized output topics:
      /perception/point_cloud  — PointCloud2 (XYZ)
      /perception/depth_image  — Image (16UC1 or 32FC1)
      /perception/rgb_image    — Image (RGB8)
    """

    def __init__(self, node: Node, config: dict):
        """
        Args:
            node: ROS2 node for creating publishers/subscribers.
            config: perception_params.yaml dict.
        """
        self._node = node
        self._config = config
        self._logger = node.get_logger()
        self._camera_frame = config.get("camera_frame", "camera_depth_frame")
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def camera_frame(self) -> str:
        return self._camera_frame

    @abstractmethod
    def connect(self) -> bool:
        """Initialize camera connection. Returns True on success."""
        ...

    @abstractmethod
    def disconnect(self) -> bool:
        """Close camera connection. Returns True on success."""
        ...

    @abstractmethod
    def get_point_cloud(self) -> Optional[PointCloud2]:
        """Get latest point cloud. Returns None if no data available."""
        ...

    @abstractmethod
    def get_depth_image(self) -> Optional[Image]:
        """Get latest depth image. Returns None if no data available."""
        ...

    @abstractmethod
    def get_rgb_image(self) -> Optional[Image]:
        """Get latest color image. Returns None if no data available."""
        ...
