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

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

# 继承自ABC 表明这是一个抽象基类,不能直接实例化
"""
为深度相机提供统一的软件接口
让上层应用能够无缝切换不同类型的深度相机:
    模拟相机
    Gazebo仿真相机
    RealSense实感相机
"""
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
        self._node = node                                  # ROS2节点实例,用于创建发布者,订阅者,日志等
        self._config = config                              # 配置
        self._logger = node.get_logger()                   # 获取ROS2节点的日志记录器
        self._camera_frame = config.get(                   # 从配置字典config读取相机坐标系名称,并存入实例变量
            "camera_frame", "camera_color_optical_frame"
        )
        self._connected = False

    # 返回连接状态
    @property
    def connected(self) -> bool:
        return self._connected

    # 返回相机帧ID
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
