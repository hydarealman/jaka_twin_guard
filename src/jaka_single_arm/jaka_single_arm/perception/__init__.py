#!/usr/bin/env python3
"""Perception module — camera factory and convenience exports.

Creates the appropriate CameraInterface implementation based on
perception_params.yaml → camera_type field.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

"""
类型检查阶段为True 运行时为False
避免循环导入
减少运行时开销
"""
if TYPE_CHECKING:
    from rclpy.node import Node
    from jaka_single_arm.perception.camera_interface import CameraInterface

"""
工厂函数:
用于根据配置文件动态创建对应的相机对象实例
工厂模式的一种简化模式: 将对象创建逻辑集中在一个函数中,客户代码只依赖抽象接口,而不依赖具体类
"""
def create_camera(node: Node, config: dict, scene_cfg: dict = None) -> CameraInterface:
    """Factory: create camera instance based on YAML config.

    Args:
        node: ROS2 node for pub/sub creation.
        config: perception_params.yaml dict.
        scene_cfg: scene_params.yaml dict (needed by MockCamera for object positions).

    Returns:
        CameraInterface implementation instance.

    Raises:
        ValueError: if camera_type is unknown.
    """
    camera_type = config.get("camera_type", "mock").lower()
    node.get_logger().info(f"Creating camera: type={camera_type}")

    if camera_type == "mock":
        from jaka_single_arm.perception.mock_camera import MockCamera
        return MockCamera(node, config, scene_cfg)
    elif camera_type == "gazebo":
        from jaka_single_arm.perception.gazebo_camera import GazeboCamera
        return GazeboCamera(node, config, scene_cfg)
    elif camera_type == "realsense":
        from jaka_single_arm.perception.realsense_camera import RealSenseCamera
        return RealSenseCamera(node, config, scene_cfg)
    else:
        raise ValueError(
            f"Unknown camera_type '{camera_type}'. "
            f"Valid: mock, gazebo, realsense"
        )
