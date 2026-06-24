#!/usr/bin/env python3
"""Perception Interface — 感知系统抽象接口。

当前是 Mock 实现（发布固定位姿），后续可替换为：
- YOLO + PCL 点云 → 6DoF 位姿
- AprilTag / Aruco 标记
- 深度学习 Grasp Pose Detection (GPD)

参考:
  - ARIAC 2024 RGBD 相机 + 逻辑相机 + 物体检测
  - moveit_grasps (PickNik) — 6DoF 抓取候选生成与评分
  - UW Bimanual System — Task Space Regions (TSR) 约束感知
"""

from __future__ import annotations

from geometry_msgs.msg import Point, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray


class PerceptionInterface:
    """感知接口 — 提供物体 6DoF 位姿。

    设计原则：
    - 所有操作目标由感知模块动态提供，禁止硬编码坐标
    - 当前 Mock 实现发布固定位姿，方便开发和调试
    - 生产环境替换 detect() 方法即可
    """

    def __init__(self, world_frame: str = "world"):
        self._world_frame = world_frame
        self._objects: dict[str, Pose] = {}

    def add_mock_object(
        self,
        object_id: str,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        size: tuple[float, float, float] = (0.1, 0.1, 0.1),
    ):
        """注册一个 Mock 物体。"""
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        self._objects[object_id] = pose

    def detect(self, object_id: str) -> Pose | None:
        """检测物体并返回 6DoF 位姿。

        当前 Mock 实现返回预注册的位姿。
        生产环境替换为相机检测逻辑。
        """
        return self._objects.get(object_id)

    def detect_all(self) -> dict[str, Pose]:
        """检测所有已知物体。"""
        return dict(self._objects)

    def set_object_pose(self, object_id: str, pose: Pose):
        """外部更新物体位姿（如从话题回调）。"""
        self._objects[object_id] = pose

    def get_object_marker(
        self, object_id: str, marker_id: int = 0
    ) -> Marker | None:
        """生成物体的 RViz 可视化 Marker。"""
        pose = self._objects.get(object_id)
        if pose is None:
            return None

        marker = Marker()
        marker.header.frame_id = self._world_frame
        marker.ns = "perception"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color.r = 1.0
        marker.color.g = 0.65
        marker.color.b = 0.0
        marker.color.a = 0.8
        return marker
