#!/usr/bin/env python3
"""Detect Objects Skill — run perception pipeline and store results in blackboard."""

from __future__ import annotations

from typing import Optional

import rclpy
from trajectory_msgs.msg import JointTrajectory

from jaka_single_arm.skills.base_skill import BaseSkill, SkillResult


class DetectObjectsSkill(BaseSkill):
    """Run object detection and write results to blackboard.

    Writes to blackboard:
        detected_objects: list[DetectedObject]
        detection_count: int
    """

    def __init__(self):
        super().__init__()
        self._detector = None  # ObjectDetector

    def configure(self, node, planner, params: dict, blackboard: dict = None):
        super().configure(node, planner, params, blackboard)
        # Object detector is set by the runner via set_detector()
        self._detector = None

    def set_detector(self, detector):
        """Inject the object detector instance."""
        self._detector = detector

    def plan(self) -> Optional[JointTrajectory]:
        """Detect objects — no motion required, returns dummy trajectory."""
        if self._detector is None:
            self._log("No detector configured")
            return None

        # Get latest point cloud from camera
        camera = self._blackboard.get("camera")
        if camera is None:
            self._log("No camera in blackboard")
            return None

        cloud = camera.get_point_cloud()
        if cloud is None:
            # Try waiting for data (spin) — Gazebo startup may need extra time
            wait_timeout = self._get_param("data_wait_timeout", 10.0)
            deadline = self._node.get_clock().now().nanoseconds / 1e9 + wait_timeout
            while cloud is None and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
                cloud = camera.get_point_cloud()
                if self._node.get_clock().now().nanoseconds / 1e9 > deadline:
                    break

        if cloud is None:
            self._log("No point cloud data available")
            return None

        # Run detection
        objects = self._detector.process(cloud)
        if not objects:
            self._log("No objects detected")
            return None

        # 融合苹果好坏识别结果（YOLO → 每个物体 health 标签；无检测走场景提示兜底）
        fusion = self._blackboard.get("health_fusion")
        if fusion is not None:
            # 先 spin 几次，让 detections/camera_info 订阅拿到最新帧
            for _ in range(5):
                rclpy.spin_once(self._node, timeout_sec=0.05)
            fusion.fuse(objects)

        # Write to blackboard
        self._blackboard["detected_objects"] = objects
        self._blackboard["detection_count"] = len(objects)

        self._log(f"Detected {len(objects)} objects")
        for obj in objects:
            self._log(f"  {obj.id}: ({obj.centroid[0]:.3f}, {obj.centroid[1]:.3f}, "
                       f"{obj.centroid[2]:.3f}) r={obj.radius:.3f} shape={obj.shape} "
                       f"health={obj.health}")

        # Return empty trajectory (no arm movement for detection)
        return JointTrajectory()

    def execute(self, trajectory: JointTrajectory) -> bool:
        # Detection doesn't move the arm
        return True

    def run(self) -> SkillResult:
        result = super().run()
        if result == SkillResult.SUCCESS:
            return SkillResult.SUCCESS
        return SkillResult.DETECT_FAILED
