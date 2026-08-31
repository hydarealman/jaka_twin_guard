#!/usr/bin/env python3
"""Detect Objects Skill — run perception pipeline and store results in blackboard."""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from trajectory_msgs.msg import JointTrajectory

from fruit_picking_arm.skills.base_skill import BaseSkill, SkillResult


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
        if self._blackboard.get("external_perception", False):
            return self._plan_external_stable_targets()
        if self._detector is None:
            self._log("No detector configured")
            return None

        # Get latest point cloud from camera
        camera = self._blackboard.get("camera")
        if camera is None:
            self._log("No camera in blackboard")
            return None

        backend = str(
            self._blackboard.get("perception_config", {}).get(
                "localizer_backend", "geometry"
            )
        ).strip().lower()
        if backend == "yolo_depth":
            return self._plan_rgbd(camera)

        if not self._camera_data_is_fresh(camera):
            self._blackboard["detected_objects"] = []
            self._blackboard["detection_count"] = 0
            self._log("Real camera data is missing or stale")
            return None

        cloud = camera.get_point_cloud()
        if cloud is None:
            # Try waiting for data (spin) — Gazebo startup may need extra time
            wait_timeout = self._get_param("data_wait_timeout", 10.0)
            deadline = time.monotonic() + wait_timeout
            while cloud is None and rclpy.ok():
                self._planner.spin_callbacks_once(timeout_sec=0.1)
                cloud = camera.get_point_cloud()
                if time.monotonic() > deadline:
                    break

        if cloud is None:
            self._log("No point cloud data available")
            return None

        if not self._camera_data_is_fresh(camera):
            self._blackboard["detected_objects"] = []
            self._blackboard["detection_count"] = 0
            self._log("Real camera data became stale while waiting for cloud")
            return None

        # Give the TransformListener time to receive /tf_static before the
        # first point cloud is processed. Wall time keeps this bounded while
        # Gazebo is paused and simulated time is not advancing.
        source_frame = cloud.header.frame_id
        tf_wait_timeout = self._get_param("tf_wait_timeout", 5.0)
        tf_deadline = time.monotonic() + tf_wait_timeout
        while (
            not self._detector.transform_ready(source_frame)
            and rclpy.ok()
            and time.monotonic() < tf_deadline
        ):
            self._planner.spin_callbacks_once(timeout_sec=0.1)

        if not self._detector.transform_ready(source_frame):
            self._log(
                f"TF not ready: {source_frame} -> detector output frame"
            )
            return None

        # Run detection
        objects = self._detector.process(cloud)
        if not objects:
            self._log("No objects detected")
            return None

        # Classify only the RGB ROI projected from each point-cloud object.
        fusion = self._blackboard.get("health_fusion")
        if fusion is not None:
            # 先 spin 几次，让 detections/camera_info 订阅拿到最新帧
            for _ in range(5):
                self._planner.spin_callbacks_once(timeout_sec=0.05)
            if not self._camera_data_is_fresh(camera):
                self._blackboard["detected_objects"] = []
                self._blackboard["detection_count"] = 0
                self._log("Real RGB/depth data became stale before classification")
                return None
            fusion.fuse(
                objects,
                source_stamp=cloud.header.stamp,
                rgb_image=camera.get_rgb_image(),
            )
            if hasattr(self._detector, "republish_latest_markers"):
                self._detector.republish_latest_markers()

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

    def _plan_external_stable_targets(self) -> Optional[JointTrajectory]:
        """Consume the same KF-stable RGB-D targets used by RViz/production."""
        wait_timeout = max(0.1, float(self._get_param("data_wait_timeout", 15.0)))
        deadline = time.monotonic() + wait_timeout
        objects = list(self._blackboard.get("external_detected_objects", []))
        while not objects and rclpy.ok() and time.monotonic() < deadline:
            self._planner.spin_callbacks_once(timeout_sec=0.1)
            objects = list(self._blackboard.get("external_detected_objects", []))
        if not objects:
            self._clear_results()
            self._log("No stable health-classified external fruit target")
            return None
        self._blackboard["detected_objects"] = objects
        self._blackboard["detection_count"] = len(objects)
        self._log("Accepted %d KF-stable external target(s)" % len(objects))
        return JointTrajectory()

    def _plan_rgbd(self, camera) -> Optional[JointTrajectory]:
        """Run the same registered RGB-D backend used by D455 debug."""
        localizer = self._blackboard.get("rgbd_localizer")
        if localizer is None:
            self._log("No RGB-D localizer configured")
            return None

        config = self._blackboard.get("perception_config", {})
        sync_delta = max(0.0, float(config.get("rgbd_sync_tolerance_s", 0.033)))
        wait_timeout = max(0.0, float(self._get_param("data_wait_timeout", 10.0)))
        deadline = time.monotonic() + wait_timeout
        pair = camera.get_synced_rgbd(sync_delta)
        while pair is None and rclpy.ok() and time.monotonic() < deadline:
            self._planner.spin_callbacks_once(timeout_sec=0.1)
            pair = camera.get_synced_rgbd(sync_delta)
        if pair is None:
            self._clear_results()
            self._log(
                "No synchronized RGB/aligned-depth pair within %.0f ms"
                % (sync_delta * 1000.0)
            )
            return None

        rgb, depth, color_info, _ = pair
        timeout = max(0.1, float(config.get("data_timeout_s", 1.0)))
        if (
            camera.get_rgb_image_age_s() > timeout
            or camera.get_aligned_depth_image_age_s() > timeout
        ):
            self._clear_results()
            self._log("Synchronized RGB-D pair is stale")
            return None
        objects = localizer.process(rgb, depth, color_info)
        if objects:
            self._detector.publish_objects(objects, rgb.header)
        else:
            self._detector.clear_markers(rgb.header)

        fusion = self._blackboard.get("health_fusion")
        if fusion is not None:
            fusion.fuse(objects, source_stamp=depth.header.stamp, rgb_image=rgb)
            if objects:
                self._detector.republish_latest_markers()

        # Unknown/weak health is not a graspable production target.
        objects = [obj for obj in objects if str(obj.health).lower() != "unknown"]
        self._blackboard["detected_objects"] = objects
        self._blackboard["detection_count"] = len(objects)
        if not objects:
            self._log("No health-classified RGB-D fruit detected")
            return None
        self._log("Detected %d RGB-D objects" % len(objects))
        return JointTrajectory()

    def _clear_results(self) -> None:
        self._blackboard["detected_objects"] = []
        self._blackboard["detection_count"] = 0

    def execute(self, trajectory: JointTrajectory) -> bool:
        # Detection doesn't move the arm
        return True

    def _camera_data_is_fresh(self, camera) -> bool:
        if self._blackboard.get("simulation_mode", False):
            return True
        config = self._blackboard.get("perception_config", {})
        timeout = max(0.1, float(config.get("data_timeout_s", 1.0)))
        return (
            camera.get_point_cloud_age_s() <= timeout
            and camera.get_rgb_image_age_s() <= timeout
        )

    def run(self) -> SkillResult:
        result = super().run()
        if result == SkillResult.SUCCESS:
            return SkillResult.SUCCESS
        return SkillResult.DETECT_FAILED
