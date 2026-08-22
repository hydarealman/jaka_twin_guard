"""Continuously publish temporally stable 3D fruit targets.

This node reuses the existing point-cloud detector and ROI quality fusion but
removes all MoveIt/behaviour-tree dependencies.  Its output is the boundary
between vision and either control architecture.
"""

# ROS2 感知节点：持续接收点云、检测水果，并通过时间稳定性筛选
# 发布可靠的三维抓取目标。

from __future__ import annotations

import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from jaka_single_arm.perception import create_camera
from jaka_single_arm.perception.health_fusion import HealthFusion
from jaka_single_arm.perception.object_detector import ObjectDetector
from jaka_single_arm.perception.real_mode import validate_real_perception_config
from jaka_single_arm.perception.target_tracker import FruitObservation, FruitTargetTracker
from jaka_single_arm.perception.yolo_depth_localizer import YoloDepthLocalizer


def _load_config(filename: str) -> dict:
    path = os.path.join(get_package_share_directory("jaka_single_arm"), "config", filename)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


class FruitTargetNode(Node):
    def __init__(self):
        super().__init__("fruit_target_node")
        perception_cfg = _load_config("perception_params.yaml")
        scene_cfg = _load_config("scene_params.yaml")

        self.declare_parameter("camera_type", perception_cfg.get("camera_type", "realsense"))
        self.declare_parameter("output_frame", perception_cfg.get("output_frame", "Link_00"))
        self.declare_parameter("output_topic", "/perception/stable_fruit_targets")
        self.declare_parameter("process_rate", 5.0)
        self.declare_parameter(
            "localizer_backend",
            perception_cfg.get("localizer_backend", "geometry"),
        )
        self.declare_parameter(
            "sync_tolerance_s",
            perception_cfg.get("rgbd_sync_tolerance_s", 0.033),
        )
        self.declare_parameter(
            "fruit_detector_model",
            os.environ.get(
                "JAKA_FRUIT_DETECTOR_MODEL",
                perception_cfg.get("fruit_detector", {}).get("model", "yolov8n.pt"),
            ),
        )
        self.declare_parameter(
            "fruit_detector_confidence",
            perception_cfg.get("fruit_detector", {}).get("confidence", 0.25),
        )
        self.declare_parameter(
            "fruit_detector_iou",
            perception_cfg.get("fruit_detector", {}).get("iou", 0.50),
        )
        self.declare_parameter(
            "fruit_detector_image_size",
            perception_cfg.get("fruit_detector", {}).get("image_size", 640),
        )
        self.declare_parameter("allow_scene_fallback", False)
        self.declare_parameter("perception_license_mode", "development")
        self.declare_parameter("model_license_approved", False)
        self.declare_parameter("force_table_center_z", False)
        self.declare_parameter("enable_table_z_fallback", False)
        self.declare_parameter("real_mode", False)
        self.declare_parameter("enable_rgb_debug_candidates", False)
        self.declare_parameter("data_timeout_s", 1.0)
        self.declare_parameter(
            "camera_info_topic",
            perception_cfg.get("classifier", {}).get(
                "camera_info_topic", "/camera/camera/color/camera_info"
            ),
        )
        self.declare_parameter("stable_min_frames", 5)
        self.declare_parameter("stable_window_size", 7)
        self.declare_parameter("stable_position_std", 0.005)
        self.declare_parameter("association_distance", 0.06)
        self.declare_parameter("tracker_stale_after_s", 0.25)
        self.declare_parameter("kalman_measurement_std_m", 0.008)
        self.declare_parameter("kalman_acceleration_std_mps2", 1.5)
        self.declare_parameter("kalman_gate_sigma", 4.0)
        self.declare_parameter("publish_kalman_predictions", False)
        self.declare_parameter("max_kalman_prediction_age_s", 0.18)
        self.declare_parameter("stable_min_confidence", 0.60)
        self.declare_parameter("stable_min_detection_confidence", 0.60)
        self.declare_parameter("stable_class_majority", 0.75)
        self.declare_parameter("stable_min_known_ratio", 0.80)
        self.declare_parameter("stable_min_health_margin", 0.15)
        self.declare_parameter(
            "detection_roi_min", perception_cfg.get("detection_roi_min", [])
        )
        self.declare_parameter(
            "detection_roi_max", perception_cfg.get("detection_roi_max", [])
        )
        self.declare_parameter(
            "point_cloud_downsample",
            perception_cfg.get("realsense_camera", {}).get(
                "point_cloud_downsample", 2
            ),
        )
        for name, default in (
            ("voxel_leaf_size", 0.005),
            ("ransac_max_iterations", 200),
            ("cluster_tolerance", 0.015),
            ("min_cluster_size", 50),
            ("max_cluster_size", 5000),
        ):
            self.declare_parameter(name, perception_cfg.get(name, default))

        gp = self.get_parameter
        camera_type = str(gp("camera_type").value)
        real_mode = bool(gp("real_mode").value)
        localizer_backend = str(gp("localizer_backend").value).strip().lower()
        force_table_center_z = bool(gp("force_table_center_z").value)
        enable_table_z_fallback = bool(gp("enable_table_z_fallback").value)
        allow_scene_fallback = bool(gp("allow_scene_fallback").value)
        if real_mode:
            validate_real_perception_config(
                camera_type=camera_type,
                allow_scene_fallback=allow_scene_fallback,
                force_table_center_z=force_table_center_z,
                enable_table_z_fallback=enable_table_z_fallback,
            )
        perception_cfg = dict(perception_cfg)
        perception_cfg["camera_type"] = camera_type
        perception_cfg["output_frame"] = str(gp("output_frame").value)
        perception_cfg["localizer_backend"] = localizer_backend
        perception_cfg["rgbd_sync_tolerance_s"] = max(
            0.005, float(gp("sync_tolerance_s").value)
        )
        perception_cfg["drop_on_transform_failure"] = True
        perception_cfg["force_table_center_z"] = force_table_center_z
        perception_cfg["enable_table_z_fallback"] = enable_table_z_fallback
        perception_cfg["detection_roi_min"] = list(
            gp("detection_roi_min").value
        )
        perception_cfg["detection_roi_max"] = list(
            gp("detection_roi_max").value
        )
        for name in (
            "voxel_leaf_size",
            "ransac_max_iterations",
            "cluster_tolerance",
            "min_cluster_size",
            "max_cluster_size",
        ):
            perception_cfg[name] = gp(name).value
        realsense_cfg = dict(perception_cfg.get("realsense_camera", {}))
        realsense_cfg["point_cloud_downsample"] = int(
            gp("point_cloud_downsample").value
        )
        perception_cfg["realsense_camera"] = realsense_cfg
        detector_cfg = dict(perception_cfg.get("fruit_detector", {}))
        detector_cfg["model"] = str(gp("fruit_detector_model").value)
        detector_cfg["confidence"] = float(gp("fruit_detector_confidence").value)
        detector_cfg["iou"] = float(gp("fruit_detector_iou").value)
        detector_cfg["image_size"] = int(gp("fruit_detector_image_size").value)
        perception_cfg["fruit_detector"] = detector_cfg
        classifier = dict(perception_cfg.get("classifier", {}))
        classifier["allow_scene_fallback"] = allow_scene_fallback
        classifier["camera_info_topic"] = str(gp("camera_info_topic").value)
        classifier["license_mode"] = str(gp("perception_license_mode").value)
        classifier["model_license_approved"] = bool(
            gp("model_license_approved").value
        )
        classifier["enable_rgb_debug_candidates"] = bool(
            gp("enable_rgb_debug_candidates").value
        )
        perception_cfg["classifier"] = classifier

        self._output_frame = perception_cfg["output_frame"]
        self._camera = create_camera(self, perception_cfg, scene_cfg)
        self._camera.connect()
        self._detector = ObjectDetector(self, perception_cfg)
        self._fusion = HealthFusion(self, perception_cfg, scene_cfg)
        self._tracker = FruitTargetTracker(
            min_frames=int(gp("stable_min_frames").value),
            window_size=int(gp("stable_window_size").value),
            association_distance=float(gp("association_distance").value),
            max_position_std=float(gp("stable_position_std").value),
            class_majority=float(gp("stable_class_majority").value),
            min_confidence=float(gp("stable_min_confidence").value),
            min_detection_confidence=float(
                gp("stable_min_detection_confidence").value
            ),
            min_health_margin=float(gp("stable_min_health_margin").value),
            min_known_ratio=float(gp("stable_min_known_ratio").value),
            stale_after=float(gp("tracker_stale_after_s").value),
            kalman_measurement_std_m=float(
                gp("kalman_measurement_std_m").value
            ),
            kalman_acceleration_std_mps2=float(
                gp("kalman_acceleration_std_mps2").value
            ),
            kalman_gate_sigma=float(gp("kalman_gate_sigma").value),
            publish_kalman_predictions=bool(
                gp("publish_kalman_predictions").value
            ),
            max_prediction_age_s=float(
                gp("max_kalman_prediction_age_s").value
            ),
        )
        self._localizer_backend = localizer_backend
        self._sync_tolerance_s = max(0.005, float(gp("sync_tolerance_s").value))
        self._localizer = None
        if self._localizer_backend == "yolo_depth":
            self._localizer = YoloDepthLocalizer(
                self,
                perception_cfg,
                camera_frame=str(
                    perception_cfg.get("realsense_camera", {}).get(
                        "camera_frame", "camera_color_optical_frame"
                    )
                ),
            )
        elif self._localizer_backend not in ("geometry", "pointcloud", "legacy"):
            raise ValueError(
                "localizer_backend must be geometry, pointcloud, legacy, or yolo_depth"
            )
        self._publisher = self.create_publisher(
            Detection3DArray, str(gp("output_topic").value), 10
        )
        self._last_cloud_stamp = None
        self._last_rgbd_stamp = None
        self._real_mode = real_mode
        self._data_timeout_s = max(0.1, float(gp("data_timeout_s").value))
        self._last_stale_warning = 0.0
        self._last_tracker_warning = 0.0
        rate = max(0.2, float(gp("process_rate").value))
        self._process_event = threading.Event()
        self._worker_stop = threading.Event()
        self._tracker_lock = threading.Lock()
        self._health_lock = threading.Lock()
        self._health_event = threading.Event()
        self._health_generation = 0
        self._health_job = None
        self._worker_thread = threading.Thread(
            target=self._processing_loop,
            name="fruit_perception_worker",
            daemon=True,
        )
        self._worker_thread.start()
        self._health_thread = threading.Thread(
            target=self._health_processing_loop,
            name="fruit_health_latest_worker",
            daemon=True,
        )
        self._health_thread.start()
        # The ROS callback only wakes the worker. Inference and point-cloud
        # work therefore cannot starve the camera subscriptions.
        self._timer = self.create_timer(1.0 / rate, self._schedule_process)
        self.get_logger().info(
            f"Stable fruit target node: camera={perception_cfg['camera_type']}, "
            f"backend={self._localizer_backend}, frame={self._output_frame}, "
            f"rate={rate:.1f}Hz, topic={gp('output_topic').value}"
        )

    def _schedule_process(self) -> None:
        self._process_event.set()

    def _processing_loop(self) -> None:
        while not self._worker_stop.is_set():
            self._process_event.wait(timeout=0.5)
            self._process_event.clear()
            if self._worker_stop.is_set():
                break
            try:
                self._process()
            except Exception as exc:
                self.get_logger().error("Perception worker failed: %s" % exc)

    def _enqueue_health_job(self, objects, rgb, depth_stamp, sync_delta_s) -> None:
        """Replace any queued quality job; never classify a backlog of old frames."""
        with self._health_lock:
            self._health_generation += 1
            self._health_job = (
                self._health_generation,
                objects,
                rgb,
                depth_stamp,
                sync_delta_s,
            )
            self._health_event.set()

    def _invalidate_health_jobs(self) -> None:
        with self._health_lock:
            self._health_generation += 1
            self._health_job = None

    def _health_processing_loop(self) -> None:
        """Run the slower MobileNet/tracker path independently of YOLO."""
        while not self._worker_stop.is_set():
            self._health_event.wait(timeout=0.5)
            if self._worker_stop.is_set():
                break
            while not self._worker_stop.is_set():
                with self._health_lock:
                    job = self._health_job
                    self._health_job = None
                    if job is None:
                        self._health_event.clear()
                        break
                try:
                    self._process_health_job(*job)
                except Exception as exc:
                    self.get_logger().error("Health worker failed: %s" % exc)

    def _process_health_job(
        self, generation, objects, rgb, depth_stamp, sync_delta_s
    ) -> None:
        self._fusion.fuse(objects, source_stamp=depth_stamp, rgb_image=rgb)
        # The classifier may have been working while YOLO received a newer
        # image.  Its debug image can keep its honest old source stamp, but an
        # obsolete result must never update the grasp tracker or target topic.
        with self._health_lock:
            if generation != self._health_generation:
                return

        observations = [
            FruitObservation(
                x=obj.centroid[0], y=obj.centroid[1], z=obj.centroid[2],
                radius=obj.radius, health=obj.health,
                detection_confidence=obj.confidence,
                health_confidence=obj.health_confidence,
                health_margin=getattr(obj, "health_margin", 0.0),
            )
            for obj in objects
        ]
        with self._tracker_lock:
            stable = self._tracker.update(
                observations, self._stamp_seconds(depth_stamp)
            )
        if not stable:
            now = time.monotonic()
            if objects and now - self._last_tracker_warning >= 2.0:
                self.get_logger().info(
                    "Tracker waiting: observations=%d reason=%s"
                    % (
                        len(observations),
                        getattr(self._tracker, "last_rejection_reason", "unknown"),
                    )
                )
                self._last_tracker_warning = now
            self._publish_empty(rgb.header)
            return
        self._publish_stable_targets(stable, rgb.header)
        self.get_logger().debug(
            "Latest health frame accepted: detections=%d stable=%d sync_delta=%.1fms"
            % (len(objects), len(stable), sync_delta_s * 1000.0)
        )

    def _process(self) -> None:
        if self._localizer_backend == "yolo_depth":
            self._process_rgbd()
            return

        # 点云数据获取与防重
        cloud = self._camera.get_point_cloud()
        cloud_age = self._camera.get_point_cloud_age_s()
        rgb = self._camera.get_rgb_image()
        rgb_age = self._camera.get_rgb_image_age_s()
        if (
            cloud is None
            or cloud_age > self._data_timeout_s
            or rgb is None
            or rgb_age > self._data_timeout_s
        ):
            self._reset_tracking()
            self._detector.clear_markers()
            self._publish_empty()
            now = time.monotonic()
            if now - self._last_stale_warning >= 2.0:
                self.get_logger().warning(
                    "Camera data unavailable/stale: cloud_age=%.3fs rgb_age=%.3fs"
                    % (cloud_age, rgb_age)
                )
                self._last_stale_warning = now
            return
        stamp_key = (cloud.header.stamp.sec, cloud.header.stamp.nanosec)
        if stamp_key == self._last_cloud_stamp:
            return
        self._last_cloud_stamp = stamp_key

        # 几何检测与分类融合
        objects = self._detector.process(cloud)
        if not objects:
            # Publish the fresh real RGB image even when no fruit passes the
            # geometry gate. This distinguishes "detector running, 0 fruit"
            # from a broken annotation pipeline in the debug window.
            self._fusion.fuse(
                [], source_stamp=cloud.header.stamp, rgb_image=rgb
            )
            self._detector.clear_markers(cloud.header)
            self._tracker.update([], self._stamp_seconds(cloud.header.stamp))
            self._publish_empty(cloud.header)
            return
        self._fusion.fuse(
            objects,
            source_stamp=cloud.header.stamp,
            rgb_image=rgb,
        )
        self._detector.republish_latest_markers()

        # 数据格式转换与时间跟踪
        observations = [
            FruitObservation(
                x=obj.centroid[0], y=obj.centroid[1], z=obj.centroid[2],
                radius=obj.radius, health=obj.health,
                detection_confidence=obj.confidence,
                health_confidence=obj.health_confidence,
                health_margin=getattr(obj, "health_margin", 0.0),
            )
            for obj in objects
        ]

        """
        把当前检测到的所有水果候选喂给跟踪器
        经过多帧历史数据验证海后,只返回那些
        真正可看的稳定目标,如果没有任何目标通过验证
        则提前终止本帧发布
        """
        stable = self._tracker.update(observations, self._stamp_seconds(cloud.header.stamp))
        if not stable:
            self._publish_empty(cloud.header)
            return

        self._publish_stable_targets(stable, cloud.header)

    def _process_rgbd(self) -> None:
        """Process the newest RGB/aligned-depth pair without blocking callbacks."""
        pair = self._camera.get_synced_rgbd(self._sync_tolerance_s)
        if pair is None:
            self._invalidate_health_jobs()
            self._reset_tracking()
            self._detector.clear_markers()
            self._publish_empty()
            now = time.monotonic()
            if now - self._last_stale_warning >= 2.0:
                self.get_logger().warning(
                    "No synchronized RGB/aligned-depth pair within %.0f ms"
                    % (self._sync_tolerance_s * 1000.0)
                )
                self._last_stale_warning = now
            return

        rgb, depth, color_info, delta_s = pair
        stamp_key = (depth.header.stamp.sec, depth.header.stamp.nanosec)
        if stamp_key == self._last_rgbd_stamp:
            return
        self._last_rgbd_stamp = stamp_key

        try:
            objects = self._localizer.process(rgb, depth, color_info)
        except Exception as exc:
            self._detector.clear_markers(rgb.header)
            self._publish_empty(rgb.header)
            now = time.monotonic()
            if now - self._last_stale_warning >= 2.0:
                self.get_logger().warning("RGB-D detector unavailable: %s" % exc)
                self._last_stale_warning = now
            return

        if objects:
            self._detector.publish_objects(objects, rgb.header)
        else:
            self._detector.clear_markers(rgb.header)
        self._enqueue_health_job(
            objects, rgb, depth.header.stamp, delta_s
        )

    def _publish_stable_targets(self, stable, source_header) -> None:
        """Pack stable tracker output while keeping the existing ROS interface."""
        output = Detection3DArray()
        output.header = source_header
        output.header.frame_id = self._output_frame
        for target in stable:
            detection = Detection3D()
            detection.header = output.header
            detection.id = target.track_id
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = target.health
            hypothesis.hypothesis.score = float(target.confidence)
            hypothesis.pose.pose.position.x = target.centroid[0]
            hypothesis.pose.pose.position.y = target.centroid[1]
            hypothesis.pose.pose.position.z = target.centroid[2]
            hypothesis.pose.pose.orientation.w = 1.0
            detection.results.append(hypothesis)
            detection.bbox.center.position.x = target.centroid[0]
            detection.bbox.center.position.y = target.centroid[1]
            detection.bbox.center.position.z = target.centroid[2]
            detection.bbox.center.orientation.w = 1.0
            diameter = target.radius * 2.0
            detection.bbox.size.x = diameter
            detection.bbox.size.y = diameter
            detection.bbox.size.z = diameter
            output.detections.append(detection)
        # Preserve the acquisition timestamp.  KF prediction, downstream
        # freshness gates and conveyor interception must all refer to the
        # physical measurement time rather than publication/processing time.
        self._publisher.publish(output)
        self.get_logger().info(
            "Published %d stable fruit target(s) in %s "
            "tracking=%d coasting=%d max_age=%.0fms"
            % (
                len(output.detections),
                self._output_frame,
                sum(1 for target in stable if target.phase == "tracking"),
                sum(1 for target in stable if target.phase == "coasting"),
                max((target.measurement_age for target in stable), default=0.0)
                * 1000.0,
            )
        )

    def _reset_tracking(self) -> None:
        with self._tracker_lock:
            self._tracker.reset()
        self._last_cloud_stamp = None
        self._last_rgbd_stamp = None

    def _publish_empty(self, source_header=None) -> None:
        output = Detection3DArray()
        if source_header is not None:
            output.header = source_header
        else:
            output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self._output_frame
        output.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(output)

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def destroy_node(self):
        self._worker_stop.set()
        self._process_event.set()
        self._health_event.set()
        worker = getattr(self, "_worker_thread", None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        health_worker = getattr(self, "_health_thread", None)
        if health_worker is not None and health_worker.is_alive():
            health_worker.join(timeout=2.0)
        self._camera.disconnect()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FruitTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
