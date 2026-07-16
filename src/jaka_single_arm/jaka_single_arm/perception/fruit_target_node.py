"""Continuously publish temporally stable 3D fruit targets.

This node reuses the existing point-cloud detector and YOLO health fusion but
removes all MoveIt/behaviour-tree dependencies.  Its output is the boundary
between vision and either control architecture.
"""

from __future__ import annotations

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from jaka_single_arm.perception import create_camera
from jaka_single_arm.perception.health_fusion import HealthFusion
from jaka_single_arm.perception.object_detector import ObjectDetector
from jaka_single_arm.perception.target_tracker import FruitObservation, FruitTargetTracker


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
        self.declare_parameter("output_frame", perception_cfg.get("output_frame", "base_link"))
        self.declare_parameter("output_topic", "/perception/stable_fruit_targets")
        self.declare_parameter("process_rate", 5.0)
        self.declare_parameter("allow_scene_fallback", False)
        self.declare_parameter("force_table_center_z", False)
        self.declare_parameter("enable_table_z_fallback", False)
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
        self.declare_parameter("stable_min_confidence", 0.55)

        gp = self.get_parameter
        perception_cfg = dict(perception_cfg)
        perception_cfg["camera_type"] = str(gp("camera_type").value)
        perception_cfg["output_frame"] = str(gp("output_frame").value)
        perception_cfg["drop_on_transform_failure"] = True
        perception_cfg["force_table_center_z"] = bool(gp("force_table_center_z").value)
        perception_cfg["enable_table_z_fallback"] = bool(
            gp("enable_table_z_fallback").value
        )
        classifier = dict(perception_cfg.get("classifier", {}))
        classifier["allow_scene_fallback"] = bool(gp("allow_scene_fallback").value)
        classifier["camera_info_topic"] = str(gp("camera_info_topic").value)
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
            min_confidence=float(gp("stable_min_confidence").value),
        )
        self._publisher = self.create_publisher(
            Detection3DArray, str(gp("output_topic").value), 10
        )
        self._last_cloud_stamp = None
        rate = max(0.2, float(gp("process_rate").value))
        self._timer = self.create_timer(1.0 / rate, self._process)
        self.get_logger().info(
            f"Stable fruit target node: camera={perception_cfg['camera_type']}, "
            f"frame={self._output_frame}, topic={gp('output_topic').value}"
        )

    def _process(self) -> None:
        cloud = self._camera.get_point_cloud()
        if cloud is None:
            return
        stamp_key = (cloud.header.stamp.sec, cloud.header.stamp.nanosec)
        if stamp_key == self._last_cloud_stamp:
            return
        self._last_cloud_stamp = stamp_key
        objects = self._detector.process(cloud)
        if not objects:
            self._tracker.update([], self._stamp_seconds(cloud.header.stamp))
            return
        self._fusion.fuse(objects, source_stamp=cloud.header.stamp)
        observations = [
            FruitObservation(
                x=obj.centroid[0], y=obj.centroid[1], z=obj.centroid[2],
                radius=obj.radius, health=obj.health,
                detection_confidence=obj.confidence,
                health_confidence=obj.health_confidence,
            )
            for obj in objects
        ]
        stable = self._tracker.update(observations, self._stamp_seconds(cloud.header.stamp))
        if not stable:
            return

        output = Detection3DArray()
        output.header = cloud.header
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
        self._publisher.publish(output)
        self.get_logger().info(
            f"Published {len(output.detections)} stable fruit target(s) in {self._output_frame}"
        )

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def destroy_node(self):
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
