#!/usr/bin/env python3
"""苹果好坏识别 ROS2 节点（原生 ROS2 重构，替代队友的 ROS1/OpenVINO 版本）。

订阅相机 RGB 图，用 YoloDetector 做 Healthy/Unhealthy 检测，发布：
  - /perception/fruit_detections  (vision_msgs/Detection2DArray)  标准检测结果
  - /perception/health_annotated  (sensor_msgs/Image, bgr8)       标注调试图

可独立运行：
  ros2 run jaka_single_arm fruit_detector_node \
      --ros-args -p image_topic:=/camera/camera/color/image_raw -p backend:=onnxruntime

参数（见 config/perception_params.yaml::classifier）：
  backend, model_onnx, model_pt, data_yaml, conf, nms,
  image_topic, detections_topic, annotated_topic
"""

from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from jaka_single_arm.perception.yolo_infer import YoloDetector


def _default_model_dir() -> str:
    """定位已安装的 models 目录（share/jaka_single_arm/models）。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("jaka_single_arm"), "models")
    except Exception:
        return os.path.join(os.path.dirname(__file__), "..", "..", "models")


def _load_classifier_cfg() -> dict:
    """从 perception_params.yaml 读取 classifier 段作为默认参数。"""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory("jaka_single_arm"),
            "config", "perception_params.yaml",
        )
        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        return cfg.get("classifier", {}) or {}
    except Exception:
        return {}


class FruitDetectorNode(Node):
    def __init__(self):
        super().__init__("fruit_detector_node")

        model_dir = _default_model_dir()
        cfg = _load_classifier_cfg()

        def pick(key, fallback):
            v = cfg.get(key, "")
            return v if v not in ("", None) else fallback

        self.declare_parameter("backend", cfg.get("backend", "onnxruntime"))
        self.declare_parameter("model_onnx", pick("model_onnx", os.path.join(model_dir, "best.onnx")))
        self.declare_parameter("model_pt", pick("model_pt", os.path.join(model_dir, "best.pt")))
        self.declare_parameter("data_yaml", pick("data_yaml", os.path.join(model_dir, "data.yaml")))
        self.declare_parameter("conf", float(cfg.get("conf", 0.25)))
        self.declare_parameter("nms", float(cfg.get("nms", 0.45)))
        self.declare_parameter("license_mode", cfg.get("license_mode", "development"))
        self.declare_parameter(
            "model_license_approved",
            bool(cfg.get("model_license_approved", False)),
        )
        self.declare_parameter(
            "image_topic", cfg.get("image_topic", "/camera/camera/color/image_raw")
        )
        self.declare_parameter("detections_topic", cfg.get("detections_topic", "/perception/fruit_detections"))
        self.declare_parameter("annotated_topic", cfg.get("annotated_topic", "/perception/health_annotated"))

        gp = self.get_parameter
        backend = gp("backend").value
        image_topic = gp("image_topic").value
        license_mode = str(gp("license_mode").value).strip().lower()
        license_approved = bool(gp("model_license_approved").value)
        if license_mode == "production" and not license_approved:
            raise RuntimeError(
                "Production perception is blocked: the configured model has "
                "not been approved for the enterprise delivery license. "
                "Set model_license_approved:=true only after the model/code "
                "and training-data licenses are recorded in MODEL_LICENSE_AUDIT.md."
            )

        self._detector = YoloDetector(
            backend=backend,
            onnx_path=gp("model_onnx").value,
            pt_path=gp("model_pt").value,
            data_yaml=gp("data_yaml").value,
            conf_threshold=float(gp("conf").value),
            nms_threshold=float(gp("nms").value),
            logger=self.get_logger(),
        )

        self._det_pub = self.create_publisher(
            Detection2DArray, gp("detections_topic").value, 10
        )
        self._img_pub = self.create_publisher(
            Image, gp("annotated_topic").value, 10
        )
        self._sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"苹果识别节点启动: backend={self._detector.backend}, "
            f"订阅={image_topic}, 类别={self._detector.class_names}"
        )
        if license_mode != "production":
            self.get_logger().warning(
                "Perception is running in development mode; do not use this "
                "model for enterprise delivery until its license is approved."
            )

    # ── 图像回调 ───────────────────────────────────────────

    def _on_image(self, msg: Image) -> None:
        bgr = self._image_to_bgr(msg)
        if bgr is None:
            return

        dets = self._detector.infer(bgr)

        # 发布 vision_msgs
        arr = Detection2DArray()
        arr.header = msg.header
        for d in dets:
            det = Detection2D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.class_name
            hyp.hypothesis.score = float(d.confidence)
            det.results.append(hyp)
            cx, cy = d.center
            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.size_x = float(d.x2 - d.x1)
            det.bbox.size_y = float(d.y2 - d.y1)
            arr.detections.append(det)
        self._det_pub.publish(arr)

        # 发布标注图
        annotated = self._detector.draw(bgr, dets)
        self._img_pub.publish(self._bgr_to_image(annotated, msg.header))

        if dets:
            good = sum(1 for d in dets if d.is_healthy)
            self.get_logger().info(
                f"检测到 {len(dets)} 个苹果: 好 {good}, 坏 {len(dets) - good}"
            )

    # ── sensor_msgs/Image <-> numpy（手动解码，避免硬依赖 cv_bridge）──

    @staticmethod
    def _image_to_bgr(msg: Image) -> np.ndarray | None:
        try:
            enc = msg.encoding.lower()
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if enc in ("rgb8", "bgr8"):
                img = buf.reshape(msg.height, msg.width, 3)
                if enc == "rgb8":
                    img = img[:, :, ::-1]  # RGB→BGR
                return np.ascontiguousarray(img)
            if enc in ("rgba8", "bgra8"):
                img = buf.reshape(msg.height, msg.width, 4)[:, :, :3]
                if enc == "rgba8":
                    img = img[:, :, ::-1]
                return np.ascontiguousarray(img)
            if enc == "mono8":
                gray = buf.reshape(msg.height, msg.width)
                return np.repeat(gray[:, :, None], 3, axis=2)
        except Exception:
            return None
        return None

    @staticmethod
    def _bgr_to_image(bgr: np.ndarray, header) -> Image:
        msg = Image()
        msg.header = header
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(bgr, dtype=np.uint8).tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FruitDetectorNode()
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
