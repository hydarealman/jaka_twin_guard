#!/usr/bin/env python3
"""Fuse point-cloud fruit geometry with ROI-only quality classification."""

from __future__ import annotations

import math
import os
import time

import cv2
import numpy as np
import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image

from fruit_picking_arm.perception.fruit_quality_classifier import FruitQualityClassifier


"""
3D-2D融合分类器:
它把点云检测到的3D水果位置,精确投射到2D彩色图上
裁剪出单个水果的特写,然后调用AI模型和判断它是健康还是不健康
最终把分类结果填回3D物体对象里
"""
class HealthFusion:
    """Project each validated 3-D fruit into RGB and classify only that ROI."""

    def __init__(self, node: Node, perception_cfg: dict, scene_cfg: dict):
        self._node = node
        self._logger = node.get_logger()
        self._scene_cfg = scene_cfg or {}
        cfg = (perception_cfg or {}).get("classifier", {})
        self._enabled = bool(cfg.get("enabled", True))
        self._min_confidence = float(cfg.get("min_health_confidence", 0.65))
        self._min_margin = float(cfg.get("min_health_margin", 0.25))
        self._roi_scale = float(cfg.get("roi_scale", 1.25))
        self._min_roi_pixels = int(cfg.get("min_roi_pixels", 32))
        self._min_visible_ratio = float(cfg.get("min_visible_ratio", 0.90))
        self._required_fruit_type = str(
            cfg.get("required_fruit_type", "apple")
        ).strip().lower()
        self._max_sync_delta = float(cfg.get("max_sync_delta", 0.15))
        self._last_classification_log = 0.0
        # Debug-only 2-D proposals use only fresh real RGB pixels.  They make
        # quality-model behaviour visible before hand-eye/table calibration,
        # but never create a 3-D target or a robot command.
        self._enable_rgb_debug_candidates = bool(
            cfg.get("enable_rgb_debug_candidates", False)
        )
        camera_type = str(
            (perception_cfg or {}).get("camera_type", "unknown")
        ).strip().lower()
        fallback_requested = bool(cfg.get("allow_scene_fallback", False))
        self._allow_scene_fallback = self._scene_fallback_allowed(
            camera_type, fallback_requested
        )
        if fallback_requested and not self._allow_scene_fallback:
            self._logger.error(
                "[SAFETY] scene fallback was requested for camera_type="
                f"'{camera_type}', but it is restricted to mock/gazebo and "
                "has been forcibly disabled"
            )
        self._fallback_confidence = float(cfg.get("sim_fallback_confidence", 1.0))
        self._object_frame = (perception_cfg or {}).get("output_frame", "world")
        license_mode = str(cfg.get("license_mode", "development")).strip().lower()
        if license_mode == "production" and not bool(
            cfg.get("model_license_approved", False)
        ):
            raise RuntimeError(
                "Production fruit-quality perception is blocked until model "
                "and training-data licenses are approved."
            )

        model_path = str(cfg.get("quality_model", "")).strip()
        if not model_path:
            model_path = os.path.join(
                get_package_share_directory("fruit_picking_arm"),
                "models",
                "fruit_quality_mobilenet_v3.onnx",
            )
        self._classifier = FruitQualityClassifier(model_path) if self._enabled else None

        info_topic = cfg.get(
            "camera_info_topic", "/camera/camera/color/camera_info"
        )
        self._camera_info: CameraInfo | None = None
        self._info_sub = node.create_subscription(
            CameraInfo, info_topic, self._on_info, qos_profile_sensor_data
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)
        self._scene_objects = self._scene_cfg.get("objects", [])
        annotated_topic = cfg.get("annotated_topic", "/perception/health_annotated")
        latest_image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._annotated_pub = node.create_publisher(
            Image, annotated_topic, latest_image_qos
        )

    def _on_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    @staticmethod
    def _scene_fallback_allowed(camera_type: str, requested: bool) -> bool:
        """Allow synthetic labels only for explicitly simulated cameras."""
        return bool(
            requested
            and str(camera_type).strip().lower() in {"mock", "gazebo"}
        )

    """
    输入: 水果列表
    返回: 判断健康与否的水果列表
    """
    def fuse(self, objects: list, source_stamp=None, rgb_image: Image | None = None) -> list:
        """Assign health/type in place; real-camera failures remain Unknown."""
        used_real = False # 标记本帧中是否又任何物体通过了真实RGB图像的分类
        if self._enabled and rgb_image is not None:
            if objects:
                used_real = self._classify_rois(objects, rgb_image, source_stamp)
            elif self._enable_rgb_debug_candidates:
                used_real = self._classify_rgb_debug_candidates(rgb_image)
            else:
                # No object can have a quality ROI.  Forward the measured RGB
                # message directly instead of converting/copying the entire
                # frame just to publish an identical annotation image.  This
                # is a frequent path while YOLO is warming up or misses a
                # small fruit, and the copy used to add avoidable CPU load.
                self._annotated_pub.publish(rgb_image)

        if self._allow_scene_fallback: # 仿真 -> 只有仿真允许使用 实车不应该执行下面的if语句
            for obj in objects:
                if str(obj.health).lower() == "unknown":
                    self._apply_scene_hint(obj)
        if not used_real and self._allow_scene_fallback:
            self._logger.info(
                "[sim-fallback] no valid RGB ROI classification; using scene hints"
            )
        return objects

    def _classify_rgb_debug_candidates(self, image_msg: Image) -> bool:
        """Draw honest real-RGB fruit candidates without inventing 3-D data.

        The proposal stage selects compact, saturated, approximately round
        regions.  The existing ONNX model supplies the displayed fruit and
        quality labels.  These boxes are visualization only.
        """
        image = self._image_to_bgr(image_msg)
        if image is None:
            return False
        annotated = image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Red/orange/yellow and green fruit colours. Low-saturation walls,
        # tables and the gray status header are excluded.
        warm = cv2.inRange(hsv, (0, 110, 90), (42, 255, 255))
        green = cv2.inRange(hsv, (43, 110, 70), (92, 255, 255))
        mask = cv2.bitwise_or(warm, green)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        image_area = float(image.shape[0] * image.shape[1])
        proposals = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            perimeter = float(cv2.arcLength(contour, True))
            if area < 70.0 or area > image_area * 0.08 or perimeter <= 0.0:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            aspect = width / max(float(height), 1.0)
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
            solidity = area / hull_area if hull_area > 0.0 else 0.0
            if not (0.62 <= aspect <= 1.55):
                continue
            if circularity < 0.48 or solidity < 0.72:
                continue
            # Large saturated rectangles (bins, labels, stools) are common in
            # the workcell. A large proposal must be distinctly round, while
            # small distant fruit may use the looser contour threshold above.
            if area > image_area * 0.012 and circularity < 0.70:
                continue
            contour_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            median_hue = float(np.median(hsv[:, :, 0][contour_mask > 0]))
            proposals.append((area, x, y, width, height, median_hue))

        drew = False
        for _area, x, y, width, height, median_hue in sorted(
            proposals, reverse=True
        )[:3]:
            side = max(width, height, self._min_roi_pixels)
            half = int(math.ceil(side * 0.56))
            center_x = x + width // 2
            center_y = y + height // 2
            x1, y1 = max(0, center_x - half), max(0, center_y - half)
            x2 = min(image.shape[1], center_x + half)
            y2 = min(image.shape[0], center_y + half)
            crop = image[y1:y2, x1:x2]
            result = self._classifier.classify(crop)
            if result is None:
                continue
            # The proposal colour supplies a coarse visible type; the quality
            # result is accepted only when the neural model agrees with it.
            # A disagreement is shown explicitly instead of presenting a
            # confident but obviously wrong label as truth.
            if median_hue <= 12.0 or median_hue >= 43.0:
                visible_type = "apple"
            elif median_hue <= 32.0:
                visible_type = "orange"
            else:
                visible_type = "unknown"
            accepted = (
                result.fruit_type == visible_type
                and
                result.confidence >= self._min_confidence
                and result.margin >= self._min_margin
            )
            quality = result.health if accepted else "unknown"
            color = (0, 190, 0) if quality == "Healthy" else (
                (0, 0, 220) if quality == "Unhealthy" else (0, 200, 255)
            )
            if result.fruit_type == visible_type:
                label = (
                    "%s | %s | %.2f"
                    % (visible_type, quality, result.confidence)
                )
            else:
                label = "%s | quality unknown" % visible_type
            cv2.rectangle(annotated, (x1, y1), (x2 - 1, y2 - 1), color, 2)
            cv2.putText(
                annotated,
                label,
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                color,
                2,
            )
            drew = True
        self._publish_annotated(annotated, image_msg)
        return drew

    # 把3D坐标投影到2D图像,裁剪出水果特写,跑AI模型,并根据严苛的门槛决定是否采纳结果
    def _classify_rois(self, objects: list, image_msg: Image, source_stamp) -> bool:
        # 检查相机内参是否已收到,并把ROS图像转换为OpenCVBGR格式
        info = self._camera_info
        image = self._image_to_bgr(image_msg)
        if info is None or image is None:
            return False

        # 时间同步检查:
        # 计算RGB图的时间戳和点云时间戳的差值,如果相差超过0.15秒,直接丢弃这一帧
        if source_stamp is not None:
            delta = abs(
                self._stamp_seconds(image_msg.header.stamp)
                - self._stamp_seconds(source_stamp)
            )
            if delta > self._max_sync_delta:
                self._logger.warning(f"Reject stale RGB/cloud pair: dt={delta:.3f}s")
                return False

        # 提取相机内参,以及相机坐标系的名称
        k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
        if fx <= 0 or fy <= 0:
            return False
        optical_frame = info.header.frame_id or "camera_color_optical_frame" # 相机系

        classified = False
        annotated = image.copy()
        for obj in objects:
            # 重置当前物体的健康撞他
            self._set_unknown(obj)

            # 3D -> 2D 投影
            detector_bbox = getattr(obj, "bbox2d", None)
            if detector_bbox is not None:
                x1, y1, x2, y2 = [float(value) for value in detector_bbox]
                projected = (
                    (x1 + x2) * 0.5,
                    (y1 + y2) * 0.5,
                    max(x2 - x1, y2 - y1) * 0.5,
                )
            else:
                projected = self._project_sphere(
                    obj.centroid, float(obj.radius), optical_frame, fx, fy, cx, cy
                )
            if projected is None:
                continue
            u, v, radius_px = projected

            # Always draw the geometric detection, even when the ROI is too
            # small, the classifier is unavailable, or the quality decision
            # is intentionally rejected. This keeps the debug image honest:
            # a yellow box means "detected, but classification is unknown".
            half = radius_px * self._roi_scale
            p1 = (max(0, int(u - half)), max(0, int(v - half)))
            p2 = (
                min(image.shape[1] - 1, int(u + half)),
                min(image.shape[0] - 1, int(v + half)),
            )
            label = "TYPE=unknown QUALITY=unknown"
            label_color = (0, 200, 255)

            # 裁剪ROI(感兴趣区域)
            crop = self._crop_complete_roi(image, u, v, radius_px)
            if crop is None:
                self._logger.warning(f"{obj.id}: reject incomplete/small fruit ROI")
                label = "TYPE=unknown QUALITY=unknown ROI=invalid"
                cv2.rectangle(annotated, p1, p2, label_color, 2)
                cv2.putText(
                    annotated,
                    label,
                    (p1[0], max(20, p1[1] - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    label_color,
                    2,
                )
                continue
            """
            输入: 裁剪出来的水果像素块
            输出: 包含种类,健康结论,置信度,判断边际值的体检报告
            """
            # 模型推理
            result = self._classifier.classify(crop)
            if result is None:
                label = "TYPE=unknown QUALITY=unknown CLASSIFIER=invalid"
                cv2.rectangle(annotated, p1, p2, label_color, 2)
                cv2.putText(
                    annotated,
                    label,
                    (p1[0], max(20, p1[1] - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    label_color,
                    2,
                )
                continue

            # 保存数据
            obj.fruit_type = result.fruit_type
            obj.health_margin = result.margin

            # 判定 ？？？
            """
            种类必须匹配
            AI置信度必须高
            健康/不健康的分数差必须够大
            """
            accepted = (
                result.fruit_type == self._required_fruit_type
                and result.confidence >= self._min_confidence
                and result.margin >= self._min_margin
            )
            if accepted:
                obj.health = result.health
                obj.health_confidence = result.confidence
                obj.class_id = 0 if result.health == "Healthy" else 1
                classified = True

            # 可视化绘制
            label_color = (0, 190, 0) if obj.health == "Healthy" else (
                (0, 0, 220) if obj.health == "Unhealthy" else (0, 200, 255)
            )
            quality = obj.health if obj.health != "unknown" else "unknown"
            label = (
                "TYPE=%s QUALITY=%s CONF=%.2f"
                % (result.fruit_type, quality, result.confidence)
            )
            cv2.rectangle(annotated, p1, p2, label_color, 2)
            cv2.putText(
                annotated,
                label,
                (p1[0], max(20, p1[1] - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                label_color,
                2,
            )
            now = time.monotonic()
            if now - self._last_classification_log >= 2.0:
                self._logger.info(
                    f"{obj.id}: ROI={result.raw_label} "
                    f"score={result.confidence:.3f} "
                    f"margin={result.margin:.3f} -> {obj.health}"
                )
                self._last_classification_log = now
        self._publish_annotated(annotated, image_msg)
        return classified

    # 将三维空间中的一个球体投影到二维图像平面上
    # 得到球心在图像上的像素坐标,以及球体在图像上对应的像素半径
    def _project_sphere(self, centroid, radius, optical_frame, fx, fy, cx, cy):
        if optical_frame == self._object_frame:
            point = np.asarray(centroid, np.float64)
        else:

            try:
                # 世界 -> 相机
                tf = self._tf_buffer.lookup_transform(
                    optical_frame,        # 相机坐标系 target
                    self._object_frame,   # 世界坐标系 source
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
            except Exception:
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            rotation = self._quat_to_matrix(q.x, q.y, q.z, q.w) # 球体半径投影(尺度缩放)
            point = rotation @ np.asarray(centroid, np.float64) + np.array(
                [t.x, t.y, t.z], np.float64
            )
        # 相机 -> 二维像素平面
        x, y, z = point
        if z <= max(radius, 1e-4):
            return None
        u = fx * x / z + cx
        v = fy * y / z + cy
        radius_px = max(fx, fy) * radius / z
        return float(u), float(v), float(radius_px)


    # 从图像中裁剪出一个以(u,v)为中心,边长为2 * half的正方形ROI(感兴趣区域)
    # 并检查裁剪区域是否足够大,且大部分位于图像边界内
    # 只有通过检查,才返回裁剪后的图像块,否则返回None
    def _crop_complete_roi(self, image, u, v, radius_px):
        # 计算缩放的半边长
        half = radius_px * self._roi_scale
        # 检查最小尺寸
        size = int(math.ceil(half * 2.0))
        if size < self._min_roi_pixels:
            return None
        # 计算裁剪边界
        raw = (u - half, v - half, u + half, v + half)
        # 将边界裁剪到图像有效范围内
        x1 = max(0, int(math.floor(raw[0])))
        y1 = max(0, int(math.floor(raw[1])))
        x2 = min(image.shape[1], int(math.ceil(raw[2])))
        y2 = min(image.shape[0], int(math.ceil(raw[3])))
        # 计算可见面积比例
        visible = max(0, x2 - x1) * max(0, y2 - y1)
        expected = max(1.0, (2.0 * half) ** 2)
        if visible / expected < self._min_visible_ratio:
            return None
        # 执行裁剪并返回
        return image[y1:y2, x1:x2]


    """
    依据显式选择的实车/仿真 scene profile 配置
    仿真环境专用的数据填充器
    不依赖任何相机图像或者AI模型
    """
    def _apply_scene_hint(self, obj) -> None:
        if not self._scene_objects:
            self._set_unknown(obj)
            return
        ox, oy = obj.centroid[0], obj.centroid[1]
        best = min(
            self._scene_objects,
            key=lambda item: math.hypot(
                item.get("position", {}).get("x", 0.0) - ox,
                item.get("position", {}).get("y", 0.0) - oy,
            ),
        )
        hint = str(best.get("health", "unknown")).lower()
        obj.health = "Unhealthy" if hint.startswith("un") else "Healthy"
        obj.health_confidence = self._fallback_confidence
        obj.health_margin = 1.0
        obj.fruit_type = "apple"
        obj.class_id = 1 if obj.health == "Unhealthy" else 0

    @staticmethod
    def _set_unknown(obj) -> None:
        obj.health = "unknown"
        obj.health_confidence = 0.0
        obj.health_margin = 0.0
        obj.fruit_type = "unknown"
        obj.class_id = -1

    @staticmethod
    def _image_to_bgr(msg: Image):
        try:
            buffer = np.frombuffer(msg.data, dtype=np.uint8)
            encoding = str(msg.encoding).lower()
            if encoding in ("rgb8", "bgr8"):
                image = buffer.reshape(msg.height, msg.width, 3)
                if encoding == "rgb8":
                    image = image[:, :, ::-1]
                return np.ascontiguousarray(image)
            if encoding in ("rgba8", "bgra8"):
                image = buffer.reshape(msg.height, msg.width, 4)[:, :, :3]
                if encoding == "rgba8":
                    image = image[:, :, ::-1]
                return np.ascontiguousarray(image)
        except (TypeError, ValueError):
            return None
        return None

    # 发布带标注的调试图像
    def _publish_annotated(self, image, source: Image) -> None:
        msg = Image()
        msg.header = source.header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
        self._annotated_pub.publish(msg)

    # 四元数转旋转矩阵
    @staticmethod
    def _quat_to_matrix(x, y, z, w):
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
        ], dtype=np.float64)

    # 时间戳转浮点数
    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9
