#!/usr/bin/env python3
"""HealthFusion —— 把 YOLO 苹果好坏检测融合到点云检测出的 3D 物体上。

数据流：
  fruit_detector_node ──/perception/fruit_detections(vision_msgs)──┐
  相机 ─────────/camera/camera/color/camera_info(内参 K)──────────┤
                                                                    ▼
  ObjectDetector(点云) ── DetectedObject[centroid(world)] ──► HealthFusion.fuse()
                                                                    │
                                        投影 world→optical→像素，匹配检测框
                                                                    ▼
                                        DetectedObject.health / health_confidence

匹配失败 / 无相机 / 分类器禁用 时，回退到 scene_params.yaml 中每个物体的
`health` 提示（按 xy 最近邻关联），并打 [sim-fallback] 日志，保证分拣演示完整。
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray


class HealthFusion:
    def __init__(self, node: Node, perception_cfg: dict, scene_cfg: dict):
        self._node = node
        self._logger = node.get_logger()
        self._scene_cfg = scene_cfg or {}

        cls_cfg = (perception_cfg or {}).get("classifier", {})
        self._enabled = cls_cfg.get("enabled", True)
        self._det_topic = cls_cfg.get("detections_topic", "/perception/fruit_detections")
        self._info_topic = cls_cfg.get(
            "camera_info_topic", "/camera/camera/color/camera_info"
        )
        # 像素匹配容差（框外时允许的最近中心距离，单位像素）
        self._match_tol = float(cls_cfg.get("pixel_match_tolerance", 80.0))
        self._max_sync_delta = float(cls_cfg.get("max_sync_delta", 0.15))
        # Fail closed for the real robot: a weak or ambiguous quality result
        # must remain Unknown and therefore cannot become a stable pick target.
        self._min_health_confidence = float(
            cls_cfg.get("min_health_confidence", 0.60)
        )
        self._min_health_margin = float(
            cls_cfg.get("min_health_margin", 0.15)
        )
        self._allow_scene_fallback = bool(
            cls_cfg.get("allow_scene_fallback", False)
        )
        self._fallback_confidence = float(
            cls_cfg.get("sim_fallback_confidence", 1.0)
        )
        self._object_frame = (perception_cfg or {}).get("output_frame", "world")

        self._latest_dets: Detection2DArray | None = None
        self._camera_info: CameraInfo | None = None

        self._det_sub = node.create_subscription(
            Detection2DArray, self._det_topic, self._on_dets, 10
        )
        self._info_sub = node.create_subscription(
            CameraInfo, self._info_topic, self._on_info,
            qos_profile_sensor_data,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        # scene 提示：id/label → health
        self._scene_objects = self._scene_cfg.get("objects", [])

    # ── 订阅回调 ───────────────────────────────────────────

    def _on_dets(self, msg: Detection2DArray) -> None:
        self._latest_dets = msg

    def _on_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    # ── 对外接口 ───────────────────────────────────────────

    def fuse(self, objects: list, source_stamp=None) -> list:
        """给每个 DetectedObject 赋 health / health_confidence。就地修改并返回。"""
        used_real = False
        if self._enabled:
            used_real = self._fuse_from_detections(objects, source_stamp)

        # 场景提示只能用于仿真。真机识别失败时必须保持 Unknown。
        if self._allow_scene_fallback:
            for obj in objects:
                if obj.health == "unknown":
                    self._apply_scene_hint(obj)
        if not used_real and self._allow_scene_fallback:
            self._logger.info(
                "[sim-fallback] 未获得有效 YOLO 检测/相机内参，健康标签来自 scene_params 提示"
            )
        return objects

    # ── 真实推理路径：投影匹配 ─────────────────────────────

    def _fuse_from_detections(self, objects: list, source_stamp=None) -> bool:
        dets = self._latest_dets
        info = self._camera_info
        if dets is None or not dets.detections or info is None:
            return False
        if source_stamp is not None:
            detection_time = self._stamp_seconds(dets.header.stamp)
            source_time = self._stamp_seconds(source_stamp)
            if detection_time > 0 and source_time > 0:
                delta = abs(detection_time - source_time)
                if delta > self._max_sync_delta:
                    self._logger.warning(
                        f"Rejecting stale RGB/point-cloud fusion: Δt={delta:.3f}s"
                    )
                    return False

        K = np.array(info.k, dtype=np.float64).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        if fx == 0 or fy == 0:
            return False

        optical_frame = info.header.frame_id or "camera_color_frame"

        # 预取所有检测框中心
        boxes = []
        for d in dets.detections:
            if not d.results:
                continue
            hyp = d.results[0].hypothesis
            bc = d.bbox.center
            # 兼容 Humble: center.position.x/y
            bx = getattr(bc, "position", bc).x if hasattr(bc, "position") else bc.x
            by = getattr(bc, "position", bc).y if hasattr(bc, "position") else bc.y
            boxes.append({
                "class_id": hyp.class_id,
                "score": float(hyp.score),
                "cx": float(bx), "cy": float(by),
                "w": float(d.bbox.size_x), "h": float(d.bbox.size_y),
            })
        if not boxes:
            return False

        matched_any = False
        for obj in objects:
            uv = self._project_to_pixel(obj.centroid, optical_frame, fx, fy, cx, cy)
            if uv is None:
                continue
            u, v = uv
            candidates = self._matching_boxes(u, v, boxes)
            best = max(candidates, key=lambda box: box["score"]) if candidates else None
            if best is not None:
                # Compare against the strongest *other-class* box.  Duplicate
                # boxes of the same class are harmless; conflicting classes
                # are rejected when their scores are too close.
                runner = max(
                    (box for box in candidates
                     if self._normalize(box["class_id"]) != self._normalize(best["class_id"])),
                    key=lambda box: box["score"],
                    default=None,
                )
                margin = best["score"] - runner["score"] if runner else 1.0
                obj.health_margin = float(max(0.0, margin))
                candidate_health = self._normalize(best["class_id"])
                accepted = (
                    candidate_health != "unknown"
                    and best["score"] >= self._min_health_confidence
                    and (runner is None or margin >= self._min_health_margin)
                )
                if accepted:
                    obj.health = candidate_health
                    obj.health_confidence = best["score"]
                    obj.class_id = 0 if obj.health == "Healthy" else 1
                    matched_any = True
                else:
                    obj.health = "unknown"
                    obj.health_confidence = 0.0
                    obj.class_id = -1
                    self._logger.warning(
                        f"  {obj.id}: reject ambiguous/weak quality result "
                        f"best={candidate_health}:{best['score']:.3f}, "
                        f"margin={margin:.3f} @px({u:.0f},{v:.0f})"
                    )
                self._logger.info(
                    f"  {obj.id}: YOLO→{obj.health} ({best['score']*100:.1f}%) "
                    f"@px({u:.0f},{v:.0f})"
                )
        return matched_any

    def _project_to_pixel(self, centroid, optical_frame, fx, fy, cx, cy):
        """world 坐标质心 → 相机光学系 → 像素 (u, v)。失败返回 None。"""
        try:
            tf = self._tf_buffer.lookup_transform(
                optical_frame, self._object_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        R = self._quat_to_matrix(q.x, q.y, q.z, q.w)
        p = R @ np.array(centroid, dtype=np.float64) + np.array([t.x, t.y, t.z])
        X, Y, Z = p
        if Z <= 1e-4:
            return None
        u = fx * X / Z + cx
        v = fy * Y / Z + cy
        return u, v

    def _match_box(self, u, v, boxes):
        """优先选包含 (u,v) 的框，否则选中心距离在容差内且最近的框。"""
        candidates = self._matching_boxes(u, v, boxes)
        return max(candidates, key=lambda box: box["score"]) if candidates else None

    def _matching_boxes(self, u, v, boxes):
        """Return boxes that can explain a projected 3-D centroid.

        Containing boxes are preferred.  When no box contains the centroid,
        only the nearest box within ``pixel_match_tolerance`` is returned.
        Keeping the candidate set lets HealthFusion compute a class margin.
        """
        containing = []
        for b in boxes:
            hw, hh = b["w"] / 2.0, b["h"] / 2.0
            if (b["cx"] - hw <= u <= b["cx"] + hw and
                    b["cy"] - hh <= v <= b["cy"] + hh):
                containing.append(b)
        if containing:
            return containing

        best, best_d = None, self._match_tol
        for b in boxes:
            d = math.hypot(b["cx"] - u, b["cy"] - v)
            if d < best_d:
                best, best_d = b, d
        return [best] if best is not None else []

    # ── 兜底路径：场景提示（xy 最近邻）────────────────────

    def _apply_scene_hint(self, obj) -> None:
        if not self._scene_objects:
            obj.health = "unknown"
            obj.health_confidence = 0.0
            obj.health_margin = 0.0
            obj.class_id = -1
            return
        ox, oy = obj.centroid[0], obj.centroid[1]
        best, best_d = None, float("inf")
        for so in self._scene_objects:
            pos = so.get("position", {})
            d = math.hypot(pos.get("x", 0) - ox, pos.get("y", 0) - oy)
            if d < best_d:
                best, best_d = so, d
        hint = (best or {}).get("health", "Healthy")
        obj.health = "Unhealthy" if str(hint).lower().startswith("un") else "Healthy"
        obj.health_confidence = self._fallback_confidence
        obj.health_margin = 1.0
        obj.class_id = 1 if obj.health == "Unhealthy" else 0

    # ── 工具 ───────────────────────────────────────────────

    @staticmethod
    def _normalize(class_id_str) -> str:
        s = str(class_id_str).strip().lower()
        if s in ("1", "unhealthy", "bad"):
            return "Unhealthy"
        if s in ("0", "healthy", "good"):
            return "Healthy"
        return "unknown"

    @staticmethod
    def _quat_to_matrix(x, y, z, w):
        return np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
        ], dtype=np.float64)

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9
