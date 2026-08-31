#!/usr/bin/env python3
"""Lightweight RGB detector + registered depth fruit localizer.

The legacy detector starts from every point in a room-sized cloud and tries to
infer which clusters are fruit.  This backend starts from a semantic apple
box, samples only the registered depth inside that box, and deprojects the
measurement with the RGB camera intrinsics.  It is deliberately independent
of the health classifier: the existing MobileNet model still owns the
healthy/rotten decision.
"""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from fruit_picking_arm.perception.fast_box_tracker import DisplayDetection, FastBoxTracker
from fruit_picking_arm.perception.object_detector import DetectedObject


class YoloDepthLocalizer:
    """YOLO apple detections converted to safe 3-D camera/world candidates."""

    def __init__(self, node: Node, config: dict, camera_frame: str = ""):
        self._node = node
        self._logger = node.get_logger()
        self._config = config
        self._output_frame = str(config.get("output_frame", "camera_color_optical_frame"))
        self._camera_frame = camera_frame or "camera_color_optical_frame"
        detector_cfg = dict(config.get("fruit_detector", {}))
        self._model_path = str(detector_cfg.get("model", "yolov8n.pt"))
        self._confidence = float(detector_cfg.get("confidence", 0.25))
        self._tracking_confidence = min(
            self._confidence,
            float(detector_cfg.get("tracking_confidence", 0.12)),
        )
        self._iou = float(detector_cfg.get("iou", 0.50))
        self._image_size = int(detector_cfg.get("image_size", 640))
        self._max_detections = max(1, int(detector_cfg.get("max_detections", 5)))
        self._inference_threads = max(1, int(detector_cfg.get("inference_threads", 1)))
        self._device = str(detector_cfg.get("device", "cpu"))
        self._fast_annotated_topic = str(
            detector_cfg.get(
                "fast_annotated_topic", "/perception/detection_annotated"
            )
        )
        latest_image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._fast_annotated_pub = node.create_publisher(
            Image, self._fast_annotated_topic, latest_image_qos
        )
        self._fast_detections_topic = str(
            detector_cfg.get(
                "fast_detections_topic", "/perception/apple_detections_2d"
            )
        )
        latest_detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._fast_detections_pub = node.create_publisher(
            Detection2DArray, self._fast_detections_topic, latest_detection_qos
        )
        self._fast_box_tracker = FastBoxTracker(
            confirm_hits=int(detector_cfg.get("display_track_confirm_hits", 2)),
            max_misses=int(detector_cfg.get("display_track_max_misses", 2)),
            max_age_s=float(detector_cfg.get("display_track_max_age_s", 0.18)),
            max_center_motion_ratio=float(
                detector_cfg.get("display_track_max_center_motion_ratio", 0.35)
            ),
        )
        # A full 424x240 frame makes a nearby fruit occupy too few pixels
        # after letterboxing. Search one overlapping tile per cycle, then
        # follow the last accepted box locally. This keeps the inference
        # budget close to one pass per cycle instead of running four passes.
        self._small_object_tiles = bool(
            detector_cfg.get("small_object_tiles", True)
        )
        self._tile_rows = max(1, int(detector_cfg.get("tile_rows", 2)))
        self._tile_cols = max(1, int(detector_cfg.get("tile_cols", 2)))
        self._tile_overlap = min(
            0.8, max(0.05, float(detector_cfg.get("tile_overlap", 0.25)))
        )
        self._local_tile_width_ratio = min(
            0.95, max(0.35, float(detector_cfg.get("local_tile_width_ratio", 0.70)))
        )
        self._local_tile_height_ratio = min(
            0.95, max(0.35, float(detector_cfg.get("local_tile_height_ratio", 0.75)))
        )
        self._max_local_tile_misses = max(
            1, int(detector_cfg.get("max_local_tile_misses", 2))
        )
        self._edge_recovery = bool(detector_cfg.get("edge_recovery", True))
        self._edge_shift_ratio = min(
            0.30, max(0.08, float(detector_cfg.get("edge_shift_ratio", 0.18)))
        )
        self._edge_batch_warmed = False
        self._tile_cursor = 0
        self._last_bbox = None
        self._last_tile_bounds = None
        self._last_prediction_bounds = None
        self._local_tile_misses = 0
        self._last_prediction_mode = "grid"
        self._min_depth = float(detector_cfg.get("min_depth_m", 0.10))
        self._max_depth = float(detector_cfg.get("max_depth_m", 5.0))
        self._min_coverage = float(detector_cfg.get("min_depth_coverage", 0.20))
        self._min_radius = float(config.get("min_object_radius", 0.015))
        self._max_radius = float(config.get("max_object_radius", 0.080))
        self._roi_min = self._as_vec(config.get("detection_roi_min", []))
        self._roi_max = self._as_vec(config.get("detection_roi_max", []))
        self._model = None
        self._apple_class_id = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)
        self._last_tf_warning = 0.0
        self._last_roi_warning = 0.0
        self._last_luma_mean = 0.0
        self._last_highlight_fraction = 0.0
        self._last_stats_log = time.monotonic()
        self._stats = {
            "frames": 0.0,
            "raw_boxes": 0.0,
            "raw_peak": 0.0,
            "no_box_frames": 0.0,
            "accepted": 0.0,
            "accepted_peak": 0.0,
            "depth_rejected": 0.0,
            "radius_rejected": 0.0,
            "tf_rejected": 0.0,
            "roi_rejected": 0.0,
            "inference_ms": 0.0,
            "luma_mean": 0.0,
            "highlight_fraction": 0.0,
        }

    @staticmethod
    def _as_vec(value):
        if value is None or len(value) != 3:
            return None
        return np.asarray(value, dtype=np.float32)

    @staticmethod
    def _image_to_bgr(msg: Image) -> np.ndarray | None:
        try:
            encoding = str(msg.encoding).lower()
            if encoding not in ("rgb8", "bgr8"):
                return None
            channels = 3
            step = int(msg.step or msg.width * channels)
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            rows = raw.reshape(msg.height, step)
            image = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
            if encoding == "rgb8":
                image = image[:, :, ::-1]
            return np.ascontiguousarray(image)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _depth_to_meters(msg: Image) -> np.ndarray | None:
        try:
            encoding = str(msg.encoding).lower()
            if encoding in ("16uc1", "mono16"):
                dtype, scale = np.dtype("<u2"), 0.001
            elif encoding in ("32fc1", "32fc"):
                dtype, scale = np.dtype("<f4"), 1.0
            else:
                return None
            step = int(msg.step or msg.width * dtype.itemsize)
            row_values = step // dtype.itemsize
            raw = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_values)
            return raw[:, : msg.width].astype(np.float32) * scale
        except (TypeError, ValueError):
            return None

    def _load_model(self) -> None:
        if self._model is not None:
            return
        try:
            import torch

            requested_device = self._device.strip().lower()
            if requested_device == "auto":
                self._device = "0" if torch.cuda.is_available() else "cpu"
                if self._device == "cpu":
                    self._logger.warning(
                        "fruit_detector.device=auto but CUDA is unavailable; "
                        "falling back to CPU"
                    )

            # Ultralytics uses OpenCV for part of its preprocessing and NMS.
            # Its default OpenCV pool is independent of PyTorch's pool, so
            # capping torch alone still allowed a CPU-only detector to occupy
            # several WSL cores and starve the USB/IP image callbacks.
            cv2.setNumThreads(1)

            # Keep the detector from taking every CPU core. On WSL2 the
            # default PyTorch pool competes with librealsense, DDS and the
            # debug GUI, which turns a healthy camera stream into a stuttering
            # stream. One intra-op thread is enough for the low-rate detector.
            torch.set_num_threads(self._inference_threads)
            try:
                torch.set_num_interop_threads(1)
            except RuntimeError:
                # This process-global value may already be fixed by another
                # torch consumer; the intra-op cap remains effective.
                pass
            from ultralytics import YOLO

            self._model = YOLO(self._model_path)
        except Exception as exc:
            raise RuntimeError(
                "Unable to load fruit detector model '%s'. Install ultralytics "
                "or provide fruit_detector.model." % self._model_path
            ) from exc
        names = getattr(self._model, "names", {})
        if isinstance(names, dict):
            pairs = names.items()
        else:
            pairs = enumerate(names)
        for class_id, name in pairs:
            if str(name).strip().lower() == "apple":
                self._apple_class_id = int(class_id)
                break
        if self._apple_class_id is None:
            raise RuntimeError(
                "The fruit detector model has no 'apple' class; available classes: %s"
                % names
            )
        self._logger.info(
            "YOLO RGB-D localizer loaded model=%s apple_class=%d "
            "conf=%.2f track_conf=%.2f imgsz=%d device=%s threads=%d "
            "max_det=%d tiles=%s edge_recovery=%s"
            % (
                self._model_path,
                self._apple_class_id,
                self._confidence,
                self._tracking_confidence,
                self._image_size,
                self._device,
                self._inference_threads,
                self._max_detections,
                self._small_object_tiles,
                self._edge_recovery,
            )
        )

    def _record_stats(self, **values) -> None:
        # Some unit-test helpers and downstream integrations construct this
        # class without running __init__.  Keep the diagnostics additive so a
        # missing optional counter cannot break perception itself.
        self._stats.setdefault("luma_mean", 0.0)
        self._stats.setdefault("highlight_fraction", 0.0)
        self._stats.setdefault("raw_peak", 0.0)
        self._stats.setdefault("accepted_peak", 0.0)
        self._stats["frames"] += 1.0
        self._stats["luma_mean"] += getattr(self, "_last_luma_mean", 0.0)
        self._stats["highlight_fraction"] += getattr(
            self, "_last_highlight_fraction", 0.0
        )
        for key, value in values.items():
            if key in self._stats:
                self._stats[key] += float(value)
        self._stats["raw_peak"] = max(
            self._stats["raw_peak"], float(values.get("raw_boxes", 0.0))
        )
        self._stats["accepted_peak"] = max(
            self._stats["accepted_peak"], float(values.get("accepted", 0.0))
        )
        now = time.monotonic()
        if now - self._last_stats_log < 2.0:
            return
        frames = max(1.0, self._stats["frames"])
        self._logger.info(
            "YOLO RGB-D stats: frames=%d raw_apple_total=%d raw_peak=%d "
            "no_box_frames=%d accepted_total=%d accepted_peak=%d "
            "reject_depth=%d reject_radius=%d reject_tf=%d "
            "reject_roi=%d luma=%.1f highlight>=250=%.2f%% "
            "infer=%.1fms device=%s threads=%d"
            % (
                int(frames),
                int(self._stats["raw_boxes"]),
                int(self._stats["raw_peak"]),
                int(self._stats["no_box_frames"]),
                int(self._stats["accepted"]),
                int(self._stats["accepted_peak"]),
                int(self._stats["depth_rejected"]),
                int(self._stats["radius_rejected"]),
                int(self._stats["tf_rejected"]),
                int(self._stats["roi_rejected"]),
                self._stats["luma_mean"] / frames,
                100.0 * self._stats["highlight_fraction"] / frames,
                self._stats["inference_ms"] / frames,
                self._device,
                self._inference_threads,
            )
        )
        for key in self._stats:
            self._stats[key] = 0.0
        self._last_stats_log = now

    def _finish(self, objects: list[DetectedObject], **stats) -> list[DetectedObject]:
        self._record_stats(**stats)
        return objects

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def process(
        self,
        rgb_msg: Image,
        aligned_depth_msg: Image,
        color_info: CameraInfo,
    ) -> list[DetectedObject]:
        image = self._image_to_bgr(rgb_msg)
        depth = self._depth_to_meters(aligned_depth_msg)
        if image is None or depth is None:
            return self._finish([], no_box_frames=1)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self._last_luma_mean = float(np.mean(gray))
        self._last_highlight_fraction = float(np.mean(gray >= 250))
        if depth.shape[:2] != image.shape[:2]:
            self._logger.warning(
                "Registered depth shape %s does not match RGB shape %s"
                % (depth.shape[:2], image.shape[:2])
            )
            return self._finish([], no_box_frames=1)
        k = np.asarray(color_info.k, dtype=np.float64).reshape(3, 3)
        fx, fy, cx, cy = float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])
        if min(fx, fy) <= 0.0:
            return self._finish([], no_box_frames=1)

        self._load_model()
        predictions, inference_ms = self._predict_boxes(image)
        # Keep the detector image semantically pure: it shows only boxes from
        # this YOLO inference. The timestamped Detection2D topic carries the
        # same raw proposals; KF continuation has its own separate topic.
        raw_display_predictions = [
            DisplayDetection(
                np.asarray(box, dtype=np.float32),
                float(score),
                predicted=False,
                confirmed=False,
            )
            for box, score in predictions
        ]
        self._publish_fast_detections(rgb_msg, raw_display_predictions)
        self._publish_fast_annotation(
            image, rgb_msg, raw_display_predictions, inference_ms
        )
        if not predictions:
            if self._last_bbox is not None:
                self._local_tile_misses += 1
                if self._local_tile_misses > self._max_local_tile_misses:
                    self._last_bbox = None
                    self._last_tile_bounds = None
            return self._finish([], no_box_frames=1, inference_ms=inference_ms)

        objects: list[DetectedObject] = []
        raw_boxes = len(predictions)
        depth_rejected = 0
        radius_rejected = 0
        tf_rejected = 0
        roi_rejected = 0
        for index, (box, score) in enumerate(predictions):
            x1, y1, x2, y2 = [float(value) for value in box]
            x1 = max(0.0, min(x1, image.shape[1] - 1.0))
            y1 = max(0.0, min(y1, image.shape[0] - 1.0))
            x2 = max(x1 + 1.0, min(x2, float(image.shape[1])))
            y2 = max(y1 + 1.0, min(y2, float(image.shape[0])))
            sample = self._sample_depth(depth, x1, y1, x2, y2)
            if sample is None:
                depth_rejected += 1
                continue
            z, coverage = sample
            u = (x1 + x2) * 0.5
            v = (y1 + y2) * 0.5
            radius_px = max(x2 - x1, y2 - y1) * 0.5
            radius = radius_px * z / max(fx, fy)
            if not self._min_radius <= radius <= self._max_radius:
                radius_rejected += 1
                continue
            point = np.asarray([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=np.float32)
            point = self._transform_point(point, rgb_msg.header, self._output_frame)
            if point is None:
                tf_rejected += 1
                continue
            if not self._inside_roi(point):
                roi_rejected += 1
                now = time.monotonic()
                if now - getattr(self, "_last_roi_warning", 0.0) > 2.0:
                    self._logger.warning(
                        "ROI_REJECT: point=(%.3f, %.3f, %.3f)m frame=%s "
                        "allowed_min=%s allowed_max=%s"
                        % (
                            point[0], point[1], point[2], self._output_frame,
                            self._roi_min.tolist(), self._roi_max.tolist(),
                        )
                    )
                    self._last_roi_warning = now
                continue
            obj = DetectedObject(
                id="yolo_%02d" % index,
                centroid=tuple(float(value) for value in point),
                radius=float(radius),
                num_points=int(coverage * max(1, int((x2 - x1) * (y2 - y1)))),
                confidence=float(score),
                bbox2d=(x1, y1, x2, y2),
                detector_label="apple",
                depth_coverage=float(coverage),
            )
            objects.append(obj)
        if objects:
            self._last_bbox = objects[0].bbox2d
            self._last_tile_bounds = self._last_prediction_bounds
            self._local_tile_misses = 0
        return self._finish(
            objects,
            raw_boxes=raw_boxes,
            accepted=len(objects),
            depth_rejected=depth_rejected,
            radius_rejected=radius_rejected,
            tf_rejected=tf_rejected,
            roi_rejected=roi_rejected,
            inference_ms=inference_ms,
        )

    def _predict_boxes(self, image: np.ndarray):
        """Run one full/tiled inference and map tile boxes to full-image pixels."""
        height, width = image.shape[:2]
        if self._edge_recovery and not self._edge_batch_warmed:
            # PyTorch/Ultralytics compiles a separate CUDA execution shape for
            # batch-4. If this happened on the first later detector miss, the
            # GUI would freeze for roughly a second. Pay that cost during the
            # normal startup warm-up before the first debug frame is emitted.
            self._model.predict(
                source=[image] * 4,
                imgsz=self._image_size,
                conf=self._tracking_confidence,
                iou=self._iou,
                classes=[self._apple_class_id],
                device=self._device,
                max_det=self._max_detections,
                verbose=False,
            )
            self._edge_batch_warmed = True
        candidates = [((0, 0, width, height), "full")]
        if self._small_object_tiles:
            if (
                self._last_bbox is not None
                and self._local_tile_misses <= self._max_local_tile_misses
            ):
                # Recenter every frame. The old implementation reused the
                # crop in which the fruit was first found; a slowly moving
                # apple eventually reached that crop boundary and was cut in
                # half for several frames before full-image search resumed.
                candidates = [
                    (self._local_tile_bounds(width, height), "track_dynamic"),
                    ((0, 0, width, height), "full_reacquire"),
                ]
            else:
                # Always search the full image first.  The previous rotating
                # one-tile strategy could wait four 5 Hz cycles before even
                # looking at an edge.  Only when the full pass misses do we
                # spend one extra pass on the next overlapping detail tile.
                bounds = self._grid_tile_bounds(width, height)
                detail = bounds[self._tile_cursor]
                self._tile_cursor = (self._tile_cursor + 1) % len(bounds)
                candidates.append((detail, "grid_after_full_miss"))

        inference_ms = 0.0
        for (x1, y1, x2, y2), mode in candidates:
            self._last_prediction_bounds = (x1, y1, x2, y2)
            crop = np.ascontiguousarray(image[y1:y2, x1:x2])
            started = time.perf_counter()
            results = self._model.predict(
                source=crop,
                imgsz=self._image_size,
                # A low proposal floor is safe only because _select_boxes
                # requires the normal acquisition score for a new target.
                # The lower score may merely continue a spatially associated
                # existing track through motion blur.
                conf=self._tracking_confidence,
                iou=self._iou,
                classes=[self._apple_class_id],
                device=self._device,
                max_det=self._max_detections,
                verbose=False,
            )
            inference_ms += (time.perf_counter() - started) * 1000.0
            self._last_prediction_mode = mode
            if not results:
                continue
            boxes = getattr(results[0], "boxes", None)
            if boxes is None or len(boxes) == 0:
                continue
            xyxy = boxes.xyxy.detach().cpu().numpy()
            scores = boxes.conf.detach().cpu().numpy()
            proposals = []
            for box, score in zip(xyxy, scores):
                mapped = np.asarray(
                    [
                        float(box[0]) + x1,
                        float(box[1]) + y1,
                        float(box[2]) + x1,
                        float(box[3]) + y1,
                    ],
                    dtype=np.float32,
                )
                proposals.append((mapped, float(score)))
            predictions = self._select_boxes(proposals, width, height)
            if predictions:
                return predictions, inference_ms

        if self._edge_recovery:
            predictions, recovery_ms, mode = self._predict_edge_views(image)
            inference_ms += recovery_ms
            if predictions:
                self._last_prediction_mode = mode
                return predictions, inference_ms
        return [], inference_ms

    def _select_boxes(self, proposals, width: int, height: int):
        """Apply acquisition hysteresis and cross-view non-maximum suppression.

        Ultralytics suppresses duplicates within one inference result, but the
        edge-recovery path maps detections from several shifted views back into
        the source image.  Without this second NMS pass, one physical apple can
        become several 3-D objects.  Conversely, do not truncate to one result:
        distinct apples must reach the multi-target KF and PlanningScene.
        """
        accepted = []
        for box, score in proposals:
            score = float(score)
            if score >= self._confidence:
                accepted.append((box, score))
                continue
            if (
                self._last_bbox is not None
                and score >= self._tracking_confidence
                and self._box_associated(box, self._last_bbox, width, height)
            ):
                accepted.append((box, score))
        ranked = sorted(accepted, key=lambda item: item[1], reverse=True)
        selected = []
        for candidate in ranked:
            if any(
                self._box_iou(candidate[0], kept[0]) >= self._iou
                for kept in selected
            ):
                continue
            selected.append(candidate)
            if len(selected) >= self._max_detections:
                break
        return selected

    @staticmethod
    def _box_iou(first, second) -> float:
        first = np.asarray(first, dtype=np.float32)
        second = np.asarray(second, dtype=np.float32)
        intersection_min = np.maximum(first[:2], second[:2])
        intersection_max = np.minimum(first[2:], second[2:])
        intersection_size = np.maximum(0.0, intersection_max - intersection_min)
        intersection = float(intersection_size[0] * intersection_size[1])
        first_area = float(np.prod(np.maximum(0.0, first[2:] - first[:2])))
        second_area = float(np.prod(np.maximum(0.0, second[2:] - second[:2])))
        union = first_area + second_area - intersection
        return intersection / max(union, 1.0)

    @staticmethod
    def _box_associated(box, previous, width: int, height: int) -> bool:
        current = np.asarray(box, dtype=np.float32)
        old = np.asarray(previous, dtype=np.float32)
        current_center = (current[:2] + current[2:]) * 0.5
        old_center = (old[:2] + old[2:]) * 0.5
        center_distance = float(np.linalg.norm(current_center - old_center))
        diagonal = max(1.0, float(np.hypot(width, height)))
        if center_distance > 0.18 * diagonal:
            return False
        intersection_min = np.maximum(current[:2], old[:2])
        intersection_max = np.minimum(current[2:], old[2:])
        intersection_size = np.maximum(0.0, intersection_max - intersection_min)
        intersection = float(intersection_size[0] * intersection_size[1])
        current_area = float(np.prod(np.maximum(0.0, current[2:] - current[:2])))
        old_area = float(np.prod(np.maximum(0.0, old[2:] - old[:2])))
        union = current_area + old_area - intersection
        return intersection / max(union, 1.0) >= 0.05 or center_distance <= 0.08 * diagonal

    def _predict_edge_views(self, image: np.ndarray):
        """Batch shifted full-scale views so image-border fruit keeps context."""
        height, width = image.shape[:2]
        shift_x = max(8, int(round(width * self._edge_shift_ratio)))
        shift_y = max(8, int(round(height * self._edge_shift_ratio)))
        padded = cv2.copyMakeBorder(
            image,
            shift_y,
            shift_y,
            shift_x,
            shift_x,
            cv2.BORDER_REFLECT_101,
        )
        shift_groups = (
            ((-shift_x, 0), (shift_x, 0), (0, -shift_y), (0, shift_y)),
            (
                (-shift_x, -shift_y), (-shift_x, shift_y),
                (shift_x, -shift_y), (shift_x, shift_y),
            ),
        )
        inference_ms = 0.0
        for group_index, shifts in enumerate(shift_groups):
            views = []
            for dx, dy in shifts:
                start_x = shift_x + dx
                start_y = shift_y + dy
                views.append(
                    np.ascontiguousarray(
                        padded[start_y:start_y + height, start_x:start_x + width]
                    )
                )
            started = time.perf_counter()
            results = self._model.predict(
                source=views,
                imgsz=self._image_size,
                conf=self._tracking_confidence,
                iou=self._iou,
                classes=[self._apple_class_id],
                device=self._device,
                max_det=self._max_detections,
                verbose=False,
            )
            inference_ms += (time.perf_counter() - started) * 1000.0
            proposals = []
            for result, (dx, dy) in zip(results or [], shifts):
                boxes = getattr(result, "boxes", None)
                if boxes is None or len(boxes) == 0:
                    continue
                xyxy = boxes.xyxy.detach().cpu().numpy()
                scores = boxes.conf.detach().cpu().numpy()
                for box, score in zip(xyxy, scores):
                    mapped = np.asarray(
                        [
                            float(box[0]) + dx,
                            float(box[1]) + dy,
                            float(box[2]) + dx,
                            float(box[3]) + dy,
                        ],
                        dtype=np.float32,
                    )
                    center = (mapped[:2] + mapped[2:]) * 0.5
                    if 0.0 <= center[0] < width and 0.0 <= center[1] < height:
                        proposals.append((mapped, float(score)))
            selected = self._select_boxes(proposals, width, height)
            if selected:
                return selected, inference_ms, "edge_batch_%d" % group_index
        return [], inference_ms, "edge_batch_miss"

    def _publish_fast_annotation(
        self, image: np.ndarray, source: Image, predictions, inference_ms: float
    ) -> None:
        publisher = getattr(self, "_fast_annotated_pub", None)
        if publisher is None:
            return
        annotated = image.copy()
        height, width = annotated.shape[:2]
        for detection in predictions:
            box, score = detection.box, detection.score
            x1, y1, x2, y2 = [int(round(float(value))) for value in box]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width - 1, x2), min(height - 1, y2)
            edge = x1 <= 2 or y1 <= 2 or x2 >= width - 3 or y2 >= height - 3
            color = (
                (0, 165, 255)
                if detection.predicted
                else ((0, 180, 255) if edge else (0, 230, 0))
            )
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            suffix = " TRACK" if detection.predicted else (" EDGE" if edge else "")
            label = "APPLE %.2f%s" % (score, suffix)
            cv2.putText(
                annotated,
                label,
                (x1, max(18, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                2,
            )
        cv2.putText(
            annotated,
            "YOLO FAST %d  %.0fms" % (len(predictions), inference_ms),
            (8, max(18, height - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
        )
        msg = Image()
        msg.header = source.header
        msg.height, msg.width = annotated.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(annotated, dtype=np.uint8).tobytes()
        publisher.publish(msg)

    def _publish_fast_detections(self, source: Image, predictions) -> None:
        """Publish timestamped boxes so displays can draw on their newest RGB."""
        publisher = getattr(self, "_fast_detections_pub", None)
        if publisher is None:
            return
        output = Detection2DArray()
        output.header = source.header
        for index, item in enumerate(predictions):
            box, score = item.box, item.score
            x1, y1, x2, y2 = [float(value) for value in box]
            detection = Detection2D()
            detection.header = source.header
            detection.id = "apple_%02d" % index
            detection.bbox.center.position.x = (x1 + x2) * 0.5
            detection.bbox.center.position.y = (y1 + y2) * 0.5
            detection.bbox.size_x = max(0.0, x2 - x1)
            detection.bbox.size_y = max(0.0, y2 - y1)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = (
                "apple_predicted" if item.predicted else "apple"
            )
            hypothesis.hypothesis.score = float(score)
            detection.results.append(hypothesis)
            output.detections.append(detection)
        publisher.publish(output)

    def _display_predictions(self, predictions, source: Image, image_shape):
        tracker = getattr(self, "_fast_box_tracker", None)
        if tracker is None:
            return [
                DisplayDetection(
                    np.asarray(box, dtype=np.float32),
                    float(score),
                    predicted=False,
                    confirmed=False,
                )
                for box, score in predictions
            ]
        return tracker.update(
            predictions,
            self._stamp_seconds(source.header.stamp),
            image_shape,
        )

    def _grid_tile_bounds(self, width: int, height: int):
        x_starts, tile_width = self._axis_tiles(width, self._tile_cols)
        y_starts, tile_height = self._axis_tiles(height, self._tile_rows)
        return [
            (x, y, x + tile_width, y + tile_height)
            for y in y_starts
            for x in x_starts
        ]

    def _local_tile_bounds(self, width: int, height: int):
        tile_width = max(32, int(round(width * self._local_tile_width_ratio)))
        tile_height = max(32, int(round(height * self._local_tile_height_ratio)))
        bx1, by1, bx2, by2 = [float(value) for value in self._last_bbox]
        center_x = (bx1 + bx2) * 0.5
        center_y = (by1 + by2) * 0.5
        x1 = int(round(center_x - tile_width * 0.5))
        y1 = int(round(center_y - tile_height * 0.5))
        x1 = min(max(0, x1), max(0, width - tile_width))
        y1 = min(max(0, y1), max(0, height - tile_height))
        return x1, y1, min(width, x1 + tile_width), min(height, y1 + tile_height)

    def _axis_tiles(self, length: int, count: int):
        if count <= 1:
            return [0], length
        denominator = count - self._tile_overlap * (count - 1)
        tile = min(length, max(32, int(round(length / denominator))))
        step = max(1, int(round(tile * (1.0 - self._tile_overlap))))
        starts = [min(index * step, max(0, length - tile)) for index in range(count)]
        return starts, tile

    def _sample_depth(self, depth, x1, y1, x2, y2):
        width = max(1, int(round(x2 - x1)))
        height = max(1, int(round(y2 - y1)))
        # Avoid bbox edges, which commonly contain background/table pixels.
        inset_x = max(1, int(width * 0.22))
        inset_y = max(1, int(height * 0.22))
        xa, xb = int(x1) + inset_x, max(int(x1) + inset_x + 1, int(x2) - inset_x)
        ya, yb = int(y1) + inset_y, max(int(y1) + inset_y + 1, int(y2) - inset_y)
        patch = depth[max(0, ya):min(depth.shape[0], yb), max(0, xa):min(depth.shape[1], xb)]
        if patch.size == 0:
            return None
        valid = patch[
            np.isfinite(patch)
            & (patch > self._min_depth)
            & (patch < self._max_depth)
        ]
        coverage = float(valid.size) / float(patch.size)
        if valid.size < 4 or coverage < self._min_coverage:
            return None
        median = float(np.median(valid))
        mad = float(np.median(np.abs(valid - median)))
        if mad > 0.002:
            valid = valid[np.abs(valid - median) <= max(0.015, 3.0 * mad)]
        if valid.size == 0:
            return None
        return float(np.median(valid)), coverage

    def _transform_point(self, point, header, output_frame):
        transformed = self.transform_points(
            np.asarray(point, dtype=np.float32).reshape(1, 3),
            header,
            output_frame,
        )
        return None if transformed is None else transformed[0]

    def transform_points(self, points, header, output_frame):
        """Transform an Nx3 point array with one timestamped TF lookup."""
        source_frame = header.frame_id or self._camera_frame
        if not output_frame or output_frame == source_frame:
            return np.asarray(points, dtype=np.float32)
        try:
            transform = self._tf_buffer.lookup_transform(
                output_frame,
                source_frame,
                rclpy.time.Time(
                    seconds=header.stamp.sec,
                    nanoseconds=header.stamp.nanosec,
                ),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            now = time.monotonic()
            if now - self._last_tf_warning > 2.0:
                self._logger.warning(
                    "TF unavailable: %s -> %s" % (source_frame, output_frame)
                )
                self._last_tf_warning = now
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.asarray([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
        ], dtype=np.float32)
        translation = np.asarray([t.x, t.y, t.z], dtype=np.float32)
        values = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        return values @ rotation.T + translation

    def _inside_roi(self, point: np.ndarray) -> bool:
        if self._roi_min is None or self._roi_max is None:
            return True
        return bool(np.all(point >= self._roi_min) and np.all(point <= self._roi_max))
