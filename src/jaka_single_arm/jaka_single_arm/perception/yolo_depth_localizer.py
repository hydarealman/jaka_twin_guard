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
from sensor_msgs.msg import CameraInfo, Image

from jaka_single_arm.perception.object_detector import DetectedObject


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
        self._iou = float(detector_cfg.get("iou", 0.50))
        self._image_size = int(detector_cfg.get("image_size", 640))
        self._max_detections = max(1, int(detector_cfg.get("max_detections", 5)))
        self._inference_threads = max(1, int(detector_cfg.get("inference_threads", 1)))
        self._device = str(detector_cfg.get("device", "cpu"))
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
        self._last_stats_log = time.monotonic()
        self._stats = {
            "frames": 0.0,
            "raw_boxes": 0.0,
            "no_box_frames": 0.0,
            "accepted": 0.0,
            "depth_rejected": 0.0,
            "radius_rejected": 0.0,
            "tf_rejected": 0.0,
            "roi_rejected": 0.0,
            "inference_ms": 0.0,
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
            "YOLO RGB-D localizer loaded model=%s apple_class=%d conf=%.2f imgsz=%d device=%s threads=%d tiles=%s"
            % (
                self._model_path,
                self._apple_class_id,
                self._confidence,
                self._image_size,
                self._device,
                self._inference_threads,
                self._small_object_tiles,
            )
        )

    def _record_stats(self, **values) -> None:
        self._stats["frames"] += 1.0
        for key, value in values.items():
            if key in self._stats:
                self._stats[key] += float(value)
        now = time.monotonic()
        if now - self._last_stats_log < 2.0:
            return
        frames = max(1.0, self._stats["frames"])
        self._logger.info(
            "YOLO RGB-D stats: frames=%d raw_apple=%d no_box_frames=%d "
            "accepted=%d reject_depth=%d reject_radius=%d reject_tf=%d "
            "reject_roi=%d infer=%.1fms cpu_threads=%d"
            % (
                int(frames),
                int(self._stats["raw_boxes"]),
                int(self._stats["no_box_frames"]),
                int(self._stats["accepted"]),
                int(self._stats["depth_rejected"]),
                int(self._stats["radius_rejected"]),
                int(self._stats["tf_rejected"]),
                int(self._stats["roi_rejected"]),
                self._stats["inference_ms"] / frames,
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
        x1, y1, x2, y2 = 0, 0, width, height
        mode = "full"
        if self._small_object_tiles:
            if (
                self._last_bbox is not None
                and self._last_tile_bounds is not None
                and self._local_tile_misses <= self._max_local_tile_misses
            ):
                # Reuse the successful grid context. A tighter crop around
                # the box sounds attractive, but small changes in context
                # can make the COCO model lose the fruit entirely.
                x1, y1, x2, y2 = self._last_tile_bounds
                mode = "track_tile"
            elif self._last_bbox is not None and self._local_tile_misses <= self._max_local_tile_misses:
                x1, y1, x2, y2 = self._local_tile_bounds(width, height)
                mode = "local"
            else:
                bounds = self._grid_tile_bounds(width, height)
                x1, y1, x2, y2 = bounds[self._tile_cursor]
                self._tile_cursor = (self._tile_cursor + 1) % len(bounds)
                mode = "grid"
        self._last_prediction_bounds = (x1, y1, x2, y2)
        crop = np.ascontiguousarray(image[y1:y2, x1:x2])
        started = time.perf_counter()
        results = self._model.predict(
            source=crop,
            imgsz=self._image_size,
            conf=self._confidence,
            iou=self._iou,
            classes=[self._apple_class_id],
            device=self._device,
            max_det=self._max_detections,
            verbose=False,
        )
        inference_ms = (time.perf_counter() - started) * 1000.0
        self._last_prediction_mode = mode
        if not results:
            return [], inference_ms
        boxes = getattr(results[0], "boxes", None)
        if boxes is None or len(boxes) == 0:
            return [], inference_ms
        xyxy = boxes.xyxy.detach().cpu().numpy()
        scores = boxes.conf.detach().cpu().numpy()
        predictions = []
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
            predictions.append((mapped, float(score)))
        return predictions, inference_ms

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
        source_frame = header.frame_id or self._camera_frame
        if not output_frame or output_frame == source_frame:
            return point
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
        return rotation @ point + translation

    def _inside_roi(self, point: np.ndarray) -> bool:
        if self._roi_min is None or self._roi_max is None:
            return True
        return bool(np.all(point >= self._roi_min) and np.all(point <= self._roi_max))
