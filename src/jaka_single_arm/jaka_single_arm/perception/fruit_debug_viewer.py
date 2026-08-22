"""Always-visible camera and perception status view for real-robot bring-up."""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection3DArray
from visualization_msgs.msg import MarkerArray, Marker


class FruitDebugViewer(Node):
    """Keep raw RGB visible until the perception pipeline produces annotations."""

    def __init__(self) -> None:
        super().__init__("fruit_debug_viewer")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "depth_fallback_topic", "/camera/camera/depth/image_rect_raw"
        )
        self.declare_parameter(
            "annotated_topic", "/perception/detection_annotated"
        )
        self.declare_parameter(
            "detections_2d_topic", "/perception/apple_detections_2d"
        )
        self.declare_parameter("target_topic", "/perception/stable_fruit_targets")
        self.declare_parameter("detected_topic", "/perception/detected_objects")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("output_topic", "/perception/debug/fruit_view")
        self.declare_parameter("depth_output_topic", "/perception/debug/depth_view")
        self.declare_parameter("publish_rate", 15.0)
        self.declare_parameter("show_windows", False)
        self.declare_parameter("rgb_window_name", "Fruit RGB Debug")
        self.declare_parameter("depth_window_name", "D455 Depth Debug")
        # Boxes are drawn on the newest raw RGB only when their source sensor
        # time is close enough. The old annotated image never replaces RGB.
        self.declare_parameter("annotated_max_age", 0.10)
        self.declare_parameter("image_timeout_s", 1.0)
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("joint_state_timeout_s", 1.0)
        self.declare_parameter("depth_timeout_s", 1.0)
        self.declare_parameter("camera_reconnect_grace_s", 30.0)
        self.declare_parameter("depth_display_min_m", 0.20)
        self.declare_parameter("depth_display_max_m", 2.00)

        get = self.get_parameter
        self._image_topic = str(get("image_topic").value)
        self._depth_topic = str(get("depth_topic").value)
        self._depth_fallback_topic = str(get("depth_fallback_topic").value)
        self._annotated_topic = str(get("annotated_topic").value)
        self._detections_2d_topic = str(get("detections_2d_topic").value)
        self._target_topic = str(get("target_topic").value)
        self._detected_topic = str(get("detected_topic").value)
        self._joint_topic = str(get("joint_state_topic").value)
        self._output_topic = str(get("output_topic").value)
        self._depth_output_topic = str(get("depth_output_topic").value)
        self._show_windows = bool(get("show_windows").value)
        self._rgb_window_name = str(get("rgb_window_name").value)
        self._depth_window_name = str(get("depth_window_name").value)
        self._annotated_max_age = float(get("annotated_max_age").value)
        self._image_timeout_s = max(0.1, float(get("image_timeout_s").value))
        self._target_timeout_s = max(0.1, float(get("target_timeout_s").value))
        self._joint_timeout_s = max(0.1, float(get("joint_state_timeout_s").value))
        self._depth_timeout_s = max(0.1, float(get("depth_timeout_s").value))
        self._depth_display_min_m = max(0.01, float(get("depth_display_min_m").value))
        self._depth_display_max_m = max(
            self._depth_display_min_m + 0.05,
            float(get("depth_display_max_m").value),
        )
        self._camera_reconnect_grace_s = max(
            self._image_timeout_s,
            float(get("camera_reconnect_grace_s").value),
        )
        self._started_at = time.monotonic()
        self._frame_lock = threading.Lock()

        # Store the newest ROS messages in the callbacks and convert at the
        # display rate only.  Converting every incoming RGB/depth frame and
        # then converting the generated debug image again can starve the
        # RealSense USB/IP callbacks on WSL.
        self._raw_msg: Image | None = None
        self._detections_2d_msg: Detection2DArray | None = None
        self._depth_msg: Image | None = None
        self._fallback_depth_msg: Image | None = None
        self._raw_image: np.ndarray | None = None
        self._raw_image_stamp = None
        self._depth_image_stamp = None
        self._raw_time = 0.0
        self._raw_frame_id = ""
        self._depth_image: np.ndarray | None = None
        self._depth_time = 0.0
        self._depth_frame_id = ""
        self._depth_min_m = 0.0
        self._depth_max_m = 0.0
        self._depth_valid_ratio = 0.0
        self._depth_stamp_text = "unknown"
        self._target_count = 0
        self._target_confidence = 0.0
        self._target_time = 0.0
        self._detected_count = 0
        self._detected_time = 0.0
        self._target_lines: list[str] = []
        self._joint_count = 0
        self._joint_time = 0.0
        self._last_status = ""
        self._last_status_signature = None
        self._last_status_log_time = 0.0
        self._rgb_previous_frame_time = 0.0
        self._depth_previous_frame_time = 0.0
        self._rgb_fps = 0.0
        self._depth_fps = 0.0
        self._window_error_reported = False
        self._windows_created: set[str] = set()
        self._primary_depth_time = 0.0

        self.create_subscription(
            Image, self._image_topic, self._on_raw_image, qos_profile_sensor_data
        )
        self.create_subscription(
            Image,
            self._depth_topic,
            lambda msg: self._on_depth_image(msg, primary=True),
            qos_profile_sensor_data,
        )
        if self._depth_fallback_topic != self._depth_topic:
            self.create_subscription(
                Image,
                self._depth_fallback_topic,
                lambda msg: self._on_depth_image(msg, primary=False),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Detection2DArray,
            self._detections_2d_topic,
            self._on_detections_2d,
            qos_profile_sensor_data,
        )
        self.create_subscription(Detection3DArray, self._target_topic, self._on_targets, 10)
        self.create_subscription(MarkerArray, self._detected_topic, self._on_detected, 10)
        self.create_subscription(JointState, self._joint_topic, self._on_joint_state, 10)
        self._image_pub = self.create_publisher(
            Image, self._output_topic, 10
        )
        self._depth_pub = self.create_publisher(
            Image, self._depth_output_topic, 10
        )
        self._status_pub = self.create_publisher(String, "/perception/debug/status", 10)

        rate = max(1.0, float(get("publish_rate").value))
        self.create_timer(1.0 / rate, self._publish_view)
        self.get_logger().info(
            "Debug views ready: rgb=%s depth=%s fallback=%s output=%s depth_output=%s"
            % (
                self._image_topic,
                self._depth_topic,
                self._depth_fallback_topic,
                self._output_topic,
                self._depth_output_topic,
            )
        )

    def _on_raw_image(self, msg: Image) -> None:
        now = time.monotonic()
        frame_time = self._message_time(msg, now)
        with self._frame_lock:
            self._raw_msg = msg
            self._raw_time = now
            self._raw_frame_id = msg.header.frame_id
            self._rgb_fps, self._rgb_previous_frame_time = self._update_rate(
                self._rgb_previous_frame_time, self._rgb_fps, frame_time
            )

    def _on_detections_2d(self, msg: Detection2DArray) -> None:
        with self._frame_lock:
            self._detections_2d_msg = msg

    def _on_depth_image(self, msg: Image, primary: bool = True) -> None:
        # The fallback topic is only used when the aligned stream has not
        # produced a frame recently.  Counting both topics made the displayed
        # depth FPS incorrect and caused duplicate conversion work.
        now = time.monotonic()
        with self._frame_lock:
            if not primary and now - self._primary_depth_time <= self._depth_timeout_s:
                return
            if primary:
                self._depth_msg = msg
                self._primary_depth_time = now
            else:
                self._fallback_depth_msg = msg
            frame_time = self._message_time(msg, now)
            self._depth_time = now
            self._depth_frame_id = msg.header.frame_id
            self._depth_stamp_text = "%d.%09d" % (
                msg.header.stamp.sec, msg.header.stamp.nanosec
            )
            self._depth_valid_ratio = self._depth_valid_ratio_from_msg(msg)
            self._depth_fps, self._depth_previous_frame_time = self._update_rate(
                self._depth_previous_frame_time, self._depth_fps, frame_time
            )

    def _on_targets(self, msg: Detection3DArray) -> None:
        self._target_count = len(msg.detections)
        self._target_time = time.monotonic()
        self._target_confidence = max(
            (
                float(detection.results[0].hypothesis.score)
                for detection in msg.detections
                if detection.results
            ),
            default=0.0,
        )
        lines: list[str] = []
        for detection in msg.detections[:5]:
            label = "unknown"
            score = 0.0
            if detection.results:
                result = detection.results[0]
                label = result.hypothesis.class_id or "unknown"
                score = float(result.hypothesis.score)
            p = detection.bbox.center.position
            lines.append(
                "%s %s %.2f xyz=(%.3f, %.3f, %.3f)"
                % (detection.id or "fruit", label, score, p.x, p.y, p.z)
            )
        self._target_lines = lines

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_count = len(msg.position)
        self._joint_time = time.monotonic()

    def _on_detected(self, msg: MarkerArray) -> None:
        self._detected_count = sum(
            1 for marker in msg.markers if marker.type == Marker.SPHERE
        )
        self._detected_time = time.monotonic()

    def _publish_view(self) -> None:
        now = time.monotonic()
        with self._frame_lock:
            raw_time = self._raw_time
            raw_frame_id = self._raw_frame_id
            raw_msg = self._raw_msg
            detections_2d_msg = self._detections_2d_msg
        if raw_msg is not None:
            stamp = (raw_msg.header.stamp.sec, raw_msg.header.stamp.nanosec)
            if stamp != self._raw_image_stamp:
                converted = self._to_bgr(raw_msg)
                if converted is not None:
                    self._raw_image = converted
                    self._raw_image_stamp = stamp
        raw_age = now - raw_time if raw_time > 0.0 else float("inf")
        camera_online = raw_age <= self._image_timeout_s
        camera_state = self._stream_state(
            raw_time, self._image_timeout_s, now
        )
        image = self._raw_image if camera_online else None
        source_frame = raw_frame_id if camera_online else ""
        source_msg = raw_msg if camera_online else None
        view_kind = "RAW RGB" if camera_online else "CAMERA " + camera_state
        target_fresh = (
            self._target_time > 0.0
            and now - self._target_time <= self._target_timeout_s
        )
        detected_fresh = (
            self._detected_time > 0.0
            and now - self._detected_time <= self._target_timeout_s
        )
        joint_fresh = (
            self._joint_time > 0.0
            and now - self._joint_time <= self._joint_timeout_s
        )
        target_count = self._target_count if target_fresh and camera_online else 0
        detected_count = self._detected_count if detected_fresh and camera_online else 0
        target_lines = self._target_lines if target_fresh and camera_online else []
        detections_match_raw = self._annotation_matches_raw(
            raw_msg, detections_2d_msg, self._annotated_max_age
        )

        has_image = image is not None
        if image is None:
            # Diagnostic card only; never substitute sensor data. A black
            # placeholder made a missing USB/IP device look like a bad viewer.
            image = self._diagnostic_card(
                {
                    "WAITING": "CAMERA WAITING FOR FIRST REAL FRAME",
                    "RECONNECTING": "CAMERA RECONNECTING",
                    "OFFLINE": "CAMERA OFFLINE",
                }.get(camera_state, "CAMERA OFFLINE"),
                [
                    "NO SIMULATION DATA",
                    "RealSense will keep retrying in the background",
                    "Check usbipd attach and D455 USB if this persists",
                ],
            )
            source_frame = ""
            view_kind = "NO IMAGE"
        else:
            image = image.copy()
            if detections_match_raw:
                self._draw_detections_2d(image, detections_2d_msg)
                view_kind = "LATEST RGB + YOLO BOX"

        # Keep the sensor image visually honest. The old 170 px overlay
        # covered roughly 70% of a 240 px D455 frame and made the real scene
        # look grey. Only reserve two compact status rows now.
        panel_height = min(58, max(1, image.shape[0]))
        if has_image:
            image = self._apply_status_overlay(image, panel_height)

        annotated_ok = (
            camera_online
            and detections_match_raw
            and detections_2d_msg is not None
        )
        recognition_text = "YOLO LIVE" if annotated_ok else "WAITING FOR FRESH BOX"
        robot_text = "%d joints" % self._joint_count if joint_fresh else "WAITING"
        cv2.putText(
            image,
            "CAMERA: %s  RGB: %.1f Hz  RECOGNITION: %s  DETECTED: %d  STABLE: %d"
            % (
                camera_state,
                self._rgb_fps if camera_online else 0.0,
                recognition_text,
                detected_count,
                target_count,
            ),
            (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.46,
            (80, 230, 80) if camera_online else (0, 200, 255), 2,
        )
        cv2.putText(
            image,
            "DEPTH: %.1f Hz valid=%.0f%%  RGB age=%.0fms  XYZ age=%.0fms conf=%.2f  %s"
            % (
                self._depth_fps,
                self._depth_valid_ratio * 100.0,
                raw_age * 1000.0 if camera_online else float("inf"),
                (now - self._target_time) * 1000.0 if target_fresh else float("inf"),
                self._target_confidence if target_fresh else 0.0,
                view_kind,
            ),
            (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (255, 255, 255), 1,
        )
        # Per-fruit TYPE/QUALITY/CONF labels are drawn next to each box by
        # HealthFusion. Do not cover the camera image with a second target list.

        output = Image()
        if source_msg is not None:
            output.header.stamp = source_msg.header.stamp
        output.header.frame_id = source_frame
        output.height, output.width = image.shape[:2]
        output.encoding = "bgr8"
        output.is_bigendian = 0
        output.step = output.width * 3
        output.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
        try:
            self._image_pub.publish(output)
        except RCLError:
            # ros2 launch can invalidate the context while a timer callback is
            # still unwinding during shutdown.  Do not turn a normal stop into
            # a misleading node crash.
            return

        status = String()
        status.data = (
            "camera=%s rgb=%.1fHz depth=%.1fHz recognition=%s detected=%d "
            "stable=%d joints=%d view=%s"
            % (
                camera_state,
                self._rgb_fps if camera_online else 0.0,
                self._depth_fps
                if self._depth_time > 0.0
                and now - self._depth_time <= self._depth_timeout_s
                else 0.0,
                recognition_text,
                detected_count,
                target_count,
                self._joint_count if joint_fresh else 0,
                view_kind,
            )
        )
        try:
            self._status_pub.publish(status)
        except RCLError:
            return
        status_signature = (
            camera_state,
            recognition_text,
            detected_count,
            target_count,
            self._joint_count if joint_fresh else 0,
            view_kind,
        )
        if (
            status_signature != self._last_status_signature
            or now - self._last_status_log_time >= 2.0
        ):
            self.get_logger().info(status.data)
            self._last_status_signature = status_signature
            self._last_status_log_time = now
        self._last_status = status.data
        if self._show_windows:
            self._show_window(self._rgb_window_name, image)
        self._publish_depth_view(now)

    @classmethod
    def _annotation_matches_raw(
        cls, raw_msg: Image | None, annotated_msg, max_delta_s: float
    ) -> bool:
        """Accept annotations only for the current raw sensor frame.

        Arrival time is deliberately not used: CPU inference can finish late,
        while the ROS source stamps still reveal that the result belongs to an
        older frame.
        """
        if raw_msg is None or annotated_msg is None:
            return False
        raw_stamp = (
            float(raw_msg.header.stamp.sec)
            + float(raw_msg.header.stamp.nanosec) * 1.0e-9
        )
        annotated_stamp = (
            float(annotated_msg.header.stamp.sec)
            + float(annotated_msg.header.stamp.nanosec) * 1.0e-9
        )
        if raw_stamp <= 0.0 or annotated_stamp <= 0.0:
            return False
        return abs(raw_stamp - annotated_stamp) <= max(0.0, float(max_delta_s))

    @staticmethod
    def _draw_detections_2d(image: np.ndarray, msg: Detection2DArray) -> None:
        height, width = image.shape[:2]
        for detection in msg.detections:
            center = detection.bbox.center.position
            half_width = float(detection.bbox.size_x) * 0.5
            half_height = float(detection.bbox.size_y) * 0.5
            x1 = max(0, int(round(float(center.x) - half_width)))
            y1 = max(0, int(round(float(center.y) - half_height)))
            x2 = min(width - 1, int(round(float(center.x) + half_width)))
            y2 = min(height - 1, int(round(float(center.y) + half_height)))
            edge = x1 <= 2 or y1 <= 2 or x2 >= width - 3 or y2 >= height - 3
            color = (0, 180, 255) if edge else (0, 230, 0)
            score = (
                float(detection.results[0].hypothesis.score)
                if detection.results
                else 0.0
            )
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                image,
                "APPLE %.2f%s" % (score, " EDGE" if edge else ""),
                (x1, max(18, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                2,
            )

    def _publish_depth_view(self, now: float) -> None:
        # Avoid an expensive color-map, image copy and DDS serialization when
        # nobody requested the separate depth view. RGB/depth callbacks and
        # FPS monitoring remain active.
        if not self._show_windows and self._depth_pub.get_subscription_count() == 0:
            return
        with self._frame_lock:
            depth_time = self._depth_time
            primary_depth_time = self._primary_depth_time
            primary_msg = self._depth_msg
            fallback_msg = self._fallback_depth_msg
            depth_fps = self._depth_fps
        depth_msg = (
            primary_msg
            if primary_msg is not None
            and now - primary_depth_time <= self._depth_timeout_s
            else fallback_msg
        )
        if depth_msg is not None:
            stamp = (depth_msg.header.stamp.sec, depth_msg.header.stamp.nanosec)
            if stamp != self._depth_image_stamp:
                converted, minimum, maximum = self._to_depth_bgr(
                    depth_msg, self._depth_display_min_m, self._depth_display_max_m
                )
                if converted is not None:
                    self._depth_image = converted
                    self._depth_min_m = minimum
                    self._depth_max_m = maximum
                    self._depth_stamp_text = "%d.%09d" % (
                        depth_msg.header.stamp.sec, depth_msg.header.stamp.nanosec
                    )
                    self._depth_frame_id = depth_msg.header.frame_id
                    self._depth_image_stamp = stamp

        depth_age = now - depth_time if depth_time > 0.0 else float("inf")
        depth_online = depth_age <= self._depth_timeout_s
        depth_state = self._stream_state(
            depth_time, self._depth_timeout_s, now
        )
        if depth_online and self._depth_image is not None:
            image = self._depth_image.copy()
            frame_id = self._depth_frame_id
            cv2.putText(
                image,
                "DEPTH: ONLINE  range=%.2f-%.2f m  frame=%s"
                % (
                    self._depth_min_m,
                    self._depth_max_m,
                    (frame_id.rsplit("/", 1)[-1] if frame_id else "unknown"),
                ),
                (12, 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                image,
                "STREAM: %.1f Hz  valid=%.1f%%  stamp=%s"
                % (
                    depth_fps,
                    self._depth_valid_ratio * 100.0,
                    self._depth_stamp_text,
                ),
                (12, 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )
        else:
            image = self._diagnostic_card(
                {
                    "WAITING": "DEPTH WAITING FOR FIRST REAL FRAME",
                    "RECONNECTING": "DEPTH RECONNECTING",
                    "OFFLINE": "DEPTH OFFLINE",
                }.get(depth_state, "DEPTH OFFLINE"),
                [
                    "RGB/depth windows use fresh real frames",
                    "No synthetic depth is generated",
                ],
            )
            frame_id = ""

        output = Image()
        if depth_msg is not None:
            output.header.stamp = depth_msg.header.stamp
        output.header.frame_id = frame_id
        output.height, output.width = image.shape[:2]
        output.encoding = "bgr8"
        output.is_bigendian = 0
        output.step = output.width * 3
        output.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
        try:
            self._depth_pub.publish(output)
        except RCLError:
            return
        if self._show_windows:
            self._show_window(self._depth_window_name, image)

    def _stream_state(self, last_frame: float, timeout_s: float, now: float) -> str:
        """Describe startup/reconnect honestly without treating old frames as live."""
        if last_frame > 0.0 and now - last_frame <= timeout_s:
            return "ONLINE"
        if last_frame <= 0.0 and now - self._started_at <= self._camera_reconnect_grace_s:
            return "WAITING"
        if last_frame > 0.0 and now - last_frame <= self._camera_reconnect_grace_s:
            return "RECONNECTING"
        return "OFFLINE"

    @staticmethod
    def _apply_status_overlay(image: np.ndarray, height: int = 58) -> np.ndarray:
        """Darken only the compact header; leave the sensor pixels below intact."""
        result = image.copy()
        header_height = min(max(1, int(height)), result.shape[0])
        overlay = result[:header_height].copy()
        overlay[:, :] = (20, 20, 20)
        result[:header_height] = cv2.addWeighted(
            overlay, 0.72, result[:header_height], 0.28, 0.0
        )
        return result

    @staticmethod
    def _message_time(msg: Image, fallback: float) -> float:
        stamp = msg.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        return value if value > 0.0 else fallback

    @staticmethod
    def _update_rate(
        previous_frame_time: float, previous_rate: float, now: float
    ) -> tuple[float, float]:
        """Return a current EWMA frame rate instead of a lifetime average."""
        if previous_frame_time <= 0.0:
            return 0.0, now
        elapsed = now - previous_frame_time
        if elapsed <= 0.0 or elapsed > 2.0:
            return 0.0, now
        instantaneous = 1.0 / elapsed
        if previous_rate <= 0.0:
            return instantaneous, now
        return 0.8 * previous_rate + 0.2 * instantaneous, now

    def _show_window(self, name: str, image: np.ndarray) -> None:
        """Show a local debug window without making ROS depend on GUI support."""
        try:
            if name not in self._windows_created:
                cv2.namedWindow(name, cv2.WINDOW_NORMAL)
                self._windows_created.add(name)
            cv2.imshow(name, image)
            cv2.waitKey(1)
        except cv2.error as exc:
            if not self._window_error_reported:
                self.get_logger().warning(
                    "OpenCV debug window unavailable (%s); ROS image topics "
                    "remain active for rqt_image_view" % exc
                )
                self._window_error_reported = True
            self._show_windows = False

    @staticmethod
    def _diagnostic_card(title: str, lines: list[str]) -> np.ndarray:
        """Create a visible status card without inventing a sensor frame."""
        image = np.full((480, 848, 3), (48, 44, 24), dtype=np.uint8)
        cv2.rectangle(image, (8, 8), (840, 472), (0, 200, 255), 3)
        cv2.putText(
            image,
            title,
            (30, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.92,
            (0, 230, 255),
            2,
        )
        for index, line in enumerate(lines, start=1):
            cv2.putText(
                image,
                line,
                (30, 90 + index * 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.68 if index == 1 else 0.62,
                (255, 255, 255) if index == 1 else (210, 220, 255),
                2 if index == 1 else 1,
            )
        return image

    @staticmethod
    def _to_bgr(msg: Image) -> np.ndarray | None:
        try:
            encoding = msg.encoding.lower()
            channels = 1 if encoding in ("mono8", "8uc1") else 3
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            row_width = int(msg.step) if msg.step else int(msg.width * channels)
            rows = raw.reshape(msg.height, row_width)
            image = rows[:, : msg.width * channels]
            if channels == 1:
                return cv2.cvtColor(image.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
            image = image.reshape(msg.height, msg.width, channels)
            if encoding in ("rgb8", "rgba8"):
                image = image[:, :, :3][:, :, ::-1]
            return np.ascontiguousarray(image)
        except (TypeError, ValueError, cv2.error):
            return None

    @staticmethod
    def _to_depth_bgr(
        msg: Image,
        display_min_m: float | None = None,
        display_max_m: float | None = None,
    ) -> tuple[np.ndarray | None, float, float]:
        """Convert Z16/32FC1 depth into a readable, normalized color image."""
        try:
            encoding = msg.encoding.lower()
            if encoding in ("16uc1", "mono16", "16sc1"):
                dtype = np.dtype("<u2" if encoding != "16sc1" else "<i2")
                scale_to_m = 0.001
            elif encoding in ("32fc1", "32fc"):
                dtype = np.dtype("<f4")
                scale_to_m = 1.0
            elif encoding in ("8uc1", "mono8"):
                dtype = np.dtype("u1")
                scale_to_m = 0.001
            else:
                return None, 0.0, 0.0

            itemsize = dtype.itemsize
            step = int(msg.step) if msg.step else int(msg.width * itemsize)
            row_values = step // itemsize
            raw = np.frombuffer(msg.data, dtype=dtype)
            rows = raw.reshape(msg.height, row_values)
            depth = rows[:, : msg.width].astype(np.float32) * scale_to_m
            valid = depth[np.isfinite(depth) & (depth > 0.05)]
            if valid.size == 0:
                return None, 0.0, 0.0

            if display_min_m is None or display_max_m is None:
                minimum, maximum = np.percentile(valid, (2.0, 98.0)).astype(float)
            else:
                minimum = float(display_min_m)
                maximum = float(display_max_m)
            maximum = max(maximum, minimum + 0.05)
            normalized = np.clip(
                (depth - minimum) * 255.0 / (maximum - minimum), 0.0, 255.0
            ).astype(np.uint8)
            normalized[~np.isfinite(depth) | (depth <= 0.05)] = 0
            color = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
            color[normalized == 0] = (0, 0, 0)
            return np.ascontiguousarray(color), minimum, maximum
        except (TypeError, ValueError, cv2.error):
            return None, 0.0, 0.0

    @staticmethod
    def _depth_valid_ratio_from_msg(msg: Image) -> float:
        try:
            encoding = msg.encoding.lower()
            if encoding in ("16uc1", "mono16", "16sc1"):
                dtype = np.dtype("<u2" if encoding != "16sc1" else "<i2")
                scale_to_m = 0.001
            elif encoding in ("32fc1", "32fc"):
                dtype = np.dtype("<f4")
                scale_to_m = 1.0
            elif encoding in ("8uc1", "mono8"):
                dtype = np.dtype("u1")
                scale_to_m = 0.001
            else:
                return 0.0
            step = int(msg.step) if msg.step else int(msg.width * dtype.itemsize)
            row_values = step // dtype.itemsize
            raw = np.frombuffer(msg.data, dtype=dtype)
            depth = raw.reshape(msg.height, row_values)[:, : msg.width].astype(
                np.float32
            ) * scale_to_m
            valid = np.isfinite(depth) & (depth > 0.05)
            return float(np.count_nonzero(valid)) / float(max(1, depth.size))
        except (TypeError, ValueError):
            return 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FruitDebugViewer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
