#!/usr/bin/env python3
"""RealSense Camera — interfaces with Intel RealSense D400 series.

Subscribes to standard realsense2_camera ROS2 driver topics.
For future deployment with physical RealSense D435/D455 cameras.

RealSense ROS2 topics (configurable via perception_params.yaml):
  /camera/camera/depth/color/points  — PointCloud2 (aligned)
  /camera/camera/depth/image_rect_raw — depth Image
  /camera/camera/color/image_raw      — color Image
"""


# CameraInterface 的 RealSense 实现：订阅点云、深度和彩色图像，
# 缓存最新数据供上层通过统一接口读取。

from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from jaka_single_arm.perception.camera_interface import CameraInterface


class RealSenseCamera(CameraInterface):
    """Bridges Intel RealSense D400 camera topics into CameraInterface.

    Requires the realsense2_camera ROS2 driver running separately:
        ros2 launch realsense2_camera rs_launch.py depth_module.profile:=848x480x30

    Usage:
        camera = RealSenseCamera(node, perception_cfg)
        if camera.connect():
            cloud = camera.get_point_cloud()
    """

    def __init__(self, node: Node, config: dict, scene_cfg: dict = None):
        super().__init__(node, config)
        rs_cfg = config.get("realsense_camera", {})

        self._depth_topic = rs_cfg.get("depth_topic", "/camera/camera/depth/color/points")
        self._depth_image_topic = rs_cfg.get(
            "depth_image_topic", "/camera/camera/depth/image_rect_raw"
        )
        self._aligned_depth_topic = rs_cfg.get(
            "aligned_depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self._color_image_topic = rs_cfg.get(
            "color_image_topic", "/camera/camera/color/image_raw"
        )
        self._color_camera_info_topic = rs_cfg.get(
            "color_camera_info_topic", "/camera/camera/color/camera_info"
        )
        self._camera_info_topic = rs_cfg.get(
            "camera_info_topic", "/camera/camera/depth/camera_info"
        )
        self._localizer_backend = str(config.get("localizer_backend", "")).strip().lower()
        self._generate_point_cloud = bool(rs_cfg.get("generate_point_cloud", True))
        self._subscribe_raw_depth = bool(rs_cfg.get("subscribe_raw_depth", True))
        self._subscribe_legacy_cloud = bool(rs_cfg.get("subscribe_legacy_cloud", True))
        if self._localizer_backend == "yolo_depth":
            # RGB-D localisation only needs RGB, registered depth and the
            # colour intrinsics. Do not carry the legacy raw-depth/cloud path
            # through WSL when it cannot be consumed by this backend.
            self._generate_point_cloud = False
            self._subscribe_raw_depth = False
            self._subscribe_legacy_cloud = False
        self._point_cloud_downsample = max(
            1, int(rs_cfg.get("point_cloud_downsample", 2))
        )

        # 最新数据缓存
        # 异步更新 同步读取 最新值缓存
        """
        避免阻塞
        适应不同频率
        轻量回调
        内存友好
        """
        # Latest data buffers
        self._latest_cloud: PointCloud2 | None = None
        self._latest_depth: Image | None = None
        self._latest_aligned_depth: Image | None = None
        self._latest_rgb: Image | None = None
        self._latest_camera_info: CameraInfo | None = None
        self._latest_color_camera_info: CameraInfo | None = None
        self._latest_cloud_time = 0.0
        self._latest_depth_time = 0.0
        self._latest_aligned_depth_time = 0.0
        self._latest_rgb_time = 0.0
        self._latest_cloud_stamp = None
        # Keep a short transport-latency history. WSL/USB-IP can queue a
        # depth frame behind RGB for several sensor periods; the timestamp
        # gate below still prevents pairing frames from different instants.
        self._rgb_frames = deque(maxlen=30)
        self._aligned_depth_frames = deque(maxlen=30)
        self._frame_lock = threading.Lock()
        self._last_sync_warning = 0.0

        # 订阅者句柄
        # Subscribers
        self._cloud_sub = None
        self._depth_sub = None
        self._aligned_depth_sub = None
        self._rgb_sub = None
        self._camera_info_sub = None
        self._color_camera_info_sub = None

    def connect(self) -> bool:
        """
        幂等性: 如果已经连接,直接返回True
        避免重复创建订阅者,多次调用connect()是安全的
        不会产生副作用
        """
        if self._connected:
            return True

        if not self._generate_point_cloud and self._subscribe_legacy_cloud:
            self._cloud_sub = self._node.create_subscription(
                PointCloud2, self._depth_topic, self._on_cloud,
                qos_profile_sensor_data,
            )
        if self._subscribe_raw_depth:
            self._depth_sub = self._node.create_subscription(
                Image, self._depth_image_topic, self._on_depth,
                qos_profile_sensor_data,
            )
        if self._aligned_depth_topic != self._depth_image_topic:
            self._aligned_depth_sub = self._node.create_subscription(
                Image, self._aligned_depth_topic, self._on_aligned_depth,
                qos_profile_sensor_data,
            )
        self._rgb_sub = self._node.create_subscription(
            Image, self._color_image_topic, self._on_rgb,
            qos_profile_sensor_data,
        )
        self._color_camera_info_sub = self._node.create_subscription(
            CameraInfo, self._color_camera_info_topic, self._on_color_camera_info,
            qos_profile_sensor_data,
        )
        if self._generate_point_cloud:
            self._camera_info_sub = self._node.create_subscription(
                CameraInfo, self._camera_info_topic, self._on_camera_info,
                qos_profile_sensor_data,
            )

        self._connected = True
        self._logger.info(
            f"RealSense subscriptions created: cloud="
            f"{'depth-derived' if self._generate_point_cloud else (self._depth_topic if self._subscribe_legacy_cloud else 'disabled')}, "
            f"raw_depth={'enabled' if self._subscribe_raw_depth else 'disabled'}, "
            f"aligned_depth={self._aligned_depth_topic}, rgb={self._color_image_topic}"
        )
        return True

    def disconnect(self) -> bool:
        self._connected = False
        for sub in (
            self._cloud_sub,
            self._depth_sub,
            self._aligned_depth_sub,
            self._rgb_sub,
            self._camera_info_sub,
            self._color_camera_info_sub,
        ):
            if sub:
                self._node.destroy_subscription(sub)
        self._cloud_sub = self._depth_sub = self._aligned_depth_sub = None
        self._rgb_sub = self._camera_info_sub = self._color_camera_info_sub = None
        return True

    def get_point_cloud(self) -> PointCloud2 | None:
        if getattr(self, "_generate_point_cloud", False):
            self._update_depth_cloud()
        return self._latest_cloud

    def get_depth_image(self) -> Image | None:
        return self._latest_depth

    def get_aligned_depth_image(self) -> Image | None:
        return self._latest_aligned_depth or self._latest_depth

    def get_color_camera_info(self) -> CameraInfo | None:
        return self._latest_color_camera_info

    def get_rgb_image(self) -> Image | None:
        return self._latest_rgb

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg
        self._latest_cloud_time = time.monotonic()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _on_color_camera_info(self, msg: CameraInfo) -> None:
        self._latest_color_camera_info = msg

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth = msg
        self._latest_depth_time = time.monotonic()

    def _on_aligned_depth(self, msg: Image) -> None:
        self._latest_aligned_depth = msg
        self._latest_aligned_depth_time = time.monotonic()
        with self._frame_lock:
            self._aligned_depth_frames.append(msg)

    def _on_rgb(self, msg: Image) -> None:
        self._latest_rgb = msg
        self._latest_rgb_time = time.monotonic()
        with self._frame_lock:
            self._rgb_frames.append(msg)

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def get_synced_rgbd(self, max_delta_s: float = 0.033):
        """Return the closest cached RGB/aligned-depth pair.

        USB/IP can deliver the two sensor streams with a fixed transport
        offset even though their sensor timestamps still match. Comparing
        only the newest RGB frame would then reject every frame. Search the
        short history instead; the pair is still required to satisfy the
        strict timestamp tolerance, so a moving object is never paired with
        an unrelated depth frame.
        """
        # Copy the tiny deques before iterating: the ROS executor appends from
        # a camera callback while the perception worker searches the history.
        frame_lock = getattr(self, "_frame_lock", None)
        if frame_lock is None:
            rgb_frames = list(self._rgb_frames)
            depth_frames = list(self._aligned_depth_frames)
        else:
            with frame_lock:
                rgb_frames = list(self._rgb_frames)
                depth_frames = list(self._aligned_depth_frames)
        if not rgb_frames or not depth_frames:
            return None
        best = min(
            [
                (
                rgb_item,
                depth_item,
                abs(
                    self._stamp_seconds(rgb_item.header.stamp)
                    - self._stamp_seconds(depth_item.header.stamp)
                ),
                )
                for rgb_item in rgb_frames
                for depth_item in depth_frames
            ],
            key=lambda item: item[2],
        )
        rgb, depth, delta = best
        info = self._latest_color_camera_info
        if info is None or delta > float(max_delta_s):
            now = time.monotonic()
            logger = getattr(self, "_logger", None)
            if (
                logger is not None
                and now - getattr(self, "_last_sync_warning", 0.0) >= 2.0
            ):
                logger.warning(
                    "RGB-D cache has no pair within %.1fms: rgb=%d depth=%d "
                    "best=%.1fms" % (
                        float(max_delta_s) * 1000.0,
                        len(rgb_frames),
                        len(depth_frames),
                        delta * 1000.0,
                    )
                )
                self._last_sync_warning = now
            return None
        return rgb, depth, info, delta

    def _update_depth_cloud(self) -> None:
        """Create a compact cloud from the latest measured depth frame.

        The RealSense driver can publish a very large textured cloud.  On
        WSL/USB-IP that message becomes the bottleneck even when the RGB and
        depth image streams are healthy.  This cloud uses only measured Z16
        depth and calibrated intrinsics; it contains no scene fallback.
        """
        depth = self._latest_depth
        info = self._latest_camera_info
        if depth is None or info is None:
            return
        stamp = (depth.header.stamp.sec, depth.header.stamp.nanosec)
        if stamp == self._latest_cloud_stamp:
            return
        cloud = self._depth_to_cloud(depth, info)
        if cloud is None:
            return
        self._latest_cloud = cloud
        self._latest_cloud_time = self._latest_depth_time
        self._latest_cloud_stamp = stamp

    def _depth_to_cloud(
        self, depth_msg: Image, info: CameraInfo
    ) -> PointCloud2 | None:
        try:
            encoding = depth_msg.encoding.lower()
            if encoding in ("16uc1", "mono16"):
                dtype = np.dtype("<u2")
                scale_to_m = 0.001
            elif encoding in ("32fc1", "32fc"):
                dtype = np.dtype("<f4")
                scale_to_m = 1.0
            else:
                self._logger.warning(
                    f"Unsupported depth encoding for cloud generation: {depth_msg.encoding}"
                )
                return None

            itemsize = dtype.itemsize
            step_bytes = int(depth_msg.step or depth_msg.width * itemsize)
            row_values = step_bytes // itemsize
            raw = np.frombuffer(depth_msg.data, dtype=dtype)
            depth = raw.reshape(depth_msg.height, row_values)[:, : depth_msg.width]
            depth = depth.astype(np.float32, copy=False) * scale_to_m

            stride = self._point_cloud_downsample
            depth = depth[::stride, ::stride]
            fy = float(info.k[4])
            fx = float(info.k[0])
            cx = float(info.k[2])
            cy = float(info.k[5])
            if min(fx, fy) <= 0.0:
                return None
            rows, cols = depth.shape
            u, v = np.meshgrid(
                np.arange(cols, dtype=np.float32) * stride,
                np.arange(rows, dtype=np.float32) * stride,
            )
            valid = np.isfinite(depth) & (depth > 0.05) & (depth < 10.0)
            if not np.any(valid):
                return None
            z = depth[valid]
            xyz = np.empty((z.size,), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
            xyz["x"] = (u[valid] - cx) * z / fx
            xyz["y"] = (v[valid] - cy) * z / fy
            xyz["z"] = z

            cloud = PointCloud2()
            cloud.header = depth_msg.header
            cloud.height = 1
            cloud.width = int(xyz.shape[0])
            cloud.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            cloud.is_bigendian = False
            cloud.point_step = 12
            cloud.row_step = cloud.point_step * cloud.width
            cloud.is_dense = True
            cloud.data = xyz.tobytes()
            return cloud
        except (TypeError, ValueError, BufferError) as exc:
            self._logger.warning(f"Unable to derive point cloud from depth: {exc}")
            return None

    def get_point_cloud_age_s(self) -> float:
        if getattr(self, "_generate_point_cloud", False):
            self._update_depth_cloud()
        if self._latest_cloud is None or self._latest_cloud_time <= 0.0:
            return math.inf
        return max(0.0, time.monotonic() - self._latest_cloud_time)

    def get_rgb_image_age_s(self) -> float:
        if self._latest_rgb is None or self._latest_rgb_time <= 0.0:
            return math.inf
        return max(0.0, time.monotonic() - self._latest_rgb_time)

    def get_aligned_depth_image_age_s(self) -> float:
        if (
            self._latest_aligned_depth is None
            or self._latest_aligned_depth_time <= 0.0
        ):
            return math.inf
        return max(0.0, time.monotonic() - self._latest_aligned_depth_time)
