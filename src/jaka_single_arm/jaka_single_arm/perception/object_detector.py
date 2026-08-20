#!/usr/bin/env python3
"""Object Detector — point cloud → object detection & shape estimation.

Pipeline:
  1. Voxel downsampling
  2. RANSAC plane segmentation → extract table plane
  3. Remove table inliers from cloud
  4. Euclidean clustering → individual objects
  5. Per-cluster: centroid (position) + bounding box (size) + sphere fit (radius)
  6. Publish detection visualization markers

Uses numpy + scipy cKDTree to avoid hard PCL Python dependency.
The resulting geometry also defines a tight RGB ROI for quality classification.

Reference:
  - mycobot_ros2 (AutomaticAddison) — PCL segmentation pipeline
  - ros2_moveit2_ur5e_grasp (Nackustb) — vision module design
  - pcl_ros / PCL library — RANSAC + Euclidean clustering algorithms
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import rclpy


def resolve_table_fallback_policy(
    camera_type: str,
    requested_center_z: bool,
    requested_z_fallback: bool,
) -> tuple[bool, bool]:
    """Allow fixed table geometry only for explicitly simulated cameras."""
    simulation_camera = str(camera_type).strip().lower() in {"mock", "gazebo"}
    return (
        bool(requested_center_z) and simulation_camera,
        bool(requested_z_fallback) and simulation_camera,
    )


@dataclass
class DetectedObject:
    """Result of object detection for a single object."""
    id: str = ""
    centroid: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Bounding box (axis-aligned)
    bbox_min: tuple[float, float, float] = (0.0, 0.0, 0.0)
    bbox_max: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Estimated shape
    shape: str = "sphere"  # sphere | box | cylinder | unknown
    radius: float = 0.0    # for sphere
    # Quality
    num_points: int = 0
    confidence: float = 0.0
    # Fruit type/quality result filled by ROI-only HealthFusion.
    health: str = "unknown"          # "Healthy" | "Unhealthy" | "unknown"
    health_confidence: float = 0.0
    fruit_type: str = "unknown"       # apple | banana | orange | unknown
    # Winning-vs-runner-up quality score margin.  The target tracker uses this
    # to reject ambiguous overlapping detections before a pick is requested.
    health_margin: float = 0.0
    class_id: int = -1               # 0=Healthy, 1=Unhealthy, -1=未知


class ObjectDetector:
    """Point cloud → object detection & pose estimation.

    Processes point clouds from any CameraInterface implementation
    and produces a list of detected objects with pose and shape info.

    Usage:
        detector = ObjectDetector(node, perception_cfg)
        objects = detector.process(cloud_msg)
        for obj in objects:
            print(f"Detected: {obj.id} at {obj.centroid}, r={obj.radius:.3f}")
    """

    def __init__(self, node: Node, config: dict):
        self._node = node
        self._logger = node.get_logger()
        self._config = config

        # Detection parameters
        self._voxel_size = config.get("voxel_leaf_size", 0.005)
        self._ransac_threshold = config.get("ransac_distance_threshold", 0.01)
        self._ransac_iterations = config.get("ransac_max_iterations", 100)
        self._cluster_tolerance = config.get("cluster_tolerance", 0.02)
        self._min_cluster_size = config.get("min_cluster_size", 100)
        self._max_cluster_size = config.get("max_cluster_size", 10000)
        self._min_confidence = config.get("min_confidence", 0.5)
        self._min_object_radius = float(config.get("min_object_radius", 0.015))
        self._max_object_radius = float(config.get("max_object_radius", 0.065))

        # Coordinate contract for downstream control. Real hardware should
        # use base_link; simulation keeps world for backwards compatibility.
        self._output_frame = config.get("output_frame", "world")
        self._drop_on_transform_failure = config.get(
            "drop_on_transform_failure", True
        )
        camera_type = str(config.get("camera_type", "unknown")).strip().lower()
        requested_center_z = bool(config.get("force_table_center_z", False))
        requested_z_fallback = bool(
            config.get("enable_table_z_fallback", False)
        )
        (
            self._force_table_center_z,
            self._enable_table_z_fallback,
        ) = resolve_table_fallback_policy(
            camera_type, requested_center_z, requested_z_fallback
        )
        simulation_camera = camera_type in {"mock", "gazebo"}
        if not simulation_camera and (requested_center_z or requested_z_fallback):
            self._logger.error(
                "[SAFETY] Fixed table-Z correction/fallback was requested for "
                f"camera_type='{camera_type}', but it is restricted to "
                "mock/gazebo. Real frames will be rejected when RANSAC fails."
            )

        # Table Z filter fallback
        self._table_z = config.get("table_top_z", 0.30)
        self._table_z_tol = config.get("table_z_tolerance", 0.02)

        roi_min = config.get("detection_roi_min", [])
        roi_max = config.get("detection_roi_max", [])
        self._roi_min = (
            np.asarray(roi_min, dtype=np.float32) if len(roi_min) == 3 else None
        )
        self._roi_max = (
            np.asarray(roi_max, dtype=np.float32) if len(roi_max) == 3 else None
        )

        # Publishers for visualization
        self._marker_pub = self._node.create_publisher(
            MarkerArray, "/perception/detected_objects", 10
        )
        # Debug: publish intermediate point clouds for RViz inspection
        self._debug_raw_pub = self._node.create_publisher(
            PointCloud2, "/perception/debug/raw_cloud", 10
        )
        self._debug_voxel_pub = self._node.create_publisher(
            PointCloud2, "/perception/debug/voxel_cloud", 10
        )
        self._debug_above_table_pub = self._node.create_publisher(
            PointCloud2, "/perception/debug/above_table_cloud", 10
        )
        self._debug_cluster_pub = self._node.create_publisher(
            PointCloud2, "/perception/debug/cluster_cloud", 10
        )

        # TF for transforming detected centroids to world frame
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        # Latest result
        self._latest_objects: list[DetectedObject] = []
        self._next_marker_id = 0

    @property
    def latest_objects(self) -> list[DetectedObject]:
        return self._latest_objects

    def transform_ready(self, source_frame: str) -> bool:
        """Return whether the camera frame can currently be transformed."""
        if not source_frame:
            return False
        if source_frame == self._output_frame:
            return True
        return self._tf_buffer.can_transform(
            self._output_frame,
            source_frame,
            rclpy.time.Time(),
        )

    def process(self, cloud_msg: PointCloud2) -> list[DetectedObject]:
        """Process a point cloud and return detected objects.

        Args:
            cloud_msg: PointCloud2 message from camera.

        Returns:
            List of DetectedObject with pose and shape estimates.
        """
        points = self._cloud_to_array(cloud_msg)
        if points is None or len(points) < self._min_cluster_size:
            self._logger.debug(f"Insufficient points: {len(points) if points is not None else 0}")
            return []

        # Convert the complete cloud to the downstream control frame before
        # table removal and clustering. This makes the workspace ROI explicit
        # and prevents robot links, camera stands, bins and backdrops from
        # becoming fruit candidates.
        marker_header = cloud_msg.header
        processing_frame = cloud_msg.header.frame_id
        if processing_frame != self._output_frame:
            transform = self._lookup_output_transform(
                processing_frame, cloud_msg.header.stamp
            )
            if transform is None:
                self._latest_objects = []
                return []
            points = self._transform_points(points, transform)
            marker_header = Header()
            marker_header.stamp = cloud_msg.header.stamp
            marker_header.frame_id = self._output_frame
            processing_frame = self._output_frame

        points = self._crop_to_detection_roi(points)
        if len(points) < self._min_cluster_size:
            self._logger.debug("No points inside detection ROI")
            return []

        # Debug: publish raw input cloud
        self._publish_debug_cloud(points, marker_header, self._debug_raw_pub)

        # Step 1: Voxel downsampling
        downsampled = self._voxel_filter(points)
        self._publish_debug_cloud(downsampled, marker_header, self._debug_voxel_pub)

        # Step 2: Remove table plane (RANSAC or Z-filter)
        above_table = self._remove_table(points, downsampled)
        self._publish_debug_cloud(above_table, marker_header, self._debug_above_table_pub)

        if len(above_table) < self._min_cluster_size:
            self._logger.debug("No points above table after plane removal")
            return []

        # Step 3: Euclidean clustering
        clusters = self._euclidean_cluster(above_table)

        # Debug: publish cluster points with rainbow colors
        self._publish_cluster_debug(above_table, clusters, marker_header)

        # Step 4: Per-cluster analysis
        objects = []
        for cluster_points in clusters:
            obj = self._analyze_cluster(cluster_points)
            if (
                obj is not None
                and obj.confidence >= self._min_confidence
                and self._min_object_radius <= obj.radius <= self._max_object_radius
            ):
                obj.id = f"object_{len(objects):02d}"
                objects.append(obj)

        # Transform centroids to the configured control frame. Never relabel
        # camera coordinates as base/world coordinates after a failed lookup.
        source_frame = processing_frame # 记录当前点云所在的坐标系
        if objects and source_frame != self._output_frame:
            objects, transform_ok = self._transform_centroids_to_output(
                objects, cloud_msg.header.frame_id, cloud_msg.header.stamp
            )
            if not transform_ok and self._drop_on_transform_failure:
                self._latest_objects = []
                return []
            # 更新可视化标记
            marker_header = Header()
            marker_header.stamp = cloud_msg.header.stamp
            marker_header.frame_id = self._output_frame if transform_ok else source_frame

        # Apply table-surface z-correction in WORLD frame.
        # Objects rest on the table, so true center.z = table_top_z + radius.
        # The raw detected z is biased upward (only top hemisphere visible from
        # above); this correction must happen AFTER TF transform to avoid
        # mixing camera-frame coords with world-frame z.
        # 修正高度
        """
        把物体质心的Z轴高度,从相机看到的表面高度
        修正为物体真实的物体中心高度
        """
        if self._force_table_center_z:
            for obj in objects:
                cx, cy, _cz = obj.centroid
                obj.centroid = (cx, cy, float(self._table_z + obj.radius))

        self._latest_objects = objects

        self._publish_markers(objects, marker_header)

        self._logger.info(
            f"Detection pipeline: {len(points)} raw → {len(downsampled)} voxel "
            f"→ {len(above_table)} above-table → {len(clusters)} clusters "
            f"→ {len(objects)} objects"
        )
        return objects

    # ── Point cloud decoding ─────────────────────────────────

    def _cloud_to_array(self, msg: PointCloud2) -> np.ndarray | None:
        """Decode PointCloud2 to Nx3 numpy array."""
        if msg is None or msg.width == 0:
            return None

        # Find XYZ field offsets
        offsets = {}
        for field in msg.fields:
            if field.name in ("x", "y", "z"):
                offsets[field.name] = field.offset

        if len(offsets) < 3:
            self._logger.warning("PointCloud2 missing x/y/z fields")
            return None

        point_step = int(msg.point_step)
        if point_step <= 0:
            self._logger.warning("PointCloud2 has an invalid point_step")
            return None

        """
        涉及内存对齐
        如果数据地址是8的倍数或者16的倍数,读取速度会翻倍,为了读取更快,相机驱动
        可能会在每一行的末尾多塞几个没用的字节作为填充
        """
        row_step = int(msg.row_step or (point_step * msg.width))  # 如果相机驱动告诉了我行距是多少,我就听他的,如果没有告诉我,我就自己算一个
        height = max(int(msg.height), 1)
        width = int(msg.width)
        # 告诉相机是大端还是小端
        # 小端 从左到右 从低字节到高字节
        # 大端 从右到左 从高字节到低字节
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")

        # Strided views handle organized clouds and row padding without a
        # Python loop over every camera pixel.
        try:
            coordinates = []
            for name in ("x", "y", "z"): # 这是一个循环 依次从元组里面取出"x" "y" "z"
                coordinates.append(np.ndarray(
                    shape=(height, width),
                    dtype=dtype,
                    buffer=msg.data,
                    offset=int(offsets[name]),
                    strides=(row_step, point_step),
                ).reshape(-1))
            points = np.column_stack(coordinates).astype(np.float32, copy=False)
        except (TypeError, ValueError, BufferError) as exc:
            self._logger.warning(f"Unable to decode PointCloud2: {exc}")
            return None

        return points[np.isfinite(points).all(axis=1)]

    """
    只保留一个三维长方体ROI内部的点,把范围内的点全部丢弃
    """
    def _crop_to_detection_roi(self, points: np.ndarray) -> np.ndarray:
        """Keep only points inside the configured control-frame workspace."""
        if self._roi_min is None or self._roi_max is None:
            return points

        """
        mask是一个一维布尔数组,长度等于点的总数
        里面每一个值都是True或False
        """
        mask = np.logical_and( # 把两个条件与起来
            np.all(points >= self._roi_min, axis=1),
            np.all(points <= self._roi_max, axis=1),
        )
        return points[mask] # 布尔索引,掩码筛选

    # ── Voxel downsampling ───────────────────────────────────
    def _voxel_filter(self, points: np.ndarray) -> np.ndarray:

        if self._voxel_size <= 0:
            return points

        finite_points = points[np.isfinite(points).all(axis=1)]
        if len(finite_points) == 0:
            return finite_points

        # 计算每个点所在的网格坐标
        voxel_indices = np.floor(
            finite_points / self._voxel_size
        ).astype(np.int64)
        _, inverse = np.unique(
            voxel_indices, axis=0, return_inverse=True
        )

        voxel_count = int(inverse.max()) + 1
        sums = np.zeros((voxel_count, 3), dtype=np.float64)
        np.add.at(sums, inverse, finite_points)
        # 计算平均质心
        counts = np.bincount(inverse, minlength=voxel_count)
        centroids = sums / counts[:, None]
        return centroids.astype(finite_points.dtype, copy=False)


    # ── Table removal ────────────────────────────────────────

    def _remove_table(self, original: np.ndarray, downsampled: np.ndarray) -> np.ndarray:
        """Remove table plane points.

        Strategy: Try RANSAC first, fall back to simple Z-filter.
        先尝试RANSAC平面拟合,若失败则回退到简单的Z轴阈值过滤
        """
        # RANSAC plane fitting
        plane_normal, plane_d = self._ransac_plane(downsampled)
        if plane_normal is not None:
            # Remove points near the detected plane
            distances = np.abs(
                original[:, 0] * plane_normal[0] +
                original[:, 1] * plane_normal[1] +
                original[:, 2] * plane_normal[2] + plane_d
            )
            above = original[distances > self._ransac_threshold * 2]
            self._logger.debug(
                f"RANSAC plane: normal=({plane_normal[0]:.2f},{plane_normal[1]:.2f},"
                f"{plane_normal[2]:.2f}), kept {len(above)}/{len(original)} points"
            )
            return above

        # A Z threshold is meaningful only when the input point cloud itself
        # is already expressed in the configured table/world frame. RealSense
        # optical Z points forward, so real launches disable this fallback.
        if not self._enable_table_z_fallback:
            self._logger.warning("RANSAC table fit failed; rejecting frame (Z fallback disabled)")
            return np.empty((0, 3), dtype=original.dtype)

        # Simulation fallback: simple Z-filter
        above = original[original[:, 2] > self._table_z + self._table_z_tol]
        self._logger.debug(f"Z-filter fallback: kept {len(above)}/{len(original)} points")
        return above

    """
    RANSAC(随机采样一致性)
    随机抽3个点算出一个平面,然后看看有多少个其他点也在这个平面上(内点)
    重复多次,选出内点数最多的那个平面
    """
    def _ransac_plane(self, points: np.ndarray) -> tuple:
        """RANSAC plane fitting. Returns (normal, d) or (None, None)."""
        if len(points) < 3:
            return None, None

        best_inliers = 0       # 内点数量最高的记录
        best_normal = None     # 该最优平面的法向量
        best_d = 0.0           # 该最优平面的常数项

        # 计算总共要进行多少次随机抽样点猜平面
        n_iter = min(self._ransac_iterations, len(points) * 2)
        for _ in range(n_iter):
            # Random 3 points
            idxs = np.random.choice(len(points), 3, replace=False)
            p1, p2, p3 = points[idxs[0]], points[idxs[1]], points[idxs[2]]

            # Plane normal = (p2-p1) × (p3-p1)
            normal = np.cross(p2 - p1, p3 - p1)
            norm = np.linalg.norm(normal)
            if norm < 1e-10:
                continue
            normal = normal / norm

            # d = -normal · p0
            d = -np.dot(normal, p1)

            # Count inliers
            distances = np.abs(np.dot(points, normal) + d)
            inliers = np.sum(distances < self._ransac_threshold)

            if inliers > best_inliers:
                best_inliers = inliers
                best_normal = normal
                best_d = d

        if best_inliers < len(points) * 0.1:
            return None, None

        return best_normal, best_d

    # ── Euclidean clustering ─────────────────────────────────

    def _euclidean_cluster(self, points: np.ndarray) -> list[np.ndarray]:
        """Cluster points using KD-tree based Euclidean clustering."""
        """
        欧几里得聚类
        分割出独立物体
        如果两个点之间的欧几里得距离小于设定的阈值,就把它们归为同一类
        然后从这个类里的所有点出发,继续寻找新的邻居,直到再也找不到新点为止
        """

        """
        KD-Tree(k维树) 是一种空间划分数据结构
        能把寻找某个点周围半径内的所有点这个操作加速到O(log N)
        极大地减少了计算量

        cKDTree是Scipy提供地C语言实现
        """
        try:
            from scipy.spatial import cKDTree
        except ImportError:
            self._logger.error(
                "scipy is unavailable; rejecting frame because safe Euclidean "
                "clustering cannot be performed"
            )
            return []

        tree = cKDTree(points)                          # 把点云建成了KD-Tree
        processed = np.zeros(len(points), dtype=bool)   # 每个点是否已被归入某个簇 初始化: 生成一个全false的布尔数组
        clusters = []                                   # 存放所有找到地簇

        for i in range(len(points)):
            if processed[i]: # 如果这个点已经被之前的簇签到过了,continue跳过
                continue

            # BFS from seed point
            cluster_mask = np.zeros(len(points), dtype=bool)
            seeds = [i]
            cluster_mask[i] = True

            while seeds:
                idx = seeds.pop()
                # Radius search
                neighbors = tree.query_ball_point(points[idx], self._cluster_tolerance)
                for nb in neighbors:
                    if not cluster_mask[nb]:
                        cluster_mask[nb] = True
                        seeds.append(nb)
                        if len(seeds) > self._max_cluster_size:
                            break

            cluster_size = np.sum(cluster_mask)
            if self._min_cluster_size <= cluster_size <= self._max_cluster_size:
                clusters.append(points[cluster_mask])
            processed |= cluster_mask # 把当前簇覆盖到的所有点,再全局签到表里全部打上已处理的标记

        return clusters


    # ── Cluster analysis ─────────────────────────────────────
    """
    把上一轮聚类找到的点云
    进行几何测量和形状分析,最终生成一个结构化的DetectedObject 对象
    """
    def _analyze_cluster(self, points: np.ndarray) -> Optional[DetectedObject]:
        """Analyze a point cluster: centroid, bounding box, sphere fit."""
        if len(points) < 10:
            return None

        obj = DetectedObject()

        # Raw centroid from point cloud — may be in camera frame.
        # Z-correction (table_top_z + radius) is applied AFTER TF transform
        # to world frame, to avoid mixing camera-frame and world-frame
        # coordinates before rotation (which would pollute all axes).
        centroid = np.mean(points, axis=0)

        # Bounding box
        bbox_min = np.min(points, axis=0)
        bbox_max = np.max(points, axis=0)
        obj.bbox_min = (float(bbox_min[0]), float(bbox_min[1]), float(bbox_min[2]))
        obj.bbox_max = (float(bbox_max[0]), float(bbox_max[1]), float(bbox_max[2]))

        # Sphere fit: use xy-extent for radius (viewed from above).
        # Bounding-box diagonal overestimates radius for hemispherical visible
        # point clouds because the z-extent is only ~r (not 2r).
        extents = bbox_max - bbox_min
        xy_radius = max(extents[0], extents[1]) / 2.0
        obj.radius = float(xy_radius)

        # Store raw centroid (uncorrected). Z-correction applied in process()
        # after TF transform to world frame.
        obj.centroid = (float(centroid[0]), float(centroid[1]), float(centroid[2]))

        # Shape classification: check if close to sphere
        extent_std = np.std(extents)
        extent_mean = np.mean(extents)
        if extent_mean > 0 and extent_std / extent_mean < 0.3:
            obj.shape = "sphere"
        elif extents[2] < extents[0] * 0.3 and extents[2] < extents[1] * 0.3:
            obj.shape = "cylinder"  # flat like a disk
        else:
            obj.shape = "unknown"

        obj.num_points = len(points)
        # Confidence based on point count relative to max
        obj.confidence = min(1.0, len(points) / max(self._min_cluster_size * 5, 1))

        return obj

    # ── Coordinate frame utilities ────────────────────────────

    def _lookup_output_transform(self, source_frame: str, stamp):
        """Look up source -> output, accepting latest for static extrinsics."""
        if not source_frame:
            self._logger.warning(
                "Point cloud frame_id is empty; target coordinates are unsafe"
            )
            return None
        try:
            when = rclpy.time.Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)
            return self._tf_buffer.lookup_transform(
                self._output_frame, source_frame, when
            )
        except Exception as stamped_error:
            try:
                return self._tf_buffer.lookup_transform(
                    self._output_frame, source_frame, rclpy.time.Time()
                )
            except Exception as latest_error:
                self._logger.warning(
                    f"TF lookup '{self._output_frame}' -> '{source_frame}' "
                    f"failed (stamp: {stamped_error}; latest: {latest_error})."
                )
                return None

    @staticmethod
    def _transform_points(points: np.ndarray, transform) -> np.ndarray:
        """Apply a geometry_msgs TransformStamped to an Nx3 array."""
        t = transform.transform.translation
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
        ], dtype=np.float32)
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        return points @ rotation.T + translation

    """
    从ROS2的TF树上查找从source_frame(源坐标系)到self._output_frame(目标坐标系)
    的坐标变换关机,并返回一个TransformStamped消息
    """
    def _transform_centroids_to_output(
        self, objects: list[DetectedObject],
        source_frame: str, stamp
    ) -> tuple[list[DetectedObject], bool]:
        """Transform centroids from source_frame to configured output frame."""
        transform = self._lookup_output_transform(source_frame, stamp)
        if transform is None:
            return objects, False
        centroids = np.asarray(
            [obj.centroid for obj in objects], dtype=np.float32
        )
        transformed = self._transform_points(centroids, transform)
        for obj, point in zip(objects, transformed):
            obj.centroid = tuple(float(value) for value in point)

        self._logger.debug(
            f"Transformed {len(objects)} centroids: {source_frame} → {self._output_frame}"
        )
        return objects, True

    # ── Visualization ────────────────────────────────────────
    """
    Numpy数组N*3转换为ROS的PointCloud2消息
    以便发布供Rviz显示或其他节点使用
    """
    def _array_to_cloud(self, points: np.ndarray, header: Header) -> PointCloud2:
        """Convert Nx3 numpy array to PointCloud2 message."""
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        data = bytearray()
        for x, y, z in points:
            data.extend(struct.pack("<fff", float(x), float(y), float(z)))
        msg.data = bytes(data)
        return msg

    def _publish_debug_cloud(self, points: np.ndarray, header: Header,
                             publisher) -> None:
        """Publish a numpy point cloud as PointCloud2 for RViz debugging."""
        """把一个Numpy点云数组转换成ROS的Pointcloud2消息,并通过指定的发布器发出去,方便在rviz里可视化中间处理结果"""
        if publisher is None or len(points) == 0:
            return
        msg = self._array_to_cloud(points, header)
        publisher.publish(msg)

    def _publish_cluster_debug(self, all_points: np.ndarray,
                               clusters: list[np.ndarray],
                               header: Header) -> None:
        """Publish clustered points with per-cluster colors (encoded as intensity)."""
        if self._debug_cluster_pub is None or len(clusters) == 0:
            return
        # Assign each cluster a unique "z-offset" so they appear at different
        # heights in RViz (makes overlapping clusters easy to distinguish).
        # Also publish a combined cloud where each cluster is shifted in z.
        # Publish separate clouds for each cluster at slightly different z-offsets
        for ci, cluster in enumerate(clusters):
            shifted = cluster.copy()
            z_offset = ci * 0.005  # 5mm per cluster for visibility
            shifted[:, 2] += z_offset

            h = Header()
            h.stamp = header.stamp
            h.frame_id = header.frame_id
            msg = self._array_to_cloud(shifted, h)
            # Use a separate topic per cluster
            pub = getattr(self, f'_cluster_pub_{ci}', None)
            if pub is None:
                pub = self._node.create_publisher(
                    PointCloud2, f"/perception/debug/cluster_{ci}", 10
                )
                setattr(self, f'_cluster_pub_{ci}', pub)
            pub.publish(msg)

        # Also publish combined cloud on main debug topic
        combined = []
        for ci, cluster in enumerate(clusters):
            shifted = cluster.copy()
            shifted[:, 2] += ci * 0.005
            combined.append(shifted)
        if combined:
            stacked = np.vstack(combined)
            self._publish_debug_cloud(stacked, header, self._debug_cluster_pub)

    def _publish_markers(self, objects: list[DetectedObject], header: Header) -> None:
        """Publish RViz markers for detected objects."""
        ma = MarkerArray()

        for i, obj in enumerate(objects):
            # Sphere marker at centroid
            m = Marker()
            m.header = header
            m.header.frame_id = header.frame_id
            m.ns = "detected_objects"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=obj.centroid[0], y=obj.centroid[1], z=obj.centroid[2])
            m.pose.orientation.w = 1.0
            r = max(obj.radius, 0.01)
            m.scale = Vector3(x=r * 2, y=r * 2, z=r * 2)
            # Color by confidence (green=high, red=low)
            m.color.r = 1.0 - obj.confidence
            m.color.g = obj.confidence
            m.color.b = 0.2
            m.color.a = 0.7

            # Text marker with info
            text = Marker()
            text.header = header
            text.header.frame_id = header.frame_id
            text.ns = "detected_labels"
            text.id = i + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(
                x=obj.centroid[0],
                y=obj.centroid[1],
                z=obj.centroid[2] + obj.radius + 0.03,
            )
            text.scale.z = 0.025
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.9
            text.text = f"{obj.shape} r={obj.radius:.3f}"

            ma.markers.append(m)
            ma.markers.append(text)

        self._marker_pub.publish(ma)
