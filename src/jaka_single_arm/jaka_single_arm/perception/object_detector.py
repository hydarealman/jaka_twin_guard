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
Ready for future upgrade to YOLO + PCL for complex fruit shapes.

Reference:
  - mycobot_ros2 (AutomaticAddison) — PCL segmentation pipeline
  - ros2_moveit2_ur5e_grasp (Nackustb) — vision module design
  - pcl_ros / PCL library — RANSAC + Euclidean clustering algorithms
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, PointStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import rclpy


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

        # Table Z filter fallback
        self._table_z = config.get("table_top_z", 0.30)
        self._table_z_tol = config.get("table_z_tolerance", 0.02)

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

        # Debug: publish raw input cloud
        self._publish_debug_cloud(points, cloud_msg.header, self._debug_raw_pub)

        # Step 1: Voxel downsampling
        downsampled = self._voxel_filter(points)
        self._publish_debug_cloud(downsampled, cloud_msg.header, self._debug_voxel_pub)

        # Step 2: Remove table plane (RANSAC or Z-filter)
        above_table = self._remove_table(points, downsampled)
        self._publish_debug_cloud(above_table, cloud_msg.header, self._debug_above_table_pub)

        if len(above_table) < self._min_cluster_size:
            self._logger.debug("No points above table after plane removal")
            return []

        # Step 3: Euclidean clustering
        clusters = self._euclidean_cluster(above_table)

        # Debug: publish cluster points with rainbow colors
        self._publish_cluster_debug(above_table, clusters, cloud_msg.header)

        # Step 4: Per-cluster analysis
        objects = []
        for cluster_points in clusters:
            obj = self._analyze_cluster(cluster_points)
            if obj is not None and obj.confidence >= self._min_confidence:
                obj.id = f"object_{len(objects):02d}"
                objects.append(obj)

        # Transform centroids to world frame if the cloud is in a camera frame.
        # Gazebo depth camera publishes in camera_depth_frame; grasp skills need world.
        marker_header = cloud_msg.header
        if objects and cloud_msg.header.frame_id not in ("", "world"):
            objects = self._transform_centroids_to_world(
                objects, cloud_msg.header.frame_id, cloud_msg.header.stamp
            )
            # Update marker header to reflect that centroids are now in world frame
            marker_header = Header()
            marker_header.stamp = cloud_msg.header.stamp
            marker_header.frame_id = "world"

        # Apply table-surface z-correction in WORLD frame.
        # Objects rest on the table, so true center.z = table_top_z + radius.
        # The raw detected z is biased upward (only top hemisphere visible from
        # above); this correction must happen AFTER TF transform to avoid
        # mixing camera-frame coords with world-frame z.
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

        points = []
        step = msg.point_step
        data = msg.data
        for i in range(msg.width):
            base = i * step
            try:
                x = struct.unpack_from("<f", data, base + offsets["x"])[0]
                y = struct.unpack_from("<f", data, base + offsets["y"])[0]
                z = struct.unpack_from("<f", data, base + offsets["z"])[0]
                points.append([x, y, z])
            except (struct.error, IndexError):
                continue

        return np.array(points, dtype=np.float32)

    # ── Voxel downsampling ───────────────────────────────────

    def _voxel_filter(self, points: np.ndarray) -> np.ndarray:
        """Downsample point cloud using voxel grid."""
        if self._voxel_size <= 0:
            return points

        voxel_size = self._voxel_size
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)

        # Unique voxels via dictionary
        voxel_dict = {}
        for i, vi in enumerate(voxel_indices):
            key = (vi[0], vi[1], vi[2])
            if key not in voxel_dict:
                voxel_dict[key] = points[i]

        return np.array(list(voxel_dict.values()), dtype=np.float32)

    # ── Table removal ────────────────────────────────────────

    def _remove_table(self, original: np.ndarray, downsampled: np.ndarray) -> np.ndarray:
        """Remove table plane points.

        Strategy: Try RANSAC first, fall back to simple Z-filter.
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

        # Fallback: simple Z-filter
        above = original[original[:, 2] > self._table_z + self._table_z_tol]
        self._logger.debug(f"Z-filter fallback: kept {len(above)}/{len(original)} points")
        return above

    def _ransac_plane(self, points: np.ndarray) -> tuple:
        """RANSAC plane fitting. Returns (normal, d) or (None, None)."""
        if len(points) < 3:
            return None, None

        best_inliers = 0
        best_normal = None
        best_d = 0.0

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
        try:
            from scipy.spatial import cKDTree
        except ImportError:
            self._logger.warning("scipy not available, returning single cluster")
            return [points]

        tree = cKDTree(points)
        processed = np.zeros(len(points), dtype=bool)
        clusters = []

        for i in range(len(points)):
            if processed[i]:
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
            processed |= cluster_mask

        return clusters

    # ── Cluster analysis ─────────────────────────────────────

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

    def _transform_centroids_to_world(
        self, objects: list[DetectedObject],
        source_frame: str, stamp
    ) -> list[DetectedObject]:
        """Transform detected object centroids from source_frame to world frame."""
        try:
            when = rclpy.time.Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)
            # Allow a short wait for the TF to become available
            transform = self._tf_buffer.lookup_transform(
                "world", source_frame, when,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self._logger.warning(
                f"TF lookup 'world'←'{source_frame}' failed ({e}). "
                f"Centroids will remain in {source_frame}."
            )
            return objects

        t = transform.transform.translation
        q = transform.transform.rotation
        import math
        # Quaternion to rotation matrix (row-major)
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
        ], dtype=np.float64)

        for obj in objects:
            cx, cy, cz = obj.centroid
            # Rotate then translate
            p = R @ np.array([cx, cy, cz])
            wx = p[0] + t.x
            wy = p[1] + t.y
            wz = p[2] + t.z
            obj.centroid = (float(wx), float(wy), float(wz))

        self._logger.debug(
            f"Transformed {len(objects)} centroids: {source_frame} → world"
        )
        return objects

    # ── Visualization ────────────────────────────────────────

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
        colors = [
            [1.0, 0.2, 0.2],  # red
            [0.2, 1.0, 0.2],  # green
            [0.2, 0.4, 1.0],  # blue
            [1.0, 0.8, 0.1],  # yellow
            [1.0, 0.3, 1.0],  # magenta
            [0.2, 1.0, 1.0],  # cyan
        ]

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
