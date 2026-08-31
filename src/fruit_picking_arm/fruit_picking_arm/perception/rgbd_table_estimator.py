"""Lightweight, ROS-independent tabletop estimation from registered RGB-D.

The estimator intentionally models only the visible horizontal support
surface.  It does not pretend that an RGB-D camera can recover hidden table
edges.  A short temporal window rejects transient depth layers before the
surface is allowed into MoveIt's planning scene.
"""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TableSurface:
    center_x: float
    center_y: float
    top_z: float
    size_x: float
    size_y: float
    tilt_deg: float
    residual_std_m: float
    inlier_fraction: float


class RgbdTableEstimator:
    """Estimate and temporally validate the dominant world-horizontal plane."""

    def __init__(
        self,
        *,
        stride: int = 6,
        min_depth_m: float = 0.20,
        max_depth_m: float = 1.50,
        world_roi_min: tuple[float, float, float] | list[float] | None = None,
        world_roi_max: tuple[float, float, float] | list[float] | None = None,
        histogram_bin_m: float = 0.010,
        plane_tolerance_m: float = 0.015,
        max_tilt_deg: float = 15.0,
        max_residual_std_m: float = 0.008,
        min_inliers: int = 180,
        min_inlier_fraction: float = 0.12,
        min_extent_m: float = 0.15,
        history_size: int = 7,
        min_stable_frames: int = 5,
        max_temporal_std_m: float = 0.006,
    ) -> None:
        self.stride = max(2, int(stride))
        self.min_depth_m = float(min_depth_m)
        self.max_depth_m = float(max_depth_m)
        self.world_roi_min = self._optional_bound(world_roi_min)
        self.world_roi_max = self._optional_bound(world_roi_max)
        if (self.world_roi_min is None) != (self.world_roi_max is None):
            raise ValueError("world_roi_min and world_roi_max must be set together")
        if (
            self.world_roi_min is not None
            and np.any(self.world_roi_min >= self.world_roi_max)
        ):
            raise ValueError("world table ROI min must be smaller than max")
        self.histogram_bin_m = max(0.002, float(histogram_bin_m))
        self.plane_tolerance_m = max(0.003, float(plane_tolerance_m))
        self.max_tilt_deg = float(max_tilt_deg)
        self.max_residual_std_m = float(max_residual_std_m)
        self.min_inliers = max(30, int(min_inliers))
        self.min_inlier_fraction = float(min_inlier_fraction)
        self.min_extent_m = float(min_extent_m)
        self.min_stable_frames = max(2, int(min_stable_frames))
        self.max_temporal_std_m = float(max_temporal_std_m)
        self._history: deque[TableSurface] = deque(
            maxlen=max(self.min_stable_frames, int(history_size))
        )
        self.last_reason = "no_measurement"

    @staticmethod
    def _optional_bound(values):
        if values is None:
            return None
        array = np.asarray(values, dtype=np.float64)
        if array.shape != (3,) or not np.all(np.isfinite(array)):
            raise ValueError("world table ROI bound must contain three finite values")
        return array

    def reset(self) -> None:
        self._history.clear()
        self.last_reason = "reset"

    def update(
        self,
        depth_m: np.ndarray,
        camera_matrix: np.ndarray,
        camera_to_world,
    ) -> TableSurface | None:
        measurement = self._estimate(depth_m, camera_matrix, camera_to_world)
        if measurement is None:
            return None
        if self._history:
            previous_z = statistics.median(item.top_z for item in self._history)
            if abs(measurement.top_z - previous_z) > 0.05:
                self._history.clear()
                self.last_reason = "surface_layer_changed"
        self._history.append(measurement)
        if len(self._history) < self.min_stable_frames:
            self.last_reason = "samples=%d<%d" % (
                len(self._history), self.min_stable_frames
            )
            return None
        z_values = [item.top_z for item in self._history]
        if statistics.pstdev(z_values) > self.max_temporal_std_m:
            self.last_reason = "temporal_z_std=%.4fm>%.4fm" % (
                statistics.pstdev(z_values), self.max_temporal_std_m
            )
            return None
        self.last_reason = "accepted"
        return TableSurface(
            center_x=statistics.median(item.center_x for item in self._history),
            center_y=statistics.median(item.center_y for item in self._history),
            top_z=statistics.median(z_values),
            # Use the lower temporal quartile instead of a single optimistic
            # extent; isolated far-depth pixels cannot enlarge the obstacle.
            size_x=float(np.quantile([item.size_x for item in self._history], 0.25)),
            size_y=float(np.quantile([item.size_y for item in self._history], 0.25)),
            tilt_deg=statistics.median(item.tilt_deg for item in self._history),
            residual_std_m=statistics.median(
                item.residual_std_m for item in self._history
            ),
            inlier_fraction=statistics.median(
                item.inlier_fraction for item in self._history
            ),
        )

    def _estimate(self, depth_m, camera_matrix, camera_to_world):
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.ndim != 2:
            self.last_reason = "invalid_depth_shape"
            return None
        k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        fx, fy, cx, cy = float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])
        if min(fx, fy) <= 0.0:
            self.last_reason = "invalid_intrinsics"
            return None
        sampled = depth[:: self.stride, :: self.stride]
        rows, cols = sampled.shape
        u, v = np.meshgrid(
            np.arange(cols, dtype=np.float32) * self.stride,
            np.arange(rows, dtype=np.float32) * self.stride,
        )
        valid = (
            np.isfinite(sampled)
            & (sampled >= self.min_depth_m)
            & (sampled <= self.max_depth_m)
        )
        if int(np.count_nonzero(valid)) < self.min_inliers:
            self.last_reason = "too_few_depth_points"
            return None
        z_camera = sampled[valid]
        camera_points = np.column_stack((
            (u[valid] - cx) * z_camera / fx,
            (v[valid] - cy) * z_camera / fy,
            z_camera,
        )).astype(np.float32)
        world_points = camera_to_world(camera_points)
        if world_points is None:
            self.last_reason = "transform_unavailable"
            return None
        points = np.asarray(world_points, dtype=np.float64)
        finite = np.all(np.isfinite(points), axis=1)
        points = points[finite]
        if self.world_roi_min is not None:
            inside = np.all(
                (points >= self.world_roi_min) & (points <= self.world_roi_max),
                axis=1,
            )
            points = points[inside]
        if len(points) < self.min_inliers:
            self.last_reason = "too_few_world_roi_points"
            return None

        world_z = points[:, 2]
        lower, upper = np.quantile(world_z, [0.01, 0.99])
        if not math.isfinite(lower + upper):
            self.last_reason = "invalid_world_z"
            return None
        if upper - lower <= 1.0e-6:
            layer_z = float(np.median(world_z))
        else:
            bins = max(2, int(math.ceil((upper - lower) / self.histogram_bin_m)))
            counts, edges = np.histogram(world_z, bins=bins, range=(lower, upper))
            index = int(np.argmax(counts))
            layer_z = 0.5 * (edges[index] + edges[index + 1])
        inliers = np.abs(world_z - layer_z) <= self.plane_tolerance_m

        for _ in range(2):
            layer = points[inliers]
            if len(layer) < self.min_inliers:
                self.last_reason = "too_few_plane_inliers"
                return None
            design = np.column_stack((layer[:, 0], layer[:, 1], np.ones(len(layer))))
            a, b, c = np.linalg.lstsq(design, layer[:, 2], rcond=None)[0]
            residual = world_z - (a * points[:, 0] + b * points[:, 1] + c)
            inliers = np.abs(residual) <= self.plane_tolerance_m

        layer = points[inliers]
        residual = layer[:, 2] - (a * layer[:, 0] + b * layer[:, 1] + c)
        fraction = float(len(layer)) / float(len(points))
        tilt_deg = math.degrees(math.atan(math.hypot(float(a), float(b))))
        residual_std = float(np.std(residual))
        if tilt_deg > self.max_tilt_deg:
            self.last_reason = "tilt=%.1fdeg>%.1fdeg" % (tilt_deg, self.max_tilt_deg)
            return None
        if residual_std > self.max_residual_std_m:
            self.last_reason = "residual_std=%.4fm>%.4fm" % (
                residual_std, self.max_residual_std_m
            )
            return None
        if fraction < self.min_inlier_fraction:
            self.last_reason = "inlier_fraction=%.2f<%.2f" % (
                fraction, self.min_inlier_fraction
            )
            return None
        x_min, x_max = np.quantile(layer[:, 0], [0.02, 0.98])
        y_min, y_max = np.quantile(layer[:, 1], [0.02, 0.98])
        size_x, size_y = float(x_max - x_min), float(y_max - y_min)
        if min(size_x, size_y) < self.min_extent_m:
            self.last_reason = "visible_extent_too_small"
            return None
        center_x, center_y = float((x_min + x_max) * 0.5), float((y_min + y_max) * 0.5)
        top_z = float(a * center_x + b * center_y + c)
        return TableSurface(
            center_x, center_y, top_z, size_x, size_y,
            tilt_deg, residual_std, fraction,
        )
