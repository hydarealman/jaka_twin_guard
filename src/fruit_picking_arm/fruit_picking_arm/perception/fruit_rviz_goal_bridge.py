#!/usr/bin/env python3
"""Human-gated RViz pick/sort stages for stable real-fruit detections.

This node never sends an arm trajectory.  It publishes exactly one inspected
MoveIt goal at a time and observes the real arm-controller status.  Only after
the operator clicks RViz Execute and the measured joints reach that goal does
it expose the next stage.  Binary gripper close/open commands are issued after
the corresponding human-approved grasp/release arm stages.
"""

from __future__ import annotations

import math
import re
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray

from fruit_picking_arm.skills.top_down_pose import (
    pregrasp_tcp_z,
    symmetric_yaw_candidates,
    top_down_quaternion,
)
from fruit_picking_arm.scene.bin_geometry import (
    bin_drop_candidates,
    bin_wall_boxes,
    normalized_bin_config,
)


def target_geometry_changed(
    previous: dict | None, current: dict, threshold_m: float
) -> bool:
    """Apply distance hysteresis without quantization-boundary chatter."""
    if previous is None or previous["source_id"] != current["source_id"]:
        return True
    displacement = math.sqrt(
        (current["x"] - previous["x"]) ** 2
        + (current["y"] - previous["y"]) ** 2
        + (current["z"] - previous["z"]) ** 2
    )
    return (
        displacement >= threshold_m
        or abs(current["radius"] - previous["radius"]) >= threshold_m
    )


def select_task_target(
    eligible: list[dict], preferred_source_id: str | None = None
) -> dict:
    """Keep one stable target until it disappears, then choose deterministically."""
    if preferred_source_id:
        for item in eligible:
            if item["source_id"] == preferred_source_id:
                return item
    return max(
        eligible,
        key=lambda item: (
            item["confidence"],
            -math.hypot(item["x"], item["y"]),
            item["source_id"],
        ),
    )


class FruitRvizGoalBridge(Node):
    """Publish the selected perceived fruit as a collision-aware RViz goal."""

    def __init__(self) -> None:
        super().__init__("fruit_rviz_goal_bridge")
        self.declare_parameter(
            "target_topic", "/perception/fruit_targets_world"
        )
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("ik_link", "gripper_tcp")
        self.declare_parameter("top_down_yaw", 3.141592654)
        self.declare_parameter("finger_tip_beyond_tcp", 0.037)
        self.declare_parameter("clearance_above_fruit", 0.050)
        self.declare_parameter("lift_height", 0.150)
        self.declare_parameter("place_approach_height", 0.120)
        self.declare_parameter("place_release_clearance", 0.015)
        self.declare_parameter("place_wall_clearance", 0.010)
        self.declare_parameter("unreachable_target_exclusion_radius", 0.060)
        self.declare_parameter("arm_goal_tolerance_rad", 0.040)
        self.declare_parameter("arm_goal_feedback_timeout_s", 2.0)
        self.declare_parameter("gripper_open_position", 0.056)
        self.declare_parameter("gripper_closed_position", 0.0)
        self.declare_parameter("gripper_max_effort", 50.0)
        self.declare_parameter("max_detection_age_s", 0.50)
        self.declare_parameter("max_future_detection_s", 0.05)
        self.declare_parameter("use_source_age_gate", True)
        self.declare_parameter("max_joint_state_age_s", 0.50)
        self.declare_parameter("target_position_change_m", 0.005)
        self.declare_parameter("goal_republish_period_s", 1.0)
        self.declare_parameter("ik_timeout_s", 0.50)
        self.declare_parameter("require_perceived_table", False)
        self.declare_parameter(
            "table_surface_topic", "/perception/table_surface"
        )
        self.declare_parameter("max_table_age_s", 0.75)
        self.declare_parameter("fruit_below_table_tolerance_m", 0.015)
        self.declare_parameter("fruit_max_height_above_table_m", 0.18)
        self.declare_parameter("scene_config_file", "scene_params_real.yaml")
        # The production CAD base passes through the camera-visible tabletop.
        # Carve only its fixed XY footprint out of the perceived collision box;
        # moving links remain collision-checked against the rest of the table.
        self.declare_parameter("table_base_cutout_enabled", True)
        # 400x400 mm base footprint plus a conservative 20 mm margin after
        # rebasing base_link to the centre of the physical bottom face.
        self.declare_parameter("table_base_cutout_min_x", -0.220)
        self.declare_parameter("table_base_cutout_max_x", 0.220)
        self.declare_parameter("table_base_cutout_min_y", -0.220)
        self.declare_parameter("table_base_cutout_max_y", 0.220)

        gp = self.get_parameter
        self._target_topic = str(gp("target_topic").value)
        self._world_frame = str(gp("world_frame").value)
        self._base_frame = str(gp("base_frame").value)
        self._planning_group = str(gp("planning_group").value)
        self._ik_link = str(gp("ik_link").value)
        self._preferred_top_down_yaw = float(gp("top_down_yaw").value)
        self._finger_tip_beyond_tcp = float(
            gp("finger_tip_beyond_tcp").value
        )
        self._clearance_above_fruit = float(
            gp("clearance_above_fruit").value
        )
        self._lift_height = float(gp("lift_height").value)
        self._place_approach_height = float(
            gp("place_approach_height").value
        )
        self._place_release_clearance = float(
            gp("place_release_clearance").value
        )
        self._place_wall_clearance = float(
            gp("place_wall_clearance").value
        )
        self._unreachable_target_exclusion_radius = max(
            0.01, float(gp("unreachable_target_exclusion_radius").value)
        )
        self._arm_goal_tolerance = float(gp("arm_goal_tolerance_rad").value)
        self._arm_goal_feedback_timeout = float(
            gp("arm_goal_feedback_timeout_s").value
        )
        self._gripper_open_position = float(gp("gripper_open_position").value)
        self._gripper_closed_position = float(
            gp("gripper_closed_position").value
        )
        self._gripper_max_effort = float(gp("gripper_max_effort").value)
        self._max_detection_age = float(gp("max_detection_age_s").value)
        self._max_future_detection = float(gp("max_future_detection_s").value)
        self._use_source_age_gate = bool(gp("use_source_age_gate").value)
        self._max_joint_state_age = float(gp("max_joint_state_age_s").value)
        self._target_change = float(gp("target_position_change_m").value)
        self._goal_republish_period = float(
            gp("goal_republish_period_s").value
        )
        self._ik_timeout = float(gp("ik_timeout_s").value)
        self._require_perceived_table = bool(gp("require_perceived_table").value)
        self._max_table_age = float(gp("max_table_age_s").value)
        self._fruit_below_table_tolerance = float(
            gp("fruit_below_table_tolerance_m").value
        )
        self._fruit_max_height_above_table = float(
            gp("fruit_max_height_above_table_m").value
        )
        self._table_base_cutout_enabled = bool(
            gp("table_base_cutout_enabled").value
        )
        self._table_base_cutout = (
            float(gp("table_base_cutout_min_x").value),
            float(gp("table_base_cutout_max_x").value),
            float(gp("table_base_cutout_min_y").value),
            float(gp("table_base_cutout_max_y").value),
        )

        self._scene_cfg = self._load_scene_config(
            str(gp("scene_config_file").value)
        )
        self._apply_scene = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._compute_ik = self.create_client(GetPositionIK, "/compute_ik")
        self._gripper_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self._vision_reset_client = self.create_client(
            Trigger, "/fruit_picking/reset_vision"
        )
        goal_qos = QoSProfile(depth=1)
        goal_qos.reliability = ReliabilityPolicy.RELIABLE
        goal_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._goal_pub = self.create_publisher(
            RobotState, "/rviz/moveit/update_custom_goal_state", goal_qos
        )
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(
            MarkerArray, "/perception/fruit_plan_markers", marker_qos
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Detection3DArray, self._target_topic, self._on_targets, 10
        )
        self.create_subscription(
            Marker,
            str(gp("table_surface_topic").value),
            self._on_table_surface,
            marker_qos,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
        self.create_subscription(
            GoalStatusArray,
            "/arm_controller/follow_joint_trajectory/_action/status",
            self._on_arm_status,
            20,
        )
        self.create_timer(0.1, self._housekeeping)

        self._joint_state: JointState | None = None
        self._joint_state_monotonic = 0.0
        self._detections: list[dict] = []
        self._selected: dict | None = None
        self._last_goal_target: dict | None = None
        self._accepted_goal_state: RobotState | None = None
        self._accepted_goal_target: dict | None = None
        self._accepted_goal_pose: PoseStamped | None = None
        self._last_goal_publish_monotonic = 0.0
        self._collision_ids: set[str] = set()
        self._table_surface: dict | None = None
        self._table_received_monotonic = 0.0
        self._last_table_geometry: tuple[float, ...] | None = None
        self._last_target_stamp_s: float | None = None
        self._last_ik_failure_log = 0.0
        self._goal_yaw: float | None = None
        self._ik_candidates: list[dict] = []
        self._request_generation = 0
        self._scene_request_in_flight = False
        # Register the fixed table as soon as MoveIt's scene service is ready.
        # Fruit geometry remains perception-derived and is added later.
        self._scene_update_pending = not self._require_perceived_table
        self._queued_generation = 0
        self._stage = "pregrasp"
        self._task_active = False
        self._place_xy: tuple[float, float] | None = None
        self._place_top_z: float | None = None
        self._completed_source_ids: set[str] = set()
        self._unreachable_positions: list[tuple[float, float, float]] = []
        self._last_queue_signature: tuple[str, ...] | None = None
        self._arm_status_ids: set[bytes] = set()
        self._stage_status_baseline: set[bytes] = set()
        self._bound_arm_goal_id: bytes | None = None
        self._pending_arm_success_at: float | None = None
        self._gripper_operation: str | None = None
        self._attached = False
        self._released = False
        self._vision_reset_pending = False
        self._vision_reset_future = None
        self._vision_reset_completed_id = ""
        self._vision_reset_retry_at = 0.0

        self.get_logger().info(
            "Fruit RViz staged task ready: pregrasp -> open -> grasp -> close "
            "-> lift -> bin hover -> release -> retract -> home. Arm motion "
            "remains RViz Plan/Execute only."
        )

    def _on_table_surface(self, marker: Marker) -> None:
        if marker.header.frame_id != self._world_frame:
            self.get_logger().error(
                "Rejecting perceived table in '%s'; expected '%s'"
                % (marker.header.frame_id, self._world_frame)
            )
            return
        values = (
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
            marker.scale.x,
            marker.scale.y,
            marker.scale.z,
        )
        if (
            not all(math.isfinite(float(value)) for value in values)
            or min(float(marker.scale.x), float(marker.scale.y)) < 0.10
            or float(marker.scale.z) <= 0.0
        ):
            self.get_logger().warning("Rejecting invalid perceived table geometry")
            return
        surface = {
            "center_x": float(marker.pose.position.x),
            "center_y": float(marker.pose.position.y),
            "center_z": float(marker.pose.position.z),
            "size_x": float(marker.scale.x),
            "size_y": float(marker.scale.y),
            "size_z": float(marker.scale.z),
        }
        self._table_surface = surface
        self._table_received_monotonic = time.monotonic()
        previous = self._last_table_geometry
        if previous is not None and all(
            abs(float(current) - old) <= tolerance
            for current, old, tolerance in zip(
                values,
                previous,
                (0.030, 0.030, 0.005, 0.030, 0.030, 0.005),
            )
        ):
            return
        self._last_table_geometry = tuple(float(value) for value in values)
        if self._task_active:
            # Freeze collision geometry for the human-approved task.  A new
            # table estimate is consumed after the current retract completes.
            return
        self._goal_yaw = None
        self._ik_candidates = []
        self._accepted_goal_state = None
        self._accepted_goal_target = None
        self._request_generation += 1
        self._queued_generation = self._request_generation
        self._scene_update_pending = True
        self._request_scene_update()

    @staticmethod
    def _load_scene_config(filename: str) -> dict:
        path = (
            Path(get_package_share_directory("fruit_picking_arm"))
            / "config"
            / filename
        )
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}

    def _on_joints(self, msg: JointState) -> None:
        self._joint_state = msg
        self._joint_state_monotonic = time.monotonic()

    def _on_targets(self, msg: Detection3DArray) -> None:
        if self._vision_reset_pending:
            # The completed task is held at HOME until the perception service
            # confirms that all pre-reset tracking history has been discarded.
            return
        if self._task_active:
            # Once the first Execute begins, use the frozen inspected target.
            # Chasing later detections would invalidate every following stage.
            return
        if msg.header.frame_id != self._world_frame:
            self.get_logger().error(
                f"Rejecting fruit targets in '{msg.header.frame_id}'; expected "
                f"'{self._world_frame}'"
            )
            self._invalidate_goal()
            return
        if self._require_perceived_table and not self._table_is_fresh():
            self.get_logger().warning(
                "Ignoring fruit targets until a fresh perceived table is stable"
            )
            self._invalidate_goal()
            return
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        now_s = self.get_clock().now().nanoseconds / 1e9
        age_s = now_s - stamp_s
        if (
            self._last_target_stamp_s is not None
            and stamp_s + 1.0e-9 < self._last_target_stamp_s
        ):
            self.get_logger().warning("Ignoring backward fruit target timestamp")
            return
        self._last_target_stamp_s = stamp_s
        if (
            stamp_s <= 0.0
            or (
                self._use_source_age_gate
                and (
                    age_s < -self._max_future_detection
                    or age_s > self._max_detection_age
                )
            )
        ):
            self.get_logger().warning(
                f"Ignoring stale/invalid fruit detection (age={age_s:.3f}s)"
            )
            self._invalidate_goal()
            return

        detections: list[dict] = []
        for index, detection in enumerate(msg.detections):
            if not detection.results:
                continue
            result = detection.results[0]
            position = detection.bbox.center.position
            radius = 0.5 * max(
                float(detection.bbox.size.x),
                float(detection.bbox.size.y),
                float(detection.bbox.size.z),
            )
            values = (position.x, position.y, position.z, radius)
            if not all(math.isfinite(float(value)) for value in values) or radius <= 0.0:
                continue
            if self._table_surface is not None and not self._fruit_on_table(
                float(position.x), float(position.y), float(position.z)
            ):
                self.get_logger().warning(
                    "Rejecting fruit outside perceived table support: "
                    "point=(%.3f, %.3f, %.3f)m" % (
                        position.x, position.y, position.z
                    )
                )
                continue
            source_id = str(detection.id or f"fruit_{index:02d}")
            if source_id in self._completed_source_ids:
                continue
            detections.append({
                "source_id": source_id,
                "collision_id": "perceived_" + re.sub(
                    r"[^A-Za-z0-9_.-]", "_", source_id
                ),
                "x": float(position.x),
                "y": float(position.y),
                "z": float(position.z),
                "radius": radius,
                "confidence": float(result.hypothesis.score),
                "health": str(result.hypothesis.class_id).strip(),
                "task_blocked": self._target_is_unreachable(
                    float(position.x), float(position.y), float(position.z)
                ),
            })
        self._detections = detections
        if not detections:
            self._invalidate_goal()
            return

        # Upstream tracking already guarantees stability. Prefer confidence;
        # distance breaks ties deterministically when several fruits are seen.
        classified = [
            item for item in detections
            if item["health"].strip().lower()
            in ("healthy", "unhealthy", "good", "bad", "0", "1")
        ]
        if not classified:
            self.get_logger().warning(
                "Stable fruit has no Healthy/Unhealthy result; refusing to "
                "start a sort task without a destination bin"
            )
            self._invalidate_goal()
            return
        eligible = [item for item in classified if not item["task_blocked"]]
        if not eligible:
            self.get_logger().warning(
                "All visible fruits are currently marked unreachable. Move one "
                "by at least %.0f mm or restart after changing the workcell pose."
                % (1000.0 * self._unreachable_target_exclusion_radius)
            )
            self._invalidate_goal()
            return
        # Confidence chooses the first fruit only. Once chosen, keep its KF
        # identity while it remains visible so small confidence fluctuations
        # between several apples cannot make the RViz goal jump between them.
        selected = select_task_target(
            eligible,
            self._selected["source_id"] if self._selected is not None else None,
        )
        self._selected = selected
        queue_signature = tuple(sorted(item["source_id"] for item in eligible))
        if queue_signature != self._last_queue_signature:
            self._last_queue_signature = queue_signature
            self.get_logger().info(
                "FRUIT_RVIZ_QUEUE: available=%d selected=%s waiting=%d ids=%s"
                % (
                    len(eligible),
                    selected["source_id"],
                    max(0, len(eligible) - 1),
                    ",".join(queue_signature),
                )
            )
        if not target_geometry_changed(
            self._last_goal_target, selected, self._target_change
        ):
            return
        previous = self._last_goal_target
        if previous is not None:
            displacement_mm = 1000.0 * math.sqrt(
                (selected["x"] - previous["x"]) ** 2
                + (selected["y"] - previous["y"]) ** 2
                + (selected["z"] - previous["z"]) ** 2
            )
            self.get_logger().warning(
                "FRUIT_TARGET_UPDATED: displacement=%.1fmm. Any previously "
                "planned path is now stale; wait for the next "
                "FRUIT_RVIZ_GOAL_READY, then click Plan again before Execute."
                % displacement_mm
            )
        self._goal_yaw = None
        self._ik_candidates = []
        self._accepted_goal_state = None
        self._accepted_goal_target = None
        self._accepted_goal_pose = None
        self._last_goal_target = dict(selected)
        self._stage = "pregrasp"
        self._place_xy = None
        self._place_top_z = None
        self._released = False
        self._request_generation += 1
        self._queued_generation = self._request_generation
        self._scene_update_pending = True
        self._request_scene_update()

    def _request_scene_update(self) -> None:
        if self._scene_request_in_flight or not self._scene_update_pending:
            return
        if not self._apply_scene.service_is_ready():
            self.get_logger().warning("Waiting for /apply_planning_scene")
            return
        if self._require_perceived_table and self._table_surface is None:
            return
        generation = self._queued_generation
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(self._table_collision())
        for kind, config in self._scene_cfg.get("bins", {}).items():
            try:
                scene.world.collision_objects.append(
                    self._bin_collision(f"bin_{kind}", config)
                )
            except ValueError as exc:
                self.get_logger().error(
                    f"Invalid real sorting bin '{kind}': {exc}"
                )
                return

        current_ids = {item["collision_id"] for item in self._detections}
        for stale_id in self._collision_ids - current_ids:
            stale = CollisionObject()
            stale.header.frame_id = self._world_frame
            stale.id = stale_id
            stale.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(stale)
        for item in self._detections:
            scene.world.collision_objects.append(self._fruit_collision(item))

        request = ApplyPlanningScene.Request()
        request.scene = scene
        self._scene_request_in_flight = True
        future = self._apply_scene.call_async(request)
        future.add_done_callback(
            lambda completed, gen=generation, ids=current_ids: self._scene_done(
                completed, gen, ids
            )
        )

    def _scene_done(self, future, generation: int, collision_ids: set[str]) -> None:
        self._scene_request_in_flight = False
        try:
            result = future.result()
        except Exception as exc:  # ROS future transports the service exception.
            self.get_logger().error(f"PlanningScene update failed: {exc}")
            return
        if result is None or not result.success:
            self.get_logger().error("PlanningScene rejected table/fruit geometry")
            return
        self._collision_ids = collision_ids
        if generation != self._request_generation:
            self._request_scene_update()
            return
        self._scene_update_pending = False
        self._build_ik_candidates()
        self._request_ik(generation, 0)

    def _selected_bin(self) -> dict:
        target = self._selected
        if target is None:
            raise ValueError("no selected fruit")
        health = target["health"].strip().lower()
        if health in ("healthy", "good", "0"):
            kind = "healthy"
        elif health in ("unhealthy", "bad", "1"):
            kind = "unhealthy"
        else:
            raise ValueError(f"unknown fruit health '{target['health']}'")
        try:
            return self._scene_cfg["bins"][kind]
        except (KeyError, TypeError) as exc:
            raise ValueError(f"real '{kind}' sorting bin is not configured") from exc

    def _pose(self, x: float, y: float, z: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._world_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation = top_down_quaternion(yaw)
        return pose

    def _build_ik_candidates(self) -> None:
        target = self._selected
        if target is None:
            self._ik_candidates = []
            return
        candidates: list[dict] = []
        if self._stage == "pregrasp":
            for yaw in symmetric_yaw_candidates(self._preferred_top_down_yaw):
                candidates.append({
                    "pose": self._pregrasp_pose(target, yaw), "yaw": yaw,
                })
        elif self._stage == "grasp":
            if self._goal_yaw is None:
                self._ik_candidates = []
                return
            # The pregrasp yaw is preferred, but a lower descent can cross a
            # wrist/joint-limit or collide with a neighbouring apple. Try the
            # eight vertical wrist orientations before
            # declaring this physical fruit unreachable.
            for yaw in symmetric_yaw_candidates(self._goal_yaw):
                candidates.append({
                    "pose": self._pose(target["x"], target["y"], target["z"], yaw),
                    "yaw": yaw,
                })
        elif self._stage == "lift":
            if self._goal_yaw is None:
                self._ik_candidates = []
                return
            candidates.append({
                "pose": self._pose(
                    target["x"], target["y"],
                    target["z"] + self._lift_height, self._goal_yaw,
                ),
                "yaw": self._goal_yaw,
            })
        elif self._stage == "place_hover":
            if self._goal_yaw is None:
                self._ik_candidates = []
                return
            try:
                bin_cfg = self._selected_bin()
                normalized = normalized_bin_config(bin_cfg)
                self._place_top_z = float(normalized["top_z"])
                points = bin_drop_candidates(
                    bin_cfg, target["radius"], self._place_wall_clearance
                )
            except ValueError as exc:
                self.get_logger().error(f"Cannot create bin drop target: {exc}")
                self._ik_candidates = []
                return
            hover_z = self._place_top_z + self._place_approach_height
            for x, y in points:
                candidates.append({
                    "pose": self._pose(x, y, hover_z, self._goal_yaw),
                    "yaw": self._goal_yaw,
                    "place_xy": (x, y),
                })
        elif self._stage in ("place_drop", "place_retract"):
            if (
                self._goal_yaw is None
                or self._place_xy is None
                or self._place_top_z is None
            ):
                self._ik_candidates = []
                return
            if self._stage == "place_drop":
                z = (
                    self._place_top_z + target["radius"]
                    + self._place_release_clearance
                )
            else:
                z = self._place_top_z + self._place_approach_height
            candidates.append({
                "pose": self._pose(
                    self._place_xy[0], self._place_xy[1], z, self._goal_yaw
                ),
                "yaw": self._goal_yaw,
            })
        self._ik_candidates = candidates

    def _request_ik(self, generation: int, candidate_index: int) -> None:
        target = self._selected
        if target is None:
            return
        if self._require_perceived_table and not self._table_is_fresh():
            self.get_logger().error("Cannot create fruit goal without fresh table perception")
            return
        if self._joint_state is None or (
            time.monotonic() - self._joint_state_monotonic
        ) > self._max_joint_state_age:
            self.get_logger().error("Cannot create fruit goal without fresh joint state")
            return
        if not self._compute_ik.service_is_ready():
            self.get_logger().warning("Waiting for /compute_ik")
            return

        if candidate_index >= len(self._ik_candidates):
            return
        candidate = self._ik_candidates[candidate_index]
        yaw = float(candidate["yaw"])
        pose = candidate["pose"]
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._planning_group
        request.ik_request.ik_link_name = self._ik_link
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = int(self._ik_timeout)
        request.ik_request.timeout.nanosec = int(
            (self._ik_timeout - int(self._ik_timeout)) * 1e9
        )
        request.ik_request.robot_state.is_diff = True
        request.ik_request.robot_state.joint_state = self._joint_state
        future = self._compute_ik.call_async(request)
        future.add_done_callback(
            lambda completed, gen=generation, target_id=target["source_id"],
            index=candidate_index, candidate_data=candidate:
            self._ik_done(completed, gen, target_id, index, candidate_data)
        )

    def _ik_done(
        self,
        future,
        generation: int,
        target_id: str,
        candidate_index: int,
        candidate: dict,
    ) -> None:
        if generation != self._request_generation:
            return
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Fruit {self._stage} IK request failed: {exc}")
            return
        if result is None or result.error_code.val != 1:
            code = result.error_code.val if result is not None else "no response"
            next_index = candidate_index + 1
            if next_index < len(self._ik_candidates):
                self._request_ik(generation, next_index)
                return
            now = time.monotonic()
            if now - self._last_ik_failure_log >= 2.0:
                self.get_logger().error(
                    "No collision-free IK for stage=%s target=%s after %d "
                    "candidate(s) (MoveIt code=%s); no RViz goal was published."
                    % (self._stage, target_id, len(self._ik_candidates), code)
                )
                self._last_ik_failure_log = now
            if self._stage in ("pregrasp", "grasp"):
                self._skip_unreachable_target(target_id)
            return
        goal = result.solution
        goal.is_diff = False
        self._goal_yaw = float(candidate["yaw"])
        target = self._selected
        if target is None:
            return
        self._accepted_goal_state = goal
        self._accepted_goal_target = dict(target)
        self._accepted_goal_pose = candidate["pose"]
        if "place_xy" in candidate:
            self._place_xy = tuple(candidate["place_xy"])
        self._bound_arm_goal_id = None
        self._pending_arm_success_at = None
        self._stage_status_baseline = set(self._arm_status_ids)
        self._publish_goal_state(goal)
        self.get_logger().info(
            "FRUIT_RVIZ_STAGE_READY: stage=%s id=%s target=(%.3f, %.3f, %.3f)m "
            "confidence=%.2f. Click Plan, inspect the path, "
            "then click Execute only if safe."
            % (
                self._stage, target_id,
                candidate["pose"].pose.position.x,
                candidate["pose"].pose.position.y,
                candidate["pose"].pose.position.z,
                target["confidence"],
            )
        )
        if self._stage == "pregrasp":
            self.get_logger().info("FRUIT_RVIZ_GOAL_READY")

    def _pregrasp_z(self, target: dict) -> float:
        return pregrasp_tcp_z(
            target["z"],
            target["radius"],
            self._finger_tip_beyond_tcp,
            self._clearance_above_fruit,
        )

    def _target_is_unreachable(self, x: float, y: float, z: float) -> bool:
        threshold_sq = self._unreachable_target_exclusion_radius ** 2
        return any(
            (x - blocked_x) ** 2
            + (y - blocked_y) ** 2
            + (z - blocked_z) ** 2 <= threshold_sq
            for blocked_x, blocked_y, blocked_z in self._unreachable_positions
        )

    def _skip_unreachable_target(self, target_id: str) -> None:
        """Restore the scene and safely expose another fruit without arm motion."""
        target = self._selected
        if target is None or target["source_id"] != target_id:
            return
        self._unreachable_positions.append(
            (target["x"], target["y"], target["z"])
        )
        failed_stage = self._stage
        self._task_active = False
        self._stage = "pregrasp"
        self._selected = None
        self._last_goal_target = None
        self._accepted_goal_state = None
        self._accepted_goal_target = None
        self._accepted_goal_pose = None
        self._goal_yaw = None
        self._ik_candidates = []
        self._bound_arm_goal_id = None
        self._pending_arm_success_at = None
        self._last_queue_signature = None
        self._place_xy = None
        self._place_top_z = None
        self._request_generation += 1
        self._queued_generation = self._request_generation
        self._scene_update_pending = True
        # At grasp time the selected sphere was intentionally removed. Adding
        # the complete frozen detection set back makes the skipped apple an
        # obstacle for every following Plan.
        self._request_scene_update()
        if self._joint_state is not None:
            current = RobotState()
            current.joint_state = self._joint_state
            current.is_diff = False
            self._publish_goal_state(current)
        self.get_logger().error(
            "FRUIT_RVIZ_TARGET_SKIPPED: stage=%s id=%s position=(%.3f, %.3f, %.3f)m; "
            "all collision-free IK candidates failed. The fruit collision sphere "
            "was restored; waiting for another target. No arm motion was sent."
            % (
                failed_stage, target_id,
                target["x"], target["y"], target["z"],
            )
        )

    def _pregrasp_pose(self, target: dict, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._world_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target["x"]
        pose.pose.position.y = target["y"]
        pose.pose.position.z = self._pregrasp_z(target)
        pose.pose.orientation = top_down_quaternion(yaw)
        return pose

    @staticmethod
    def _goal_uuid(status) -> bytes:
        return bytes(status.goal_info.goal_id.uuid)

    def _on_arm_status(self, msg: GoalStatusArray) -> None:
        statuses = {self._goal_uuid(item): item.status for item in msg.status_list}
        self._arm_status_ids.update(statuses)
        if self._accepted_goal_state is None or self._stage == "complete":
            return

        if self._bound_arm_goal_id is None:
            for item in msg.status_list:
                goal_id = self._goal_uuid(item)
                if (
                    goal_id not in self._stage_status_baseline
                    and item.status
                    in (
                        GoalStatus.STATUS_ACCEPTED,
                        GoalStatus.STATUS_EXECUTING,
                        GoalStatus.STATUS_SUCCEEDED,
                        GoalStatus.STATUS_CANCELED,
                        GoalStatus.STATUS_ABORTED,
                    )
                ):
                    self._bound_arm_goal_id = goal_id
                    if self._stage == "pregrasp":
                        self._task_active = True
                    self.get_logger().info(
                        f"RVIZ_EXECUTE_OBSERVED: stage={self._stage}; "
                        "target and collision scene are now frozen"
                    )
                    break
        if self._bound_arm_goal_id is None:
            return

        status = statuses.get(self._bound_arm_goal_id)
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._pending_arm_success_at is None:
                self._pending_arm_success_at = time.monotonic()
        elif status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            label = "canceled" if status == GoalStatus.STATUS_CANCELED else "aborted"
            self.get_logger().error(
                f"RViz arm execution {label} at stage={self._stage}; "
                "the stage will not advance"
            )
            self._stage_status_baseline.add(self._bound_arm_goal_id)
            self._bound_arm_goal_id = None
            self._pending_arm_success_at = None

    def _joint_goal_reached(self) -> tuple[bool, float]:
        if self._joint_state is None or self._accepted_goal_state is None:
            return False, math.inf
        current = dict(zip(self._joint_state.name, self._joint_state.position))
        goal = dict(zip(
            self._accepted_goal_state.joint_state.name,
            self._accepted_goal_state.joint_state.position,
        ))
        names = [f"joint_{index}" for index in range(1, 7)]
        if not all(name in current and name in goal for name in names):
            return False, math.inf
        max_error = max(abs(float(current[name]) - float(goal[name])) for name in names)
        return max_error <= self._arm_goal_tolerance, max_error

    def _complete_arm_stage(self) -> None:
        completed = self._stage
        self._accepted_goal_state = None
        self._accepted_goal_pose = None
        self._bound_arm_goal_id = None
        self._pending_arm_success_at = None
        self.get_logger().info(f"RVIZ_STAGE_EXECUTED: stage={completed}")
        if completed == "pregrasp":
            # The real gripper is fail-closed and has no measured aperture.
            # Open it only after the operator-approved pregrasp motion has
            # finished, then expose the descent goal.  This prevents a closed
            # four-finger gripper from descending onto the fruit.
            self._send_gripper("pregrasp_open")
        elif completed == "grasp":
            self._send_gripper("close")
        elif completed == "lift":
            self._advance_stage("place_hover")
        elif completed == "place_hover":
            self._advance_stage("place_drop")
        elif completed == "place_drop":
            self._send_gripper("open")
        elif completed == "place_retract":
            self._publish_home_goal()
        elif completed == "retreat_home":
            self._finish_task()

    def _advance_stage(self, stage: str) -> None:
        self._stage = stage
        self._request_generation += 1
        self._build_ik_candidates()
        if not self._ik_candidates:
            self.get_logger().error(
                f"Cannot generate candidates for stage={stage}; task is held"
            )
            return
        self._request_ik(self._request_generation, 0)

    def _publish_home_goal(self) -> None:
        try:
            positions = [float(value) for value in self._scene_cfg["home_pose"]]
        except (KeyError, TypeError, ValueError):
            self.get_logger().error("Cannot return home: real home_pose is invalid")
            return
        if len(positions) != 6 or not all(math.isfinite(value) for value in positions):
            self.get_logger().error("Cannot return home: home_pose must contain 6 numbers")
            return
        goal = RobotState()
        goal.is_diff = False
        goal.joint_state.header.frame_id = self._world_frame
        goal.joint_state.header.stamp = self.get_clock().now().to_msg()
        goal.joint_state.name = [f"joint_{index}" for index in range(1, 7)]
        goal.joint_state.position = positions
        self._stage = "retreat_home"
        self._accepted_goal_state = goal
        self._accepted_goal_target = dict(self._selected) if self._selected else None
        self._accepted_goal_pose = None
        self._bound_arm_goal_id = None
        self._pending_arm_success_at = None
        self._stage_status_baseline = set(self._arm_status_ids)
        self._publish_goal_state(goal)
        self.get_logger().info(
            "FRUIT_RVIZ_STAGE_READY: stage=retreat_home. Click Plan, inspect "
            "the return path, then Execute only if safe."
        )

    def _apply_transition_scene(
        self, scene: PlanningScene, description: str, callback
    ) -> None:
        if self._scene_request_in_flight:
            self.get_logger().error(
                f"Cannot {description}: another PlanningScene update is active"
            )
            return
        if not self._apply_scene.service_is_ready():
            self.get_logger().error(f"Cannot {description}: scene service unavailable")
            return
        request = ApplyPlanningScene.Request()
        request.scene = scene
        self._scene_request_in_flight = True
        future = self._apply_scene.call_async(request)

        def done(completed):
            self._scene_request_in_flight = False
            try:
                result = completed.result()
            except Exception as exc:
                self.get_logger().error(f"Failed to {description}: {exc}")
                return
            if result is None or not result.success:
                self.get_logger().error(f"MoveIt refused to {description}")
                return
            callback()

        future.add_done_callback(done)

    def _remove_selected_world(self, callback, description: str) -> None:
        target = self._selected
        if target is None:
            self.get_logger().error(f"Cannot {description}: selected fruit is missing")
            return
        scene = PlanningScene()
        scene.is_diff = True
        remove = CollisionObject()
        remove.header.frame_id = self._world_frame
        remove.id = target["collision_id"]
        remove.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(remove)

        def removed():
            self._collision_ids.discard(target["collision_id"])
            callback()

        self._apply_transition_scene(scene, description, removed)

    def _attach_selected(self, callback) -> None:
        target = self._selected
        if target is None:
            self.get_logger().error("Cannot attach a missing selected fruit")
            return
        attached = AttachedCollisionObject()
        attached.link_name = self._ik_link
        attached.touch_links = [
            "gripper_base", "gripper_center_slider",
            "gripper_drive_neg_x", "gripper_drive_neg_y",
            "gripper_drive_pos_x", "gripper_drive_pos_y",
            "gripper_finger_neg_x", "gripper_finger_neg_y",
            "gripper_finger_pos_x", "gripper_finger_pos_y",
        ]
        attached.object.header.frame_id = self._ik_link
        attached.object.id = target["collision_id"]
        attached.object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [target["radius"]]
        pose = Pose()
        pose.orientation.w = 1.0
        attached.object.primitives.append(primitive)
        attached.object.primitive_poses.append(pose)
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)

        def attached_done():
            self._attached = True
            self.get_logger().info(
                "PlanningScene fruit attached to gripper_tcp for lift/place avoidance"
            )
            callback()

        self._apply_transition_scene(scene, "attach selected fruit", attached_done)

    def _detach_selected(self, callback) -> None:
        target = self._selected
        if target is None:
            self.get_logger().error("Cannot detach a missing selected fruit")
            return
        attached = AttachedCollisionObject()
        attached.link_name = self._ik_link
        attached.object.id = target["collision_id"]
        attached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)

        def detached_done():
            self._attached = False
            self._released = True

            # MoveIt detaching semantics can return the object to the world at
            # the release pose.  Leaving that sphere there makes the gripper's
            # first retract state collide with the just-released fruit, so the
            # manual task can never finish and no second fruit can be chosen.
            # The physical bin remains a collision object; remove only this
            # completed fruit sphere before exposing the retract goal.
            def released_world_removed():
                self._completed_source_ids.add(target["source_id"])
                callback()

            self._remove_selected_world(
                released_world_removed,
                "remove released fruit before collision-free retract",
            )

        self._apply_transition_scene(scene, "detach released fruit", detached_done)

    def _send_gripper(self, operation: str) -> None:
        if self._gripper_operation is not None:
            return
        if not self._gripper_client.server_is_ready():
            self.get_logger().error(
                f"Cannot {operation} gripper: GripperCommand server unavailable; "
                "task is held"
            )
            return
        goal = GripperCommand.Goal()
        goal.command.position = (
            self._gripper_closed_position
            if operation == "close" else self._gripper_open_position
        )
        goal.command.max_effort = self._gripper_max_effort
        self._gripper_operation = operation
        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, op=operation: self._gripper_goal_response(completed, op)
        )

    def _gripper_goal_response(self, future, operation: str) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._gripper_operation = None
            self.get_logger().error(f"Gripper {operation} request failed: {exc}")
            return
        if handle is None or not handle.accepted:
            self._gripper_operation = None
            self.get_logger().error(f"Gripper {operation} command was rejected")
            return
        handle.get_result_async().add_done_callback(
            lambda completed, op=operation: self._gripper_result(completed, op)
        )

    def _gripper_result(self, future, operation: str) -> None:
        self._gripper_operation = None
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(f"Gripper {operation} result failed: {exc}")
            return
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Gripper {operation} did not succeed; task is held"
            )
            return
        self.get_logger().info(f"GRIPPER_{operation.upper()}_SUCCEEDED")
        if operation == "pregrasp_open":
            self._remove_selected_world(
                lambda: self._advance_stage("grasp"),
                "remove selected fruit before intentional grasp contact",
            )
        elif operation == "close":
            self._attach_selected(lambda: self._advance_stage("lift"))
        else:
            self._detach_selected(lambda: self._advance_stage("place_retract"))

    def _finish_task(self) -> None:
        completed_id = self._selected["source_id"] if self._selected else "unknown"
        self._stage = "complete"
        self._task_active = False
        self._accepted_goal_state = None
        self._accepted_goal_target = None
        self._accepted_goal_pose = None
        self._last_goal_target = None
        self._released = False
        self._selected = None
        self._detections = []
        self._last_queue_signature = None
        self._place_xy = None
        self._place_top_z = None
        # Remove every fruit collision from the inspected snapshot.  The
        # fixed table and bins are rebuilt unchanged by the scene update.
        self._request_generation += 1
        self._queued_generation = self._request_generation
        self._scene_update_pending = True
        self._request_scene_update()
        self._begin_light_vision_reset(completed_id)
        if self._joint_state is not None:
            current = RobotState()
            current.joint_state = self._joint_state
            current.is_diff = False
            self._publish_goal_state(current)
        self.get_logger().info(
            f"FRUIT_RVIZ_TASK_COMPLETE: id={completed_id}; returned HOME; "
            "light vision reset requested"
        )

    def _begin_light_vision_reset(self, completed_id: str) -> None:
        """Asynchronously clear tracker history after the final HOME Execute."""
        self._vision_reset_pending = True
        self._vision_reset_completed_id = str(completed_id)
        self._vision_reset_future = None
        self._vision_reset_retry_at = 0.0
        self._try_light_vision_reset()

    def _try_light_vision_reset(self) -> None:
        if not self._vision_reset_pending or self._vision_reset_future is not None:
            return
        if time.monotonic() < self._vision_reset_retry_at:
            return
        if not self._vision_reset_client.service_is_ready():
            self._vision_reset_retry_at = time.monotonic() + 0.5
            return
        future = self._vision_reset_client.call_async(Trigger.Request())
        self._vision_reset_future = future
        future.add_done_callback(self._light_vision_reset_done)

    def _light_vision_reset_done(self, future) -> None:
        self._vision_reset_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Light vision reset service failed: {exc}")
            self._vision_reset_retry_at = time.monotonic() + 1.0
            return
        if response is None or not response.success:
            message = response.message if response is not None else "no response"
            self.get_logger().error(
                f"Light vision reset was rejected: {message}; task remains at HOME"
            )
            self._vision_reset_retry_at = time.monotonic() + 1.0
            return
        completed_id = self._vision_reset_completed_id
        self._vision_reset_pending = False
        self._vision_reset_completed_id = ""
        self._last_target_stamp_s = None
        self.get_logger().info(
            "RVIZ_LIGHT_VISION_RESET_COMPLETE: completed=%s; old target, "
            "tracking history and fruit collision snapshot cleared; waiting "
            "for a new 5-frame-stable fruit"
            % completed_id
        )

    def _table_collision(self) -> CollisionObject:
        sensed = self._table_surface
        if sensed is None:
            table = self._scene_cfg["table"]
            center_x = float(table["center"]["x"])
            center_y = float(table["center"]["y"])
            size_x = float(table["size"]["x"])
            size_y = float(table["size"]["y"])
            size_z = float(table["size"]["z"])
            center_z = float(table["top_z"]) - size_z * 0.5
        else:
            center_x = sensed["center_x"]
            center_y = sensed["center_y"]
            center_z = sensed["center_z"]
            size_x = sensed["size_x"]
            size_y = sensed["size_y"]
            size_z = sensed["size_z"]
        obj = CollisionObject()
        obj.header.frame_id = self._world_frame
        obj.id = "work_table"
        obj.operation = CollisionObject.ADD
        rectangles = [(center_x, center_y, size_x, size_y)]
        if sensed is not None and self._table_base_cutout_enabled:
            rectangles = self._subtract_base_cutout(
                center_x, center_y, size_x, size_y
            )
        for rect_x, rect_y, rect_size_x, rect_size_y in rectangles:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [rect_size_x, rect_size_y, size_z]
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x = rect_x
            pose.position.y = rect_y
            pose.position.z = center_z
            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
        return obj

    def _subtract_base_cutout(
        self, center_x: float, center_y: float, size_x: float, size_y: float
    ) -> list[tuple[float, float, float, float]]:
        """Split a tabletop rectangle around the fixed CAD base footprint."""
        table_min_x = center_x - size_x * 0.5
        table_max_x = center_x + size_x * 0.5
        table_min_y = center_y - size_y * 0.5
        table_max_y = center_y + size_y * 0.5
        cut_min_x, cut_max_x, cut_min_y, cut_max_y = self._table_base_cutout
        cut_min_x = max(table_min_x, cut_min_x)
        cut_max_x = min(table_max_x, cut_max_x)
        cut_min_y = max(table_min_y, cut_min_y)
        cut_max_y = min(table_max_y, cut_max_y)
        if cut_min_x >= cut_max_x or cut_min_y >= cut_max_y:
            return [(center_x, center_y, size_x, size_y)]

        rectangles: list[tuple[float, float, float, float]] = []

        def append_rect(min_x: float, max_x: float, min_y: float, max_y: float):
            if max_x - min_x > 0.001 and max_y - min_y > 0.001:
                rectangles.append(
                    (
                        (min_x + max_x) * 0.5,
                        (min_y + max_y) * 0.5,
                        max_x - min_x,
                        max_y - min_y,
                    )
                )

        append_rect(table_min_x, cut_min_x, table_min_y, table_max_y)
        append_rect(cut_max_x, table_max_x, table_min_y, table_max_y)
        append_rect(cut_min_x, cut_max_x, table_min_y, cut_min_y)
        append_rect(cut_min_x, cut_max_x, cut_max_y, table_max_y)
        return rectangles

    def _fruit_collision(self, target: dict) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self._world_frame
        obj.id = target["collision_id"]
        obj.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [target["radius"]]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = target["x"]
        pose.position.y = target["y"]
        pose.position.z = target["z"]
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        return obj

    def _bin_collision(self, object_id: str, config: dict) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self._world_frame
        obj.id = object_id
        obj.operation = CollisionObject.ADD
        for wall in bin_wall_boxes(config):
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(wall.size)
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x, pose.position.y, pose.position.z = wall.center
            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
        return obj

    def _publish_visuals(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._world_frame
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        # A dedicated, uncluttered physical base triad makes the surveyed
        # coordinate contract visible even when RViz's full TF display is
        # crowded.  J1=0 lies in the red +X / blue +Z plane.
        axis_origin = Point(x=0.0, y=0.0, z=0.12)
        axis_specs = (
            ("+X FRONT / J1=0", Point(x=0.30, y=0.0, z=0.12), (1.0, 0.05, 0.05)),
            ("+Y LEFT", Point(x=0.0, y=0.30, z=0.12), (0.05, 1.0, 0.05)),
            ("+Z UP", Point(x=0.0, y=0.0, z=0.42), (0.05, 0.20, 1.0)),
        )
        for axis_id, (label, endpoint, color) in enumerate(axis_specs):
            arrow = Marker()
            arrow.header.frame_id = self._base_frame
            arrow.header.stamp = now
            arrow.ns = "physical_base_axes"
            arrow.id = axis_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [axis_origin, endpoint]
            arrow.scale.x = 0.012
            arrow.scale.y = 0.025
            arrow.scale.z = 0.035
            arrow.color.r, arrow.color.g, arrow.color.b = color
            arrow.color.a = 1.0
            markers.markers.append(arrow)

            text = Marker()
            text.header = arrow.header
            text.ns = "physical_base_axis_labels"
            text.id = axis_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.orientation.w = 1.0
            text.pose.position = Point(
                x=endpoint.x, y=endpoint.y, z=endpoint.z + 0.025
            )
            text.scale.z = 0.035
            text.color.r, text.color.g, text.color.b = color
            text.color.a = 1.0
            text.text = label
            markers.markers.append(text)
        if self._table_surface is not None:
            table = Marker()
            table.header.frame_id = self._world_frame
            table.header.stamp = now
            table.ns = "perceived_table"
            table.id = 0
            table.type = Marker.CUBE
            table.action = Marker.ADD
            table.pose.orientation.w = 1.0
            table.pose.position.x = self._table_surface["center_x"]
            table.pose.position.y = self._table_surface["center_y"]
            table.pose.position.z = self._table_surface["center_z"]
            table.scale.x = self._table_surface["size_x"]
            table.scale.y = self._table_surface["size_y"]
            table.scale.z = self._table_surface["size_z"]
            table.color.r = 0.10
            table.color.g = 0.75
            table.color.b = 0.95
            table.color.a = 0.55
            markers.markers.append(table)
        marker_id = 10
        for kind, config in self._scene_cfg.get("bins", {}).items():
            try:
                walls = bin_wall_boxes(config)
            except ValueError:
                continue
            color = (
                (0.18, 0.78, 0.30)
                if str(kind).lower() == "healthy"
                else (0.80, 0.30, 0.15)
            )
            for wall in walls:
                marker = Marker()
                marker.header.frame_id = self._world_frame
                marker.header.stamp = now
                marker.ns = f"sorting_bin_{kind}"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                (
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                ) = wall.center
                marker.scale.x, marker.scale.y, marker.scale.z = wall.size
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 0.70
                markers.markers.append(marker)
        # Draw one conspicuous sphere exactly over the PlanningScene collision
        # sphere.  It uses the same accepted geometry, so the operator sees one
        # fruit rather than a visually displaced duplicate.
        if (
            self._last_goal_target is not None
            and not self._attached
            and not self._released
        ):
            target = self._last_goal_target
            fruit = Marker()
            fruit.header.frame_id = self._world_frame
            fruit.header.stamp = now
            fruit.ns = "selected_fruit"
            fruit.id = 0
            fruit.type = Marker.SPHERE
            fruit.action = Marker.ADD
            fruit.pose.orientation.w = 1.0
            fruit.pose.position.x = target["x"]
            fruit.pose.position.y = target["y"]
            fruit.pose.position.z = target["z"]
            diameter = 2.0 * target["radius"]
            fruit.scale.x = diameter
            fruit.scale.y = diameter
            fruit.scale.z = diameter
            fruit.color.r = 1.0
            fruit.color.g = 0.32
            fruit.color.b = 0.02
            fruit.color.a = 0.92
            markers.markers.append(fruit)
        if self._accepted_goal_pose is not None:
            stage_text = Marker()
            stage_text.header.frame_id = self._world_frame
            stage_text.header.stamp = now
            stage_text.ns = "fruit_task_stage"
            stage_text.id = 0
            stage_text.type = Marker.TEXT_VIEW_FACING
            stage_text.action = Marker.ADD
            stage_text.pose.orientation.w = 1.0
            stage_text.pose.position.x = self._accepted_goal_pose.pose.position.x
            stage_text.pose.position.y = self._accepted_goal_pose.pose.position.y
            stage_text.pose.position.z = self._accepted_goal_pose.pose.position.z + 0.08
            stage_text.scale.z = 0.045
            stage_text.color.r = 1.0
            stage_text.color.g = 1.0
            stage_text.color.b = 0.1
            stage_text.color.a = 1.0
            stage_text.text = f"NEXT: {self._stage} | Plan -> inspect -> Execute"
            markers.markers.append(stage_text)
        self._marker_pub.publish(markers)

        # Publish one target frame only.  Reusing one child name prevents stale
        # pregrasp/grasp/place frames from looking like duplicate flying fruit.
        if self._accepted_goal_pose is not None:
            pose = self._accepted_goal_pose
            transform = TransformStamped()
            transform.header = pose.header
            transform.header.stamp = now
            transform.child_frame_id = "fruit_task_target"
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = pose.pose.position.z
            transform.transform.rotation = pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)

    def _invalidate_goal(self) -> None:
        if self._task_active:
            return
        had_target = self._selected is not None or bool(self._collision_ids)
        self._detections = []
        self._selected = None
        self._last_queue_signature = None
        self._goal_yaw = None
        self._ik_candidates = []
        self._accepted_goal_state = None
        self._accepted_goal_target = None
        self._accepted_goal_pose = None
        self._last_goal_target = None
        self._released = False
        if not had_target:
            return
        self._request_generation += 1
        self._queued_generation = self._request_generation
        self._scene_update_pending = True
        self._request_scene_update()
        # Move RViz's query goal back to the current measured state so a lost
        # target cannot silently leave a stale fruit goal selected.
        if self._joint_state is not None:
            current = RobotState()
            current.joint_state = self._joint_state
            current.is_diff = False
            self._publish_goal_state(current)
        self.get_logger().warning(
            "Fruit target lost/invalid: RViz goal reset to current robot state"
        )

    def _housekeeping(self) -> None:
        self._publish_visuals()
        self._try_light_vision_reset()
        if self._pending_arm_success_at is not None:
            reached, max_error = self._joint_goal_reached()
            if reached:
                self._complete_arm_stage()
            elif (
                time.monotonic() - self._pending_arm_success_at
                >= self._arm_goal_feedback_timeout
            ):
                self.get_logger().error(
                    "Controller reported success but measured arm did not reach "
                    f"stage={self._stage} goal (max joint error={max_error:.4f}rad); "
                    "the stage is held and will not advance"
                )
                if self._bound_arm_goal_id is not None:
                    self._stage_status_baseline.add(self._bound_arm_goal_id)
                self._bound_arm_goal_id = None
                self._pending_arm_success_at = None
        if (
            self._accepted_goal_state is not None
            and time.monotonic() - self._last_goal_publish_monotonic
            >= self._goal_republish_period
        ):
            self._publish_goal_state(self._accepted_goal_state)
        if self._scene_update_pending and not self._scene_request_in_flight:
            self._request_scene_update()

    def _publish_goal_state(self, goal: RobotState) -> None:
        self._goal_pub.publish(goal)
        self._last_goal_publish_monotonic = time.monotonic()

    def _table_is_fresh(self) -> bool:
        return (
            self._table_surface is not None
            and time.monotonic() - self._table_received_monotonic
            <= self._max_table_age
        )

    def _fruit_on_table(self, x: float, y: float, z: float) -> bool:
        table = self._table_surface
        if table is None:
            return not self._require_perceived_table
        margin = 0.04
        if abs(x - table["center_x"]) > table["size_x"] * 0.5 + margin:
            return False
        if abs(y - table["center_y"]) > table["size_y"] * 0.5 + margin:
            return False
        top_z = table["center_z"] + table["size_z"] * 0.5
        return (
            top_z - self._fruit_below_table_tolerance
            <= z
            <= top_z + self._fruit_max_height_above_table
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FruitRvizGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
