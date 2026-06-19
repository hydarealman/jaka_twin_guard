#!/usr/bin/env python3
"""RViz dual-arm massage demo with a bed scene and collision-aware motion."""

from __future__ import annotations

import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


LEFT_JOINTS = [
    "left_joint_1",
    "left_joint_2",
    "left_joint_3",
    "left_joint_4",
    "left_joint_5",
    "left_joint_6",
]

RIGHT_JOINTS = [
    "right_joint_1",
    "right_joint_2",
    "right_joint_3",
    "right_joint_4",
    "right_joint_5",
    "right_joint_6",
]

ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS

# Choreography primitives: hover, paired press, alternating knead, stroke, wrist roll.
MASSAGE_STAGE_NAMES = [
    "hover above shoulders",
    "paired shoulder press",
    "release shoulder pressure",
    "left shoulder knead",
    "right shoulder knead",
    "paired shoulder roll inward",
    "paired shoulder roll outward",
    "left shoulder percussion tap",
    "right shoulder percussion tap",
    "sweep toward mid back",
    "left mid-back knead",
    "right mid-back knead",
    "paired mid-back press",
    "release mid-back pressure",
    "second mid-back compression",
    "roll palms inward",
    "roll palms outward",
    "left mid-back percussion tap",
    "right mid-back percussion tap",
    "center mid-back squeeze",
    "return sweep",
    "return shoulder press",
    "left finishing knead",
    "right finishing knead",
    "paired finishing press",
    "release shoulder pressure",
    "release to hover",
]

LEFT_SHOULDER_HOVER = [0.014, 1.662, -1.579, 1.488, 1.571, 1.585]
LEFT_SHOULDER_PRESS = [0.014, 1.663, -1.729, 1.637, 1.571, 1.585]
LEFT_SHOULDER_KNEAD = [0.014, 1.663, -1.729, 1.637, 1.571, 1.665]
LEFT_SHOULDER_ROLL_IN = [0.014, 1.663, -1.729, 1.637, 1.571, 1.705]
LEFT_SHOULDER_ROLL_OUT = [0.014, 1.663, -1.729, 1.637, 1.571, 1.465]
LEFT_MID_HOVER = [0.011, 1.409, -1.424, 1.586, 1.571, 1.582]
LEFT_MID_PRESS = [0.011, 1.409, -1.574, 1.736, 1.571, 1.582]
LEFT_MID_KNEAD = [0.011, 1.409, -1.574, 1.736, 1.571, 1.662]
LEFT_MID_ROLL_IN = [0.011, 1.409, -1.574, 1.736, 1.571, 1.682]
LEFT_MID_ROLL_OUT = [0.011, 1.409, -1.574, 1.736, 1.571, 1.482]

RIGHT_SHOULDER_HOVER = [-0.518, 1.662, -1.579, 1.487, 1.571, 1.052]
RIGHT_SHOULDER_PRESS = [-0.518, 1.663, -1.729, 1.637, 1.571, 1.052]
RIGHT_SHOULDER_KNEAD = [-0.518, 1.663, -1.729, 1.637, 1.571, 0.972]
RIGHT_SHOULDER_ROLL_IN = [-0.518, 1.663, -1.729, 1.637, 1.571, 0.932]
RIGHT_SHOULDER_ROLL_OUT = [-0.518, 1.663, -1.729, 1.637, 1.571, 1.172]
RIGHT_MID_HOVER = [-0.418, 1.409, -1.424, 1.586, 1.571, 1.152]
RIGHT_MID_PRESS = [-0.418, 1.409, -1.574, 1.736, 1.571, 1.152]
RIGHT_MID_KNEAD = [-0.418, 1.409, -1.574, 1.736, 1.571, 1.072]
RIGHT_MID_ROLL_IN = [-0.418, 1.409, -1.574, 1.736, 1.571, 1.052]
RIGHT_MID_ROLL_OUT = [-0.418, 1.409, -1.574, 1.736, 1.571, 1.252]

LEFT_WAYPOINTS = [
    (1.0, LEFT_SHOULDER_HOVER),
    (1.8, LEFT_SHOULDER_PRESS),
    (2.5, LEFT_SHOULDER_HOVER),
    (3.2, LEFT_SHOULDER_KNEAD),
    (3.9, LEFT_SHOULDER_HOVER),
    (4.6, LEFT_SHOULDER_ROLL_IN),
    (5.3, LEFT_SHOULDER_ROLL_OUT),
    (6.0, LEFT_SHOULDER_PRESS),
    (6.7, LEFT_SHOULDER_HOVER),
    (7.6, LEFT_MID_HOVER),
    (8.3, LEFT_MID_KNEAD),
    (9.0, LEFT_MID_HOVER),
    (9.8, LEFT_MID_PRESS),
    (10.5, LEFT_MID_HOVER),
    (11.2, LEFT_MID_PRESS),
    (11.9, LEFT_MID_ROLL_IN),
    (12.6, LEFT_MID_ROLL_OUT),
    (13.3, LEFT_MID_PRESS),
    (14.0, LEFT_MID_HOVER),
    (14.7, LEFT_MID_KNEAD),
    (15.6, LEFT_MID_HOVER),
    (16.4, LEFT_SHOULDER_PRESS),
    (17.1, LEFT_SHOULDER_KNEAD),
    (17.8, LEFT_SHOULDER_HOVER),
    (18.5, LEFT_SHOULDER_PRESS),
    (19.2, LEFT_SHOULDER_HOVER),
    (20.0, LEFT_SHOULDER_HOVER),
]

RIGHT_WAYPOINTS = [
    (1.0, RIGHT_SHOULDER_HOVER),
    (1.8, RIGHT_SHOULDER_PRESS),
    (2.5, RIGHT_SHOULDER_HOVER),
    (3.2, RIGHT_SHOULDER_HOVER),
    (3.9, RIGHT_SHOULDER_KNEAD),
    (4.6, RIGHT_SHOULDER_ROLL_IN),
    (5.3, RIGHT_SHOULDER_ROLL_OUT),
    (6.0, RIGHT_SHOULDER_HOVER),
    (6.7, RIGHT_SHOULDER_PRESS),
    (7.6, RIGHT_MID_HOVER),
    (8.3, RIGHT_MID_HOVER),
    (9.0, RIGHT_MID_KNEAD),
    (9.8, RIGHT_MID_PRESS),
    (10.5, RIGHT_MID_HOVER),
    (11.2, RIGHT_MID_PRESS),
    (11.9, RIGHT_MID_ROLL_IN),
    (12.6, RIGHT_MID_ROLL_OUT),
    (13.3, RIGHT_MID_HOVER),
    (14.0, RIGHT_MID_PRESS),
    (14.7, RIGHT_MID_KNEAD),
    (15.6, RIGHT_MID_HOVER),
    (16.4, RIGHT_SHOULDER_PRESS),
    (17.1, RIGHT_SHOULDER_HOVER),
    (17.8, RIGHT_SHOULDER_KNEAD),
    (18.5, RIGHT_SHOULDER_PRESS),
    (19.2, RIGHT_SHOULDER_HOVER),
    (20.0, RIGHT_SHOULDER_HOVER),
]

MASSAGE_POINTS_LEFT = [
    Point(x=0.44, y=-0.13, z=0.46),
    Point(x=0.44, y=-0.13, z=0.405),
    Point(x=0.44, y=-0.13, z=0.44),
    Point(x=0.44, y=-0.15, z=0.405),
    Point(x=0.44, y=-0.13, z=0.46),
    Point(x=0.44, y=-0.15, z=0.405),
    Point(x=0.44, y=-0.11, z=0.405),
    Point(x=0.44, y=-0.13, z=0.405),
    Point(x=0.44, y=-0.13, z=0.46),
    Point(x=0.55, y=-0.13, z=0.420),
    Point(x=0.55, y=-0.15, z=0.395),
    Point(x=0.55, y=-0.13, z=0.420),
    Point(x=0.55, y=-0.13, z=0.395),
    Point(x=0.55, y=-0.13, z=0.420),
    Point(x=0.55, y=-0.13, z=0.395),
    Point(x=0.55, y=-0.15, z=0.398),
    Point(x=0.55, y=-0.11, z=0.398),
    Point(x=0.55, y=-0.13, z=0.395),
    Point(x=0.55, y=-0.13, z=0.420),
    Point(x=0.55, y=-0.15, z=0.395),
    Point(x=0.55, y=-0.13, z=0.420),
    Point(x=0.44, y=-0.13, z=0.405),
    Point(x=0.44, y=-0.15, z=0.405),
    Point(x=0.44, y=-0.13, z=0.46),
    Point(x=0.44, y=-0.13, z=0.405),
    Point(x=0.44, y=-0.13, z=0.46),
    Point(x=0.44, y=-0.13, z=0.46),
]

MASSAGE_POINTS_RIGHT = [
    Point(x=0.44, y=0.13, z=0.46),
    Point(x=0.44, y=0.13, z=0.405),
    Point(x=0.44, y=0.13, z=0.44),
    Point(x=0.44, y=0.13, z=0.46),
    Point(x=0.44, y=0.15, z=0.405),
    Point(x=0.44, y=0.15, z=0.405),
    Point(x=0.44, y=0.11, z=0.405),
    Point(x=0.44, y=0.13, z=0.46),
    Point(x=0.44, y=0.13, z=0.405),
    Point(x=0.55, y=0.13, z=0.420),
    Point(x=0.55, y=0.13, z=0.420),
    Point(x=0.55, y=0.15, z=0.395),
    Point(x=0.55, y=0.13, z=0.395),
    Point(x=0.55, y=0.13, z=0.420),
    Point(x=0.55, y=0.13, z=0.395),
    Point(x=0.55, y=0.15, z=0.398),
    Point(x=0.55, y=0.11, z=0.398),
    Point(x=0.55, y=0.13, z=0.420),
    Point(x=0.55, y=0.13, z=0.395),
    Point(x=0.55, y=0.15, z=0.395),
    Point(x=0.55, y=0.13, z=0.420),
    Point(x=0.44, y=0.13, z=0.405),
    Point(x=0.44, y=0.13, z=0.46),
    Point(x=0.44, y=0.15, z=0.405),
    Point(x=0.44, y=0.13, z=0.405),
    Point(x=0.44, y=0.13, z=0.46),
    Point(x=0.44, y=0.13, z=0.46),
]

assert len(MASSAGE_STAGE_NAMES) == len(LEFT_WAYPOINTS) == len(RIGHT_WAYPOINTS)
assert len(MASSAGE_POINTS_LEFT) == len(MASSAGE_POINTS_RIGHT) == len(MASSAGE_STAGE_NAMES)

BED_CENTER_X = 0.70
BED_CENTER_Y = 0.0
BED_FRAME_CENTER_Z = 0.24
BED_FRAME_SIZE_X = 1.20
BED_FRAME_SIZE_Y = 0.66
BED_FRAME_SIZE_Z = 0.08
MATTRESS_CENTER_Z = 0.31
MATTRESS_SIZE_X = 1.12
MATTRESS_SIZE_Y = 0.56
MATTRESS_SIZE_Z = 0.06
PILLOW_CENTER_X = 0.28
PILLOW_CENTER_Z = 0.375
PILLOW_SIZE_X = 0.20
PILLOW_SIZE_Y = 0.34
PILLOW_SIZE_Z = 0.07
PATIENT_BODY_CENTER_X = 0.58
PATIENT_BODY_CENTER_Z = 0.385
PATIENT_BODY_SIZE_X = 0.60
PATIENT_BODY_SIZE_Y = 0.30
PATIENT_BODY_SIZE_Z = 0.08
PATIENT_HEAD_X = 0.27
PATIENT_HEAD_Z = 0.435

SAMPLE_PERIOD = 0.10
DEFAULT_TRAJECTORY_START_DELAY = 0.10
DEFAULT_VELOCITY_SCALING = 0.45
DEFAULT_ACCELERATION_SCALING = 0.45
DEFAULT_TRAJECTORY_TIME_SCALE = 0.65
DEFAULT_EXACT_TARGET_SPEED = 0.50
DEFAULT_FALLBACK_JOINT_SPEED = 0.45
DEFAULT_MIN_SETTLE_DURATION = 0.10
PLANNING_GROUP = "both_arms"
PLANNER_ID = "RRTConnectkConfigDefault"


def duration_msg(seconds: float) -> Duration:
    whole_seconds = int(seconds)
    return Duration(
        sec=whole_seconds,
        nanosec=int((seconds - whole_seconds) * 1_000_000_000),
    )


def duration_seconds(duration: Duration) -> float:
    return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0


def shifted_duration(duration: Duration, offset: float) -> Duration:
    return duration_msg(duration_seconds(duration) + offset)


class DualArmMassageDemo(Node):
    def __init__(self):
        super().__init__("dual_arm_massage_demo")
        self.marker_topic = (
            self.declare_parameter("marker_topic", "/rviz_visual_tools")
            .get_parameter_value()
            .string_value
        )
        self.hold_seconds = (
            self.declare_parameter("hold_seconds", 8.0)
            .get_parameter_value()
            .double_value
        )
        self.trajectory_start_delay = (
            self.declare_parameter(
                "trajectory_start_delay",
                DEFAULT_TRAJECTORY_START_DELAY,
            )
            .get_parameter_value()
            .double_value
        )
        self.velocity_scaling = self._clamp(
            self.declare_parameter("velocity_scaling", DEFAULT_VELOCITY_SCALING)
            .get_parameter_value()
            .double_value,
            0.01,
            1.0,
        )
        self.acceleration_scaling = self._clamp(
            self.declare_parameter("acceleration_scaling", DEFAULT_ACCELERATION_SCALING)
            .get_parameter_value()
            .double_value,
            0.01,
            1.0,
        )
        self.trajectory_time_scale = self._clamp(
            self.declare_parameter("trajectory_time_scale", DEFAULT_TRAJECTORY_TIME_SCALE)
            .get_parameter_value()
            .double_value,
            0.25,
            1.0,
        )
        self.exact_target_speed = max(
            0.05,
            self.declare_parameter("exact_target_speed", DEFAULT_EXACT_TARGET_SPEED)
            .get_parameter_value()
            .double_value,
        )
        self.fallback_joint_speed = max(
            0.05,
            self.declare_parameter("fallback_joint_speed", DEFAULT_FALLBACK_JOINT_SPEED)
            .get_parameter_value()
            .double_value,
        )
        self.min_settle_duration = max(
            0.02,
            self.declare_parameter("min_settle_duration", DEFAULT_MIN_SETTLE_DURATION)
            .get_parameter_value()
            .double_value,
        )
        self.repeat_count = (
            self.declare_parameter("repeat_count", 0)
            .get_parameter_value()
            .integer_value
        )

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.motion_plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.state_validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.left_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/left_arm_controller/follow_joint_trajectory",
        )
        self.right_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/right_arm_controller/follow_joint_trajectory",
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        self.latest_joint_state = {}
        self.pending_results = 0
        self.completed_cycles = 0
        self.cycle_failed = False
        self.left_trajectory = None
        self.right_trajectory = None
        self.shutdown_requested = False
        self.total_duration = LEFT_WAYPOINTS[-1][0]
        self.marker_timer = self.create_timer(0.25, self.publish_markers)

    def _clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def run(self) -> bool:
        self.publish_markers()
        if not self._wait_for_services():
            return False

        start_positions = self._wait_for_start_positions(timeout_sec=30.0)
        if start_positions is None:
            return False

        if not self._apply_bed_collision():
            return False

        planned_trajectory = self._plan_massage_trajectory(start_positions)
        if planned_trajectory is None:
            return False

        self._scale_trajectory_timing(planned_trajectory)

        if not self._validate_planned_trajectory(planned_trajectory):
            return False

        self.left_trajectory, self.right_trajectory = self._split_combined_trajectory(
            planned_trajectory
        )
        self.total_duration = duration_seconds(self.left_trajectory.points[-1].time_from_start)
        self._send_next_cycle()
        return True

    def _send_next_cycle(self) -> None:
        self.pending_results = 2
        self.cycle_failed = False
        cycle_label = (
            f"{self.completed_cycles + 1}/{self.repeat_count}"
            if self.repeat_count > 0
            else f"{self.completed_cycles + 1}/infinite"
        )
        self.get_logger().info(
            f"Sending collision-aware synchronized massage cycle {cycle_label}."
        )
        self._send_goal(self.left_client, self.left_trajectory, "left arm")
        self._send_goal(self.right_client, self.right_trajectory, "right arm")

    def _wait_for_services(self) -> bool:
        for label, client in (
            ("apply_planning_scene", self.apply_scene_client),
            ("plan_kinematic_path", self.motion_plan_client),
            ("check_state_validity", self.state_validity_client),
        ):
            if not client.wait_for_service(timeout_sec=30.0):
                self.get_logger().error(f"Timed out waiting for service {label}.")
                return False

        for label, client in (
            ("left arm", self.left_client),
            ("right arm", self.right_client),
        ):
            if not client.wait_for_server(timeout_sec=60.0):
                self.get_logger().error(f"Timed out waiting for {label} trajectory action server.")
                return False
            self.get_logger().info(f"{label} trajectory action server is ready.")
        return True

    def _joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.latest_joint_state[name] = position

    def _wait_for_start_positions(self, timeout_sec: float):
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok():
            if all(joint in self.latest_joint_state for joint in ALL_JOINTS):
                return [self.latest_joint_state[joint] for joint in ALL_JOINTS]
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                self.get_logger().error("Timed out waiting for /joint_states.")
                return None
            self.publish_markers()
            rclpy.spin_once(self, timeout_sec=0.1)

    def _apply_bed_collision(self) -> bool:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "massage_bed_collision"
        collision_object.operation = CollisionObject.ADD

        self._add_box(
            collision_object,
            [BED_FRAME_SIZE_X, BED_FRAME_SIZE_Y, BED_FRAME_SIZE_Z],
            BED_CENTER_X,
            BED_CENTER_Y,
            BED_FRAME_CENTER_Z,
        )
        self._add_box(
            collision_object,
            [MATTRESS_SIZE_X, MATTRESS_SIZE_Y, MATTRESS_SIZE_Z],
            BED_CENTER_X,
            BED_CENTER_Y,
            MATTRESS_CENTER_Z,
        )
        self._add_box(
            collision_object,
            [PILLOW_SIZE_X, PILLOW_SIZE_Y, PILLOW_SIZE_Z],
            PILLOW_CENTER_X,
            BED_CENTER_Y,
            PILLOW_CENTER_Z,
        )

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_object)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error("Failed to apply massage_bed_collision to MoveIt.")
            return False

        self.get_logger().info("Added massage bed collision geometry to MoveIt planning scene.")
        return True

    def _add_box(self, collision_object, dimensions, x, y, z) -> None:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)

    def _plan_massage_trajectory(self, start_positions):
        targets = [left[1] + right[1] for left, right in zip(LEFT_WAYPOINTS, RIGHT_WAYPOINTS)]
        combined = JointTrajectory()
        combined.joint_names = ALL_JOINTS
        first_point = JointTrajectoryPoint()
        first_point.positions = list(start_positions)
        first_point.time_from_start = duration_msg(0.0)
        combined.points.append(first_point)

        current_positions = list(start_positions)
        time_offset = 0.0
        for target_index, target_positions in enumerate(targets):
            stage_name = MASSAGE_STAGE_NAMES[target_index]
            label = f"{stage_name} ({target_index + 1}/{len(targets)})"
            max_delta = max(
                abs(current - target)
                for current, target in zip(current_positions, target_positions)
            )
            if max_delta <= 0.002:
                self.get_logger().info(f"Already at {label}; skipping zero-distance plan.")
                current_positions = list(target_positions)
                continue

            segment = self._plan_joint_segment(current_positions, target_positions, label)
            if segment is None:
                return None

            time_offset = self._append_segment(combined, segment, current_positions, time_offset)
            time_offset = self._append_exact_target_if_needed(
                combined,
                target_positions,
                time_offset,
            )
            current_positions = list(combined.points[-1].positions)

        self.get_logger().info(
            f"MoveIt planned collision-aware massage motion with "
            f"{len(combined.points)} points and duration {time_offset:.2f}s."
        )
        return combined

    def _scale_trajectory_timing(self, trajectory: JointTrajectory) -> None:
        if abs(self.trajectory_time_scale - 1.0) <= 1e-6:
            return

        old_duration = duration_seconds(trajectory.points[-1].time_from_start)
        for point in trajectory.points:
            scaled_time = duration_seconds(point.time_from_start) * self.trajectory_time_scale
            point.time_from_start = duration_msg(scaled_time)
        new_duration = duration_seconds(trajectory.points[-1].time_from_start)
        self.get_logger().info(
            f"Scaled massage trajectory timing from {old_duration:.2f}s "
            f"to {new_duration:.2f}s."
        )

    def _plan_joint_segment(self, start_positions, goal_positions, label: str):
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = PLANNING_GROUP
        motion_request.planner_id = PLANNER_ID
        motion_request.num_planning_attempts = 12
        motion_request.allowed_planning_time = 8.0
        motion_request.max_velocity_scaling_factor = self.velocity_scaling
        motion_request.max_acceleration_scaling_factor = self.acceleration_scaling

        motion_request.start_state.is_diff = True
        motion_request.start_state.joint_state = JointState()
        motion_request.start_state.joint_state.name = ALL_JOINTS
        motion_request.start_state.joint_state.position = start_positions

        goal_constraints = Constraints()
        for joint_name, joint_position in zip(ALL_JOINTS, goal_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_position
            joint_constraint.tolerance_above = 0.004
            joint_constraint.tolerance_below = 0.004
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        motion_request.goal_constraints.append(goal_constraints)

        future = self.motion_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=12.0)
        result = future.result()
        if result is None:
            self.get_logger().error(f"MoveIt planner did not respond for {label}.")
            return None

        response = result.motion_plan_response
        if response.error_code.val != 1:
            self.get_logger().error(
                f"MoveIt failed to plan {label}; error_code={response.error_code.val}."
            )
            return None

        trajectory = response.trajectory.joint_trajectory
        if not trajectory.points:
            self.get_logger().error(f"MoveIt returned an empty trajectory for {label}.")
            return None

        self.get_logger().info(
            f"MoveIt planned {label}: {len(trajectory.points)} points, "
            f"{response.planning_time:.3f}s planning time."
        )
        return trajectory

    def _append_segment(
        self,
        combined: JointTrajectory,
        segment: JointTrajectory,
        fallback_positions,
        time_offset: float,
    ) -> float:
        local_duration = duration_seconds(segment.points[-1].time_from_start)
        if local_duration <= 0.001:
            final_positions = self._positions_from_point(
                segment,
                segment.points[-1],
                fallback_positions,
            )
            max_delta = max(abs(a - b) for a, b in zip(fallback_positions, final_positions))
            local_duration = max(0.30, max_delta / self.fallback_joint_speed)

        appended = 0
        point_count = len(segment.points)
        for point_index, point in enumerate(segment.points):
            local_time = duration_seconds(point.time_from_start)
            if local_time <= 0.001 and point_count > 1:
                local_time = local_duration * point_index / (point_count - 1)
            if local_time <= 0.001 and combined.points:
                continue

            new_point = JointTrajectoryPoint()
            new_point.positions = self._positions_from_point(
                segment,
                point,
                fallback_positions,
            )
            new_point.time_from_start = duration_msg(time_offset + local_time)
            combined.points.append(new_point)
            appended += 1

        if appended == 0:
            new_point = JointTrajectoryPoint()
            new_point.positions = self._positions_from_point(
                segment,
                segment.points[-1],
                fallback_positions,
            )
            new_point.time_from_start = duration_msg(time_offset + local_duration)
            combined.points.append(new_point)

        return time_offset + local_duration

    def _append_exact_target_if_needed(
        self,
        combined: JointTrajectory,
        target_positions,
        time_offset: float,
    ) -> float:
        current_positions = list(combined.points[-1].positions)
        max_delta = max(abs(a - b) for a, b in zip(current_positions, target_positions))
        if max_delta <= 1e-5:
            return time_offset

        settle_duration = max(self.min_settle_duration, max_delta / self.exact_target_speed)
        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        point.time_from_start = duration_msg(time_offset + settle_duration)
        combined.points.append(point)
        return time_offset + settle_duration

    def _positions_from_point(self, trajectory: JointTrajectory, point, fallback_positions):
        position_by_name = {
            joint_name: fallback_positions[index]
            for index, joint_name in enumerate(ALL_JOINTS)
        }
        for joint_name, joint_position in zip(trajectory.joint_names, point.positions):
            position_by_name[joint_name] = joint_position
        return [position_by_name[joint_name] for joint_name in ALL_JOINTS]

    def _validate_planned_trajectory(self, trajectory: JointTrajectory) -> bool:
        total_time = duration_seconds(trajectory.points[-1].time_from_start)
        sample_count = max(int(total_time / SAMPLE_PERIOD) + 1, len(trajectory.points) - 1)
        for index in range(sample_count + 1):
            elapsed = total_time * index / sample_count
            positions = self._interpolate_positions(trajectory, elapsed)

            request = GetStateValidity.Request()
            request.group_name = PLANNING_GROUP
            request.robot_state.is_diff = True
            request.robot_state.joint_state = JointState()
            request.robot_state.joint_state.name = ALL_JOINTS
            request.robot_state.joint_state.position = positions

            future = self.state_validity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            result = future.result()
            if result is None:
                self.get_logger().error("MoveIt state validity service did not respond.")
                return False
            if not result.valid:
                contacts = ", ".join(
                    f"{contact.contact_body_1}<->{contact.contact_body_2}"
                    for contact in result.contacts[:6]
                )
                self.get_logger().error(
                    f"Collision check failed at t={elapsed:.2f}s "
                    f"(sample {index}/{sample_count}): {contacts or 'invalid state'}"
                )
                return False

        self.get_logger().info(
            f"MoveIt accepted {sample_count + 1} sampled states for massage motion."
        )
        return True

    def _interpolate_positions(self, trajectory: JointTrajectory, elapsed: float):
        if elapsed <= duration_seconds(trajectory.points[0].time_from_start):
            return list(trajectory.points[0].positions)

        for index in range(1, len(trajectory.points)):
            previous_point = trajectory.points[index - 1]
            next_point = trajectory.points[index]
            t0 = duration_seconds(previous_point.time_from_start)
            t1 = duration_seconds(next_point.time_from_start)
            if elapsed <= t1:
                if t1 <= t0:
                    return list(next_point.positions)
                ratio = (elapsed - t0) / (t1 - t0)
                return [
                    previous_point.positions[i]
                    + (next_point.positions[i] - previous_point.positions[i]) * ratio
                    for i in range(len(previous_point.positions))
                ]

        return list(trajectory.points[-1].positions)

    def _split_combined_trajectory(self, trajectory: JointTrajectory):
        left_trajectory = JointTrajectory()
        left_trajectory.joint_names = LEFT_JOINTS
        right_trajectory = JointTrajectory()
        right_trajectory.joint_names = RIGHT_JOINTS
        joint_index = {joint_name: index for index, joint_name in enumerate(trajectory.joint_names)}

        for point in trajectory.points:
            left_point = JointTrajectoryPoint()
            left_point.positions = [point.positions[joint_index[joint]] for joint in LEFT_JOINTS]
            left_point.time_from_start = shifted_duration(
                point.time_from_start,
                self.trajectory_start_delay,
            )
            left_trajectory.points.append(left_point)

            right_point = JointTrajectoryPoint()
            right_point.positions = [point.positions[joint_index[joint]] for joint in RIGHT_JOINTS]
            right_point.time_from_start = shifted_duration(
                point.time_from_start,
                self.trajectory_start_delay,
            )
            right_trajectory.points.append(right_point)

        return left_trajectory, right_trajectory

    def _send_goal(self, client: ActionClient, trajectory: JointTrajectory, label: str) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = duration_msg(0.8)
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._on_goal_response(done, label))

    def _on_goal_response(self, future, label: str) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{label} massage trajectory goal was rejected.")
            self._mark_result_done()
            return

        self.get_logger().info(f"{label} massage trajectory goal accepted.")
        goal_handle.get_result_async().add_done_callback(
            lambda done: self._on_result(done, label)
        )

    def _on_result(self, future, label: str) -> None:
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"{label} massage trajectory finished successfully.")
        else:
            self.cycle_failed = True
            self.get_logger().error(
                f"{label} massage trajectory failed with error code "
                f"{result.error_code}: {result.error_string}"
            )
        self._mark_result_done()

    def _mark_result_done(self) -> None:
        self.pending_results -= 1
        if self.pending_results == 0:
            self.completed_cycles += 1
            if self.cycle_failed:
                self.get_logger().error("Massage cycle failed; stopping loop.")
                self.create_timer(self.hold_seconds, self._shutdown_once)
                return

            if self.repeat_count == 0 or self.completed_cycles < self.repeat_count:
                self.get_logger().info(
                    f"Massage cycle {self.completed_cycles} complete; starting next loop."
                )
                self._send_next_cycle()
                return

            self.get_logger().info(
                f"Completed {self.completed_cycles} massage cycle(s); holding RViz bed scene."
            )
            self.create_timer(self.hold_seconds, self._shutdown_once)

    def _shutdown_once(self) -> None:
        if not self.shutdown_requested:
            self.shutdown_requested = True
            self.get_logger().info("Massage demo finished.")
            rclpy.shutdown()

    def publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        marker_array.markers.extend(
            [
                self._box_marker(
                    now,
                    1,
                    "bed",
                    Point(x=BED_CENTER_X, y=BED_CENTER_Y, z=BED_FRAME_CENTER_Z),
                    Point(x=BED_FRAME_SIZE_X, y=BED_FRAME_SIZE_Y, z=BED_FRAME_SIZE_Z),
                    (0.38, 0.27, 0.17, 1.0),
                ),
                self._box_marker(
                    now,
                    2,
                    "mattress",
                    Point(x=BED_CENTER_X, y=BED_CENTER_Y, z=MATTRESS_CENTER_Z),
                    Point(x=MATTRESS_SIZE_X, y=MATTRESS_SIZE_Y, z=MATTRESS_SIZE_Z),
                    (0.82, 0.88, 0.94, 1.0),
                ),
                self._box_marker(
                    now,
                    3,
                    "pillow",
                    Point(x=PILLOW_CENTER_X, y=BED_CENTER_Y, z=PILLOW_CENTER_Z),
                    Point(x=PILLOW_SIZE_X, y=PILLOW_SIZE_Y, z=PILLOW_SIZE_Z),
                    (0.88, 0.92, 0.98, 1.0),
                ),
                self._box_marker(
                    now,
                    4,
                    "patient_body",
                    Point(
                        x=PATIENT_BODY_CENTER_X,
                        y=BED_CENTER_Y,
                        z=PATIENT_BODY_CENTER_Z,
                    ),
                    Point(
                        x=PATIENT_BODY_SIZE_X,
                        y=PATIENT_BODY_SIZE_Y,
                        z=PATIENT_BODY_SIZE_Z,
                    ),
                    (0.24, 0.52, 0.76, 0.55),
                ),
                self._sphere_marker(
                    now,
                    5,
                    "patient_head",
                    Point(x=PATIENT_HEAD_X, y=BED_CENTER_Y, z=PATIENT_HEAD_Z),
                    0.11,
                    (0.24, 0.52, 0.76, 0.55),
                ),
                self._line_marker(now, 10, "left_massage_path", MASSAGE_POINTS_LEFT, (0.0, 0.75, 0.95, 1.0)),
                self._line_marker(now, 11, "right_massage_path", MASSAGE_POINTS_RIGHT, (0.95, 0.58, 0.20, 1.0)),
            ]
        )
        for index, point in enumerate(MASSAGE_POINTS_LEFT + MASSAGE_POINTS_RIGHT):
            marker_array.markers.append(
                self._sphere_marker(
                    now,
                    20 + index,
                    "massage_target",
                    point,
                    0.025,
                    (0.1, 0.9, 0.45, 0.9),
                )
            )

        self.marker_pub.publish(marker_array)

    def _base_marker(self, now, marker_id: int, namespace: str, marker_type: int):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = now
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _box_marker(self, now, marker_id: int, namespace: str, position: Point, scale: Point, color):
        marker = self._base_marker(now, marker_id, namespace, Marker.CUBE)
        marker.pose.position = position
        marker.scale.x = scale.x
        marker.scale.y = scale.y
        marker.scale.z = scale.z
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _sphere_marker(self, now, marker_id: int, namespace: str, position: Point, diameter: float, color):
        marker = self._base_marker(now, marker_id, namespace, Marker.SPHERE)
        marker.pose.position = position
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _line_marker(self, now, marker_id: int, namespace: str, points, color):
        marker = self._base_marker(now, marker_id, namespace, Marker.LINE_STRIP)
        marker.points = points
        marker.scale.x = 0.012
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker


def main():
    rclpy.init()
    node = DualArmMassageDemo()
    if not node.run():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
