#!/usr/bin/env python3
"""Gazebo Classic carry demo with a gravity-driven cargo box."""

from __future__ import annotations

import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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

LEFT_WAYPOINTS = [
    (1.0, [-0.767, 1.370, -1.969, 0.600, 2.374, 0.000]),
    (3.0, [-0.601, 1.112, -2.210, 1.098, 2.540, 0.000]),
    (4.5, [-0.381, 1.133, -2.255, 1.122, 2.761, 0.000]),
    (5.2, [-0.381, 1.321, -2.197, 0.877, 2.761, 0.000]),
    (5.9, [-0.381, 1.472, -2.093, 0.621, 2.761, 0.000]),
    (6.6, [-0.381, 1.571, -1.951, 0.380, 2.761, 0.000]),
    (7.3, [-0.381, 1.622, -1.775, 0.153, 2.761, 0.000]),
    (8.0, [-0.381, 1.632, -1.626, -0.006, 2.761, 0.000]),
    (9.0, [-0.272, 1.290, -1.240, -0.049, 2.869, 0.000]),
    (10.0, [-0.212, 0.874, -0.595, -0.279, 2.929, 0.000]),
    (11.5, [-0.212, 0.914, -0.838, -0.076, 2.929, 0.000]),
    (13.0, [-0.212, 0.914, -0.838, -0.076, 2.929, 0.000]),
    (14.0, [-0.212, 0.914, -0.838, -0.076, 2.929, 0.000]),
    (15.5, [-0.417, 0.850, -0.727, -0.123, 2.725, 0.000]),
]

RIGHT_WAYPOINTS = [
    (1.0, [0.297, 1.326, -1.912, 0.586, 0.297, 0.000]),
    (3.0, [0.096, 1.093, -2.169, 1.077, 0.096, 0.000]),
    (4.5, [-0.148, 1.126, -2.241, 1.114, -0.148, 0.001]),
    (5.2, [-0.148, 1.312, -2.184, 0.873, -0.148, 0.000]),
    (5.9, [-0.148, 1.461, -2.081, 0.620, -0.148, 0.000]),
    (6.6, [-0.148, 1.560, -1.940, 0.380, -0.148, 0.000]),
    (7.3, [-0.148, 1.611, -1.764, 0.154, -0.148, 0.000]),
    (8.0, [-0.148, 1.621, -1.616, -0.005, -0.148, 0.000]),
    (9.0, [-0.105, 1.281, -1.229, -0.052, -0.105, -0.001]),
    (10.0, [-0.082, 0.865, -0.578, -0.284, -0.082, -0.003]),
    (11.5, [-0.082, 0.905, -0.823, -0.083, -0.082, 0.001]),
    (13.0, [-0.082, 0.905, -0.823, -0.082, -0.082, 0.000]),
    (14.0, [-0.082, 0.905, -0.823, -0.082, -0.082, 0.000]),
    (15.5, [0.131, 0.810, -0.655, -0.155, 0.131, 0.000]),
]

JOINT_ORIGINS = [
    ((0.0, -0.00022535, 0.12015), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (1.5708, 0.0, 0.0)),
    ((0.43, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((0.3685, -0.00001185, -0.114), (0.0, 0.0, 0.0)),
    ((0.0, -0.1135, 0.0), (1.5708, 0.0, 0.0)),
    ((0.0, 0.107, 0.0), (-1.5708, 0.0, 0.0)),
]

CARGO_SIZE_X = 0.18
CARGO_SIZE_Y = 0.345
CARGO_SIZE_Z = 0.12
LEFT_GRIP_CONTACT_Z = 0.012
RIGHT_GRIP_CONTACT_Z = 0.012
LEFT_CONTACT_LOCAL_OFFSET = (0.0, 0.0, LEFT_GRIP_CONTACT_Z)
RIGHT_CONTACT_LOCAL_OFFSET = (0.0, 0.0, RIGHT_GRIP_CONTACT_Z)
GRIP_CAPTURE_RADIUS = 0.16
GRIP_DISTANCE_MIN = 0.335
GRIP_CAPTURE_DISTANCE_MAX = 0.350
GRIP_DISTANCE_MAX = 0.365
SAMPLE_PERIOD = 0.1
TRAJECTORY_START_DELAY = 0.5
GAZEBO_MODEL_NAME = "jaka_c5_dual"
GAZEBO_SYNC_PERIOD = 0.10
PICK_X = 0.36
CARGO_INITIAL_Y = 0.02
TABLE_X = 0.90
TABLE_Y = 0.0
TABLE_CENTER_Z = 0.28
TABLE_SIZE_X = 0.75
TABLE_SIZE_Y = 0.70
TABLE_SIZE_Z = 0.04
TABLE_TOP_Z = TABLE_CENTER_Z + TABLE_SIZE_Z / 2.0
TABLE_LEG_SIZE = 0.04
TABLE_LEG_HEIGHT = TABLE_TOP_Z - TABLE_SIZE_Z
PLANNING_GROUP = "both_arms"
PLANNER_ID = "RRTConnectkConfigDefault"
GRASP_STAGE_INDEX = 2
RELEASE_STAGE_INDEX = len(LEFT_WAYPOINTS) - 1


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


def add(a: Point, b: Point) -> Point:
    return Point(x=a.x + b.x, y=a.y + b.y, z=a.z + b.z)


def subtract(a: Point, b: Point) -> Point:
    return Point(x=a.x - b.x, y=a.y - b.y, z=a.z - b.z)


def scale(a: Point, factor: float) -> Point:
    return Point(x=a.x * factor, y=a.y * factor, z=a.z * factor)


def dot(a: Point, b: Point) -> float:
    return a.x * b.x + a.y * b.y + a.z * b.z


def cross(a: Point, b: Point) -> Point:
    return Point(
        x=a.y * b.z - a.z * b.y,
        y=a.z * b.x - a.x * b.z,
        z=a.x * b.y - a.y * b.x,
    )


def norm(a: Point) -> float:
    return (a.x * a.x + a.y * a.y + a.z * a.z) ** 0.5


def normalize(a: Point, fallback: Point) -> Point:
    length = norm(a)
    if length < 1e-6:
        return fallback
    return scale(a, 1.0 / length)


def matmul(a, b):
    return [[sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4)] for row in range(4)]


def transform_matrix(xyz, rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def z_rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def fk_transform(joint_positions, base_y):
    transform = transform_matrix((0.0, base_y, 0.0), (0.0, 0.0, 0.0))
    for (xyz, rpy), joint_position in zip(JOINT_ORIGINS, joint_positions):
        transform = matmul(transform, transform_matrix(xyz, rpy))
        transform = matmul(transform, z_rotation(joint_position))
    return transform


def fk_link_transforms(joint_positions, base_y):
    transform = transform_matrix((0.0, base_y, 0.0), (0.0, 0.0, 0.0))
    transforms = [transform]
    for (xyz, rpy), joint_position in zip(JOINT_ORIGINS, joint_positions):
        transform = matmul(transform, transform_matrix(xyz, rpy))
        transform = matmul(transform, z_rotation(joint_position))
        transforms.append(transform)
    return transforms


def point_from_matrix(
    transform,
    local_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Point:
    return Point(
        x=transform[0][3] + sum(transform[0][index] * local_offset[index] for index in range(3)),
        y=transform[1][3] + sum(transform[1][index] * local_offset[index] for index in range(3)),
        z=transform[2][3] + sum(transform[2][index] * local_offset[index] for index in range(3)),
    )


def fk_tip(
    joint_positions,
    base_y,
    local_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    transform = fk_transform(joint_positions, base_y)
    return point_from_matrix(transform, local_offset)


def rigid_inverse(transform):
    inverse = [
        [transform[0][0], transform[1][0], transform[2][0], 0.0],
        [transform[0][1], transform[1][1], transform[2][1], 0.0],
        [transform[0][2], transform[1][2], transform[2][2], 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    translation = (transform[0][3], transform[1][3], transform[2][3])
    for row in range(3):
        inverse[row][3] = -sum(inverse[row][col] * translation[col] for col in range(3))
    return inverse


def matrix_from_axes(origin: Point, x_axis: Point, y_axis: Point, z_axis: Point):
    return [
        [x_axis.x, y_axis.x, z_axis.x, origin.x],
        [x_axis.y, y_axis.y, z_axis.y, origin.y],
        [x_axis.z, y_axis.z, z_axis.z, origin.z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def pose_from_matrix(transform) -> Pose:
    x_axis = Point(x=transform[0][0], y=transform[1][0], z=transform[2][0])
    y_axis = Point(x=transform[0][1], y=transform[1][1], z=transform[2][1])
    z_axis = Point(x=transform[0][2], y=transform[1][2], z=transform[2][2])
    quaternion = quaternion_from_axes(x_axis, y_axis, z_axis)

    pose = Pose()
    pose.position.x = transform[0][3]
    pose.position.y = transform[1][3]
    pose.position.z = transform[2][3]
    pose.orientation = quaternion
    return pose


def cargo_matrix_from_tips(left_tip: Point, right_tip: Point):
    midpoint = scale(add(left_tip, right_tip), 0.5)
    y_axis = normalize(subtract(right_tip, left_tip), Point(x=0.0, y=1.0, z=0.0))
    up = Point(x=0.0, y=0.0, z=1.0)
    if abs(dot(y_axis, up)) > 0.92:
        up = Point(x=1.0, y=0.0, z=0.0)
    x_axis = normalize(cross(y_axis, up), Point(x=1.0, y=0.0, z=0.0))
    z_axis = normalize(cross(x_axis, y_axis), Point(x=0.0, y=0.0, z=1.0))
    return matrix_from_axes(midpoint, x_axis, y_axis, z_axis)


def quaternion_from_axes(x_axis: Point, y_axis: Point, z_axis: Point) -> Quaternion:
    m00, m01, m02 = x_axis.x, y_axis.x, z_axis.x
    m10, m11, m12 = x_axis.y, y_axis.y, z_axis.y
    m20, m21, m22 = x_axis.z, y_axis.z, z_axis.z
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = (trace + 1.0) ** 0.5 * 2.0
        return Quaternion(x=(m21 - m12) / s, y=(m02 - m20) / s, z=(m10 - m01) / s, w=0.25 * s)
    if m00 > m11 and m00 > m22:
        s = (1.0 + m00 - m11 - m22) ** 0.5 * 2.0
        return Quaternion(x=0.25 * s, y=(m01 + m10) / s, z=(m02 + m20) / s, w=(m21 - m12) / s)
    if m11 > m22:
        s = (1.0 + m11 - m00 - m22) ** 0.5 * 2.0
        return Quaternion(x=(m01 + m10) / s, y=0.25 * s, z=(m12 + m21) / s, w=(m02 - m20) / s)
    s = (1.0 + m22 - m00 - m11) ** 0.5 * 2.0
    return Quaternion(x=(m02 + m20) / s, y=(m12 + m21) / s, z=0.25 * s, w=(m10 - m01) / s)


def cargo_pose_from_tips(left_tip: Point, right_tip: Point):
    center = scale(add(left_tip, right_tip), 0.5)
    y_axis = normalize(subtract(right_tip, left_tip), Point(x=0.0, y=1.0, z=0.0))
    up = Point(x=0.0, y=0.0, z=1.0)
    x_axis = normalize(cross(y_axis, up), Point(x=1.0, y=0.0, z=0.0))
    z_axis = normalize(cross(x_axis, y_axis), Point(x=0.0, y=0.0, z=1.0))
    return {
        "center": center,
        "orientation": quaternion_from_axes(x_axis, y_axis, z_axis),
        "tip_distance": norm(subtract(right_tip, left_tip)),
    }


class GazeboCarryDemo(Node):
    def __init__(self):
        super().__init__("gazebo_carry_demo")
        self.entity_state_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.state_validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.motion_plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
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
        self.start_time = None
        self.cargo_center = Point(x=PICK_X, y=CARGO_INITIAL_Y, z=CARGO_SIZE_Z / 2.0)
        self.cargo_state = "free"
        self.release_logged = False
        self.samples = []
        self.total_duration = LEFT_WAYPOINTS[-1][0]
        self.stage_end_times = {0: 0.0}
        self.grasp_enable_time = TRAJECTORY_START_DELAY
        self.left_trajectory = None
        self.right_trajectory = None
        self.last_gazebo_sync_time = -GAZEBO_SYNC_PERIOD
        self.timer = None

    def wait_for_services(self) -> bool:
        for label, client in (
            ("set_entity_state", self.entity_state_client),
            ("check_state_validity", self.state_validity_client),
            ("apply_planning_scene", self.apply_scene_client),
            ("plan_kinematic_path", self.motion_plan_client),
        ):
            if not client.wait_for_service(timeout_sec=90.0):
                self.get_logger().error(f"Timed out waiting for service {label}")
                return False
        for label, client in (
            ("left_arm_controller/follow_joint_trajectory", self.left_client),
            ("right_arm_controller/follow_joint_trajectory", self.right_client),
        ):
            if not client.wait_for_server(timeout_sec=90.0):
                self.get_logger().error(f"Timed out waiting for action {label}")
                return False
        return True

    def prepare_demo(self) -> bool:
        if not self._apply_table_to_planning_scene():
            return False

        planned_trajectory = self._plan_carry_trajectory_with_moveit()
        if planned_trajectory is None:
            return False

        self.samples = self._build_samples(planned_trajectory)
        if not self._validate_samples_with_moveit():
            return False
        self.get_logger().info(
            f"MoveIt accepted {len(self.samples)} sampled states from the planned trajectory."
        )
        self.left_trajectory, self.right_trajectory = self._split_combined_trajectory(
            planned_trajectory
        )
        self.total_duration = duration_seconds(
            self.left_trajectory.points[-1].time_from_start
        )
        self.grasp_enable_time = (
            self.stage_end_times.get(GRASP_STAGE_INDEX, 0.0)
            + TRAJECTORY_START_DELAY
            + 0.10
        )

        self._send_trajectory(self.left_client, self.left_trajectory, "left arm")
        self._send_trajectory(self.right_client, self.right_trajectory, "right arm")
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.on_timer)
        return True

    def elapsed(self) -> float:
        if self.start_time is None:
            return 0.0
        return (self.get_clock().now() - self.start_time).nanoseconds / 1_000_000_000.0

    def _build_samples(self, trajectory: JointTrajectory):
        samples = []
        total_time = duration_seconds(trajectory.points[-1].time_from_start)
        sample_count = int(total_time / SAMPLE_PERIOD) + 1
        for index in range(sample_count + 1):
            elapsed = min(index * SAMPLE_PERIOD, total_time)
            positions = self._interpolate_planned_positions(trajectory, elapsed)
            left_positions = positions[: len(LEFT_JOINTS)]
            right_positions = positions[len(LEFT_JOINTS):]
            left_tip = fk_tip(left_positions, -0.25, LEFT_CONTACT_LOCAL_OFFSET)
            right_tip = fk_tip(right_positions, 0.25, RIGHT_CONTACT_LOCAL_OFFSET)
            tip_pose = cargo_pose_from_tips(left_tip, right_tip)
            samples.append(
                {
                    "time": elapsed,
                    "left": left_positions,
                    "right": right_positions,
                    "left_tip": left_tip,
                    "right_tip": right_tip,
                    "tip_pose": tip_pose,
                    "gazebo_link_poses": self._build_gazebo_link_poses(
                        left_positions,
                        right_positions,
                    ),
                }
            )
        return samples

    def on_timer(self):
        elapsed = self.elapsed()
        sample_index = min(int(elapsed / SAMPLE_PERIOD), len(self.samples) - 1)
        sample = self.samples[sample_index]
        tip_pose = sample["tip_pose"]
        self._sync_gazebo_robot(sample, elapsed)

        should_release = elapsed >= self.total_duration + 0.8
        grip_open = tip_pose["tip_distance"] > GRIP_DISTANCE_MAX
        grasp_ready = elapsed >= self.grasp_enable_time

        if self.cargo_state == "grasped" and (should_release or grip_open):
            self.cargo_state = "free"
            if not self.release_logged:
                self.release_logged = True
                if grip_open:
                    self.get_logger().info("Released cargo_box by opening both arm tips.")
                else:
                    self.get_logger().info("Released cargo_box; Gazebo gravity now owns it.")
            return

        if (
            self.cargo_state == "free"
            and grasp_ready
            and not should_release
            and self._can_grasp(tip_pose)
        ):
            self.cargo_state = "grasped"
            self.get_logger().info(
                "Constrained cargo_box to both arm contact patches "
                f"(contact distance {tip_pose['tip_distance']:.3f} m, "
                f"cargo width {CARGO_SIZE_Y:.3f} m)."
            )

        if self.cargo_state == "grasped":
            self.cargo_center = tip_pose["center"]
            self._set_cargo_state(tip_pose)

        if elapsed > self.total_duration + 7.0:
            self.get_logger().info("Gazebo carry demo finished.")
            rclpy.shutdown()

    def _apply_table_to_planning_scene(self) -> bool:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "work_table_collision"
        collision_object.operation = CollisionObject.ADD

        def add_box(dimensions, x, y, z):
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

        add_box([TABLE_SIZE_X, TABLE_SIZE_Y, TABLE_SIZE_Z], TABLE_X, TABLE_Y, TABLE_CENTER_Z)

        leg_x_offset = TABLE_SIZE_X / 2.0 - TABLE_LEG_SIZE
        leg_y_offset = TABLE_SIZE_Y / 2.0 - TABLE_LEG_SIZE
        leg_z = TABLE_LEG_HEIGHT / 2.0
        for x_offset in (-leg_x_offset, leg_x_offset):
            for y_offset in (-leg_y_offset, leg_y_offset):
                add_box(
                    [TABLE_LEG_SIZE, TABLE_LEG_SIZE, TABLE_LEG_HEIGHT],
                    TABLE_X + x_offset,
                    TABLE_Y + y_offset,
                    leg_z,
                )

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_object)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None or not future.result().success:
            self.get_logger().error("Failed to apply work_table_collision to MoveIt planning scene.")
            return False

        self.get_logger().info("Added work_table_collision to MoveIt planning scene.")
        return True

    def _plan_carry_trajectory_with_moveit(self):
        joint_names = LEFT_JOINTS + RIGHT_JOINTS
        targets = [left[1] + right[1] for left, right in zip(LEFT_WAYPOINTS, RIGHT_WAYPOINTS)]
        self.stage_end_times = {0: 0.0}

        combined = JointTrajectory()
        combined.joint_names = joint_names
        first_point = JointTrajectoryPoint()
        first_point.positions = targets[0]
        first_point.time_from_start = duration_msg(0.0)
        combined.points.append(first_point)

        start_positions = targets[0]
        time_offset = 0.0
        for target_index in range(1, len(targets)):
            label = f"stage {target_index}/{len(targets) - 1}"
            if GRASP_STAGE_INDEX < target_index < RELEASE_STAGE_INDEX:
                segment = self._make_locked_grip_segment(
                    start_positions,
                    targets[target_index],
                    label,
                )
            else:
                segment = self._plan_joint_segment(
                    start_positions,
                    targets[target_index],
                    label,
                )
            if segment is None:
                return None

            time_offset = self._append_segment(combined, segment, start_positions, time_offset)
            time_offset = self._append_exact_target_if_needed(
                combined,
                targets[target_index],
                time_offset,
            )
            self.stage_end_times[target_index] = time_offset
            start_positions = list(combined.points[-1].positions)

        self.get_logger().info(
            f"MoveIt planned a collision-aware {PLANNING_GROUP} trajectory with "
            f"{len(combined.points)} points and duration {time_offset:.2f}s."
        )
        return combined

    def _make_locked_grip_segment(self, start_positions, goal_positions, label: str):
        max_delta = max(abs(a - b) for a, b in zip(start_positions, goal_positions))
        local_duration = max(1.0, max_delta / 0.18)
        point_count = max(2, int(local_duration / 0.08) + 1)

        trajectory = JointTrajectory()
        trajectory.joint_names = LEFT_JOINTS + RIGHT_JOINTS
        for point_index in range(point_count):
            ratio = point_index / (point_count - 1)
            point = JointTrajectoryPoint()
            point.positions = [
                start_positions[i] + (goal_positions[i] - start_positions[i]) * ratio
                for i in range(len(start_positions))
            ]
            point.time_from_start = duration_msg(local_duration * ratio)
            trajectory.points.append(point)

        self.get_logger().info(
            f"Using locked-grip synchronized interpolation for {label}: "
            f"{point_count} points."
        )
        return trajectory

    def _append_exact_target_if_needed(self, combined, target_positions, time_offset: float):
        current_positions = list(combined.points[-1].positions)
        max_delta = max(abs(a - b) for a, b in zip(current_positions, target_positions))
        if max_delta <= 1e-5:
            return time_offset

        settle_duration = max(0.25, max_delta / 0.12)
        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        point.time_from_start = duration_msg(time_offset + settle_duration)
        combined.points.append(point)
        return time_offset + settle_duration

    def _plan_joint_segment(self, start_positions, goal_positions, label: str):
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = PLANNING_GROUP
        motion_request.planner_id = PLANNER_ID
        motion_request.num_planning_attempts = 10
        motion_request.allowed_planning_time = 8.0
        motion_request.max_velocity_scaling_factor = 0.15
        motion_request.max_acceleration_scaling_factor = 0.15

        motion_request.start_state.is_diff = True
        motion_request.start_state.joint_state = JointState()
        motion_request.start_state.joint_state.name = LEFT_JOINTS + RIGHT_JOINTS
        motion_request.start_state.joint_state.position = start_positions

        goal_constraints = Constraints()
        for joint_name, joint_position in zip(LEFT_JOINTS + RIGHT_JOINTS, goal_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_position
            joint_constraint.tolerance_above = 0.003
            joint_constraint.tolerance_below = 0.003
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

    def _append_segment(self, combined, segment, fallback_positions, time_offset: float) -> float:
        local_duration = duration_seconds(segment.points[-1].time_from_start)
        if local_duration <= 0.001:
            final_positions = self._positions_from_point(
                segment,
                segment.points[-1],
                fallback_positions,
            )
            max_delta = max(abs(a - b) for a, b in zip(fallback_positions, final_positions))
            local_duration = max(1.0, max_delta / 0.2)

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

    def _positions_from_point(self, trajectory, point, fallback_positions):
        position_by_name = {
            joint_name: fallback_positions[index]
            for index, joint_name in enumerate(LEFT_JOINTS + RIGHT_JOINTS)
        }
        for joint_name, joint_position in zip(trajectory.joint_names, point.positions):
            position_by_name[joint_name] = joint_position
        return [position_by_name[joint_name] for joint_name in LEFT_JOINTS + RIGHT_JOINTS]

    def _validate_samples_with_moveit(self) -> bool:
        for index, sample in enumerate(self.samples):
            request = GetStateValidity.Request()
            request.group_name = PLANNING_GROUP
            request.robot_state.is_diff = True
            request.robot_state.joint_state = JointState()
            request.robot_state.joint_state.name = LEFT_JOINTS + RIGHT_JOINTS
            request.robot_state.joint_state.position = sample["left"] + sample["right"]

            future = self.state_validity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            result = future.result()
            if result is None:
                self.get_logger().error("MoveIt state validity service did not respond.")
                return False
            if not result.valid:
                contacts = ", ".join(
                    f"{contact.contact_body_1}<->{contact.contact_body_2}"
                    for contact in result.contacts[:4]
                )
                self.get_logger().error(
                    f"Collision check failed at t={sample['time']:.2f}s "
                    f"(sample {index}/{len(self.samples)}): {contacts or 'invalid state'}"
                )
                return False

            if not self._validate_cargo_table_clearance(sample, index):
                return False
        return True

    def _validate_cargo_table_clearance(self, sample, sample_index) -> bool:
        center = sample["tip_pose"]["center"]
        bottom_z = center.z - CARGO_SIZE_Z / 2.0
        if not self._point_over_table(center):
            return True

        min_bottom_z = TABLE_TOP_Z - 0.005
        if bottom_z < min_bottom_z:
            self.get_logger().error(
                f"Cargo penetrates table at t={sample['time']:.2f}s "
                f"(sample {sample_index}/{len(self.samples)}): bottom_z={bottom_z:.3f}, "
                f"table_top_z={TABLE_TOP_Z:.3f}."
            )
            return False

        return True

    def _point_over_table(self, point: Point) -> bool:
        half_x = TABLE_SIZE_X / 2.0 + CARGO_SIZE_X / 2.0
        half_y = TABLE_SIZE_Y / 2.0 + CARGO_SIZE_Y / 2.0
        return (
            abs(point.x - TABLE_X) <= half_x
            and abs(point.y - TABLE_Y) <= half_y
        )

    def _can_grasp(self, tip_pose) -> bool:
        tip_distance = tip_pose["tip_distance"]
        center_error = norm(subtract(tip_pose["center"], self.cargo_center))
        return (
            GRIP_DISTANCE_MIN <= tip_distance <= GRIP_CAPTURE_DISTANCE_MAX
            and center_error <= GRIP_CAPTURE_RADIUS
            and tip_pose["center"].z >= CARGO_SIZE_Z * 0.45
        )

    def _interpolate_planned_positions(self, trajectory: JointTrajectory, elapsed: float):
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

        joint_index = {
            joint_name: index
            for index, joint_name in enumerate(trajectory.joint_names)
        }

        for point in trajectory.points:
            left_point = JointTrajectoryPoint()
            left_point.positions = [
                point.positions[joint_index[joint_name]]
                for joint_name in LEFT_JOINTS
            ]
            left_point.time_from_start = shifted_duration(
                point.time_from_start,
                TRAJECTORY_START_DELAY,
            )
            left_trajectory.points.append(left_point)

            right_point = JointTrajectoryPoint()
            right_point.positions = [
                point.positions[joint_index[joint_name]]
                for joint_name in RIGHT_JOINTS
            ]
            right_point.time_from_start = shifted_duration(
                point.time_from_start,
                TRAJECTORY_START_DELAY,
            )
            right_trajectory.points.append(right_point)

        return left_trajectory, right_trajectory

    def _send_trajectory(self, client, trajectory, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = duration_msg(0.5)
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._on_goal_response(done, label))

    def _on_goal_response(self, future, label):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{label} trajectory goal was rejected.")
            return
        self.get_logger().info(f"{label} trajectory goal accepted by Gazebo controller.")
        goal_handle.get_result_async().add_done_callback(
            lambda done: self._on_goal_result(done, label)
        )

    def _on_goal_result(self, future, label):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"{label} Gazebo trajectory finished successfully.")
        else:
            self.get_logger().error(
                f"{label} Gazebo trajectory failed with error code {result.error_code}: "
                f"{result.error_string}"
            )

    def _set_cargo_state(self, tip_pose):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = "cargo_box"
        request.state.reference_frame = "world"
        request.state.pose.position = tip_pose["center"]
        request.state.pose.orientation = tip_pose["orientation"]
        self.entity_state_client.call_async(request)

    def _build_gazebo_link_poses(self, left_positions, right_positions):
        poses = []
        for prefix, positions, base_y in (
            ("left_", left_positions, -0.25),
            ("right_", right_positions, 0.25),
        ):
            transforms = fk_link_transforms(positions, base_y)
            for index, transform in enumerate(transforms):
                poses.append((f"{prefix}Link_0{index}", pose_from_matrix(transform)))
        return poses

    def _sync_gazebo_robot(self, sample, elapsed: float):
        if elapsed - self.last_gazebo_sync_time < GAZEBO_SYNC_PERIOD:
            return
        self.last_gazebo_sync_time = elapsed

        for link_name, pose in sample["gazebo_link_poses"]:
            request = SetEntityState.Request()
            request.state = EntityState()
            request.state.name = f"{GAZEBO_MODEL_NAME}::{link_name}"
            request.state.reference_frame = "world"
            request.state.pose = pose
            self.entity_state_client.call_async(request)


def main():
    rclpy.init()
    node = GazeboCarryDemo()
    if not node.wait_for_services() or not node.prepare_demo():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
