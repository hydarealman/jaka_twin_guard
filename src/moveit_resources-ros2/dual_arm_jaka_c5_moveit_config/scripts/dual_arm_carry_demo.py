#!/usr/bin/env python3
"""RViz cooperative carry demo for the dual JAKA C5 mock controllers."""

from __future__ import annotations

import sys
import math

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener
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
LEFT_CONTACT_FRAME = "left_grip_contact"
RIGHT_CONTACT_FRAME = "right_grip_contact"
GROUND_Z = 0.0
PICK_X = 0.36
CARGO_INITIAL_Y = 0.02
TABLE_X = 0.90
TABLE_Y = 0.0
TABLE_TOP_Z = 0.30
TABLE_SIZE_X = 0.75
TABLE_SIZE_Y = 0.70
TABLE_SIZE_Z = 0.04
TABLE_CENTER_Z = TABLE_TOP_Z - TABLE_SIZE_Z / 2.0
TABLE_LEG_SIZE = 0.04
TABLE_LEG_HEIGHT = TABLE_TOP_Z - TABLE_SIZE_Z
TABLE_SUPPORT_TOLERANCE = 0.08
SAMPLE_PERIOD = 0.1
TRAJECTORY_START_DELAY = 0.5
PLANNING_GROUP = "both_arms"
PLANNER_ID = "RRTConnectkConfigDefault"
GRASP_STAGE_INDEX = 2
RELEASE_STAGE_INDEX = len(LEFT_WAYPOINTS) - 1
ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS
STARTUP_STABLE_SECONDS = 0.6
STARTUP_STABLE_DELTA = 0.002
STARTUP_START_POSE_TOLERANCE = 0.04
GRIP_FACE_ALIGNMENT_MIN = 0.90
GRIP_PAD_UP_ALIGNMENT_MIN = 0.85
GRIP_DISTANCE_VALIDATE_MIN = CARGO_SIZE_Y - 0.020
GRIP_DISTANCE_VALIDATE_MAX = CARGO_SIZE_Y + 0.025


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


def quaternion_from_axes(x_axis: Point, y_axis: Point, z_axis: Point):
    m00, m01, m02 = x_axis.x, y_axis.x, z_axis.x
    m10, m11, m12 = x_axis.y, y_axis.y, z_axis.y
    m20, m21, m22 = x_axis.z, y_axis.z, z_axis.z

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = (trace + 1.0) ** 0.5 * 2.0
        return (
            (m21 - m12) / s,
            (m02 - m20) / s,
            (m10 - m01) / s,
            0.25 * s,
        )
    if m00 > m11 and m00 > m22:
        s = (1.0 + m00 - m11 - m22) ** 0.5 * 2.0
        return (
            0.25 * s,
            (m01 + m10) / s,
            (m02 + m20) / s,
            (m21 - m12) / s,
        )
    if m11 > m22:
        s = (1.0 + m11 - m00 - m22) ** 0.5 * 2.0
        return (
            (m01 + m10) / s,
            0.25 * s,
            (m12 + m21) / s,
            (m02 - m20) / s,
        )

    s = (1.0 + m22 - m00 - m11) ** 0.5 * 2.0
    return (
        (m02 + m20) / s,
        (m12 + m21) / s,
        0.25 * s,
        (m10 - m01) / s,
    )


def rotate_vector(quaternion, vector: tuple[float, float, float]) -> tuple[float, float, float]:
    q_vector = Point(x=quaternion.x, y=quaternion.y, z=quaternion.z)
    v = Point(x=vector[0], y=vector[1], z=vector[2])
    uv = cross(q_vector, v)
    uuv = cross(q_vector, uv)
    rotated = add(v, scale(add(scale(uv, quaternion.w), uuv), 2.0))
    return rotated.x, rotated.y, rotated.z


def point_from_transform(
    transform,
    local_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Point:
    translation = transform.transform.translation
    offset = rotate_vector(transform.transform.rotation, local_offset)
    return Point(
        x=translation.x + offset[0],
        y=translation.y + offset[1],
        z=translation.z + offset[2],
    )


def matmul(a, b):
    return [
        [
            sum(a[row][k] * b[k][col] for k in range(4))
            for col in range(4)
        ]
        for row in range(4)
    ]


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


def point_from_matrix(
    transform,
    local_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Point:
    return Point(
        x=transform[0][3] + sum(transform[0][index] * local_offset[index] for index in range(3)),
        y=transform[1][3] + sum(transform[1][index] * local_offset[index] for index in range(3)),
        z=transform[2][3] + sum(transform[2][index] * local_offset[index] for index in range(3)),
    )


def vector_from_matrix(transform, local_vector: tuple[float, float, float]) -> Point:
    return Point(
        x=sum(transform[0][index] * local_vector[index] for index in range(3)),
        y=sum(transform[1][index] * local_vector[index] for index in range(3)),
        z=sum(transform[2][index] * local_vector[index] for index in range(3)),
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
    qx, qy, qz, qw = quaternion_from_axes(x_axis, y_axis, z_axis)

    pose = Pose()
    pose.position.x = transform[0][3]
    pose.position.y = transform[1][3]
    pose.position.z = transform[2][3]
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def cargo_matrix_from_tips(left_tip: Point, right_tip: Point):
    midpoint = scale(add(left_tip, right_tip), 0.5)
    y_axis = normalize(subtract(right_tip, left_tip), Point(x=0.0, y=1.0, z=0.0))
    world_up = Point(x=0.0, y=0.0, z=1.0)
    if abs(dot(y_axis, world_up)) > 0.92:
        world_up = Point(x=1.0, y=0.0, z=0.0)

    x_axis = normalize(cross(y_axis, world_up), Point(x=1.0, y=0.0, z=0.0))
    z_axis = normalize(cross(x_axis, y_axis), Point(x=0.0, y=0.0, z=1.0))
    return matrix_from_axes(midpoint, x_axis, y_axis, z_axis)


class DualArmCarryDemo(Node):
    def __init__(self) -> None:
        super().__init__("dual_arm_carry_demo")

        self.declare_parameter("marker_topic", "/rviz_visual_tools")
        self.declare_parameter("action_server_timeout_sec", 30.0)
        self.declare_parameter("hold_seconds", 8.0)
        self.declare_parameter("gravity", 9.81)
        self.declare_parameter("release_after_motion", True)
        self.declare_parameter("release_delay_seconds", 0.8)
        self.declare_parameter("grip_capture_radius", 0.16)
        self.declare_parameter("grip_distance_min", 0.335)
        self.declare_parameter("grip_capture_distance_max", 0.350)
        self.declare_parameter("grip_distance_max", 0.365)
        self.declare_parameter("startup_state_timeout_sec", 20.0)

        marker_topic = self.get_parameter("marker_topic").value
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
        )
        self.tf_buffer = Buffer() # 缓存所有坐标系TF变换关系的内存池
        self.tf_listener = TransformListener(self.tf_buffer, self) # 后台先成功,不断接收/tf和/tf_static话题,更新tf_buffer

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
        self.state_validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.motion_plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")

        self.start_time = None
        self.pending_results = 0
        self.shutdown_requested = False
        self.total_duration = LEFT_WAYPOINTS[-1][0]
        self.hold_seconds = float(self.get_parameter("hold_seconds").value)
        self.release_after_motion = bool(self.get_parameter("release_after_motion").value)
        self.release_delay_seconds = float(self.get_parameter("release_delay_seconds").value)
        self.gravity = float(self.get_parameter("gravity").value)
        self.grip_capture_radius = float(self.get_parameter("grip_capture_radius").value)
        self.grip_distance_min = float(self.get_parameter("grip_distance_min").value)
        self.grip_capture_distance_max = float(
            self.get_parameter("grip_capture_distance_max").value
        )
        self.grip_distance_max = float(self.get_parameter("grip_distance_max").value)
        self.waiting_for_tf_logged = False # 防止日志刷屏的懒汉开关,如果找不到tf只报错一次，后面不再重复刷
        self.trail: list[Point] = []
        self.cargo_state = "free"
        self.cargo_center = Point(x=PICK_X, y=CARGO_INITIAL_Y, z=CARGO_SIZE_Z / 2.0)
        self.cargo_velocity = Point(x=0.0, y=0.0, z=0.0)
        self.cargo_orientation = (0.0, 0.0, 0.0, 1.0)
        self.last_physics_time = self.get_clock().now()
        self.release_logged = False
        self.left_trajectory = None
        self.right_trajectory = None
        self.stage_end_times = {0: 0.0}
        self.grasp_enable_time = TRAJECTORY_START_DELAY
        self.current_joint_positions: dict[str, float] = {}
        self.latest_joint_state_time = None
        self.timer = self.create_timer(0.05, self.publish_markers)
        self._publish_waiting_markers()

    def start(self) -> bool:
        timeout = float(self.get_parameter("action_server_timeout_sec").value)
        self._publish_waiting_markers()
        if not self._wait_for_action_server(self.left_client, "left arm", timeout):
            return False
        if not self._wait_for_action_server(self.right_client, "right arm", timeout):
            return False
        if not self._wait_for_startup_state():
            return False
        if not self._apply_table_to_planning_scene():
            return False

        planned_trajectory = self._plan_carry_trajectory_with_moveit()
        if planned_trajectory is None:
            return False

        if not self._validate_planned_trajectory_with_moveit(planned_trajectory):
            return False

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

        self.get_logger().info("Sending synchronized planned carry trajectories.")
        self.start_time = self.get_clock().now()
        self.pending_results = 2

        self._send_goal(self.left_client, self.left_trajectory, "left arm")
        self._send_goal(self.right_client, self.right_trajectory, "right arm")
        return True

    def _on_joint_state(self, msg: JointState) -> None:
        for joint_name, position in zip(msg.name, msg.position):
            if joint_name in ALL_JOINTS:
                self.current_joint_positions[joint_name] = position
        self.latest_joint_state_time = self.get_clock().now()

    def _current_positions(self):
        if any(joint_name not in self.current_joint_positions for joint_name in ALL_JOINTS):
            return None
        return [self.current_joint_positions[joint_name] for joint_name in ALL_JOINTS]

    def _startup_tip_tfs_ready(self) -> bool:
        try:
            self.tf_buffer.lookup_transform("world", LEFT_CONTACT_FRAME, Time())
            self.tf_buffer.lookup_transform("world", RIGHT_CONTACT_FRAME, Time())
            return True
        except TransformException:
            return False

    def _wait_for_startup_state(self) -> bool:
        timeout = float(self.get_parameter("startup_state_timeout_sec").value)
        target_start = LEFT_WAYPOINTS[0][1] + RIGHT_WAYPOINTS[0][1]
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1_000_000_000)
        stable_since = None
        last_positions = None
        waiting_logged = False

        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            positions = self._current_positions()
            if positions is None or not self._startup_tip_tfs_ready():
                stable_since = None
                if not waiting_logged:
                    self.get_logger().info(
                        "Waiting for stable /joint_states and end-effector TFs before planning."
                    )
                    waiting_logged = True
                continue

            max_motion = (
                0.0
                if last_positions is None
                else max(abs(a - b) for a, b in zip(positions, last_positions))
            )
            now = self.get_clock().now()
            if max_motion <= STARTUP_STABLE_DELTA:
                if stable_since is None:
                    stable_since = now
            else:
                stable_since = now
            last_positions = positions

            if (
                stable_since is not None
                and (now - stable_since).nanoseconds / 1_000_000_000.0
                >= STARTUP_STABLE_SECONDS
            ):
                start_error = max(abs(a - b) for a, b in zip(positions, target_start))
                if start_error > STARTUP_START_POSE_TOLERANCE:
                    self.get_logger().error(
                        "Robot is not at the demo start pose after startup. "
                        f"Max joint error is {start_error:.3f} rad; refusing to send a "
                        "trajectory that would visibly snap at the beginning."
                    )
                    return False
                self.get_logger().info(
                    "Startup robot state is stable; "
                    f"max start-pose error {start_error:.4f} rad."
                )
                return True

        self.get_logger().error(
            "Timed out waiting for stable /joint_states and end-effector TFs."
        )
        return False

    def _publish_waiting_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        marker_array.markers.append(self._cargo_marker(now))
        marker_array.markers.append(self._table_marker(now))
        marker_array.markers.append(self._state_marker(now))
        self.marker_pub.publish(marker_array)

    def _apply_table_to_planning_scene(self) -> bool:
        if not self.apply_scene_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("Timed out waiting for /apply_planning_scene.")
            return False

        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "work_table_collision"
        collision_object.operation = CollisionObject.ADD

        def add_box(dimensions: list[float], x: float, y: float, z: float) -> None:
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
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error("Failed to add work_table_collision to MoveIt planning scene.")
            return False

        self.get_logger().info("Added work_table_collision to MoveIt planning scene.")
        return True

    def _plan_carry_trajectory_with_moveit(self):
        if not self.motion_plan_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("Timed out waiting for /plan_kinematic_path.")
            return None

        joint_names = LEFT_JOINTS + RIGHT_JOINTS
        targets = [left[1] + right[1] for left, right in zip(LEFT_WAYPOINTS, RIGHT_WAYPOINTS)]
        self.stage_end_times = {0: 0.0}

        # 一种关节在时间轴上的预期运动序列
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
            # 当索引位于搬运中段时,识别出双臂末端与货物构成刚性闭环
            # 此时调用纯数学线性插值,强制保持双臂末端相对位姿恒定
            # 规避Moveit再壁画约束下的求解失效问题
            if GRASP_STAGE_INDEX < target_index < RELEASE_STAGE_INDEX:
                segment = self._make_locked_grip_segment(
                    start_positions,
                    targets[target_index],
                    label,
                )
            else:
            # 再首尾非抓取段,系统处于开链状态
            # 调用Moveit标准RRT规划器,实现避障与动力学最优
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

    def _append_segment(
        self,
        combined: JointTrajectory,
        segment: JointTrajectory,
        fallback_positions,
        time_offset: float,
    ) -> float:
        local_duration = duration_seconds(segment.points[-1].time_from_start)
        if local_duration <= 0.001:
            max_delta = max(
                abs(a - b)
                for a, b in zip(fallback_positions, self._positions_from_point(segment, segment.points[-1], fallback_positions))
            )
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

    def _positions_from_point(self, trajectory: JointTrajectory, point, fallback_positions):
        position_by_name = {
            joint_name: fallback_positions[index]
            for index, joint_name in enumerate(LEFT_JOINTS + RIGHT_JOINTS)
        }
        for joint_name, joint_position in zip(trajectory.joint_names, point.positions):
            position_by_name[joint_name] = joint_position
        return [position_by_name[joint_name] for joint_name in LEFT_JOINTS + RIGHT_JOINTS]

    def _validate_planned_trajectory_with_moveit(self, trajectory: JointTrajectory) -> bool:
        if not self.state_validity_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("Timed out waiting for /check_state_validity.")
            return False

        total_time = duration_seconds(trajectory.points[-1].time_from_start)
        sample_count = int(total_time / SAMPLE_PERIOD) + 1
        for index in range(sample_count + 1):
            elapsed = min(index * SAMPLE_PERIOD, total_time)
            positions = self._interpolate_planned_positions(trajectory, elapsed)
            left_positions = positions[: len(LEFT_JOINTS)]
            right_positions = positions[len(LEFT_JOINTS):]

            request = GetStateValidity.Request()
            request.group_name = PLANNING_GROUP
            request.robot_state.is_diff = True
            request.robot_state.joint_state = JointState()
            request.robot_state.joint_state.name = LEFT_JOINTS + RIGHT_JOINTS
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
                    for contact in result.contacts[:4]
                )
                self.get_logger().error(
                    f"Collision check failed at t={elapsed:.2f}s "
                    f"(sample {index}/{sample_count}): {contacts or 'invalid state'}"
                )
                return False

            left_transform = fk_transform(left_positions, -0.25)
            right_transform = fk_transform(right_positions, 0.25)
            left_tip = point_from_matrix(left_transform, LEFT_CONTACT_LOCAL_OFFSET)
            right_tip = point_from_matrix(right_transform, RIGHT_CONTACT_LOCAL_OFFSET)
            tip_pose = self._tip_cargo_pose(left_tip, right_tip)
            if not self._validate_cargo_table_clearance(elapsed, tip_pose, index, sample_count):
                return False
            if self._is_carry_sample_time(elapsed) and not self._validate_grip_contact_geometry(
                elapsed,
                left_transform,
                right_transform,
                left_tip,
                right_tip,
                tip_pose,
                index,
                sample_count,
            ):
                return False

        self.get_logger().info(
            f"MoveIt accepted {sample_count + 1} sampled states for group both_arms "
            "from the planned trajectory."
        )
        return True

    def _is_carry_sample_time(self, elapsed: float) -> bool:
        grasp_time = self.stage_end_times.get(GRASP_STAGE_INDEX)
        release_start_time = self.stage_end_times.get(RELEASE_STAGE_INDEX - 1)
        if grasp_time is None:
            return False
        if elapsed < grasp_time + 0.05:
            return False
        if release_start_time is not None and elapsed > release_start_time + 0.05:
            return False
        return True

    def _validate_grip_contact_geometry(
        self,
        elapsed: float,
        left_transform,
        right_transform,
        left_tip: Point,
        right_tip: Point,
        tip_pose: dict,
        sample_index: int,
        sample_count: int,
    ) -> bool:
        tip_distance = tip_pose["tip_distance"]
        if not (GRIP_DISTANCE_VALIDATE_MIN <= tip_distance <= GRIP_DISTANCE_VALIDATE_MAX):
            self.get_logger().error(
                f"Grip contact distance invalid during carry at t={elapsed:.2f}s "
                f"(sample {sample_index}/{sample_count}): distance={tip_distance:.3f}, "
                f"expected {CARGO_SIZE_Y:.3f} m."
            )
            return False

        left_face_normal = vector_from_matrix(left_transform, (0.0, 0.0, 1.0))
        right_face_normal = vector_from_matrix(right_transform, (0.0, 0.0, 1.0))
        left_to_right = normalize(subtract(right_tip, left_tip), Point(x=0.0, y=1.0, z=0.0))
        right_to_left = scale(left_to_right, -1.0)
        left_alignment = dot(left_face_normal, left_to_right)
        right_alignment = dot(right_face_normal, right_to_left)

        if (
            left_alignment < GRIP_FACE_ALIGNMENT_MIN
            or right_alignment < GRIP_FACE_ALIGNMENT_MIN
        ):
            self.get_logger().error(
                f"Grip pad face is not pointing at the cargo side at t={elapsed:.2f}s "
                f"(sample {sample_index}/{sample_count}): "
                f"left_alignment={left_alignment:.3f}, "
                f"right_alignment={right_alignment:.3f}."
            )
            return False

        world_up = Point(x=0.0, y=0.0, z=1.0)
        left_pad_up = vector_from_matrix(left_transform, (0.0, 1.0, 0.0))
        right_pad_up = vector_from_matrix(right_transform, (0.0, 1.0, 0.0))
        left_up_alignment = abs(dot(left_pad_up, world_up))
        right_up_alignment = abs(dot(right_pad_up, world_up))
        if (
            left_up_alignment < GRIP_PAD_UP_ALIGNMENT_MIN
            or right_up_alignment < GRIP_PAD_UP_ALIGNMENT_MIN
        ):
            self.get_logger().error(
                f"Grip pad is tilted instead of staying vertical at t={elapsed:.2f}s "
                f"(sample {sample_index}/{sample_count}): "
                f"left_up={left_up_alignment:.3f}, right_up={right_up_alignment:.3f}."
            )
            return False

        return True

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

    def _validate_cargo_table_clearance(
        self,
        elapsed: float,
        tip_pose: dict,
        sample_index: int,
        sample_count: int,
    ) -> bool:
        center = tip_pose["center"]
        bottom_z = center.z - CARGO_SIZE_Z / 2.0
        if not self._point_over_table(center):
            return True

        min_bottom_z = TABLE_TOP_Z - 0.005
        if bottom_z < min_bottom_z:
            self.get_logger().error(
                f"Cargo penetrates table at t={elapsed:.2f}s "
                f"(sample {sample_index}/{sample_count}): bottom_z={bottom_z:.3f}, "
                f"table_top_z={TABLE_TOP_Z:.3f}."
            )
            return False

        return True

    def _wait_for_action_server(
        self,
        client: ActionClient,
        label: str,
        timeout: float,
    ) -> bool:
        waited = 0.0
        while rclpy.ok() and waited < timeout:
            self._publish_waiting_markers()
            if client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(f"{label} trajectory action server is ready.")
                return True
            waited += 1.0
            self.get_logger().info(f"Waiting for {label} trajectory action server...")

        self.get_logger().error(f"Timed out waiting for {label} trajectory action server.")
        return False

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

    def _send_goal(self, client: ActionClient, trajectory: JointTrajectory, label: str) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = duration_msg(0.5)

        future = client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._on_goal_response(done, label))

    def _on_goal_response(self, future, label: str) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{label} trajectory goal was rejected.")
            self._mark_result_done()
            return

        self.get_logger().info(f"{label} trajectory goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done: self._on_result(done, label))

    def _on_result(self, future, label: str) -> None:
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"{label} trajectory finished successfully.")
        else:
            self.get_logger().error(
                f"{label} trajectory failed with error code {result.error_code}: "
                f"{result.error_string}"
            )
        self._mark_result_done()

    def _mark_result_done(self) -> None:
        self.pending_results -= 1
        if self.pending_results == 0:
            self.get_logger().info("Dual-arm carry motion complete; holding final object marker.")

    def elapsed_seconds(self) -> float:
        if self.start_time is None:
            return 0.0
        return (self.get_clock().now() - self.start_time).nanoseconds / 1_000_000_000.0

    def publish_markers(self) -> None:
        now_clock = self.get_clock().now()
        now = now_clock.to_msg()
        try:
            left_tip = point_from_transform(
                self.tf_buffer.lookup_transform("world", LEFT_CONTACT_FRAME, Time()),
            )
            right_tip = point_from_transform(
                self.tf_buffer.lookup_transform("world", RIGHT_CONTACT_FRAME, Time()),
            )
        except TransformException as exc:
            if not self.waiting_for_tf_logged:
                self.get_logger().info(f"Waiting for end-effector TFs: {exc}")
                self.waiting_for_tf_logged = True
            marker_array = MarkerArray()
            marker_array.markers.append(self._cargo_marker(now))
            marker_array.markers.append(self._table_marker(now))
            marker_array.markers.append(self._state_marker(now))
            self.marker_pub.publish(marker_array)
            return

        if self.start_time is None:
            self.cargo_state = "free"
            self.cargo_center = Point(x=PICK_X, y=CARGO_INITIAL_Y, z=CARGO_SIZE_Z / 2.0)
            self.cargo_velocity = Point(x=0.0, y=0.0, z=0.0)
            self.cargo_orientation = (0.0, 0.0, 0.0, 1.0)
            self.last_physics_time = now_clock
            marker_array = MarkerArray()
            marker_array.markers.append(self._cargo_marker(now))
            marker_array.markers.append(self._contact_marker(now, left_tip, 2))
            marker_array.markers.append(self._contact_marker(now, right_tip, 3))
            marker_array.markers.append(self._support_line_marker(now, left_tip, right_tip))
            marker_array.markers.append(self._state_marker(now))
            marker_array.markers.append(self._table_marker(now))
            self.marker_pub.publish(marker_array)
            return

        dt = (now_clock - self.last_physics_time).nanoseconds / 1_000_000_000.0
        self.last_physics_time = now_clock
        dt = min(max(dt, 0.001), 0.1)

        tip_pose = self._tip_cargo_pose(left_tip, right_tip)
        self._update_cargo_physics(tip_pose, dt)
        self.trail.append(self.cargo_center)
        self.trail = self.trail[-300:]
        marker_array = MarkerArray()
        marker_array.markers.append(self._cargo_marker(now))
        marker_array.markers.append(self._contact_marker(now, left_tip, 2))
        marker_array.markers.append(self._contact_marker(now, right_tip, 3))
        marker_array.markers.append(self._path_marker(now))
        marker_array.markers.append(self._support_line_marker(now, left_tip, right_tip))
        marker_array.markers.append(self._state_marker(now))
        marker_array.markers.append(self._table_marker(now))

        self.marker_pub.publish(marker_array)

        if (
            not self.shutdown_requested
            and self.start_time is not None
            and self.elapsed_seconds()
            > self.total_duration + self.release_delay_seconds + self.hold_seconds
        ):
            self.shutdown_requested = True
            self.get_logger().info("Carry demo finished.")
            rclpy.shutdown()

    def _tip_cargo_pose(self, left_tip: Point, right_tip: Point) -> dict:
        midpoint = scale(add(left_tip, right_tip), 0.5)
        y_axis = normalize(subtract(right_tip, left_tip), Point(x=0.0, y=1.0, z=0.0))
        world_up = Point(x=0.0, y=0.0, z=1.0)
        if abs(dot(y_axis, world_up)) > 0.92:
            world_up = Point(x=1.0, y=0.0, z=0.0)

        x_axis = normalize(cross(y_axis, world_up), Point(x=1.0, y=0.0, z=0.0))
        z_axis = normalize(cross(x_axis, y_axis), Point(x=0.0, y=0.0, z=1.0))
        qx, qy, qz, qw = quaternion_from_axes(x_axis, y_axis, z_axis)

        return {
            "center": midpoint,
            "orientation": (qx, qy, qz, qw),
            "tip_distance": norm(subtract(right_tip, left_tip)),
        }

    def _update_cargo_physics(self, tip_pose: dict, dt: float) -> None:
        should_release = (
            self.release_after_motion
            and self.start_time is not None
            and self.elapsed_seconds() >= self.total_duration + self.release_delay_seconds
        )
        grasp_ready = (
            self.start_time is not None
            and self.elapsed_seconds() >= self.grasp_enable_time
        )
        can_grasp = grasp_ready and self._can_grasp(tip_pose)

        if self.cargo_state == "placed":
            self._place_cargo_on_table()
            return

        if self.cargo_state == "grasped":
            if should_release or not self._can_hold(tip_pose):
                self._release_cargo(should_release)
            else:
                self._set_cargo_from_tip_pose(tip_pose, dt)
                return

        if self.cargo_state == "free" and can_grasp and not should_release:
            self.cargo_state = "grasped"
            self.get_logger().info(
                "Object grasped by both arm contact patches "
                f"(contact distance {tip_pose['tip_distance']:.3f} m, "
                f"cargo width {CARGO_SIZE_Y:.3f} m)."
            )
            self._set_cargo_from_tip_pose(tip_pose, dt)
            return

        self._integrate_free_fall(dt)

    def _can_grasp(self, tip_pose: dict) -> bool:
        tip_distance = tip_pose["tip_distance"]
        center_error = norm(subtract(tip_pose["center"], self.cargo_center))
        return (
            self.grip_distance_min <= tip_distance <= self.grip_capture_distance_max
            and center_error <= self.grip_capture_radius
            and tip_pose["center"].z >= GROUND_Z + CARGO_SIZE_Z * 0.45
        )

    def _can_hold(self, tip_pose: dict) -> bool:
        return self.grip_distance_min <= tip_pose["tip_distance"] <= self.grip_distance_max

    def _release_cargo(self, timed_release: bool) -> None:
        if self._is_over_table():
            self.cargo_state = "placed"
            self._place_cargo_on_table()
            if not self.release_logged:
                self.release_logged = True
                self.get_logger().info("Object placed on the table and supported by the tabletop.")
            return

        self.cargo_state = "free"
        if not self.release_logged:
            self.release_logged = True
            if timed_release:
                self.get_logger().info("Object released; gravity is now acting on it.")
            else:
                self.get_logger().info("Object released by opening the two arm tips.")

    def _place_cargo_on_table(self) -> None:
        self.cargo_center.z = TABLE_TOP_Z + CARGO_SIZE_Z / 2.0
        self.cargo_velocity = Point(x=0.0, y=0.0, z=0.0)

    def _set_cargo_from_tip_pose(self, tip_pose: dict, dt: float) -> None:
        previous_center = self.cargo_center
        self.cargo_center = tip_pose["center"]
        self.cargo_center.z = max(self.cargo_center.z, GROUND_Z + CARGO_SIZE_Z / 2.0)
        self.cargo_orientation = tip_pose["orientation"]
        if dt > 0.0:
            self.cargo_velocity = scale(subtract(self.cargo_center, previous_center), 1.0 / dt)

    def _integrate_free_fall(self, dt: float) -> None:
        self.cargo_velocity.z -= self.gravity * dt
        self.cargo_center = add(self.cargo_center, scale(self.cargo_velocity, dt))

        support_surface_z = self._support_surface_z()
        floor_center_z = support_surface_z + CARGO_SIZE_Z / 2.0
        if self.cargo_center.z <= floor_center_z:
            self.cargo_center.z = floor_center_z
            if self.cargo_velocity.z < 0.0:
                self.cargo_velocity.z = 0.0
            self.cargo_velocity.x *= 0.82
            self.cargo_velocity.y *= 0.82
            if support_surface_z == TABLE_TOP_Z:
                self.cargo_state = "placed"

    def _support_surface_z(self) -> float:
        if self._is_over_table():
            cargo_bottom_z = self.cargo_center.z - CARGO_SIZE_Z / 2.0
            if cargo_bottom_z >= TABLE_TOP_Z - TABLE_SUPPORT_TOLERANCE:
                return TABLE_TOP_Z
        return GROUND_Z

    def _point_over_table(self, point: Point) -> bool:
        half_x = TABLE_SIZE_X / 2.0 + CARGO_SIZE_X / 2.0
        half_y = TABLE_SIZE_Y / 2.0 + CARGO_SIZE_Y / 2.0
        return (
            abs(point.x - TABLE_X) <= half_x
            and abs(point.y - TABLE_Y) <= half_y
        )

    def _is_over_table(self) -> bool:
        return self._point_over_table(self.cargo_center)

    def _base_marker(self, now, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = now
        marker.ns = "dual_arm_carry_demo"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _cargo_marker(self, now) -> Marker:
        marker = self._base_marker(now, 1, Marker.CUBE)
        marker.pose.position = self.cargo_center
        qx, qy, qz, qw = self.cargo_orientation
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = CARGO_SIZE_X
        marker.scale.y = CARGO_SIZE_Y
        marker.scale.z = CARGO_SIZE_Z
        if self.cargo_state == "grasped":
            marker.color.r = 0.12
            marker.color.g = 0.42
            marker.color.b = 0.95
        else:
            marker.color.r = 0.85
            marker.color.g = 0.36
            marker.color.b = 0.12
        marker.color.a = 0.82
        return marker

    def _contact_marker(self, now, point: Point, marker_id: int) -> Marker:
        marker = self._base_marker(now, marker_id, Marker.SPHERE)
        marker.pose.position = point
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.035
        if self.cargo_state == "grasped":
            marker.color.r = 0.25
            marker.color.g = 0.95
            marker.color.b = 0.35
        else:
            marker.color.r = 0.95
            marker.color.g = 0.68
            marker.color.b = 0.18
        marker.color.a = 0.95
        return marker

    def _path_marker(self, now) -> Marker:
        marker = self._base_marker(now, 4, Marker.LINE_STRIP)
        marker.scale.x = 0.012
        marker.color.r = 0.05
        marker.color.g = 0.85
        marker.color.b = 0.72
        marker.color.a = 0.7
        marker.points = self.trail
        return marker

    def _support_line_marker(self, now, left_tip: Point, right_tip: Point) -> Marker:
        marker = self._base_marker(now, 5, Marker.LINE_STRIP)
        marker.scale.x = 0.018
        if self.cargo_state == "grasped":
            marker.color.r = 0.25
            marker.color.g = 0.95
            marker.color.b = 0.35
            marker.color.a = 0.95
        else:
            marker.color.r = 0.65
            marker.color.g = 0.65
            marker.color.b = 0.65
            marker.color.a = 0.35
        marker.points = [left_tip, right_tip]
        return marker

    def _state_marker(self, now) -> Marker:
        marker = self._base_marker(now, 6, Marker.TEXT_VIEW_FACING)
        marker.pose.position = add(self.cargo_center, Point(x=0.0, y=0.0, z=0.16))
        marker.scale.z = 0.06
        if self.cargo_state == "grasped":
            marker.text = "GRASPED"
        elif self.cargo_state == "placed":
            marker.text = "PLACED"
        else:
            marker.text = "FREE / GRAVITY"
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.85
        return marker

    def _table_marker(self, now) -> Marker:
        marker = self._base_marker(now, 7, Marker.CUBE)
        marker.pose.position.x = TABLE_X
        marker.pose.position.y = TABLE_Y
        marker.pose.position.z = TABLE_CENTER_Z
        marker.scale.x = TABLE_SIZE_X
        marker.scale.y = TABLE_SIZE_Y
        marker.scale.z = TABLE_SIZE_Z
        marker.color.r = 0.55
        marker.color.g = 0.44
        marker.color.b = 0.32
        marker.color.a = 0.75
        return marker


def main() -> None:
    rclpy.init()
    node = DualArmCarryDemo()
    if not node.start():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
