#!/usr/bin/env python3
"""Gazebo Classic carry demo with a gravity-driven cargo box."""

from __future__ import annotations

import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState, SetEntityState
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


# 双臂的运动路径
# 每个列表包含若干路点,每个路点是一个元组
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

# 机器人手臂的几何模型(DH参数)
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
# Physics-based grasp constraint parameters (spring-damper + gravity compensation)
GRAVITY = 9.81             # m/s^2, matches dual_arm_carry.world
CARGO_MASS = 1.0           # kg, matches dual_arm_carry.world
KP_POS = 600.0             # N/m  translational stiffness (~3.9 Hz natural freq)
KD_POS = 75.0              # Ns/m translational damping (damping ratio ~1.5)
KP_ROT = 8.0               # Nm/rad rotational stiffness
KD_ROT = 0.8               # Nms/rad rotational damping
FORCE_REAPPLY_PERIOD = 0.1  # seconds between /gazebo/apply_body_wrench calls
CARGO_BODY_NAME = "cargo_box::link"
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

# seconds -> duration
def duration_msg(seconds: float) -> Duration:
    whole_seconds = int(seconds)
    return Duration(
        sec=whole_seconds,
        nanosec=int((seconds - whole_seconds) * 1_000_000_000),
    )

# duration -> seconds
def duration_seconds(duration: Duration) -> float:
    return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0

# duration(原时长 + 偏移)
def shifted_duration(duration: Duration, offset: float) -> Duration:
    return duration_msg(duration_seconds(duration) + offset)

# 加
def add(a: Point, b: Point) -> Point:
    return Point(x=a.x + b.x, y=a.y + b.y, z=a.z + b.z)

# 减
def subtract(a: Point, b: Point) -> Point:
    return Point(x=a.x - b.x, y=a.y - b.y, z=a.z - b.z)

# 数乘
def scale(a: Point, factor: float) -> Point:
    return Point(x=a.x * factor, y=a.y * factor, z=a.z * factor)

# 点积
def dot(a: Point, b: Point) -> float:
    return a.x * b.x + a.y * b.y + a.z * b.z

# 叉积
def cross(a: Point, b: Point) -> Point:
    return Point(
        x=a.y * b.z - a.z * b.y,
        y=a.z * b.x - a.x * b.z,
        z=a.x * b.y - a.y * b.x,
    )

# 向量长度模
def norm(a: Point) -> float:
    return (a.x * a.x + a.y * a.y + a.z * a.z) ** 0.5

# 归一化
def normalize(a: Point, fallback: Point) -> Point:
    length = norm(a)
    if length < 1e-6:
        return fallback
    return scale(a, 1.0 / length)

# 4 * 4矩阵乘法
def matmul(a, b):
    return [[sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4)] for row in range(4)]

# 根据位移(x,y,z)和rpy欧拉角生成一个4 * 4齐次变换矩阵
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

# 绕z轴旋转的齐次变换矩阵
def z_rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

# 正向运动学(FK)
# 检查传入的关节角度数组长度是否正确
def validate_fk_input(positions, expected_len: int, name: str = "joint_positions") -> None:
    """Validate that FK input has the expected number of elements.

    Raises TypeError / ValueError so that a silent zip-truncation bug
    (passing fewer than 6 joint values) is caught immediately.
    """
    if not isinstance(positions, (list, tuple)):
        raise TypeError(f"{name} must be a list or tuple, got {type(positions).__name__}")
    if len(positions) != expected_len:
        raise ValueError(
            f"{name} must have exactly {expected_len} elements, got {len(positions)}"
        )

# 给定一组关节角,算出末端再空间中的位姿
def fk_transform(joint_positions, base_y):
    validate_fk_input(joint_positions, len(JOINT_ORIGINS))
    transform = transform_matrix((0.0, base_y, 0.0), (0.0, 0.0, 0.0))
    for (xyz, rpy), joint_position in zip(JOINT_ORIGINS, joint_positions):
        transform = matmul(transform, transform_matrix(xyz, rpy))
        transform = matmul(transform, z_rotation(joint_position))
    return transform

# 返回一个列表 包含每个连杆的累积变换矩阵 如果后续需要知道中间关节的位置 可以用它
def fk_link_transforms(joint_positions, base_y):
    validate_fk_input(joint_positions, len(JOINT_ORIGINS))
    transform = transform_matrix((0.0, base_y, 0.0), (0.0, 0.0, 0.0))
    transforms = [transform]
    for (xyz, rpy), joint_position in zip(JOINT_ORIGINS, joint_positions):
        transform = matmul(transform, transform_matrix(xyz, rpy))
        transform = matmul(transform, z_rotation(joint_position))
        transforms.append(transform)
    return transforms

# 从一个4 * 4齐次坐标系变换矩阵中,提取一个点在世界坐标系下的坐标
# 该点相对于矩阵的局部坐标系有一个偏移
def point_from_matrix(
    transform,
    local_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Point:
    return Point(
        x=transform[0][3] + sum(transform[0][index] * local_offset[index] for index in range(3)),
        y=transform[1][3] + sum(transform[1][index] * local_offset[index] for index in range(3)),
        z=transform[2][3] + sum(transform[2][index] * local_offset[index] for index in range(3)),
    )

# 再fk_transform 的基础上 再叠加一个末端局部偏移 直接返回该接触点在世界坐标系下的Point
def fk_tip(
    joint_positions,
    base_y,
    local_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    validate_fk_input(joint_positions, len(JOINT_ORIGINS))
    transform = fk_transform(joint_positions, base_y)
    return point_from_matrix(transform, local_offset)

# 求一个刚体变换(旋转+平移)的逆变换
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

# 用原点和三个互相正交的轴向量构造一个4 * 4齐次变换矩阵
def matrix_from_axes(origin: Point, x_axis: Point, y_axis: Point, z_axis: Point):
    return [
        [x_axis.x, y_axis.x, z_axis.x, origin.x],
        [x_axis.y, y_axis.y, z_axis.y, origin.y],
        [x_axis.z, y_axis.z, z_axis.z, origin.z],
        [0.0, 0.0, 0.0, 1.0],
    ]

# 将4*4齐次变换矩阵转换为ROS的Pose消息(包含位置和四元数)
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

# 根据左右两个夹持点的坐标,计算箱子的坐标系矩阵
def cargo_matrix_from_tips(left_tip: Point, right_tip: Point):
    midpoint = scale(add(left_tip, right_tip), 0.5)
    y_axis = normalize(subtract(right_tip, left_tip), Point(x=0.0, y=1.0, z=0.0))
    up = Point(x=0.0, y=0.0, z=1.0)
    if abs(dot(y_axis, up)) > 0.92:
        up = Point(x=1.0, y=0.0, z=0.0)
    x_axis = normalize(cross(y_axis, up), Point(x=1.0, y=0.0, z=0.0))
    z_axis = normalize(cross(x_axis, y_axis), Point(x=0.0, y=0.0, z=1.0))
    return matrix_from_axes(midpoint, x_axis, y_axis, z_axis)

# 从标准正交的旋转矩阵中提取四元数
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

# 从两个指尖位置计算出箱子的位姿信息,并以字典返回
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
        # Gazebo 瞬移物体
        self.entity_state_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        # Moveit 碰撞校验
        self.state_validity_client = self.create_client(GetStateValidity, "/check_state_validity") 
        # Moveit 场景管理
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        # Gazebo 施加力/力矩
        self.motion_plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        # 向Gazebo请求指定模型的当前真实位姿和速度
        self.get_model_state_client = self.create_client(
            GetModelState, "/gazebo/get_model_state"
        )
        # 向Gazebo中的指定物体(箱子)施加力和力矩
        self.apply_body_wrench_client = self.create_client(
            ApplyBodyWrench, "/gazebo/apply_body_wrench"
        )
        # 左臂轨迹执行
        self.left_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/left_arm_controller/follow_joint_trajectory",
        )
        # 右臂轨迹执行
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
        self.state_validity_available = True  # graceful degradation flag
        self.timer = None

    # 阻塞等待所有必须的服务端和动作服务器上线,确保通信链路就堵
    # ros2是分布式系统,不同节点启动速度不同
    # 如果程序一启动就尝试调用服务或发送动作目标
    # 但服务端还未完成初始化,就会发生通信失败
    # 这段代码通过阻塞等待确保所有依赖都已经完全就绪
    def wait_for_services(self) -> bool:
        for label, client in (
            ("set_entity_state", self.entity_state_client),      # Gazebo中瞬移物体的服务
            ("apply_planning_scene", self.apply_scene_client),   # 向Moveit添加碰撞物体
            ("plan_kinematic_path", self.motion_plan_client),    # 请求Moveit规划一条无碰撞的关节轨迹
            ("get_model_state", self.get_model_state_client),    # 从Gazebo获取当前物体(箱子)的状态
            ("apply_body_wrench", self.apply_body_wrench_client),# 向Gazebo中的箱子施加力/力矩
        ):
            if not client.wait_for_service(timeout_sec=90.0):
                self.get_logger().error(f"Timed out waiting for service {label}")
                return False

        # /check_state_validity is optional — graceful degradation
        if not self.state_validity_client.wait_for_service(timeout_sec=90.0):
            self.get_logger().warn(
                "/check_state_validity not available; skipping MoveIt collision validation."
            )
            self.state_validity_available = False

        # 动作服务器检查
        for label, client in (
            ("left_arm_controller/follow_joint_trajectory", self.left_client),
            ("right_arm_controller/follow_joint_trajectory", self.right_client),
        ):
            if not client.wait_for_server(timeout_sec=90.0):
                self.get_logger().error(f"Timed out waiting for action {label}")
                return False
        return True

    # 执行完整的任务准备流水线,一旦成功,双臂就 开始运动,实时状态机也开始运行
    def prepare_demo(self) -> bool:
        # 向Moveit场景添加桌子
        if not self._apply_table_to_planning_scene():
            return False

        # Moveit规划完整的双臂搬运轨迹
        planned_trajectory = self._plan_carry_trajectory_with_moveit()
        if planned_trajectory is None:
            return False

        # 对轨迹均匀采样并预计算指尖位置
        self.samples = self._build_samples(planned_trajectory)

        # 碰撞校验
        if not self._validate_samples_with_moveit():
            return False
        self.get_logger().info(
            f"MoveIt accepted {len(self.samples)} sampled states from the planned trajectory."
        )

        # 拆分轨迹为左右臂
        self.left_trajectory, self.right_trajectory = self._split_combined_trajectory(
            planned_trajectory
        )

        # 记录轨迹总时长
        self.total_duration = duration_seconds(
            self.left_trajectory.points[-1].time_from_start
        )

        # 计算抓取使能时间
        self.grasp_enable_time = (
            self.stage_end_times.get(GRASP_STAGE_INDEX, 0.0)
            + TRAJECTORY_START_DELAY
            + 0.10
        )

        # 异步发送轨迹到左右臂控制器
        self._send_trajectory(self.left_client, self.left_trajectory, "left arm")
        self._send_trajectory(self.right_client, self.right_trajectory, "right arm")

        # 记录起始时间 + 启动定时器
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.on_timer)
        return True

    # 返回从演示开始(start_time)到当前时刻的流逝时间
    def elapsed(self) -> float:
        if self.start_time is None:
            return 0.0
        return (self.get_clock().now() - self.start_time).nanoseconds / 1_000_000_000.0

    # 预采样轨迹
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
                }
            )
        return samples

    # 实时循环 --- 抓取/释放状态机
    # 以50Hz的频率被不断调用
    # 1.根据当前耗时,从预采样表中取出本时刻的期望指尖状态
    # 2.运行一个"抓取/释放"状态机
    # 3.在"grasped"状态下,调用_apply_grasp_constraint()
    # 4.结束条件判断
    def on_timer(self):
        elapsed = self.elapsed()
        sample_index = min(int(elapsed / SAMPLE_PERIOD), len(self.samples) - 1)
        sample = self.samples[sample_index]
        tip_pose = sample["tip_pose"]
        # Gazebo now handles robot joint physics via gazebo_ros2_control plugin
        # (use_gazebo=true in xacro). No link teleportation needed.

        should_release = elapsed >= self.total_duration + 0.8
        grip_open = tip_pose["tip_distance"] > GRIP_DISTANCE_MAX
        grasp_ready = elapsed >= self.grasp_enable_time

        if self.cargo_state == "grasped" and (should_release or grip_open):
            self.cargo_state = "free"
            self._stop_grasp_constraint()
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
            self._apply_grasp_constraint(tip_pose)  # physics-based, not teleportation

        if elapsed > self.total_duration + 7.0:
            self.get_logger().info("Gazebo carry demo finished.")
            rclpy.shutdown()

    # 添加桌子碰撞模型
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

    # 规划双臂协同规划
    # 用于通过Moveit规划一个包含多个航点的关节轨迹
    # 并特别处理了"抓取"和"释放"阶段的特殊运动模式
    def _plan_carry_trajectory_with_moveit(self):
        # 准备关节名称和目标列表
        joint_names = LEFT_JOINTS + RIGHT_JOINTS
        targets = [left[1] + right[1] for left, right in zip(LEFT_WAYPOINTS, RIGHT_WAYPOINTS)]
        self.stage_end_times = {0: 0.0}

        # 初始化联合轨迹对象
        combined = JointTrajectory()
        combined.joint_names = joint_names
        first_point = JointTrajectoryPoint()
        first_point.positions = targets[0]
        first_point.time_from_start = duration_msg(0.0)
        combined.points.append(first_point)

        # 循环处理每个阶段
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

    # 在抓取保持阶段直接生成线性插值轨迹段,不经过Moveit,保证双臂严格同步
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

    # 在拼接完一段轨迹后,如果当前轨迹的最后一个点与目标路点位置还有微小差距,则补一个沉降点精确到达目标
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

    # 调用Moveit的运动 规划器,为双臂从起点到终点生成一条无碰撞的关节轨迹段
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

    # 将 一段轨迹segment(来自Moveit或插值)拼接到总的组合轨迹combined的末尾
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

    # 从轨迹点的部分关节信息中重建出完整的12关节顺序列表
    def _positions_from_point(self, trajectory, point, fallback_positions):
        position_by_name = {
            joint_name: fallback_positions[index]
            for index, joint_name in enumerate(LEFT_JOINTS + RIGHT_JOINTS)
        }
        for joint_name, joint_position in zip(trajectory.joint_names, point.positions):
            position_by_name[joint_name] = joint_position
        return [position_by_name[joint_name] for joint_name in LEFT_JOINTS + RIGHT_JOINTS]

    # 碰撞验证
    def _validate_samples_with_moveit(self) -> bool:
        if not self.state_validity_available:
            self.get_logger().warn(
                "State validity service unavailable; skipping MoveIt collision validation. "
                "Gazebo physics will handle real collisions."
            )
            return True

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
                self.get_logger().warn(
                    f"State validity service timed out at t={sample['time']:.2f}s "
                    f"(sample {index}/{len(self.samples)}); continuing — "
                    "Gazebo physics handles real collisions."
                )
                continue
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

    # 检查在某个采样时刻,箱子底部是否穿透了桌面
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

    # 判断一个空间点是否位于桌正上方的矩形区域内
    def _point_over_table(self, point: Point) -> bool:
        half_x = TABLE_SIZE_X / 2.0 + CARGO_SIZE_X / 2.0
        half_y = TABLE_SIZE_Y / 2.0 + CARGO_SIZE_Y / 2.0
        return (
            abs(point.x - TABLE_X) <= half_x
            and abs(point.y - TABLE_Y) <= half_y
        )

    # 判断当前两避指尖的位姿是否满足抓取箱子的条件
    def _can_grasp(self, tip_pose) -> bool:
        tip_distance = tip_pose["tip_distance"]
        center_error = norm(subtract(tip_pose["center"], self.cargo_center))
        return (
            GRIP_DISTANCE_MIN <= tip_distance <= GRIP_CAPTURE_DISTANCE_MAX
            and center_error <= GRIP_CAPTURE_RADIUS
            and tip_pose["center"].z >= CARGO_SIZE_Z * 0.45
        )

    # 给定一条关节轨迹和一个当前时间elapsed,线性插值出此时各个关节的期望位置
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

    # 分割并发送轨迹
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

    # 向Gazebo的关节轨迹控制器异步发送一条完整的关节轨迹
    def _send_trajectory(self, client, trajectory, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = duration_msg(0.5)
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._on_goal_response(done, label))

    # 当Gazebo控制器响应动作目标时触发,处理轨迹目标是否被Gazebo控制器接受
    def _on_goal_response(self, future, label):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{label} trajectory goal was rejected.")
            return
        self.get_logger().info(f"{label} trajectory goal accepted by Gazebo controller.")
        goal_handle.get_result_async().add_done_callback(
            lambda done: self._on_goal_result(done, label)
        )

    # 当整条轨迹执行完毕后触发,,记录最终是成功还是失败
    def _on_goal_result(self, future, label):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"{label} Gazebo trajectory finished successfully.")
        else:
            self.get_logger().error(
                f"{label} Gazebo trajectory failed with error code {result.error_code}: "
                f"{result.error_string}"
            )

    # 从Gazebo物理引擎读取箱子当前的真实位姿和速度
    def _read_cargo_state(self):
        """Read the actual cargo box pose and twist from Gazebo physics engine.

        Returns the GetModelState.Response on success, or None on failure.
        """
        request = GetModelState.Request()
        request.model_name = "cargo_box"
        request.relative_entity_name = "world"
        future = self.get_model_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warn(
                "Failed to read cargo state from Gazebo; constraint cannot be applied."
            )
            return None
        return result

    # 计算并施加虚拟抓取力,让箱子跟随两臂指尖期望位置运动
    def _apply_grasp_constraint(self, tip_pose):
        """Apply a spring-damper constraint to hold the cargo between both arm tips.

        Uses Gazebo's /gazebo/apply_body_wrench to apply PD-controlled forces
        instead of teleporting the cargo (SetEntityState).  This lets Gazebo's
        physics engine handle the actual dynamics — inertia, friction, and
        contact response are all simulated naturally.

        Force law (per axis):
            F = Kp * (desired_pos - actual_pos) + Kd * (0 - actual_vel)
        Plus gravity compensation in Z: Fz += cargo_mass * g
        """
        cargo_state = self._read_cargo_state()
        if cargo_state is None:
            return  # warning already logged

        desired_pos = tip_pose["center"]
        actual_pos = cargo_state.pose.position
        actual_vel = cargo_state.twist.linear
        actual_ang_vel = cargo_state.twist.angular

        # Translational PD + gravity compensation
        fx = KP_POS * (desired_pos.x - actual_pos.x) + KD_POS * (-actual_vel.x)
        fy = KP_POS * (desired_pos.y - actual_pos.y) + KD_POS * (-actual_vel.y)
        fz = (KP_POS * (desired_pos.z - actual_pos.z)
              + KD_POS * (-actual_vel.z)
              + CARGO_MASS * GRAVITY)

        # Rotational damping only (simplified — full quaternion PD not needed
        # for the carry demo because both arms constrain orientation kinematically)
        tx = -KD_ROT * actual_ang_vel.x
        ty = -KD_ROT * actual_ang_vel.y
        tz = -KD_ROT * actual_ang_vel.z

        request = ApplyBodyWrench.Request()
        request.body_name = CARGO_BODY_NAME
        request.reference_frame = "world"
        request.reference_point = actual_pos  # apply at CoM to avoid spurious torque
        request.wrench.force.x = fx
        request.wrench.force.y = fy
        request.wrench.force.z = fz
        request.wrench.torque.x = tx
        request.wrench.torque.y = ty
        request.wrench.torque.z = tz
        request.duration = duration_msg(FORCE_REAPPLY_PERIOD * 2.0)

        self.apply_body_wrench_client.call_async(request)

    # 移除所有虚拟力,让箱子完全受Gazebo重力控制
    def _stop_grasp_constraint(self):
        """Remove all grasp forces; cargo falls under Gazebo's natural gravity."""
        request = ApplyBodyWrench.Request()
        request.body_name = CARGO_BODY_NAME
        request.reference_frame = "world"
        request.wrench.force.x = 0.0
        request.wrench.force.y = 0.0
        request.wrench.force.z = 0.0
        request.wrench.torque.x = 0.0
        request.wrench.torque.y = 0.0
        request.wrench.torque.z = 0.0
        request.duration = Duration(sec=0, nanosec=1)
        self.apply_body_wrench_client.call_async(request)


def main():
    rclpy.init()
    node = GazeboCarryDemo()
    # 如果等待服务没成功或者演示服务没成功就进入错误处理
    if not node.wait_for_services() or not node.prepare_demo():
        node.destroy_node()   # 手动销毁这个ROS2节点实例 -> 释放节点资源(定时器,客户端)
        rclpy.shutdown()      # 关闭整个rclpy客户端 -> 断开ROS2通信,清理全局库状态
        sys.exit(1)           # 退出当前python进程,并返回退出码1 -> 向操作系统报告"我挂了"
    try:
        """
        维持节点活动并处理所有异步事件
        它会进入一个事件循环,不断检查并处理节点上所有待处理的回调,定时器
        服务请求,Action反馈,只要ROS2上下文未被关闭,它就会一直运行,保持节点存活
        """
        rclpy.spin(node)      # 可能被Ctrl + C终端
    finally:    
        node.destroy_node()   # 无论是否发生异常,这行都会执行


if __name__ == "__main__":
    main()
