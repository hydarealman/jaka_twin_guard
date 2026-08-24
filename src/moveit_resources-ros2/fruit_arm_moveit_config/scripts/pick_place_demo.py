#!/usr/bin/env python3
"""
单臂 Pick-and-Place 仿真 —— 从桌面抓取水果放入料框。

核心思路：
  使用 MoveIt2 的 /compute_ik 服务，根据水果的 3D 位置动态求解关节角度，
  不再依赖手工估算的硬编码关节值。

流程：
  HOME → 逐个水果 (hover IK → grasp IK → 夹紧 → retreat → bin IK → 松开) → HOME
"""

from __future__ import annotations

import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped, Vector3
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


# ============================================================================
# 常量
# ============================================================================

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GRIPPER_JOINTS = ["left_finger_joint", "right_finger_joint"]
ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS

GRIPPER_OPEN = [0.04, -0.04]
# 闭合时留 0.002m 间隙，避免手指碰撞盒重叠（手指厚 0.008m，在 Y=±0.005 时内边距=0.002m）
GRIPPER_CLOSED = [0.005, -0.005]

PLANNING_GROUP = "arm"
PLANNER_ID = "RRTConnectkConfigDefault"
PLANNING_TIME = 8.0
VELOCITY_SCALING = 0.50
ACCELERATION_SCALING = 0.50
JOINT_TOLERANCE = 0.005
SAMPLE_PERIOD = 0.10

IK_LINK = "gripper_tcp"
IK_TIMEOUT = 0.5          # IK 求解超时 (秒)
HOVER_OFFSET_Z = 0.12     # TCP 悬停高度

# 场景尺寸（世界坐标系，米）
TABLE_CX, TABLE_CY, TABLE_TOP_Z = 0.70, 0.0, 0.30
TABLE_SX, TABLE_SY, TABLE_SZ = 0.75, 0.70, 0.04
TABLE_CZ = TABLE_TOP_Z - TABLE_SZ / 2.0
LEG_SIZE = 0.04
LEG_H = TABLE_TOP_Z - TABLE_SZ

# 料框位置（从 (0.70,0.55) 挪近到 (0.55,0.45)，缩短距离便于够到）
BIN_X, BIN_Y, BIN_TOP_Z = 0.55, 0.45, 0.30
BIN_SX, BIN_SY, BIN_SZ = 0.20, 0.20, 0.15
BIN_CZ = BIN_TOP_Z - BIN_SZ / 2.0

# ============================================================================
# HOME 位姿 —— 大臂后仰、安全回缩，远离桌子和料框
# 关节习惯（参考 massage demo）：j2 > 0 = 大臂后仰, j3 < 0 = 肘前弯
# ============================================================================
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# 水果列表（位置 + 半径）
FRUITS = [
    {"id": "fruit_apple",  "x": 0.50, "y": -0.20, "radius": 0.035,
     "color": (1.0, 0.15, 0.15), "label": "Apple"},
    # 该目标保持在当前 CAD 运动学模型经测试的有效 IK 区域内。
    {"id": "fruit_orange", "x": 0.55, "y": -0.05, "radius": 0.040,
     "color": (1.0, 0.55, 0.05), "label": "Orange"},
    {"id": "fruit_plum",   "x": 0.40, "y":  0.15, "radius": 0.030,
     "color": (0.60, 0.15, 0.60), "label": "Plum"},
    {"id": "fruit_lime",   "x": 0.60, "y": -0.08, "radius": 0.028,
     "color": (0.15, 0.80, 0.20), "label": "Lime"},
]


# ============================================================================
# 工具函数
# ============================================================================

def duration_msg(seconds: float) -> Duration:
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))

def duration_seconds(d: Duration) -> float:
    return float(d.sec) + float(d.nanosec) / 1e9

def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """从 RPY 构造 Quaternion（ROS 顺序: x,y,z,w）。"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.w = cr * cp * cy + sr * sp * sy
    return q


# ============================================================================
# 主节点
# ============================================================================

class PickPlaceDemo(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_demo")

        self.declare_parameter("marker_topic", "/rviz_visual_tools")
        self.declare_parameter("action_server_timeout_sec", 120.0)
        self.declare_parameter("hold_seconds", 8.0)

        marker_topic = self.get_parameter("marker_topic").value
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
        )
        # TF 诊断：监听 /tf 和 /tf_static
        self.tf_sub = self.create_subscription(
            TFMessage, "/tf", self._on_tf, 10,
        )
        # /tf_static 使用 TRANSIENT_LOCAL（latched），订阅端也必须匹配才能收到缓存消息
        self.tf_static_sub = self.create_subscription(
            TFMessage, "/tf_static", self._on_tf_static,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._tf_robot_frames: set[str] = set()     # /tf 上的动态连杆帧
        self._tf_static_frames: set[str] = set()     # /tf_static 上的固定连杆帧
        self._tf_msg_count = 0
        self._tf_last_time = self.get_clock().now()

        # --- 服务/动作客户端 ---
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene",
        )
        self.motion_plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path",
        )
        self.state_validity_client = self.create_client(
            GetStateValidity, "/check_state_validity",
        )
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik",
        )
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory",
        )

        self.current_joint_positions: dict[str, float] = {}
        self.fruit_states: dict[str, str] = {f["id"]: "free" for f in FRUITS}
        # 水果抓取时 fruit → gripper_tcp 的偏移量（世界坐标系），用于刚体附着模拟
        self.grasped_fruit_offsets: dict[str, tuple[float, float, float]] = {}
        # TF 监听器，用于实时查询 gripper_tcp 在世界坐标系中的位姿
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_timer = self.create_timer(0.2, self._publish_markers)
        # 每 3 秒打印一次当前关节角度，方便确认机械臂状态
        self.joint_monitor_timer = self.create_timer(3.0, self._print_joint_status)

    # ------------------------------------------------------------------
    # 关节状态回调
    # ------------------------------------------------------------------
    def _on_joint_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in ALL_JOINTS:
                self.current_joint_positions[name] = pos

    def _on_tf(self, msg: TFMessage) -> None:
        """监听 /tf（动态关节变换）。"""
        self._tf_msg_count += 1
        self._tf_last_time = self.get_clock().now()
        for t in msg.transforms:
            child = t.child_frame_id
            if child.startswith("/"):
                child = child[1:]
            if child.startswith("arm_link_") or child in ("tool_flange", "gripper_tcp"):
                self._tf_robot_frames.add(child)

    def _on_tf_static(self, msg: TFMessage) -> None:
        """监听 /tf_static（固定关节变换：world→base_link、tool_flange→gripper_tcp 等）。"""
        for t in msg.transforms:
            child = t.child_frame_id
            if child.startswith("/"):
                child = child[1:]
            # 收集所有固定帧，包括 base_link、CAD base、gripper 和 camera。
            self._tf_static_frames.add(child)

    def _print_joint_status(self) -> None:
        """每3秒打印当前关节角度 + TF 诊断，方便确认机械臂是否在运动。"""
        arm = self._arm_now()
        if arm is None:
            return
        parts = [f"J{i+1}: {arm[i]:+.3f}" for i in range(6)]
        self.get_logger().info(f"  角度: {' | '.join(parts)}")

        # TF 诊断
        if self._tf_msg_count == 0:
            self.get_logger().warn(f"  ⚠ /tf: 从未收到消息！robot_state_publisher 可能未启动")
        else:
            elapsed = (self.get_clock().now() - self._tf_last_time).nanoseconds / 1e9
            if elapsed > 2.0:
                self.get_logger().warn(
                    f"  ⚠ /tf: 上次收到 {elapsed:.1f}s 前 (共 {self._tf_msg_count} 条)"
                )
            dyn = sorted(self._tf_robot_frames)
            stc = sorted(self._tf_static_frames)
            self.get_logger().info(
                f"  TF: 动态={len(dyn)}帧({' '.join(dyn[:5])}{'...' if len(dyn)>5 else ''}) | "
                f"静态={len(stc)}帧({' '.join(stc[:8])}{'...' if len(stc)>8 else ''})"
            )
            if "base_link" not in self._tf_static_frames:
                self.get_logger().error(
                    "  ⚠ 致命: base_link 不在 /tf_static 中！RobotModel 无法找到机械臂根连杆"
                )

    def _arm_now(self) -> list[float] | None:
        try:
            return [self.current_joint_positions[j] for j in ARM_JOINTS]
        except KeyError:
            return None

    def _gripper_now(self) -> list[float] | None:
        try:
            return [self.current_joint_positions[j] for j in GRIPPER_JOINTS]
        except KeyError:
            return None

    def _log_joint_delta(self, label: str, before: list[float], after: list[float]) -> None:
        """打印关节角度变化，方便确认机械臂确实在运动。"""
        if before is None or after is None:
            return
        max_delta = max(abs(a - b) for a, b in zip(before, after))
        # 只打印变化最大的3个关节
        deltas = [(abs(a - b), i) for i, (a, b) in enumerate(zip(before, after))]
        deltas.sort(reverse=True)
        parts = []
        for d, i in deltas[:3]:
            if d > 0.001:  # 忽略微小变化
                parts.append(f"J{i+1}: {before[i]:+.3f}→{after[i]:+.3f} (Δ{d:.3f})")
        if parts:
            self.get_logger().info(f"  关节变化 [{label}]: {' | '.join(parts)}  maxΔ={max_delta:.3f}rad")
        else:
            self.get_logger().warn(f"  关节无变化 [{label}] maxΔ={max_delta:.4f}rad")

    # ==================================================================
    # IK 求解 —— 根据末端位姿计算关节角度
    # ==================================================================

    def _solve_ik(self, pose_stamped: PoseStamped,
                  seed: list[float] | None = None) -> list[float] | None:
        """
        调用 /compute_ik 服务，把 gripper_tcp 的 target pose 转成关节角度。
        返回 6 个关节值，失败返回 None。
        """
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("IK service not available")
            return None

        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = PLANNING_GROUP
        ik.pose_stamped = pose_stamped
        ik.ik_link_name = IK_LINK
        ik.timeout.sec = int(IK_TIMEOUT)
        ik.timeout.nanosec = int((IK_TIMEOUT - int(IK_TIMEOUT)) * 1e9)
        ik.avoid_collisions = True

        # 种子状态：用当前关节值，让 IK 找附近解
        if seed is None:
            seed = self._arm_now()
        if seed is not None:
            ik.robot_state.is_diff = True
            ik.robot_state.joint_state.name = ARM_JOINTS
            ik.robot_state.joint_state.position = seed

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=IK_TIMEOUT + 2.0)
        result = future.result()

        if result is None:
            self.get_logger().warn("IK: no response")
            return None
        if result.error_code.val != 1:   # 1 = SUCCESS
            self.get_logger().warn(f"IK failed: code={result.error_code.val}")
            return None

        sol = result.solution.joint_state
        joint_map = {n: p for n, p in zip(sol.name, sol.position)}
        try:
            return [joint_map[j] for j in ARM_JOINTS]
        except KeyError as e:
            self.get_logger().warn(f"IK solution missing joint: {e}")
            return None

    def _grasp_pose(self, fruit: dict, hover: bool = False) -> PoseStamped:
        """
        构造 gripper_tcp 的抓取/悬停位姿。

        抓取时 TCP 直接位于水果中心；悬停时沿 world +Z 抬高。
        姿态：Z 轴向下（指向桌面），偏航角对准水果方向。
        """
        fruit_z = TABLE_TOP_Z + fruit["radius"]
        tool_z = fruit_z
        if hover:
            tool_z += HOVER_OFFSET_Z

        yaw = math.atan2(fruit["y"], fruit["x"])

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = fruit["x"]
        ps.pose.position.y = fruit["y"]
        ps.pose.position.z = tool_z
        # 姿态：绕 X 旋转 π（Z 向下），再绕 Z 旋转 yaw 对准水果
        ps.pose.orientation = quaternion_from_rpy(math.pi, 0.0, yaw)
        return ps

    def _bin_pose(self, hover: bool = False) -> PoseStamped:
        """构造 gripper_tcp 在料框上方的位姿。"""
        yaw = math.atan2(BIN_Y, BIN_X)
        z = BIN_TOP_Z - BIN_SZ * 0.5 + 0.04  # 放入框内
        if hover:
            z = BIN_TOP_Z + HOVER_OFFSET_Z

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = BIN_X
        ps.pose.position.y = BIN_Y
        ps.pose.position.z = z
        ps.pose.orientation = quaternion_from_rpy(math.pi, 0.0, yaw)
        return ps

    # ==================================================================
    # 主流程
    # ==================================================================

    def run(self) -> bool:
        self.get_logger().info("=== Pick-and-Place Demo (IK-based) ===")

        if not self._wait_for_all():
            return False

        # 发布场景物体（桌子、料框、水果）
        if not self._apply_scene():
            return False

        # 手爪张开（初始状态已经是 HOME + open）
        self._send_gripper(GRIPPER_OPEN)

        # ---------- 逐个水果：IK 求解 → 规划 → 执行 ----------
        for fruit in FRUITS:
            fid = fruit["id"]
            if self.fruit_states.get(fid) == "placed":
                continue

            self.get_logger().info(f"--- {fruit['label']} ---")

            # a) 求解 hover 关节角（水果上方悬停）
            hover_pose = self._grasp_pose(fruit, hover=True)
            hover_joints = self._solve_ik(hover_pose)
            if hover_joints is None:
                self.get_logger().error(f"IK failed for hover_{fid}, skip")
                continue

            # b) 规划并执行到 hover
            if not self._move_joints_now(hover_joints, f"hover_{fid}"):
                continue

            # c) 求解 grasp 关节角（水果表面）
            grasp_pose = self._grasp_pose(fruit, hover=False)
            grasp_joints = self._solve_ik(grasp_pose)
            if grasp_joints is None:
                self.get_logger().error(f"IK failed for grasp_{fid}, skip")
                continue

            # d) 下降到抓取位置
            if not self._move_joints_now(grasp_joints, f"grasp_{fid}"):
                continue

            # e) 闭合手爪
            self.get_logger().info(f"抓取 {fruit['label']} ...")
            self._send_gripper(GRIPPER_CLOSED)
            self.fruit_states[fid] = "grasped"
            # 记录抓取瞬间 fruit → gripper_tcp 的偏移，用于刚体附着渲染
            gp = self._grasp_pose(fruit, hover=False)
            tfx, tfy, tfz = gp.pose.position.x, gp.pose.position.y, gp.pose.position.z
            fx, fy, fz = fruit["x"], fruit["y"], TABLE_TOP_Z + fruit["radius"]
            self.grasped_fruit_offsets[fid] = (fx - tfx, fy - tfy, fz - tfz)
            self._remove_fruit_scene(fid)   # 从碰撞场景中移除

            # f) 退回到 hover
            if not self._move_joints_now(hover_joints, f"retreat_{fid}"):
                continue

            # g) 移动到料框 hover
            bin_hover_pose = self._bin_pose(hover=True)
            bin_hover_joints = self._solve_ik(bin_hover_pose)
            if bin_hover_joints is None:
                self.get_logger().error(f"IK failed for bin_hover_{fid}, skip")
                continue
            if not self._move_joints_now(bin_hover_joints, f"bin_hover_{fid}"):
                continue

            # h) 下降放入料框
            bin_drop_pose = self._bin_pose(hover=False)
            bin_drop_joints = self._solve_ik(bin_drop_pose)
            if bin_drop_joints is None:
                self.get_logger().error(f"IK failed for bin_drop_{fid}, skip")
                continue
            if not self._move_joints_now(bin_drop_joints, f"bin_drop_{fid}"):
                continue

            # i) 松开
            self.get_logger().info(f"释放 {fruit['label']} ...")
            self._send_gripper(GRIPPER_OPEN)
            self.fruit_states[fid] = "placed"

            # j) 从料框撤回
            self._move_joints_now(bin_hover_joints, f"bin_retract_{fid}")

        # ---------- 回到 HOME ----------
        arm_now = self._arm_now()
        if arm_now:
            self._move_joints(arm_now, HOME, "return_home")

        self.get_logger().info("=== 全部完成 ===")
        self.create_timer(self.get_parameter("hold_seconds").value, self._shutdown)
        return True

    # ==================================================================
    # 关节空间运动：规划 → 校验 → 执行
    # ==================================================================

    def _move_joints(self, start: list[float], goal: list[float],
                     label: str) -> bool:
        traj = self._plan_joints(start, goal, label)
        if traj is None:
            return False
        if not self._validate(traj, label):
            return False
        before = self._arm_now()
        ok = self._execute(traj, label)
        if ok:
            after = self._arm_now()
            self._log_joint_delta(label, before, after)
        return ok

    def _move_joints_now(self, goal: list[float], label: str) -> bool:
        start = self._arm_now()
        if start is None:
            self.get_logger().error(f"No joint state for {label}")
            return False
        return self._move_joints(start, goal, label)

    # ==================================================================
    # 规划
    # ==================================================================

    def _plan_joints(self, start: list[float], goal: list[float],
                     label: str) -> JointTrajectory | None:
        req = GetMotionPlan.Request()
        mr = req.motion_plan_request
        mr.group_name = PLANNING_GROUP
        mr.planner_id = PLANNER_ID
        mr.num_planning_attempts = 8
        mr.allowed_planning_time = PLANNING_TIME
        mr.max_velocity_scaling_factor = VELOCITY_SCALING
        mr.max_acceleration_scaling_factor = ACCELERATION_SCALING

        mr.start_state.is_diff = True
        mr.start_state.joint_state = JointState()
        mr.start_state.joint_state.name = ARM_JOINTS
        mr.start_state.joint_state.position = start

        constraints = Constraints()
        for jn, jp in zip(ARM_JOINTS, goal):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = jp
            jc.tolerance_above = JOINT_TOLERANCE
            jc.tolerance_below = JOINT_TOLERANCE
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        mr.goal_constraints.append(constraints)

        future = self.motion_plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=12.0)
        result = future.result()

        if result is None:
            self.get_logger().error(f"No plan response for {label}")
            return None

        resp = result.motion_plan_response
        if resp.error_code.val != 1:
            self.get_logger().error(
                f"Plan failed {label}: code={resp.error_code.val}"
            )
            return None

        traj = resp.trajectory.joint_trajectory
        if not traj.points:
            self.get_logger().error(f"Empty trajectory for {label}")
            return None

        self.get_logger().info(
            f"Planned {label}: {len(traj.points)} pts, "
            f"{duration_seconds(traj.points[-1].time_from_start):.2f}s"
        )
        return traj

    # ==================================================================
    # 碰撞校验
    # ==================================================================

    def _validate(self, traj: JointTrajectory, label: str) -> bool:
        if not self.state_validity_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("State validity unavailable; skip check")
            return True

        total = duration_seconds(traj.points[-1].time_from_start)
        n = max(3, int(total / SAMPLE_PERIOD) + 1)
        for i in range(n + 1):
            t = min(i * SAMPLE_PERIOD, total)
            pos = self._sample(traj, t)
            req = GetStateValidity.Request()
            req.group_name = PLANNING_GROUP
            req.robot_state.is_diff = True
            req.robot_state.joint_state = JointState()
            req.robot_state.joint_state.name = ARM_JOINTS
            req.robot_state.joint_state.position = pos

            future = self.state_validity_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            r = future.result()
            if r is None:
                continue
            if not r.valid:
                cts = ", ".join(
                    f"{c.contact_body_1}<->{c.contact_body_2}"
                    for c in r.contacts[:4]
                )
                self.get_logger().error(
                    f"Collision {label} t={t:.2f}s [{i}/{n}]: {cts or 'invalid'}"
                )
                return False
        self.get_logger().info(f"Validated {label} ({n} samples)")
        return True

    @staticmethod
    def _sample(traj: JointTrajectory, t: float) -> list[float]:
        pts = traj.points
        if t <= duration_seconds(pts[0].time_from_start):
            return list(pts[0].positions)
        for i in range(1, len(pts)):
            t0 = duration_seconds(pts[i - 1].time_from_start)
            t1 = duration_seconds(pts[i].time_from_start)
            if t <= t1:
                if t1 <= t0:
                    return list(pts[i].positions)
                r = (t - t0) / (t1 - t0)
                return [pts[i - 1].positions[j] +
                        (pts[i].positions[j] - pts[i - 1].positions[j]) * r
                        for j in range(len(pts[i].positions))]
        return list(pts[-1].positions)

    # ==================================================================
    # 执行
    # ==================================================================

    def _execute(self, traj: JointTrajectory, label: str) -> bool:
        grip = self._gripper_now()
        if grip is None:
            grip = GRIPPER_OPEN
        jmap = {n: i for i, n in enumerate(traj.joint_names)}

        full = JointTrajectory()
        full.joint_names = ALL_JOINTS
        for pt in traj.points:
            fp = JointTrajectoryPoint()
            fp.positions = [
                pt.positions[jmap[j]] if j in jmap
                else grip[GRIPPER_JOINTS.index(j)]
                for j in ALL_JOINTS
            ]
            fp.time_from_start = pt.time_from_start
            full.points.append(fp)

        # --- 诊断：验证轨迹确实会让机械臂运动 ---
        current = self._arm_now()
        if current is not None and full.points:
            first_arm = [full.points[0].positions[ALL_JOINTS.index(j)] for j in ARM_JOINTS]
            last_arm = [full.points[-1].positions[ALL_JOINTS.index(j)] for j in ARM_JOINTS]
            first_delta = max(abs(a - b) for a, b in zip(first_arm, current))
            last_delta = max(abs(a - b) for a, b in zip(last_arm, current))
            dur = duration_seconds(full.points[-1].time_from_start)
            self.get_logger().info(
                f"  轨迹 {label}: {len(full.points)} pts, {dur:.2f}s, "
                f"起点Δ={first_delta:.4f}rad, 终点Δ={last_delta:.4f}rad"
            )
            if last_delta < 0.005:
                self.get_logger().error(
                    f"  ⚠ 轨迹退化 [{label}]: 终点与当前位置几乎相同 (maxΔ={last_delta:.4f}rad)!"
                )
            # 打印前 2 个点和最后 1 个点的关节值，方便对比
            for i in (0, 1, len(full.points) - 1):
                if i < len(full.points):
                    pt_arm = [full.points[i].positions[ALL_JOINTS.index(j)] for j in ARM_JOINTS]
                    t = duration_seconds(full.points[i].time_from_start)
                    parts = [f"J{j+1}:{v:+.3f}" for j, v in enumerate(pt_arm)]
                    self.get_logger().info(f"    pt[{i}] t={t:.2f}s: {' | '.join(parts)}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = full
        goal.goal_time_tolerance = duration_msg(0.8)

        f = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=60.0)
        gh = f.result()
        if not gh.accepted:
            self.get_logger().error(f"Goal rejected: {label}")
            return False

        self.get_logger().info(f"Executing {label} ({len(full.points)} pts)...")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=120.0)
        r = rf.result().result
        if r.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Exec failed {label}: {r.error_code} {r.error_string}"
            )
            return False
        self.get_logger().info(f"Done: {label}")
        return True

    def _send_gripper(self, pos: list[float]) -> None:
        arm = self._arm_now()
        if arm is None:
            return
        traj = JointTrajectory()
        traj.joint_names = ALL_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = list(arm) + list(pos)
        pt.time_from_start = duration_msg(1.0)
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = duration_msg(0.8)
        f = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=30.0)
        gh = f.result()
        if gh.accepted:
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=30.0)

    # ==================================================================
    # 场景管理
    # ==================================================================

    def _apply_scene(self) -> bool:
        if not self.apply_scene_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("Timeout /apply_planning_scene")
            return False

        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "pick_place_scene"
        co.operation = CollisionObject.ADD

        def bx(dims, x, y, z):
            p = SolidPrimitive()
            p.type = SolidPrimitive.BOX
            p.dimensions = dims
            ps = Pose()
            ps.orientation.w = 1.0
            ps.position.x = x
            ps.position.y = y
            ps.position.z = z
            co.primitives.append(p)
            co.primitive_poses.append(ps)

        # 桌面 + 4 条腿
        bx([TABLE_SX, TABLE_SY, TABLE_SZ], TABLE_CX, TABLE_CY, TABLE_CZ)
        lx, ly = TABLE_SX / 2 - LEG_SIZE, TABLE_SY / 2 - LEG_SIZE
        for sx in (-lx, lx):
            for sy in (-ly, ly):
                bx([LEG_SIZE, LEG_SIZE, LEG_H],
                   TABLE_CX + sx, TABLE_CY + sy, LEG_H / 2)

        # 料框（5 块薄壁）
        wt = 0.01
        hx, hy = BIN_SX / 2, BIN_SY / 2
        wz = BIN_TOP_Z - BIN_SZ / 2
        bx([BIN_SX, BIN_SY, wt],         BIN_X, BIN_Y, BIN_TOP_Z - BIN_SZ)
        bx([wt, BIN_SY, BIN_SZ],         BIN_X - hx, BIN_Y, wz)
        bx([wt, BIN_SY, BIN_SZ],         BIN_X + hx, BIN_Y, wz)
        bx([BIN_SX, wt, BIN_SZ],         BIN_X, BIN_Y - hy, wz)
        bx([BIN_SX, wt, BIN_SZ],         BIN_X, BIN_Y + hy, wz)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)

        for fruit in FRUITS:
            fo = CollisionObject()
            fo.header.frame_id = "world"
            fo.id = fruit["id"]
            fo.operation = CollisionObject.ADD
            sp = SolidPrimitive()
            sp.type = SolidPrimitive.SPHERE
            sp.dimensions = [fruit["radius"]]
            fp = Pose()
            fp.orientation.w = 1.0
            fp.position.x = fruit["x"]
            fp.position.y = fruit["y"]
            fp.position.z = TABLE_TOP_Z + fruit["radius"]
            fo.primitives.append(sp)
            fo.primitive_poses.append(fp)
            scene.world.collision_objects.append(fo)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        r = fut.result()
        if r is None or not r.success:
            self.get_logger().error("Apply scene failed")
            return False
        self.get_logger().info(f"Scene: table + bin + {len(FRUITS)} fruits")
        return True

    def _remove_fruit_scene(self, fid: str) -> None:
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = fid
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    # ==================================================================
    # 启动
    # ==================================================================

    def _wait_for_all(self) -> bool:
        to = self.get_parameter("action_server_timeout_sec").value
        for lb, cl in (
            ("apply_scene", self.apply_scene_client),
            ("plan", self.motion_plan_client),
            ("validity", self.state_validity_client),
            ("ik", self.ik_client),
        ):
            if not cl.wait_for_service(timeout_sec=to):
                self.get_logger().error(f"Timeout service {lb}")
                return False
        if not self.arm_client.wait_for_server(timeout_sec=to):
            self.get_logger().error("Timeout arm_controller action")
            return False
        self.get_logger().info("Services ready.")

        deadline = self.get_clock().now().nanoseconds / 1e9 + 30.0
        self.get_logger().info("Waiting for /joint_states ...")
        while rclpy.ok():
            if self._arm_now() and self._gripper_now():
                self.get_logger().info("Joints OK.")
                return True
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                self.get_logger().error("Timeout joint_states")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    # ==================================================================
    # RViz 可视化标记
    # ==================================================================

    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # 桌子（棕色半透明）
        ma.markers.append(self._cube(
            now, 1, "scene", TABLE_CX, TABLE_CY, TABLE_CZ,
            TABLE_SX, TABLE_SY, TABLE_SZ, (0.50, 0.35, 0.22, 0.85),
        ))
        # 料框
        ma.markers.append(self._cube(
            now, 2, "scene", BIN_X, BIN_Y, BIN_CZ,
            BIN_SX, BIN_SY, BIN_SZ, (0.25, 0.40, 0.60, 0.65),
        ))

        # ── 查询 gripper_tcp 当前位姿（用于附着被抓水果）──
        tf_pos = None  # (x, y, z) in world frame
        try:
            t = self.tf_buffer.lookup_transform(
                "world", "gripper_tcp", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            tf_pos = (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            )
        except TransformException:
            pass  # TF 还没准备好，稍后重试

        # 水果
        for i, fr in enumerate(FRUITS):
            fid = fr["id"]
            st = self.fruit_states.get(fid, "free")
            r, g, b = fr["color"]

            if st == "free":
                # 自由状态：渲染在桌面原位，明亮可见
                x, y, z = fr["x"], fr["y"], TABLE_TOP_Z + fr["radius"]
                a = 0.90
                m = self._sphere(now, 10 + i, "fruit", x, y, z,
                                 fr["radius"], (r, g, b, a))
                ma.markers.append(m)

            elif st == "grasped":
                # 被抓取：附着在 gripper_tcp 上，跟随机械臂运动
                if tf_pos is not None:
                    ox, oy, oz = self.grasped_fruit_offsets.get(
                        fid, (0.0, 0.0, 0.0 + fr["radius"]))
                    x, y, z = tf_pos[0] + ox, tf_pos[1] + oy, tf_pos[2] + oz
                else:
                    x, y, z = fr["x"], fr["y"], TABLE_TOP_Z + fr["radius"]
                a = 0.90
                m = self._sphere(now, 10 + i, "fruit", x, y, z,
                                 fr["radius"], (r, g, b, a))
                ma.markers.append(m)

            elif st == "placed":
                # 已放入料框：渲染在料框底部，降低 alpha 表示已完成
                x, y = BIN_X, BIN_Y
                z = BIN_TOP_Z - BIN_SZ + fr["radius"] + 0.01
                a = 0.55
                m = self._sphere(now, 10 + i, "fruit", x, y, z,
                                 fr["radius"], (r, g, b, a))
                ma.markers.append(m)

        self.marker_pub.publish(ma)

    @staticmethod
    def _cube(now, mid, ns, x, y, z, sx, sy, sz, color):
        m = Marker()
        m.header.frame_id = "world"; m.header.stamp = now
        m.ns = ns; m.id = mid; m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position = Point(x=x, y=y, z=z)
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=sx, y=sy, z=sz)
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    @staticmethod
    def _sphere(now, mid, ns, x, y, z, r, color):
        m = Marker()
        m.header.frame_id = "world"; m.header.stamp = now
        m.ns = ns; m.id = mid; m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position = Point(x=x, y=y, z=z)
        m.pose.orientation.w = 1.0
        d = r * 2.0; m.scale = Vector3(x=d, y=d, z=d)
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def _shutdown(self) -> None:
        self.get_logger().info("Shutdown.")
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = PickPlaceDemo()
    try:
        if not node.run():
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
