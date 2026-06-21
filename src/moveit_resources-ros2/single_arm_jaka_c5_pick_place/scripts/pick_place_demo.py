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
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
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

IK_LINK = "tool_flange"
IK_TIMEOUT = 0.5          # IK 求解超时 (秒)
HOVER_OFFSET_Z = 0.12     # 悬停高度（tool_flange 在水果上方）

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
HOME = [0.0, 1.50, -1.50, 1.50, 1.57, 0.0]

# 水果列表（位置 + 半径）
FRUITS = [
    {"id": "fruit_apple",  "x": 0.50, "y": -0.20, "radius": 0.035,
     "color": (1.0, 0.15, 0.15), "label": "Apple"},
    {"id": "fruit_orange", "x": 0.70, "y":  0.10, "radius": 0.040,
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
        self.marker_timer = self.create_timer(0.2, self._publish_markers)

    # ------------------------------------------------------------------
    # 关节状态回调
    # ------------------------------------------------------------------
    def _on_joint_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in ALL_JOINTS:
                self.current_joint_positions[name] = pos

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

    # ==================================================================
    # IK 求解 —— 根据末端位姿计算关节角度
    # ==================================================================

    def _solve_ik(self, pose_stamped: PoseStamped,
                  seed: list[float] | None = None) -> list[float] | None:
        """
        调用 /compute_ik 服务，把 tool_flange 的 target pose 转成关节角度。
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
        构造 tool_flange 的抓取/悬停位姿。

        手指指尖到 tool_flange 原点约 0.086 m（gripper_base 厚度 0.006 + 手指长度 0.08）。
        抓取时 tool_flange 放在水果中心上方 0.086 m 处。
        悬停时在此基础上再抬高 HOVER_OFFSET_Z。
        姿态：Z 轴向下（指向桌面），偏航角对准水果方向。
        """
        fruit_z = TABLE_TOP_Z + fruit["radius"]
        tool_z = fruit_z + 0.086          # tool_flange 在指尖上方
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
        """构造 tool_flange 在料框上方的位姿。"""
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
        return self._execute(traj, label)

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

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = full
        goal.goal_time_tolerance = duration_msg(0.8)

        f = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=60.0)
        gh = f.result()
        if not gh.accepted:
            self.get_logger().error(f"Goal rejected: {label}")
            return False

        self.get_logger().info(f"Executing {label} ...")
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

        # 桌子
        ma.markers.append(self._cube(
            now, 1, "sc", TABLE_CX, TABLE_CY, TABLE_CZ,
            TABLE_SX, TABLE_SY, TABLE_SZ, (0.55, 0.44, 0.32, 0.7),
        ))
        # 料框
        ma.markers.append(self._cube(
            now, 2, "sc", BIN_X, BIN_Y, BIN_CZ,
            BIN_SX, BIN_SY, BIN_SZ, (0.25, 0.40, 0.60, 0.6),
        ))
        # 水果
        for i, fr in enumerate(FRUITS):
            st = self.fruit_states.get(fr["id"], "free")
            a = 0.35 if st == "grasped" else (0.22 if st == "placed" else 0.85)
            r, g, b = fr["color"]
            ma.markers.append(self._sphere(
                now, 10 + i, "fr",
                fr["x"], fr["y"], TABLE_TOP_Z + fr["radius"], fr["radius"],
                (r, g, b, a),
            ))
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
