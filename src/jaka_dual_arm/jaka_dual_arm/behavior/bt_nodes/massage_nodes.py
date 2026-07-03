#!/usr/bin/env python3
"""按摩专用 BT 节点 — BehaviorTree.CPP 兼容.

5 个节点:
    WaitServices       — BtCondition: 等待 MoveIt + 控制器服务就绪
    SetupMassageScene  — BtActionNode: 注册床+人体碰撞对象
    InitSurfaceModel   — BtActionNode: 加载 YAML → 构建曲面模型 + 路径生成器
    RunMassageCycle    — BtActionNode: 核心 60 阶段迭代 (生成→规划→执行)
    RetreatToHome      — BtActionNode: 双臂回到安全初始位姿

节点间通过 blackboard 共享状态:
    blackboard["surface"]         — BackSurfaceModel
    blackboard["path_generator"]  — PathGenerator
    blackboard["planner"]         — DualArmPlannerServer
    blackboard["stages"]          — 60 阶段定义列表
    blackboard["body_config"]     — 人体模型 YAML
    blackboard["safety"]          — SafetyMonitor (可选)
    blackboard["impedance"]       — VirtualImpedanceController (可选)

参考:
  - behavior/bt_nodes/bt_node_base.py — BtNode/BtActionNode/BtCondition/BtAsyncNode
  - behavior/bt_nodes/carry_nodes.py  — 搬运 BT 节点 (参考模式)
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from jaka_dual_arm.behavior.bt_nodes.bt_node_base import (
    NodeStatus, BtActionNode, BtCondition, BtAsyncNode,
)
from jaka_dual_arm.skills.path_generator import (
    BackSurfaceModel, PathGenerator, TECHNIQUE_CONFIG,
)

# ── Choreographer imports (v6: skill primitive library) ──
try:
    from jaka_dual_arm.massage.skill_primitives import SkillLibrary
    from jaka_dual_arm.massage.choreographer import (
        MassageChoreographer, StageDef, StageKind,
    )
    _CHOREOGRAPHER_AVAILABLE = True
except ImportError:
    _CHOREOGRAPHER_AVAILABLE = False

# ── Executor-aware spin helper ──────────────────────────────────

def _duration_seconds(d: "Duration") -> float:
    """Convert builtin_interfaces/msg/Duration to seconds."""
    return float(d.sec) + float(d.nanosec) / 1e9


def _duration_msg(seconds: float) -> Duration:
    seconds = max(0.0, float(seconds))
    msg = Duration()
    msg.sec = int(seconds)
    msg.nanosec = int(round((seconds - msg.sec) * 1e9))
    if msg.nanosec >= 1_000_000_000:
        msg.sec += 1
        msg.nanosec -= 1_000_000_000
    return msg


def _spin_future(blackboard: dict, future, timeout_sec: float = 5.0):
    """Spin until future completes, using the MultiThreadedExecutor from blackboard.

    This ensures BOTH massage_runner AND planner nodes get their callbacks
    processed, which is essential when the future belongs to a client on the
    planner node (e.g., /apply_planning_scene, /plan_kinematic_path).
    """
    executor = blackboard.get("executor")
    if executor is not None:
        executor.spin_until_future_complete(future, timeout_sec)
    else:
        # Fallback: spin just the runner node (will miss planner callbacks!)
        node = blackboard.get("node")
        if node is not None:
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=timeout_sec
            )


def _planner_float_param(blackboard: dict, name: str, default: float) -> float:
    planner = blackboard.get("planner")
    if planner is None or not hasattr(planner, "get_parameter"):
        return default
    try:
        return float(planner.get_parameter(name).value)
    except Exception:
        return default


def _nearest_angle(target: float, reference: float) -> float:
    return reference + math.atan2(
        math.sin(target - reference),
        math.cos(target - reference),
    )


# Joint names for left/right arms
LEFT_JOINTS = [
    "left_joint_1", "left_joint_2", "left_joint_3",
    "left_joint_4", "left_joint_5", "left_joint_6",
]
RIGHT_JOINTS = [
    "right_joint_1", "right_joint_2", "right_joint_3",
    "right_joint_4", "right_joint_5", "right_joint_6",
]
ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS

# Home joint positions (hover悬停, Link_06垂直向下)
HOME_LEFT = [0.0, 0.75, -1.10, 1.30, 1.57, 1.50]
HOME_RIGHT = [0.0, 0.75, -1.10, 1.30, 1.57, 1.50]


# ═══════════════════════════════════════════════════════════════
# WaitServices
# ═══════════════════════════════════════════════════════════════

class WaitServices(BtCondition):
    """等待 MoveIt 规划服务和控制器 Action 服务就绪。"""

    def __init__(self, name: str = "WaitServices"):
        super().__init__(name)
        self._timeout = 30.0
        self._start_time: Optional[float] = None
        self._all_ready_logged = False
        self._ctrl_clients = {}  # cached action clients for controller check

    def on_start(self):
        self._start_time = time.time()

    def evaluate(self) -> bool:
        node: Optional[Node] = self.blackboard.get("node")
        if node is None:
            return False

        elapsed = time.time() - self._start_time

        # Check MoveIt planning service
        planner = self.blackboard.get("planner")
        if planner is not None:
            if not planner._motion_plan_client.wait_for_service(timeout_sec=0.1):
                if elapsed > self._timeout:
                    node.get_logger().error(
                        f"Timeout waiting for /plan_kinematic_path ({self._timeout}s)"
                    )
                    return False
                return False

            # Also check apply_planning_scene
            if not planner._apply_scene_client.wait_for_service(timeout_sec=0.1):
                if elapsed > self._timeout:
                    node.get_logger().error("Timeout waiting for /apply_planning_scene")
                    return False
                return False

        # Check controller action servers (cache clients to avoid recreating each tick)
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        for arm in ["left", "right"]:
            controller = f"/{arm}_arm_controller/follow_joint_trajectory"
            if arm not in self._ctrl_clients:
                self._ctrl_clients[arm] = ActionClient(
                    node, FollowJointTrajectory, controller
                )
            if not self._ctrl_clients[arm].wait_for_server(timeout_sec=0.1):
                if elapsed > self._timeout:
                    node.get_logger().error(
                        f"Timeout waiting for {controller} ({self._timeout}s)"
                    )
                    return False
                return False

        if not self._all_ready_logged:
            node.get_logger().info("All services ready (MoveIt + controllers).")
            self._all_ready_logged = True
        return True


# ═══════════════════════════════════════════════════════════════
# SetupMassageScene
# ═══════════════════════════════════════════════════════════════

class SetupMassageScene(BtActionNode):
    """向 PlanningScene 注册床和人体碰撞对象。

    从 body_config 读取床体尺寸和人体段参数，构建 CollisionObject 并发布。
    """

    def __init__(self, name: str = "SetupMassageScene"):
        super().__init__(name)

    def execute(self) -> NodeStatus:
        node: Optional[Node] = self.blackboard.get("node")
        body_cfg: Optional[dict] = self.blackboard.get("body_config")
        if node is None or body_cfg is None:
            return NodeStatus.FAILURE

        try:
            self._setup_bed(node, body_cfg)  # now handles bed + body in one call
            node.get_logger().info("Massage scene registered.")
            return NodeStatus.SUCCESS
        except Exception as e:
            node.get_logger().error(f"Failed to setup scene: {e}")
            import traceback
            node.get_logger().error(traceback.format_exc())
            return NodeStatus.FAILURE

    def _setup_bed(self, node: Node, cfg: dict):
        """注册床框 + 床垫 + 人体碰撞对象到 PlanningScene。

        Gazebo 提供真实视觉模型，此方法仅负责 MoveIt 规划所需的碰撞几何体。
        """
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene
        from shape_msgs.msg import SolidPrimitive

        objects = []
        bed = cfg.get("bed", {})

        # Bed frame
        frame = bed.get("frame", {})
        frame_size = frame.get("size", {"x": 1.20, "y": 0.66, "z": 0.08})
        frame_bottom = frame.get("bottom_z", 0.0)
        frame_z = frame_bottom + frame_size["z"] / 2.0
        obj = CollisionObject()
        obj.id = "bed_frame"
        obj.header.frame_id = "world"
        obj.operation = CollisionObject.ADD
        obj.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[frame_size["x"], frame_size["y"], frame_size["z"]],
        ))
        obj.primitive_poses.append(_make_pose_msg(
            bed["center"]["x"], bed["center"]["y"], frame_z))
        objects.append(obj)

        # Mattress
        mattress = bed.get("mattress", {})
        mat_size = mattress.get("size", {"x": 1.12, "y": 0.56, "z": 0.06})
        mat_bottom = mattress.get("bottom_z", 0.08)
        mat_top = mat_bottom + mat_size["z"]  # mattress top surface (z=0.14)
        mat_z = mat_bottom + mat_size["z"] / 2.0
        obj2 = CollisionObject()
        obj2.id = "mattress"
        obj2.header.frame_id = "world"
        obj2.operation = CollisionObject.ADD
        obj2.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[mat_size["x"], mat_size["y"], mat_size["z"]],
        ))
        obj2.primitive_poses.append(_make_pose_msg(
            bed["center"]["x"], bed["center"]["y"], mat_z))
        objects.append(obj2)

        # Body segments — placed ON mattress, not sunken in
        self._add_body_objects(cfg, objects, mat_top)

        # Apply all collision objects in ONE request
        self._apply_scene(node, objects)

    def _add_body_objects(self, cfg: dict, objects: list, mat_top: float):
        """添加简化人体碰撞模型 — 用于 MoveIt 快速规划。

        碰撞模型保持简单（几个大形状），Gazebo 负责真实视觉和物理碰撞。
        重要：所有碰撞体底部必须 ≥ mat_top (床垫顶 z=0.14)，不能陷到床里。
        """
        import math
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive

        def _add_box(oid, cx, cy, cz, sx, sy, sz):
            obj = CollisionObject(); obj.id = oid; obj.header.frame_id = "world"
            obj.operation = CollisionObject.ADD
            obj.primitives.append(SolidPrimitive(
                type=SolidPrimitive.BOX, dimensions=[sx, sy, sz]))
            obj.primitive_poses.append(_make_pose_msg(cx, cy, cz))
            objects.append(obj)

        def _add_sphere(oid, cx, cy, cz, r):
            obj = CollisionObject(); obj.id = oid; obj.header.frame_id = "world"
            obj.operation = CollisionObject.ADD
            obj.primitives.append(SolidPrimitive(
                type=SolidPrimitive.SPHERE, dimensions=[r]))
            obj.primitive_poses.append(_make_pose_msg(cx, cy, cz))
            objects.append(obj)

        def _add_cylinder(oid, cx, cy, cz, height, radius,
                          qx=0.0, qy=0.0, qz=0.0, qw=1.0):
            obj = CollisionObject(); obj.id = oid; obj.header.frame_id = "world"
            obj.operation = CollisionObject.ADD
            obj.primitives.append(SolidPrimitive(
                type=SolidPrimitive.CYLINDER, dimensions=[height, radius]))
            obj.primitive_poses.append(_make_pose_msg(cx, cy, cz, qx, qy, qz, qw))
            objects.append(obj)

        segments = cfg.get("body_segments", [])
        SQRT2_2 = 0.70710678

        # ═══════════════════════════════════════════════════════
        # 5段阶梯躯干碰撞体 — 跟随脊柱z曲线变化，替代原来的单平板 box
        # 每段 top = z_surface (体表), bottom = mat_top (床垫顶)
        # Y宽度按 half_w 缩窄：颈/C7窄→肩胛宽→腰骶窄 = 正确沙漏轮廓
        # ═══════════════════════════════════════════════════════
        for oid, cx, yw, z_surf, xl in [
            # (id,   x中心, Y全宽, z体表,  X长度)
            # z_surf 取值参考 massage.world 对应区域圆柱顶面 (back_08~back_32):
            #   C7 x=0.39 → back_08 surface=0.207
            #   shoulder x=0.47 → back_12 surface=0.233 (肩峰最高)
            #  上背 x=0.57 → back_17 surface=0.223
            #  中背 x=0.67 → back_22 surface=0.196
            #  腰骶 x=0.79 → back_28 surface=0.183
            # 这里取保守值（比 Gazebo 低 0.5-1.8cm 以免阻挡规划路径）
            # 但必须高于旧值且肩>颈以确保 RViz 视觉不"陷床"。
            ("torso_c7",  0.380, 0.240, 0.205, 0.10),  # C7/颈椎区
            ("torso_sho", 0.470, 0.360, 0.220, 0.10),  # 肩胛区 (最宽最高)
            ("torso_up",  0.575, 0.340, 0.210, 0.12),  # 上背 T1-8
            ("torso_mid", 0.670, 0.280, 0.196, 0.12),  # 中背 T9-L1
            ("torso_lum", 0.790, 0.290, 0.183, 0.14),  # 腰骶区
        ]:
            h = z_surf - mat_top         # 碰撞体高度 = 体表 - 床垫顶
            zc = mat_top + h / 2         # 中心z
            _add_box(oid, cx, 0.0, zc, xl, yw, h)

        # ═══════════════════════════════════════════════════════
        # 头部: x=0.13 (比 back_00 x=0.23 更靠床头)，彻底不阻挡 C7 区
        # 右臂从 x=0.69/y=0.45 向内规划时，到 C7 x=0.31 的路径会绕过头部
        # 颈圆柱小到不遮挡按摩区，主要用于防止机械臂碰触枕骨区
        # ═══════════════════════════════════════════════════════
        _add_sphere("head", 0.13, 0.0, 0.23, 0.055)

        # 颈: 短圆柱（仅防碰，不阻挡 C7）
        _add_cylinder("neck", 0.26, 0.0, 0.195, 0.030, 0.022)

        # ═══════════════════════════════════════════════════════
        # 双臂: 贴体侧俯卧位，沿X轴水平 (qy=0.707, qw=0.707 = 绕Y轴转90°)
        # ═══════════════════════════════════════════════════════
        for side, sy in [("left", -1.0), ("right", 1.0)]:
            # 上臂: 肩x≈0.43, 肘x≈0.62, 中心x≈0.525, length=0.19
            # y=±0.29: 比肩部最宽背部边缘(±0.2075)更靠外，不与背部重叠
            _add_cylinder(f"{side}_upper_arm", 0.525, sy * 0.29, 0.17,
                          0.19, 0.030, 0.0, 0.707, 0.0, 0.707)
            _add_sphere(f"{side}_elbow", 0.62, sy * 0.295, 0.165, 0.032)
            # 前臂: 肘x≈0.62, 腕x≈0.78, 中心x≈0.70, length=0.16
            _add_cylinder(f"{side}_forearm", 0.700, sy * 0.30, 0.16,
                          0.16, 0.028, 0.0, 0.707, 0.0, 0.707)
            _add_sphere(f"{side}_wrist", 0.78, sy * 0.305, 0.155, 0.026)
            _add_sphere(f"{side}_hand", 0.79, sy * 0.31, 0.16, 0.035)

        # ═══════════════════════════════════════════════════════
        # 双腿: 沿X轴水平躺平 (qy=0.707, qw=0.707 = 绕Y轴转90°)
        # ═══════════════════════════════════════════════════════
        for side, sy in [("left", -1.0), ("right", 1.0)]:
            # 大腿: 髋x≈0.86, 膝x≈1.10, 中心x≈0.98, length=0.24
            _add_cylinder(f"{side}_thigh", 0.98, sy * 0.09, 0.17,
                          0.24, 0.042, 0.0, 0.707, 0.0, 0.707)
            _add_sphere(f"{side}_knee", 1.10, sy * 0.095, 0.165, 0.043)
            # 小腿: 膝x≈1.10, 踝x≈1.32, 中心x≈1.21, length=0.22
            _add_cylinder(f"{side}_calf", 1.21, sy * 0.10, 0.16,
                          0.22, 0.038, 0.0, 0.707, 0.0, 0.707)
            _add_sphere(f"{side}_ankle", 1.32, sy * 0.095, 0.150, 0.034)
            _add_box(f"{side}_foot", 1.37, sy * 0.09, 0.150,
                     0.13, 0.065, 0.035)
            _add_sphere(f"{side}_toe", 1.43, sy * 0.09, 0.148, 0.032)

    def _apply_scene(self, node: Node, objects: list):
        """Apply collision objects via /apply_planning_scene service with retry."""
        from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
        from moveit_msgs.srv import ApplyPlanningScene
        from std_msgs.msg import ColorRGBA

        planner = self.blackboard.get("planner")
        if planner is None:
            node.get_logger().error("No planner, cannot apply scene")
            return

        # Wait for service
        if not planner._apply_scene_client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(
                "/apply_planning_scene not available after 5s — scene NOT applied"
            )
            return

        # 人体肤色 vs 床体白蓝色区分
        BED_IDS = {"bed_frame", "mattress"}
        colors = []
        for obj in objects:
            oc = ObjectColor()
            oc.id = obj.id
            if obj.id in BED_IDS:
                oc.color = ColorRGBA(r=0.75, g=0.78, b=0.85, a=0.9)
            else:
                oc.color = ColorRGBA(r=0.88, g=0.72, b=0.60, a=0.9)
            colors.append(oc)

        stale_torso = CollisionObject()
        stale_torso.id = "torso"
        stale_torso.header.frame_id = "world"
        stale_torso.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.world.collision_objects = [stale_torso] + objects
        scene.object_colors = colors
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = planner._apply_scene_client.call_async(req)
        _spin_future(self.blackboard, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            node.get_logger().info(
                f"Scene applied: {len(objects)} collision objects registered"
            )
        else:
            node.get_logger().error("ApplyPlanningScene failed")


def _make_pose_msg(x: float, y: float, z: float,
                   qx: float = 0.0, qy: float = 0.0,
                   qz: float = 0.0, qw: float = 1.0) -> Pose:
    from geometry_msgs.msg import Point, Quaternion
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


# ═══════════════════════════════════════════════════════════════
# InitSurfaceModel
# ═══════════════════════════════════════════════════════════════

class InitSurfaceModel(BtActionNode):
    """从 body_config 构建 Catmull-Rom 样条曲面模型 + 路径生成器。

    写入 blackboard:
        blackboard["surface"]         — BackSurfaceModel 实例
        blackboard["path_generator"]  — PathGenerator 实例
    """

    def __init__(self, name: str = "InitSurfaceModel"):
        super().__init__(name)

    def execute(self) -> NodeStatus:
        node: Optional[Node] = self.blackboard.get("node")
        body_cfg: Optional[dict] = self.blackboard.get("body_config")

        if node is None or body_cfg is None:
            return NodeStatus.FAILURE

        try:
            surface = BackSurfaceModel(body_cfg)
            self.blackboard["surface"] = surface
            self.blackboard["path_generator"] = PathGenerator(surface)

            node.get_logger().info(
                f"Surface model initialized: "
                f"{len(body_cfg.get('body_segments', []))} segments, "
                f"spine arc={surface.spine.total_arc_length:.3f}m, "
                f"7 zones, {len(body_cfg.get('acupoints', []))} acupoints"
            )
            return NodeStatus.SUCCESS
        except Exception as e:
            node.get_logger().error(f"Failed to init surface model: {e}")
            return NodeStatus.FAILURE


# ═══════════════════════════════════════════════════════════════
# RunMassageCycle
# ═══════════════════════════════════════════════════════════════

class RunMassageCycle(BtActionNode):
    """核心: 遍历 60 阶段，每阶段 生成路径 → 规划 → 执行。

    单臂操作: 另一臂保持当前位置 (hover)
    双臂操作: 先规划冲突检测，无冲突则并行，有冲突则串行
    每阶段间: SafetyMonitor 检查 (若 blackboard 中存在)
    """

    def __init__(self, name: str = "RunMassageCycle"):
        super().__init__(name)
        self._traj_index = 0

    def execute(self) -> NodeStatus:
        node: Optional[Node] = self.blackboard.get("node")
        stages: Optional[List[dict]] = self.blackboard.get("stages")
        generator: Optional[PathGenerator] = self.blackboard.get("path_generator")
        planner = self.blackboard.get("planner")
        safety = self.blackboard.get("safety")

        if not all([node, stages, generator]):
            node.get_logger().error("Missing blackboard: node/stages/generator") \
                if node else None
            return NodeStatus.FAILURE

        total = len(stages)
        node.get_logger().info(f"Starting massage cycle: {total} stages (infinite loop)")

        cycle = 0
        while rclpy.ok():
            cycle += 1
            node.get_logger().info(
                f"╔══ Massage Cycle {cycle} ═══════════════════════════"
            )

            for idx, stage in enumerate(stages):
                # Safety check — WARN/SLOW: log only; HALT: skip stage; ESTOP: abort
                if safety is not None:
                    from jaka_dual_arm.control.safety_monitor import SafetyLevel
                    level = safety.check()
                    if level.value >= SafetyLevel.ESTOP.value:
                        node.get_logger().error(
                            f"Safety ESTOP at cycle {cycle} stage {idx + 1}: {level}"
                        )
                        return NodeStatus.FAILURE
                    elif level.value >= SafetyLevel.HALT.value:
                        node.get_logger().warn(
                            f"Safety HALT at stage {idx + 1}, skipping stage (继续循环)"
                        )
                        continue

                stage_id = stage.get("id", idx + 1)
                stage_name = stage.get("name", f"Stage {stage_id}")
                node.get_logger().info(
                    f"[C{cycle} {idx + 1}/{total}] {stage_name}"
                )

                try:
                    left_def = stage.get("left", {})
                    right_def = stage.get("right", {})

                    # ── 双臂并行: 同时规划 → 同时执行 ──
                    # 规划阶段从同一 /joint_states 快照出发,
                    # MoveIt 在各自规划组内独立求解, 避免臂间干涉.
                    left_poses = generator.generate(left_def) if left_def else []
                    right_poses = generator.generate(right_def) if right_def else []
                    left_traj, right_traj = self._plan_coordinated_stage(
                        node,
                        planner,
                        left_poses,
                        right_poses,
                        left_def.get("technique", "hover"),
                        right_def.get("technique", "hover"),
                    )

                    # 并行执行: 同时发送, 同时等待完成
                    if left_traj or right_traj:
                        if not self._execute_dual_arm(
                            node, left_traj, right_traj,
                            left_def.get("technique", "hover"),
                            right_def.get("technique", "hover")
                        ):
                            return NodeStatus.FAILURE

                except Exception as e:
                    node.get_logger().error(
                        f"Cycle {cycle} Stage {stage_id} failed: {e}"
                    )
                    return NodeStatus.FAILURE

                # Allow ROS to process callbacks (use executor to spin both nodes)
                executor = self.blackboard.get("executor")
                if executor is not None:
                    executor.spin_once(timeout_sec=0.01)
                else:
                    rclpy.spin_once(node, timeout_sec=0.01)

            node.get_logger().info(
                f"╚══ Cycle {cycle} complete ({total} stages) — continuing ══"
            )

        # Should never reach here unless rclpy is shutting down
        node.get_logger().info("Massage loop terminated (rclpy shutdown).")
        return NodeStatus.SUCCESS

    def _plan_coordinated_stage(
        self,
        node: Node,
        planner,
        left_poses: List[Pose],
        right_poses: List[Pose],
        left_technique: str,
        right_technique: str,
        left_start: Optional[List[float]] = None,
        right_start: Optional[List[float]] = None,
    ) -> tuple[Optional[JointTrajectory], Optional[JointTrajectory]]:
        """Plan a massage stage with coordinated dual-arm planning when possible."""
        left_traj: Optional[JointTrajectory] = None
        right_traj: Optional[JointTrajectory] = None
        enable_dual_pose = False
        if planner is not None and hasattr(planner, "get_parameter"):
            try:
                enable_dual_pose = bool(
                    planner.get_parameter("enable_dual_pose_planning").value
                )
            except Exception:
                enable_dual_pose = False

        can_dual_pose = (
            planner is not None
            and enable_dual_pose
            and left_poses
            and right_poses
            and len(left_poses) == 1
            and len(right_poses) == 1
            and left_start is None
            and right_start is None
            and hasattr(planner, "plan_dual_pose_target")
            and hasattr(planner, "split_dual_trajectory")
        )
        if can_dual_pose:
            def shifted(src: Pose, dz: float) -> Pose:
                pose = Pose()
                pose.position = Point(
                    x=src.position.x,
                    y=src.position.y,
                    z=src.position.z + dz,
                )
                pose.orientation = src.orientation
                return pose

            attempts = [
                (0.0, 0.0, "direct"),
                (0.06, 0.06, "6cm/6cm"),
                (0.12, 0.12, "12cm/12cm"),
                (0.06, 0.12, "6cm/12cm"),
                (0.12, 0.06, "12cm/6cm"),
            ]
            for left_dz, right_dz, label in attempts:
                node.get_logger().info(
                    f"coordinated dual plan: {left_technique}/{right_technique} "
                    f"from {label}"
                )
                dual_traj = planner.plan_dual_pose_target(
                    shifted(left_poses[0], left_dz),
                    shifted(right_poses[0], right_dz),
                )
                if dual_traj is None or not dual_traj.points:
                    continue
                if hasattr(planner, "validate_trajectory_collision_free"):
                    if not planner.validate_trajectory_collision_free(
                        dual_traj,
                        label=f"dual pose {left_technique}/{right_technique}",
                    ):
                        continue
                left_traj, right_traj = planner.split_dual_trajectory(dual_traj)
                if left_traj is not None or right_traj is not None:
                    return left_traj, right_traj

            node.get_logger().warn(
                "coordinated dual pose planning failed; falling back to "
                "single-arm plans with synchronized validation"
            )

        dual_stage = bool(left_poses and right_poses)
        if left_poses:
            left_traj = self._plan_arm(
                node, planner, left_poses, "left", left_technique, left_start
            )
            if dual_stage and left_traj is None:
                node.get_logger().warn(
                    "dual massage stage skipped: left arm has no safe plan"
                )
                return None, None
        if right_poses:
            right_traj = self._plan_arm(
                node, planner, right_poses, "right", right_technique, right_start
            )
            if dual_stage and right_traj is None:
                node.get_logger().warn(
                    "dual massage stage skipped: right arm has no safe plan"
                )
                return None, None
        return left_traj, right_traj

    def _plan_arm(
        self,
        node: Node,
        planner,
        poses: List[Pose],
        arm: str,
        technique: str,
        start_positions: Optional[List[float]] = None,
    ) -> Optional[JointTrajectory]:
        """为单臂规划轨迹。

        策略: 始终从上方 3cm/6cm 开始规划。
        因为即使是 hover 目标，直接规划也可能与头/颈碰撞体重叠。
        from-above 规划成功率高 (~0.1s)，直接规划作为最后回退。
        """
        if not poses:
            return None

        if len(poses) == 1:
            target_pose = poses[0]
            group = f"{arm}_arm"

            if planner is not None:
                # Use hover-height working poses for RViz/MoveIt massage demo.
                # Lower contact-like targets often collide with the body object.
                for dz, label in [(0.08, "8cm"), (0.12, "12cm")]:
                    node.get_logger().info(
                        f"{arm} arm: {technique} → plan from {label} above"
                    )
                    approach_pose = Pose()
                    approach_pose.position = Point(
                        x=target_pose.position.x,
                        y=target_pose.position.y,
                        z=target_pose.position.z + dz,
                    )
                    approach_pose.orientation = target_pose.orientation
                    if (
                        start_positions is not None
                        and hasattr(planner, "plan_pose_target_from_start")
                    ):
                        traj = planner.plan_pose_target_from_start(
                            start_positions,
                            approach_pose,
                            group=group,
                        )
                    else:
                        traj = planner.plan_pose_target(approach_pose, group=group)
                    if traj is not None and traj.points:
                        return traj

                node.get_logger().warn(
                    f"{arm} arm: no safe approach plan for {technique}; skipping"
                )

            node.get_logger().error(
                f"{arm} arm: ALL plans FAILED for "
                f"({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, "
                f"{target_pose.position.z:.3f}) technique={technique}"
            )
            return None
        else:
            use_cartesian_massage = False
            if planner is not None and hasattr(planner, "get_parameter"):
                try:
                    use_cartesian_massage = bool(
                        planner.get_parameter("use_cartesian_massage_path").value
                    )
                except Exception:
                    use_cartesian_massage = False

            # Multi-pose: in demo mode, avoid repeatedly trying Cartesian paths
            # that collide with the human body object. MoveIt positions the arm;
            # a local joint-space primitive makes the massage visibly active.
            if use_cartesian_massage:
                traj = self._plan_cartesian(node, poses, arm)
                if traj is not None and traj.points:
                    return traj

            # Fallback: approach from-above to FIRST waypoint (zone center).
            # NEVER fallback to poses[-1] — that jumps the arm to the end
            # of a ~108-point circular path = random position = SPASM.
            if planner is not None and len(poses) > 1:
                group = f"{arm}_arm"
                first_pose = poses[0]
                for dz, label in [(0.08, "8cm"), (0.12, "12cm")]:
                    node.get_logger().info(
                        f"{arm} arm: {technique} Cartesian fallback via {label} approach"
                    )
                    approach_pose = Pose()
                    approach_pose.position = Point(
                        x=first_pose.position.x,
                        y=first_pose.position.y,
                        z=first_pose.position.z + dz,
                    )
                    approach_pose.orientation = first_pose.orientation
                    if (
                        start_positions is not None
                        and hasattr(planner, "plan_pose_target_from_start")
                    ):
                        approach_traj = planner.plan_pose_target_from_start(
                            start_positions,
                            approach_pose,
                            group=group,
                        )
                    else:
                        approach_traj = planner.plan_pose_target(
                            approach_pose,
                            group=group,
                        )
                    if approach_traj is None or not approach_traj.points:
                        continue

                    if use_cartesian_massage:
                        follow_traj = self._plan_cartesian(
                            node,
                            poses,
                            arm,
                            start_joint_names=list(approach_traj.joint_names),
                            start_positions=list(approach_traj.points[-1].positions),
                        )
                        if follow_traj is not None and follow_traj.points:
                            return self._concat_arm_trajectories(
                            [approach_traj, follow_traj],
                            node,
                            planner,
                            label=f"{arm} {technique} approach+cartesian",
                            start_positions=start_positions,
                        )

                    prefer_surface_motion = technique in (
                        "line_press",
                        "line_knead",
                    )
                    if prefer_surface_motion:
                        surface_motion = self._make_surface_massage_motion(
                            poses,
                            approach_traj,
                            technique,
                            arm,
                            node,
                            planner,
                            dz,
                        )
                        if surface_motion is not None:
                            return self._concat_arm_trajectories(
                                [approach_traj, surface_motion],
                                node,
                                planner,
                                label=f"{arm} {technique} approach+surface-motion",
                                start_positions=start_positions,
                            )

                    local_motion = self._make_local_massage_motion(
                        approach_traj,
                        technique,
                        node,
                    )
                    if local_motion is not None:
                        return self._concat_arm_trajectories(
                            [approach_traj, local_motion],
                            node,
                            planner,
                            label=f"{arm} {technique} approach+local-motion",
                            start_positions=start_positions,
                        )

                    node.get_logger().warn(
                        f"{arm} arm: Cartesian fallback from {label} approach failed"
                    )
                node.get_logger().warn(
                    f"{arm} arm: no safe approach for {technique}; "
                    "skipping this massage segment"
                )
            return None

    def _plan_cartesian(
        self,
        node: Node,
        poses: List[Pose],
        arm: str,
        start_joint_names: Optional[List[str]] = None,
        start_positions: Optional[List[float]] = None,
    ) -> Optional[JointTrajectory]:
        """调用 MoveIt2 /compute_cartesian_path 规划 Cartesian 路径。"""
        from moveit_msgs.srv import GetCartesianPath

        planner = self.blackboard.get("planner")
        if planner is None:
            return None

        # Create client if not exist
        cartesian_client = getattr(self, "_cartesian_client", None)
        if cartesian_client is None:
            cartesian_client = node.create_client(
                GetCartesianPath, "/compute_cartesian_path"
            )
            self._cartesian_client = cartesian_client

        if not cartesian_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().warn("Cartesian path service not available")
            return None

        req = GetCartesianPath.Request()
        req.group_name = f"{arm}_arm"
        req.waypoints = poses
        req.max_step = 0.03           # 3cm 步长 (机械臂可轻松处理)
        req.jump_threshold = 0.0      # 禁止跳跃
        req.avoid_collisions = True

        if start_joint_names is not None and start_positions is not None:
            req.start_state.is_diff = True
            req.start_state.joint_state.name = list(start_joint_names)
            req.start_state.joint_state.position = list(start_positions)
        else:
            # Get current joint state as start
            js = planner.current_joint_state
            if js is not None:
                req.start_state.is_diff = True
                req.start_state.joint_state = js

        future = cartesian_client.call_async(req)
        _spin_future(self.blackboard, future, timeout_sec=10.0)
        result = future.result()

        if result is None:
            node.get_logger().warn("Cartesian path: no response")
            return None

        if result.error_code.val != 1:
            node.get_logger().warn(
                f"Cartesian path failed: code={result.error_code.val}, "
                f"completion={result.fraction:.1%}"
            )
            return None

        # Reject paths with too low completion (<50% of waypoints planned)
        if result.fraction < 0.5:
            node.get_logger().warn(
                f"Cartesian path completion too low: {result.fraction:.1%}, "
                f"falling back to joint-space plan"
            )
            return None

        traj = result.solution.joint_trajectory
        if hasattr(planner, "_stabilize_trajectory"):
            traj = planner._stabilize_trajectory(traj)
        node.get_logger().info(
            f"Cartesian path: {len(traj.points)} pts "
            f"({result.fraction:.1%} of path)"
        )
        return traj

    def _make_surface_massage_motion(
        self,
        poses: List[Pose],
        seed_traj: JointTrajectory,
        technique: str,
        arm: str,
        node: Node,
        planner,
        lift: float,
    ) -> Optional[JointTrajectory]:
        """Build a visible end-effector massage path with waypoint IK.

        This avoids depending on MoveIt's Cartesian path planner near the body
        collision object, while still making the tool tip travel along the
        generated back-surface path instead of just wiggling joints in place.
        """
        if (
            not poses
            or seed_traj is None
            or not seed_traj.points
            or planner is None
            or not hasattr(planner, "compute_ik")
        ):
            return None

        joint_names = list(seed_traj.joint_names)
        seed = list(seed_traj.points[-1].positions)
        if len(seed) != len(joint_names):
            return None

        tech = technique or "press"
        if tech in ("line_press", "line_knead"):
            max_samples, duration = 18, 7.0
        elif tech in ("scrub", "wave"):
            max_samples, duration = 20, 6.5
        elif tech.startswith("knead") or tech.startswith("rub"):
            max_samples, duration = 22, 6.8
        elif tech in ("press", "deep_press", "vibrate"):
            max_samples, duration = 10, 4.0
        else:
            max_samples, duration = 16, 5.5

        if len(poses) <= max_samples:
            sampled = list(poses)
        else:
            sampled = []
            for i in range(max_samples):
                src_idx = round(i * (len(poses) - 1) / max(max_samples - 1, 1))
                sampled.append(poses[src_idx])

        traj = JointTrajectory()
        traj.joint_names = joint_names
        start_pt = JointTrajectoryPoint()
        start_pt.positions = list(seed)
        start_pt.velocities = [0.0] * len(joint_names)
        start_pt.effort = []
        start_pt.time_from_start = _duration_msg(0.0)
        traj.points.append(start_pt)

        previous = list(seed)
        solved = 0
        for pose_idx, src_pose in enumerate(sampled, start=1):
            pose = Pose()
            pose.position = Point(
                x=src_pose.position.x,
                y=src_pose.position.y,
                z=src_pose.position.z + lift,
            )
            pose.orientation = src_pose.orientation

            stamped = PoseStamped()
            stamped.header.frame_id = "world"
            stamped.pose = pose

            ik = planner.compute_ik(
                stamped,
                f"{arm}_arm",
                timeout_sec=0.25,
                seed_joint_names=joint_names,
                seed_positions=previous,
            )
            if ik is None:
                continue

            ik = [
                _nearest_angle(value, previous[i])
                for i, value in enumerate(ik)
            ]
            max_delta = max(abs(a - b) for a, b in zip(ik, previous))
            if max_delta > 0.70:
                node.get_logger().warn(
                    f"{arm} {technique}: IK waypoint {pose_idx} skipped "
                    f"(jump {max_delta:.2f}rad)"
                )
                continue

            solved += 1
            pt = JointTrajectoryPoint()
            pt.positions = ik
            pt.velocities = [0.0] * len(joint_names)
            pt.effort = []
            ratio = solved / max(max_samples, 1)
            pt.time_from_start = _duration_msg(duration * ratio)
            traj.points.append(pt)
            previous = ik

        min_points = 4 if len(poses) > 4 else 2
        if solved < min_points:
            node.get_logger().warn(
                f"{arm} {technique}: surface IK only solved {solved} points"
            )
            return None

        last_time = _duration_seconds(traj.points[-1].time_from_start)
        if last_time <= 0.0:
            return None

        # Re-time solved points over the full intended duration.
        point_count = len(traj.points)
        for i, pt in enumerate(traj.points):
            pt.time_from_start = _duration_msg(duration * i / max(point_count - 1, 1))

        node.get_logger().info(
            f"{arm} {technique}: surface-motion IK path "
            f"{point_count} pts, {duration:.1f}s"
        )
        return traj

    def _make_local_massage_motion(
        self,
        seed_traj: JointTrajectory,
        technique: str,
        node: Node,
    ) -> Optional[JointTrajectory]:
        """Create a small visible massage motion around the reached working pose.

        This is a demo-safe fallback when MoveIt Cartesian contact planning refuses
        paths near the human collision object.
        """
        if seed_traj is None or not seed_traj.points:
            return None

        joint_names = list(seed_traj.joint_names)
        start = list(seed_traj.points[-1].positions)
        if len(start) < 6:
            return None

        tech = technique or "press"
        if tech in ("line_press", "line_knead"):
            point_count, duration = 34, 5.2
            mode = "long_stroke"
        elif tech in ("scrub", "wave"):
            point_count, duration = 32, 4.8
            mode = "slow_sweep"
        elif tech.startswith("knead") or tech.startswith("rub"):
            point_count, duration = 32, 5.0
            mode = "soft_knead"
        elif tech in ("tap", "strike", "pound"):
            point_count, duration = 18, 3.0
            mode = "soft_pulse"
        elif tech in ("vibrate",):
            point_count, duration = 20, 3.2
            mode = "micro_release"
        else:
            point_count, duration = 20, 3.4
            mode = "soft_pulse"

        side_sign = -1.0 if joint_names[0].startswith("right_") else 1.0

        traj = JointTrajectory()
        traj.joint_names = joint_names
        for i in range(point_count):
            ratio = i / max(point_count - 1, 1)
            theta = 2.0 * math.pi * ratio
            positions = list(start)

            if mode == "long_stroke":
                # One slow push and release. Starts/ends at the working pose.
                stroke = math.sin(math.pi * ratio)
                breathe = 0.5 - 0.5 * math.cos(2.0 * math.pi * ratio)
                positions[0] += side_sign * 0.035 * stroke
                positions[1] += 0.018 * breathe
                positions[2] -= 0.014 * breathe
            elif mode == "slow_sweep":
                # Small back-and-forth wipe, no wrist twisting.
                sweep = math.sin(2.0 * math.pi * ratio)
                soften = 0.5 - 0.5 * math.cos(2.0 * math.pi * ratio)
                positions[0] += side_sign * 0.026 * sweep
                positions[1] += 0.014 * soften
                positions[2] -= 0.010 * soften
            elif mode == "soft_knead":
                # Gentle oval kneading around the same working point.
                positions[1] += 0.016 * math.sin(2.0 * theta)
                positions[2] -= 0.012 * (0.5 - 0.5 * math.cos(2.0 * theta))
                positions[4] += 0.020 * math.sin(theta)
            elif mode == "micro_release":
                micro = 0.5 - 0.5 * math.cos(6.0 * math.pi * ratio)
                positions[1] += 0.006 * micro
                positions[2] -= 0.004 * micro
            else:
                pulse = 0.5 - 0.5 * math.cos(4.0 * math.pi * ratio)
                positions[1] += 0.014 * pulse
                positions[2] -= 0.010 * pulse

            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.velocities = [0.0] * len(joint_names)
            pt.effort = []
            pt.time_from_start = _duration_msg(duration * ratio)
            traj.points.append(pt)

        node.get_logger().warn(
            f"{technique}: using conservative massage primitive "
            f"({point_count} pts, {duration:.1f}s)"
        )
        return traj

    def _concat_arm_trajectories(
        self,
        segments: List[JointTrajectory],
        node: Node,
        planner,
        label: str,
        start_positions: Optional[List[float]] = None,
    ) -> Optional[JointTrajectory]:
        """Concatenate same-arm trajectory segments into one controller goal."""
        valid = [seg for seg in segments if seg is not None and seg.points]
        if not valid:
            return None

        joint_names = list(valid[0].joint_names)
        if not joint_names:
            return None
        for seg in valid[1:]:
            if set(seg.joint_names) != set(joint_names):
                node.get_logger().warn(
                    f"{label}: cannot concatenate different joint sets"
                )
                return None

        out = JointTrajectory()
        out.joint_names = joint_names
        offset = 0.0
        for seg_idx, seg in enumerate(valid):
            by_name = {name: i for i, name in enumerate(seg.joint_names)}
            for pt in seg.points:
                local_time = _duration_seconds(pt.time_from_start)
                if seg_idx > 0 and local_time <= 1e-6:
                    continue
                new_pt = JointTrajectoryPoint()
                new_pt.positions = [
                    pt.positions[by_name[name]]
                    for name in joint_names
                ]
                if pt.velocities:
                    new_pt.velocities = [
                        pt.velocities[by_name[name]]
                        for name in joint_names
                    ]
                else:
                    new_pt.velocities = [0.0] * len(joint_names)
                if pt.accelerations:
                    new_pt.accelerations = [
                        pt.accelerations[by_name[name]]
                        for name in joint_names
                    ]
                new_pt.effort = []
                new_pt.time_from_start = _duration_msg(offset + local_time)
                out.points.append(new_pt)
            offset += _duration_seconds(seg.points[-1].time_from_start)

        if hasattr(planner, "_stabilize_trajectory"):
            out = planner._stabilize_trajectory(
                out,
                start_positions=start_positions,
            )
        node.get_logger().info(
            f"{label}: concatenated {len(valid)} segments, {len(out.points)} pts"
        )
        return out

    def _execute_trajectory(self, node: Node, traj: JointTrajectory,
                            arm: str, technique: str) -> bool:
        """发送轨迹到控制器并等待执行完成。"""
        # Build action client name
        controller_name = f"/{arm}_arm_controller/follow_joint_trajectory"
        action_client = getattr(self, f"_{arm}_action_client", None)

        if action_client is None:
            from control_msgs.action import FollowJointTrajectory
            from rclpy.action import ActionClient
            action_client = ActionClient(node, FollowJointTrajectory, controller_name)
            setattr(self, f"_{arm}_action_client", action_client)

        if not action_client.wait_for_server(timeout_sec=2.0):
            node.get_logger().warn(f"Controller {controller_name} not ready, skipping")
            return False

        from control_msgs.action import FollowJointTrajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_future = action_client.send_goal_async(goal)
        _spin_future(self.blackboard, send_future, timeout_sec=5.0)

        if not send_future.result():
            node.get_logger().error(f"Failed to send goal to {arm} arm")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            node.get_logger().error(f"Goal rejected by {arm} arm controller")
            return False

        # Wait for execution
        result_future = goal_handle.get_result_async()
        timeout = (
            _duration_seconds(traj.points[-1].time_from_start) + 15.0
            if traj.points else 30.0
        )
        _spin_future(self.blackboard, result_future, timeout_sec=timeout)

        result_msg = result_future.result()
        if result_msg is None:
            node.get_logger().error(
                f"{arm.capitalize()} arm: {technique} timed out/no result"
            )
            return False

        result = result_msg.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            node.get_logger().error(
                f"{arm.capitalize()} arm: {technique} failed "
                f"code={result.error_code}: {result.error_string}"
            )
            return False

        node.get_logger().info(
            f"{arm.capitalize()} arm: {technique} done "
            f"({len(traj.points)} pts)"
        )

        # ── 沉降等待 ────────────────────────────────────────────
        # 控制器报告 SUCCESSFUL 不代表机械臂物理已经静止。
        # Gazebo 物理仿真有惯性/阻尼，如果紧接着发送下一段轨迹，JTC 会
        # 从"当前非零速度"开始插补 → 速度再激励 → 抖动级联。
        # 这里等待实际关节速度回落到阈值以下再返回。
        arm_joints = {
            f"{arm}_joint_1", f"{arm}_joint_2", f"{arm}_joint_3",
            f"{arm}_joint_4", f"{arm}_joint_5", f"{arm}_joint_6",
        }
        planner = self.blackboard.get("planner")
        settle_timeout = _planner_float_param(
            self.blackboard, "stage_settle_timeout", 0.15
        )
        settle_threshold = _planner_float_param(
            self.blackboard, "stage_settle_threshold", 0.08
        )
        settle_start = time.time()
        while rclpy.ok():
            js = planner.current_joint_state if planner else None
            if js is not None:
                arm_vels = [
                    abs(v) for n, v in zip(js.name, js.velocity)
                    if n in arm_joints and v is not None
                ]
                if arm_vels and max(arm_vels) <= settle_threshold:
                    break
            if time.time() - settle_start >= settle_timeout:
                if arm_vels:
                    node.get_logger().info(
                        f"{arm.capitalize()} arm: settle done (v_max={max(arm_vels):.4f} rad/s)"
                    )
                break
            # Spin to let /joint_states update
            executor = self.blackboard.get("executor")
            if executor is not None:
                executor.spin_once(timeout_sec=0.05)
            else:
                rclpy.spin_once(node, timeout_sec=0.05)

        return True

    def _prepare_coordinated_execution(
        self,
        node: Node,
        left_traj: Optional[JointTrajectory],
        right_traj: Optional[JointTrajectory],
        start_positions: Optional[List[float]] = None,
    ) -> Optional[tuple[Optional[JointTrajectory], Optional[JointTrajectory]]]:
        """Validate synchronized execution and re-time/replan if needed."""
        planner = self.blackboard.get("planner")
        if planner is None or not hasattr(planner, "merge_dual_trajectories"):
            return left_traj, right_traj

        if left_traj is None and right_traj is None:
            return left_traj, right_traj

        allow_parking_recovery = False
        if hasattr(planner, "get_parameter"):
            try:
                allow_parking_recovery = bool(
                    planner.get_parameter("allow_parking_recovery").value
                )
            except Exception:
                allow_parking_recovery = False

        def predicted_start_for(
            traj: Optional[JointTrajectory],
        ) -> Optional[List[float]]:
            if (
                traj is None
                or start_positions is None
                or not hasattr(planner, "_get_joint_names_for_group")
            ):
                return None
            both_names = planner._get_joint_names_for_group("both_arms")
            if len(start_positions) != len(both_names):
                return None
            start_map = dict(zip(both_names, start_positions))
            if not all(name in start_map for name in traj.joint_names):
                return None
            return [start_map[name] for name in traj.joint_names]

        # Single-arm motion still gets checked against the other arm held still.
        if left_traj is None or right_traj is None:
            merged = planner.merge_dual_trajectories(
                left_traj,
                right_traj,
                start_positions=start_positions,
            )
            if (
                merged is not None
                and hasattr(planner, "validate_trajectory_collision_free")
                and planner.validate_trajectory_collision_free(merged, label="single-arm stage")
            ):
                return left_traj, right_traj
            if (
                start_positions is None
                and hasattr(planner, "plan_dual_joint_goal_from_trajectories")
            ):
                replanned = planner.plan_dual_joint_goal_from_trajectories(
                    left_traj,
                    right_traj,
                )
                if replanned is not None and hasattr(planner, "split_dual_trajectory"):
                    return planner.split_dual_trajectory(replanned)
            if (
                start_positions is None
                and
                allow_parking_recovery
                and hasattr(planner, "plan_dual_joint_goal_via_parking")
            ):
                replanned = planner.plan_dual_joint_goal_via_parking(
                    left_traj,
                    right_traj,
                )
                if replanned is not None and hasattr(planner, "split_dual_trajectory"):
                    node.get_logger().info(
                        "using parking recovery for this single-arm stage"
                    )
                    return planner.split_dual_trajectory(replanned)
            node.get_logger().warn(
                "single-arm trajectory is not collision-safe; skipping this "
                "stage and continuing pattern"
            )
            return None, None

        merged = planner.merge_dual_trajectories(
            left_traj,
            right_traj,
            start_positions=start_positions,
        )
        if (
            merged is not None
            and hasattr(planner, "validate_trajectory_collision_free")
            and planner.validate_trajectory_collision_free(merged, label="dual synchronized stage")
        ):
            return left_traj, right_traj

        if hasattr(planner, "find_safe_dual_timing"):
            timing = planner.find_safe_dual_timing(
                left_traj,
                right_traj,
                start_positions=start_positions,
            )
            if timing is not None:
                left_delay, right_delay = timing
                if hasattr(planner, "trajectory_with_start_delay"):
                    left_traj = planner.trajectory_with_start_delay(
                        left_traj,
                        left_delay,
                        start_positions=predicted_start_for(left_traj),
                    )
                    right_traj = planner.trajectory_with_start_delay(
                        right_traj,
                        right_delay,
                        start_positions=predicted_start_for(right_traj),
                    )
                return left_traj, right_traj

        if (
            start_positions is None
            and hasattr(planner, "plan_dual_joint_goal_from_trajectories")
        ):
            replanned = planner.plan_dual_joint_goal_from_trajectories(
                left_traj,
                right_traj,
            )
            if replanned is not None and hasattr(planner, "split_dual_trajectory"):
                node.get_logger().info(
                    "using both_arms coordinated replan for this stage"
                )
                return planner.split_dual_trajectory(replanned)

        if (
            start_positions is None
            and
            allow_parking_recovery
            and hasattr(planner, "plan_dual_joint_goal_via_parking")
        ):
            replanned = planner.plan_dual_joint_goal_via_parking(
                left_traj,
                right_traj,
            )
            if replanned is not None and hasattr(planner, "split_dual_trajectory"):
                node.get_logger().info(
                    "using parking recovery for this dual-arm stage"
                )
                return planner.split_dual_trajectory(replanned)

        node.get_logger().warn(
            "dual-arm trajectories predicted unsafe; skipping this stage and "
            "continuing pattern"
        )
        return None, None

    def _execute_dual_arm(self, node: Node, left_traj: Optional[JointTrajectory],
                          right_traj: Optional[JointTrajectory],
                          left_tech: str, right_tech: str,
                          prevalidated: bool = False) -> bool:
        """双臂并行执行: 同时发送目标到左右臂控制器, 等待双方完成.

        与顺序执行 (左→等→右→等) 不同, 此方法先发送两个 goal,
        再同时等待两个 result, 实现双臂真正同时运动.
        """
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient

        # ── 解析 trajectories ──
        goals: list[tuple[str, JointTrajectory, str]] = []
        if left_traj and left_traj.points:
            goals.append(("left", left_traj, left_tech))
        if right_traj and right_traj.points:
            goals.append(("right", right_traj, right_tech))

        if not goals:
            return True

        # ── Stage 1: 并行发送所有 goal ──
        if not prevalidated:
            coordinated = self._prepare_coordinated_execution(
                node,
                left_traj,
                right_traj,
            )
            if coordinated is None:
                return False
            left_traj, right_traj = coordinated

            goals = []
            if left_traj and left_traj.points:
                goals.append(("left", left_traj, left_tech))
            if right_traj and right_traj.points:
                goals.append(("right", right_traj, right_tech))
            if not goals:
                node.get_logger().warn(
                    "No safe executable trajectory for this stage; continuing"
                )
                return True

        pending: list[tuple[str, Any, str, float]] = []  # (arm, goal_handle_future, tech, max_duration)
        for arm, traj, tech in goals:
            controller = f"/{arm}_arm_controller/follow_joint_trajectory"
            action_client = getattr(self, f"_{arm}_action_client", None)
            if action_client is None:
                action_client = ActionClient(node, FollowJointTrajectory, controller)
                setattr(self, f"_{arm}_action_client", action_client)

            if not action_client.wait_for_server(timeout_sec=2.0):
                node.get_logger().warn(f"Controller {controller} not ready, skipping {arm}")
                continue

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            send_future = action_client.send_goal_async(goal)
            max_dur = (
                _duration_seconds(traj.points[-1].time_from_start) + 15.0
                if traj.points else 30.0
            )
            pending.append((arm, send_future, tech, max_dur))

        if not pending:
            return True

        # ── Stage 2: 等待 goal 被接受 ──
        accepted: list[tuple[str, Any, str, float]] = []  # (arm, goal_handle, tech, max_dur)
        for arm, send_future, tech, max_dur in pending:
            _spin_future(self.blackboard, send_future, timeout_sec=5.0)
            result = send_future.result()
            if result and result.accepted:
                accepted.append((arm, result, tech, max_dur))
            else:
                node.get_logger().error(f"{arm.capitalize()} arm: goal rejected")

        if not accepted:
            return False

        # ── Stage 3: 并行等待所有 arm 完成 ──
        # 使用 MultiThreadedExecutor 同时等待多个 result_future
        result_futures: list[tuple[str, Any, str]] = []
        max_timeout = 0.0
        for arm, goal_handle, tech, max_dur in accepted:
            rf = goal_handle.get_result_async()
            result_futures.append((arm, rf, tech))
            max_timeout = max(max_timeout, max_dur)

        for arm, rf, tech in result_futures:
            _spin_future(self.blackboard, rf, timeout_sec=max_timeout)
            result_msg = rf.result()
            if result_msg is None:
                node.get_logger().error(f"{arm.capitalize()} arm: {tech} timed out")
                return False
            if result_msg.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                node.get_logger().error(
                    f"{arm.capitalize()} arm: {tech} failed "
                    f"code={result_msg.result.error_code}: {result_msg.result.error_string}"
                )
                return False
            node.get_logger().info(
                f"{arm.capitalize()} arm: {tech} done "
                f"({len(result_msg.result.trajectory.points) if hasattr(result_msg.result, 'trajectory') else '?'} pts)"
            )

        # ── Stage 4: 双臂沉降等待 ──
        settle_start = time.time()
        settle_timeout = _planner_float_param(
            self.blackboard, "stage_settle_timeout", 0.15
        )
        settle_threshold = _planner_float_param(
            self.blackboard, "stage_settle_threshold", 0.08
        )
        planner = self.blackboard.get("planner")
        arm_joints_set = set()
        for arm, _, _ in goals:
            for j in range(1, 7):
                arm_joints_set.add(f"{arm}_joint_{j}")

        while rclpy.ok():
            js = planner.current_joint_state if planner else None
            if js is not None:
                arm_vels = [
                    abs(v) for n, v in zip(js.name, js.velocity)
                    if n in arm_joints_set and v is not None
                ]
                if arm_vels and max(arm_vels) <= settle_threshold:
                    break
            if time.time() - settle_start >= settle_timeout:
                break
            executor = self.blackboard.get("executor")
            if executor is not None:
                executor.spin_once(timeout_sec=0.05)
            else:
                rclpy.spin_once(node, timeout_sec=0.05)

        return True


# ═══════════════════════════════════════════════════════════════
# RunMassagePattern — v6 编排器驱动 (NEW)
# ═══════════════════════════════════════════════════════════════

class RunMassagePattern(BtActionNode):
    """v6核心: 使用编排器从高层pattern动态生成带过渡的阶段序列.

    替代 RunMassageCycle, 区别:
      - RunMassageCycle: 遍历 massage_stages.yaml 的70个硬编码阶段
      - RunMassagePattern: 从 massage_patterns.yaml 加载pattern → 编排器展开
        → 自动插入过渡原语 → 执行

    Blackboard 需要:
        "massage_pattern"  — pattern名称 (如 "综合推拿")
        "path_generator"   — PathGenerator (工业系统) 或 None (旧demo)
        "surface"          — BackSurfaceModel (可选)
    """

    def __init__(self, name: str = "RunMassagePattern"):
        super().__init__(name)
        self._choreographer: Optional["MassageChoreographer"] = None
        self._stages: List[Any] = []
        self._stage_index: int = 0
        self._cycle: int = 0

    def _current_group_positions(
        self,
        planner,
        group: str,
    ) -> Optional[List[float]]:
        if (
            planner is None
            or not hasattr(planner, "_get_joint_names_for_group")
            or not hasattr(planner, "_get_start_positions")
        ):
            return None
        names = planner._get_joint_names_for_group(group)
        positions = planner._get_start_positions(names)
        return list(positions) if positions is not None else None

    def _final_positions_for_names(
        self,
        traj: Optional[JointTrajectory],
        names: List[str],
        fallback: List[float],
    ) -> List[float]:
        if traj is None or not traj.points:
            return list(fallback)
        by_name = {name: i for i, name in enumerate(traj.joint_names)}
        if not all(name in by_name for name in names):
            return list(fallback)
        last = traj.points[-1]
        return [last.positions[by_name[name]] for name in names]

    def _plan_cached_cycle(
        self,
        node: Node,
        generator: Optional[PathGenerator],
        planner,
        start_index: int = 0,
        max_massage_stages: Optional[int] = None,
    ) -> Optional[List[Dict[str, Any]]]:
        """Preplan a pattern window from predicted joint states."""
        if generator is None or planner is None:
            return None
        if (
            not hasattr(planner, "_get_joint_names_for_group")
            or not hasattr(planner, "merge_dual_trajectories")
        ):
            return None

        left_names = planner._get_joint_names_for_group("left_arm")
        right_names = planner._get_joint_names_for_group("right_arm")
        left_start = self._current_group_positions(planner, "left_arm")
        right_start = self._current_group_positions(planner, "right_arm")
        if left_start is None or right_start is None:
            node.get_logger().warn("Preplan cache unavailable: no joint state yet")
            return None

        total = len(self._stages)
        start_index = max(0, min(start_index, total))
        planned: List[Dict[str, Any]] = []
        executable = 0
        skipped = 0
        massage_seen = 0
        start_time = time.time()
        node.get_logger().info(
            f"Preplanning cycle {self._cycle + 1} window from "
            f"stage {start_index + 1}/{total}"
        )

        for idx in range(start_index, total):
            stage = self._stages[idx]
            if (
                max_massage_stages is not None
                and massage_seen >= max_massage_stages
                and stage.kind == StageKind.MASSAGE
            ):
                break
            stage_name = stage.name or f"Stage {stage.stage_id}"
            record: Dict[str, Any] = {
                "idx": idx,
                "stage": stage,
                "stage_name": stage_name,
                "left_traj": None,
                "right_traj": None,
                "left_tech": "hover",
                "right_tech": "hover",
            }

            if stage.kind == StageKind.TRANSITION:
                planned.append(record)
                continue

            massage_seen += 1

            try:
                left_def = stage.to_dict_left()
                right_def = stage.to_dict_right()
                left_tech = left_def.get("technique", "hover")
                right_tech = right_def.get("technique", "hover")
                record["left_tech"] = left_tech
                record["right_tech"] = right_tech

                left_poses = generator.generate(left_def)
                right_poses = generator.generate(right_def)
                left_traj, right_traj = self._plan_coordinated_stage(
                    node,
                    planner,
                    left_poses,
                    right_poses,
                    left_tech,
                    right_tech,
                    left_start=left_start,
                    right_start=right_start,
                )

                if left_traj or right_traj:
                    coordinated = self._prepare_coordinated_execution(
                        node,
                        left_traj,
                        right_traj,
                        start_positions=list(left_start) + list(right_start),
                    )
                    if coordinated is None:
                        left_traj = None
                        right_traj = None
                    else:
                        left_traj, right_traj = coordinated

                if left_traj or right_traj:
                    record["left_traj"] = left_traj
                    record["right_traj"] = right_traj
                    left_start = self._final_positions_for_names(
                        left_traj,
                        left_names,
                        left_start,
                    )
                    right_start = self._final_positions_for_names(
                        right_traj,
                        right_names,
                        right_start,
                    )
                    executable += 1
                else:
                    skipped += 1
                    node.get_logger().warn(
                        f"Preplan skipped unsafe stage {idx + 1}/{total}: "
                        f"{stage_name}"
                    )

            except Exception as e:
                skipped += 1
                node.get_logger().warn(
                    f"Preplan failed for stage {idx + 1}/{total} "
                    f"{stage_name}: {e}"
                )

            planned.append(record)

        elapsed = time.time() - start_time
        node.get_logger().info(
            f"Preplan cache ready: {len(planned)} stages, "
            f"{executable} executable, {skipped} skipped, {elapsed:.1f}s"
        )
        return planned

    def execute(self) -> NodeStatus:
        node: Optional[Node] = self.blackboard.get("node")
        generator: Optional[PathGenerator] = self.blackboard.get("path_generator")
        planner = self.blackboard.get("planner")
        safety = self.blackboard.get("safety")

        if node is None:
            return NodeStatus.FAILURE

        # ── 首次执行: 加载pattern并展开 ──
        if self._choreographer is None:
            if not _CHOREOGRAPHER_AVAILABLE:
                node.get_logger().error(
                    "Choreographer not available — import failed. "
                    "Falling back to RunMassageCycle."
                )
                return NodeStatus.FAILURE

            pattern_name = self.blackboard.get("massage_pattern", "综合推拿")
            self._choreographer = MassageChoreographer(SkillLibrary())

            try:
                pattern = self._choreographer.load_pattern(pattern_name)
                node.get_logger().info(
                    f"加载 pattern: '{pattern_name}' — {pattern.description}"
                )
            except Exception as e:
                node.get_logger().error(f"加载 pattern 失败: {e}")
                return NodeStatus.FAILURE

            self._stages = self._choreographer.compose(pattern)
            massage_count = sum(
                1 for s in self._stages if s.kind == StageKind.MASSAGE
            )
            trans_count = sum(
                1 for s in self._stages if s.kind == StageKind.TRANSITION
            )
            node.get_logger().info(
                f"编排器展开: {len(self._stages)} 阶段 "
                f"({massage_count} 按摩 + {trans_count} 过渡)"
            )

        total = len(self._stages)
        if total == 0:
            node.get_logger().error("No stages to execute.")
            return NodeStatus.FAILURE
        preplan_window = int(
            max(
                1.0,
                _planner_float_param(
                    self.blackboard,
                    "preplan_window_massage_stages",
                    3.0,
                ),
            )
        )

        while rclpy.ok():
            node.get_logger().info(
                f"╔══ Massage Pattern Cycle {self._cycle + 1} ═══════════"
            )

            stage_cursor = 0
            while stage_cursor < total and rclpy.ok():
                planned_cycle = self._plan_cached_cycle(
                    node,
                    generator,
                    planner,
                    start_index=stage_cursor,
                    max_massage_stages=preplan_window,
                )
                if planned_cycle is None:
                    node.get_logger().error(
                        "Unable to build preplanned massage cache."
                    )
                    return NodeStatus.FAILURE
                if not planned_cycle:
                    break

                for cached in planned_cycle:
                    idx = int(cached["idx"])
                    stage = cached["stage"]
                    # Safety check
                    if safety is not None:
                        from jaka_dual_arm.control.safety_monitor import SafetyLevel
                        level = safety.check()
                        if level.value >= SafetyLevel.ESTOP.value:
                            node.get_logger().error(
                                f"Safety ESTOP at stage {idx + 1}"
                            )
                            return NodeStatus.FAILURE
                        elif level.value >= SafetyLevel.HALT.value:
                            node.get_logger().warn(
                                f"Safety HALT at stage {idx + 1}, skipping"
                            )
                            continue

                    stage_name = cached["stage_name"]

                    # Transition stages only model force fade timing. Keep them very
                    # short in demo mode so the arms do not appear to pause.
                    if stage.kind == StageKind.TRANSITION:
                        if stage.transition:
                            node.get_logger().info(
                                f"[{idx + 1}/{total}] {stage_name} "
                                f"({stage.transition.trans_type.value}, "
                                f"{stage.transition.duration:.1f}s)"
                            )
                            pause_scale = _planner_float_param(
                                self.blackboard, "transition_pause_scale", 0.0
                            )
                            pause_max = _planner_float_param(
                                self.blackboard, "transition_pause_max", 0.05
                            )
                            pause = min(
                                stage.transition.duration * pause_scale,
                                pause_max,
                            )
                            if pause > 0.0:
                                time.sleep(pause)
                        else:
                            node.get_logger().info(
                                f"[{idx + 1}/{total}] {stage_name} (transition)"
                            )
                        continue

                    try:
                        left_tech = cached.get("left_tech", "hover")
                        right_tech = cached.get("right_tech", "hover")

                        node.get_logger().info(
                            f"[{idx + 1}/{total}] {stage_name} "
                            f"({left_tech}/{right_tech}) cached"
                        )

                        left_traj = cached.get("left_traj")
                        right_traj = cached.get("right_traj")

                        if left_traj or right_traj:
                            if not self._execute_dual_arm(
                                node,
                                left_traj,
                                right_traj,
                                left_tech,
                                right_tech,
                                prevalidated=True,
                            ):
                                return NodeStatus.FAILURE
                        else:
                            node.get_logger().warn(
                                f"[{idx + 1}/{total}] {stage_name}: "
                                "cached stage has no safe trajectory; continuing"
                            )

                    except Exception as e:
                        node.get_logger().error(
                            f"Stage {stage.stage_id} ({stage_name}) failed: {e}"
                        )
                        import traceback
                        node.get_logger().error(traceback.format_exc())
                        return NodeStatus.FAILURE

                    executor = self.blackboard.get("executor")
                    if executor is not None:
                        executor.spin_once(timeout_sec=0.01)
                    else:
                        rclpy.spin_once(node, timeout_sec=0.01)

                stage_cursor = int(planned_cycle[-1]["idx"]) + 1

            self._cycle += 1
            node.get_logger().info(
                f"╚══ Pattern cycle {self._cycle} complete "
                f"({total} stages) — continuing ══"
            )

        node.get_logger().info("Massage pattern loop terminated (rclpy shutdown).")
        return NodeStatus.SUCCESS

    # _plan_arm, _plan_cartesian, _execute_trajectory — 复用RunMassageCycle的实现
    # (这些方法在RunMassageCycle中已定义, RunMassagePattern通过继承无法直接访问,
    #  所以这里复制必要的引用 — 实际上它们通过self访问同一实例的方法)

    def _plan_arm(self, *args, **kwargs):
        """委托给 RunMassageCycle._plan_arm (同一实例的静态方法)."""
        return RunMassageCycle._plan_arm(self, *args, **kwargs)

    def _plan_coordinated_stage(self, *args, **kwargs):
        return RunMassageCycle._plan_coordinated_stage(self, *args, **kwargs)

    def _plan_cartesian(self, *args, **kwargs):
        return RunMassageCycle._plan_cartesian(self, *args, **kwargs)

    def _make_surface_massage_motion(self, *args, **kwargs):
        return RunMassageCycle._make_surface_massage_motion(self, *args, **kwargs)

    def _make_local_massage_motion(self, *args, **kwargs):
        return RunMassageCycle._make_local_massage_motion(self, *args, **kwargs)

    def _concat_arm_trajectories(self, *args, **kwargs):
        return RunMassageCycle._concat_arm_trajectories(self, *args, **kwargs)

    def _execute_trajectory(self, *args, **kwargs):
        return RunMassageCycle._execute_trajectory(self, *args, **kwargs)

    def _prepare_coordinated_execution(self, *args, **kwargs):
        return RunMassageCycle._prepare_coordinated_execution(self, *args, **kwargs)

    def _execute_dual_arm(self, *args, **kwargs):
        return RunMassageCycle._execute_dual_arm(self, *args, **kwargs)


# ═══════════════════════════════════════════════════════════════
# RetreatToHome
# ═══════════════════════════════════════════════════════════════

class RetreatToHome(BtActionNode):
    """双臂回到安全初始位姿 (悬停状态)。"""

    def __init__(self, name: str = "RetreatToHome"):
        super().__init__(name)

    def execute(self) -> NodeStatus:
        node: Optional[Node] = self.blackboard.get("node")
        planner = self.blackboard.get("planner")

        if node is None:
            return NodeStatus.FAILURE

        node.get_logger().info("Retreating to home position...")

        try:
            if planner is not None:
                # Plan each arm separately (avoids 12-joint traj sent to 6-joint controller)
                for arm, joints, home in [
                    ("left", LEFT_JOINTS, HOME_LEFT),
                    ("right", RIGHT_JOINTS, HOME_RIGHT),
                ]:
                    traj = planner.plan_joint_target(
                        left_target=home if arm == "left" else None,
                        right_target=home if arm == "right" else None,
                        left_joints=LEFT_JOINTS,
                        right_joints=RIGHT_JOINTS,
                    )
                    if traj and traj.points:
                        self._send_and_wait(node, traj, arm)
                    else:
                        node.get_logger().warn(f"{arm} arm home plan failed, skipping")

                node.get_logger().info("Home position reached.")
                return NodeStatus.SUCCESS
        except Exception as e:
            node.get_logger().error(f"Retreat failed: {e}")

        return NodeStatus.FAILURE

    def _send_and_wait(self, node: Node, traj: JointTrajectory, arm: str):
        """发送 JointTrajectory 到指定臂并等待完成。

        自动过滤轨迹中的关节名称，使 12 关节轨迹可以发给单臂控制器。
        """
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient

        controller = f"/{arm}_arm_controller/follow_joint_trajectory"
        client = ActionClient(node, FollowJointTrajectory, controller)

        if not client.wait_for_server(timeout_sec=2.0):
            node.get_logger().warn(f"Controller {controller} not ready, skipping")
            return

        # Filter trajectory to this arm's joints only
        arm_joints = set(LEFT_JOINTS if arm == "left" else RIGHT_JOINTS)
        filtered = JointTrajectory()
        filtered.header = traj.header
        filtered.joint_names = [j for j in traj.joint_names if j in arm_joints]

        # Map full-traj indices → filtered indices
        idx_map = [traj.joint_names.index(j) for j in filtered.joint_names]

        from trajectory_msgs.msg import JointTrajectoryPoint
        for pt in traj.points:
            fpt = JointTrajectoryPoint()
            fpt.positions = [pt.positions[i] for i in idx_map]
            if pt.velocities:
                fpt.velocities = [pt.velocities[i] for i in idx_map]
            fpt.time_from_start = pt.time_from_start
            filtered.points.append(fpt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = filtered

        future = client.send_goal_async(goal)
        _spin_future(self.blackboard, future, timeout_sec=5.0)
        if future.result() and future.result().accepted:
            result_future = future.result().get_result_async()
            timeout = (filtered.points[-1].time_from_start.sec + 10.0
                       if filtered.points else 20.0)
            _spin_future(self.blackboard, result_future, timeout_sec=timeout)


# ═══════════════════════════════════════════════════════════════
# Node Registry Factory
# ═══════════════════════════════════════════════════════════════

def create_massage_node_registry() -> "NodeRegistry":
    """创建按摩 BT 节点注册表。

    Returns:
        NodeRegistry with all 5 massage nodes registered
    """
    from jaka_dual_arm.behavior.bt_engine import NodeRegistry

    registry = NodeRegistry()
    registry.register("WaitServices", lambda: WaitServices())
    registry.register("SetupMassageScene", lambda: SetupMassageScene())
    registry.register("InitSurfaceModel", lambda: InitSurfaceModel())
    registry.register("RunMassageCycle", lambda: RunMassageCycle())
    registry.register("RunMassagePattern", lambda: RunMassagePattern())
    registry.register("RetreatToHome", lambda: RetreatToHome())
    return registry
