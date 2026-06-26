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

import time
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from jaka_dual_arm.behavior.bt_nodes.bt_node_base import (
    NodeStatus, BtActionNode, BtCondition, BtAsyncNode,
)
from jaka_dual_arm.skills.path_generator import (
    BackSurfaceModel, PathGenerator, TECHNIQUE_CONFIG,
)

# ── Executor-aware spin helper ──────────────────────────────────

def _duration_seconds(d: "Duration") -> float:
    """Convert builtin_interfaces/msg/Duration to seconds."""
    return float(d.sec) + float(d.nanosec) / 1e9


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

                    # ── Left arm: generate → plan → execute ──
                    # (Execute left FIRST so right arm planner sees left's final
                    #  position via /joint_states, avoiding cross-arm collisions)
                    left_poses = generator.generate(left_def) if left_def else []
                    if left_poses:
                        left_traj = self._plan_arm(node, planner, left_poses, "left",
                                                   left_def.get("technique", "hover"))
                        if left_traj:
                            if not self._execute_trajectory(
                                node, left_traj, "left",
                                left_def.get("technique", "hover")
                            ):
                                return NodeStatus.FAILURE

                    # ── Right arm: generate → plan → execute ──
                    # (Left arm has already moved; planner avoids its new position)
                    right_poses = generator.generate(right_def) if right_def else []
                    if right_poses:
                        right_traj = self._plan_arm(node, planner, right_poses, "right",
                                                    right_def.get("technique", "hover"))
                        if right_traj:
                            if not self._execute_trajectory(
                                node, right_traj, "right",
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

    def _plan_arm(self, node: Node, planner, poses: List[Pose],
                  arm: str, technique: str) -> Optional[JointTrajectory]:
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
                from geometry_msgs.msg import Point

                # Level 1-2: approach from above (high success rate, fast)
                # 6cm/12cm 避免机械臂穿过头部球体(r=0.055)和颈部到达C7区
                for dz, label in [(0.06, "6cm"), (0.12, "12cm")]:
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
                    traj = planner.plan_pose_target(approach_pose, group=group)
                    if traj is not None and traj.points:
                        return traj

                # Level 3: direct plan as last resort
                node.get_logger().warn(
                    f"{arm} arm: approach plans failed, trying direct"
                )
                traj = planner.plan_pose_target(target_pose, group=group)
                if traj is not None and traj.points:
                    return traj

            node.get_logger().error(
                f"{arm} arm: ALL plans FAILED for "
                f"({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, "
                f"{target_pose.position.z:.3f}) technique={technique}"
            )
            return None
        else:
            # Multi-pose: Cartesian path
            traj = self._plan_cartesian(node, poses, arm)
            if traj is not None and traj.points:
                return traj

            # Fallback: approach from-above to FIRST waypoint (zone center).
            # NEVER fallback to poses[-1] — that jumps the arm to the end
            # of a ~108-point circular path = random position = SPASM.
            if planner is not None and len(poses) > 1:
                group = f"{arm}_arm"
                first_pose = poses[0]
                for dz in [0.06, 0.12]:
                    approach_pose = Pose()
                    approach_pose.position = Point(
                        x=first_pose.position.x,
                        y=first_pose.position.y,
                        z=first_pose.position.z + dz,
                    )
                    approach_pose.orientation = first_pose.orientation
                    traj = planner.plan_pose_target(approach_pose, group=group)
                    if traj is not None and traj.points:
                        return traj
                # Last resort: direct to first pose
                traj = planner.plan_pose_target(first_pose, group=group)
                if traj is not None and traj.points:
                    return traj
            return None

    def _plan_cartesian(self, node: Node, poses: List[Pose],
                        arm: str) -> Optional[JointTrajectory]:
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

        # Get current joint state as start
        js = planner.current_joint_state
        if js is not None:
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
        settle_timeout = 2.0
        settle_threshold = 0.04  # rad/s — 接近停止
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
    registry.register("RetreatToHome", lambda: RetreatToHome())
    return registry
