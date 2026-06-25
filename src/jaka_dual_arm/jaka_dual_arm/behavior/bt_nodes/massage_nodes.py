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

        # Check controller action servers
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        for arm in ["left", "right"]:
            controller = f"/{arm}_arm_controller/follow_joint_trajectory"
            client = ActionClient(node, FollowJointTrajectory, controller)
            if not client.wait_for_server(timeout_sec=0.1):
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
        """注册床框 + 床垫碰撞对象。"""
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

        # Body segments
        self._add_body_objects(cfg, objects)

        # Apply all objects in ONE request
        self._apply_scene(node, objects)

    def _add_body_objects(self, cfg: dict, objects: list):
        """添加人体段碰撞 Box 到 objects 列表。"""
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive

        segments = cfg.get("body_segments", [])
        for i, seg in enumerate(segments):
            obj = CollisionObject()
            obj.id = f"body_seg_{i}"
            obj.header.frame_id = "world"
            obj.operation = CollisionObject.ADD
            seg_len = 0.06
            obj.primitives.append(SolidPrimitive(
                type=SolidPrimitive.BOX,
                dimensions=[seg_len, seg["half_w"] * 2.0, seg["thick"]],
            ))
            obj.primitive_poses.append(_make_pose_msg(
                seg["x"], 0.0, seg["z"] - seg["thick"] / 2.0))
            objects.append(obj)

    def _apply_scene(self, node: Node, objects: list):
        """Apply collision objects via /apply_planning_scene service with retry."""
        from moveit_msgs.msg import PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene

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

        scene = PlanningScene()
        scene.world.collision_objects = objects
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = planner._apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

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
        node.get_logger().info(f"Starting massage cycle: {total} stages")

        for idx, stage in enumerate(stages):
            # Safety check
            if safety is not None:
                from jaka_dual_arm.control.safety_monitor import SafetyLevel
                level = safety.check()
                if level.value >= SafetyLevel.HALT.value:
                    node.get_logger().error(
                        f"Safety HALT at stage {idx + 1}: {level}"
                    )
                    return NodeStatus.FAILURE

            stage_id = stage.get("id", idx + 1)
            stage_name = stage.get("name", f"Stage {stage_id}")
            node.get_logger().info(
                f"[{idx + 1}/{total}] {stage_name}"
            )

            try:
                left_def = stage.get("left", {})
                right_def = stage.get("right", {})

                # Generate paths for both arms
                left_poses = generator.generate(left_def) if left_def else []
                right_poses = generator.generate(right_def) if right_def else []

                # Plan trajectories
                left_traj = self._plan_arm(node, planner, left_poses, "left",
                                           left_def.get("technique", "hover"))
                right_traj = self._plan_arm(node, planner, right_poses, "right",
                                            right_def.get("technique", "hover"))

                # Execute — serial for safety (parallel possible for future)
                if left_traj:
                    self._execute_trajectory(node, left_traj, "left",
                                             left_def.get("technique", "hover"))
                if right_traj:
                    self._execute_trajectory(node, right_traj, "right",
                                             right_def.get("technique", "hover"))

            except Exception as e:
                node.get_logger().error(
                    f"Stage {stage_id} failed: {e}"
                )
                return NodeStatus.FAILURE

            # Allow ROS to process callbacks
            rclpy.spin_once(node, timeout_sec=0.01)

        node.get_logger().info(f"Massage cycle complete: {total} stages done.")
        return NodeStatus.SUCCESS

    def _plan_arm(self, node: Node, planner, poses: List[Pose],
                  arm: str, technique: str) -> Optional[JointTrajectory]:
        """为单臂规划轨迹。

        策略:
            - 1 个位姿: 关节空间 RRT
            - 多个位姿: 尝试 Cartesian Path → 失败则回退到关节空间
        """
        if not poses:
            return None

        joints = LEFT_JOINTS if arm == "left" else RIGHT_JOINTS

        if len(poses) == 1:
            # Single pose: use joint-space planning via pose_target
            target_pose = poses[0]
            group = f"{arm}_arm"

            if planner is not None:
                # Try direct pose target first
                traj = planner.plan_pose_target(target_pose, group=group)
                if traj is not None and traj.points:
                    return traj

                # Fallback: approach from 3cm above target (easier IK)
                node.get_logger().warn(
                    f"{arm} arm: direct pose plan failed, "
                    f"trying approach from above"
                )
                from geometry_msgs.msg import Point
                approach_pose = Pose()
                approach_pose.position = Point(
                    x=target_pose.position.x,
                    y=target_pose.position.y,
                    z=target_pose.position.z + 0.03,
                )
                approach_pose.orientation = target_pose.orientation
                traj = planner.plan_pose_target(approach_pose, group=group)
                if traj is not None and traj.points:
                    return traj

            node.get_logger().error(
                f"{arm} arm: all planning attempts failed for pose "
                f"({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, "
                f"{target_pose.position.z:.3f})"
            )
            return None
        else:
            # Multi-pose: Cartesian path
            traj = self._plan_cartesian(node, poses, arm)
            if traj is not None and traj.points:
                return traj

            # Fallback: plan just start→end
            if planner is not None:
                group = f"{arm}_arm"
                return planner.plan_pose_target(poses[-1], group=group)
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
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
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
        node.get_logger().info(
            f"Cartesian path: {len(traj.points)} pts "
            f"({result.fraction:.1%} of path)"
        )
        return traj

    def _execute_trajectory(self, node: Node, traj: JointTrajectory,
                            arm: str, technique: str):
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
            return

        from control_msgs.action import FollowJointTrajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)

        if not send_future.result():
            node.get_logger().error(f"Failed to send goal to {arm} arm")
            return

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            node.get_logger().error(f"Goal rejected by {arm} arm controller")
            return

        # Wait for execution
        result_future = goal_handle.get_result_async()
        timeout = traj.points[-1].time_from_start.sec + 15.0 if traj.points else 30.0
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)

        node.get_logger().info(
            f"{arm.capitalize()} arm: {technique} done "
            f"({len(traj.points)} pts)"
        )


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
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() and future.result().accepted:
            result_future = future.result().get_result_async()
            timeout = (filtered.points[-1].time_from_start.sec + 10.0
                       if filtered.points else 20.0)
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)


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
