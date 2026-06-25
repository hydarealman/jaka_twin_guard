#!/usr/bin/env python3
"""工业级按摩系统 — 薄层 BT 驱动.

架构:
    massage_runner (本文件)  — Node + Blackboard + BtEngine.tick()
    massage_task.xml         — BT 结构定义 (5 节点 Sequence)
    massage_nodes.py         — 5 个自定义 BT 节点实现
    path_generator.py        — 背部曲面模型 + 路径生成 (Layer 3+4)
    planner_server.py        — MoveIt2 规划 (Layer 2)

对比旧版:
    旧: 689行单体 Node, 手写 if/elif 状态机, 硬编码关节角偏移
    新: ~180行 BT driver, XML 编排, 曲面模型驱动轨迹生成

用法:
    ros2 launch jaka_dual_arm industrial_massage.launch.py
"""

from __future__ import annotations

import os
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node

# Layer 2: Motion Planning
from jaka_dual_arm.planner.planner_server import DualArmPlannerServer

# Layer 3+4: Surface Model + Path Generation
from jaka_dual_arm.skills.path_generator import BackSurfaceModel, PathGenerator

# Layer 5: BehaviorTree
from jaka_dual_arm.behavior.bt_engine import BtEngine, NodeRegistry
from jaka_dual_arm.behavior.bt_nodes.bt_node_base import NodeStatus
from jaka_dual_arm.behavior.bt_nodes.massage_nodes import create_massage_node_registry

# Control (optional, loaded if configs available)
from jaka_dual_arm.control.safety_monitor import SafetyMonitor, SafetyLimits
from jaka_dual_arm.control.virtual_impedance import VirtualImpedanceController


# ═══════════════════════════════════════════════════════════════
# Massage Runner Node
# ═══════════════════════════════════════════════════════════════

class MassageRunnerNode(Node):
    """按摩系统主节点 — BT 引擎宿主.

    职责:
        1. 持有 Planner/Safety/Impedance 等资源
        2. 配置 Blackboard 供所有 BT 节点共享
        3. 加载 BT XML → BtEngine.tick() 循环
        4. 报告完成状态
    """

    def __init__(self,
                 body_config: dict,
                 stages_config: dict,
                 impedance_config: Optional[dict] = None,
                 safety_config: Optional[dict] = None):
        super().__init__("massage_runner")

        self._body_cfg = body_config
        self._stages_cfg = stages_config
        self._imp_cfg = impedance_config or {}
        self._safety_cfg = safety_config or {}

        # ── Layer 2: Planner ──
        self._planner = DualArmPlannerServer()
        self.get_logger().info("Planner server created.")

        # ── Layer 1: Safety Monitor (optional) ──
        self._safety: Optional[SafetyMonitor] = None
        if self._safety_cfg:
            limits = SafetyLimits.from_yaml(self._safety_cfg)
            self._safety = SafetyMonitor(self, limits)
            self.get_logger().info(
                f"Safety monitor active (joint_timeout={limits.joint_state_timeout}s, "
                f"pos_margin={limits.joint_position_margin:.3f}rad)"
            )

        # ── Layer 1: Virtual Impedance (optional) ──
        self._impedance: Optional[VirtualImpedanceController] = None
        if self._imp_cfg:
            self._impedance = VirtualImpedanceController(self)
            self.get_logger().info("Virtual impedance active.")

        # ── Blackboard — BT 节点共享数据 ──
        self._blackboard: Dict[str, Any] = {
            "node": self,
            "body_config": body_config,
            "stages": stages_config.get("stages", []),
            "planner": self._planner,
            "safety": self._safety,
            "impedance": self._impedance,
            "surface": None,           # InitSurfaceModel 填充
            "path_generator": None,     # InitSurfaceModel 填充
        }

        # ── BT Engine ──
        self._registry = create_massage_node_registry()
        self._engine = BtEngine(self, self._blackboard, self._registry)
        self._task_done = False

    # ── Early Scene Registration ──────────────────────────────

    def _apply_scene_early(self):
        """提前向 PlanningScene 注册床+人体碰撞对象。

        不等 BT engine 执行 SetupMassageScene 节点，确保 RViz 在
        move_group 就绪后立即显示场景。
        """
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene
        from shape_msgs.msg import SolidPrimitive
        import math

        # 等待 /apply_planning_scene 服务
        if not self._planner._apply_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                "/apply_planning_scene not available after 10s — "
                "scene will be applied later by BT engine"
            )
            return

        objects = []
        bed = self._body_cfg.get("bed", {})
        mat_top = bed.get("mattress", {}).get("top_z", 0.14)

        # ── Bed frame ──
        frame = bed.get("frame", {})
        frame_size = frame.get("size", {"x": 1.20, "y": 0.66, "z": 0.08})
        frame_bottom = frame.get("bottom_z", 0.0)
        frame_z = frame_bottom + frame_size["z"] / 2.0
        obj = CollisionObject()
        obj.id = "bed_frame"; obj.header.frame_id = "world"
        obj.operation = CollisionObject.ADD
        obj.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[frame_size["x"], frame_size["y"], frame_size["z"]],
        ))
        obj.primitive_poses.append(_make_pose_msg(
            bed["center"]["x"], bed["center"]["y"], frame_z))
        objects.append(obj)

        # ── Mattress ──
        mattress = bed.get("mattress", {})
        mat_size = mattress.get("size", {"x": 1.12, "y": 0.56, "z": 0.06})
        mat_bottom = mattress.get("bottom_z", 0.08)
        mat_z = mat_bottom + mat_size["z"] / 2.0
        obj2 = CollisionObject()
        obj2.id = "mattress"; obj2.header.frame_id = "world"
        obj2.operation = CollisionObject.ADD
        obj2.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[mat_size["x"], mat_size["y"], mat_size["z"]],
        ))
        obj2.primitive_poses.append(_make_pose_msg(
            bed["center"]["x"], bed["center"]["y"], mat_z))
        objects.append(obj2)

        # ── Torso (simplified box covering spine x:0.35→0.88) ──
        torso_x = 0.615; torso_z = mat_top + 0.03
        obj3 = CollisionObject()
        obj3.id = "torso"; obj3.header.frame_id = "world"
        obj3.operation = CollisionObject.ADD
        obj3.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX, dimensions=[0.55, 0.36, 0.06]))
        obj3.primitive_poses.append(_make_pose_msg(torso_x, 0.0, torso_z))
        objects.append(obj3)

        # ── Head + Neck ──
        obj4 = CollisionObject()
        obj4.id = "head"; obj4.header.frame_id = "world"
        obj4.operation = CollisionObject.ADD
        obj4.primitives.append(SolidPrimitive(
            type=SolidPrimitive.SPHERE, dimensions=[0.055]))
        obj4.primitive_poses.append(_make_pose_msg(0.20, 0.0, 0.24))
        objects.append(obj4)

        obj5 = CollisionObject()
        obj5.id = "neck"; obj5.header.frame_id = "world"
        obj5.operation = CollisionObject.ADD
        obj5.primitives.append(SolidPrimitive(
            type=SolidPrimitive.CYLINDER, dimensions=[0.035, 0.025]))
        obj5.primitive_poses.append(_make_pose_msg(0.285, 0.0, 0.20))
        objects.append(obj5)

        # ── Arms + Legs (simplified) ──
        for side, sy in [("left", -1), ("right", 1)]:
            ua_cz = max(mat_top + 0.03, 0.18)
            o = CollisionObject()
            o.id = f"{side}_upper_arm"; o.header.frame_id = "world"
            o.operation = CollisionObject.ADD
            o.primitives.append(SolidPrimitive(
                type=SolidPrimitive.CYLINDER, dimensions=[0.17, 0.030]))
            o.primitive_poses.append(_make_pose_msg(0.49, sy*0.225, ua_cz))
            objects.append(o)

            fa_cz = max(mat_top + 0.03, 0.16)
            o2 = CollisionObject()
            o2.id = f"{side}_forearm"; o2.header.frame_id = "world"
            o2.operation = CollisionObject.ADD
            o2.primitives.append(SolidPrimitive(
                type=SolidPrimitive.CYLINDER, dimensions=[0.15, 0.028]))
            o2.primitive_poses.append(_make_pose_msg(0.64, sy*0.255, fa_cz))
            objects.append(o2)

            o3 = CollisionObject()
            o3.id = f"{side}_hand"; o3.header.frame_id = "world"
            o3.operation = CollisionObject.ADD
            o3.primitives.append(SolidPrimitive(
                type=SolidPrimitive.SPHERE, dimensions=[0.035]))
            o3.primitive_poses.append(_make_pose_msg(
                0.76, sy*0.26, max(mat_top+0.035, 0.17)))
            objects.append(o3)

        for side, sy in [("left", -1), ("right", 1)]:
            th_cz = max(mat_top + 0.04, 0.18)
            o = CollisionObject()
            o.id = f"{side}_thigh"; o.header.frame_id = "world"
            o.operation = CollisionObject.ADD
            o.primitives.append(SolidPrimitive(
                type=SolidPrimitive.CYLINDER, dimensions=[0.22, 0.042]))
            o.primitive_poses.append(_make_pose_msg(0.935, sy*0.11, th_cz))
            objects.append(o)

            ca_cz = max(mat_top + 0.04, 0.17)
            o2 = CollisionObject()
            o2.id = f"{side}_calf"; o2.header.frame_id = "world"
            o2.operation = CollisionObject.ADD
            o2.primitives.append(SolidPrimitive(
                type=SolidPrimitive.CYLINDER, dimensions=[0.20, 0.038]))
            o2.primitive_poses.append(_make_pose_msg(1.12, sy*0.12, ca_cz))
            objects.append(o2)

        # ── Apply ──
        scene = PlanningScene()
        scene.world.collision_objects = objects
        scene.is_diff = True
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._planner._apply_scene_client.call_async(req)

        # Spin until done
        import rclpy
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 5.0:
                break
        if future.done() and future.result() and future.result().success:
            self.get_logger().info(
                f"Scene applied early: {len(objects)} collision objects registered"
            )
        else:
            self.get_logger().warn("Early scene application incomplete (BT will retry)")

    # ── Public API ──

    def run(self):
        """主循环: 加载 BT → Tick 直到完成."""
        # ── 提前注册场景 (让 RViz 立即显示床+人体，不等 BT 进度) ──
        self._apply_scene_early()

        # Load XML
        share_dir = _get_share_dir()
        xml_path = os.path.join(share_dir, "behavior", "trees", "massage_task.xml")
        if not os.path.exists(xml_path):
            # Fallback: source tree (development)
            src_root = os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.dirname(__file__))))  # massage/__init__ → massage → jaka_dual_arm → src
            xml_path = os.path.join(
                src_root, "src", "jaka_dual_arm", "jaka_dual_arm",
                "behavior", "trees", "massage_task.xml"
            )
            if not os.path.exists(xml_path):
                self.get_logger().error(f"BT XML not found. Checked share and source tree.")
                return

        self._engine.load_xml(xml_path)
        if not self._engine.select_tree("MassageTask"):
            self.get_logger().error("Failed to select MassageTask tree.")
            return

        self.get_logger().info("=" * 60)
        self.get_logger().info("  Industrial Massage System — BT Engine Starting")
        self.get_logger().info(f"  Stages: {len(self._blackboard['stages'])}")
        self.get_logger().info(f"  Techniques: {len(self._registry.registered_types)} node types")
        self.get_logger().info("=" * 60)

        # ── Tick Loop (MultiThreadedExecutor: spins both massage_runner + planner) ──
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.add_node(self._planner)

        # Pass executor to blackboard and planner so both can spin all nodes
        self._blackboard["executor"] = executor
        self._planner._shared_executor = executor

        start_time = time.time()

        while rclpy.ok():
            status = self._engine.tick()

            if status == NodeStatus.SUCCESS:
                elapsed = time.time() - start_time
                self.get_logger().info(
                    f"Massage task completed normally after {elapsed:.1f}s "
                    f"({self._engine.tick_count} ticks)"
                )
                self._task_done = True
                break

            elif status == NodeStatus.FAILURE:
                self.get_logger().error(
                    f"Massage task FAILED: {self._engine.failure_reason}"
                )
                break

            # Spin both nodes via executor
            executor.spin_once(timeout_sec=0.05)

        # Report
        if self._task_done:
            self.get_logger().info("✓ Massage system shut down cleanly.")
        else:
            self.get_logger().error("✗ Massage stopped unexpectedly.")

    def shutdown(self):
        """Clean up resources."""
        self._engine.halt()
        if self._planner:
            self._planner.destroy_node()
        self.destroy_node()

    @property
    def task_done(self) -> bool:
        return self._task_done


# ═══════════════════════════════════════════════════════════════
# Utility
# ═══════════════════════════════════════════════════════════════

def _make_pose_msg(x: float, y: float, z: float,
                   qx: float = 0.0, qy: float = 0.0,
                   qz: float = 0.0, qw: float = 1.0) -> "Pose":
    """Create a geometry_msgs/Pose message."""
    from geometry_msgs.msg import Point, Quaternion, Pose as PoseMsg
    pose = PoseMsg()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def _get_share_dir() -> str:
    """Get the install share directory."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("jaka_dual_arm")
    except Exception:
        return os.path.join(
            os.path.dirname(__file__), "..", "..", "..", "install",
            "jaka_dual_arm", "share", "jaka_dual_arm"
        )
