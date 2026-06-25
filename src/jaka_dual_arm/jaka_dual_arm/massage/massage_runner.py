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
            limits = SafetyLimits(**self._safety_cfg.get("limits", {}))
            self._safety = SafetyMonitor(self, limits)
            self.get_logger().info("Safety monitor active.")

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

    # ── Public API ──

    def run(self):
        """主循环: 加载 BT → Tick 直到完成."""
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
