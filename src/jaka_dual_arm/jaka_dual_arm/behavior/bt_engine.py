#!/usr/bin/env python3
"""BT Engine — 加载 BehaviorTree XML 并执行。

设计目标:
    与 BehaviorTree.CPP v4 的 XML 格式兼容，支持 Groot 可视化编辑。
    当 C++ BehaviorTree.CPP 可用时，替换此引擎为原生 C++ 执行器；
    在此之前，使用 Python BT 节点库完成相同功能。

支持的 XML 元素:
    <BehaviorTree>  — 树定义 (可嵌套子树)
    <Sequence>      — 顺序执行
    <Fallback>      — 备选执行
    <RetryUntilSuccessful> — 重试装饰器
    <SubTree>       — 子树引用
    <SetBlackboard> — 写入黑板
    自定义 Node      — 从 bt_nodes 模块动态加载

用法:
    engine = BtEngine(node, blackboard, node_registry)
    engine.load_xml("behavior/trees/carry_task.xml")
    engine.select_tree("CarryTask")
    while engine.tick() == NodeStatus.RUNNING:
        rclpy.spin_once(node)
    if engine.success:
        print("Task complete!")

参考:
  - BehaviorTree.CPP v4 — XMLParser + Tree + Blackboard
  - py_trees — Python BT 引擎
  - ManyMove — ROS2 BT Action 集成
"""

from __future__ import annotations

import importlib
import os
import xml.etree.ElementTree as ET
from typing import Any, Callable, Optional

import rclpy
from rclpy.node import Node

from jaka_dual_arm.behavior.bt_nodes.bt_node_base import (
    NodeStatus, BtNode, Sequence, Fallback, BtActionNode, BtAsyncNode, BtCondition,
)


# ── 节点注册表 ──────────────────────────────────────────────


class NodeRegistry:
    """BT 节点工厂 — 按名称创建节点实例。

    注册节点类型:
        registry.register("DetectObject", lambda: DetectObject())
        registry.register("PlanPhase", lambda: PlanPhase())
    """

    def __init__(self):
        self._factories: dict[str, Callable[[], BtNode]] = {}

    def register(self, node_type: str, factory: Callable[[], BtNode]):
        """注册节点工厂。"""
        self._factories[node_type] = factory

    def create(self, node_type: str, name: str = "") -> Optional[BtNode]:
        """创建节点实例。"""
        factory = self._factories.get(node_type)
        if factory is None:
            return None
        node = factory()
        node.name = name or node_type
        return node

    @property
    def registered_types(self) -> list[str]:
        return list(self._factories.keys())


def create_carry_node_registry(
    node: Node,
    perception=None,
    scene_cfg: dict = None,
    left_joints: list[str] = None,
    right_joints: list[str] = None,
    left_action=None,
    right_action=None,
    motion_client=None,
    get_start_positions=None,
    plan_fn=None,
) -> NodeRegistry:
    """创建搬运任务的节点注册表。

    注册所有搬运 BT 需要的节点类型，绑定到具体的 ROS2 资源上。
    """
    from jaka_dual_arm.behavior.bt_nodes.carry_nodes import (
        WaitServices, DetectObject, PlanPhase, ExecuteTrajectory, CheckGrasp,
    )

    registry = NodeRegistry()

    # WaitServices
    ws = WaitServices()
    ws._motion_client = motion_client
    ws._left_client = left_action
    ws._right_client = right_action
    ws._joints_ready = False
    registry.register("WaitServices", lambda: ws)

    # DetectObject
    registry.register("DetectObject", lambda: DetectObject(perception=perception))

    # PlanPhase
    def _make_plan():
        p = PlanPhase()
        p.config["scene_cfg"] = scene_cfg or {}
        p.config["left_joints"] = left_joints or []
        p.config["right_joints"] = right_joints or []
        p.config["joint_names"] = (left_joints or []) + (right_joints or [])
        p.config["get_start_positions"] = get_start_positions
        p.config["plan_fn"] = plan_fn
        return p
    registry.register("PlanPhase", _make_plan)

    # ExecuteTrajectory
    def _make_exec():
        e = ExecuteTrajectory()
        e.config["left_joints"] = left_joints or []
        e.config["right_joints"] = right_joints or []
        e.config["left_action"] = left_action
        e.config["right_action"] = right_action
        e.config["node"] = node
        return e
    registry.register("ExecuteTrajectory", _make_exec)

    # CheckGrasp
    cg = CheckGrasp()
    registry.register("CheckGrasp", lambda: cg)

    return registry


# ── XML 解析器 ───────────────────────────────────────────────


class BtXmlParser:
    """解析 BehaviorTree XML 并构建树结构。"""

    def __init__(self, registry: NodeRegistry, blackboard: dict[str, Any] = None):
        self._registry = registry
        self._blackboard = blackboard or {}
        self._sub_trees: dict[str, ET.Element] = {}

    def parse_file(self, xml_path: str) -> dict[str, BtNode]:
        """解析 XML 文件，返回所有 BehaviorTree 定义。

        Returns:
            {tree_id: root_node} 字典
        """
        tree = ET.parse(xml_path)
        root = tree.getroot()

        trees: dict[str, BtNode] = {}

        for child in root:
            if child.tag == "BehaviorTree":
                tree_id = child.attrib.get("ID", "Main")
                # 先收集子树
                self._collect_sub_trees(child)
                # 解析主树 (第一个 Sequence/Fallback 等)
                root_node = self._parse_element(child[0]) if len(child) > 0 else None
                if root_node:
                    trees[tree_id] = root_node

        return trees

    def _collect_sub_trees(self, root_element: ET.Element):
        """收集所有子树定义 (后续可引用)。"""
        for child in root_element:
            if child.tag == "BehaviorTree":
                self._sub_trees[child.attrib.get("ID", "")] = child

    def _parse_element(self, element: ET.Element) -> Optional[BtNode]:
        """递归解析单个 XML 元素为 BT 节点。"""
        tag = element.tag
        name = element.attrib.get("name", tag)
        children = list(element)

        # ── 控制节点 ──
        if tag == "Sequence":
            return Sequence(
                name=name,
                children=[n for c in children if (n := self._parse_element(c)) is not None],
            )
        elif tag == "Fallback":
            return Fallback(
                name=name,
                children=[n for c in children if (n := self._parse_element(c)) is not None],
            )
        elif tag == "RetryUntilSuccessful":
            max_retries = int(element.attrib.get("num_attempts", "3"))
            child_node = self._parse_element(children[0]) if children else None
            from jaka_dual_arm.behavior.bt_nodes.bt_node_base import RetryNode
            return RetryNode(name=name, child=child_node, max_retries=max_retries)

        # ── 子树引用 ──
        elif tag == "SubTree":
            sub_id = element.attrib.get("ID", "")
            sub_elem = self._sub_trees.get(sub_id)
            if sub_elem is not None:
                # 取子树的第一个控制节点
                for child in sub_elem:
                    if child.tag in ("Sequence", "Fallback", "SubTree"):
                        return self._parse_element(child)
            return None

        # ── 黑板写入 ──
        elif tag == "SetBlackboard":
            key = element.attrib.get("output_key", "")
            value = element.attrib.get("value", "")

            class _SetBb(BtActionNode):
                def execute(self2):
                    self._blackboard[key] = value
                    return NodeStatus.SUCCESS
            return _SetBb(name=name)

        # ── 叶子节点 (自定义 Action/Condition) ──
        else:
            node = self._registry.create(tag, name)
            if node is None:
                return None

            # 把 XML 属性写入节点 config
            for attr_key, attr_val in element.attrib.items():
                if attr_key != "name":
                    node.config[attr_key] = attr_val

            # 绑定黑板
            node.blackboard = self._blackboard
            return node


# ── BT 引擎 ──────────────────────────────────────────────────


class BtEngine:
    """行为树执行引擎。

    用法:
        engine = BtEngine(node, blackboard, registry)
        engine.load_xml("path/to/tree.xml")
        engine.select_tree("CarryTask")

        while engine.tick() == NodeStatus.RUNNING:
            rclpy.spin_once(node, timeout_sec=0.05)

        if engine.success:
            print("SUCCESS")
        else:
            print(f"FAILED: {engine.failure_reason}")
    """

    def __init__(self, node: Node, blackboard: dict[str, Any],
                 registry: NodeRegistry):
        self._node = node
        self._logger = node.get_logger()
        self._blackboard = blackboard
        self._registry = registry
        self._trees: dict[str, BtNode] = {}
        self._active_tree: Optional[BtNode] = None
        self._active_tree_id: str = ""
        self._success = False
        self._failure_reason: str = ""
        self._tick_count = 0

    def load_xml(self, xml_path: str):
        """加载 XML 行为树文件。"""
        parser = BtXmlParser(self._registry, self._blackboard)
        self._trees = parser.parse_file(xml_path)
        self._logger.info(f"Loaded {len(self._trees)} tree(s) from {xml_path}:")
        for tid, root in self._trees.items():
            self._logger.info(f"  - {tid}")

    def select_tree(self, tree_id: str) -> bool:
        """选择要执行的行为树。"""
        root = self._trees.get(tree_id)
        if root is None:
            self._logger.error(f"Tree '{tree_id}' not found. Available: {list(self._trees.keys())}")
            return False
        self._active_tree = root
        self._active_tree_id = tree_id
        self._success = False
        self._failure_reason = ""
        self._tick_count = 0
        self._logger.info(f"Selected tree: {tree_id}")
        return True

    def tick(self) -> NodeStatus:
        """执行行为树的一个 tick 周期。

        Returns:
            RUNNING — 任务仍在进行中
            SUCCESS — 任务完成
            FAILURE — 任务失败
        """
        if self._active_tree is None:
            self._failure_reason = "No active tree selected"
            return NodeStatus.FAILURE

        self._tick_count += 1
        status = self._active_tree.tick()

        if status == NodeStatus.SUCCESS:
            self._success = True
            self._logger.info(f"[BT:{self._active_tree_id}] SUCCESS ({self._tick_count} ticks)")
        elif status == NodeStatus.FAILURE:
            self._success = False
            self._failure_reason = f"Tree returned FAILURE at tick {self._tick_count}"
            self._logger.error(f"[BT:{self._active_tree_id}] FAILURE ({self._tick_count} ticks)")
        elif status == NodeStatus.RUNNING:
            pass  # 继续

        return status

    def halt(self):
        """中断当前执行的行为树。"""
        if self._active_tree:
            self._active_tree.halt()
            self._logger.info(f"[BT:{self._active_tree_id}] HALTED")

    @property
    def success(self) -> bool:
        return self._success

    @property
    def failure_reason(self) -> str:
        return self._failure_reason

    @property
    def tick_count(self) -> int:
        return self._tick_count

    @property
    def blackboard(self) -> dict[str, Any]:
        return self._blackboard
