#!/usr/bin/env python3
"""BT Engine — 加载 BehaviorTree XML 并执行。

与 jaka_dual_arm/behavior/bt_engine.py 同架构。
使用 jaka_single_arm 本地 bt_node_base 避免跨包依赖。

支持的 XML 元素:
    <BehaviorTree>  — 树定义
    <Sequence>      — 顺序执行
    <Fallback>      — 备选执行
    <RetryUntilSuccessful> — 重试装饰器
    <SubTree>       — 子树引用
    <SetBlackboard> — 写入黑板
    自定义 Node      — 通过 NodeRegistry 动态创建

用法:
    engine = BtEngine(node, blackboard, node_registry)
    engine.load_xml("behavior/trees/pick_place_task.xml")
    engine.select_tree("PickPlaceTask")
    while engine.tick() == NodeStatus.RUNNING:
        rclpy.spin_once(node)

参考:
  - BehaviorTree.CPP v4 — XMLParser + Tree + Blackboard
  - jaka_dual_arm/behavior/bt_engine.py — 同架构双版本
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Any, Callable, Optional

from rclpy.node import Node

from jaka_single_arm.behavior.bt_node_base import (
    NodeStatus, BtNode, BtActionNode,
    Sequence, Fallback, RetryNode,
)


class NodeRegistry:
    """BT 节点工厂 — 按名称创建节点实例。"""

    def __init__(self):
        self._factories: dict[str, Callable[[], BtNode]] = {}

    def register(self, node_type: str, factory: Callable[[], BtNode]):
        self._factories[node_type] = factory

    def create(self, node_type: str, name: str = "") -> Optional[BtNode]:
        factory = self._factories.get(node_type)
        if factory is None:
            return None
        node = factory()
        node.name = name or node_type
        return node

    @property
    def registered_types(self) -> list[str]:
        return list(self._factories.keys())


class BtXmlParser:
    """解析 BehaviorTree XML 并构建树结构。"""

    def __init__(self, registry: NodeRegistry, blackboard: dict[str, Any] = None):
        self._registry = registry
        self._blackboard = blackboard or {}
        self._sub_trees: dict[str, ET.Element] = {}

    def parse_file(self, xml_path: str) -> dict[str, BtNode]:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        trees: dict[str, BtNode] = {}

        for child in root:
            if child.tag == "BehaviorTree":
                tree_id = child.attrib.get("ID", "Main")
                self._collect_sub_trees(child)
                root_node = self._parse_element(child[0]) if len(child) > 0 else None
                if root_node:
                    trees[tree_id] = root_node
        return trees

    def _collect_sub_trees(self, root_element: ET.Element):
        for child in root_element:
            if child.tag == "BehaviorTree":
                self._sub_trees[child.attrib.get("ID", "")] = child

    def _parse_element(self, element: ET.Element) -> Optional[BtNode]:
        tag = element.tag
        name = element.attrib.get("name", tag)
        children = list(element)

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
            return RetryNode(name=name, child=child_node, max_retries=max_retries)
        elif tag == "SubTree":
            sub_id = element.attrib.get("ID", "")
            sub_elem = self._sub_trees.get(sub_id)
            if sub_elem is not None:
                for child in sub_elem:
                    if child.tag in ("Sequence", "Fallback", "SubTree"):
                        return self._parse_element(child)
            return None
        elif tag == "SetBlackboard":
            key = element.attrib.get("output_key", "")
            value = element.attrib.get("value", "")

            class _SetBb(BtActionNode):
                def execute(self2):
                    self._blackboard[key] = value
                    return NodeStatus.SUCCESS
            return _SetBb(name=name)
        else:
            # Custom leaf node
            node = self._registry.create(tag, name)
            if node is None:
                return None
            for attr_key, attr_val in element.attrib.items():
                if attr_key != "name":
                    node.config[attr_key] = attr_val
            node.blackboard = self._blackboard
            return node


class BtEngine:
    """行为树执行引擎。"""

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
        parser = BtXmlParser(self._registry, self._blackboard)
        self._trees = parser.parse_file(xml_path)
        self._logger.info(f"Loaded {len(self._trees)} tree(s) from {xml_path}")
        for tid in self._trees.keys():
            self._logger.info(f"  - {tid}")

    def select_tree(self, tree_id: str) -> bool:
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

        return status

    def reset(self):
        """Reset the active tree to IDLE so it can be re-ticked.

        Use this to restart the same tree for a new target (e.g. next object).
        Does NOT clear the blackboard — target_object should be updated first.
        """
        if self._active_tree:
            self._active_tree.reset()
            self._tick_count = 0
            self._success = False
            self._failure_reason = ""
            self._logger.debug(f"[BT:{self._active_tree_id}] Reset for next run.")

    def halt(self):
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
