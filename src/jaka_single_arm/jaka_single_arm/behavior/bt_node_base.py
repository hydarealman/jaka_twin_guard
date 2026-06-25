#!/usr/bin/env python3
"""BT Node 基类 — 兼容 BehaviorTree.CPP 节点模型。

与 jaka_dual_arm/behavior/bt_nodes/bt_node_base.py 完全相同。
独立复制以避免跨包依赖。

定义标准 BT 节点生命周期:
    - on_start()    — 节点第一次 tick 时调用
    - on_running()  — 节点处于 RUNNING 状态时调用
    - on_halt()     — 节点被中断时调用

返回状态:
    - SUCCESS  — 节点成功完成
    - FAILURE  — 节点执行失败
    - RUNNING  — 节点仍在执行中 (异步)

与 BehaviorTree.CPP 的对应:
    SyncActionNode  → BtActionNode (同步)
    AsyncActionNode → BtAsyncNode (异步)
    ConditionNode   → BtCondition
    DecoratorNode   → BtDecorator
    ControlNode     → Sequence / Fallback / Parallel

参考:
  - BehaviorTree.CPP v4.x — 标准 BT 库 (C++)
  - py_trees — Python BT 库 (ROS2 Python 生态)
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import Any, Optional


class NodeStatus(Enum):
    """BT 节点状态。"""
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()
    IDLE = auto()


class BtNode(ABC):
    """BT 节点基类。"""

    def __init__(self, name: str = ""):
        self.name = name
        self._status = NodeStatus.IDLE
        self.blackboard: dict[str, Any] = {}
        self.config: dict[str, Any] = {}

    @property
    def status(self) -> NodeStatus:
        return self._status

    def tick(self) -> NodeStatus:
        if self._status == NodeStatus.IDLE:
            self.on_start()
            self._status = NodeStatus.RUNNING

        if self._status == NodeStatus.RUNNING:
            try:
                self._status = self.on_running()
            except Exception as e:
                self._status = NodeStatus.FAILURE
                self.on_halt()
                raise

        return self._status

    def halt(self):
        if self._status == NodeStatus.RUNNING:
            self.on_halt()
            self._status = NodeStatus.FAILURE

    def reset(self):
        self._status = NodeStatus.IDLE

    def on_start(self):
        pass

    def on_running(self) -> NodeStatus:
        return NodeStatus.SUCCESS

    def on_halt(self):
        pass


class BtActionNode(BtNode):
    """同步 Action 节点。"""

    @abstractmethod
    def execute(self) -> NodeStatus:
        ...

    def on_running(self) -> NodeStatus:
        return self.execute()


class BtAsyncNode(BtNode):
    """异步 Action 节点 — 支持 RUNNING 状态。"""

    def __init__(self, name: str = ""):
        super().__init__(name)
        self._goal_sent = False

    def on_start(self):
        super().on_start()
        self._goal_sent = False

    def on_running(self) -> NodeStatus:
        if not self._goal_sent:
            self.send_goal()
            self._goal_sent = True
            return NodeStatus.RUNNING
        return self.check_result()

    @abstractmethod
    def send_goal(self):
        ...

    @abstractmethod
    def check_result(self) -> NodeStatus:
        ...


class BtCondition(BtNode):
    """条件节点。"""

    def on_running(self) -> NodeStatus:
        if self.evaluate():
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

    @abstractmethod
    def evaluate(self) -> bool:
        ...


# ── Control Nodes ────────────────────────────────────────────


class Sequence(BtNode):
    """顺序节点 — 依次执行子节点，任一 FAILURE 则整体 FAILURE。"""

    def __init__(self, name: str = "", children: list[BtNode] = None):
        super().__init__(name)
        self.children = children or []
        self._current_index = 0

    def on_start(self):
        self._current_index = 0

    def on_running(self) -> NodeStatus:
        while self._current_index < len(self.children):
            child = self.children[self._current_index]
            status = child.tick()
            if status == NodeStatus.FAILURE:
                self._current_index = 0
                return NodeStatus.FAILURE
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            self._current_index += 1
        self._current_index = 0
        return NodeStatus.SUCCESS

    def on_halt(self):
        if self._current_index < len(self.children):
            self.children[self._current_index].halt()


class Fallback(BtNode):
    """备选节点 — 依次尝试子节点，任一 SUCCESS 则整体 SUCCESS。"""

    def __init__(self, name: str = "", children: list[BtNode] = None):
        super().__init__(name)
        self.children = children or []
        self._current_index = 0

    def on_start(self):
        self._current_index = 0

    def on_running(self) -> NodeStatus:
        while self._current_index < len(self.children):
            child = self.children[self._current_index]
            status = child.tick()
            if status == NodeStatus.SUCCESS:
                self._current_index = 0
                return NodeStatus.SUCCESS
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            self._current_index += 1
        self._current_index = 0
        return NodeStatus.FAILURE

    def on_halt(self):
        if self._current_index < len(self.children):
            self.children[self._current_index].halt()


class BtDecorator(BtNode):
    """装饰器节点。"""

    def __init__(self, name: str = "", child: BtNode = None):
        super().__init__(name)
        self.child = child

    def on_running(self) -> NodeStatus:
        if self.child is None:
            return NodeStatus.SUCCESS
        return self.child.tick()

    def on_halt(self):
        if self.child:
            self.child.halt()


class RetryNode(BtDecorator):
    """重试节点 — 子节点 FAILURE 时重试 n 次。"""

    def __init__(self, name: str = "", child: BtNode = None, max_retries: int = 3):
        super().__init__(name, child)
        self.max_retries = max_retries
        self._retry_count = 0

    def on_start(self):
        self._retry_count = 0

    def on_running(self) -> NodeStatus:
        if self.child is None:
            return NodeStatus.FAILURE
        status = self.child.tick()
        if status == NodeStatus.SUCCESS:
            return NodeStatus.SUCCESS
        elif status == NodeStatus.RUNNING:
            return NodeStatus.RUNNING
        self._retry_count += 1
        if self._retry_count < self.max_retries:
            self.child.reset()
            return NodeStatus.RUNNING
        return NodeStatus.FAILURE
