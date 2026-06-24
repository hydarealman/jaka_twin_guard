"""BT 节点库 — 兼容 BehaviorTree.CPP v4 模型。

节点模块:
    bt_node_base  — BT 基类 (BtNode, BtActionNode, BtAsyncNode, BtCondition)
    carry_nodes   — 搬运任务专用节点 (DetectObject, PlanPhase, ExecuteTrajectory, CheckGrasp)

控制节点:
    Sequence       — 顺序执行 (所有子节点 SUCCESS 才整体 SUCCESS)
    Fallback       — 备选执行 (任一子节点 SUCCESS 则整体 SUCCESS)
    RetryNode      — 重试装饰器 (子节点 FAILURE 时重试 n 次)
    BtDecorator    — 装饰器基类

引擎:
    BtEngine       — BT 执行器 (加载 XML，tick 驱动)
    BtXmlParser    — XML 解析器 (BehaviorTree.CPP v4 格式)
    NodeRegistry   — 节点工厂 (类型名 → 实例)

与 BehaviorTree.CPP 的对应:
    SyncActionNode  → BtActionNode
    AsyncActionNode → BtAsyncNode
    ConditionNode   → BtCondition
    ControlNode     → Sequence / Fallback
    DecoratorNode   → RetryNode / BtDecorator
    Blackboard      → dict[str, Any]

参考:
  - BehaviorTree.CPP v4.x (C++ 标准 BT 库)
  - py_trees (Python BT 库)
  - Groot (BT 可视化编辑器)
"""
