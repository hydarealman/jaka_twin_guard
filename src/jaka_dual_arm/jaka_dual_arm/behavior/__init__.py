"""Layer 5: Task Orchestration — BT Engine + BT Nodes + Task Runner.

模块:
    bt_engine    — 行为树执行引擎 (XML 加载 + tick 驱动)
    bt_runner    — 搬运任务运行器 (内联状态机 + 安全监控 + 力控)
    bt_nodes/    — 自定义 BT 节点库 (兼容 BehaviorTree.CPP v4)
    trees/       — BT XML 定义文件 (兼容 Groot 可视化编辑器)
"""
