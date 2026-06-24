#!/usr/bin/env python3
"""BehaviorTree Runner — 加载并运行行为树任务。

这是任务编排的顶层入口。行为树定义了 pick-and-place 流程:
    Detect → PlanPick → Execute → CheckGrasp → PlanPlace → Execute

用法:
    runner = BTRunner(node, config)
    runner.build_tree()      # 构建行为树
    runner.run()             # 循环 tick

参考:
  - ManyMove (pastoriomarco/manymove) — BT 节点库 + ROS2 Action 集成模式
  - BehaviorTree.CPP — XML 定义任务流、黑板数据共享
  - py_trees — Python 行为树运行时（本项目直接依赖）
"""

from __future__ import annotations

import py_trees
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

from jaka_dual_arm.scene.scene_manager import SceneManager
from jaka_dual_arm.scene.perception_interface import PerceptionInterface
from jaka_dual_arm.planner.planner_server import DualArmPlannerServer
from jaka_dual_arm.skills.approach import ApproachSkill
from jaka_dual_arm.skills.grasp import GraspSkill
from jaka_dual_arm.skills.lift import LiftSkill
from jaka_dual_arm.skills.place import PlaceSkill
from jaka_dual_arm.behavior.bt_nodes.detect_object import DetectObject
from jaka_dual_arm.behavior.bt_nodes.plan_pick import PlanPick
from jaka_dual_arm.behavior.bt_nodes.plan_place import PlanPlace
from jaka_dual_arm.behavior.bt_nodes.check_grasp import CheckGrasp


class CarryTaskRunner(Node):
    """双臂搬运任务运行器。

    集成了:
    - SceneManager    (场景管理)
    - Perception      (感知接口)
    - PlannerServer   (运动规划)
    - Skill Library   (操作技能)
    - BehaviorTree    (任务编排)
    """

    def __init__(
        self,
        scene_config: dict,
        robot_config: dict,
        planner_config: dict,
        skill_config: dict,
    ):
        super().__init__("carry_task_runner")

        # ── Layer 2: Scene & Perception ──
        self.scene_mgr = SceneManager()
        self.scene_mgr.set_scene_dict(scene_config)
        self.perception = PerceptionInterface(
            scene_config.get("world_frame", "world")
        )

        # ── Layer 3: Planner ──
        self.planner = DualArmPlannerServer()

        # ── Layer 4: Skills ──
        left_joints = robot_config["joints"]["left"]
        right_joints = robot_config["joints"]["right"]
        ctrl = robot_config["controllers"]

        self.left_client = ActionClient(
            self, FollowJointTrajectory, ctrl["left"]
        )
        self.right_client = ActionClient(
            self, FollowJointTrajectory, ctrl["right"]
        )

        skill_kwargs = dict(
            node=self,
            planner=self.planner,
            left_client=self.left_client,
            right_client=self.right_client,
            left_joints=left_joints,
            right_joints=right_joints,
        )

        self.approach = ApproachSkill(**skill_kwargs)
        self.grasp = GraspSkill(**skill_kwargs)
        self.lift = LiftSkill(**skill_kwargs)
        self.place = PlaceSkill(**skill_kwargs)

        # 配置技能参数
        self.approach.configure(skill_config.get("approach", {}))
        self.grasp.configure(skill_config.get("grasp", {}))
        self.lift.configure(skill_config.get("lift", {}))
        self.place.configure(skill_config.get("place", {}))

        # ── Layer 5: BehaviorTree ──
        self._tree: py_trees.trees.BehaviourTree | None = None

    # ── Setup ──────────────────────────────────────────

    def setup_scene(self) -> bool:
        """初始化场景：注册桌子、货物等碰撞对象。"""
        if not self.scene_mgr.register_table():
            self.get_logger().warn("Failed to register table in PlanningScene")
        if not self.scene_mgr.register_cargo():
            self.get_logger().warn("Failed to register cargo in PlanningScene")
        if not self.scene_mgr.register_bin():
            self.get_logger().debug("Bin not enabled or registration failed")

        # 从 YAML 中的初始位姿注册 Mock 感知物体
        cargo = self.scene_mgr._config.get("cargo", {})
        init = cargo.get("initial_pose", {})
        size = cargo.get("size", {})
        self.perception.add_mock_object(
            "cargo_box",
            (init.get("x", 0.36), init.get("y", 0.02), init.get("z", 0.06)),
            size=(
                size.get("x", 0.18),
                size.get("y", 0.345),
                size.get("z", 0.12),
            ),
        )
        return True

    def build_tree(self) -> py_trees.trees.BehaviourTree:
        """构建搬运任务行为树。

        结构:
            Sequence ("Carry Task")
            ├── DetectObject("cargo_box")
            ├── PlanPick
            ├── CheckGrasp
            │   └── Fallback
            │       ├── Retry (max 3)
            │       └── GoHome (placeholder)
            ├── Lift
            ├── PlanPlace
            └── Release
        """
        root = py_trees.composites.Sequence("CarryTask", memory=True)

        # 检测物体
        detect = DetectObject("DetectCargo", "cargo_box", self.perception)

        # 抓取
        plan_pick = PlanPick("PlanPick", self.grasp)

        # 抓取验证 → 重试或回退
        check = CheckGrasp("CheckGrasp")
        retry = py_trees.decorators.Retry(
            "RetryGrasp",
            child=py_trees.composites.Sequence(
                "RetrySeq", memory=False,
                children=[
                    PlanPick("RetryPick", self.grasp),
                    CheckGrasp("RetryCheck"),
                ],
            ),
            num_attempts=3,
        )
        grasp_fallback = py_trees.composites.Selector(
            "GraspOrRetry", memory=False,
            children=[check, retry],
        )

        # 放置
        plan_place = PlanPlace("PlanPlace", self.place)

        # 组装树
        root.add_children([
            detect,
            plan_pick,
            grasp_fallback,
            plan_place,
        ])

        self._tree = py_trees.trees.BehaviourTree(root)
        return self._tree

    def run_cycle(self, tick_period: float = 0.1):
        """以固定频率循环 tick 行为树。"""
        if self._tree is None:
            self.build_tree()

        self._tree.setup(timeout=15.0)

        def _tick():
            self._tree.tick()
            status = self._tree.root.status
            if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                self.get_logger().info(f"Task finished: {status}")
                rclpy.shutdown()

        self.create_timer(tick_period, _tick)
        self.get_logger().info("BehaviorTree runner started.")

    def run(self):
        """运行行为树（阻塞模式）。"""
        if self._tree is None:
            self.build_tree()

        self._tree.setup(timeout=15.0)
        try:
            while rclpy.ok():
                self._tree.tick()
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._tree.root.status in (
                    py_trees.common.Status.SUCCESS,
                    py_trees.common.Status.FAILURE,
                ):
                    self.get_logger().info(
                        f"Task finished: {self._tree.root.status}"
                    )
                    break
        except KeyboardInterrupt:
            pass
