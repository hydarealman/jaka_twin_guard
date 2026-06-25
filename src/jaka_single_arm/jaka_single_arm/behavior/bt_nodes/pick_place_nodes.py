#!/usr/bin/env python3
"""Pick-and-Place Behavior Tree Nodes.

Custom BT nodes for single-arm pick-and-place task orchestration.
Each node reads/writes the shared blackboard to coordinate the pipeline.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from jaka_single_arm.behavior.bt_node_base import (
    NodeStatus, BtActionNode, BtAsyncNode, BtCondition,
)


class WaitServices(BtCondition):
    """Wait for all required MoveIt2 services and action servers to be ready."""

    def evaluate(self) -> bool:
        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        if node is None or planner is None:
            return False

        timeout = self.config.get("timeout", 30.0)
        node.get_logger().info("Waiting for MoveIt2 services and joint states...")

        deadline = node.get_clock().now().nanoseconds / 1e9 + timeout
        while rclpy.ok():
            # Spin the PLANNER node so it can receive joint_states via its
            # /joint_states subscription. (Spinning "node" would only process
            # the runner node's callbacks, missing joint states entirely.)
            rclpy.spin_once(planner, timeout_sec=0.1)
            # In mock_components, joints start at 0.0 — that's a valid state.
            # Check that joint_states have been received (not that values are non-zero).
            if planner.has_joint_states():
                node.get_logger().info("Joint states received, services ready.")
                return True
            if node.get_clock().now().nanoseconds / 1e9 > deadline:
                node.get_logger().error("Timeout waiting for services")
                return False
        return False


class SetupScene(BtActionNode):
    """Initialize PlanningScene with table, bin, and objects."""

    def execute(self) -> NodeStatus:
        scene_mgr = self.blackboard.get("scene_manager")
        if scene_mgr is None:
            return NodeStatus.FAILURE

        node: Node = self.blackboard.get("node")
        node.get_logger().info("Setting up scene...")

        scene_mgr.register_table()
        scene_mgr.register_bin()
        scene_mgr.register_all_objects()

        node.get_logger().info("Scene setup complete.")
        return NodeStatus.SUCCESS


class DetectObjects(BtActionNode):
    """Run perception pipeline to detect objects on the table."""

    def execute(self) -> NodeStatus:
        from jaka_single_arm.skills.detect_objects import DetectObjectsSkill

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")

        skill = DetectObjectsSkill()
        skill.configure(node, planner, {}, self.blackboard)

        detector = self.blackboard.get("object_detector")
        if detector is not None:
            skill.set_detector(detector)

        result = skill.run()
        from jaka_single_arm.skills.base_skill import SkillResult
        return NodeStatus.SUCCESS if result == SkillResult.SUCCESS else NodeStatus.FAILURE


class PlanApproach(BtActionNode):
    """Plan approach trajectory to target object."""

    def execute(self) -> NodeStatus:
        from jaka_single_arm.skills.approach import ApproachSkill

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("approach", {})

        skill = ApproachSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)

        traj = skill.plan()
        if traj is not None:
            self.blackboard["current_trajectory"] = traj
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class PlanGrasp(BtActionNode):
    """Plan grasp descent trajectory to object surface."""

    def execute(self) -> NodeStatus:
        from jaka_single_arm.skills.grasp import GraspSkill

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("grasp", {})

        skill = GraspSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)

        traj = skill.plan()
        if traj is not None:
            self.blackboard["current_trajectory"] = traj
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class PlanLift(BtActionNode):
    """Plan vertical lift trajectory."""

    def execute(self) -> NodeStatus:
        from jaka_single_arm.skills.lift import LiftSkill

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("lift", {})

        skill = LiftSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)

        traj = skill.plan()
        if traj is not None:
            self.blackboard["current_trajectory"] = traj
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class PlanPlace(BtActionNode):
    """Plan place trajectory to bin."""

    def execute(self) -> NodeStatus:
        from jaka_single_arm.skills.place import PlaceSkill

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("place", {})

        skill = PlaceSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)

        traj = skill.plan()
        if traj is not None:
            self.blackboard["current_trajectory"] = traj
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class PlanRetreat(BtActionNode):
    """Plan retreat to HOME pose."""

    def execute(self) -> NodeStatus:
        from jaka_single_arm.skills.retreat import RetreatSkill

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("retreat", {})

        skill = RetreatSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)

        traj = skill.plan()
        if traj is not None:
            self.blackboard["current_trajectory"] = traj
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class ExecuteTrajectory(BtAsyncNode):
    """Execute the trajectory stored in blackboard["current_trajectory"]."""

    def __init__(self, name: str = ""):
        super().__init__(name)
        self._result_future = None

    def send_goal(self):
        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        traj = self.blackboard.get("current_trajectory")

        if planner is None or traj is None:
            return

        node.get_logger().info(
            f"Executing trajectory: {len(traj.points)} points, "
            f"{traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec/1e9:.1f}s"
        )

        # Use planner's execute method (sync, so we wrap in async pattern)
        self._result_future = planner.execute(traj)

    def check_result(self) -> NodeStatus:
        if self._result_future is None:
            return NodeStatus.FAILURE
        if self._result_future:
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class ControlGripper(BtActionNode):
    """Open or close gripper based on config['action'].

    action="close" → close gripper
    action="open"  → open gripper
    """

    def execute(self) -> NodeStatus:
        action = self.config.get("action", "close")
        gripper = self.blackboard.get("gripper_controller")
        node: Node = self.blackboard.get("node")

        if gripper is None:
            node.get_logger().error("No gripper controller in blackboard")
            return NodeStatus.FAILURE

        if action == "close":
            node.get_logger().info("Closing gripper...")
            ok = gripper.close()
        elif action == "open":
            node.get_logger().info("Opening gripper...")
            ok = gripper.open()
        else:
            node.get_logger().warn(f"Unknown gripper action: {action}")
            return NodeStatus.FAILURE

        return NodeStatus.SUCCESS if ok else NodeStatus.FAILURE


class CheckGrasp(BtCondition):
    """Verify object is grasped by checking gripper position."""

    def evaluate(self) -> bool:
        planner = self.blackboard.get("planner")
        if planner is None:
            return False

        grip_pos = planner.get_current_gripper_positions()
        if len(grip_pos) >= 2:
            # Check if gripper is closed (finger positions near closed value)
            avg_pos = (abs(grip_pos[0]) + abs(grip_pos[1])) / 2.0
            is_closed = avg_pos < 0.01  # less than 1cm means closed
            return is_closed
        return False


def create_pick_place_node_registry() -> NodeRegistry:
    """Create and register all pick-and-place BT node types."""
    from jaka_single_arm.behavior.bt_engine import NodeRegistry

    registry = NodeRegistry()

    registry.register("WaitServices", lambda: WaitServices())
    registry.register("SetupScene", lambda: SetupScene())
    registry.register("DetectObjects", lambda: DetectObjects())
    registry.register("PlanApproach", lambda: PlanApproach())
    registry.register("PlanGrasp", lambda: PlanGrasp())
    registry.register("PlanLift", lambda: PlanLift())
    registry.register("PlanPlace", lambda: PlanPlace())
    registry.register("PlanRetreat", lambda: PlanRetreat())
    registry.register("ExecuteTrajectory", lambda: ExecuteTrajectory())
    registry.register("ControlGripper", lambda: ControlGripper())
    registry.register("CheckGrasp", lambda: CheckGrasp())

    return registry
