#!/usr/bin/env python3
"""Pick-and-Place Behavior Tree Nodes.

Custom BT nodes for single-arm pick-and-place task orchestration.
Each node reads/writes the shared blackboard to coordinate the pipeline.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node

from fruit_picking_arm.behavior.bt_node_base import (
    NodeStatus, BtActionNode, BtAsyncNode, BtCondition,
)

if TYPE_CHECKING:
    from fruit_picking_arm.behavior.bt_engine import NodeRegistry


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
            if planner.has_joint_states() and planner.services_ready():
                node.get_logger().info(
                    "Complete joint state and MoveIt/controller endpoints ready."
                )
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

        if not scene_mgr.register_table():
            node.get_logger().error("Scene setup failed: table registration")
            return NodeStatus.FAILURE
        if not scene_mgr.register_bin():
            node.get_logger().error("Scene setup failed: bin registration")
            return NodeStatus.FAILURE
        if self.blackboard.get("simulation_mode", False):
            expected = len(
                self.blackboard.get("scene_config", {}).get("objects", [])
            )
            registered = scene_mgr.register_all_objects()
            if registered != expected:
                node.get_logger().error(
                    f"Scene setup failed: registered {registered}/{expected} "
                    "simulation fruit"
                )
                return NodeStatus.FAILURE

        node.get_logger().info("Scene setup complete.")
        return NodeStatus.SUCCESS


class DetectObjects(BtActionNode):
    """Run perception pipeline to detect objects on the table."""

    def execute(self) -> NodeStatus:
        from fruit_picking_arm.skills.detect_objects import DetectObjectsSkill

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("detect_objects", {})

        skill = DetectObjectsSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)

        detector = self.blackboard.get("object_detector")
        if detector is not None:
            skill.set_detector(detector)

        result = skill.run()
        from fruit_picking_arm.skills.base_skill import SkillResult
        if result != SkillResult.SUCCESS:
            return NodeStatus.FAILURE

        # YAML fruit are simulation fixtures only. On real hardware, register
        # the exact perceived fruit geometry and fail before motion if MoveIt
        # cannot accept every collision object.
        if not self.blackboard.get("simulation_mode", False):
            scene_mgr = self.blackboard.get("scene_manager")
            objects = self.blackboard.get("detected_objects", [])
            if scene_mgr is None or not scene_mgr.register_detected_objects(objects):
                node.get_logger().error(
                    "Failed to register perceived fruit collision objects"
                )
                return NodeStatus.FAILURE
        return NodeStatus.SUCCESS


class PlanApproach(BtActionNode):
    """Plan approach trajectory to target object."""

    def execute(self) -> NodeStatus:
        from fruit_picking_arm.skills.approach import ApproachSkill

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
        from fruit_picking_arm.skills.grasp import GraspSkill

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("grasp", {})

        skill = GraspSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)

        # Contact with the target fruit is intentional.  Remove only that
        # fruit's world collision body before grasp IK; restore it on failure.
        removed = None
        scene_mgr = self.blackboard.get("scene_manager")
        target = self.blackboard.get("target_object")
        if scene_mgr is None or target is None:
            node.get_logger().error("Cannot plan grasp without scene/target")
            return NodeStatus.FAILURE
        if self.blackboard.get("simulation_mode", False):
            if hasattr(target, "centroid"):
                tx, ty, _ = target.centroid
            else:
                position = target.get("position", {})
                tx = target.get("x", position.get("x", 0.0))
                ty = target.get("y", position.get("y", 0.0))
            removed = scene_mgr.remove_nearest_object(float(tx), float(ty))
        else:
            removed = scene_mgr.remove_detected_object(target)
        if removed is None:
            node.get_logger().error("Refusing grasp without exact target collision removal")
            return NodeStatus.FAILURE

        traj = skill.plan()
        if traj is not None:
            if removed is not None:
                self.blackboard["removed_target_collision_id"] = removed[0]
            self.blackboard["current_trajectory"] = traj
            return NodeStatus.SUCCESS
        if not scene_mgr.register_object(removed[1]):
            node.get_logger().error("Failed to restore target collision after plan failure")
        return NodeStatus.FAILURE


class PlanLift(BtActionNode):
    """Plan vertical lift trajectory."""

    def execute(self) -> NodeStatus:
        from fruit_picking_arm.skills.lift import LiftSkill

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
    """Plan AND execute full multi-stage place sequence.

    Internal flow: hover above bin → descend → release gripper → retract.
    All execution is handled synchronously within this node (multi-stage).
    """

    def execute(self) -> NodeStatus:
        from fruit_picking_arm.skills.place import PlaceSkill
        from fruit_picking_arm.skills.base_skill import SkillResult

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("place", {})

        skill = PlaceSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)

        result = skill.run()
        if result == SkillResult.SUCCESS:
            return NodeStatus.SUCCESS
        node.get_logger().error(f"PlanPlace: {result.name}")
        return NodeStatus.FAILURE


class PlanRetreat(BtActionNode):
    """Plan retreat to HOME pose."""

    def execute(self) -> NodeStatus:
        from fruit_picking_arm.skills.retreat import RetreatSkill

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
            if self.blackboard.get("simulation_mode", False):
                target = self.blackboard.get("target_object")
                radius = float(getattr(target, "radius", 0.03))
                ok = gripper.close_for_radius(radius)
                scene_mgr = self.blackboard.get("scene_manager")
                if ok and scene_mgr is not None:
                    ok = scene_mgr.set_simulated_grasp(True)
                    self.blackboard["simulation_grasp_attached"] = ok
            else:
                ok = gripper.close()
        elif action == "open":
            node.get_logger().info("Opening gripper...")
            ok = gripper.open()
            if (
                ok
                and self.blackboard.get("simulation_mode", False)
                and self.blackboard.get("simulation_grasp_attached", False)
            ):
                scene_mgr = self.blackboard.get("scene_manager")
                ok = scene_mgr is not None and scene_mgr.set_simulated_grasp(False)
                self.blackboard["simulation_grasp_attached"] = False
        else:
            node.get_logger().warn(f"Unknown gripper action: {action}")
            return NodeStatus.FAILURE

        return NodeStatus.SUCCESS if ok else NodeStatus.FAILURE


class CheckGrasp(BtCondition):
    """Fail-closed grasp check using measured opening vs perceived diameter."""

    def evaluate(self) -> bool:
        if self.blackboard.get("simulation_mode", False):
            return bool(self.blackboard.get("simulation_grasp_attached", False))

        planner = self.blackboard.get("planner")
        if planner is None:
            return False

        grip_pos = planner.get_current_gripper_positions()
        target = self.blackboard.get("target_object")
        node: Node = self.blackboard.get("node")
        if len(grip_pos) != 2 or target is None:
            if node is not None:
                node.get_logger().error(
                    "Grasp verification unavailable: missing measured jaws/target"
                )
            return False

        try:
            radius = float(getattr(target, "radius"))
            finger_thickness = float(
                self.blackboard.get("gripper_config", {}).get(
                    "finger_thickness", 0.012
                )
            )
            clear_opening = max(
                0.0, abs(float(grip_pos[0]) - float(grip_pos[1])) - finger_thickness
            )
        except (AttributeError, TypeError, ValueError):
            return False

        expected_diameter = 2.0 * radius
        minimum_held_opening = max(0.005, expected_diameter * 0.50)
        maximum_held_opening = expected_diameter * 1.50 + 0.005
        grasped = minimum_held_opening <= clear_opening <= maximum_held_opening
        if node is not None:
            log = node.get_logger().info if grasped else node.get_logger().error
            log(
                f"Grasp verification: measured opening={clear_opening:.3f}m, "
                f"expected diameter={expected_diameter:.3f}m, grasped={grasped}"
            )
        return grasped


def create_pick_place_node_registry() -> "NodeRegistry":
    """Create and register all pick-and-place BT node types."""
    from fruit_picking_arm.behavior.bt_engine import NodeRegistry

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
