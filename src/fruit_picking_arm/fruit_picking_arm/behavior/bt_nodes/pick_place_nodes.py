#!/usr/bin/env python3
"""Pick-and-Place Behavior Tree Nodes.

Custom BT nodes for single-arm pick-and-place task orchestration.
Each node reads/writes the shared blackboard to coordinate the pipeline.
"""

from __future__ import annotations

import time
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

        # Startup timeout is wall-clock based. Gazebo simulated time may run
        # faster or pause, neither of which should shorten or freeze this gate.
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            # Use the task's shared executor. Calling rclpy.spin_once(planner)
            # here would transfer the planner out of that executor and break
            # later operator-approval callbacks.
            planner.spin_callbacks_once(timeout_sec=0.1)
            # In mock_components, joints start at 0.0 — that's a valid state.
            # Check that joint_states have been received (not that values are non-zero).
            if planner.has_joint_states() and planner.services_ready():
                node.get_logger().info(
                    "Complete joint state and MoveIt/controller endpoints ready."
                )
                return True
            if time.monotonic() > deadline:
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

        scene_config = self.blackboard.get("scene_config", {})
        if (
            scene_config.get("profile") == "real"
            and not bool(scene_config.get("workcell_surveyed", False))
        ):
            node.get_logger().error(
                "Real sorting bins are not surveyed. The tabletop comes from "
                "fresh D455 depth; measure both bins, fill "
                "scene_params_real.yaml, then set workcell_surveyed: true."
            )
            return NodeStatus.FAILURE

        if self.blackboard.get("external_perception", False):
            timeout = float(self.config.get("table_timeout", 15.0))
            deadline = time.monotonic() + timeout
            surface = self.blackboard.get("perceived_table")
            while surface is None and rclpy.ok():
                planner = self.blackboard.get("planner")
                planner.spin_callbacks_once(timeout_sec=0.1)
                surface = self.blackboard.get("perceived_table")
                if time.monotonic() > deadline:
                    break
            if surface is None or not scene_mgr.register_perceived_table(surface):
                node.get_logger().error(
                    "Scene setup failed: no fresh RGB-D perceived table"
                )
                return NodeStatus.FAILURE
        elif not scene_mgr.register_table():
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
    """Plan the motion from the lifted fruit to the selected bin hover."""

    def execute(self) -> NodeStatus:
        from fruit_picking_arm.skills.place import PlaceSkill
        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("place", {})

        skill = PlaceSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)

        trajectory = skill.plan()
        if trajectory is not None:
            self.blackboard["current_trajectory"] = trajectory
            return NodeStatus.SUCCESS
        node.get_logger().error("PlanPlace hover failed")
        return NodeStatus.FAILURE


class PlanPlaceDrop(BtActionNode):
    """Plan the inspected descent from bin hover to the release pose."""

    def execute(self) -> NodeStatus:
        from fruit_picking_arm.skills.place import PlaceSkill

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("place", {})
        skill = PlaceSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)
        trajectory = skill.plan_drop()
        if trajectory is not None:
            self.blackboard["current_trajectory"] = trajectory
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class PlanPlaceRetract(BtActionNode):
    """Plan the inspected vertical retreat after releasing the fruit."""

    def execute(self) -> NodeStatus:
        from fruit_picking_arm.skills.place import PlaceSkill

        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        skill_cfg = self.blackboard.get("skill_config", {}).get("place", {})
        skill = PlaceSkill()
        skill.configure(node, planner, skill_cfg, self.blackboard)
        trajectory = skill.plan_retract()
        if trajectory is not None:
            self.blackboard["current_trajectory"] = trajectory
            return NodeStatus.SUCCESS
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
        self._label = ""

    def send_goal(self):
        node: Node = self.blackboard.get("node")
        planner = self.blackboard.get("planner")
        traj = self.blackboard.get("current_trajectory")

        if planner is None or traj is None:
            return

        duration = (
            traj.points[-1].time_from_start.sec
            + traj.points[-1].time_from_start.nanosec / 1e9
        )
        self._label = (
            f"{self.name}（{len(traj.points)}个轨迹点，约{duration:.1f}秒）"
        )
        if not planner.publish_trajectory_preview(traj):
            self._result_future = False
            return
        node.get_logger().info(f"Executing trajectory: {self._label}")
        started_at = time.monotonic()
        self._result_future = planner.execute(traj)
        node.get_logger().info(
            f"STAGE_TIMING: stage={self.name} planned_s={duration:.3f} "
            f"actual_s={time.monotonic() - started_at:.3f} "
            f"success={bool(self._result_future)}"
        )

    def check_result(self) -> NodeStatus:
        if self._result_future is None:
            return NodeStatus.FAILURE
        if self._result_future:
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class ControlGripper(BtActionNode):
    """Open or close gripper based on config['action'].

    action="close" → close gripper
        action="open"  → open gripper and detach the released fruit
        action="prepare_open" → open before descent without detaching
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
                scene_mgr = self.blackboard.get("scene_manager")
                target = self.blackboard.get("target_object")
                if ok and scene_mgr is not None and target is not None:
                    ok = scene_mgr.attach_detected_object(target)
        elif action in ("open", "prepare_open"):
            node.get_logger().info("Opening gripper...")
            ok = gripper.open()
            if (
                ok
                and action == "open"
                and self.blackboard.get("simulation_mode", False)
                and self.blackboard.get("simulation_grasp_attached", False)
            ):
                scene_mgr = self.blackboard.get("scene_manager")
                ok = scene_mgr is not None and scene_mgr.set_simulated_grasp(False)
                self.blackboard["simulation_grasp_attached"] = False
            elif (
                ok
                and action == "open"
                and not self.blackboard.get("simulation_mode", False)
            ):
                scene_mgr = self.blackboard.get("scene_manager")
                target = self.blackboard.get("target_object")
                ok = (
                    scene_mgr is not None
                    and target is not None
                    and scene_mgr.detach_detected_object(target)
                )
        else:
            node.get_logger().warn(f"Unknown gripper action: {action}")
            return NodeStatus.FAILURE

        return NodeStatus.SUCCESS if ok else NodeStatus.FAILURE


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
    registry.register("PlanPlaceDrop", lambda: PlanPlaceDrop())
    registry.register("PlanPlaceRetract", lambda: PlanPlaceRetract())
    registry.register("PlanRetreat", lambda: PlanRetreat())
    registry.register("ExecuteTrajectory", lambda: ExecuteTrajectory())
    registry.register("ControlGripper", lambda: ControlGripper())

    return registry
