#!/usr/bin/env python3
"""Pick-and-Place Task Runner — orchestrates the full pipeline.

Creates all layer components, fills the blackboard, loads the BT tree,
and runs the main execution loop.

Reference:
  - jaka_dual_arm/behavior/bt_runner.py — carry task runner
  - jaka_dual_arm/massage/massage_runner.py — massage runner node
"""

from __future__ import annotations

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from jaka_single_arm.control.safety_monitor import SafetyMonitor, SafetyLimits, SafetyLevel
from jaka_single_arm.control.gripper_controller import GripperController
from jaka_single_arm.perception import create_camera
from jaka_single_arm.perception.object_detector import ObjectDetector
from jaka_single_arm.scene.scene_manager import SceneManager
from jaka_single_arm.planner.planner_server import SingleArmPlannerServer
from jaka_single_arm.behavior.bt_engine import BtEngine, NodeRegistry
from jaka_single_arm.behavior.bt_nodes.pick_place_nodes import create_pick_place_node_registry
from jaka_single_arm.behavior.bt_node_base import NodeStatus


class PickPlaceRunner(Node):
    """Main runner node for single-arm pick-and-place.

    Creates all components, sets up the behavior tree, and runs the
    main tick loop. Supports multi-object processing by iterating
    over detected_objects and restarting the per-object BT subtree.

    Usage:
        node = PickPlaceRunner(robot_cfg, scene_cfg, ...)
        node.run()
    """

    def __init__(self, robot_cfg: dict, scene_cfg: dict,
                 perception_cfg: dict, planner_cfg: dict,
                 skill_cfg: dict, behavior_cfg: dict,
                 safety_cfg: dict, gripper_cfg: dict):
        super().__init__("pick_place_runner")

        self._robot_cfg = robot_cfg
        self._scene_cfg = scene_cfg
        self._perception_cfg = perception_cfg
        self._planner_cfg = planner_cfg
        self._skill_cfg = skill_cfg
        self._behavior_cfg = behavior_cfg
        self._safety_cfg = safety_cfg
        self._gripper_cfg = gripper_cfg

        # ── Create Layer Components ──────────────────────────

        # Layer 1: Control
        self.get_logger().info("Initializing control layer...")
        limits = SafetyLimits.from_yaml(safety_cfg)
        self._safety = SafetyMonitor(self, limits, on_estop=self._on_estop)
        self._safety.set_arm_joints(robot_cfg.get("arm_joints",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]))

        # Layer 2: Scene
        self.get_logger().info("Initializing scene manager...")
        self._scene_mgr = SceneManager()
        self._scene_mgr.set_scene_dict(scene_cfg)

        # Layer 2: Perception
        self.get_logger().info("Initializing perception...")
        # Allow launch file to override camera_type (e.g. "gazebo" from sim_gazebo.launch.py)
        self.declare_parameter("camera_type", perception_cfg.get("camera_type", "mock"))
        override_type = self.get_parameter("camera_type").value
        if override_type != perception_cfg.get("camera_type"):
            self.get_logger().info(
                f"camera_type overridden by launch: "
                f"'{perception_cfg.get('camera_type')}' → '{override_type}'"
            )
            perception_cfg = dict(perception_cfg)
            perception_cfg["camera_type"] = override_type
        self._camera = create_camera(self, perception_cfg, scene_cfg)
        self._camera.connect()
        self._object_detector = ObjectDetector(self, perception_cfg)

        # Layer 3: Planner
        self.get_logger().info("Initializing planner server...")
        self._planner = SingleArmPlannerServer()
        self._planner.configure(planner_cfg, robot_cfg)
        # Critical: register runner with planner so safety monitor's
        # joint_state callback keeps firing during blocking plan+execute calls.
        self._planner.set_runner_node(self)

        # Gripper
        self._gripper = GripperController(self, self._planner, gripper_cfg)

        # Layer 5: Behavior Tree
        self.get_logger().info("Initializing behavior tree...")
        self._blackboard: dict = {
            "node": self,
            "planner": self._planner,
            "scene_manager": self._scene_mgr,
            "camera": self._camera,
            "object_detector": self._object_detector,
            "gripper_controller": self._gripper,
            "safety_monitor": self._safety,
            "scene_config": scene_cfg,
            "robot_config": robot_cfg,
            "skill_config": skill_cfg,
            "behavior_config": behavior_cfg,
            "perception_config": perception_cfg,
            "home_pose": scene_cfg.get("home_pose", [0.0, 1.5, -1.5, 1.5, 1.57, 0.0]),
            "detected_objects": [],
            "detection_count": 0,
            "target_object": None,
            "current_trajectory": None,
        }

        # Set first object as target (from YAML — perception will override)
        objects = scene_cfg.get("objects", [])
        if objects:
            self._blackboard["target_object"] = objects[0]
            self.get_logger().info(f"Initial target: {objects[0].get('label', objects[0].get('id'))}")

        # Create node registry and BT engine
        self._registry = create_pick_place_node_registry()
        self._engine = BtEngine(self, self._blackboard, self._registry)

        # Load BT XML
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("jaka_single_arm")
        xml_path = os.path.join(share_dir, "behavior", "trees", "pick_place_task.xml")
        self._engine.load_xml(xml_path)

        # Markers for RViz
        self._marker_pub = self.create_publisher(MarkerArray, "/rviz_visual_tools", 10)
        self._marker_timer = self.create_timer(0.2, self._publish_markers)

        # Publish markers IMMEDIATELY (before main loop) so RViz shows
        # scene objects as soon as the node starts.
        self._publish_markers()
        self.get_logger().info("Scene markers published (table, bin, fruits).")

        self.get_logger().info("PickPlaceRunner initialized. Ready to run.")

    def run(self) -> bool:
        """Execute the BT main loop. Returns True on success.

        Multi-object strategy:
          1. Run SetupTree once (WaitServices + SetupScene + DetectObjects).
          2. For each detected object, run PickPlaceTree.
          3. If a single-object pick fails, skip to the next object.
        """
        self.get_logger().info("=== Pick-and-Place Task Starting ===")

        # Use MultiThreadedExecutor to spin both nodes
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.add_node(self._planner)

        spin_period = self._behavior_cfg.get("execution", {}).get("spin_period", 0.05)
        check_safety = self._behavior_cfg.get("safety", {}).get("check_every_tick", True)

        # ── Phase 1: Setup + Detection ──────────────────────
        self._engine.select_tree("SetupTree")
        self.get_logger().info("Phase 1: Running setup & detection...")

        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=spin_period)
                status = self._engine.tick()

                if status == NodeStatus.SUCCESS:
                    self.get_logger().info("Setup & detection complete.")
                    break
                elif status == NodeStatus.FAILURE:
                    self.get_logger().error(
                        f"Setup failed: {self._engine.failure_reason}"
                    )
                    self._cleanup(executor)
                    return False

                if check_safety:
                    level = self._safety.check()
                    if level == SafetyLevel.ESTOP:
                        self.get_logger().error("EMERGENCY STOP during setup!")
                        self._cleanup(executor)
                        return False
                    elif level == SafetyLevel.HALT:
                        self.get_logger().error("Safety HALT during setup")
                        self._engine.halt()
                        self._cleanup(executor)
                        return False

        except KeyboardInterrupt:
            self.get_logger().info("Interrupted by user.")
            self._cleanup(executor)
            return False

        # ── Phase 2: Per-object pick-and-place ───────────────
        detected = self._blackboard.get("detected_objects", [])
        if not detected:
            self.get_logger().warn("No objects detected — nothing to pick!")
            self._cleanup(executor)
            return True  # Not a failure — just nothing to do

        self.get_logger().info(
            f"Phase 2: Processing {len(detected)} detected object(s)..."
        )

        self._engine.select_tree("PickPlaceTree")
        success_count = 0

        for obj_idx, obj in enumerate(detected):
            if not rclpy.ok():
                break

            self._blackboard["target_object"] = obj
            self._blackboard["_object_index"] = obj_idx
            oid = getattr(obj, "id", f"obj_{obj_idx}")
            centroid = getattr(obj, "centroid", (0, 0, 0))
            radius = getattr(obj, "radius", 0.0)
            shape = getattr(obj, "shape", "?")
            conf = getattr(obj, "confidence", 0.0)
            self.get_logger().info(
                f"--- Object {obj_idx + 1}/{len(detected)}: {oid} "
                f"@ ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}) "
                f"r={radius:.3f} shape={shape} conf={conf:.2f} ---"
            )

            # Reset BT for this object
            self._engine.reset()

            try:
                while rclpy.ok():
                    executor.spin_once(timeout_sec=spin_period)
                    status = self._engine.tick()

                    if status == NodeStatus.SUCCESS:
                        self.get_logger().info(
                            f"Object {oid}: PICK & PLACE SUCCESS"
                        )
                        success_count += 1
                        break
                    elif status == NodeStatus.FAILURE:
                        self.get_logger().error(
                            f"Object {oid}: FAILED — {self._engine.failure_reason}"
                        )
                        break

                    if check_safety:
                        level = self._safety.check()
                        if level == SafetyLevel.ESTOP:
                            self.get_logger().error("EMERGENCY STOP triggered!")
                            self._cleanup(executor)
                            return False
                        elif level == SafetyLevel.HALT:
                            self.get_logger().error("Safety HALT — stopping task")
                            self._engine.halt()
                            self._cleanup(executor)
                            return False

            except Exception as e:
                import traceback
                self.get_logger().error(
                    f"Exception processing {oid}: {e}\n{traceback.format_exc()}"
                )
                continue

        self.get_logger().info(
            f"=== Task Complete: {success_count}/{len(detected)} objects placed ==="
        )
        self._cleanup(executor)
        return success_count > 0

    def _cleanup(self, executor: MultiThreadedExecutor):
        """Stop camera and remove nodes from executor."""
        self._camera.disconnect()
        try:
            executor.remove_node(self._planner)
            executor.remove_node(self)
        except Exception:
            pass

    def _on_estop(self, violations: list[str]):
        """Emergency stop callback."""
        self.get_logger().fatal(f"ESTOP triggered: {violations}")

    # ── RViz Visualization ──────────────────────────────────

    def _publish_markers(self) -> None:
        """Publish scene visualization markers."""
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()
        mid = 0

        scene = self._scene_cfg

        # Table
        table = scene.get("table", {})
        if table:
            tc = table["center"]
            ts = table["size"]
            top_z = table["top_z"]
            tcz = top_z - ts["z"] / 2.0
            ma.markers.append(self._cube_marker(
                now, mid, "table", tc["x"], tc["y"], tcz,
                ts["x"], ts["y"], ts["z"], (0.50, 0.35, 0.22, 0.85)
            ))
            mid += 1

        # Bin
        bin_cfg = scene.get("bin", {})
        if bin_cfg:
            bc = bin_cfg["center"]
            bs = bin_cfg["size"]
            top_z = bin_cfg["top_z"]
            bcz = top_z - bs["z"] / 2.0
            ma.markers.append(self._cube_marker(
                now, mid, "bin", bc["x"], bc["y"], bcz,
                bs["x"], bs["y"], bs["z"], (0.25, 0.40, 0.60, 0.65)
            ))
            mid += 1

        # Objects from YAML
        for obj in scene.get("objects", []):
            pos = obj.get("position", {})
            radius = obj.get("radius", 0.03)
            color = obj.get("color", [1.0, 1.0, 1.0, 0.9])
            table_top = table.get("top_z", 0.30) if table else 0.30

            ma.markers.append(self._sphere_marker(
                now, mid, "objects",
                pos.get("x", 0.5), pos.get("y", 0.0),
                table_top + radius, radius, tuple(color)
            ))
            mid += 1

        self._marker_pub.publish(ma)

    @staticmethod
    def _cube_marker(now, mid, ns, x, y, z, sx, sy, sz, color):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = now
        m.ns = ns; m.id = mid
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position = Point(x=x, y=y, z=z)
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=sx, y=sy, z=sz)
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    @staticmethod
    def _sphere_marker(now, mid, ns, x, y, z, r, color):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = now
        m.ns = ns; m.id = mid
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position = Point(x=x, y=y, z=z)
        m.pose.orientation.w = 1.0
        d = r * 2.0
        m.scale = Vector3(x=d, y=d, z=d)
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m
