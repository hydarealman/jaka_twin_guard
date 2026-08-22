#!/usr/bin/env python3
"""Pick-and-Place Task Runner — orchestrates the full pipeline.

Creates all layer components, fills the blackboard, loads the BT tree,
and runs the main execution loop.

Reference:
  - jaka_dual_arm/behavior/bt_runner.py — carry task runner
  - jaka_dual_arm/massage/massage_runner.py — massage runner node
"""

from __future__ import annotations

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
from jaka_single_arm.perception.yolo_depth_localizer import YoloDepthLocalizer
from jaka_single_arm.perception.health_fusion import HealthFusion
from jaka_single_arm.perception.real_mode import validate_real_perception_config
from jaka_single_arm.scene.scene_manager import SceneManager
from jaka_single_arm.planner.planner_server import SingleArmPlannerServer
from jaka_single_arm.behavior.bt_engine import BtEngine
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
        self.declare_parameter("real_mode", False)
        self.declare_parameter("data_timeout_s", 1.0)
        real_mode = bool(self.get_parameter("real_mode").value)
        self.declare_parameter(
            "perception_output_frame", perception_cfg.get("output_frame", "world")
        )
        self.declare_parameter(
            "force_table_center_z", perception_cfg.get("force_table_center_z", False)
        )
        self.declare_parameter(
            "enable_table_z_fallback",
            perception_cfg.get("enable_table_z_fallback", False),
        )
        classifier_cfg = perception_cfg.get("classifier", {})
        self.declare_parameter(
            "allow_scene_fallback", classifier_cfg.get("allow_scene_fallback", False)
        )
        self.declare_parameter(
            "camera_info_topic",
            classifier_cfg.get(
                "camera_info_topic", "/camera/camera/color/camera_info"
            ),
        )
        self.declare_parameter("perception_license_mode", "development")
        self.declare_parameter("model_license_approved", False)
        if real_mode:
            validate_real_perception_config(
                camera_type=override_type,
                allow_scene_fallback=bool(
                    self.get_parameter("allow_scene_fallback").value
                ),
                force_table_center_z=bool(
                    self.get_parameter("force_table_center_z").value
                ),
                enable_table_z_fallback=bool(
                    self.get_parameter("enable_table_z_fallback").value
                ),
            )
        if override_type != perception_cfg.get("camera_type"):
            self.get_logger().info(
                f"camera_type overridden by launch: "
                f"'{perception_cfg.get('camera_type')}' → '{override_type}'"
            )
        perception_cfg = dict(perception_cfg)
        perception_cfg["camera_type"] = override_type
        # One backend contract for every entry point: physical D455 uses
        # semantic RGB + registered depth; Mock/Gazebo keep geometry.
        backend_default = (
            "geometry" if override_type in ("mock", "gazebo") else "yolo_depth"
        )
        self.declare_parameter("localizer_backend", backend_default)
        localizer_backend = str(
            self.get_parameter("localizer_backend").value
        ).strip().lower()
        if override_type == "realsense" and localizer_backend != "yolo_depth":
            self.get_logger().warning(
                "RealSense production perception requires yolo_depth; "
                "overriding localizer_backend=%s" % localizer_backend
            )
            localizer_backend = "yolo_depth"
        elif override_type in ("mock", "gazebo"):
            localizer_backend = "geometry"
        perception_cfg["localizer_backend"] = localizer_backend
        detector_model = os.environ.get("JAKA_FRUIT_DETECTOR_MODEL", "").strip()
        if detector_model:
            fruit_detector_cfg = dict(perception_cfg.get("fruit_detector", {}))
            fruit_detector_cfg["model"] = detector_model
            perception_cfg["fruit_detector"] = fruit_detector_cfg
        perception_cfg["output_frame"] = self.get_parameter(
            "perception_output_frame"
        ).value
        perception_cfg["force_table_center_z"] = bool(
            self.get_parameter("force_table_center_z").value
        )
        perception_cfg["enable_table_z_fallback"] = bool(
            self.get_parameter("enable_table_z_fallback").value
        )
        self._real_mode = real_mode
        self._data_timeout_s = max(
            0.1, float(self.get_parameter("data_timeout_s").value)
        )
        self._perception_watchdog_enabled = False
        self._perception_fault = False
        if real_mode:
            # Scene fixtures may still describe the physical table/bin, but
            # simulated fruit entries are never allowed to seed a real run.
            scene_cfg = dict(scene_cfg)
            scene_cfg["objects"] = []
            self._scene_cfg = scene_cfg
            self._scene_mgr.set_scene_dict(scene_cfg)
        classifier_cfg = dict(classifier_cfg)
        classifier_cfg["allow_scene_fallback"] = bool(
            self.get_parameter("allow_scene_fallback").value
        )
        classifier_cfg["camera_info_topic"] = self.get_parameter(
            "camera_info_topic"
        ).value
        classifier_cfg["license_mode"] = self.get_parameter(
            "perception_license_mode"
        ).value
        classifier_cfg["model_license_approved"] = bool(
            self.get_parameter("model_license_approved").value
        )
        perception_cfg["classifier"] = classifier_cfg
        perception_cfg["data_timeout_s"] = self._data_timeout_s
        self._perception_cfg = perception_cfg
        self._camera = create_camera(self, perception_cfg, scene_cfg)
        self._camera.connect()
        if real_mode:
            self._perception_watchdog_timer = self.create_timer(
                0.2, self._perception_watchdog
            )
        self._object_detector = ObjectDetector(self, perception_cfg)
        self._rgbd_localizer = None
        if localizer_backend == "yolo_depth":
            self._rgbd_localizer = YoloDepthLocalizer(
                self,
                perception_cfg,
                camera_frame=str(perception_cfg.get("camera_frame", "")),
            )
        # The selected localizer supplies a tight fruit ROI; MobileNet only
        # assigns health inside that measured ROI.
        self._health_fusion = HealthFusion(self, perception_cfg, scene_cfg)
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
            "rgbd_localizer": self._rgbd_localizer,
            "health_fusion": self._health_fusion,
            "gripper_controller": self._gripper,
            "safety_monitor": self._safety,
            "scene_config": scene_cfg,
            "robot_config": robot_cfg,
            "gripper_config": gripper_cfg,
            "skill_config": skill_cfg,
            "behavior_config": behavior_cfg,
            "perception_config": perception_cfg,
            "home_pose": scene_cfg.get("home_pose"),
            "detected_objects": [],
            "detection_count": 0,
            "target_object": None,
            "current_trajectory": None,
            "simulation_mode": override_type in ("mock", "gazebo"),
            "simulation_grasp_attached": False,
        }

        # YAML targets are simulation fixtures and must never seed a real run.
        objects = scene_cfg.get("objects", [])
        if self._blackboard["simulation_mode"] and objects:
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
            self.get_logger().error("No objects detected — task failed")
            self._cleanup(executor)
            return False

        self.get_logger().info(
            f"Phase 2: Processing {len(detected)} detected object(s)..."
        )

        if not self._perception_is_fresh():
            self.get_logger().error(
                "Real camera data became stale before motion; refusing the task"
            )
            self._cleanup(executor)
            return False
        self._perception_watchdog_enabled = True

        self._engine.select_tree("PickPlaceTree")
        success_count = 0

        for obj_idx, obj in enumerate(detected):
            if not rclpy.ok():
                break

            # Never pick or silently route an object whose quality classifier
            # returned Unknown.  It must be re-observed or handled manually;
            # defaulting Unknown to the healthy bin would create a false sort.
            health = str(getattr(obj, "health", "unknown")).strip().lower()
            if health not in ("healthy", "unhealthy", "good", "bad", "0", "1"):
                self.get_logger().warning(
                    f"Skipping object {getattr(obj, 'id', obj_idx)}: quality is Unknown"
                )
                if not self._blackboard["simulation_mode"]:
                    self._cleanup(executor)
                    return False
                continue

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
                    if self._perception_fault:
                        self.get_logger().error(
                            "Real camera data lost during task; motion cancelled"
                        )
                        self._engine.halt()
                        self._cleanup(executor)
                        return False
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
                        if not self._blackboard["simulation_mode"]:
                            self._cleanup(executor)
                            return False
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
                if not self._blackboard["simulation_mode"]:
                    self._cleanup(executor)
                    return False

            # If a later BT step failed after the simulated grasp was made,
            # never carry that stale constraint into the next fruit.
            if self._blackboard.get("simulation_grasp_attached", False):
                self.get_logger().warning(
                    f"Object {oid}: releasing residual simulated grasp"
                )
                self._gripper.open()
                self._scene_mgr.set_simulated_grasp(False)
                self._blackboard["simulation_grasp_attached"] = False

        self.get_logger().info(
            f"=== Task Complete: {success_count}/{len(detected)} objects placed ==="
        )
        self._cleanup(executor)
        return success_count == len(detected)

    def _cleanup(self, executor: MultiThreadedExecutor):
        """Stop camera and remove nodes from executor."""
        self._perception_watchdog_enabled = False
        self._camera.disconnect()
        try:
            executor.remove_node(self._planner)
            executor.remove_node(self)
        except Exception:
            pass
        executor.shutdown(timeout_sec=1.0)

    def _perception_is_fresh(self) -> bool:
        if not self._real_mode:
            return True
        if self._perception_cfg.get("localizer_backend") == "yolo_depth":
            return (
                self._camera.get_rgb_image_age_s() <= self._data_timeout_s
                and self._camera.get_aligned_depth_image_age_s()
                <= self._data_timeout_s
            )
        return (
            self._camera.get_point_cloud_age_s() <= self._data_timeout_s
            and self._camera.get_rgb_image_age_s() <= self._data_timeout_s
        )

    def _perception_watchdog(self) -> None:
        if not self._real_mode or not self._perception_watchdog_enabled:
            return
        if not self._perception_is_fresh():
            if not self._perception_fault:
                self.get_logger().fatal(
                    "Real camera stream timed out; cancelling active motion"
                )
            self._perception_fault = True
            self._planner.cancel_active_goal()

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

        # Bins（好坏两个料框；兼容旧式单 bin）
        bins_cfg = scene.get("bins")
        bin_render = []
        if bins_cfg:
            bin_render = [
                (bins_cfg.get("healthy"), (0.25, 0.55, 0.30, 0.65)),   # 绿：好果框
                (bins_cfg.get("unhealthy"), (0.60, 0.30, 0.20, 0.65)),  # 棕：坏果框
            ]
        elif scene.get("bin"):
            bin_render = [(scene.get("bin"), (0.25, 0.40, 0.60, 0.65))]

        for bin_cfg, color in bin_render:
            if not bin_cfg:
                continue
            bc = bin_cfg["center"]
            bs = bin_cfg["size"]
            top_z = bin_cfg["top_z"]
            bcz = top_z - bs["z"] / 2.0
            ma.markers.append(self._cube_marker(
                now, mid, "bins", bc["x"], bc["y"], bcz,
                bs["x"], bs["y"], bs["z"], color
            ))
            mid += 1

        # Objects from YAML are visualization fixtures for simulation only.
        fixture_objects = (
            scene.get("objects", [])
            if self._blackboard.get("simulation_mode", False)
            else []
        )
        for obj in fixture_objects:
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
