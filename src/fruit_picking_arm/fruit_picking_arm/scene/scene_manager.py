#!/usr/bin/env python3
"""Scene Manager — PlanningScene collision object management.

Manages table, bin, and object collision bodies for the pick-and-place task.
All geometry parameters come from YAML config.

Supports objects of type: sphere, box, cylinder.
Ready for arbitrary object lists from the explicit real/simulation scene profile.

Reference:
  - jaka_dual_arm/scene/scene_manager.py — dual-arm version
"""

from __future__ import annotations

import math
import re

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool

from fruit_picking_arm.scene.bin_geometry import (
    bin_wall_boxes,
    normalized_bin_config,
)


class SceneManager(Node):
    """PlanningScene collision object manager.

    Usage:
        mgr = SceneManager()
        mgr.set_scene_dict(scene_cfg)
        mgr.register_table()
        mgr.register_bin()
        mgr.register_all_objects()
        mgr.remove_object("fruit_apple_1")
    """

    def __init__(self, node_name: str = "scene_manager"):
        super().__init__(node_name)

        self._apply_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._get_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self._sim_grasp_client = self.create_client(
            SetBool, "/simulation/fruit_gripper/set_attached"
        )

        self._config: dict = {}
        self._world_frame: str = "world"
        self._detected_objects: dict[str, tuple[str, dict]] = {}

    def set_scene_dict(self, cfg: dict):
        """Set scene configuration from pre-loaded YAML dict."""
        self._config = cfg
        self._world_frame = cfg.get("world_frame", "world")
        self.get_logger().info(
            f"Scene configured: {len(cfg.get('objects', []))} objects, "
            f"frame={self._world_frame}"
        )

    # ── Table ────────────────────────────────────────────────

    def register_table(self) -> bool:
        """Register table with 4 legs as collision objects."""
        table = self._config.get("table", {})
        if not table:
            return False

        tc = table["center"]
        ts = table["size"]
        top_z = table["top_z"]
        leg_s = table.get("leg_size", 0.04)
        leg_h = table.get("leg_height", top_z - ts["z"])

        obj = CollisionObject()
        obj.header.frame_id = self._world_frame
        obj.id = "work_table"
        obj.operation = CollisionObject.ADD

        def _box(dims, x, y, z):
            p = SolidPrimitive()
            p.type = SolidPrimitive.BOX
            p.dimensions = list(dims)
            ps = Pose()
            ps.orientation.w = 1.0
            ps.position.x = x
            ps.position.y = y
            ps.position.z = z
            obj.primitives.append(p)
            obj.primitive_poses.append(ps)

        # Table top
        table_cz = top_z - ts["z"] / 2.0
        _box([ts["x"], ts["y"], ts["z"]], tc["x"], tc["y"], table_cz)

        # 4 legs
        lx = ts["x"] / 2.0 - leg_s
        ly = ts["y"] / 2.0 - leg_s
        leg_cz = leg_h / 2.0
        for sx in (-lx, lx):
            for sy in (-ly, ly):
                _box([leg_s, leg_s, leg_h], tc["x"] + sx, tc["y"] + sy, leg_cz)

        return self._apply_object(obj)

    def register_perceived_table(self, surface: dict) -> bool:
        """Register a fresh RGB-D tabletop while carving out the fixed base.

        The table estimator publishes a conservative axis-aligned visible
        rectangle.  The robot base physically passes through that rectangle,
        so it is split around the already validated CAD-base footprint.
        """
        try:
            center_x = float(surface["center_x"])
            center_y = float(surface["center_y"])
            center_z = float(surface["center_z"])
            size_x = float(surface["size_x"])
            size_y = float(surface["size_y"])
            size_z = float(surface["size_z"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().error("Perceived table geometry is incomplete")
            return False
        if (
            not all(math.isfinite(v) for v in (
                center_x, center_y, center_z, size_x, size_y, size_z
            ))
            or min(size_x, size_y, size_z) <= 0.0
        ):
            self.get_logger().error("Perceived table geometry is invalid")
            return False

        rectangles = self._subtract_rectangle(
            center_x,
            center_y,
            size_x,
            size_y,
            (-0.220, 0.220, -0.220, 0.220),
        )
        obj = CollisionObject()
        obj.header.frame_id = self._world_frame
        obj.id = "work_table"
        obj.operation = CollisionObject.ADD
        for rect_x, rect_y, rect_size_x, rect_size_y in rectangles:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [rect_size_x, rect_size_y, size_z]
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x = rect_x
            pose.position.y = rect_y
            pose.position.z = center_z
            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
        return self._apply_object(obj)

    @staticmethod
    def _subtract_rectangle(center_x, center_y, size_x, size_y, cutout):
        table_min_x = center_x - size_x * 0.5
        table_max_x = center_x + size_x * 0.5
        table_min_y = center_y - size_y * 0.5
        table_max_y = center_y + size_y * 0.5
        cut_min_x = max(table_min_x, float(cutout[0]))
        cut_max_x = min(table_max_x, float(cutout[1]))
        cut_min_y = max(table_min_y, float(cutout[2]))
        cut_max_y = min(table_max_y, float(cutout[3]))
        if cut_min_x >= cut_max_x or cut_min_y >= cut_max_y:
            return [(center_x, center_y, size_x, size_y)]

        rectangles = []
        for min_x, max_x, min_y, max_y in (
            (table_min_x, cut_min_x, table_min_y, table_max_y),
            (cut_max_x, table_max_x, table_min_y, table_max_y),
            (cut_min_x, cut_max_x, table_min_y, cut_min_y),
            (cut_min_x, cut_max_x, cut_max_y, table_max_y),
        ):
            if max_x - min_x > 0.001 and max_y - min_y > 0.001:
                rectangles.append((
                    (min_x + max_x) * 0.5,
                    (min_y + max_y) * 0.5,
                    max_x - min_x,
                    max_y - min_y,
                ))
        return rectangles

    # ── Bin ──────────────────────────────────────────────────

    def register_bin(self) -> bool:
        """注册料框碰撞体。

        支持两种配置：
          - 新式 `bins: {healthy:{...}, unhealthy:{...}}` → 注册多个料框
          - 旧式单 `bin: {...}` → 注册单个料框
        """
        bins_cfg = self._config.get("bins")
        if bins_cfg:
            ok = True
            for kind, cfg in bins_cfg.items():
                ok = self._register_one_bin(f"bin_{kind}", cfg) and ok
            return ok
        bin_cfg = self._config.get("bin", {})
        if not bin_cfg:
            self.get_logger().error("No sorting bins are configured")
            return False
        return self._register_one_bin("bin", bin_cfg)

    def _register_one_bin(self, bin_id: str, bin_cfg: dict) -> bool:
        """把单个料框注册为 5 面薄墙碰撞体。

        ``size`` is the measured outer envelope, not the distance between wall
        centre lines.  Invalid or impossible geometry fails closed.
        """
        try:
            walls = bin_wall_boxes(bin_cfg)
        except ValueError as exc:
            self.get_logger().error(f"Invalid sorting bin '{bin_id}': {exc}")
            return False

        obj = CollisionObject()
        obj.header.frame_id = self._world_frame
        obj.id = bin_id
        obj.operation = CollisionObject.ADD

        for wall in walls:
            p = SolidPrimitive()
            p.type = SolidPrimitive.BOX
            p.dimensions = list(wall.size)
            ps = Pose()
            ps.orientation.w = 1.0
            ps.position.x, ps.position.y, ps.position.z = wall.center
            obj.primitives.append(p)
            obj.primitive_poses.append(ps)

        return self._apply_object(obj)

    # ── Objects ──────────────────────────────────────────────

    def register_object(self, obj_cfg: dict) -> bool:
        """Register a single object from YAML config.

        Supports shapes: sphere, box, cylinder.
        """
        shape = obj_cfg.get("shape", "sphere")
        pos = obj_cfg.get("position", {"x": 0.0, "y": 0.0})
        oid = obj_cfg.get("id", "unknown_object")

        table_top = self._config.get("table", {}).get("top_z", 0.30)

        obj = CollisionObject()
        obj.header.frame_id = self._world_frame
        obj.id = oid
        obj.operation = CollisionObject.ADD

        p = SolidPrimitive()

        if shape == "sphere":
            p.type = SolidPrimitive.SPHERE
            radius = obj_cfg.get("radius", 0.03)
            p.dimensions = [radius]
            z = table_top + radius

        elif shape == "box":
            p.type = SolidPrimitive.BOX
            sz = obj_cfg.get("size", {"x": 0.05, "y": 0.05, "z": 0.05})
            p.dimensions = [sz["x"], sz["y"], sz["z"]]
            z = table_top + sz["z"] / 2.0

        elif shape == "cylinder":
            p.type = SolidPrimitive.CYLINDER
            radius = obj_cfg.get("radius", 0.03)
            height = obj_cfg.get("height", 0.08)
            p.dimensions = [height, radius]
            z = table_top + height / 2.0

        else:
            self.get_logger().error(f"Unknown shape '{shape}' for {oid}")
            return False

        ps = Pose()
        ps.orientation.w = 1.0
        ps.position.x = pos["x"]
        ps.position.y = pos["y"]
        ps.position.z = float(pos.get("z", z))

        obj.primitives.append(p)
        obj.primitive_poses.append(ps)

        return self._apply_object(obj)

    def register_all_objects(self) -> int:
        """Register all objects from scene config. Returns count of registered objects."""
        objects = self._config.get("objects", [])
        count = 0
        for obj_cfg in objects:
            if self.register_object(obj_cfg):
                count += 1
        self.get_logger().info(f"Registered {count}/{len(objects)} objects in scene")
        return count

    def register_detected_objects(self, objects: list) -> bool:
        """Register perception results instead of YAML simulation fruit.

        Every detected fruit becomes a collision object in the configured
        control frame. Any malformed object or PlanningScene failure rejects
        the complete detection cycle before the arm is allowed to move.
        """
        previous = dict(self._detected_objects)
        incoming_source_ids = {
            str(getattr(detected, "id", f"object_{index:02d}"))
            for index, detected in enumerate(objects)
        }
        # Fruits that disappeared from the fresh snapshot must not remain as
        # ghost obstacles in MoveIt during the next one-target cycle.  An
        # interrupted pick may already have removed the world body (grasp IK)
        # or attached it to the gripper; MoveIt rejects a REMOVE of an absent
        # object, so cleanup is best-effort — the ADD below still validates
        # the service before the arm is allowed to move.
        for source_id, (collision_id, _config) in previous.items():
            if source_id not in incoming_source_ids:
                self._remove_detected_object_tolerant(source_id, collision_id)

        # move_group persists its scene across runner restarts; fetch the live
        # scene once to (a) skip targets already attached to the gripper and
        # (b) purge leftover bodies from previous sessions below.
        inventory = self._fetch_scene_inventory()
        attached_names = inventory[1] if inventory is not None else set()

        registered: dict[str, tuple[str, dict]] = {}
        skipped_attached = 0
        for index, detected in enumerate(objects):
            source_id = str(getattr(detected, "id", f"object_{index:02d}"))
            safe_id = re.sub(r"[^A-Za-z0-9_.-]", "_", source_id)
            collision_id = f"perceived_{safe_id}"
            try:
                x, y, z = (float(value) for value in detected.centroid)
                radius = float(detected.radius)
            except (AttributeError, TypeError, ValueError):
                self.get_logger().error(
                    f"Malformed perceived object '{source_id}'"
                )
                return False
            if (
                not all(math.isfinite(value) for value in (x, y, z, radius))
                or radius <= 0.0
            ):
                self.get_logger().error(
                    f"Invalid perceived geometry for '{source_id}'"
                )
                return False
            config = {
                "id": collision_id,
                "shape": "sphere",
                "position": {"x": x, "y": y, "z": z},
                "radius": radius,
            }
            if collision_id in attached_names:
                self.get_logger().info(
                    f"Skipping world registration of '{source_id}': "
                    "already attached to the gripper"
                )
                skipped_attached += 1
                continue
            if not self.register_object(config):
                return False
            registered[source_id] = (collision_id, config)
        self._detected_objects = registered
        # Leftover perceived bodies from a previous runner session must not
        # outlive the snapshot: they can overlap a freshly attached fruit and
        # force MoveIt to jiggle the plan start (impossible on the real board).
        self.purge_unknown_perceived_objects(incoming_source_ids)
        return len(registered) == len(objects) - skipped_attached

    def remove_detected_object(self, detected) -> tuple[str, dict] | None:
        """Remove the exact perceived target collision object for grasping."""
        source_id = str(getattr(detected, "id", ""))
        entry = self._detected_objects.get(source_id)
        if entry is None:
            self.get_logger().error(
                f"No registered collision object for perceived target '{source_id}'"
            )
            return None
        if not self.remove_object(entry[0]):
            return None
        return entry

    def attach_detected_object(self, detected, link_name: str = "gripper_tcp") -> bool:
        """Attach the selected perceived sphere after the close command.

        This is a planning model, not proof of physical grasp.  It makes the
        carried fruit participate in subsequent lift/place collision checks.
        """
        source_id = str(getattr(detected, "id", ""))
        entry = self._detected_objects.get(source_id)
        if entry is None:
            self.get_logger().error(
                f"Cannot attach unregistered perceived target '{source_id}'"
            )
            return False
        collision_id, config = entry
        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.touch_links = [
            "gripper_base", "gripper_center_slider",
            "gripper_drive_neg_x", "gripper_drive_neg_y",
            "gripper_drive_pos_x", "gripper_drive_pos_y",
            "gripper_finger_neg_x", "gripper_finger_neg_y",
            "gripper_finger_pos_x", "gripper_finger_pos_y",
        ]
        attached.object.header.frame_id = link_name
        attached.object.id = collision_id
        attached.object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [float(config["radius"])]
        pose = Pose()
        pose.orientation.w = 1.0
        attached.object.primitives.append(primitive)
        attached.object.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        # PlanGrasp already removed this exact world collision body before
        # solving contact IK. Match the field-validated RViz bridge here:
        # attach the carried geometry only. Sending a second REMOVE for the
        # now-absent world object makes MoveIt reject the whole scene diff.
        return self._apply_scene(scene, f"attach '{collision_id}'")

    def detach_detected_object(self, detected) -> bool:
        """Remove the carried planning sphere after physical gripper opening.

        This mirrors the validated RViz bridge: detaching may cause MoveIt to
        return the sphere to the world, which would block the immediate
        retract. Remove that released sphere in the same scene transaction.
        """
        source_id = str(getattr(detected, "id", ""))
        entry = self._detected_objects.get(source_id)
        if entry is None:
            self.get_logger().error(
                f"Cannot detach unregistered perceived target '{source_id}'"
            )
            return False
        attached = AttachedCollisionObject()
        attached.link_name = "gripper_tcp"
        attached.object.id = entry[0]
        attached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        remove = CollisionObject()
        remove.header.frame_id = self._world_frame
        remove.id = entry[0]
        remove.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(remove)
        ok = self._apply_scene(scene, f"detach and remove '{entry[0]}'")
        if ok:
            self._detected_objects.pop(source_id, None)
        return ok

    def clear_detected_objects(self) -> bool:
        """Remove every fruit-only collision body from the current snapshot.

        The fixed perceived table and surveyed bins are intentionally left in
        place.  This is the PlanningScene half of a light vision reset; the
        perception service independently clears its temporal trackers.
        """
        entries = list(self._detected_objects.values())
        if not entries:
            return True

        # Bodies are removed one at a time: MoveIt rejects a diff that
        # removes an already-absent object (an interrupted pick may have
        # removed the target for grasp IK or attached it to the gripper),
        # and one absent body must not leave the rest of the snapshot behind.
        for collision_id, _config in entries:
            self._remove_detected_object_tolerant(collision_id, collision_id)
        self._detected_objects.clear()
        self.get_logger().info(
            "LIGHT_VISION_SCENE_CLEARED: removed %d perceived fruit collision object(s); "
            "table and bins preserved" % len(entries)
        )
        return True

    def update_object_pose(self, object_id: str, position: Point,
                           orientation: Quaternion = None):
        """Update object pose (from perception)."""
        obj = CollisionObject()
        obj.header.frame_id = self._world_frame
        obj.id = object_id
        obj.operation = CollisionObject.MOVE
        ps = Pose()
        ps.position = position
        ps.orientation = orientation if orientation else Quaternion(w=1.0)
        obj.primitive_poses.append(ps)
        p = SolidPrimitive()
        p.type = SolidPrimitive.BOX
        p.dimensions = [0.01, 0.01, 0.01]
        obj.primitives.append(p)
        self._apply_object(obj)

    def remove_object(self, object_id: str) -> bool:
        """Remove object from PlanningScene."""
        obj = CollisionObject()
        obj.header.frame_id = self._world_frame
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        return self._apply_object(obj)

    def _remove_detected_object_tolerant(
        self, source_id: str, collision_id: str
    ) -> bool:
        """Best-effort removal of one perceived-fruit world collision body.

        MoveIt rejects a scene diff that removes an object that is not in the
        scene, and an interrupted pick may already have removed the target
        for grasp IK (or attached it to the gripper, where it belongs while
        the physical fruit is still held).  Absence is the desired end state
        for cleanup, so a rejected REMOVE is treated as already-clean; the
        registration ADD that follows still validates the PlanningScene
        service before the arm is allowed to move.
        """
        if self.remove_object(collision_id):
            return True
        self.get_logger().warning(
            f"Perceived object '{source_id}' is not in the world scene "
            "(already removed or attached); treating cleanup as complete"
        )
        return True

    def remove_nearest_object(
        self, x: float, y: float, max_distance: float = 0.10
    ) -> tuple[str, dict] | None:
        """Remove the configured fruit nearest a perceived grasp target.

        Grasping intentionally brings the fingers into contact with the
        selected fruit.  Keeping that fruit as a world collision object makes
        collision-aware IK reject the grasp.  Only the matched fruit is
        removed; table, bins, and all other fruit stay collision-active.
        """
        nearest = None
        nearest_distance = float("inf")
        for obj_cfg in self._config.get("objects", []):
            position = obj_cfg.get("position", {})
            distance = math.hypot(
                float(position.get("x", 0.0)) - x,
                float(position.get("y", 0.0)) - y,
            )
            if distance < nearest_distance:
                nearest = obj_cfg
                nearest_distance = distance

        if nearest is None or nearest_distance > max_distance:
            self.get_logger().warn(
                f"No scene fruit matches perceived target ({x:.3f}, {y:.3f})"
            )
            return None

        object_id = str(nearest.get("id", ""))
        if not object_id or not self.remove_object(object_id):
            return None
        self.get_logger().info(
            f"Removed grasp target collision '{object_id}' "
            f"(match distance={nearest_distance:.3f}m)"
        )
        return object_id, nearest

    def set_simulated_grasp(self, attached: bool) -> bool:
        """Attach/detach the nearby fruit through Gazebo's fixed constraint."""
        if not self._sim_grasp_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                "Physical Gazebo grasp service is not available"
            )
            return False
        request = SetBool.Request()
        request.data = attached
        future = self._sim_grasp_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=4.0)
        result = future.result()
        if result is None or not result.success:
            message = result.message if result is not None else "service timeout"
            self.get_logger().error(f"Physical Gazebo grasp failed: {message}")
            return False
        action = "attached" if attached else "detached"
        self.get_logger().info(
            f"Physical Gazebo fruit {action}: {result.message}"
        )
        return True

    # ── Utilities ────────────────────────────────────────────

    def has_objects(self) -> bool:
        return len(self._config.get("objects", [])) > 0

    def get_objects(self) -> list[dict]:
        return list(self._config.get("objects", []))

    def get_table_top_z(self) -> float:
        return self._config.get("table", {}).get("top_z", 0.30)

    def get_bin_center(self) -> dict:
        return self._config.get("bin", {}).get("center", {"x": 0.55, "y": 0.45})

    def get_bin_top_z(self) -> float:
        bin_cfg = self._config.get("bin", {})
        if not bin_cfg:
            return 0.30
        return normalized_bin_config(bin_cfg)["top_z"]

    def get_bin_size(self) -> dict:
        return self._config.get("bin", {}).get("size", {"x": 0.20, "y": 0.20, "z": 0.15})

    # ── Internal ─────────────────────────────────────────────

    def _apply_object(self, obj: CollisionObject) -> bool:
        if not self._apply_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                f"PlanningScene service unavailable for object '{obj.id}'"
            )
            return False
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        return self._apply_scene(scene, f"object '{obj.id}'")

    def _apply_scene(self, scene: PlanningScene, description: str) -> bool:
        """Apply one validated planning-scene diff synchronously."""
        if not self._apply_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                f"PlanningScene service unavailable for {description}"
            )
            return False

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(f"Failed to apply {description}")
            return False
        self.get_logger().debug(f"Applied: {description}")
        return True

    def _fetch_scene_inventory(self):
        """Return (world_object_ids, attached_object_ids) from the live scene.

        Returns None when the PlanningScene service is unavailable.  The
        request only asks for world object names and attached object bodies,
        keeping the round trip small.
        """
        if not self._get_scene_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("GetPlanningScene service unavailable")
            return None

        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        future = self._get_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        if response is None:
            self.get_logger().error("GetPlanningScene returned no scene")
            return None

        world = {str(obj.id) for obj in response.scene.world.collision_objects}
        attached = {
            str(obj.object.id)
            for obj in response.scene.robot_state.attached_collision_objects
        }
        return world, attached

    def purge_unknown_perceived_objects(self, keep_source_ids) -> bool:
        """Remove leftover perceived fruit bodies this process did not register.

        move_group persists its PlanningScene across runner restarts, so a
        fresh runner starts with empty bookkeeping while the scene may still
        hold ``perceived_fruit_track_*`` bodies from a previous session.  A
        stale body sitting where the grasped fruit now is makes MoveIt detect
        a start-state collision and jiggle the plan start — a trajectory the
        real board cannot execute.  Remove every perceived world body not in
        keep_source_ids; attached bodies are not world objects and stay.
        """
        inventory = self._fetch_scene_inventory()
        if inventory is None:
            return False
        world, _attached = inventory

        keep_ids = set()
        for source_id in keep_source_ids or ():
            safe_id = re.sub(r"[^A-Za-z0-9_.-]", "_", str(source_id))
            keep_ids.add(f"perceived_{safe_id}")

        purged = []
        for name in sorted(world):
            if name.startswith("perceived_") and name not in keep_ids:
                if self._remove_detected_object_tolerant(name, name):
                    purged.append(name)
        if purged:
            self.get_logger().warning(
                "PURGED_STALE_SCENE_OBJECTS: removed %d leftover perceived "
                "body(s): %s", len(purged), ", ".join(purged)
            )
        return True

    def detach_all_perceived_attached(self) -> bool:
        """Remove every attached perceived fruit body from the PlanningScene.

        A recovery that ends with a fruit still in the gripper leaves its
        attached body behind; on the next pick the body is a ghost obstacle
        that no longer matches any tracked fruit.  Bookkeeping and physical
        gripper state are not touched — this is scene hygiene only, and an
        absent body (already detached) is not an error.
        """
        inventory = self._fetch_scene_inventory()
        if inventory is None:
            return False
        _world, attached = inventory

        removed = 0
        for name in sorted(attached):
            if not name.startswith("perceived_"):
                continue
            attached_obj = AttachedCollisionObject()
            attached_obj.link_name = "gripper_tcp"
            attached_obj.object.id = name
            attached_obj.object.operation = CollisionObject.REMOVE
            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects.append(attached_obj)
            if self._apply_scene(scene, f"detach leftover '{name}'"):
                removed += 1
        if removed:
            self.get_logger().warning(
                "SCENE_HYGIENE: removed %d leftover attached fruit body(s)",
                removed,
            )
        return True
