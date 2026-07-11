#!/usr/bin/env python3
"""Scene Manager — PlanningScene collision object management.

Manages table, bin, and object collision bodies for the pick-and-place task.
All geometry parameters come from YAML config.

Supports objects of type: sphere, box, cylinder.
Ready for arbitrary object lists from scene_params.yaml.

Reference:
  - jaka_dual_arm/scene/scene_manager.py — dual-arm version
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


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

        self._config: dict = {}
        self._world_frame: str = "world"

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
            return True
        return self._register_one_bin("bin", bin_cfg)

    def _register_one_bin(self, bin_id: str, bin_cfg: dict) -> bool:
        """把单个料框注册为 5 面薄墙碰撞体。"""
        bc = bin_cfg["center"]
        bs = bin_cfg["size"]
        top_z = bin_cfg["top_z"]
        t = bin_cfg.get("wall_thickness", 0.01)
        hx, hy = bs["x"] / 2.0, bs["y"] / 2.0
        wz = top_z - bs["z"] / 2.0

        obj = CollisionObject()
        obj.header.frame_id = self._world_frame
        obj.id = bin_id
        obj.operation = CollisionObject.ADD

        walls = [
            ([bs["x"], bs["y"], t], bc["x"], bc["y"], top_z - bs["z"]),  # bottom
            ([t, bs["y"], bs["z"]], bc["x"] - hx, bc["y"], wz),           # -x wall
            ([t, bs["y"], bs["z"]], bc["x"] + hx, bc["y"], wz),           # +x wall
            ([bs["x"], t, bs["z"]], bc["x"], bc["y"] - hy, wz),           # -y wall
            ([bs["x"], t, bs["z"]], bc["x"], bc["y"] + hy, wz),           # +y wall
        ]
        for dims, wx, wy, w_zz in walls:
            p = SolidPrimitive()
            p.type = SolidPrimitive.BOX
            p.dimensions = list(dims)
            ps = Pose()
            ps.orientation.w = 1.0
            ps.position.x = wx
            ps.position.y = wy
            ps.position.z = w_zz
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
            self.get_logger().warn(f"Unknown shape '{shape}' for {oid}, defaulting to sphere")
            p.type = SolidPrimitive.SPHERE
            p.dimensions = [0.03]
            z = table_top + 0.03

        ps = Pose()
        ps.orientation.w = 1.0
        ps.position.x = pos["x"]
        ps.position.y = pos["y"]
        ps.position.z = z

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
        return self._config.get("bin", {}).get("top_z", 0.30)

    def get_bin_size(self) -> dict:
        return self._config.get("bin", {}).get("size", {"x": 0.20, "y": 0.20, "z": 0.15})

    # ── Internal ─────────────────────────────────────────────

    def _apply_object(self, obj: CollisionObject) -> bool:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(f"Failed to apply object '{obj.id}'")
            return False
        self.get_logger().debug(f"Applied: {obj.id}")
        return True
