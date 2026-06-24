#!/usr/bin/env python3
"""Scene Manager — 管理 PlanningScene（桌子、料框、货物碰撞体）。

负责：
- 从 YAML 加载场景布局
- 向 MoveIt 注册/更新碰撞对象
- 对接感知接口动态更新物体位姿
"""

from __future__ import annotations

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


class SceneManager(Node):
    """场景管理器 — PlanningScene 碰撞对象的统一入口。

    用法:
        manager = SceneManager()
        manager.load_scene("config/scene_params.yaml")
        manager.register_table()       # 添加桌子碰撞体
        manager.register_cargo()       # 添加货物碰撞体
        manager.update_object("cargo_box", new_pose)
    """

    def __init__(self, node_name: str = "scene_manager"):
        super().__init__(node_name)

        self._apply_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        self._config: dict = {}
        self._registered_objects: dict[str, CollisionObject] = {}

    # ── 配置加载 ──────────────────────────────────────────

    def load_scene(self, yaml_path: str) -> dict:
        """从 YAML 文件加载场景参数。"""
        try:
            share_dir = get_package_share_directory("jaka_dual_arm")
            full_path = f"{share_dir}/{yaml_path}"
            with open(full_path) as f:
                self._config = yaml.safe_load(f)
            self.get_logger().info(f"Loaded scene config: {yaml_path}")
        except (FileNotFoundError, KeyError) as e:
            self.get_logger().warn(f"Could not load {yaml_path}: {e}")
            self._config = self._default_scene()
        return self._config

    def set_scene_dict(self, cfg: dict):
        """直接从字典设置场景参数（当 YAML 由外部加载时）。"""
        self._config = cfg

    @staticmethod
    def _default_scene() -> dict:
        """默认场景（当配置文件不可用时）。"""
        return {
            "world_frame": "world",
            "cargo": {
                "size": {"x": 0.18, "y": 0.345, "z": 0.12},
                "initial_pose": {"x": 0.36, "y": 0.02, "z": 0.06},
            },
            "table": {
                "center": {"x": 0.90, "y": 0.0},
                "top_z": 0.30,
                "size": {"x": 0.75, "y": 0.70, "z": 0.04},
                "leg_size": 0.04,
            },
        }

    # ── 对象注册 ──────────────────────────────────────────

    def register_table(self) -> bool:
        """将工作台添加到 PlanningScene。"""
        if "table" not in self._config:
            return False

        cfg = self._config["table"]
        frame = self._config.get("world_frame", "world")
        table_top_z = cfg["top_z"]
        sx, sy, sz = cfg["size"]["x"], cfg["size"]["y"], cfg["size"]["z"]
        cx, cy = cfg["center"]["x"], cfg["center"]["y"]
        table_center_z = table_top_z - sz / 2.0
        leg_s = cfg.get("leg_size", 0.04)
        leg_h = table_top_z - sz

        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.id = "work_table"
        obj.operation = CollisionObject.ADD

        def _add_box(dims, x, y, z):
            p = SolidPrimitive()
            p.type = SolidPrimitive.BOX
            p.dimensions = list(dims)
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            obj.primitives.append(p)
            obj.primitive_poses.append(pose)

        _add_box([sx, sy, sz], cx, cy, table_center_z)
        leg_x_off = sx / 2.0 - leg_s
        leg_y_off = sy / 2.0 - leg_s
        leg_z = leg_h / 2.0
        for xo in (-leg_x_off, leg_x_off):
            for yo in (-leg_y_off, leg_y_off):
                _add_box([leg_s, leg_s, leg_h], cx + xo, cy + yo, leg_z)

        return self._apply_object(obj)

    def register_cargo(self) -> bool:
        """将货物添加到 PlanningScene（初始位姿）。"""
        cfg = self._config.get("cargo", {})
        obj = self._make_box_object(
            "cargo_box",
            cfg.get("size", {"x": 0.18, "y": 0.345, "z": 0.12}),
            cfg.get("initial_pose", {"x": 0.36, "y": 0.02, "z": 0.06}),
        )
        return self._apply_object(obj)

    def register_bin(self) -> bool:
        """添加料框（中空结构：5 个薄壁）。"""
        cfg = self._config.get("bin", {})
        if not cfg.get("enabled", False):
            return True

        frame = self._config.get("world_frame", "world")
        cx, cy, cz = cfg["center"]["x"], cfg["center"]["y"], cfg["center"]["z"]
        sx, sy, sz = cfg["size"]["x"], cfg["size"]["y"], cfg["size"]["z"]
        t = cfg.get("wall_thickness", 0.005)

        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.id = "bin"
        obj.operation = CollisionObject.ADD

        walls = [
            ([sx, t, sz], 0,  sy / 2, 0),
            ([sx, t, sz], 0, -sy / 2, 0),
            ([t, sy, sz],  sx / 2, 0, 0),
            ([t, sy, sz], -sx / 2, 0, 0),
            ([sx, sy, t], 0, 0, -sz / 2),
        ]
        for dims, dx, dy, dz in walls:
            p = SolidPrimitive()
            p.type = SolidPrimitive.BOX
            p.dimensions = list(dims)
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x = cx + dx
            pose.position.y = cy + dy
            pose.position.z = cz + dz
            obj.primitives.append(p)
            obj.primitive_poses.append(pose)

        return self._apply_object(obj)

    def update_object_pose(self, object_id: str, position: Point, orientation=None):
        """动态更新物体位姿（从感知模块调用）。"""
        obj = CollisionObject()
        obj.header.frame_id = self._config.get("world_frame", "world")
        obj.id = object_id
        obj.operation = CollisionObject.MOVE
        pose = Pose()
        pose.position = position
        if orientation is not None:
            pose.orientation = orientation
        else:
            pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)
        # 用一个 dummy primitive 来满足 MOVE 操作的要求
        p = SolidPrimitive()
        p.type = SolidPrimitive.BOX
        p.dimensions = [0.01, 0.01, 0.01]
        obj.primitives.append(p)
        self._apply_object(obj)

    def remove_object(self, object_id: str):
        """从 PlanningScene 移除对象。"""
        obj = CollisionObject()
        obj.header.frame_id = self._config.get("world_frame", "world")
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        self._apply_object(obj)

    # ── 内部方法 ──────────────────────────────────────────

    def _make_box_object(self, oid: str, size: dict, pose_dict: dict) -> CollisionObject:
        """创建一个 SolidPrimitive.BOX 碰撞对象。"""
        obj = CollisionObject()
        obj.header.frame_id = self._config.get("world_frame", "world")
        obj.id = oid
        obj.operation = CollisionObject.ADD

        p = SolidPrimitive()
        p.type = SolidPrimitive.BOX
        p.dimensions = [size["x"], size["y"], size["z"]]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = pose_dict["x"]
        pose.position.y = pose_dict["y"]
        pose.position.z = pose_dict["z"]
        obj.primitives.append(p)
        obj.primitive_poses.append(pose)
        return obj

    def _apply_object(self, obj: CollisionObject) -> bool:
        """发送碰撞对象到 MoveIt。"""
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(f"Failed to apply object '{obj.id}'.")
            return False
        self._registered_objects[obj.id] = obj
        self.get_logger().info(f"Applied collision object: {obj.id}")
        return True
