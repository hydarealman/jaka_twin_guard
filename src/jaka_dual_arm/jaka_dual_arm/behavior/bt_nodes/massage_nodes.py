#!/usr/bin/env python3
"""按摩专用 BT 节点 — BehaviorTree.CPP 兼容.

5 个节点:
    WaitServices       — BtCondition: 等待 MoveIt + 控制器服务就绪
    SetupMassageScene  — BtActionNode: 注册床+人体碰撞对象
    InitSurfaceModel   — BtActionNode: 加载 YAML → 构建曲面模型 + 路径生成器
    RunMassageCycle    — BtActionNode: 核心 60 阶段迭代 (生成→规划→执行)
    RetreatToHome      — BtActionNode: 双臂回到安全初始位姿

节点间通过 blackboard 共享状态:
    blackboard["surface"]         — BackSurfaceModel
    blackboard["path_generator"]  — PathGenerator
    blackboard["planner"]         — DualArmPlannerServer
    blackboard["stages"]          — 60 阶段定义列表
    blackboard["body_config"]     — 人体模型 YAML
    blackboard["safety"]          — SafetyMonitor (可选)
    blackboard["impedance"]       — VirtualImpedanceController (可选)

参考:
  - behavior/bt_nodes/bt_node_base.py — BtNode/BtActionNode/BtCondition/BtAsyncNode
  - behavior/bt_nodes/carry_nodes.py  — 搬运 BT 节点 (参考模式)
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from jaka_dual_arm.behavior.bt_nodes.bt_node_base import (
    NodeStatus, BtActionNode, BtCondition, BtAsyncNode,
)
from jaka_dual_arm.skills.path_generator import (
    BackSurfaceModel, PathGenerator, TECHNIQUE_CONFIG,
)

# ── Executor-aware spin helper ──────────────────────────────────

def _spin_future(blackboard: dict, future, timeout_sec: float = 5.0):
    """Spin until future completes, using the MultiThreadedExecutor from blackboard.

    This ensures BOTH massage_runner AND planner nodes get their callbacks
    processed, which is essential when the future belongs to a client on the
    planner node (e.g., /apply_planning_scene, /plan_kinematic_path).
    """
    executor = blackboard.get("executor")
    if executor is not None:
        executor.spin_until_future_complete(future, timeout_sec)
    else:
        # Fallback: spin just the runner node (will miss planner callbacks!)
        node = blackboard.get("node")
        if node is not None:
            rclpy.spin_until_future_complete(node, future, timeout_sec)


# Joint names for left/right arms
LEFT_JOINTS = [
    "left_joint_1", "left_joint_2", "left_joint_3",
    "left_joint_4", "left_joint_5", "left_joint_6",
]
RIGHT_JOINTS = [
    "right_joint_1", "right_joint_2", "right_joint_3",
    "right_joint_4", "right_joint_5", "right_joint_6",
]
ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS

# Home joint positions (hover悬停, Link_06垂直向下)
HOME_LEFT = [0.0, 0.75, -1.10, 1.30, 1.57, 1.50]
HOME_RIGHT = [0.0, 0.75, -1.10, 1.30, 1.57, 1.50]


# ═══════════════════════════════════════════════════════════════
# WaitServices
# ═══════════════════════════════════════════════════════════════

class WaitServices(BtCondition):
    """等待 MoveIt 规划服务和控制器 Action 服务就绪。"""

    def __init__(self, name: str = "WaitServices"):
        super().__init__(name)
        self._timeout = 30.0
        self._start_time: Optional[float] = None
        self._all_ready_logged = False
        self._ctrl_clients = {}  # cached action clients for controller check

    def on_start(self):
        self._start_time = time.time()

    def evaluate(self) -> bool:
        node: Optional[Node] = self.blackboard.get("node")
        if node is None:
            return False

        elapsed = time.time() - self._start_time

        # Check MoveIt planning service
        planner = self.blackboard.get("planner")
        if planner is not None:
            if not planner._motion_plan_client.wait_for_service(timeout_sec=0.1):
                if elapsed > self._timeout:
                    node.get_logger().error(
                        f"Timeout waiting for /plan_kinematic_path ({self._timeout}s)"
                    )
                    return False
                return False

            # Also check apply_planning_scene
            if not planner._apply_scene_client.wait_for_service(timeout_sec=0.1):
                if elapsed > self._timeout:
                    node.get_logger().error("Timeout waiting for /apply_planning_scene")
                    return False
                return False

        # Check controller action servers (cache clients to avoid recreating each tick)
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        for arm in ["left", "right"]:
            controller = f"/{arm}_arm_controller/follow_joint_trajectory"
            if arm not in self._ctrl_clients:
                self._ctrl_clients[arm] = ActionClient(
                    node, FollowJointTrajectory, controller
                )
            if not self._ctrl_clients[arm].wait_for_server(timeout_sec=0.1):
                if elapsed > self._timeout:
                    node.get_logger().error(
                        f"Timeout waiting for {controller} ({self._timeout}s)"
                    )
                    return False
                return False

        if not self._all_ready_logged:
            node.get_logger().info("All services ready (MoveIt + controllers).")
            self._all_ready_logged = True
        return True


# ═══════════════════════════════════════════════════════════════
# SetupMassageScene
# ═══════════════════════════════════════════════════════════════

class SetupMassageScene(BtActionNode):
    """向 PlanningScene 注册床和人体碰撞对象。

    从 body_config 读取床体尺寸和人体段参数，构建 CollisionObject 并发布。
    """

    def __init__(self, name: str = "SetupMassageScene"):
        super().__init__(name)

    def execute(self) -> NodeStatus:
        node: Optional[Node] = self.blackboard.get("node")
        body_cfg: Optional[dict] = self.blackboard.get("body_config")
        if node is None or body_cfg is None:
            return NodeStatus.FAILURE

        try:
            self._setup_bed(node, body_cfg)  # now handles bed + body in one call
            node.get_logger().info("Massage scene registered.")
            return NodeStatus.SUCCESS
        except Exception as e:
            node.get_logger().error(f"Failed to setup scene: {e}")
            import traceback
            node.get_logger().error(traceback.format_exc())
            return NodeStatus.FAILURE

    def _setup_bed(self, node: Node, cfg: dict):
        """注册床框 + 床垫碰撞对象 + 发布彩色人体 Marker。"""
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene
        from shape_msgs.msg import SolidPrimitive

        objects = []
        bed = cfg.get("bed", {})

        # Bed frame
        frame = bed.get("frame", {})
        frame_size = frame.get("size", {"x": 1.20, "y": 0.66, "z": 0.08})
        frame_bottom = frame.get("bottom_z", 0.0)
        frame_z = frame_bottom + frame_size["z"] / 2.0
        obj = CollisionObject()
        obj.id = "bed_frame"
        obj.header.frame_id = "world"
        obj.operation = CollisionObject.ADD
        obj.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[frame_size["x"], frame_size["y"], frame_size["z"]],
        ))
        obj.primitive_poses.append(_make_pose_msg(
            bed["center"]["x"], bed["center"]["y"], frame_z))
        objects.append(obj)

        # Mattress
        mattress = bed.get("mattress", {})
        mat_size = mattress.get("size", {"x": 1.12, "y": 0.56, "z": 0.06})
        mat_bottom = mattress.get("bottom_z", 0.08)
        mat_top = mat_bottom + mat_size["z"]  # mattress top surface (z=0.14)
        mat_z = mat_bottom + mat_size["z"] / 2.0
        obj2 = CollisionObject()
        obj2.id = "mattress"
        obj2.header.frame_id = "world"
        obj2.operation = CollisionObject.ADD
        obj2.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[mat_size["x"], mat_size["y"], mat_size["z"]],
        ))
        obj2.primitive_poses.append(_make_pose_msg(
            bed["center"]["x"], bed["center"]["y"], mat_z))
        objects.append(obj2)

        # Body segments — placed ON mattress, not sunken in
        self._add_body_objects(cfg, objects, mat_top)

        # Apply all collision objects in ONE request
        self._apply_scene(node, objects)

        # Publish colored visual markers for human body (different from bed color)
        self._publish_body_markers(node, cfg, mat_top)

    def _add_body_objects(self, cfg: dict, objects: list, mat_top: float):
        """添加光滑人体碰撞模型：圆柱体躯干/四肢 + 球体头/手脚。

        所有碰撞体底部靠在床垫上 (z ≥ mat_top)，人体不会陷到床里。
        背部: Y轴圆柱体 (横向)，12 主段 + 11 插值 = 23 根光滑曲面
        头部: 缩小前移，不阻挡 C7 按摩区 (x≈0.38)
        四肢: 圆柱体沿肢体方向定向，手/脚为小球体
        """
        import math
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive

        # ── 方向向量 → 四元数 ──
        def _quat_z_to_dir(dx: float, dy: float, dz: float):
            length = math.sqrt(dx*dx + dy*dy + dz*dz)
            if length < 1e-10:
                return (0.0, 0.0, 0.0, 1.0)
            dx, dy, dz = dx/length, dy/length, dz/length
            cos_a = dz
            if cos_a > 0.9999:
                return (0.0, 0.0, 0.0, 1.0)
            if cos_a < -0.9999:
                return (1.0, 0.0, 0.0, 0.0)
            half_cos = math.sqrt((1.0 + cos_a) * 0.5)
            half_sin = math.sqrt((1.0 - cos_a) * 0.5)
            axis_len = math.sqrt(dx*dx + dy*dy)
            qx = -dy / axis_len * half_sin
            qy = dx / axis_len * half_sin
            qz = 0.0
            qw = half_cos
            return (qx, qy, qz, qw)

        def _add_box(oid, cx, cy, cz, sx, sy, sz):
            obj = CollisionObject()
            obj.id = oid; obj.header.frame_id = "world"
            obj.operation = CollisionObject.ADD
            obj.primitives.append(SolidPrimitive(
                type=SolidPrimitive.BOX, dimensions=[sx, sy, sz]))
            obj.primitive_poses.append(_make_pose_msg(cx, cy, cz))
            objects.append(obj)

        def _add_sphere(oid, cx, cy, cz, r):
            obj = CollisionObject()
            obj.id = oid; obj.header.frame_id = "world"
            obj.operation = CollisionObject.ADD
            obj.primitives.append(SolidPrimitive(
                type=SolidPrimitive.SPHERE, dimensions=[r]))
            obj.primitive_poses.append(_make_pose_msg(cx, cy, cz))
            objects.append(obj)

        def _add_cylinder(oid, cx, cy, cz, height, radius,
                          qx=0.0, qy=0.0, qz=0.0, qw=1.0):
            obj = CollisionObject()
            obj.id = oid; obj.header.frame_id = "world"
            obj.operation = CollisionObject.ADD
            obj.primitives.append(SolidPrimitive(
                type=SolidPrimitive.CYLINDER, dimensions=[height, radius]))
            obj.primitive_poses.append(_make_pose_msg(cx, cy, cz, qx, qy, qz, qw))
            objects.append(obj)

        segments = cfg.get("body_segments", [])
        SQRT2_2 = 0.70710678

        # ═══════════════════════════════════════════════════════
        # 背部: Y轴圆柱体，底部靠在床垫上 (z_bottom = mat_top)
        # Z→Y 旋转: 绕X轴 +90° → qx=√2/2, qw=√2/2
        # ═══════════════════════════════════════════════════════
        for i, seg in enumerate(segments):
            r = seg["thick"] * 0.35          # 薄圆柱，不阻挡按摩路径
            h = seg["half_w"] * 2.0          # 横向跨背宽度
            z_c = max(mat_top + r, seg["z"] - 0.010)  # 底部=床垫, 顶部≈体表
            _add_cylinder(f"back_{i:02d}_{seg['name']}",
                          seg["x"], 0.0, z_c, h, r,
                          qx=SQRT2_2, qw=SQRT2_2)

        # 段间插值 (11根)
        for i in range(len(segments) - 1):
            s0, s1 = segments[i], segments[i + 1]
            x_mid = (s0["x"] + s1["x"]) * 0.5
            hw_mid = (s0["half_w"] + s1["half_w"]) * 0.5
            th_mid = (s0["thick"] + s1["thick"]) * 0.5
            r_mid = th_mid * 0.35
            z_surf_mid = (s0["z"] + s1["z"]) * 0.5
            z_mid = max(mat_top + r_mid, z_surf_mid - 0.010)
            _add_cylinder(f"back_i{i:02d}", x_mid, 0.0, z_mid,
                          hw_mid * 2.0, r_mid, qx=SQRT2_2, qw=SQRT2_2)

        # ═══════════════════════════════════════════════════════
        # 头部: 缩小前移，不阻挡 C7 (x≈0.38)
        #   r=0.065 → 范围 x:0.175-0.305, z:0.165-0.295
        #   C7 区 (x≥0.33) 完全避开
        # ═══════════════════════════════════════════════════════
        _add_sphere("head", 0.24, 0.0, 0.23, 0.065)

        # 颈: 短细圆柱，位于头与 C7 之间
        _add_cylinder("neck", 0.30, 0.0, 0.205, 0.04, 0.035)

        # ═══════════════════════════════════════════════════════
        # 双臂: 圆柱体沿肢体方向
        # ═══════════════════════════════════════════════════════
        for side, sy in [("left", -1.0), ("right", 1.0)]:
            # 上臂: 肩(0.42, ±0.20, 0.20) → 肘(0.56, ±0.25, 0.16)
            ua_dir = (0.14, sy * 0.05, -0.04)
            ua_q = _quat_z_to_dir(*ua_dir)
            ua_cx = 0.49; ua_cy = sy * 0.225
            ua_r = 0.030; ua_cz = max(mat_top + ua_r, 0.18)
            _add_cylinder(f"{side}_upper_arm", ua_cx, ua_cy, ua_cz,
                          0.17, ua_r, *ua_q)

            # 前臂: 肘(0.56, ±0.25, 0.16) → 腕(0.72, ±0.26, 0.13)
            fa_dir = (0.16, sy * 0.01, -0.03)
            fa_q = _quat_z_to_dir(*fa_dir)
            fa_cx = 0.64; fa_cy = sy * 0.255
            fa_r = 0.028; fa_cz = max(mat_top + fa_r, 0.145)
            _add_cylinder(f"{side}_forearm", fa_cx, fa_cy, fa_cz,
                          0.15, fa_r, *fa_q)

            # 手掌: 小球体
            hr = 0.035; hz = max(mat_top + hr, 0.13)
            _add_sphere(f"{side}_hand", 0.76, sy * 0.26, hz, hr)

        # ═══════════════════════════════════════════════════════
        # 双腿: 圆柱体沿腿方向 (底部=床垫)
        # ═══════════════════════════════════════════════════════
        for side, sy in [("left", -1.0), ("right", 1.0)]:
            # 大腿: 髋(0.85, ±0.10, 0.15) → 膝(1.02, ±0.12, 0.12)
            th_dir = (0.17, sy * 0.02, -0.03)
            th_q = _quat_z_to_dir(*th_dir)
            th_r = 0.042; th_cz = max(mat_top + th_r, 0.135)
            _add_cylinder(f"{side}_thigh", 0.935, sy * 0.11, th_cz,
                          0.22, th_r, *th_q)

            # 小腿: 膝(1.02, ±0.12, 0.12) → 踝(1.22, ±0.12, 0.07)
            ca_dir = (0.20, 0.0, -0.05)
            ca_q = _quat_z_to_dir(*ca_dir)
            ca_r = 0.038; ca_cz = max(mat_top + ca_r, 0.095)
            _add_cylinder(f"{side}_calf", 1.12, sy * 0.12, ca_cz,
                          0.20, ca_r, *ca_q)

            # 脚: 扁 Box，紧贴床垫
            ft_cz = max(mat_top + 0.02, 0.05)
            _add_box(f"{side}_foot", 1.30, sy * 0.12, ft_cz,
                     0.16, 0.07, 0.04)

    def _publish_body_markers(self, node: Node, cfg: dict, mat_top: float):
        """发布彩色人体 Marker (肤色) 到 /rviz_visual_tools，与床的 PlanningScene 颜色区分。"""
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point as Pt, Quaternion as Qt
        from std_msgs.msg import ColorRGBA

        # Create publisher if not already cached
        if not hasattr(self, "_marker_pub"):
            self._marker_pub = node.create_publisher(
                MarkerArray, "/rviz_visual_tools", 10)
            self._marker_id = 0

        markers = MarkerArray()
        mid = 0

        def _mk(_id, _ns, _type, cx, cy, cz, sx, sy, sz,
                qx=0.0, qy=0.0, qz=0.0, qw=1.0, r=0.86, g=0.72, b=0.60, a=0.80):
            m = Marker()
            m.header.frame_id = "world"
            m.ns = _ns
            m.id = _id
            m.type = _type
            m.action = Marker.ADD
            m.pose.position = Pt(x=cx, y=cy, z=cz)
            m.pose.orientation = Qt(x=qx, y=qy, z=qz, w=qw)
            m.scale.x = sx; m.scale.y = sy; m.scale.z = sz
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            m.lifetime.sec = 0  # persistent
            return m

        segments = cfg.get("body_segments", [])

        # Back cylinders — skin tone
        for i, seg in enumerate(segments):
            r = seg["thick"] * 0.35; h = seg["half_w"] * 2.0
            z_c = max(mat_top + r, seg["z"] - 0.010)
            markers.markers.append(_mk(mid, "body", Marker.CYLINDER,
                seg["x"], 0.0, z_c, h, r, r, 0.7071, 0.0, 0.0, 0.7071))
            mid += 1

        # Interpolated back cylinders
        for i in range(len(segments) - 1):
            s0, s1 = segments[i], segments[i + 1]
            xm = (s0["x"] + s1["x"]) * 0.5
            hwm = (s0["half_w"] + s1["half_w"]) * 0.5
            thm = (s0["thick"] + s1["thick"]) * 0.5
            rm = thm * 0.35
            zm = max(mat_top + rm, (s0["z"] + s1["z"]) * 0.5 - 0.010)
            markers.markers.append(_mk(mid, "body", Marker.CYLINDER,
                xm, 0.0, zm, hwm * 2.0, rm, rm, 0.7071, 0.0, 0.0, 0.7071))
            mid += 1

        # Head
        markers.markers.append(_mk(mid, "body", Marker.SPHERE,
            0.24, 0.0, 0.23, 0.065*2, 0.065*2, 0.065*2))
        mid += 1

        # Neck
        markers.markers.append(_mk(mid, "body", Marker.CYLINDER,
            0.30, 0.0, 0.205, 0.04, 0.035, 0.035))
        mid += 1

        # Arms + hands
        for side, sy in [("left", -1.0), ("right", 1.0)]:
            markers.markers.append(_mk(mid, "body", Marker.CYLINDER,
                0.49, sy*0.225, 0.165, 0.17, 0.030, 0.030))
            mid += 1
            markers.markers.append(_mk(mid, "body", Marker.CYLINDER,
                0.64, sy*0.255, 0.145, 0.15, 0.028, 0.028))
            mid += 1
            markers.markers.append(_mk(mid, "body", Marker.SPHERE,
                0.76, sy*0.26, 0.13, 0.07, 0.07, 0.07))
            mid += 1

        # Legs + feet
        for side, sy in [("left", -1.0), ("right", 1.0)]:
            markers.markers.append(_mk(mid, "body", Marker.CYLINDER,
                0.935, sy*0.11, 0.135, 0.22, 0.042, 0.042))
            mid += 1
            markers.markers.append(_mk(mid, "body", Marker.CYLINDER,
                1.12, sy*0.12, 0.095, 0.20, 0.038, 0.038))
            mid += 1
            markers.markers.append(_mk(mid, "body", Marker.CUBE,
                1.30, sy*0.12, 0.05, 0.16, 0.07, 0.04))
            mid += 1

        # Bed markers — cool gray, different from body skin tone
        bed = cfg.get("bed", {})
        bcx = bed["center"]["x"]; bcy = bed["center"]["y"]
        frame = bed.get("frame", {})
        fz = frame.get("bottom_z", 0.0) + frame.get("size", {}).get("z", 0.08)/2
        fs = frame.get("size", {})
        markers.markers.append(_mk(mid, "bed", Marker.CUBE,
            bcx, bcy, fz, fs.get("x",1.2), fs.get("y",0.66), fs.get("z",0.08),
            r=0.35, g=0.35, b=0.40, a=0.85))
        mid += 1

        mattress = bed.get("mattress", {})
        ms = mattress.get("size", {})
        mz = mattress.get("bottom_z", 0.08) + ms.get("z", 0.06)/2
        markers.markers.append(_mk(mid, "mattress", Marker.CUBE,
            bcx, bcy, mz, ms.get("x",1.12), ms.get("y",0.56), ms.get("z",0.06),
            r=0.50, g=0.55, b=0.70, a=0.80))
        mid += 1

        self._marker_pub.publish(markers)
        node.get_logger().info(
            f"Visual markers published: {mid} markers on /rviz_visual_tools"
        )

    def _apply_scene(self, node: Node, objects: list):
        """Apply collision objects via /apply_planning_scene service with retry."""
        from moveit_msgs.msg import PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene

        planner = self.blackboard.get("planner")
        if planner is None:
            node.get_logger().error("No planner, cannot apply scene")
            return

        # Wait for service
        if not planner._apply_scene_client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(
                "/apply_planning_scene not available after 5s — scene NOT applied"
            )
            return

        scene = PlanningScene()
        scene.world.collision_objects = objects
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = planner._apply_scene_client.call_async(req)
        _spin_future(self.blackboard, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            node.get_logger().info(
                f"Scene applied: {len(objects)} collision objects registered"
            )
        else:
            node.get_logger().error("ApplyPlanningScene failed")


def _make_pose_msg(x: float, y: float, z: float,
                   qx: float = 0.0, qy: float = 0.0,
                   qz: float = 0.0, qw: float = 1.0) -> Pose:
    from geometry_msgs.msg import Point, Quaternion
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


# ═══════════════════════════════════════════════════════════════
# InitSurfaceModel
# ═══════════════════════════════════════════════════════════════

class InitSurfaceModel(BtActionNode):
    """从 body_config 构建 Catmull-Rom 样条曲面模型 + 路径生成器。

    写入 blackboard:
        blackboard["surface"]         — BackSurfaceModel 实例
        blackboard["path_generator"]  — PathGenerator 实例
    """

    def __init__(self, name: str = "InitSurfaceModel"):
        super().__init__(name)

    def execute(self) -> NodeStatus:
        node: Optional[Node] = self.blackboard.get("node")
        body_cfg: Optional[dict] = self.blackboard.get("body_config")

        if node is None or body_cfg is None:
            return NodeStatus.FAILURE

        try:
            surface = BackSurfaceModel(body_cfg)
            self.blackboard["surface"] = surface
            self.blackboard["path_generator"] = PathGenerator(surface)

            node.get_logger().info(
                f"Surface model initialized: "
                f"{len(body_cfg.get('body_segments', []))} segments, "
                f"spine arc={surface.spine.total_arc_length:.3f}m, "
                f"7 zones, {len(body_cfg.get('acupoints', []))} acupoints"
            )
            return NodeStatus.SUCCESS
        except Exception as e:
            node.get_logger().error(f"Failed to init surface model: {e}")
            return NodeStatus.FAILURE


# ═══════════════════════════════════════════════════════════════
# RunMassageCycle
# ═══════════════════════════════════════════════════════════════

class RunMassageCycle(BtActionNode):
    """核心: 遍历 60 阶段，每阶段 生成路径 → 规划 → 执行。

    单臂操作: 另一臂保持当前位置 (hover)
    双臂操作: 先规划冲突检测，无冲突则并行，有冲突则串行
    每阶段间: SafetyMonitor 检查 (若 blackboard 中存在)
    """

    def __init__(self, name: str = "RunMassageCycle"):
        super().__init__(name)
        self._traj_index = 0

    def execute(self) -> NodeStatus:
        node: Optional[Node] = self.blackboard.get("node")
        stages: Optional[List[dict]] = self.blackboard.get("stages")
        generator: Optional[PathGenerator] = self.blackboard.get("path_generator")
        planner = self.blackboard.get("planner")
        safety = self.blackboard.get("safety")

        if not all([node, stages, generator]):
            node.get_logger().error("Missing blackboard: node/stages/generator") \
                if node else None
            return NodeStatus.FAILURE

        total = len(stages)
        node.get_logger().info(f"Starting massage cycle: {total} stages (infinite loop)")

        cycle = 0
        while rclpy.ok():
            cycle += 1
            node.get_logger().info(
                f"╔══ Massage Cycle {cycle} ═══════════════════════════"
            )

            for idx, stage in enumerate(stages):
                # Safety check
                if safety is not None:
                    from jaka_dual_arm.control.safety_monitor import SafetyLevel
                    level = safety.check()
                    if level.value >= SafetyLevel.HALT.value:
                        node.get_logger().error(
                            f"Safety HALT at cycle {cycle} stage {idx + 1}: {level}"
                        )
                        return NodeStatus.FAILURE

                stage_id = stage.get("id", idx + 1)
                stage_name = stage.get("name", f"Stage {stage_id}")
                node.get_logger().info(
                    f"[C{cycle} {idx + 1}/{total}] {stage_name}"
                )

                try:
                    left_def = stage.get("left", {})
                    right_def = stage.get("right", {})

                    # ── Left arm: generate → plan → execute ──
                    # (Execute left FIRST so right arm planner sees left's final
                    #  position via /joint_states, avoiding cross-arm collisions)
                    left_poses = generator.generate(left_def) if left_def else []
                    if left_poses:
                        left_traj = self._plan_arm(node, planner, left_poses, "left",
                                                   left_def.get("technique", "hover"))
                        if left_traj:
                            self._execute_trajectory(node, left_traj, "left",
                                                     left_def.get("technique", "hover"))

                    # ── Right arm: generate → plan → execute ──
                    # (Left arm has already moved; planner avoids its new position)
                    right_poses = generator.generate(right_def) if right_def else []
                    if right_poses:
                        right_traj = self._plan_arm(node, planner, right_poses, "right",
                                                    right_def.get("technique", "hover"))
                        if right_traj:
                            self._execute_trajectory(node, right_traj, "right",
                                                     right_def.get("technique", "hover"))

                except Exception as e:
                    node.get_logger().error(
                        f"Cycle {cycle} Stage {stage_id} failed: {e}"
                    )
                    return NodeStatus.FAILURE

                # Allow ROS to process callbacks (use executor to spin both nodes)
                executor = self.blackboard.get("executor")
                if executor is not None:
                    executor.spin_once(timeout_sec=0.01)
                else:
                    rclpy.spin_once(node, timeout_sec=0.01)

            node.get_logger().info(
                f"╚══ Cycle {cycle} complete ({total} stages) — continuing ══"
            )

        # Should never reach here unless rclpy is shutting down
        node.get_logger().info("Massage loop terminated (rclpy shutdown).")
        return NodeStatus.SUCCESS

    def _plan_arm(self, node: Node, planner, poses: List[Pose],
                  arm: str, technique: str) -> Optional[JointTrajectory]:
        """为单臂规划轨迹。

        策略: 始终从上方 3cm/6cm 开始规划。
        因为即使是 hover 目标，直接规划也可能与头/颈碰撞体重叠。
        from-above 规划成功率高 (~0.1s)，直接规划作为最后回退。
        """
        if not poses:
            return None

        if len(poses) == 1:
            target_pose = poses[0]
            group = f"{arm}_arm"

            if planner is not None:
                from geometry_msgs.msg import Point

                # Level 1-2: approach from above (high success rate, fast)
                for dz, label in [(0.03, "3cm"), (0.06, "6cm")]:
                    node.get_logger().info(
                        f"{arm} arm: {technique} → plan from {label} above"
                    )
                    approach_pose = Pose()
                    approach_pose.position = Point(
                        x=target_pose.position.x,
                        y=target_pose.position.y,
                        z=target_pose.position.z + dz,
                    )
                    approach_pose.orientation = target_pose.orientation
                    traj = planner.plan_pose_target(approach_pose, group=group)
                    if traj is not None and traj.points:
                        return traj

                # Level 3: direct plan as last resort
                node.get_logger().warn(
                    f"{arm} arm: approach plans failed, trying direct"
                )
                traj = planner.plan_pose_target(target_pose, group=group)
                if traj is not None and traj.points:
                    return traj

            node.get_logger().error(
                f"{arm} arm: ALL plans FAILED for "
                f"({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, "
                f"{target_pose.position.z:.3f}) technique={technique}"
            )
            return None
        else:
            # Multi-pose: Cartesian path
            traj = self._plan_cartesian(node, poses, arm)
            if traj is not None and traj.points:
                return traj

            # Fallback: plan just start→end
            if planner is not None:
                group = f"{arm}_arm"
                return planner.plan_pose_target(poses[-1], group=group)
            return None

    def _plan_cartesian(self, node: Node, poses: List[Pose],
                        arm: str) -> Optional[JointTrajectory]:
        """调用 MoveIt2 /compute_cartesian_path 规划 Cartesian 路径。"""
        from moveit_msgs.srv import GetCartesianPath

        planner = self.blackboard.get("planner")
        if planner is None:
            return None

        # Create client if not exist
        cartesian_client = getattr(self, "_cartesian_client", None)
        if cartesian_client is None:
            cartesian_client = node.create_client(
                GetCartesianPath, "/compute_cartesian_path"
            )
            self._cartesian_client = cartesian_client

        if not cartesian_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().warn("Cartesian path service not available")
            return None

        req = GetCartesianPath.Request()
        req.group_name = f"{arm}_arm"
        req.waypoints = poses
        req.max_step = 0.03           # 3cm 步长 (机械臂可轻松处理)
        req.jump_threshold = 0.0      # 禁止跳跃
        req.avoid_collisions = True

        # Get current joint state as start
        js = planner.current_joint_state
        if js is not None:
            req.start_state.joint_state = js

        future = cartesian_client.call_async(req)
        _spin_future(self.blackboard, future, timeout_sec=10.0)
        result = future.result()

        if result is None:
            node.get_logger().warn("Cartesian path: no response")
            return None

        if result.error_code.val != 1:
            node.get_logger().warn(
                f"Cartesian path failed: code={result.error_code.val}, "
                f"completion={result.fraction:.1%}"
            )
            return None

        # Reject paths with too low completion (<50% of waypoints planned)
        if result.fraction < 0.5:
            node.get_logger().warn(
                f"Cartesian path completion too low: {result.fraction:.1%}, "
                f"falling back to joint-space plan"
            )
            return None

        traj = result.solution.joint_trajectory
        node.get_logger().info(
            f"Cartesian path: {len(traj.points)} pts "
            f"({result.fraction:.1%} of path)"
        )
        return traj

    def _execute_trajectory(self, node: Node, traj: JointTrajectory,
                            arm: str, technique: str):
        """发送轨迹到控制器并等待执行完成。"""
        # Build action client name
        controller_name = f"/{arm}_arm_controller/follow_joint_trajectory"
        action_client = getattr(self, f"_{arm}_action_client", None)

        if action_client is None:
            from control_msgs.action import FollowJointTrajectory
            from rclpy.action import ActionClient
            action_client = ActionClient(node, FollowJointTrajectory, controller_name)
            setattr(self, f"_{arm}_action_client", action_client)

        if not action_client.wait_for_server(timeout_sec=2.0):
            node.get_logger().warn(f"Controller {controller_name} not ready, skipping")
            return

        from control_msgs.action import FollowJointTrajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_future = action_client.send_goal_async(goal)
        _spin_future(self.blackboard, send_future, timeout_sec=5.0)

        if not send_future.result():
            node.get_logger().error(f"Failed to send goal to {arm} arm")
            return

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            node.get_logger().error(f"Goal rejected by {arm} arm controller")
            return

        # Wait for execution
        result_future = goal_handle.get_result_async()
        timeout = traj.points[-1].time_from_start.sec + 15.0 if traj.points else 30.0
        _spin_future(self.blackboard, result_future, timeout_sec=timeout)

        node.get_logger().info(
            f"{arm.capitalize()} arm: {technique} done "
            f"({len(traj.points)} pts)"
        )


# ═══════════════════════════════════════════════════════════════
# RetreatToHome
# ═══════════════════════════════════════════════════════════════

class RetreatToHome(BtActionNode):
    """双臂回到安全初始位姿 (悬停状态)。"""

    def __init__(self, name: str = "RetreatToHome"):
        super().__init__(name)

    def execute(self) -> NodeStatus:
        node: Optional[Node] = self.blackboard.get("node")
        planner = self.blackboard.get("planner")

        if node is None:
            return NodeStatus.FAILURE

        node.get_logger().info("Retreating to home position...")

        try:
            if planner is not None:
                # Plan each arm separately (avoids 12-joint traj sent to 6-joint controller)
                for arm, joints, home in [
                    ("left", LEFT_JOINTS, HOME_LEFT),
                    ("right", RIGHT_JOINTS, HOME_RIGHT),
                ]:
                    traj = planner.plan_joint_target(
                        left_target=home if arm == "left" else None,
                        right_target=home if arm == "right" else None,
                        left_joints=LEFT_JOINTS,
                        right_joints=RIGHT_JOINTS,
                    )
                    if traj and traj.points:
                        self._send_and_wait(node, traj, arm)
                    else:
                        node.get_logger().warn(f"{arm} arm home plan failed, skipping")

                node.get_logger().info("Home position reached.")
                return NodeStatus.SUCCESS
        except Exception as e:
            node.get_logger().error(f"Retreat failed: {e}")

        return NodeStatus.FAILURE

    def _send_and_wait(self, node: Node, traj: JointTrajectory, arm: str):
        """发送 JointTrajectory 到指定臂并等待完成。

        自动过滤轨迹中的关节名称，使 12 关节轨迹可以发给单臂控制器。
        """
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient

        controller = f"/{arm}_arm_controller/follow_joint_trajectory"
        client = ActionClient(node, FollowJointTrajectory, controller)

        if not client.wait_for_server(timeout_sec=2.0):
            node.get_logger().warn(f"Controller {controller} not ready, skipping")
            return

        # Filter trajectory to this arm's joints only
        arm_joints = set(LEFT_JOINTS if arm == "left" else RIGHT_JOINTS)
        filtered = JointTrajectory()
        filtered.header = traj.header
        filtered.joint_names = [j for j in traj.joint_names if j in arm_joints]

        # Map full-traj indices → filtered indices
        idx_map = [traj.joint_names.index(j) for j in filtered.joint_names]

        from trajectory_msgs.msg import JointTrajectoryPoint
        for pt in traj.points:
            fpt = JointTrajectoryPoint()
            fpt.positions = [pt.positions[i] for i in idx_map]
            if pt.velocities:
                fpt.velocities = [pt.velocities[i] for i in idx_map]
            fpt.time_from_start = pt.time_from_start
            filtered.points.append(fpt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = filtered

        future = client.send_goal_async(goal)
        _spin_future(self.blackboard, future, timeout_sec=5.0)
        if future.result() and future.result().accepted:
            result_future = future.result().get_result_async()
            timeout = (filtered.points[-1].time_from_start.sec + 10.0
                       if filtered.points else 20.0)
            _spin_future(self.blackboard, result_future, timeout_sec=timeout)


# ═══════════════════════════════════════════════════════════════
# Node Registry Factory
# ═══════════════════════════════════════════════════════════════

def create_massage_node_registry() -> "NodeRegistry":
    """创建按摩 BT 节点注册表。

    Returns:
        NodeRegistry with all 5 massage nodes registered
    """
    from jaka_dual_arm.behavior.bt_engine import NodeRegistry

    registry = NodeRegistry()
    registry.register("WaitServices", lambda: WaitServices())
    registry.register("SetupMassageScene", lambda: SetupMassageScene())
    registry.register("InitSurfaceModel", lambda: InitSurfaceModel())
    registry.register("RunMassageCycle", lambda: RunMassageCycle())
    registry.register("RetreatToHome", lambda: RetreatToHome())
    return registry
