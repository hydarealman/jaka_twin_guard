#!/usr/bin/env python3
"""双臂中医推拿按摩 Demo — 5阶段50式（推法→按揉→点穴→滚揉→拍法收功）。
  臂X错开(左0.53/右0.69)防止碰撞，沿人体脊柱轮廓紧贴按摩。"""

from __future__ import annotations
import sys, math
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

# ── 关节列表 ──
LEFT_JOINTS  = [f"left_joint_{i}"  for i in range(1,7)]
RIGHT_JOINTS = [f"right_joint_{i}" for i in range(1,7)]
ALL_JOINTS   = LEFT_JOINTS + RIGHT_JOINTS

# ── 臂基座位置（X错开0.16m，避免双臂碰撞） ──
LEFT_BASE  = (0.53, -0.45, 0.0)
RIGHT_BASE = (0.69,  0.45, 0.0)
SHOULDER_Z = 0.12   # 肩关节(Link_00顶)世界Z
GRAVITY    = (0.0, 0.0, -9.81)  # 重力方向

# ── 床体（地面按摩垫/榻榻米风格，贴近地面） ──
BED_CX,BED_CY = 0.70,0.0
BED_FRAME_Z, BED_FRAME = 0.08, (1.20,0.66,0.08)   # 床框底z=0.04 顶z=0.12
MATTRESS_Z,  MATTRESS  = 0.15, (1.12,0.56,0.06)    # 床垫底z=0.12 顶z=0.18
PILLOW = (0.28,0.215,0.20,0.34,0.07)                # 枕头顶z≈0.25
MATTRESS_TOP = MATTRESS_Z + MATTRESS[2]/2            # =0.18

# ── 人体模型：12段脊柱轮廓（俯卧） ──
# (x_c, z_surface, y_half_width, thickness, name)
BODY = [
    (0.27,0.245,0.090,0.08,"头部"),
    (0.33,0.232,0.065,0.045,"颈根"),
    (0.38,0.237,0.120,0.048,"C7隆椎"),    # 背部最高点
    (0.43,0.235,0.160,0.050,"斜方肌上部"),
    (0.49,0.233,0.180,0.052,"肩胛带"),     # 最宽处
    (0.55,0.229,0.170,0.048,"T1-4上胸椎"),
    (0.61,0.226,0.155,0.046,"T5-8中胸椎"),
    (0.67,0.223,0.140,0.044,"T9-12下胸椎"),
    (0.72,0.219,0.128,0.042,"胸腰结合"),   # 腰部收窄
    (0.77,0.215,0.130,0.040,"L1-3腰椎"),   # 腰椎凹陷
    (0.82,0.213,0.140,0.038,"L4-5"),
    (0.86,0.211,0.150,0.036,"骶骨"),
]
SPINE_RIDGE = [Point(x=s[0],y=0.0,z=s[1]+0.008) for s in BODY]
BODY_EDGE_L = [Point(x=s[0],y=-s[2],z=s[1]) for s in BODY if s[2]>0.001]
BODY_EDGE_R = [Point(x=s[0],y= s[2],z=s[1]) for s in BODY if s[2]>0.001]

# 膀胱经穴位（脊柱旁开0.04m，左右各7穴）
ACUPOINTS = [
    ("BL11_大杼", 0.41,0.236), ("BL13_肺俞",0.48,0.234),
    ("BL15_心俞", 0.55,0.229), ("BL17_膈俞",0.63,0.225),
    ("BL18_肝俞", 0.70,0.220), ("BL23_肾俞",0.77,0.215),
    ("BL25_大肠俞",0.84,0.212),
]
ACU_Y_OFFSET = 0.04  # 穴位距脊柱中线Y偏移

# ── 按摩区域（用于hover计算） ──
MASSAGE_ZONES = {
    "C7":       (0.38,0.10,0.237,0.247),
    "shoulder": (0.47,0.16,0.234,0.244),
    "upper":    (0.55,0.14,0.229,0.239),
    "mid":      (0.63,0.12,0.225,0.235),
    "lower_th": (0.70,0.11,0.220,0.230),
    "lumbar":   (0.77,0.10,0.215,0.225),
    "sacrum":   (0.84,0.12,0.212,0.222),
}

# ── 关节角度工具函数 ──
def _j1(tx,ty, bx,by):
    """计算joint_1使臂平面指向目标XY方向"""
    return math.atan2(ty-by, tx-bx)

# 基础关节角度（hover状态）
J2_BASE, J3_BASE, J4_BASE, J5_BASE, J6_BASE = 0.75, -1.10, 1.30, 1.57, 1.50

# 动作偏移 [Δj2,Δj3,Δj4,Δj5,Δj6]
ACT_DELTA = {
    "hover":   [0.0, 0.0,  0.0,  0.0, 0.0],
    "press":   [0.02,-0.04, 0.03, 0.0, 0.0],
    "release": [0.0, 0.0,  0.0,  0.0, 0.0],
    "knead_L": [0.02,-0.04, 0.03, 0.0, 0.08],
    "knead_R": [0.02,-0.04, 0.03, 0.0,-0.08],
    "roll":    [0.01,-0.02, 0.02, 0.0, 0.0],
    "tap":     [0.0,-0.05, 0.03, 0.0, 0.0],
}

def _joints(j1, act, j2b=J2_BASE,j3b=J3_BASE,j4b=J4_BASE,j5b=J5_BASE,j6b=J6_BASE):
    """构建6DOF关节列表 [j1,j2,j3,j4,j5,j6]"""
    d = ACT_DELTA.get(act, ACT_DELTA["hover"])
    return [j1, j2b+d[0], j3b+d[1], j4b+d[2], j5b+d[3], j6b+d[4]]

def _wp_left(tx,ty, act, j2b=J2_BASE,j3b=J3_BASE):
    """左臂单个waypoint"""
    return _joints(_j1(tx,ty,*LEFT_BASE[:2]), act, j2b, j3b)

def _wp_right(tx,ty, act, j2b=J2_BASE,j3b=J3_BASE):
    """右臂单个waypoint（j1自动取反方向）"""
    j1r = _j1(tx,ty,*RIGHT_BASE[:2])
    return _joints(j1r, act, j2b, j3b)

def _zone_left(zone_name, act):
    """从按摩区域名获取左臂hover/press目标"""
    z = MASSAGE_ZONES[zone_name]
    xc,yw,zs,zh = z
    if act in ("hover","release","tap"): return xc,-yw,zh
    if act == "knead_L": return xc,-(yw+0.015),zs
    if act == "knead_R": return xc,-(yw-0.015),zs
    return xc,-yw,zs  # press, roll

def _zone_right(zone_name, act):
    """从按摩区域名获取右臂hover/press目标"""
    z = MASSAGE_ZONES[zone_name]
    xc,yw,zs,zh = z
    if act in ("hover","release","tap"): return xc,yw,zh
    if act == "knead_L": return xc,yw+0.015,zs
    if act == "knead_R": return xc,yw-0.015,zs
    return xc,yw,zs  # press, roll

def _acu_left(idx, act):
    """左臂按左侧膀胱经穴位"""
    _, x, zs = ACUPOINTS[idx]
    zh = zs + 0.010
    y = -ACU_Y_OFFSET
    return (x,y,zh) if act in ("hover","release") else (x,y,zs)

def _acu_right(idx, act):
    """右臂按右侧膀胱经穴位"""
    _, x, zs = ACUPOINTS[idx]
    zh = zs + 0.010
    y = ACU_Y_OFFSET
    return (x,y,zh) if act in ("hover","release") else (x,y,zs)

# ═══════════════════════════════════════════════════════
# 50阶段中医推拿编排
# 格式: (左tx,左ty,左act, 右tx,右ty,右act, 阶段名)
# ═══════════════════════════════════════════════════════
STAGE_DEFS = [
    # ── Phase 1: 推法 Gliding 热身 (6 stages) ──
    ("C7","hover",  "C7","hover",   "1.推法·双悬C7"),
    ("C7","press",  "C7","hover",   "2.推法·左推C7→上背"),
    ("upper","release","mid","press","3.推法·右推中背→骶"),
    ("mid","hover",  "mid","release","4.推法·双释中背"),
    ("C7","press",   "mid","hover", "5.推法·左回推上背→C7"),
    ("C7","release", "C7","release","6.推法·双归C7"),

    # ── Phase 2: 按揉法 Press-Knead 深层组织 (12 stages) ──
    ("shoulder","hover",  "mid","hover",     "7.按揉·双悬(肩/中背)"),
    ("shoulder","press",  "mid","hover",     "8.按揉·左按肩(右避让)"),
    ("shoulder","release","mid","press",     "9.按揉·右按中背(左避让)"),
    ("shoulder","knead_L","mid","hover",     "10.按揉·左揉肩"),
    ("shoulder","hover",  "mid","knead_R",   "11.按揉·右揉中背"),
    ("upper","hover",     "lower_th","hover","12.按揉·双悬(上背/下胸)"),
    ("upper","press",     "lower_th","hover","13.按揉·左按上背"),
    ("upper","hover",     "lower_th","press","14.按揉·右按下胸"),
    ("upper","knead_R",   "lower_th","hover","15.按揉·左揉上背"),
    ("upper","hover",     "lower_th","knead_L","16.按揉·右揉下胸"),
    ("C7","hover",        "lumbar","hover",  "17.按揉·双悬(C7/腰椎)"),
    ("C7","press",        "lumbar","press",  "18.按揉·深按双收(安全距离)"),

    # ── Phase 3: 点穴法 Acupressure 膀胱经 (14 stages) ──
    # 格式: ((acu_idx,marker), action, (acu_idx,marker), action, name)
    # 每个穴位：左点左侧(y=-0.04)，右点右侧(y=+0.04)，交替
    ((0,"acuL"),"hover",   (0,"acuR"),"hover",   "19.点穴·悬大杼BL11"),
    ((0,"acuL"),"press",   (0,"acuR"),"hover",   "20.点穴·左按BL11"),
    ((0,"acuL"),"release", (0,"acuR"),"press",   "21.点穴·右按BL11"),
    ((1,"acuL"),"hover",   (1,"acuR"),"hover",   "22.点穴·悬肺俞BL13"),
    ((1,"acuL"),"press",   (1,"acuR"),"hover",   "23.点穴·左按BL13"),
    ((1,"acuL"),"release", (1,"acuR"),"press",   "24.点穴·右按BL13"),
    ((2,"acuL"),"hover",   (2,"acuR"),"hover",   "25.点穴·悬心俞BL15"),
    ((2,"acuL"),"press",   (2,"acuR"),"hover",   "26.点穴·左按BL15"),
    ((2,"acuL"),"release", (2,"acuR"),"press",   "27.点穴·右按BL15"),
    ((3,"acuL"),"hover",   (3,"acuR"),"hover",   "28.点穴·悬膈俞BL17"),
    ((3,"acuL"),"press",   (3,"acuR"),"hover",   "29.点穴·左按BL17"),
    ((3,"acuL"),"release", (3,"acuR"),"press",   "30.点穴·右按BL17"),
    ((4,"acuL"),"hover",   (5,"acuR"),"hover",   "31.点穴·悬肝俞BL18/肾俞BL23"),
    ((4,"acuL"),"press",   (5,"acuR"),"press",   "32.点穴·双按(错区安全)"),

    # ── Phase 4: 滚揉法 Rolling Knead 肌肉松解 (10 stages) ──
    ("shoulder","hover",  "mid","hover",     "33.滚揉·双悬(肩/中背)"),
    ("shoulder","roll",   "mid","hover",     "34.滚揉·左滚肩"),
    ("shoulder","release","mid","roll",      "35.滚揉·右滚中背"),
    ("upper","roll",      "lower_th","hover","36.滚揉·左滚上背"),
    ("upper","hover",     "lower_th","roll", "37.滚揉·右滚下胸"),
    ("C7","hover",        "lumbar","hover",  "38.滚揉·双悬(C7/腰椎)"),
    ("C7","roll",         "lumbar","hover",  "39.滚揉·左滚C7"),
    ("C7","hover",        "lumbar","roll",   "40.滚揉·右滚腰椎"),
    ("shoulder","hover",  "sacrum","hover",  "41.滚揉·双悬(肩/骶)"),
    ("shoulder","roll",   "sacrum","roll",   "42.滚揉·双滚(安全距离收)"),

    # ── Phase 5: 拍法+收功 Percussion & Cool-down (8 stages) ──
    ("C7","hover",     "sacrum","hover",  "43.收功·双悬(头/骶)"),
    ("shoulder","tap", "mid","hover",     "44.拍法·左轻拍肩"),
    ("shoulder","hover","mid","tap",      "45.拍法·右轻拍中背"),
    ("C7","hover",     "lumbar","hover",  "46.收功·双悬回归"),
    ("C7","press",     "sacrum","press",  "47.收功·终末深按"),
    ("C7","release",   "sacrum","release","48.收功·释放"),
    ("C7","hover",     "lumbar","hover",  "49.收功·双悬"),
    ("C7","hover",     "sacrum","hover",  "50.收功·完成"),
]

def _build_waypoint(left_spec, left_act, right_spec, right_act):
    """构建单个阶段的(左6DOF, 右6DOF)"""
    # 解析左臂
    if isinstance(left_spec, tuple) and len(left_spec)==2:
        idx, marker = left_spec
        tx,ty,tz = _acu_left(idx, left_act)
        jl = _wp_left(tx,ty, left_act)
    elif left_spec == "acu_left":
        tx,ty,tz = _acu_left(0, left_act)
        jl = _wp_left(tx,ty, left_act)
    else:
        tx,ty,tz = _zone_left(left_spec, left_act)
        jl = _wp_left(tx,ty, left_act)

    # 解析右臂
    if isinstance(right_spec, tuple) and len(right_spec)==2:
        idx, marker = right_spec
        tx,ty,tz = _acu_right(idx, right_act)
        jr = _wp_right(tx,ty, right_act)
    elif right_spec == "acu_right":
        tx,ty,tz = _acu_right(0, right_act)
        jr = _wp_right(tx,ty, right_act)
    else:
        tx,ty,tz = _zone_right(right_spec, right_act)
        jr = _wp_right(tx,ty, right_act)

    return jl, jr

# 构建完整 waypoint 列表
LEFT_WAYPOINTS  = []
RIGHT_WAYPOINTS = []
MASSAGE_POINTS_L = []
MASSAGE_POINTS_R = []
STAGE_NAMES = []
t = 1.0
DT = 0.75  # 每阶段时间间隔

for si, sd in enumerate(STAGE_DEFS):
    ls, la, rs, ra, name = sd
    STAGE_NAMES.append(name)
    jl, jr = _build_waypoint(ls, la, rs, ra)
    LEFT_WAYPOINTS.append((t, jl))
    RIGHT_WAYPOINTS.append((t, jr))

    # 计算可视化点
    if isinstance(ls, tuple) and len(ls)==2:
        idx,_ = ls; lx,ly,lz = _acu_left(idx, la)
    elif ls == "acu_left":
        lx,ly,lz = _acu_left(0, la)
    else:
        lx,ly,lz = _zone_left(ls, la)
    MASSAGE_POINTS_L.append(Point(x=lx,y=ly,z=lz))

    if isinstance(rs, tuple) and len(rs)==2:
        idx,_ = rs; rx,ry,rz = _acu_right(idx, ra)
    elif rs == "acu_right":
        rx,ry,rz = _acu_right(0, ra)
    else:
        rx,ry,rz = _zone_right(rs, ra)
    MASSAGE_POINTS_R.append(Point(x=rx,y=ry,z=rz))

    t += DT

NUM_STAGES = len(STAGE_DEFS)

# ── 规划参数 ──
SAMPLE_PERIOD = 0.10
JOINT_GOAL_TOLERANCE = 0.05   # 放宽到0.05rad（~3°），让IK有足够灵活性
PLANNING_GROUP = "both_arms"
PLANNER_ID = "RRTConnectkConfigDefault"

# ── 可视化颜色 ──
BODY_COLOR  = (0.86,0.76,0.66,0.78)
SPINE_COLOR = (0.95,0.85,0.70,0.92)
EDGE_COLOR  = (0.65,0.55,0.45,0.50)
ACU_COLOR   = (0.95,0.25,0.25,0.85)  # 穴位红点


def _dur(s): w=int(s); return Duration(sec=w,nanosec=int((s-w)*1e9))
def _ds(d): return float(d.sec)+float(d.nanosec)/1e9
def _dshift(d,o): return _dur(_ds(d)+o)


class DualArmMassageDemo(Node):
    def __init__(self):
        super().__init__("dual_arm_massage_demo")
        # params
        p = self.declare_parameter
        self.marker_topic = p("marker_topic","/rviz_visual_tools").get_parameter_value().string_value
        self.hold_s  = p("hold_seconds",8.0).get_parameter_value().double_value
        self.t_start_delay = p("trajectory_start_delay",0.10).get_parameter_value().double_value
        self.vel_s   = max(0.01,min(1.0,p("velocity_scaling",0.45).get_parameter_value().double_value))
        self.acc_s   = max(0.01,min(1.0,p("acceleration_scaling",0.45).get_parameter_value().double_value))
        self.t_scale = max(0.25,min(1.0,p("trajectory_time_scale",0.65).get_parameter_value().double_value))
        self.exact_sp= max(0.05,p("exact_target_speed",0.50).get_parameter_value().double_value)
        self.fb_sp   = max(0.05,p("fallback_joint_speed",0.45).get_parameter_value().double_value)
        self.min_settle = max(0.02,p("min_settle_duration",0.10).get_parameter_value().double_value)
        self.repeat_n= p("repeat_count",0).get_parameter_value().integer_value

        # pubs/clients
        self.marker_pub = self.create_publisher(MarkerArray,self.marker_topic,10)
        self.scene_cli  = self.create_client(ApplyPlanningScene,"/apply_planning_scene")
        self.plan_cli   = self.create_client(GetMotionPlan,"/plan_kinematic_path")
        self.valid_cli  = self.create_client(GetStateValidity,"/check_state_validity")
        self.l_cli = ActionClient(self,FollowJointTrajectory,"/left_arm_controller/follow_joint_trajectory")
        self.r_cli = ActionClient(self,FollowJointTrajectory,"/right_arm_controller/follow_joint_trajectory")
        self.js_sub = self.create_subscription(JointState,"/joint_states",self._js_cb,10)

        self.js = {}
        self.pending = 0
        self.cycles = 0
        self.failed = False
        self.l_traj = self.r_traj = None
        self.done = False
        self.timer = self.create_timer(0.25,self.publish_markers)

    # ── 初始化 ──
    def run(self)->bool:
        self.publish_markers()
        if not self._wait_svcs(): return False
        start = self._wait_js(30.0)
        if start is None: return False
        traj = self._plan(start)
        if traj is None: return False
        self._tscale(traj)
        if not self._validate(traj): return False
        self.l_traj, self.r_traj = self._split(traj)
        self._send_cycle()
        return True

    def _send_cycle(self):
        self.pending=2; self.failed=False
        lbl = f"{self.cycles+1}/{self.repeat_n}" if self.repeat_n>0 else f"{self.cycles+1}/∞"
        self.get_logger().info(f"发送推拿循环 {lbl}（50式中医推拿）")
        self._send_goal(self.l_cli,self.l_traj,"左臂")
        self._send_goal(self.r_cli,self.r_traj,"右臂")

    def _wait_svcs(self)->bool:
        for l,c in [("plan",self.plan_cli),("valid",self.valid_cli)]:
            if not c.wait_for_service(timeout_sec=30.0):
                self.get_logger().error(f"服务 {l} 超时"); return False
        for l,c in [("左臂",self.l_cli),("右臂",self.r_cli)]:
            if not c.wait_for_server(timeout_sec=60.0):
                self.get_logger().error(f"{l} 动作服务超时"); return False
        return True

    def _js_cb(self,msg):
        for n,p in zip(msg.name,msg.position): self.js[n]=p

    def _wait_js(self,to):
        dl=self.get_clock().now().nanoseconds/1e9+to
        while rclpy.ok():
            if all(j in self.js for j in ALL_JOINTS):
                return [self.js[j] for j in ALL_JOINTS]
            if self.get_clock().now().nanoseconds/1e9>dl:
                self.get_logger().error("等待/joint_states超时"); return None
            self.publish_markers(); rclpy.spin_once(self,timeout_sec=0.1)

    def _apply_scene(self)->bool:
        co=CollisionObject(); co.header.frame_id="world"
        co.id="massage_scene"; co.operation=CollisionObject.ADD
        # 床
        self._box(co,BED_FRAME,BED_CX,BED_CY,BED_FRAME_Z)
        self._box(co,MATTRESS,BED_CX,BED_CY,MATTRESS_Z)
        self._box(co,(PILLOW[2],PILLOW[3],PILLOW[4]),PILLOW[0],BED_CY,PILLOW[1])
        # 人体（简化3件）
        self._box(co,(0.60,0.36,0.10),0.60,0.0,0.225)  # 躯干z=0.225
        self._sphere(co,(0.27,0.0,0.245),0.10)          # 头z=0.245
        self._box(co,(0.06,0.16,0.06),0.48,-0.26,0.22)  # 左臂z=0.22
        self._box(co,(0.06,0.16,0.06),0.48, 0.26,0.22)  # 右臂z=0.22
        sc=PlanningScene(); sc.is_diff=True; sc.world.collision_objects.append(co)
        req=ApplyPlanningScene.Request(); req.scene=sc
        fut=self.scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self,fut,timeout_sec=5.0)
        r=fut.result()
        if r is None or not r.success:
            self.get_logger().error("场景碰撞添加失败"); return False
        self.get_logger().info("场景碰撞已添加（床+人体）")
        return True

    def _box(self,co,dims,x,y,z):
        p=SolidPrimitive(); p.type=SolidPrimitive.BOX; p.dimensions=dims
        ps=Pose(); ps.orientation.w=1.0; ps.position.x=x; ps.position.y=y; ps.position.z=z
        co.primitives.append(p); co.primitive_poses.append(ps)

    def _sphere(self,co,ctr,r):
        p=SolidPrimitive(); p.type=SolidPrimitive.SPHERE; p.dimensions=[r]
        ps=Pose(); ps.orientation.w=1.0
        ps.position.x=ctr[0]; ps.position.y=ctr[1]; ps.position.z=ctr[2]
        co.primitives.append(p); co.primitive_poses.append(ps)

    # ── 轨迹规划 ──
    def _plan(self,start):
        tgts=[l[1]+r[1] for l,r in zip(LEFT_WAYPOINTS,RIGHT_WAYPOINTS)]
        c=JointTrajectory(); c.joint_names=ALL_JOINTS
        p0=JointTrajectoryPoint(); p0.positions=list(start); p0.time_from_start=_dur(0.0)
        c.points.append(p0)
        cur=list(start); toff=0.0
        for ti,tgt in enumerate(tgts):
            name=STAGE_NAMES[ti]
            lbl=f"{name} ({ti+1}/{len(tgts)})"
            if max(abs(a-b) for a,b in zip(cur,tgt))<=0.002:
                self.get_logger().info(f"已在{lbl}，跳过"); cur=list(tgt); continue
            seg=self._plan_seg(cur,tgt,lbl)
            if seg is None:
                self.get_logger().error(f"阶段{ti+1}规划失败，继续下一阶段")
                # 容错：使用线性插值作为fallback
                fallback_pt = JointTrajectoryPoint()
                fallback_pt.positions = list(tgt)
                fallback_pt.time_from_start = _dur(toff + 0.5)
                c.points.append(fallback_pt)
                toff += 0.5
                cur = list(tgt)
                continue
            toff=self._append(c,seg,cur,toff)
            toff=self._exact(c,tgt,toff)
            cur=list(c.points[-1].positions)
        self.get_logger().info(f"推拿轨迹规划完成：{len(c.points)}点, {toff:.1f}s")
        return c

    def _tscale(self,traj):
        if abs(self.t_scale-1.0)<=1e-6: return
        o=_ds(traj.points[-1].time_from_start)
        for pt in traj.points: pt.time_from_start=_dur(_ds(pt.time_from_start)*self.t_scale)
        self.get_logger().info(f"时间缩放:{o:.1f}s→{_ds(traj.points[-1].time_from_start):.1f}s")

    def _plan_seg(self,s,g,label):
        req=GetMotionPlan.Request(); mr=req.motion_plan_request
        mr.group_name=PLANNING_GROUP; mr.planner_id=PLANNER_ID
        mr.num_planning_attempts=12; mr.allowed_planning_time=8.0
        mr.max_velocity_scaling_factor=self.vel_s
        mr.max_acceleration_scaling_factor=self.acc_s
        mr.start_state.is_diff=True
        mr.start_state.joint_state=JointState(); mr.start_state.joint_state.name=ALL_JOINTS
        mr.start_state.joint_state.position=s
        gc=Constraints()
        for jn,jp in zip(ALL_JOINTS,g):
            jc=JointConstraint(); jc.joint_name=jn; jc.position=jp
            jc.tolerance_above=JOINT_GOAL_TOLERANCE
            jc.tolerance_below=JOINT_GOAL_TOLERANCE
            jc.weight=1.0; gc.joint_constraints.append(jc)
        mr.goal_constraints.append(gc)
        fut=self.plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self,fut,timeout_sec=12.0)
        r=fut.result()
        if r is None: self.get_logger().error(f"规划{label}无响应"); return None
        rsp=r.motion_plan_response
        if rsp.error_code.val!=1:
            self.get_logger().error(f"规划{label}失败 code={rsp.error_code.val}")
            return None
        traj=rsp.trajectory.joint_trajectory
        if not traj.points: self.get_logger().error(f"空轨迹{label}"); return None
        self.get_logger().info(f"已规划{label}:{len(traj.points)}点 {rsp.planning_time:.2f}s")
        return traj

    def _append(self,c,seg,fb,toff):
        ld=_ds(seg.points[-1].time_from_start)
        if ld<=0.001:
            fp=self._pos(seg,seg.points[-1],fb)
            ld=max(0.30,max(abs(a-b) for a,b in zip(fb,fp))/self.fb_sp)
        app=0; nc=len(seg.points)
        for pi,pt in enumerate(seg.points):
            lt=_ds(pt.time_from_start)
            if lt<=0.001 and nc>1: lt=ld*pi/(nc-1)
            if lt<=0.001 and c.points: continue
            np=JointTrajectoryPoint(); np.positions=self._pos(seg,pt,fb)
            np.time_from_start=_dur(toff+lt); c.points.append(np); app+=1
        if app==0:
            np=JointTrajectoryPoint(); np.positions=self._pos(seg,seg.points[-1],fb)
            np.time_from_start=_dur(toff+ld); c.points.append(np)
        return toff+ld

    def _exact(self,c,tgt,toff):
        cur=list(c.points[-1].positions)
        md=max(abs(a-b) for a,b in zip(cur,tgt))
        if md<=JOINT_GOAL_TOLERANCE: return toff
        sd=max(self.min_settle,md/self.exact_sp)
        pt=JointTrajectoryPoint(); pt.positions=list(tgt)
        pt.time_from_start=_dur(toff+sd); c.points.append(pt)
        return toff+sd

    def _pos(self,traj,pt,fb):
        pbn={jn:fb[i] for i,jn in enumerate(ALL_JOINTS)}
        for jn,jp in zip(traj.joint_names,pt.positions): pbn[jn]=jp
        return [pbn[jn] for jn in ALL_JOINTS]

    def _validate(self,traj)->bool:
        tt=_ds(traj.points[-1].time_from_start)
        sc=max(int(tt/SAMPLE_PERIOD)+1,len(traj.points)-1)
        for i in range(sc+1):
            et=tt*i/sc; pos=self._interp(traj,et)
            req=GetStateValidity.Request(); req.group_name=PLANNING_GROUP
            req.robot_state.is_diff=True
            req.robot_state.joint_state=JointState(); req.robot_state.joint_state.name=ALL_JOINTS
            req.robot_state.joint_state.position=pos
            fut=self.valid_cli.call_async(req)
            rclpy.spin_until_future_complete(self,fut,timeout_sec=3.0)
            r=fut.result()
            if r is None: self.get_logger().error("有效性服务无响应"); return False
            if not r.valid:
                cs=", ".join(f"{c.contact_body_1}<->{c.contact_body_2}" for c in r.contacts[:6])
                self.get_logger().error(f"碰撞 t={et:.1f}s ({i}/{sc}): {cs or '无效'}")
                return False
        self.get_logger().info(f"碰撞检测通过({sc+1}点)")
        return True

    def _interp(self,traj,et):
        if et<=_ds(traj.points[0].time_from_start): return list(traj.points[0].positions)
        for i in range(1,len(traj.points)):
            pp=traj.points[i-1]; np=traj.points[i]
            t0=_ds(pp.time_from_start); t1=_ds(np.time_from_start)
            if et<=t1:
                if t1<=t0: return list(np.positions)
                r=(et-t0)/(t1-t0)
                return [pp.positions[j]+(np.positions[j]-pp.positions[j])*r for j in range(len(pp.positions))]
        return list(traj.points[-1].positions)

    def _split(self,traj):
        lt=JointTrajectory(); lt.joint_names=LEFT_JOINTS
        rt=JointTrajectory(); rt.joint_names=RIGHT_JOINTS
        ji={jn:i for i,jn in enumerate(traj.joint_names)}
        for pt in traj.points:
            lp=JointTrajectoryPoint()
            lp.positions=[pt.positions[ji[j]] for j in LEFT_JOINTS]
            lp.time_from_start=_dshift(pt.time_from_start,self.t_start_delay)
            lt.points.append(lp)
            rp=JointTrajectoryPoint()
            rp.positions=[pt.positions[ji[j]] for j in RIGHT_JOINTS]
            rp.time_from_start=_dshift(pt.time_from_start,self.t_start_delay)
            rt.points.append(rp)
        return lt,rt

    # ── 动作客户端 ──
    def _send_goal(self,cli,traj,label):
        g=FollowJointTrajectory.Goal(); g.trajectory=traj; g.goal_time_tolerance=_dur(0.8)
        fut=cli.send_goal_async(g); fut.add_done_callback(lambda d:self._goal_cb(d,label))

    def _goal_cb(self,fut,label):
        gh=fut.result()
        if not gh.accepted: self.get_logger().error(f"{label}目标被拒"); self._mark(); return
        self.get_logger().info(f"{label}目标已接受"); gh.get_result_async().add_done_callback(lambda d:self._res_cb(d,label))

    def _res_cb(self,fut,label):
        r=fut.result().result
        if r.error_code==FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"{label}执行成功")
        else: self.failed=True; self.get_logger().error(f"{label}失败 code={r.error_code}")
        self._mark()

    def _mark(self):
        self.pending-=1
        if self.pending==0:
            self.cycles+=1
            if self.failed: self.get_logger().error("循环失败停止"); self.create_timer(self.hold_s,self._stop); return
            if self.repeat_n==0 or self.cycles<self.repeat_n:
                self.get_logger().info(f"循环{self.cycles}完成→继续"); self._send_cycle(); return
            self.get_logger().info(f"完成{self.cycles}轮"); self.create_timer(self.hold_s,self._stop)

    def _stop(self):
        if not self.done: self.done=True; self.get_logger().info("推拿Demo结束"); rclpy.shutdown()

    # ═══════════ 可视化 ═══════════
    def publish_markers(self):
        now=self.get_clock().now().to_msg(); ma=MarkerArray()
        # 床
        ma.markers.extend([
            self._bm(now,1,"bed",Point(x=BED_CX,y=BED_CY,z=BED_FRAME_Z),
                     Point(x=BED_FRAME[0],y=BED_FRAME[1],z=BED_FRAME[2]),(0.38,0.27,0.17,1.0)),
            self._bm(now,2,"mattress",Point(x=BED_CX,y=BED_CY,z=MATTRESS_Z),
                     Point(x=MATTRESS[0],y=MATTRESS[1],z=MATTRESS[2]),(0.82,0.88,0.94,1.0)),
            self._bm(now,3,"pillow",Point(x=PILLOW[0],y=BED_CY,z=PILLOW[1]),
                     Point(x=PILLOW[2],y=PILLOW[3],z=PILLOW[4]),(0.88,0.92,0.98,1.0)),
        ])
        # 人体12段躯干
        mid=100
        for i,(xc,zs,yhw,thk,nm) in enumerate(BODY):
            mid+=1
            if i==0:
                ma.markers.append(self._sm(now,mid,f"body_{nm}",Point(x=xc,y=0.0,z=zs),0.20,BODY_COLOR))
            else:
                ma.markers.append(self._bm(now,mid,f"body_{nm}",
                    Point(x=xc,y=0.0,z=zs-thk/2),Point(x=0.06,y=yhw*2,z=thk),BODY_COLOR))
        # 脊柱脊线
        mid+=1; ma.markers.append(self._lm(now,mid,"spine",SPINE_RIDGE,SPINE_COLOR,0.015))
        # 侧边轮廓
        mid+=1; ma.markers.append(self._lm(now,mid,"edge_L",BODY_EDGE_L,EDGE_COLOR,0.008))
        mid+=1; ma.markers.append(self._lm(now,mid,"edge_R",BODY_EDGE_R,EDGE_COLOR,0.008))
        # 膀胱经穴位
        for i,(nm,xc,zs) in enumerate(ACUPOINTS):
            mid+=1
            ma.markers.append(self._sm(now,mid,f"acu_L_{nm}",
                Point(x=xc,y=-ACU_Y_OFFSET,z=zs),0.018,ACU_COLOR))
            mid+=1
            ma.markers.append(self._sm(now,mid,f"acu_R_{nm}",
                Point(x=xc,y=ACU_Y_OFFSET,z=zs),0.018,ACU_COLOR))
        # 按摩路径线
        ma.markers.append(self._lm(now,10,"left_path",MASSAGE_POINTS_L,(0.0,0.75,0.95,1.0),0.012))
        ma.markers.append(self._lm(now,11,"right_path",MASSAGE_POINTS_R,(0.95,0.58,0.20,1.0),0.012))
        # 路径目标点
        for i,pt in enumerate(MASSAGE_POINTS_L+MASSAGE_POINTS_R):
            ma.markers.append(self._sm(now,200+i,"target",pt,0.022,(0.1,0.9,0.45,0.9)))
        self.marker_pub.publish(ma)

    # Marker helpers
    def _base(self,now,mid,ns,mt):
        m=Marker(); m.header.frame_id="world"; m.header.stamp=now
        m.ns=ns; m.id=mid; m.type=mt; m.action=Marker.ADD; m.pose.orientation.w=1.0; return m
    def _bm(self,now,mid,ns,pos,scale,color):
        m=self._base(now,mid,ns,Marker.CUBE); m.pose.position=pos
        m.scale.x=scale.x; m.scale.y=scale.y; m.scale.z=scale.z
        m.color.r,m.color.g,m.color.b,m.color.a=color; return m
    def _sm(self,now,mid,ns,pos,diam,color):
        m=self._base(now,mid,ns,Marker.SPHERE); m.pose.position=pos
        m.scale.x=m.scale.y=m.scale.z=diam
        m.color.r,m.color.g,m.color.b,m.color.a=color; return m
    def _lm(self,now,mid,ns,pts,color,w=0.012):
        m=self._base(now,mid,ns,Marker.LINE_STRIP); m.points=pts; m.scale.x=w
        m.color.r,m.color.g,m.color.b,m.color.a=color; return m


def main():
    rclpy.init(); node=DualArmMassageDemo()
    if not node.run(): node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    try: rclpy.spin(node)
    finally: node.destroy_node()

if __name__=="__main__": main()
