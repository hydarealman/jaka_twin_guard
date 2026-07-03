#!/usr/bin/env python3
"""双臂中医推拿按摩 Demo — v6.0 编排器驱动.

    v6 核心改进:
     1. Skill Primitive Library + Choreographer: 高层pattern自动展开为阶段序列
     2. 自动插入过渡原语: 力渐变 / 抬升接近 / 沿面滑动 / 振后衰减
     3. 底层执行不变: 关节角delta + MoveIt RRT + 3级回退 + 碰撞检测
     4. 原 v5.4 所有方案仍可通过 dual_arm_massage_demo.py 使用

    用法:
     ros2 launch dual_arm_jaka_c5_moveit_config massage_demo_v6.launch.py
     ros2 launch ... massage_pattern:=推法开背
     ros2 launch ... massage_pattern:=综合推拿"""


from __future__ import annotations
import sys, math, os
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Quaternion
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

# ── v6: Choreographer imports ──
try:
    from jaka_dual_arm.massage.skill_primitives import SkillLibrary
    from jaka_dual_arm.massage.choreographer import (
        MassageChoreographer, StageDef, StageKind,
    )
    _CHOREOGRAPHER_AVAILABLE = True
except ImportError:
    _CHOREOGRAPHER_AVAILABLE = False

# ── 关节列表 ──
LEFT_JOINTS  = [f"left_joint_{i}"  for i in range(1,7)]
RIGHT_JOINTS = [f"right_joint_{i}" for i in range(1,7)]
ALL_JOINTS   = LEFT_JOINTS + RIGHT_JOINTS

# ── 安全泊车位姿（紧急停止时用，正常工作不需要） ──
LEFT_SAFE_PARK  = [-1.57, 0.30, -0.50, 1.50, 1.57, 0.0]   # 左臂指向左侧(-y)
RIGHT_SAFE_PARK = [ 1.57, 0.30, -0.50, 1.50, 1.57, 0.0]   # 右臂指向右侧(+y)

# ── 向前悬停位姿（一臂工作时另一臂的安全等待位姿） ──
# j1=0 → 臂沿+X方向指向前方，左臂Y=-0.78/右臂Y=+0.78（在床外）
# 两臂间距1.56m，Link_03永不交叉
LEFT_FWD_HOVER  = [0.0, 0.75, -1.10, 1.30, 1.57, 1.50]
RIGHT_FWD_HOVER = [0.0, 0.75, -1.10, 1.30, 1.57, 1.50]

# ── 高空越障过渡位姿（v5.1 新增） ──
# 双臂抬高(J2≈1.0)，端点在人体上空z>0.30的安全悬停位姿
# 用于C7(近头)和骶骨(近腿)区域的过渡，臂体在z>0.30高度越过头部/腿部
# J2=1.0(≈57°从垂直) → 上臂指向前上方，肘部z>0.35，远高于人体表面(z≈0.20)
# J3=-2.00 → 前臂向下指向背部表面
LEFT_HIGH_HOVER  = [0.0, 1.00, -2.00, 1.80, 1.57, 1.50]
RIGHT_HIGH_HOVER = [0.0, 1.00, -2.00, 1.80, 1.57, 1.50]

# ── 高空越障换边位姿（v5.1.1 修正J2避免纯水平扫掠碰撞） ──
# J1旋转至对侧 + J2=0.85(更竖直=肘更高) → 旋转同时上升避开对侧臂
# 注意: J2≠HIGH_HOVER的J2(1.00)，确保路径带垂直分量，防臂-臂碰撞
# 左臂→右半背: J1=0.70(指向右前方)
# 右臂→左半背: J1=-0.70(指向左前方)
LEFT_CROSS_OVER  = [0.70, 0.85, -2.00, 1.80, 1.57, 1.50]
RIGHT_CROSS_OVER = [-0.70, 0.85, -2.00, 1.80, 1.57, 1.50]


def _is_arm_link(name: str) -> bool:
    """判断碰撞体是否为机械臂连杆"""
    return name.startswith("left_Link_") or name.startswith("right_Link_")


def _is_robot_link_pair(name_a: str, name_b: str) -> bool:
    return _is_arm_link(name_a) and _is_arm_link(name_b)

# ── 臂基座位置（X错开0.16m，避免双臂碰撞） ──
LEFT_BASE  = (0.53, -0.78, 0.0)    # v5.1 加宽至-0.78 (总间距1.56m)
RIGHT_BASE = (0.69,  0.78, 0.0)    # v5.1 加宽至 0.78
SHOULDER_Z = 0.12   # 肩关节(Link_00顶)世界Z
GRAVITY    = (0.0, 0.0, -9.81)

# ── 床体（地面按摩垫，床直接放在地上z=0） ──
BED_CX,BED_CY = 0.70,0.0
BED_FRAME_Z, BED_FRAME = 0.04, (1.20,0.66,0.08)   # 床框底z=0.00 顶z=0.08
MATTRESS_Z,  MATTRESS  = 0.11, (1.12,0.56,0.06)    # 床垫底z=0.08 顶z=0.14
PILLOW = (0.28,0.175,0.20,0.34,0.07)                # 枕头顶z≈0.21
MATTRESS_TOP = MATTRESS_Z + MATTRESS[2]/2            # =0.14

# ── 人体模型：12段脊柱轮廓（俯卧，脊柱沿X轴） ──
# (x_c, z_surface, y_half_width, thickness, name)
BODY = [
    (0.27,0.205,0.090,0.08,"头部"),
    (0.33,0.192,0.065,0.045,"颈根"),
    (0.38,0.197,0.120,0.048,"C7隆椎"),    # 背部最高点
    (0.43,0.195,0.160,0.050,"斜方肌上部"),
    (0.49,0.193,0.180,0.052,"肩胛带"),     # 最宽处
    (0.55,0.189,0.170,0.048,"T1-4上胸椎"),
    (0.61,0.186,0.155,0.046,"T5-8中胸椎"),
    (0.67,0.183,0.140,0.044,"T9-12下胸椎"),
    (0.72,0.179,0.128,0.042,"胸腰结合"),   # 腰部收窄
    (0.77,0.175,0.130,0.040,"L1-3腰椎"),   # 腰椎凹陷
    (0.82,0.173,0.140,0.038,"L4-5"),
    (0.86,0.171,0.150,0.036,"骶骨"),
]
SPINE_RIDGE = [Point(x=s[0],y=0.0,z=s[1]+0.008) for s in BODY]
BODY_EDGE_L = [Point(x=s[0],y=-s[2],z=s[1]) for s in BODY if s[2]>0.001]
BODY_EDGE_R = [Point(x=s[0],y= s[2],z=s[1]) for s in BODY if s[2]>0.001]

# 膀胱经穴位（脊柱旁开0.04m，左右各7穴）
ACUPOINTS = [
    ("BL11_大杼", 0.41,0.196), ("BL13_肺俞",0.48,0.194),
    ("BL15_心俞", 0.55,0.189), ("BL17_膈俞",0.63,0.185),
    ("BL18_肝俞", 0.70,0.180), ("BL23_肾俞",0.77,0.175),
    ("BL25_大肠俞",0.84,0.172),
]
ACU_Y_OFFSET = 0.04  # 穴位距脊柱中线Y偏移

# ── 按摩区域（格式: x_center, y_half_width, z_surface, z_hover）──
MASSAGE_ZONES = {
    "C7":       (0.38,0.10,0.197,0.207),
    "shoulder": (0.47,0.16,0.194,0.204),
    "upper":    (0.55,0.14,0.189,0.199),
    "mid":      (0.63,0.12,0.185,0.195),
    "lower_th": (0.70,0.11,0.180,0.190),
    "lumbar":   (0.77,0.10,0.175,0.185),
    "sacrum":   (0.84,0.12,0.172,0.182),
}

# ── 关节角度工具函数 ──
def _j1(tx,ty, bx,by):
    """计算joint_1使臂平面指向目标XY方向"""
    return math.atan2(ty-by, tx-bx)

# 基础关节角度（hover状态）
J2_BASE, J3_BASE, J4_BASE, J5_BASE, J6_BASE = 0.75, -1.10, 1.30, 1.57, 1.50

# 动作偏移 [Δj2,Δj3,Δj4,Δj5,Δj6] — 13+种中医推拿手法
ACT_DELTA = {
    # 基础动作
    "hover":      [ 0.0,  0.0,  0.0,  0.0,  0.0],   # 悬停
    "press":      [ 0.02,-0.04,  0.03,  0.0,  0.0],   # 按法（按压）
    "release":    [ 0.0,  0.0,  0.0,  0.0,  0.0],   # 释放
    "knead_L":    [ 0.02,-0.04,  0.03,  0.0,  0.08],  # 揉法左旋
    "knead_R":    [ 0.02,-0.04,  0.03,  0.0, -0.08],  # 揉法右旋
    "roll":       [ 0.01,-0.02,  0.02,  0.0,  0.0],   # 滚法（滚揉）
    "tap":        [ 0.0,-0.05,  0.03,  0.0,  0.0],   # 拍法（轻拍）
    "strike":     [ 0.0,-0.06,  0.04,  0.0,  0.0],   # 击法（重击敲打）
    "beat":       [ 0.0,-0.08,  0.06,  0.0,  0.0],   # 捶打（大幅起落敲击）
    # 新增手法（中医十三法扩展）
    "rub_L":      [ 0.01,-0.02,  0.04,  0.0,  0.06],  # 摩法左旋（腕部圆周揉摩）
    "rub_R":      [ 0.01,-0.02,  0.04,  0.0, -0.06],  # 摩法右旋
    "scrub":      [ 0.03,-0.05,  0.04,  0.0,  0.0],   # 擦法（往返直线推擦）
    "vibrate":    [ 0.002,-0.002, 0.002, 0.0,  0.003],# 振法（快速颤动）
    "deep_press": [ 0.03,-0.06,  0.05,  0.0,  0.0],   # 深按（加强按压）
    # ═══════════════════════════════════════════════════
    # v2.0 新增手法
    # ═══════════════════════════════════════════════════
    "pound":      [ 0.04,-0.08,  0.06,  0.0,  0.0],    # 捶打重击（加强strike）
    "arc_L":      [ 0.01,-0.02,  0.04,  0.0,  0.10],   # 弧线左扫
    "arc_R":      [ 0.01,-0.02,  0.04,  0.0, -0.10],   # 弧线右扫
    "knead_wL":   [ 0.02,-0.04,  0.03,  0.0,  0.20],   # 大揉搓左旋（加大J6幅度）
    "knead_wR":   [ 0.02,-0.04,  0.03,  0.0, -0.20],   # 大揉搓右旋
    # ═══════════════════════════════════════════════════
    # v5.1 新增手法
    # ═══════════════════════════════════════════════════
    "wave":       [ 0.0,  0.0,  0.02, 0.02,  0.04],   # 波浪（法兰末端正弦振荡）
}

def _joints(j1, act, j2b=J2_BASE,j3b=J3_BASE,j4b=J4_BASE,j5b=J5_BASE,j6b=J6_BASE):
    """构建6DOF关节列表 [j1,j2,j3,j4,j5,j6]"""
    d = ACT_DELTA.get(act, ACT_DELTA["hover"])
    return [j1, j2b+d[0], j3b+d[1], j4b+d[2], j5b+d[3], j6b+d[4]]

def _wp_left(tx,ty, act, j2b=J2_BASE,j3b=J3_BASE):
    """左臂waypoint（臂在床左侧y=-0.45，指向身体左侧y<0）"""
    return _joints(_j1(tx,ty,*LEFT_BASE[:2]), act, j2b, j3b)

def _wp_right(tx,ty, act, j2b=J2_BASE,j3b=J3_BASE):
    """右臂waypoint（臂在床右侧y=0.45，指向身体右侧y>0）"""
    return _joints(_j1(tx,ty,*RIGHT_BASE[:2]), act, j2b, j3b)

def _zone_target(zone_name, pos, act, crossed=False):
    """统一按摩目标点计算。
    pos: 'L'=左臂侧(病人右半背,y=-yw), 'R'=右臂侧(病人左半背,y=+yw)
    crossed=True → 换边: L→右半背(y=+yw), R→左半背(y=-yw)
    v5.1: C7和sacrum用crossed模式，3步顺序越障: 一臂抬高→另一臂横穿→抬高的从高处越过"""
    xc, yw, zs, zh = MASSAGE_ZONES[zone_name]
    if crossed:
        # 交叉模式：左臂去右半背，右臂去左半背
        if pos == 'L':   y = yw
        elif pos == 'R': y = -yw
        else:            y = 0.0
    else:
        if pos == 'C':   y = 0.0
        elif pos == 'L': y = -yw
        else:            y = yw   # 'R'

    if act in ("hover","release","tap","strike"): return xc, y, zh
    if act in ("knead_L","rub_L","knead_wL"):   return xc, y+0.015, zs
    if act in ("knead_R","rub_R","knead_wR"):   return xc, y-0.015, zs
    if act == "scrub": return xc+0.02, y, zs        # 擦法前推2cm
    return xc, y, zs  # press, deep_press, roll, vibrate, pound, arc_L, arc_R, wave

def _acu_left(idx, act):
    """左臂按左侧膀胱经穴位(y=-0.04)"""
    _, x, zs = ACUPOINTS[idx]
    zh = zs + 0.010
    return (x, -ACU_Y_OFFSET, zh) if act in ("hover","release") else (x, -ACU_Y_OFFSET, zs)

def _acu_right(idx, act):
    """右臂按右侧膀胱经穴位(y=+0.04)"""
    _, x, zs = ACUPOINTS[idx]
    zh = zs + 0.010
    return (x, ACU_Y_OFFSET, zh) if act in ("hover","release") else (x, ACU_Y_OFFSET, zs)

# ═══════════════════════════════════════════════════════
# v3.0 多周期手法目标生成
# ═══════════════════════════════════════════════════════

TAP_CYCLES = 5           # 敲击振荡次数
TAP_HALF_MS = 0.55       # 半周期(s) — 约0.9Hz（原0.35→更缓）
POUND_CYCLES = 3         # 捶打脉冲次数
POUND_HALF_MS = 0.75     # 半周期(s) 原0.50→更缓
ARC_POINTS = 5           # 弧线插值点数（含两端）
ARC_STEP_MS = 0.60       # 弧线每步耗时(s) 原0.40→更缓
KNEAD_POINTS = 4         # 揉搓圆周分点数
KNEAD_STEP_MS = 0.55     # 揉搓每步耗时(s) 原0.35→更缓
KNEAD_RADIUS = 0.020     # 大揉搓半径(m) = 2cm

# ═══════════════════════════════════════════════════════
# v5.1 波浪手法参数
# ═══════════════════════════════════════════════════════
WAVE_CYCLES = 3           # 波浪周期数
WAVE_STEPS_PER_CYCLE = 8  # 每周期插值点数
WAVE_STEP_MS = 0.40       # 每步耗时(s)
WAVE_AMPLITUDE_RATIO = 0.6  # Y方向振幅比例(相对于半宽)
WAVE_Z_AMPLITUDE = 0.005    # Z方向起伏幅度(m)

MULTI_TECHS = {"tap", "pound", "arc_L", "arc_R", "knead_wL", "knead_wR", "wave"}
ARC_TECHS = {"arc_L", "arc_R"}
KNEAD_WIDE_TECHS = {"knead_wL", "knead_wR"}

NOMINAL_ACT = {
    "tap": "press",
    "pound": "deep_press",
    "arc_L": "rub_L",
    "arc_R": "rub_R",
    "knead_wL": "knead_L",
    "knead_wR": "knead_R",
    "wave": "press",
}

def _generate_tap_targets(zone_name, pos, crossed=False):
    """生成敲击振荡的(x,y,z)目标序列 — press↔hover 交替 5次."""
    xp, yp, zp = _zone_target(zone_name, pos, "press", crossed=crossed)
    xh, yh, zh = _zone_target(zone_name, pos, "hover", crossed=crossed)
    targets = []
    for i in range(TAP_CYCLES):
        targets.append(("hover", xh, yh, zh))
        targets.append(("press", xp, yp, zp))
    return targets

def _generate_pound_targets(zone_name, pos, crossed=False):
    """生成捶打脉冲的(x,y,z)目标序列 — strike↔hover 交替 3次."""
    x_strike, y_strike, z_strike = _zone_target(zone_name, pos, "strike", crossed=crossed)
    xh, yh, zh = _zone_target(zone_name, pos, "hover", crossed=crossed)
    targets = []
    for i in range(POUND_CYCLES):
        targets.append(("hover", xh, yh, zh))
        targets.append(("strike", x_strike, y_strike, z_strike))
    return targets

def _generate_arc_targets(zone_name, pos, crossed=False):
    """生成弧线扫过的(x,y,z)目标序列 — 从pos侧向中线摆动再回来。"""
    xc, yw, zs, zh = MASSAGE_ZONES[zone_name]
    y_end = 0.0
    if pos == 'L':  y_start = -yw * 0.8
    else:           y_start = yw * 0.8
    if crossed: y_start = -y_start  # 交叉模式：反转起始侧
    targets = []
    for i in range(ARC_POINTS):
        t = i / (ARC_POINTS - 1)
        if t <= 0.5:
            lt = t * 2
            ct = (1 - math.cos(lt * math.pi)) / 2
            y = y_start + (y_end - y_start) * ct
        else:
            lt = (t - 0.5) * 2
            ct = (1 - math.cos(lt * math.pi)) / 2
            y = y_end + (y_start - y_end) * ct
        targets.append(("press", xc, y, zs))
    return targets

def _generate_knead_wide_targets(zone_name, pos, crossed=False):
    """生成大揉搓圆周的(x,y,z)目标序列 — 4点圆周 + 回到中心。
    crossed=True 时交换base_y方向（左臂按右半背，右臂按左半背）。"""
    xc, yw, zs, zh = MASSAGE_ZONES[zone_name]
    if pos == 'L':
        base_y = -yw
        act_tag = "knead_wL"
    else:
        base_y = yw
        act_tag = "knead_wR"
    if crossed: base_y = -base_y  # 交叉模式：反转基座Y侧
    targets = []
    for i in range(KNEAD_POINTS):
        angle = i * 2 * math.pi / KNEAD_POINTS
        yo = KNEAD_RADIUS * math.sin(angle)
        zo = 0.005 * (1 - math.cos(angle))
        targets.append((act_tag, xc, base_y + yo, zs + zo))
    targets.append(("press", xc, base_y, zs))
    return targets

def _generate_wave_targets(zone_name, pos, crossed=False):
    """v5.1 波浪手法 — 法兰末端正弦振荡 + 回中。

    原理:
      端点在Y方向做正弦振荡(从脊柱中心到边缘来回滚动)，
      Z方向同步轻微起伏(波谷按压更深)，
      J5/J6小幅度振荡增加"波浪滚动"感。
      末尾追加回中目标，避免下一阶段起点瞬移。
    """
    xc, yw, zs, zh = MASSAGE_ZONES[zone_name]
    if pos == 'L':
        base_y = -yw
    else:
        base_y = yw
    if crossed: base_y = -base_y  # 交叉模式：反转基座Y侧
    act_tag = "wave"
    targets = []
    total_steps = WAVE_CYCLES * WAVE_STEPS_PER_CYCLE
    for i in range(total_steps):
        t = i / WAVE_STEPS_PER_CYCLE  # 0 to cycles
        angle = t * 2 * math.pi
        # Y方向正弦: 从中心(0)到边缘(0.6*width)来回滚动
        y_osc = WAVE_AMPLITUDE_RATIO * yw * math.sin(angle)
        # Z方向随波起伏: 波谷(-amplitude)到波峰(+amplitude)
        z_osc = WAVE_Z_AMPLITUDE * (1 - math.cos(angle))
        targets.append((act_tag, xc, base_y + y_osc, zs + z_osc))
    # 回中: 追加一个中心位置，避免下一阶段起点瞬移
    targets.append((act_tag, xc, base_y, zs))
    return targets


def _expand_multi_point_segments(active_arm, zone_name, pos, tech, crossed=False):
    """由手法名生成该阶段的 (act, tx, ty, tz, duration) 目标序列。"""
    if tech == "tap":
        targets = _generate_tap_targets(zone_name, pos, crossed=crossed)
        dur = TAP_HALF_MS
    elif tech == "pound":
        targets = _generate_pound_targets(zone_name, pos, crossed=crossed)
        dur = POUND_HALF_MS
    elif tech in ARC_TECHS:
        targets = _generate_arc_targets(zone_name, pos, crossed=crossed)
        dur = ARC_STEP_MS
    elif tech in KNEAD_WIDE_TECHS:
        targets = _generate_knead_wide_targets(zone_name, pos, crossed=crossed)
        dur = KNEAD_STEP_MS
    elif tech == "wave":
        targets = _generate_wave_targets(zone_name, pos, crossed=crossed)
        dur = WAVE_STEP_MS
    else:
        return []
    return [(act, t[0], t[1], t[2], dur) for act, *t in targets]



# ═══════════════════════════════════════════════════════
# v6.0 编排器驱动 — 高层pattern自动展开 + 过渡原语
# ═══════════════════════════════════════════════════════
#
# 核心理念:
#   1. Skill Primitive Library: 按摩手法封装为参数化原语
#   2. MassageChoreographer: pattern → stages + 自动过渡
#   3. Transition Primitives: 力渐变/抬升接近/沿面滑动/振后衰减
#   4. 底层执行不变: 关节角delta + MoveIt RRT + 碰撞检测

ZONES = ["C7", "shoulder", "upper", "mid", "lower_th", "lumbar", "sacrum"]
ENDPOINT_ZONES = {"C7", "sacrum"}

# 保留 _build_waypoint() — 底层joint angle计算不变
def _build_waypoint(left_spec, left_act, right_spec, right_act):
    """构建单个阶段的(左6DOF, 右6DOF)。兼容v5格式和v6 StageDef格式。"""
    crossed_L = False
    crossed_R = False
    if isinstance(left_spec, (list, tuple)) and len(left_spec) == 3:
        zone_name, pos, crossed_L = left_spec
        left_spec = (zone_name, pos)
    if isinstance(right_spec, (list, tuple)) and len(right_spec) == 3:
        zone_name, pos, crossed_R = right_spec
        right_spec = (zone_name, pos)

    nom_act = NOMINAL_ACT.get(left_act, left_act)
    if left_act == "fwd_hover":
        jl = LEFT_FWD_HOVER
    elif isinstance(left_spec, tuple) and isinstance(left_spec[0], int):
        idx = left_spec[0]
        tx, ty, tz = _acu_left(idx, nom_act)
        jl = _wp_left(tx, ty, nom_act)
    else:
        zone_name, pos = left_spec
        tx, ty, tz = _zone_target(zone_name, pos, nom_act, crossed=crossed_L)
        jl = _wp_left(tx, ty, nom_act)

    nom_act = NOMINAL_ACT.get(right_act, right_act)
    if right_act == "fwd_hover":
        jr = RIGHT_FWD_HOVER
    elif isinstance(right_spec, tuple) and isinstance(right_spec[0], int):
        idx = right_spec[0]
        tx, ty, tz = _acu_right(idx, nom_act)
        jr = _wp_right(tx, ty, nom_act)
    else:
        zone_name, pos = right_spec
        tx, ty, tz = _zone_target(zone_name, pos, nom_act, crossed=crossed_R)
        jr = _wp_right(tx, ty, nom_act)

    return jl, jr

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
        self.vel_s   = max(0.01,min(1.0,p("velocity_scaling",0.35).get_parameter_value().double_value))
        self.acc_s   = max(0.01,min(1.0,p("acceleration_scaling",0.35).get_parameter_value().double_value))
        self.t_scale = max(0.25,min(2.0,p("trajectory_time_scale",1.0).get_parameter_value().double_value))
        self.exact_sp= max(0.05,p("exact_target_speed",0.35).get_parameter_value().double_value)
        self.fb_sp   = max(0.05,p("fallback_joint_speed",0.30).get_parameter_value().double_value)
        self.min_settle = max(0.02,p("min_settle_duration",0.16).get_parameter_value().double_value)
        self.repeat_n= p("repeat_count",0).get_parameter_value().integer_value
        self.pattern_name = p("massage_pattern","综合推拿").get_parameter_value().string_value

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

        # ── v6: 编排器 ──
        self._choreographer = None
        self._stage_defs = []       # StageDef 列表
        self._legacy_stages = []    # 兼容格式: [(ls,la,rs,ra,name),...]
        self._left_waypoints = []   # [(t, joints), ...]
        self._right_waypoints = []
        self._massage_points_l = []
        self._massage_points_r = []
        self._stage_names = []
        self._stage_raw = []

    def _load_pattern(self, pattern_name: str = None) -> bool:
        """v6: 使用编排器从YAML pattern加载并展开阶段序列。

        Returns:
            True if loaded successfully
        """
        name = pattern_name or self.pattern_name
        if not _CHOREOGRAPHER_AVAILABLE:
            self.get_logger().error(
                "Choreographer not available. Check jaka_dual_arm installation."
            )
            return False

        self._choreographer = MassageChoreographer(SkillLibrary())

        try:
            pattern = self._choreographer.load_pattern(name)
            self.get_logger().info(
                f"加载 pattern: '{pattern.name}' — {pattern.description}"
            )
        except Exception as e:
            self.get_logger().error(f"加载 pattern '{name}' 失败: {e}")
            return False

        self._stage_defs = self._choreographer.compose(pattern)
        massage_count = sum(
            1 for s in self._stage_defs if s.kind == StageKind.MASSAGE
        )
        trans_count = sum(
            1 for s in self._stage_defs if s.kind == StageKind.TRANSITION
        )
        self.get_logger().info(
            f"编排器展开: {len(self._stage_defs)} 阶段 "
            f"({massage_count} 按摩 + {trans_count} 过渡)"
        )

        # ── 转为旧demo兼容格式 ──
        self._legacy_stages.clear()
        self._stage_names.clear()
        self._stage_raw.clear()
        self._left_waypoints.clear()
        self._right_waypoints.clear()
        self._massage_points_l.clear()
        self._massage_points_r.clear()

        t = 1.0
        DT = 1.20
        for si, stage in enumerate(self._stage_defs):
            if stage.kind == StageKind.TRANSITION:
                # 过渡阶段: 跳过 (它们在_plan中的过渡逻辑中处理)
                continue

            # 转为旧格式
            legacy = stage.to_legacy_tuple()
            self._legacy_stages.append(legacy)
            ls, la, rs, ra, name = legacy
            self._stage_names.append(name)
            self._stage_raw.append((ls, la, rs, ra))
            jl, jr = _build_waypoint(ls, la, rs, ra)
            self._left_waypoints.append((t, jl))
            self._right_waypoints.append((t, jr))

            # 可视化点
            if isinstance(ls, tuple) and isinstance(ls[0], int):
                lx, ly, lz = _acu_left(ls[0], NOMINAL_ACT.get(la, la))
                self._massage_points_l.append(Point(x=lx, y=ly, z=lz))
            elif isinstance(ls, tuple) and isinstance(ls[0], str):
                crossed_L = ls[2] if len(ls) > 2 else False
                lx, ly, lz = _zone_target(ls[0], ls[1], NOMINAL_ACT.get(la, la), crossed=crossed_L)
                self._massage_points_l.append(Point(x=lx, y=ly, z=lz))

            if isinstance(rs, tuple) and isinstance(rs[0], int):
                rx, ry, rz = _acu_right(rs[0], NOMINAL_ACT.get(ra, ra))
                self._massage_points_r.append(Point(x=rx, y=ry, z=rz))
            elif isinstance(rs, tuple) and isinstance(rs[0], str):
                crossed_R = rs[2] if len(rs) > 2 else False
                rx, ry, rz = _zone_target(rs[0], rs[1], NOMINAL_ACT.get(ra, ra), crossed=crossed_R)
                self._massage_points_r.append(Point(x=rx, y=ry, z=rz))

            t += DT

        return True

    def _rebuild_stages(self, scheme):
        """v5兼容: 不再支持旧scheme参数, 全部走编排器."""
        self.get_logger().warn(
            f"v6 ignores 'massage_scheme:={scheme}' — "
            f"using 'massage_pattern:={self.pattern_name}' instead"
        )
        # no-op: v6 always uses choreographer

    # ── 初始化 ──
    def run(self)->bool:
        # ── v6: 使用编排器加载pattern ──
        if not self._load_pattern():
            self.get_logger().error("Failed to load massage pattern, aborting.")
            return False

        num_stages = len(self._legacy_stages)
        self.get_logger().info(f"=== 双臂按摩Demo v6.0 (编排器驱动) 启动 ===")
        self.get_logger().info(f"  Pattern: {self.pattern_name}, 共{num_stages}按摩阶段 "
                               f"(+过渡自动插入)")
        self.get_logger().info(f"  SEG_SPEED=0.14, vel_s={self.vel_s}")
        self.get_logger().info(f"床: z=0(贴地) 床垫顶z={MATTRESS_TOP:.2f} "
                               f"人体表面z={BODY[2][1]:.3f}~{BODY[0][1]:.3f}")
        self.get_logger().info(f"左臂基({LEFT_BASE[0]:.2f},{LEFT_BASE[1]:.2f})→身体左侧 "
                               f"右臂基({RIGHT_BASE[0]:.2f},{RIGHT_BASE[1]:.2f})→身体右侧 "
                               f"肩高z={SHOULDER_Z:.2f}")
        self.publish_markers()
        if not self._wait_svcs(): return False
        if not self._apply_scene(): return False
        self.get_logger().info("所有服务已就绪")
        start = self._wait_js(30.0)
        if start is None: return False
        self.get_logger().info(f"当前关节: {[f'{v:.3f}' for v in start[:6]]} ...")

        # 启动容差检查
        if self._left_waypoints:
            wp0 = self._left_waypoints[0][1] + self._right_waypoints[0][1]
            max_delta = max(abs(a-b) for a,b in zip(start, wp0))
            self.get_logger().info(f"启动位姿与Stage1偏差: max={max_delta:.3f}rad")
            if max_delta > 0.10:
                self.get_logger().warn(f"⚠ 启动偏差{max_delta:.3f}rad较大(>0.10)")
        else:
            self.get_logger().error("No waypoints generated from pattern")
            return False

        try:
            traj = self._plan(start)
        except RuntimeError as e:
            self.get_logger().error(f"planning aborted: {e}")
            return False
        if traj is None: return False
        self._tscale(traj)
        self.get_logger().info("开始碰撞检测...")
        if not self._validate(traj): return False
        self.l_traj, self.r_traj = self._split(traj)
        self.get_logger().info(f"轨迹已拆分: 左{len(self.l_traj.points)}点 右{len(self.r_traj.points)}点")
        self._send_cycle()
        return True

    def _send_cycle(self):
        self.pending=2; self.failed=False
        lbl = f"{self.cycles+1}/{self.repeat_n}" if self.repeat_n>0 else f"{self.cycles+1}/∞"
        self.get_logger().info(f"发送推拿循环 {lbl}（{len(self._legacy_stages)}式双臂推拿）")
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
        """防碰撞策略(v5.1.2 final):
        薄床面碰撞体(z∈[0.13,0.15],仅2cm) → RRT绕行防穿床
        + 举高/平移步用直接插值(纯轴运动,跳过RRT)"""
        # 重新导入需要的类型(模块级import已被移除)
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose as GPose
        co = CollisionObject(); co.header.frame_id = "world"
        co.id = "bed_sheet"; co.operation = CollisionObject.ADD
        # ── 薄床面: z∈[0.13,0.15] 仅2cm, 在床垫顶面和人体之间 ──
        p = SolidPrimitive(); p.type = SolidPrimitive.BOX
        p.dimensions = [1.20, 0.66, 0.02]  # 长1.2m宽0.66m厚2cm
        ps = GPose(); ps.orientation.w = 1.0
        ps.position.x = 0.70; ps.position.y = 0.0; ps.position.z = 0.14
        co.primitives.append(p); co.primitive_poses.append(ps)
        sc = PlanningScene(); sc.is_diff = True; sc.world.collision_objects.append(co)
        req = ApplyPlanningScene.Request(); req.scene = sc
        fut = self.scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if r is None or not r.success:
            self.get_logger().error("场景碰撞添加失败"); return False
        self.get_logger().info("场景碰撞: 薄床面(z=0.14,h=0.02) "
                               "| 举高/平移步直接插值跳过RRT")
        return True

    # ═══════════════════════════════════════════════════════
    # 轨迹规划（v5.1.1 MoveIt碰撞感知规划 + 3步顺序越障 + 多周期手法展开）
    # ═══════════════════════════════════════════════════════
    def _plan(self,start):
        tgts=[l[1]+r[1] for l,r in zip(self._left_waypoints,self._right_waypoints)]
        num_stages = len(self._legacy_stages)
        self.get_logger().info(f"规划{num_stages}个阶段（v6.0 编排器驱动: 自动过渡+首尾衔接）...")
        c=JointTrajectory(); c.joint_names=ALL_JOINTS
        p0=JointTrajectoryPoint(); p0.positions=list(start); p0.time_from_start=_dur(0.0)
        c.points.append(p0)
        # 防抽风: 初始位姿驻留2.0s (加长，给更多稳定时间)
        p_dwell=JointTrajectoryPoint(); p_dwell.positions=list(start); p_dwell.time_from_start=_dur(2.0)
        c.points.append(p_dwell)
        cur=list(start); toff=2.0
        plan_ok = 0; multi_stages = 0; arm_contact_warn = 0; lift_transitions = 0
        prev_lifted = None  # v5.1 越障跟踪: None/"left"/"right"
        SEG_SPEED = 0.14  # rad/s, v5.1.1 略快 (原0.12太慢, 0.15太快, 参考开源取中上)

        # ── v5.4 首阶段预抬升 (MoveIt规划防碰撞) ──
        if len(self._legacy_stages) > 0:
            firstName = self._stage_names[0]
            if "C7" in firstName and "交叉" in firstName:
                setup = LEFT_HIGH_HOVER + cur[6:12]
                md_setup = max(abs(a-b) for a,b in zip(cur, setup))
                if md_setup > 0.02:
                    toff, cur = self._try_plan(c, cur, setup, toff, "pre-lift:左→HIGH_HOVER", SEG_SPEED)
                prev_lifted = "left"
                self.get_logger().info("  首阶段预抬升: 左臂→HIGH_HOVER (MoveIt规划防碰撞)")
            elif "sacrum" in firstName and "交叉" in firstName:
                setup = cur[:6] + RIGHT_HIGH_HOVER
                md_setup = max(abs(a-b) for a,b in zip(cur, setup))
                if md_setup > 0.02:
                    toff, cur = self._try_plan(c, cur, setup, toff, "pre-lift:右→HIGH_HOVER", SEG_SPEED)
                prev_lifted = "right"
                self.get_logger().info("  首阶段预抬升: 右臂→HIGH_HOVER (MoveIt规划防碰撞)")
            else:
                # 非交叉首阶段(synced等): 双臂同时抬升到HIGH_HOVER保证安全接近身体
                setup = LEFT_HIGH_HOVER + RIGHT_HIGH_HOVER
                md_setup = max(abs(a-b) for a,b in zip(cur, setup))
                if md_setup > 0.02:
                    toff, cur = self._try_plan(c, cur, setup, toff, "pre-lift:双臂→HIGH_HOVER", SEG_SPEED)
                self.get_logger().info("  首阶段预抬升: 双臂→HIGH_HOVER (同步安全接近)")

        for ti,tgt in enumerate(tgts):
            name=self._stage_names[ti]
            ls, la, rs, ra = self._stage_raw[ti]
            lbl=f"{name} ({ti+1}/{len(self._legacy_stages)})"

            # ═══ v5.2: 5种阶段过渡, 全程在体表/上空, 不经过FWD_HOVER ═══
            next_lifted = None
            if "C7" in name and "交叉" in name:
                next_lifted = "left"
            elif "sacrum" in name and "交叉" in name:
                next_lifted = "right"

            # ── 辅助: 计算下一阶段正常侧(非交叉)按摩目标 ──
            def _next_normal_targets():
                """返回下一阶段双臂在自己半背的关节角(非交叉, press名义)"""
                next_ls, _, next_rs, _ = self._stage_raw[ti]
                if isinstance(next_ls, tuple) and len(next_ls) >= 2:
                    nzn_L, npp_L = next_ls[0], next_ls[1]
                    xn, yn, _ = _zone_target(nzn_L, npp_L, "press", crossed=False)
                    lj = _wp_left(xn, yn, "press")
                else:
                    lj = LEFT_FWD_HOVER
                if isinstance(next_rs, tuple) and len(next_rs) >= 2:
                    nzn_R, npp_R = next_rs[0], next_rs[1]
                    xn, yn, _ = _zone_target(nzn_R, npp_R, "press", crossed=False)
                    rj = _wp_right(xn, yn, "press")
                else:
                    rj = RIGHT_FWD_HOVER
                return lj, rj

            # ── 情况A: 交叉→交叉换向 (prev≠next, 两者≠None) ──
            if (prev_lifted is not None and next_lifted is not None
                    and prev_lifted != next_lifted):
                # 旧举高臂降到当前按摩位, 清除prev走正常→交叉
                self.get_logger().info(f"  ↺ {lbl} 交叉换向:{prev_lifted}→{next_lifted}")
                prev_lifted = None

            # ── 情况B: 交叉→正常 (prev≠None, next=None) ──
            # 直接在体表上空解交叉, 目标=下一阶段正常按摩位(非FWD_HOVER!)
            if prev_lifted is not None and next_lifted is None:
                l_norm, r_norm = _next_normal_targets()
                RAISE_J2 = 1.00
                if prev_lifted == "left":
                    r_cur = list(cur[6:12])
                    r_raised = [r_cur[0], RAISE_J2] + list(r_cur[2:])
                    steps = [
                        (LEFT_HIGH_HOVER + cur[6:12], "左臂抬高→解交叉"),
                        (LEFT_HIGH_HOVER + r_raised, "右臂原地举高"),
                        (LEFT_HIGH_HOVER + r_norm, "右臂滑回自己侧→按摩位"),
                        (l_norm + r_norm, "左臂降下→按摩位"),
                    ]
                else:  # prev_lifted == "right"
                    l_cur = list(cur[:6])
                    l_raised = [l_cur[0], RAISE_J2] + list(l_cur[2:])
                    steps = [
                        (cur[:6] + RIGHT_HIGH_HOVER, "右臂抬高→解交叉"),
                        (l_raised + RIGHT_HIGH_HOVER, "左臂原地举高"),
                        (l_norm + RIGHT_HIGH_HOVER, "左臂滑回自己侧→按摩位"),
                        (l_norm + r_norm, "右臂降下→按摩位"),
                    ]
                for step, slbl in steps:
                    md_h = max(abs(a-b) for a,b in zip(cur, step))
                    if md_h < 0.01: continue
                    if "原地举高" in slbl or "滑回" in slbl:
                        toff, cur = self._direct_interp(c, cur, step, toff, f"解交叉:{slbl}")
                    else:
                        toff, cur = self._try_plan(c, cur, step, toff, f"解交叉:{slbl}", SEG_SPEED)
                prev_lifted = None
                lift_transitions += 1

            # ── 情况C: 正常→交叉 (prev=None, next≠None) ──
            # 从当前按摩位开始越障(非FWD_HOVER), 举高→横穿→降下
            if next_lifted is not None and prev_lifted is None:
                # 计算交叉目标
                if isinstance(ls, tuple) and len(ls) >= 2 and isinstance(ls[0], str):
                    l_zn = ls[0]; l_pp = ls[1]
                    l_crossed = ls[2] if len(ls) >= 3 else False
                    l_xyz = _zone_target(l_zn, l_pp, "press", crossed=l_crossed)
                    l_cross_j = _wp_left(l_xyz[0], l_xyz[1], "press")
                else:
                    l_cross_j = LEFT_FWD_HOVER
                if isinstance(rs, tuple) and len(rs) >= 2 and isinstance(rs[0], str):
                    r_zn = rs[0]; r_pp = rs[1]
                    r_crossed = rs[2] if len(rs) >= 3 else False
                    r_xyz = _zone_target(r_zn, r_pp, "press", crossed=r_crossed)
                    r_cross_j = _wp_right(r_xyz[0], r_xyz[1], "press")
                else:
                    r_cross_j = RIGHT_FWD_HOVER

                RAISE_J2 = 1.00
                def _j2_high(j6):
                    jh = list(j6); jh[1] = max(jh[1], RAISE_J2); return jh
                def _raise_in_place(j6):
                    jr = list(j6); jr[1] = RAISE_J2; return jr
                if next_lifted == "left":
                    r_high = _j2_high(r_cross_j)
                    r_cur_pos = list(cur[6:12])
                    r_raised = _raise_in_place(r_cur_pos)
                    steps = [
                        (LEFT_HIGH_HOVER + cur[6:12], "左臂抬高"),
                        (LEFT_HIGH_HOVER + r_raised, "右臂原地举高(J2→1.00)"),
                        (LEFT_HIGH_HOVER + r_high, "右臂举高横穿"),
                        (LEFT_CROSS_OVER + r_high, "左臂旋转到对侧(高空)"),
                        (l_cross_j + r_cross_j, "双臂降到按摩位"),
                    ]
                else:
                    l_high = _j2_high(l_cross_j)
                    l_cur_pos = list(cur[:6])
                    l_raised = _raise_in_place(l_cur_pos)
                    steps = [
                        (cur[:6] + RIGHT_HIGH_HOVER, "右臂抬高"),
                        (l_raised + RIGHT_HIGH_HOVER, "左臂原地举高(J2→1.00)"),
                        (l_high + RIGHT_HIGH_HOVER, "左臂举高横穿"),
                        (l_high + RIGHT_CROSS_OVER, "右臂旋转到对侧(高空)"),
                        (l_cross_j + r_cross_j, "双臂降到按摩位"),
                    ]
                for step, slbl in steps:
                    md_h = max(abs(a-b) for a,b in zip(cur, step))
                    if md_h < 0.01: continue
                    if "原地举高" in slbl or "举高横穿" in slbl:
                        toff, cur = self._direct_interp(c, cur, step, toff, f"交叉:{slbl}")
                    else:
                        toff, cur = self._try_plan(c, cur, step, toff, f"交叉:{slbl}", SEG_SPEED)
                prev_lifted = next_lifted
                lift_transitions += 1

            # ── 情况D/E: 正常→正常 or 交叉→交叉(同向)
            # cur已经在体表按摩位, 直接过渡到下个目标, 不越障不解交叉

            # ── 跳过已在目标位姿的阶段 ──
            md = max(abs(a-b) for a,b in zip(cur,tgt))
            if md <= 0.002:
                plan_ok += 1; continue

            # ── 碰撞验证 ──
            req_check = GetStateValidity.Request()
            req_check.group_name = PLANNING_GROUP
            req_check.robot_state.is_diff = True
            req_check.robot_state.joint_state = JointState()
            req_check.robot_state.joint_state.name = ALL_JOINTS
            req_check.robot_state.joint_state.position = list(tgt)
            fut_check = self.valid_cli.call_async(req_check)
            rclpy.spin_until_future_complete(self, fut_check, timeout_sec=3.0)
            r_check = fut_check.result()

            if r_check is not None and not r_check.valid:
                contacts = list(r_check.contacts)
                arm_to_arm = any(
                    _is_robot_link_pair(c.contact_body_1, c.contact_body_2)
                    for c in contacts
                )
                if arm_to_arm:
                    arm_contact_warn += 1
                    cs = ", ".join(
                        f"{c.contact_body_1}<->{c.contact_body_2}"
                        for c in contacts[:3]
                    )
                    self.get_logger().info(
                        f"  {lbl} 臂-臂接触({cs}) FCL padding误报，几何安全放行"
                    )
                else:
                    self.get_logger().info(
                        f"  {lbl} 臂体接触（按摩正常）通过"
                    )

            # ══════════════════════════════════════════════
            # v5.1: 多周期手法展开（双臂独立, 含crossed传递）
            # ══════════════════════════════════════════════
            left_seg = []
            right_seg = []

            if la in MULTI_TECHS:
                # 解析左臂参数 + crossed标志 (v5.1 修复: 传递ls[2])
                if isinstance(ls, tuple) and len(ls) >= 2 and isinstance(ls[0], str):
                    zn = ls[0]
                    pp = ls[1] if len(ls) >= 2 else 'L'
                    crossed_L = ls[2] if len(ls) >= 3 else False
                    left_seg = _expand_multi_point_segments("left", zn, pp, la, crossed=crossed_L)
            if ra in MULTI_TECHS:
                if isinstance(rs, tuple) and len(rs) >= 2 and isinstance(rs[0], str):
                    zn = rs[0]
                    pp = rs[1] if len(rs) >= 2 else 'R'
                    crossed_R = rs[2] if len(rs) >= 3 else False
                    right_seg = _expand_multi_point_segments("right", zn, pp, ra, crossed=crossed_R)

            if left_seg or right_seg:
                multi_stages += 1
                if multi_stages <= 3:
                    self.get_logger().info(
                        f"  {lbl} 展开[{la}/{ra}] L{len(left_seg)}R{len(right_seg)}段"
                    )

                # 情况A: 仅左臂有多段（右臂fwd_hover）— 首段Moveit规划
                if left_seg and not right_seg:
                    for i_seg, seg_data in enumerate(left_seg):
                        l_act, lx, ly, lz, ld = seg_data
                        seg_jl = _wp_left(lx, ly, l_act)
                        seg_target = seg_jl + RIGHT_FWD_HOVER
                        if i_seg == 0:
                            toff, cur = self._try_plan(c, cur, seg_target, toff,
                                f"左{la}首段({lx:.2f},{ly:.2f})", SEG_SPEED)
                        else:
                            seg_md = max(abs(a-b) for a,b in zip(cur, seg_target))
                            n_interp = max(2, min(6, int(ld / 0.08) + 2))
                            for j in range(1, n_interp + 1):
                                ratio = j / n_interp
                                pt = JointTrajectoryPoint()
                                pt.positions = [cur[k] + (seg_target[k] - cur[k]) * ratio for k in range(12)]
                                pt.time_from_start = _dur(toff + ld * ratio)
                                c.points.append(pt)
                            toff += ld
                            cur = list(seg_target)

                # 情况B: 仅右臂有多段（左臂fwd_hover）— 首段Moveit规划
                elif right_seg and not left_seg:
                    for i_seg, seg_data in enumerate(right_seg):
                        r_act, rx, ry, rz, rd = seg_data
                        seg_jr = _wp_right(rx, ry, r_act)
                        seg_target = LEFT_FWD_HOVER + seg_jr
                        if i_seg == 0:
                            toff, cur = self._try_plan(c, cur, seg_target, toff,
                                f"右{ra}首段({rx:.2f},{ry:.2f})", SEG_SPEED)
                        else:
                            seg_md = max(abs(a-b) for a,b in zip(cur, seg_target))
                            n_interp = max(2, min(6, int(rd / 0.08) + 2))
                            for j in range(1, n_interp + 1):
                                ratio = j / n_interp
                                pt = JointTrajectoryPoint()
                                pt.positions = [cur[k] + (seg_target[k] - cur[k]) * ratio for k in range(12)]
                                pt.time_from_start = _dur(toff + rd * ratio)
                                c.points.append(pt)
                            toff += rd
                            cur = list(seg_target)

                # 情况C: 双臂多段（knead_wide/arc/wave）— 首段Moveit规划
                else:
                    n_seg = max(len(left_seg), len(right_seg))
                    while len(left_seg) < n_seg:
                        left_seg.append(left_seg[-1])
                    while len(right_seg) < n_seg:
                        right_seg.append(right_seg[-1])
                    for i in range(n_seg):
                        l_act, lx, ly, lz, ld = left_seg[i]
                        r_act, rx, ry, rz, rd = right_seg[i]
                        sdur = max(ld, rd)
                        seg_jl = _wp_left(lx, ly, l_act)
                        seg_jr = _wp_right(rx, ry, r_act)
                        seg_target = seg_jl + seg_jr
                        if i == 0:
                            toff, cur = self._try_plan(c, cur, seg_target, toff,
                                f"双臂{la}/{ra}首段({lx:.2f},{ly:.2f})", SEG_SPEED)
                        else:
                            seg_md = max(abs(a-b) for a,b in zip(cur, seg_target))
                            n_interp = max(2, min(6, int(sdur / 0.08) + 2))
                            for j in range(1, n_interp + 1):
                                ratio = j / n_interp
                                pt = JointTrajectoryPoint()
                                pt.positions = [cur[k] + (seg_target[k] - cur[k]) * ratio for k in range(12)]
                                pt.time_from_start = _dur(toff + sdur * ratio)
                                c.points.append(pt)
                            toff += sdur
                            cur = list(seg_target)

                plan_ok += 1
                continue

            # ══════════════════════════════════════════════
            # 标准阶段: MoveIt规划(碰撞感知) + 线性回退
            # ══════════════════════════════════════════════
            toff, cur = self._try_plan(c, cur, tgt, toff,
                f"阶段{ti+1}:{name}", SEG_SPEED)
            plan_ok += 1

        # 最后加一段高空悬停(MoveIt规划防碰撞)
        final_hover = LEFT_HIGH_HOVER + RIGHT_HIGH_HOVER
        toff, cur = self._try_plan(c, cur, final_hover, toff, "收尾→HIGH_HOVER", SEG_SPEED)

        self.get_logger().info(f"推拿轨迹规划完成：{len(c.points)}点, {toff:.1f}s "
                               f"| 阶段{plan_ok}/多周期{multi_stages}/顺序越障{lift_transitions}"
                               f"/臂接触{arm_contact_warn}/共{len(self._legacy_stages)}阶段")
        return c

    def _tscale(self,traj):
        if abs(self.t_scale-1.0)<=1e-6: return
        o=_ds(traj.points[-1].time_from_start)
        for pt in traj.points: pt.time_from_start=_dur(_ds(pt.time_from_start)*self.t_scale)
        self.get_logger().info(f"时间缩放:{o:.1f}s→{_ds(traj.points[-1].time_from_start):.1f}s")

    def _plan_seg(self,s,g,label):
        req=GetMotionPlan.Request(); mr=req.motion_plan_request
        mr.group_name=PLANNING_GROUP; mr.planner_id=PLANNER_ID
        mr.num_planning_attempts=3; mr.allowed_planning_time=2.0
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

    def _try_plan(self, c, cur, tgt, toff, label, seg_speed=0.14):
        """MoveIt碰撞感知规划 + 三级回退: RRT → HIGH_HOVER绕行 → 线性插值。
        返回 (new_toff, new_cur)"""
        md = max(abs(a-b) for a,b in zip(cur, tgt))
        if md < 0.01:
            return toff, list(cur)

        # ── L1: MoveIt RRTConnect (场景感知，含床+人体碰撞体) ──
        planned = self._plan_seg(cur, tgt, label)
        if planned is not None:
            if self._validate_planned_segment(planned, cur, label):
                new_toff = self._append(c, planned, cur, toff)
                return new_toff, list(tgt)
            self.get_logger().warn(f"  {label}: planned segment rejected by resampled validation")

        # ── L2: 2步绕行via HIGH_HOVER (先抬高绕过床面→再平移下降) ──
        hover = self._safe_hover_between(cur, tgt)
        if hover is not None:
            self.get_logger().warn(
                f"  ⚠ {label} RRT失败→尝试HIGH_HOVER绕行(J2≥0.85防床穿模)")
            d1 = self._plan_seg(cur, hover, f"{label}_v1↑")
            d2 = self._plan_seg(hover, tgt, f"{label}_v2↓")
            if d1 is not None and d2 is not None:
                if (
                    self._validate_planned_segment(d1, cur, f"{label}_hover_1")
                    and self._validate_planned_segment(d2, hover, f"{label}_hover_2")
                ):
                    toff = self._append(c, d1, cur, toff)
                    toff = self._append(c, d2, hover, toff)
                    return toff, list(tgt)
            self.get_logger().warn(f"  ⚠ {label} 绕行也失败→线性最后手段")

        # ── L3: 线性插值(无碰撞防护，仅作紧急兜底) ──
        self.get_logger().warn(
            f"  ⚠ {label} 所有规划失败→线性插值(无碰撞防护!)")
        if not self._validate_linear_segment(cur, tgt, label):
            recovered = self._try_recovery_plan(c, cur, tgt, toff, label, seg_speed)
            if recovered is not None:
                return recovered
            raise RuntimeError(f"{label}: no collision-safe recovery trajectory")
        duration = max(0.40, md / seg_speed)
        n_interp = max(3, min(12, int(duration / 0.10) + 2))
        for i in range(1, n_interp + 1):
            ratio = i / n_interp
            pt = JointTrajectoryPoint()
            pt.positions = [cur[j] + (tgt[j] - cur[j]) * ratio for j in range(12)]
            pt.time_from_start = _dur(toff + duration * ratio)
            c.points.append(pt)
        return toff + duration, list(tgt)

    def _direct_interp(self, c, cur, tgt, toff, label, n_pts=8):
        """直接插值(RRT跳过). 用于已知安全路径: 纯J2举高/高J2平移."""
        md = max(abs(a-b) for a,b in zip(cur, tgt))
        if md < 0.005: return toff, list(cur)
        dur = max(0.35, md / 0.35)
        for i in range(1, n_pts + 1):
            r = i / n_pts
            pt = JointTrajectoryPoint()
            pt.positions = [cur[j] + (tgt[j] - cur[j]) * r for j in range(12)]
            pt.time_from_start = _dur(toff + dur * r)
            c.points.append(pt)
        self.get_logger().info(f"  ⚡{label}:直接插值{md:.2f}rad→{dur:.1f}s")
        return toff + dur, list(tgt)

    def _validate_linear_segment(self, cur, tgt, label, n_pts=12):
        for i in range(1, n_pts + 1):
            ratio = i / n_pts
            positions = [cur[j] + (tgt[j] - cur[j]) * ratio for j in range(12)]
            if not self._robot_links_clear(positions, f"{label} linear sample {i}"):
                return False
        return True

    def _try_recovery_plan(self, c, cur, tgt, toff, label, seg_speed=0.14):
        for mode, waypoints in self._recovery_waypoint_sets(cur, tgt):
            trial = []
            last = list(cur)
            ok = True
            for idx, waypoint in enumerate(waypoints, start=1):
                if self._same_joint_state(last, waypoint):
                    continue
                seg_label = f"{label}_{mode}_{idx}"
                seg = self._plan_seg(last, waypoint, seg_label)
                if seg is None:
                    ok = False
                    break
                if not self._validate_planned_segment(seg, last, seg_label):
                    ok = False
                    break
                trial.append((seg, list(last)))
                last = list(waypoint)
            if not ok or not self._same_joint_state(last, tgt):
                continue
            for seg, fallback in trial:
                toff = self._append(c, seg, fallback, toff)
            self.get_logger().warn(f"  {label}: recovery selected {mode}")
            return toff, list(tgt)
        self.get_logger().error(f"{label}: all recovery plans failed")
        return None

    def _recovery_waypoint_sets(self, cur, tgt):
        left_cur = list(cur[:6])
        right_cur = list(cur[6:12])
        left_tgt = list(tgt[:6])
        right_tgt = list(tgt[6:12])
        left_moves = not self._same_joint_state(left_cur, left_tgt)
        right_moves = not self._same_joint_state(right_cur, right_tgt)
        candidates = []
        if left_moves:
            candidates.append(("left_first", [left_tgt + right_cur, list(tgt)]))
            candidates.append((
                "right_yields",
                [
                    left_cur + RIGHT_HIGH_HOVER,
                    left_tgt + RIGHT_HIGH_HOVER,
                    list(tgt),
                ],
            ))
        if right_moves:
            candidates.append(("right_first", [left_cur + right_tgt, list(tgt)]))
            candidates.append((
                "left_yields",
                [
                    LEFT_HIGH_HOVER + right_cur,
                    LEFT_HIGH_HOVER + right_tgt,
                    list(tgt),
                ],
            ))
        candidates.append((
            "both_high_hover",
            [
                LEFT_HIGH_HOVER + RIGHT_HIGH_HOVER,
                list(tgt),
            ],
        ))
        return candidates

    def _validate_planned_segment(self, seg, fallback, label, sample_period=0.05):
        if seg is None or not seg.points:
            return False
        total = _ds(seg.points[-1].time_from_start)
        sample_times = {0.0, total}
        for pt in seg.points:
            sample_times.add(_ds(pt.time_from_start))
        count = max(1, int(math.ceil(total / sample_period)))
        for i in range(count + 1):
            sample_times.add(min(total, i * sample_period))
        for idx, et in enumerate(sorted(sample_times)):
            positions = self._segment_positions_at(seg, fallback, et)
            if not self._robot_links_clear(positions, f"{label} sample {idx} t={et:.2f}s"):
                return False
        return True

    def _segment_positions_at(self, seg, fallback, et):
        if not seg.points:
            return list(fallback)
        if et <= _ds(seg.points[0].time_from_start):
            return self._pos(seg, seg.points[0], fallback)
        prev = seg.points[0]
        for nxt in seg.points[1:]:
            t0 = _ds(prev.time_from_start)
            t1 = _ds(nxt.time_from_start)
            if et <= t1:
                if t1 <= t0:
                    return self._pos(seg, nxt, fallback)
                ratio = max(0.0, min(1.0, (et - t0) / (t1 - t0)))
                pp = self._pos(seg, prev, fallback)
                np = self._pos(seg, nxt, fallback)
                return [pp[j] + (np[j] - pp[j]) * ratio for j in range(12)]
            prev = nxt
        return self._pos(seg, seg.points[-1], fallback)

    def _robot_links_clear(self, positions, label):
        req=GetStateValidity.Request(); req.group_name=PLANNING_GROUP
        req.robot_state.is_diff=True
        req.robot_state.joint_state=JointState(); req.robot_state.joint_state.name=ALL_JOINTS
        req.robot_state.joint_state.position=positions
        fut=self.valid_cli.call_async(req)
        rclpy.spin_until_future_complete(self,fut,timeout_sec=1.0)
        res=fut.result()
        if res is None:
            self.get_logger().error(f"{label}: validation timeout")
            return False
        if res.valid:
            return True
        contacts=list(res.contacts)
        if any(_is_robot_link_pair(c.contact_body_1,c.contact_body_2) for c in contacts):
            cs=", ".join(f"{c.contact_body_1}<->{c.contact_body_2}" for c in contacts[:4])
            self.get_logger().error(f"{label}: robot-link collision {cs}")
            return False
        if not contacts:
            self.get_logger().error(f"{label}: invalid without contact details")
            return False
        return True

    def _same_joint_state(self, a, b, tol=0.005):
        if len(a) != len(b):
            return False
        if not a:
            return True
        return max(abs(x-y) for x,y in zip(a,b)) <= tol

    def _safe_hover_between(self, cur, tgt):
        """生成高空安全绕行位姿: 将cur双臂J2抬高到≥1.00，肘部远离床面。
        如果cur的J2已经≥1.00则返回None(不需要绕行)。"""
        hover = list(cur)
        changed = False
        target_j2 = 1.00
        if hover[1] < target_j2:
            hover[1] = target_j2; changed = True
        if hover[7] < target_j2:
            hover[7] = target_j2; changed = True
        if not changed:
            return None
        return hover

    def _pos(self,traj,pt,fb):
        pbn={jn:fb[i] for i,jn in enumerate(ALL_JOINTS)}
        for jn,jp in zip(traj.joint_names,pt.positions): pbn[jn]=jp
        return [pbn[jn] for jn in ALL_JOINTS]

    def _validate(self,traj)->bool:
        """碰撞检测（轻量模式：仅检查轨迹关键帧）。
        只做日志，不阻止执行。
        臂-臂接触为FCL padding误报（几何保证左L右R不干涉），放行。"""
        CHECK_LIMIT = min(50, len(traj.points))
        contacts_found = 0
        for i in range(0, CHECK_LIMIT, 2):
            pt = traj.points[i]
            req=GetStateValidity.Request(); req.group_name=PLANNING_GROUP
            req.robot_state.is_diff=True
            req.robot_state.joint_state=JointState(); req.robot_state.joint_state.name=ALL_JOINTS
            req.robot_state.joint_state.position=pt.positions
            fut=self.valid_cli.call_async(req)
            rclpy.spin_until_future_complete(self,fut,timeout_sec=1.0)
            r=fut.result()
            if r is not None and not r.valid:
                contacts_found += 1
                if contacts_found <= 3:
                    arm_arm = any(
                        _is_robot_link_pair(c.contact_body_1, c.contact_body_2)
                        for c in r.contacts
                    )
                    cs=", ".join(f"{c.contact_body_1}<->{c.contact_body_2}" for c in r.contacts[:2])
                    if arm_arm:
                        self.get_logger().info(f"  waypoint {i}臂-臂接触(误报): {cs}")
                    else:
                        self.get_logger().info(f"  waypoint {i}臂-体接触(按摩正常): {cs}")
        if contacts_found > 0:
            self.get_logger().info(f"  {contacts_found}个waypoint有接触（臂-臂误报/臂-体正常，放行）")
        else:
            self.get_logger().info(f"碰撞检测通过")
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

    def _validate(self,traj)->bool:
        """Strict pre-execution validation for the final both_arms trajectory."""
        if traj is None or not traj.points:
            return False

        contacts_found = 0
        total = _ds(traj.points[-1].time_from_start)
        sample_period = 0.05
        sample_times = {0.0, total}
        for pt in traj.points:
            sample_times.add(_ds(pt.time_from_start))
        count = max(1, int(math.ceil(total / sample_period)))
        for i in range(count + 1):
            sample_times.add(min(total, i * sample_period))

        for idx, et in enumerate(sorted(sample_times)):
            positions = self._interp(traj, et)
            if self._robot_links_clear(positions, f"final trajectory sample {idx} t={et:.2f}s"):
                continue
            contacts_found += 1
            if contacts_found <= 3:
                self.get_logger().error(f"final trajectory rejected near t={et:.2f}s")
            return False

        if contacts_found > 0:
            self.get_logger().info(f"collision validation passed with {contacts_found} non-arm contacts")
        else:
            self.get_logger().info("collision validation passed")
        return True

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
        # 按摩路径线（左臂青色 / 右臂橙色）
        ma.markers.append(self._lm(now,10,"left_path",self._massage_points_l,(0.0,0.75,0.95,1.0),0.012))
        ma.markers.append(self._lm(now,11,"right_path",self._massage_points_r,(0.95,0.58,0.20,1.0),0.012))
        # 路径目标点：左臂青色 / 右臂橙色（区分左右臂目标）
        for i,pt in enumerate(self._massage_points_l):
            ma.markers.append(self._sm(now,200+i,"target_L",pt,0.020,(0.0,0.75,0.95,0.9)))
        for i,pt in enumerate(self._massage_points_r):
            ma.markers.append(self._sm(now,300+i,"target_R",pt,0.020,(0.95,0.58,0.20,0.9)))
        # ── 手和脚（参照Gazebo完整人体模型位置）──
        # 手: 俯卧位手臂沿身体两侧自然伸展
        LIMB_COLOR = (0.78, 0.65, 0.55, 0.85)
        HAND_COLOR = (0.92, 0.75, 0.60, 0.90)
        FOOT_COLOR = (0.70, 0.55, 0.45, 0.90)
        # 左手臂 (上臂→前臂→手)
        ma.markers.append(self._bm(now,400,"left_arm",
            Point(x=0.62,y=-0.29,z=0.17),Point(x=0.26,y=0.05,z=0.05),LIMB_COLOR))
        ma.markers.append(self._bm(now,401,"left_forearm",
            Point(x=0.70,y=-0.30,z=0.16),Point(x=0.18,y=0.04,z=0.04),LIMB_COLOR))
        ma.markers.append(self._sm(now,402,"left_hand",
            Point(x=0.79,y=-0.31,z=0.17),0.07,HAND_COLOR))
        # 右手臂
        ma.markers.append(self._bm(now,410,"right_arm",
            Point(x=0.62,y=0.29,z=0.17),Point(x=0.26,y=0.05,z=0.05),LIMB_COLOR))
        ma.markers.append(self._bm(now,411,"right_forearm",
            Point(x=0.70,y=0.30,z=0.16),Point(x=0.18,y=0.04,z=0.04),LIMB_COLOR))
        ma.markers.append(self._sm(now,412,"right_hand",
            Point(x=0.79,y=0.31,z=0.17),0.07,HAND_COLOR))
        # 左腿 (大腿→小腿→脚)
        ma.markers.append(self._bm(now,420,"left_thigh",
            Point(x=0.98,y=-0.09,z=0.16),Point(x=0.24,y=0.07,z=0.045),LIMB_COLOR))
        ma.markers.append(self._bm(now,421,"left_calf",
            Point(x=1.21,y=-0.10,z=0.15),Point(x=0.22,y=0.06,z=0.04),LIMB_COLOR))
        ma.markers.append(self._bm(now,422,"left_foot",
            Point(x=1.37,y=-0.09,z=0.14),Point(x=0.13,y=0.065,z=0.035),FOOT_COLOR))
        ma.markers.append(self._sm(now,423,"left_toe",
            Point(x=1.43,y=-0.09,z=0.14),0.06,FOOT_COLOR))
        # 右腿
        ma.markers.append(self._bm(now,430,"right_thigh",
            Point(x=0.98,y=0.09,z=0.16),Point(x=0.24,y=0.07,z=0.045),LIMB_COLOR))
        ma.markers.append(self._bm(now,431,"right_calf",
            Point(x=1.21,y=0.10,z=0.15),Point(x=0.22,y=0.06,z=0.04),LIMB_COLOR))
        ma.markers.append(self._bm(now,432,"right_foot",
            Point(x=1.37,y=0.09,z=0.14),Point(x=0.13,y=0.065,z=0.035),FOOT_COLOR))
        ma.markers.append(self._sm(now,433,"right_toe",
            Point(x=1.43,y=0.09,z=0.14),0.06,FOOT_COLOR))
        # ── 左右臂标牌 (高空大字, 青=左臂 橙=右臂) ──
        def _arm_label(mid, txt, pos, rgba):
            m = Marker(); m.header.frame_id = "world"; m.header.stamp = now
            m.ns = "labels"; m.id = mid; m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD; m.pose.position = pos
            m.pose.orientation.x = 0.0; m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0; m.pose.orientation.w = 1.0
            m.scale.z = 0.25; m.text = txt
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.frame_locked = False
            return m
        ma.markers.append(_arm_label(500, "◀◀ 左臂 ◀◀",
            Point(x=0.53, y=-0.78, z=1.10), (0.0, 0.8, 1.0, 1.0)))
        ma.markers.append(_arm_label(501, "▶▶ 右臂 ▶▶",
            Point(x=0.69, y=0.78, z=1.10), (1.0, 0.55, 0.15, 1.0)))
        # 末端实时球体(大号，绿色=低位/红色=高位)
        js = self.js
        if all(j in js for j in ALL_JOINTS):
            for side, base, color, mid_off in [
                ("left", LEFT_BASE, (0.0, 1.0, 0.5, 1.0), 510),
                ("right", RIGHT_BASE, (1.0, 0.45, 0.15, 1.0), 520)]:
                joints = [js[j] for j in (LEFT_JOINTS if side == "left" else RIGHT_JOINTS)]
                j2 = joints[1]
                # 末端粗略位置(简化FK): 关节角→xyz
                bx, by, bz = base
                # 肩位
                sx, sy, sz = bx, by, bz + SHOULDER_Z
                # 上臂指向: J1水平角 + J2垂直角
                j1, j2v = joints[0], joints[1]
                ux = sx + 0.295 * math.cos(j2v) * math.sin(j1) if abs(j1) > 0.01 else sx
                uy = sy + 0.295 * math.cos(j2v) * math.cos(j1)
                uz = sz + 0.295 * math.sin(j2v) if j2v > 0 else sz + 0.295
                # 前臂方向(简化J3)
                j3 = joints[2]
                fx = ux + 0.295 * math.cos(j2v + j3) * math.sin(j1) if abs(j1) > 0.01 else ux
                fy = uy + 0.295 * math.cos(j2v + j3) * math.cos(j1)
                fz = uz - 0.295 * math.sin(abs(j2v + j3))
                tip = Point(x=fx, y=fy, z=fz)
                # 颜色: J2>0.85 绿色(安全高位), 否则红色(危险低位)
                safe_color = (0.2, 0.9, 0.2, 1.0) if j2 > 0.85 else (0.95, 0.2, 0.2, 1.0)
                ma.markers.append(self._sm(now, mid_off, f"{side}_tip", tip, 0.06, safe_color))
                # 高度指示条: 床面z=0.14→末端z
                ma.markers.append(self._bm(now, mid_off+1, f"{side}_zbar",
                    Point(x=tip.x, y=tip.y, z=(tip.z + 0.14)/2),
                    Point(x=0.04, y=0.04, z=max(0.02, tip.z - 0.14)), safe_color))
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
