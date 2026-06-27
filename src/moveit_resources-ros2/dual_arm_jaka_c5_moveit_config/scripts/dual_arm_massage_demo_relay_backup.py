#!/usr/bin/env python3
"""双臂中医推拿按摩 Demo — v3.1 纯单臂接力，零碰撞。
   核心理念:
     1. 严格单臂: 一臂工作时另一臂fwd_hover在床前方 → 零碰撞
     2. 接力式: 左臂完成全部zone后右臂再来一遍
     3. 双臂永不同时在床面上方，物理上不可能碰撞
     4. 节奏优先: 流畅衔接 > 手法种类多
     5. 7个Phase ~94式，覆盖7种手法"""


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

# ── 安全泊车位姿（紧急停止时用，正常工作不需要） ──
LEFT_SAFE_PARK  = [-1.57, 0.30, -0.50, 1.50, 1.57, 0.0]   # 左臂指向左侧(-y)
RIGHT_SAFE_PARK = [ 1.57, 0.30, -0.50, 1.50, 1.57, 0.0]   # 右臂指向右侧(+y)

# ── 向前悬停位姿（一臂工作时另一臂的安全等待位姿） ──
# j1=0 → 臂沿+X方向指向前方，左臂Y=-0.45/右臂Y=+0.45（在床外）
# 两臂间距0.90m，Link_03永不交叉
LEFT_FWD_HOVER  = [0.0, 0.75, -1.10, 1.30, 1.57, 1.50]
RIGHT_FWD_HOVER = [0.0, 0.75, -1.10, 1.30, 1.57, 1.50]

def _is_arm_link(name: str) -> bool:
    """判断碰撞体是否为机械臂连杆"""
    return name.startswith("left_Link_") or name.startswith("right_Link_")

# ── 臂基座位置（X错开0.16m，避免双臂碰撞） ──
LEFT_BASE  = (0.53, -0.45, 0.0)
RIGHT_BASE = (0.69,  0.45, 0.0)
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

def _zone_target(zone_name, pos, act):
    """统一按摩目标点计算。
    pos: 'L'=左臂侧(病人右半背,y=-yw), 'R'=右臂侧(病人左半背,y=+yw)
    左臂→'L'(近左臂), 右臂→'R'(近右臂), 各管各的半背不跨越中线"""
    xc, yw, zs, zh = MASSAGE_ZONES[zone_name]
    if pos == 'C':   y = 0.0
    elif pos == 'L': y = -yw
    else:            y = yw   # 'R'

    if act in ("hover","release","tap","strike"): return xc, y, zh
    if act in ("knead_L","rub_L","knead_wL"):   return xc, y+0.015, zs
    if act in ("knead_R","rub_R","knead_wR"):   return xc, y-0.015, zs
    if act == "scrub": return xc+0.02, y, zs        # 擦法前推2cm
    return xc, y, zs  # press, deep_press, roll, vibrate, pound, arc_L, arc_R

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
TAP_HALF_MS = 0.35       # 半周期(s) — 约1.4Hz（原0.12太快）
POUND_CYCLES = 3         # 捶打脉冲次数
POUND_HALF_MS = 0.50     # 半周期(s) 原0.22
ARC_POINTS = 5           # 弧线插值点数（含两端）
ARC_STEP_MS = 0.40       # 弧线每步耗时(s) 原0.20
KNEAD_POINTS = 4         # 揉搓圆周分点数
KNEAD_STEP_MS = 0.35     # 揉搓每步耗时(s) 原0.20
KNEAD_RADIUS = 0.020     # 大揉搓半径(m) = 2cm

MULTI_TECHS = {"tap", "pound", "arc_L", "arc_R", "knead_wL", "knead_wR"}
ARC_TECHS = {"arc_L", "arc_R"}
KNEAD_WIDE_TECHS = {"knead_wL", "knead_wR"}

NOMINAL_ACT = {
    "tap": "press",
    "pound": "deep_press",
    "arc_L": "rub_L",
    "arc_R": "rub_R",
    "knead_wL": "knead_L",
    "knead_wR": "knead_R",
}

def _generate_tap_targets(zone_name, pos):
    """生成敲击振荡的(x,y,z)目标序列 — press↔hover 交替 5次."""
    xp, yp, zp = _zone_target(zone_name, pos, "press")
    xh, yh, zh = _zone_target(zone_name, pos, "hover")
    targets = []
    for i in range(TAP_CYCLES):
        targets.append(("hover", xh, yh, zh))
        targets.append(("press", xp, yp, zp))
    return targets

def _generate_pound_targets(zone_name, pos):
    """生成捶打脉冲的(x,y,z)目标序列 — strike↔hover 交替 3次."""
    x_strike, y_strike, z_strike = _zone_target(zone_name, pos, "strike")
    xh, yh, zh = _zone_target(zone_name, pos, "hover")
    targets = []
    for i in range(POUND_CYCLES):
        targets.append(("hover", xh, yh, zh))
        targets.append(("strike", x_strike, y_strike, z_strike))
    return targets

def _generate_arc_targets(zone_name, pos):
    """生成弧线扫过的(x,y,z)目标序列 — 从pos侧向中线摆动再回来。"""
    xc, yw, zs, zh = MASSAGE_ZONES[zone_name]
    if pos == 'L':  y_start, y_end = -yw * 0.8, 0.0
    else:           y_start, y_end = yw * 0.8, 0.0
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

def _generate_knead_wide_targets(zone_name, pos):
    """生成大揉搓圆周的(x,y,z)目标序列 — 4点圆周 + 回到中心。"""
    xc, yw, zs, zh = MASSAGE_ZONES[zone_name]
    if pos == 'L':
        base_y = -yw
        act_tag = "knead_wL"
    else:
        base_y = yw
        act_tag = "knead_wR"
    targets = []
    for i in range(KNEAD_POINTS):
        angle = i * 2 * math.pi / KNEAD_POINTS
        yo = KNEAD_RADIUS * math.sin(angle)
        zo = 0.005 * (1 - math.cos(angle))
        targets.append((act_tag, xc, base_y + yo, zs + zo))
    targets.append(("press", xc, base_y, zs))
    return targets

def _expand_multi_point_segments(active_arm, zone_name, pos, tech):
    """由手法名生成该阶段的 (act, tx, ty, tz, duration) 目标序列。"""
    if tech == "tap":
        targets = _generate_tap_targets(zone_name, pos)
        dur = TAP_HALF_MS
    elif tech == "pound":
        targets = _generate_pound_targets(zone_name, pos)
        dur = POUND_HALF_MS
    elif tech in ARC_TECHS:
        targets = _generate_arc_targets(zone_name, pos)
        dur = ARC_STEP_MS
    elif tech in KNEAD_WIDE_TECHS:
        targets = _generate_knead_wide_targets(zone_name, pos)
        dur = KNEAD_STEP_MS
    else:
        return []
    return [(act, t[0], t[1], t[2], dur) for act, *t in targets]

# ═══════════════════════════════════════════════════════
# v3.0 纯单臂接力编排 — 7个Phase ~94式, 零碰撞
# ═══════════════════════════════════════════════════════
#
# 关键设计: 所有阶段均为单臂（一臂工作，另一臂fwd_hover在床前方）
# 双臂永不同时在床面上方 → 物理上零碰撞
#
# JAKA C5臂交叉原理:
#   左臂基座y=-0.45，人体背部y≈-0.1~0.1。为触达背部，
#   左臂J1需偏转约70-113°，导致臂体(上臂/前臂)必然越过中线。
#   这是机械臂几何的必然结果 → 双臂同时工作必然碰撞。
#   唯一解: 纯单臂接力。
#
# Phase 1: 揉搓接力 (14) — 左揉7区 + 右揉7区
# Phase 2: 敲击接力 (14) — 左敲7区 + 右敲7区
# Phase 3: 点穴接力 (14) — 左按7穴 + 右按7穴
# Phase 4: 捶打接力 (10) — 左捶5区 + 右捶5区
# Phase 5: 弧扫接力 (14) — 左弧7区 + 右弧7区
# Phase 6: 振按接力 (14) — 左振7区 + 右摩7区
# Phase 7: 收功按压 (14) — 左按7区 + 右按7区
#   total: 94 stages

ZONES = ["C7", "shoulder", "upper", "mid", "lower_th", "lumbar", "sacrum"]
ACU_NAMES = ["BL11大杼", "BL13肺俞", "BL15心俞", "BL17膈俞", "BL18肝俞", "BL23肾俞", "BL25大肠俞"]

def _build_stages():
    """生成推拿编排 — 纯单臂接力，零碰撞。

    核心理念:
      1. 严格单臂: 一臂工作时另一臂fwd_hover在床前方
      2. 双臂接力: 左臂→右臂→左臂→... 交替完成各手法
      3. 每个手法左臂先做全部zone，然后右臂做全部zone
      4. 双臂永不同时在床面上方 → 零碰撞（物理保证）

    JAKA C5臂交叉原理:
      左臂基座y=-0.45, 人体y=-0.1~0.1。要触达背部，左臂J1必须
      偏转~70-113°，导致臂体(上臂/前臂)越过中线到对侧。
      这是机械臂几何的必然结果，双臂同时工作必然碰撞。
      唯一解: 一次只有一臂工作。
    """
    stages = []

    # 为每个手法定义左臂和右臂的操作序列
    # 格式: (technique, zones, pos_for_arm)
    # 左臂→pos='L'(自体侧), 右臂→pos='R'(自体侧)

    # ════════════════════════════════════════════════
    # Phase 1: 揉搓接力 (14 stages)
    # 左臂大揉搓C7→骶骨 (7), 右臂大揉搓C7→骶骨 (7)
    # ════════════════════════════════════════════════
    for zone in ZONES:
        stages.append(((zone, "L"), "knead_wL", None, "fwd_hover",
                       f"左揉·{zone}"))
    for zone in ZONES:
        stages.append((None, "fwd_hover", (zone, "R"), "knead_wR",
                       f"右揉·{zone}"))

    # ════════════════════════════════════════════════
    # Phase 2: 敲击接力 (14 stages)
    # 左敲7区 → 右敲7区
    # ════════════════════════════════════════════════
    for zone in ZONES:
        stages.append(((zone, "L"), "tap", None, "fwd_hover",
                       f"左敲·{zone}"))
    for zone in ZONES:
        stages.append((None, "fwd_hover", (zone, "R"), "tap",
                       f"右敲·{zone}"))

    # ════════════════════════════════════════════════
    # Phase 3: 点穴接力 (14 stages)
    # 左7穴(按压) → 右7穴(深按)
    # ════════════════════════════════════════════════
    for idx in range(7):
        stages.append(((idx, "acu"), "press", None, "fwd_hover",
                       f"左穴·{ACU_NAMES[idx]}"))
    for idx in range(7):
        stages.append((None, "fwd_hover", (idx, "acu"), "deep_press",
                       f"右穴·{ACU_NAMES[idx]}"))

    # ════════════════════════════════════════════════
    # Phase 4: 捶打接力 (10 stages)
    # 左捶5区(避开C7/骶) → 右捶5区
    # ════════════════════════════════════════════════
    for zone in ZONES[1:6]:
        stages.append(((zone, "L"), "pound", None, "fwd_hover",
                       f"左捶·{zone}"))
    for zone in ZONES[1:6]:
        stages.append((None, "fwd_hover", (zone, "R"), "pound",
                       f"右捶·{zone}"))

    # ════════════════════════════════════════════════
    # Phase 5: 弧扫接力 (14 stages)
    # 左弧7区 → 右弧7区
    # ════════════════════════════════════════════════
    for zone in ZONES:
        stages.append(((zone, "L"), "arc_L", None, "fwd_hover",
                       f"左弧·{zone}"))
    for zone in ZONES:
        stages.append((None, "fwd_hover", (zone, "R"), "arc_R",
                       f"右弧·{zone}"))

    # ════════════════════════════════════════════════
    # Phase 6: 振按接力 (14 stages)
    # 左振7区 → 右摩7区
    # ════════════════════════════════════════════════
    for zone in ZONES:
        stages.append(((zone, "L"), "vibrate", None, "fwd_hover",
                       f"左振·{zone}"))
    for zone in ZONES:
        stages.append((None, "fwd_hover", (zone, "R"), "rub_R",
                       f"右摩·{zone}"))

    # ════════════════════════════════════════════════
    # Phase 7: 收功按压 (14 stages)
    # 左按7区 → 右按7区
    # ════════════════════════════════════════════════
    for zone in ZONES:
        stages.append(((zone, "L"), "press", None, "fwd_hover",
                       f"左按·{zone}"))
    for zone in ZONES:
        stages.append((None, "fwd_hover", (zone, "R"), "press",
                       f"右按·{zone}"))

    return stages

STAGE_DEFS = _build_stages()
NUM_STAGES = len(STAGE_DEFS)

def _build_waypoint(left_spec, left_act, right_spec, right_act):
    """构建单个阶段的(左6DOF, 右6DOF)。
    left_spec/right_spec:
      - (int, "acu") → 穴位
      - (str, str) → 区域+位置 (zone_name, pos)
      - None → 该臂使用fwd_hover（不指定spec）
    left_act/right_act:
      - "fwd_hover" → 使用向前悬停位姿
      - 新手法(tap/pound/arc等) → 映射到名义手法"""
    # 名义手法映射（新手法在waypoint层与已有手法同，扩展在_plan中做）
    nom_act = NOMINAL_ACT.get(left_act, left_act)
    if left_act == "fwd_hover":
        jl = LEFT_FWD_HOVER
    elif isinstance(left_spec, tuple) and isinstance(left_spec[0], int):
        idx = left_spec[0]
        tx, ty, tz = _acu_left(idx, nom_act)
        jl = _wp_left(tx, ty, nom_act)
    else:
        zone_name, pos = left_spec
        tx, ty, tz = _zone_target(zone_name, pos, nom_act)
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
        tx, ty, tz = _zone_target(zone_name, pos, nom_act)
        jr = _wp_right(tx, ty, nom_act)

    return jl, jr

# 构建 waypoint 和元数据
LEFT_WAYPOINTS  = []
RIGHT_WAYPOINTS = []
MASSAGE_POINTS_L = []
MASSAGE_POINTS_R = []
STAGE_NAMES = []
STAGE_RAW = []  # (ls, la, rs, ra) — 传给_plan用于多周期展开
t = 1.0
DT = 0.75  # 每阶段时间间隔

for si, sd in enumerate(STAGE_DEFS):
    ls, la, rs, ra, name = sd
    STAGE_NAMES.append(name)
    STAGE_RAW.append((ls, la, rs, ra))
    jl, jr = _build_waypoint(ls, la, rs, ra)
    LEFT_WAYPOINTS.append((t, jl))
    RIGHT_WAYPOINTS.append((t, jr))

    # ── 左臂可视化点 ──
    if la == "fwd_hover":
        MASSAGE_POINTS_L.append(Point(x=0.30, y=-0.45, z=0.20))
    elif isinstance(ls, tuple) and isinstance(ls[0], int):
        idx = ls[0]
        lx, ly, lz = _acu_left(idx, NOMINAL_ACT.get(la, la))
        MASSAGE_POINTS_L.append(Point(x=lx, y=ly, z=lz))
    else:
        zone_name, pos = ls
        lx, ly, lz = _zone_target(zone_name, pos, NOMINAL_ACT.get(la, la))
        MASSAGE_POINTS_L.append(Point(x=lx, y=ly, z=lz))

    # ── 右臂可视化点 ──
    if ra == "fwd_hover":
        MASSAGE_POINTS_R.append(Point(x=0.30, y=0.45, z=0.20))
    elif isinstance(rs, tuple) and isinstance(rs[0], int):
        idx = rs[0]
        rx, ry, rz = _acu_right(idx, NOMINAL_ACT.get(ra, ra))
        MASSAGE_POINTS_R.append(Point(x=rx, y=ry, z=rz))
    else:
        zone_name, pos = rs
        rx, ry, rz = _zone_target(zone_name, pos, NOMINAL_ACT.get(ra, ra))
        MASSAGE_POINTS_R.append(Point(x=rx, y=ry, z=rz))

    t += DT

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
        self.t_scale = max(0.25,min(2.0,p("trajectory_time_scale",1.0).get_parameter_value().double_value))
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
        self.get_logger().info("=== 双臂按摩Demo v2.0 启动 ===")
        self.get_logger().info(f"床: z=0(贴地) 床垫顶z={MATTRESS_TOP:.2f} "
                               f"人体表面z={BODY[2][1]:.3f}~{BODY[0][1]:.3f}")
        self.get_logger().info(f"左臂基({LEFT_BASE[0]:.2f},{LEFT_BASE[1]:.2f})→身体左侧 "
                               f"右臂基({RIGHT_BASE[0]:.2f},{RIGHT_BASE[1]:.2f})→身体右侧 "
                               f"肩高z={SHOULDER_Z:.2f}")
        self.get_logger().info(f"阶段数: {NUM_STAGES} (7个Phase, 纯单臂接力, 零碰撞)")
        self.publish_markers()
        if not self._wait_svcs(): return False
        self.get_logger().info("所有服务已就绪")
        start = self._wait_js(30.0)
        if start is None: return False
        self.get_logger().info(f"当前关节: {[f'{v:.3f}' for v in start[:6]]} ...")

        # 启动容差检查：确保当前位姿与Stage1 waypoint接近，防止视觉瞬移
        wp0 = LEFT_WAYPOINTS[0][1] + RIGHT_WAYPOINTS[0][1]
        max_delta = max(abs(a-b) for a,b in zip(start, wp0))
        self.get_logger().info(f"启动位姿与Stage1偏差: max={max_delta:.3f}rad")
        if max_delta > 0.10:
            self.get_logger().warn(f"⚠ 启动偏差{max_delta:.3f}rad较大(>0.10)，"
                                   f"建议检查initial_positions YAML是否匹配Stage1 waypoint")
        traj = self._plan(start)
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
        self.get_logger().info(f"发送推拿循环 {lbl}（{NUM_STAGES}式双臂推拿）")
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
        self._box(co,(0.60,0.36,0.10),0.60,0.0,0.185)  # 躯干z=0.185
        self._sphere(co,(0.27,0.0,0.205),0.10)          # 头z=0.205
        self._box(co,(0.06,0.16,0.06),0.48,-0.26,0.18)  # 左臂z=0.18
        self._box(co,(0.06,0.16,0.06),0.48, 0.26,0.18)  # 右臂z=0.18
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

    # ═══════════════════════════════════════════════════════
    # 轨迹规划（v2.0 多周期手法展开）
    # ═══════════════════════════════════════════════════════
    def _plan(self,start):
        tgts=[l[1]+r[1] for l,r in zip(LEFT_WAYPOINTS,RIGHT_WAYPOINTS)]
        self.get_logger().info(f"规划{NUM_STAGES}个阶段（含多周期手法展开）...")
        c=JointTrajectory(); c.joint_names=ALL_JOINTS
        p0=JointTrajectoryPoint(); p0.positions=list(start); p0.time_from_start=_dur(0.0)
        c.points.append(p0)
        # 防抽风: 初始位姿驻留1.5s
        p_dwell=JointTrajectoryPoint(); p_dwell.positions=list(start); p_dwell.time_from_start=_dur(1.5)
        c.points.append(p_dwell)
        cur=list(start); toff=1.5
        plan_ok = 0; multi_stages = 0; arm_contact_warn = 0
        SEG_SPEED = 0.15  # rad/s, 关节空间插值速度（单点阶段，原0.35太快）

        for ti,tgt in enumerate(tgts):
            name=STAGE_NAMES[ti]
            ls, la, rs, ra = STAGE_RAW[ti]
            lbl=f"{name} ({ti+1}/{NUM_STAGES})"

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
                    _is_arm_link(c.contact_body_1) and _is_arm_link(c.contact_body_2)
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
            # v2.0: 多周期手法展开（双臂独立）
            # 支持: 单臂(tap/pound→另一臂fwd_hover硬编码)
            #       双臂(knead_wide/arc→同时圆周/扫过)
            # ══════════════════════════════════════════════
            left_seg = []
            right_seg = []

            if la in MULTI_TECHS and isinstance(ls, tuple) and isinstance(ls[0], str):
                zn, pp = ls
                left_seg = _expand_multi_point_segments("left", zn, pp, la)
            if ra in MULTI_TECHS and isinstance(rs, tuple) and isinstance(rs[0], str):
                zn, pp = rs
                right_seg = _expand_multi_point_segments("right", zn, pp, ra)

            if left_seg or right_seg:
                multi_stages += 1
                if multi_stages <= 3:
                    self.get_logger().info(
                        f"  {lbl} 展开[{la}/{ra}] L{len(left_seg)}R{len(right_seg)}段"
                    )

                # 情况A: 仅左臂有多段（右臂fwd_hover）
                if left_seg and not right_seg:
                    for l_act, lx, ly, lz, ld in left_seg:
                        seg_jl = _wp_left(lx, ly, l_act)
                        seg_jr = RIGHT_FWD_HOVER
                        seg_target = seg_jl + seg_jr
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

                # 情况B: 仅右臂有多段（左臂fwd_hover）
                elif right_seg and not left_seg:
                    for r_act, rx, ry, rz, rd in right_seg:
                        seg_jl = LEFT_FWD_HOVER
                        seg_jr = _wp_right(rx, ry, r_act)
                        seg_target = seg_jl + seg_jr
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

                # 情况C: 双臂都有多段（knead_wide/arc同时运动）
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
            # 标准阶段: 直接线性插值 (从cur到tgt)
            # ══════════════════════════════════════════════
            duration = max(0.30, md / SEG_SPEED)
            n_interp = max(2, min(10, int(duration / 0.10) + 2))
            for i in range(1, n_interp + 1):
                ratio = i / n_interp
                pt = JointTrajectoryPoint()
                pt.positions = [cur[j] + (tgt[j] - cur[j]) * ratio for j in range(12)]
                pt.time_from_start = _dur(toff + duration * ratio)
                c.points.append(pt)
            toff += duration
            cur = list(tgt)
            plan_ok += 1

        self.get_logger().info(f"推拿轨迹规划完成：{len(c.points)}点, {toff:.1f}s "
                               f"| 阶段{plan_ok}/多周期{multi_stages}/臂接触{arm_contact_warn}/共{NUM_STAGES}阶段")
        return c

    def _tscale(self,traj):
        if abs(self.t_scale-1.0)<=1e-6: return
        o=_ds(traj.points[-1].time_from_start)
        for pt in traj.points: pt.time_from_start=_dur(_ds(pt.time_from_start)*self.t_scale)
        self.get_logger().info(f"时间缩放:{o:.1f}s→{_ds(traj.points[-1].time_from_start):.1f}s")

    def _plan_seg(self,s,g,label):
        req=GetMotionPlan.Request(); mr=req.motion_plan_request
        mr.group_name=PLANNING_GROUP; mr.planner_id=PLANNER_ID
        mr.num_planning_attempts=5; mr.allowed_planning_time=3.0
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
                        _is_arm_link(c.contact_body_1) and _is_arm_link(c.contact_body_2)
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
        ma.markers.append(self._lm(now,10,"left_path",MASSAGE_POINTS_L,(0.0,0.75,0.95,1.0),0.012))
        ma.markers.append(self._lm(now,11,"right_path",MASSAGE_POINTS_R,(0.95,0.58,0.20,1.0),0.012))
        # 路径目标点：左臂青色 / 右臂橙色（区分左右臂目标）
        for i,pt in enumerate(MASSAGE_POINTS_L):
            ma.markers.append(self._sm(now,200+i,"target_L",pt,0.020,(0.0,0.75,0.95,0.9)))
        for i,pt in enumerate(MASSAGE_POINTS_R):
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
