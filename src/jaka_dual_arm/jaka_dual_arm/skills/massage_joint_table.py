#!/usr/bin/env python3
"""Joint-space reference table for direct massage execution.
┌──────────────────────────────────────────────────────────────┐
│  替代 OMPL 规划层的轻量方案:                                   │
│  1. 启动时: 对每个 (zone, position, technique)                 │
│     调用 MoveIt IK → 缓存关节角                                │
│  2. 按摩时: 查表 → plan_joint_target_direct → 执行            │
├──────────────────────────────────────────────────────────────┤
│  保留: BackSurfaceModel / PathGenerator / OMPL 规划          │
│  用户后续设 _table_mode=False 可切换回原链路                  │
└──────────────────────────────────────────────────────────────┘

用法:
    from jaka_dual_arm.skills.massage_joint_table import (
        compute_joint_reference_table,
        get_joint_target,
    )
    table = compute_joint_reference_table(planner, body_cfg)
    target = get_joint_target(table, stage_def, "left")
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion


# ═══════════════════════════════════════════════════════════════
# Zone 参数 (独立于 body_params.yaml)
# 从 massage_body_params.yaml 提取 — 如有更新需手动同步
# ═══════════════════════════════════════════════════════════════

ZONE_PARAMS = {
    'C7':       {'x': 0.38, 'z_surface': 0.197, 'half_w': 0.10},
    'shoulder': {'x': 0.47, 'z_surface': 0.194, 'half_w': 0.16},
    'upper':    {'x': 0.55, 'z_surface': 0.189, 'half_w': 0.14},
    'mid':      {'x': 0.63, 'z_surface': 0.185, 'half_w': 0.12},
    'lower_th': {'x': 0.70, 'z_surface': 0.180, 'half_w': 0.11},
    'lumbar':   {'x': 0.77, 'z_surface': 0.175, 'half_w': 0.10},
    'sacrum':   {'x': 0.84, 'z_surface': 0.172, 'half_w': 0.12},
}

# 横向位置 → y 偏移 (相对于 zone half_w 的系数)
POSITION_Y_FACTOR = {
    'L': -0.8,   # 左侧 (负 y)
    'C': 0.0,    # 脊柱中线
    'R': 0.8,    # 右侧 (正 y)
}

HOVER_Z_OFFSET = 0.015  # hover 比 press 高 1.5cm
ACU_Y_OFFSET = 0.04     # 穴位旁开 4cm


# ═══════════════════════════════════════════════════════════════
# IK 辅助
# ═══════════════════════════════════════════════════════════════

def _make_pose_stamped(x: float, y: float, z: float) -> PoseStamped:
    """创建 PoseStamped, 工具指向下方 (IK 会做微调)."""
    ps = PoseStamped()
    ps.header.frame_id = "world"
    ps.pose.position = Point(x=x, y=y, z=z)
    # 通用"指向下"四元数 — IK 配合种子会找到附近有效构型
    ps.pose.orientation = Quaternion(x=0.0, y=0.7071, z=0.0, w=0.7071)
    return ps


def _try_ik(planner, x: float, y: float, z: float,
            arm: str, label: str) -> Optional[list[float]]:
    """调用一次 IK 并记录结果."""
    ps = _make_pose_stamped(x, y, z)
    result = planner.compute_ik(ps, f"{arm}_arm", timeout_sec=1.0)
    if result:
        planner.get_logger().info(
            f"  IK OK: {label} = "
            f"[{', '.join(f'{v:.3f}' for v in result)}]"
        )
    else:
        planner.get_logger().warn(
            f"  IK FAIL: {label} at ({x:.3f}, {y:.3f}, {z:.3f})"
        )
    return result


# ═══════════════════════════════════════════════════════════════
# 主入口
# ═══════════════════════════════════════════════════════════════

def compute_joint_reference_table(planner, body_cfg: dict) -> dict:
    """启动时通过 MoveIt IK 构建关节参考表.

    表结构:
        Zone press:  ("zone", zone, position, arm) → [j1..j6]
        Zone hover:  ("zone", zone, position, arm, "hover") → [j1..j6]
        Acu press:   ("acu", idx, arm) → [j1..j6]
        Acu hover:   ("acu", idx, arm, "hover") → [j1..j6]

    Args:
        planner: DualArmPlannerServer (必须已连接 /compute_ik 服务)
        body_cfg: massage_body_params.yaml 字典

    Returns:
        dict: 关节参考表, 部分 key 可能为 None (IK 失败)
    """
    # 合并 YAML 中的 zone 参数 (覆盖默认值)
    zones = dict(ZONE_PARAMS)
    cfg_zones = body_cfg.get('massage_zones', {})
    for zname, zdata in cfg_zones.items():
        if zname in zones:
            zones[zname] = {
                'x': zdata.get('x', zones[zname]['x']),
                'z_surface': zdata.get('z_surface', zones[zname]['z_surface']),
                'half_w': zdata.get('half_w', zones[zname]['half_w']),
            }

    table: dict = {}
    expected = 0

    # ── Zone 条目 ──
    for zname, zdata in zones.items():
        for position in ['L', 'C', 'R']:
            y = POSITION_Y_FACTOR[position] * zdata['half_w']
            for arm in ['left', 'right']:
                y_arm = -y if arm == 'right' else y  # 右臂 y 对称镜像

                # Press (表面接触)
                label = f"{zname} {position} {arm} press"
                ik = _try_ik(planner, zdata['x'], y_arm,
                             zdata['z_surface'], arm, label)
                table[("zone", zname, position, arm)] = ik
                expected += 1

                # Hover (高于表面 1.5cm)
                label_h = f"{zname} {position} {arm} hover"
                ik_h = _try_ik(planner, zdata['x'], y_arm,
                               zdata['z_surface'] + HOVER_Z_OFFSET,
                               arm, label_h)
                table[("zone", zname, position, arm, "hover")] = ik_h

    # ── 穴位条目 ──
    acupoints = body_cfg.get('acupoints', [])
    acu_y_off = body_cfg.get('acupoint_y_offset', ACU_Y_OFFSET)
    for idx, acu in enumerate(acupoints):
        acu_x = acu.get('x', 0.5)
        acu_z = acu.get('z', 0.18)
        for arm in ['left', 'right']:
            y_arm = -acu_y_off if arm == 'left' else acu_y_off

            label = f"acu_{idx} {arm} press"
            ik = _try_ik(planner, acu_x, y_arm, acu_z, arm, label)
            table[("acu", idx, arm)] = ik
            expected += 1

            label_h = f"acu_{idx} {arm} hover"
            ik_h = _try_ik(planner, acu_x, y_arm, acu_z + HOVER_Z_OFFSET,
                           arm, label_h)
            table[("acu", idx, arm, "hover")] = ik_h

    success = sum(1 for v in table.values() if v is not None)
    planner.get_logger().info(
        f"Joint table built: {success}/{expected} IK solutions OK"
    )
    return table


def get_joint_target(table: dict, stage_def: dict,
                     arm: str) -> Optional[list[float]]:
    """从表中查一个臂的阶段关节目标.

    Args:
        table: compute_joint_reference_table() 返回的表
        stage_def: 阶段定义中 left/right 字典,
                   包含 zone/position/technique 或 acupoint/technique
        arm: "left" 或 "right"

    Returns:
        [j1..j6] 或 None (IK 失败或表中无此条目)
    """
    technique = stage_def.get("technique", "press")
    is_hover = technique in ("hover", "release")

    if "acupoint" in stage_def:
        idx = int(stage_def["acupoint"])
        if is_hover:
            return table.get(("acu", idx, arm, "hover"))
        return table.get(("acu", idx, arm))

    zone = stage_def.get("zone", "")
    position = stage_def.get("position", "C")
    if is_hover:
        return table.get(("zone", zone, position, arm, "hover"))
    return table.get(("zone", zone, position, arm))
