#!/usr/bin/env python3
"""过渡原语 — 按摩阶段之间的平滑衔接.

自动判断并生成阶段间过渡:
  - FORCE_RAMP:   同区域深度变化 → 逐步改变z_offset
  - SURFACE_SLIDE: 同区域横向移动 → 沿体表滑动
  - LIFT_APPROACH: 跨区域移动 → 抬离→空中平移→接近
  - SETTLE_WAIT:   振法之后 → 等待残余振荡衰减
  - BLEND:         通用位姿混合 → 线性插值

设计原则:
  1. 纯数据模块, 无ROS依赖
  2. 输入是前后两个SkillPrimitive + 区域信息, 输出是过渡stage列表
  3. 对旧demo (关节角delta) 和工业系统 (Cartesian路径) 都适用

用法:
    from jaka_dual_arm.massage.transition_primitives import (
        TransitionType, transition_between
    )
    transition = transition_between(prev_prim, next_prim, same_zone=True, depth_changed=True)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple


# ═══════════════════════════════════════════════════════════════
# 过渡类型
# ═══════════════════════════════════════════════════════════════

class TransitionType(Enum):
    """过渡类型枚举."""
    NONE           = "none"            # 无需过渡 (实际上不会用到)
    BLEND          = "blend"           # 通用插值混合
    FORCE_RAMP     = "force_ramp"      # 力度渐变 (同点, 改变下压深度)
    SURFACE_SLIDE  = "surface_slide"   # 沿体表滑动 (同区, 横向移动)
    LIFT_APPROACH  = "lift_approach"   # 抬升→平移→下降 (跨区域)
    SETTLE_WAIT    = "settle_wait"     # 振后衰减等待
    CROSS_OVER     = "cross_over"      # 双臂交叉越障 (端点区域)
    DIRECT         = "direct"          # 直接过渡 (无需特殊处理)


# ═══════════════════════════════════════════════════════════════
# 过渡原语
# ═══════════════════════════════════════════════════════════════

@dataclass
class TransitionPrimitive:
    """单个过渡阶段.

    Attributes:
        trans_type: 过渡类型
        description: 描述文字 (日志用)
        waypoints: 中间路点列表, 每个元素是 (u, v, z_offset) 或 None=用target端点的值
            - 旧demo: 忽略uv, 只用z_offset, waypoints数量决定插值步数
            - 工业系统: uv用于在曲面模型上采样
        duration: 过渡总时长(s)
        z_profile: 深度变化曲线 — [(ratio, z_offset), ...] 其中ratio∈[0,1]
    """
    trans_type: TransitionType
    description: str = ""
    waypoints: List[Tuple[Optional[float], Optional[float], Optional[float]]] = field(default_factory=list)
    duration: float = 1.0
    z_profile: List[Tuple[float, float]] = field(default_factory=list)  # [(ratio, z_offset)]


# ═══════════════════════════════════════════════════════════════
# 过渡决策
# ═══════════════════════════════════════════════════════════════

def decide_transition(
    prev_category: str,
    next_category: str,
    prev_z: float,
    next_z: float,
    same_zone: bool = True,
    same_side: bool = True,
    is_cross_over: bool = False,
) -> TransitionType:
    """根据前后阶段属性自动决定过渡类型.

    Args:
        prev_category: 前一个原语的类别 ("stationary"/"circular"/"vibratory"/"linear")
        next_category: 后一个原语的类别
        prev_z: 前一个阶段的z_offset (mm or m, 负=压入)
        next_z: 后一个阶段的z_offset
        same_zone: 是否在同一按摩区域
        same_side: 是否在同一侧(L/C/R)
        is_cross_over: 是否需要双臂交叉越障

    Returns:
        推荐的过渡类型
    """
    # ── 交叉越障 → 特殊过渡 ──
    if is_cross_over:
        return TransitionType.CROSS_OVER

    # ── 振法之后 → 等待振荡衰减 ──
    if prev_category == "vibratory":
        return TransitionType.SETTLE_WAIT

    # ── 同区域 ──
    if same_zone:
        # 深度变化 → 力渐变
        if abs(prev_z - next_z) > 0.002:
            return TransitionType.FORCE_RAMP

        # 横向移动 → 沿面滑动
        if not same_side:
            return TransitionType.SURFACE_SLIDE

        # 同点同深 → 直接过渡
        return TransitionType.DIRECT

    # ── 不同区域 → 抬升接近 ──
    return TransitionType.LIFT_APPROACH


def compute_transition(
    trans_type: TransitionType,
    prev_z: float = 0.0,
    next_z: float = 0.0,
    lift_height: float = 0.04,
    description: str = "",
) -> TransitionPrimitive:
    """根据过渡类型生成过渡阶段.

    Args:
        trans_type: 过渡类型
        prev_z: 起始深度偏移
        next_z: 目标深度偏移
        lift_height: 抬升高度 (LIFT_APPROACH用)
        description: 描述文字

    Returns:
        TransitionPrimitive 实例
    """
    if trans_type == TransitionType.DIRECT:
        return TransitionPrimitive(
            trans_type=TransitionType.DIRECT,
            description=description or "直接过渡",
            duration=0.3,
        )

    elif trans_type == TransitionType.FORCE_RAMP:
        # 渐变深度: 分5步从prev_z到next_z
        steps = 5
        z_profile = [
            (i / (steps - 1), prev_z + (next_z - prev_z) * i / (steps - 1))
            for i in range(steps)
        ]
        return TransitionPrimitive(
            trans_type=TransitionType.FORCE_RAMP,
            description=description or f"力度渐变 z={prev_z:.3f}→{next_z:.3f}",
            z_profile=z_profile,
            duration=1.5,
        )

    elif trans_type == TransitionType.SURFACE_SLIDE:
        # 沿体表滑动: 3个中间点
        return TransitionPrimitive(
            trans_type=TransitionType.SURFACE_SLIDE,
            description=description or "沿体表横向滑动",
            waypoints=[(None, None, prev_z)] * 2 + [(None, None, next_z)],
            duration=1.0,
        )

    elif trans_type == TransitionType.LIFT_APPROACH:
        # 三段式: 抬离体表 → 空中平移 → 接近新目标
        z_profile = [
            (0.0, prev_z),        # 起点: 当前深度
            (0.3, lift_height),   # 30%时间: 抬到悬停高度
            (0.7, lift_height),   # 70%时间: 保持悬停(空中平移)
            (1.0, next_z),        # 终点: 降到目标深度
        ]
        return TransitionPrimitive(
            trans_type=TransitionType.LIFT_APPROACH,
            description=description or f"抬升接近({lift_height:.3f}m)",
            z_profile=z_profile,
            duration=2.0,
        )

    elif trans_type == TransitionType.SETTLE_WAIT:
        # 振后等待: 延迟0.5s让残余振荡衰减
        return TransitionPrimitive(
            trans_type=TransitionType.SETTLE_WAIT,
            description=description or "振后衰减等待",
            z_profile=[(0.0, prev_z), (1.0, prev_z)],
            duration=0.5,
        )

    elif trans_type == TransitionType.CROSS_OVER:
        # 双臂交叉越障: 交给执行层处理 (旧demo的HIGH_HOVER逻辑)
        return TransitionPrimitive(
            trans_type=TransitionType.CROSS_OVER,
            description=description or "双臂交叉越障",
            z_profile=[
                (0.0, prev_z),
                (0.25, lift_height),
                (0.75, lift_height),
                (1.0, next_z),
            ],
            duration=3.0,
        )

    else:
        # 默认: 直接过渡
        return TransitionPrimitive(
            trans_type=TransitionType.DIRECT,
            description=description or "默认直接过渡",
            duration=0.3,
        )


def transition_between(
    prev_prim: Any,  # SkillPrimitive (避免循环import)
    next_prim: Any,
    same_zone: bool = True,
    same_side: bool = True,
    is_cross_over: bool = False,
    lift_height: float = 0.04,
) -> TransitionPrimitive:
    """一站式计算两个按摩阶段之间的过渡.

    Args:
        prev_prim: 前一个 SkillPrimitive
        next_prim: 后一个 SkillPrimitive
        same_zone: 同一区域?
        same_side: 同一侧?
        is_cross_over: 交叉越障?
        lift_height: 抬升高度

    Returns:
        TransitionPrimitive
    """
    trans_type = decide_transition(
        prev_category=prev_prim.category.value,
        next_category=next_prim.category.value,
        prev_z=prev_prim.z_offset,
        next_z=next_prim.z_offset,
        same_zone=same_zone,
        same_side=same_side,
        is_cross_over=is_cross_over,
    )

    desc = f"[{prev_prim.display_name}]→[{next_prim.display_name}] {trans_type.value}"

    return compute_transition(
        trans_type=trans_type,
        prev_z=prev_prim.z_offset,
        next_z=next_prim.z_offset,
        lift_height=lift_height,
        description=desc,
    )
