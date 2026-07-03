#!/usr/bin/env python3
"""按摩动作编排器 — 高层描述 → 带过渡的阶段序列.

将 MassagePattern (高层描述) 展开为可执行阶段列表, 自动在阶段间
插入过渡原语, 确保动作流程平滑、自然。

编排流程:
  1. 加载 YAML pattern → MassagePattern 对象
  2. 展开每个 Movement → StageDef 列表
  3. 相邻阶段之间 → 自动决定并插入 TransitionPrimitive
  4. 输出完整的 [StageDef] — 两套系统可直接消费

输出兼容:
  旧demo:    stage.to_legacy_tuple() → ((zone,pos,crossed), tech, (zone,pos,crossed), tech, name)
  工业系统:  stage.to_dict() → {"zone":"C7", "position":"L", "technique":"press"}

用法:
    from jaka_dual_arm.massage.choreographer import (
        MassagePattern, MassageChoreographer
    )
    from jaka_dual_arm.massage.skill_primitives import SkillLibrary

    lib = SkillLibrary()
    ch = MassageChoreographer(lib)

    pattern = ch.load_pattern("推法开背")
    stages = ch.compose(pattern)
    # stages 是一个 StageDef 列表, 包含按摩阶段和过渡阶段
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from jaka_dual_arm.massage.skill_primitives import (
    SkillPrimitive, SkillLibrary, PrimitiveCategory,
    ZONE_ORDER, ENDPOINT_ZONES, SAFE_ZONES,
)
from jaka_dual_arm.massage.transition_primitives import (
    TransitionType, TransitionPrimitive,
    decide_transition, compute_transition, transition_between,
)


# ═══════════════════════════════════════════════════════════════
# 数据模型
# ═══════════════════════════════════════════════════════════════

class StageKind(Enum):
    """阶段类型."""
    MASSAGE    = "massage"     # 按摩阶段
    TRANSITION = "transition"  # 过渡阶段


class Side(Enum):
    """操作侧别."""
    LEFT  = "L"
    RIGHT = "R"
    CENTER = "C"
    BOTH  = "B"   # 双臂同区对称


@dataclass
class StageDef:
    """单个阶段定义 — 编排器输出单元.

    同时兼容两套系统:
      - 旧demo: 调用 to_legacy_tuple() 得到原有格式
      - 工业系统: 调用 to_dict() / to_dict_left() / to_dict_right()
    """
    kind: StageKind = StageKind.MASSAGE
    stage_id: int = 0
    name: str = ""

    # ── 按摩阶段专用 ──
    skill_name: str = ""          # 技能原语名 (如 "press", "knead_L")
    zone: str = ""                # 区域名 (如 "C7", "mid")
    side: str = "C"               # L / C / R / B
    crossed: bool = False         # 是否交叉越障
    acupoint_idx: Optional[int] = None  # 穴位索引 (0-6), None=区域模式

    # 双阶段拆分 (同区双臂对称操作时, left≠right)
    dual_side: bool = False       # True时 left_spec≠right_spec

    # ── 过渡阶段专用 ──
    transition: Optional[TransitionPrimitive] = None

    def to_dict(self) -> Dict[str, Any]:
        """转为工业系统兼容的 dict 格式."""
        d: Dict[str, Any] = {}
        if self.acupoint_idx is not None:
            d["acupoint"] = self.acupoint_idx
        else:
            d["zone"] = self.zone
        d["position"] = self.side
        d["technique"] = self.skill_name
        return d

    def to_dict_left(self) -> Dict[str, Any]:
        """左臂的 dict (与to_dict相同, 显式接口)."""
        if self.dual_side:
            d = self.to_dict()
            d["position"] = "L_SAFE"
            return d
        return self.to_dict()

    def to_dict_right(self) -> Dict[str, Any]:
        """右臂的 dict — 处理对称映射."""
        d: Dict[str, Any] = {}
        if self.acupoint_idx is not None:
            d["acupoint"] = self.acupoint_idx
        else:
            d["zone"] = self.zone

        # 侧别映射: L↔R 翻转。dual_side 使用固定安全车道,
        # 左臂守左车道, 右臂守右车道, 不做交叉/中线覆盖。
        if self.dual_side:
            d["position"] = "R_SAFE"
        elif self.side == "L":
            d["position"] = "R"
        elif self.side == "R":
            d["position"] = "L"
        else:
            d["position"] = self.side

        # 手法映射: 左右旋翻转
        tech = self.skill_name
        if tech.endswith("_L"):
            tech = tech[:-2] + "_R"
        elif tech.endswith("_R"):
            tech = tech[:-2] + "_L"
        d["technique"] = tech
        return d

    def to_legacy_tuple(self):
        """转为旧demo兼容的元组格式.

        Returns:
            (left_spec, left_act, right_spec, right_act, name)
            其中 left_spec=(zone, pos[, crossed]) 或 (acu_idx, "acu")
        """
        # 左臂spec
        if self.acupoint_idx is not None:
            left_spec = (self.acupoint_idx, "acu")
        elif self.dual_side:
            left_spec = (self.zone, "L", False)
        else:
            left_spec = (self.zone, self.side, self.crossed)

        # 右臂spec — 对称映射
        if self.acupoint_idx is not None:
            right_spec = (self.acupoint_idx, "acu")
        elif self.dual_side:
            right_spec = (self.zone, "R", False)
        else:
            right_side = {"L": "R", "R": "L"}.get(self.side, self.side)
            right_spec = (self.zone, right_side, self.crossed)

        # 手法映射
        left_act = self.skill_name
        right_act = self.skill_name
        if self.skill_name.endswith("_L"):
            right_act = self.skill_name[:-2] + "_R"
        elif self.skill_name.endswith("_R"):
            right_act = self.skill_name[:-2] + "_L"

        return (left_spec, left_act, right_spec, right_act, self.name)


@dataclass
class MassageMovement:
    """单个按摩动作 — pattern中的一个步骤.

    Attributes:
        skill: 技能原语名 (如 "press", "knead_L", "line_press")
        zones: 覆盖的区域列表 (如 ["C7"] 或 ["C7","shoulder",...,"sacrum"])
        side: 操作侧别 "L"/"R"/"C"/"B"
        acupoints: 穴位索引列表 (如 [0,1,2,3,4,5,6]), None=区域模式
        depth_override: 覆盖原语的z_offset (None=用默认)
        duration_override: 覆盖原语的时长 (None=用默认)
        crossed: 是否交叉越障
    """
    skill: str
    zones: List[str] = field(default_factory=list)
    side: str = "C"
    acupoints: Optional[List[int]] = None
    depth_override: Optional[float] = None
    duration_override: Optional[float] = None
    crossed: bool = False

    def __post_init__(self):
        # 默认区域: 所有7个区
        if not self.zones and self.acupoints is None:
            self.zones = list(ZONE_ORDER)


@dataclass
class MassagePattern:
    """高层按摩模式定义.

    Attributes:
        name: 模式名称 (如 "推法开背")
        description: 描述文字
        movements: 动作序列
    """
    name: str
    description: str = ""
    movements: List[MassageMovement] = field(default_factory=list)


# ═══════════════════════════════════════════════════════════════
# 编排器
# ═══════════════════════════════════════════════════════════════

class MassageChoreographer:
    """按摩动作编排器 — pattern → stages + transitions.

    用法:
        lib = SkillLibrary()
        ch = MassageChoreographer(lib)
        pattern = ch.load_pattern("推法开背")
        stages = ch.compose(pattern)
    """

    # 末端区域列表 (C7和骶骨需要双臂交叉越障)
    ENDPOINT_ZONES = ENDPOINT_ZONES

    def __init__(self, skill_lib: Optional[SkillLibrary] = None):
        self._lib = skill_lib or SkillLibrary()
        self._lift_height = 0.04  # 默认抬升高度

    # ── 核心 API ──

    def compose(self, pattern: MassagePattern) -> List[StageDef]:
        """将 massage pattern 展开为带过渡的阶段序列.

        Args:
            pattern: 高层按摩模式

        Returns:
            [StageDef, ...] — 按摩阶段和过渡阶段交替排列
        """
        if not pattern.movements:
            return []

        all_stages: List[StageDef] = []
        stage_counter = 0

        for mov in pattern.movements:
            # 展开单个 movement → 多个基础 stage
            mov_stages = self._expand_movement(mov, stage_counter)
            stage_counter += len(mov_stages)

            # 在前后 movement 之间自动插入过渡
            if all_stages and mov_stages:
                prev = all_stages[-1]
                next_first = mov_stages[0]
                trans = self._insert_transition(prev, next_first, stage_counter)
                if trans:
                    all_stages.append(trans)
                    stage_counter += 1

            all_stages.extend(mov_stages)

        # 最后一个阶段之后: 收尾悬停过渡
        if all_stages:
            final = StageDef(
                kind=StageKind.TRANSITION,
                stage_id=stage_counter,
                name="收尾悬停",
                transition=compute_transition(
                    TransitionType.LIFT_APPROACH,
                    prev_z=0.0, next_z=0.04,
                    lift_height=self._lift_height,
                    description="收尾→安全悬停",
                ),
            )
            all_stages.append(final)

        return all_stages

    def compose_legacy(self, pattern: MassagePattern) -> List[Tuple]:
        """输出旧demo兼容的元组列表.

        Returns:
            [(left_spec, left_act, right_spec, right_act, name), ...]
        """
        stages = self.compose(pattern)
        result = []
        for s in stages:
            if s.kind == StageKind.MASSAGE:
                result.append(s.to_legacy_tuple())
        return result

    # ── Movement 展开 ──

    def _expand_movement(self, mov: MassageMovement,
                         start_id: int) -> List[StageDef]:
        """将一个 movement 展开为多个 StageDef.

        例如: "推法开背 L侧" → 7个press stage (C7→骶骨, 每个一个stage)
        """
        stages = []
        prim = self._lib.require(mov.skill)

        dual_side = mov.side.upper() in ("B", "BOTH", "DUAL")

        # 穴位模式
        if mov.acupoints is not None:
            for i, acu_idx in enumerate(mov.acupoints):
                crossed = False if dual_side else self._is_crossed(
                    zone="", acupoint_idx=acu_idx, side=mov.side
                )
                sd = StageDef(
                    kind=StageKind.MASSAGE,
                    stage_id=start_id + i,
                    name=(
                        f"{prim.display_name}·穴位{acu_idx}"
                        f"{'(协同)' if dual_side else ''}"
                    ),
                    skill_name=mov.skill,
                    acupoint_idx=acu_idx,
                    side="B" if dual_side else mov.side,
                    crossed=crossed,
                    dual_side=dual_side,
                )
                # 深度覆盖
                if mov.depth_override is not None:
                    sd.skill_name = mov.skill  # keep skill, depth override handled by executor
                stages.append(sd)
            return stages

        # 区域模式
        if not mov.zones:
            mov.zones = list(ZONE_ORDER)

        for i, zone in enumerate(mov.zones):
            crossed = False if dual_side else (
                mov.crossed or self._is_crossed(zone, side=mov.side)
            )
            sd = StageDef(
                kind=StageKind.MASSAGE,
                stage_id=start_id + i,
                name=(
                    f"{prim.display_name}·{zone}"
                    f"{'(协同)' if dual_side else ('(交叉)' if crossed else '')}"
                ),
                skill_name=mov.skill,
                zone=zone,
                side="B" if dual_side else mov.side,
                crossed=crossed,
                dual_side=dual_side,
            )
            stages.append(sd)

        return stages

    def _is_crossed(self, zone: str, acupoint_idx: Optional[int] = None,
                    side: str = "C") -> bool:
        """判断是否需要双臂交叉越障."""
        if side == "C":
            return False
        if acupoint_idx is not None:
            # 穴位0(BL11大杼, 近C7) 和 穴位6(BL25大肠俞, 近骶骨) 可能需要交叉
            return acupoint_idx in (0, 6)
        return zone in self.ENDPOINT_ZONES and side != "C"

    # ── 过渡插入 ──

    def _insert_transition(self, prev: StageDef, next_stage: StageDef,
                           stage_id: int) -> Optional[StageDef]:
        """在两个按摩stage之间自动插入过渡.

        Args:
            prev: 前一个阶段
            next_stage: 后一个阶段
            stage_id: 过渡阶段的ID

        Returns:
            Transition StageDef, 或 None (不需要过渡)
        """
        # 获取前后阶段的原语
        try:
            prev_prim = self._lib.require(prev.skill_name)
        except KeyError:
            prev_prim = self._lib.require("hover")
        try:
            next_prim = self._lib.require(next_stage.skill_name)
        except KeyError:
            next_prim = self._lib.require("hover")

        # 判断区域和侧别是否相同
        same_zone = (
            prev.zone == next_stage.zone
            and prev.acupoint_idx == next_stage.acupoint_idx
        )
        same_side = prev.side == next_stage.side

        # 调用过渡原语
        trans = transition_between(
            prev_prim=prev_prim,
            next_prim=next_prim,
            same_zone=same_zone,
            same_side=same_side,
            is_cross_over=(prev.crossed or next_stage.crossed),
            lift_height=self._lift_height,
        )

        # 直接过渡 → 不需要插入额外的过渡stage
        if trans.trans_type == TransitionType.DIRECT:
            return None

        return StageDef(
            kind=StageKind.TRANSITION,
            stage_id=stage_id,
            name=f"过渡:{trans.description}",
            transition=trans,
        )

    # ── YAML 加载 ──

    def load_pattern(self, name: str,
                     yaml_path: Optional[str] = None) -> MassagePattern:
        """从 YAML 文件加载按摩模式.

        Args:
            name: 模式名称 (如 "推法开背")
            yaml_path: YAML文件路径, None=使用默认config目录

        Returns:
            MassagePattern 对象
        """
        import os
        import yaml

        if yaml_path is None:
            # 默认路径: jaka_dual_arm/config/massage_patterns.yaml
            share_dir = self._find_share_dir()
            yaml_path = os.path.join(share_dir, "config", "massage_patterns.yaml")
            if not os.path.exists(yaml_path):
                # 开发模式回退
                src_root = os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.dirname(__file__))))
                yaml_path = os.path.join(
                    src_root, "src", "jaka_dual_arm", "config",
                    "massage_patterns.yaml"
                )

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(
                f"Pattern YAML not found: {yaml_path}. "
                f"Checked both install share and source tree."
            )

        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        patterns = data.get("patterns", [])
        for p in patterns:
            if p.get("name") == name:
                return self._parse_pattern(p)

        available = [p.get("name", "?") for p in patterns]
        raise KeyError(
            f"Pattern '{name}' not found in {yaml_path}. "
            f"Available: {available}"
        )

    def list_patterns(self, yaml_path: Optional[str] = None) -> List[str]:
        """列出所有可用的 pattern 名称."""
        import os
        import yaml

        if yaml_path is None:
            share_dir = self._find_share_dir()
            yaml_path = os.path.join(share_dir, "config", "massage_patterns.yaml")
            if not os.path.exists(yaml_path):
                src_root = os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.dirname(__file__))))
                yaml_path = os.path.join(
                    src_root, "src", "jaka_dual_arm", "config",
                    "massage_patterns.yaml"
                )

        if not os.path.exists(yaml_path):
            return []

        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        return [p.get("name", "?") for p in data.get("patterns", [])]

    def _parse_pattern(self, data: dict) -> MassagePattern:
        """解析 YAML dict → MassagePattern."""
        movements = []
        for m in data.get("movements", []):
            mov = MassageMovement(
                skill=m.get("skill", "hover"),
                zones=m.get("zones", []),
                side=m.get("side", "C"),
                acupoints=m.get("acupoints"),
                depth_override=m.get("depth_override"),
                duration_override=m.get("duration_override"),
                crossed=m.get("crossed", False),
            )
            movements.append(mov)
        return MassagePattern(
            name=data.get("name", "unnamed"),
            description=data.get("description", ""),
            movements=movements,
        )

    def _find_share_dir(self) -> str:
        """查找 install share 目录."""
        try:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory("jaka_dual_arm")
        except Exception:
            import os
            return os.path.join(
                os.path.dirname(__file__), "..", "..", "..",
                "install", "jaka_dual_arm", "share", "jaka_dual_arm"
            )
