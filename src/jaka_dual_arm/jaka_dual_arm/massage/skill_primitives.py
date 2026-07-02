#!/usr/bin/env python3
"""技能原语库 — 按摩手法的结构化定义.

将按摩动作抽象为可复用的参数化技能原语。每个原语封装了:
  - 轨迹形状 (路径图元类型)
  - 深度/力度参数
  - 时长参数
  - 分类标签 (用于过渡决策)

设计原则:
  1. 纯数据模块, 无ROS依赖 — 旧demo和工业系统共用
  2. 每个原语有明确的"类别" — 编排器据此决定阶段间过渡方式
  3. 原语的technique名与两套系统的执行层兼容:
     - 旧demo: ACT_DELTA[technique] → 关节角偏移
     - 工业系统: TECHNIQUE_CONFIG[technique] → Cartesian路径参数

用法:
    from jaka_dual_arm.massage.skill_primitives import SkillLibrary
    lib = SkillLibrary()
    prim = lib.get("press")
    print(prim.category)  # "stationary"
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


# ═══════════════════════════════════════════════════════════════
# 原语分类
# ═══════════════════════════════════════════════════════════════

class PrimitiveCategory(Enum):
    """技能原语的运动类别 — 编排器用于过渡决策."""
    STATIONARY  = "stationary"   # 驻点: press, hover, vibrate
    CIRCULAR    = "circular"     # 圆周: knead_L/R, rub_L/R
    LINEAR      = "linear"       # 直线: line_press, line_knead, scrub
    VIBRATORY   = "vibratory"    # 振荡: vibrate, tap, strike, wave
    DISCRETE    = "discrete"     # 离散点: tap, strike (多点敲击)


# ═══════════════════════════════════════════════════════════════
# 技能原语数据类
# ═══════════════════════════════════════════════════════════════

@dataclass
class SkillPrimitive:
    """单个按摩技能原语.

    Attributes:
        name: 唯一标识名 (如 "press", "knead_L")
        display_name: 中文显示名 (如 "按法", "揉法左旋")
        technique: 底层手法名 — 同时映射到:
            - 旧demo ACT_DELTA[technique]
            - 工业系统 TECHNIQUE_CONFIG[technique]
        category: 运动类别 — 编排器过渡决策依据
        z_offset: 默认深度偏移(m), 负=压入体内
        default_duration: 默认单次执行时长(s)
        force_level: 力度级别 0.0(悬停) ~ 1.0(深压)
        multi_cycle: 是否多周期手法 (需要展开为子阶段)
        description: 手法说明
    """
    name: str
    display_name: str
    technique: str
    category: PrimitiveCategory
    z_offset: float = 0.0
    default_duration: float = 1.5
    force_level: float = 0.0
    multi_cycle: bool = False
    description: str = ""

    # 可选: 自定义路径参数 (覆盖 TECHNIQUE_CONFIG 默认值)
    path_params: Dict[str, Any] = field(default_factory=dict)

    # 可选: 旧demo专用 — 关节角偏移 [Δj2,Δj3,Δj4,Δj5,Δj6]
    joint_delta: Optional[List[float]] = None


# ═══════════════════════════════════════════════════════════════
# 技能原语库
# ═══════════════════════════════════════════════════════════════

class SkillLibrary:
    """技能原语注册表 — 按名称查询原语定义.

    用法:
        lib = SkillLibrary()
        prim = lib.get("press")
        all_stationary = lib.by_category(PrimitiveCategory.STATIONARY)
    """

    def __init__(self):
        self._primitives: Dict[str, SkillPrimitive] = {}
        self._register_defaults()

    def _register_defaults(self):
        """注册所有内置按摩技能原语."""
        prims = [

            # ── 驻点类 (STATIONARY) ──
            SkillPrimitive(
                name="hover", display_name="悬停", technique="hover",
                category=PrimitiveCategory.STATIONARY,
                z_offset=0.040, default_duration=0.8, force_level=0.0,
                joint_delta=[0.0, 0.0, 0.0, 0.0, 0.0],
                description="悬停在体表上方, 不接触",
            ),
            SkillPrimitive(
                name="press", display_name="按法", technique="press",
                category=PrimitiveCategory.STATIONARY,
                z_offset=-0.015, default_duration=2.0, force_level=0.4,
                joint_delta=[0.02, -0.04, 0.03, 0.0, 0.0],
                description="驻点按压, 沉入皮下1.5cm",
            ),
            SkillPrimitive(
                name="deep_press", display_name="深按", technique="deep_press",
                category=PrimitiveCategory.STATIONARY,
                z_offset=-0.030, default_duration=3.0, force_level=0.7,
                joint_delta=[0.03, -0.06, 0.05, 0.0, 0.0],
                description="深压松解深层肌筋膜, 沉入3cm",
            ),
            SkillPrimitive(
                name="release", display_name="释放", technique="release",
                category=PrimitiveCategory.STATIONARY,
                z_offset=0.040, default_duration=0.5, force_level=0.0,
                joint_delta=[0.0, 0.0, 0.0, 0.0, 0.0],
                description="快速释放压力, 回到悬停高度",
            ),

            # ── 圆周类 (CIRCULAR) ──
            SkillPrimitive(
                name="knead_L", display_name="揉法左旋", technique="knead_L",
                category=PrimitiveCategory.CIRCULAR,
                z_offset=-0.010, default_duration=3.0, force_level=0.3,
                multi_cycle=True,
                joint_delta=[0.02, -0.04, 0.03, 0.0, 0.08],
                description="圆周按揉(左旋), 放松浅层肌筋膜",
            ),
            SkillPrimitive(
                name="knead_R", display_name="揉法右旋", technique="knead_R",
                category=PrimitiveCategory.CIRCULAR,
                z_offset=-0.010, default_duration=3.0, force_level=0.3,
                multi_cycle=True,
                joint_delta=[0.02, -0.04, 0.03, 0.0, -0.08],
                description="圆周按揉(右旋)",
            ),
            SkillPrimitive(
                name="knead_wL", display_name="大揉搓左旋", technique="knead_wL",
                category=PrimitiveCategory.CIRCULAR,
                z_offset=-0.010, default_duration=4.0, force_level=0.4,
                multi_cycle=True,
                joint_delta=[0.02, -0.04, 0.03, 0.0, 0.20],
                description="大幅揉搓, 4点圆周+回中, 半径2cm",
            ),
            SkillPrimitive(
                name="knead_wR", display_name="大揉搓右旋", technique="knead_wR",
                category=PrimitiveCategory.CIRCULAR,
                z_offset=-0.010, default_duration=4.0, force_level=0.4,
                multi_cycle=True,
                joint_delta=[0.02, -0.04, 0.03, 0.0, -0.20],
                description="大幅揉搓右旋",
            ),
            SkillPrimitive(
                name="rub_L", display_name="摩法左旋", technique="rub_L",
                category=PrimitiveCategory.CIRCULAR,
                z_offset=0.0, default_duration=2.5, force_level=0.2,
                multi_cycle=True,
                joint_delta=[0.01, -0.02, 0.04, 0.0, 0.06],
                description="腕部圆周揉摩",
            ),
            SkillPrimitive(
                name="rub_R", display_name="摩法右旋", technique="rub_R",
                category=PrimitiveCategory.CIRCULAR,
                z_offset=0.0, default_duration=2.5, force_level=0.2,
                multi_cycle=True,
                joint_delta=[0.01, -0.02, 0.04, 0.0, -0.06],
                description="腕部圆周揉摩右旋",
            ),

            # ── 振荡/振动类 (VIBRATORY) ──
            SkillPrimitive(
                name="vibrate", display_name="振法", technique="vibrate",
                category=PrimitiveCategory.VIBRATORY,
                z_offset=0.0, default_duration=2.0, force_level=0.1,
                joint_delta=[0.002, -0.002, 0.002, 0.0, 0.003],
                description="高频微幅振动, 松解筋膜粘连, 振后需settle等待",
            ),
            SkillPrimitive(
                name="wave", display_name="波浪", technique="wave",
                category=PrimitiveCategory.VIBRATORY,
                z_offset=-0.005, default_duration=4.0, force_level=0.2,
                multi_cycle=True,
                joint_delta=[0.0, 0.0, 0.02, 0.02, 0.04],
                description="Y向正弦振荡+Z起伏, 3周期×8步, 末尾回中",
            ),
            SkillPrimitive(
                name="tap", display_name="拍法", technique="tap",
                category=PrimitiveCategory.DISCRETE,
                z_offset=-0.005, default_duration=3.0, force_level=0.2,
                multi_cycle=True,
                joint_delta=[0.0, -0.05, 0.03, 0.0, 0.0],
                description="5次hover↔press交替拍击, ~0.9Hz",
            ),
            SkillPrimitive(
                name="strike", display_name="击法", technique="strike",
                category=PrimitiveCategory.DISCRETE,
                z_offset=-0.010, default_duration=2.0, force_level=0.4,
                multi_cycle=True,
                joint_delta=[0.0, -0.06, 0.04, 0.0, 0.0],
                description="重击敲打",
            ),
            SkillPrimitive(
                name="pound", display_name="捶打", technique="pound",
                category=PrimitiveCategory.DISCRETE,
                z_offset=-0.010, default_duration=3.0, force_level=0.5,
                multi_cycle=True,
                joint_delta=[0.04, -0.08, 0.06, 0.0, 0.0],
                description="3次hover↔strike交替捶打, ~0.67Hz",
            ),

            # ── 直线类 (LINEAR) — 复合原语, 横跨多个区域 ──
            SkillPrimitive(
                name="line_press", display_name="推法(直线按压)", technique="press",
                category=PrimitiveCategory.LINEAR,
                z_offset=-0.015, default_duration=8.0, force_level=0.4,
                description="沿脊柱方向直线移动+按压, 每区驻点按压",
                path_params={"primitive": "line", "n_pts": 30},
            ),
            SkillPrimitive(
                name="line_knead", display_name="揉法(直线按揉)", technique="knead_L",
                category=PrimitiveCategory.LINEAR,
                z_offset=-0.010, default_duration=12.0, force_level=0.3,
                multi_cycle=True,
                description="沿脊柱方向直线移动+圆周按揉",
                path_params={"primitive": "line", "n_pts": 20},
            ),
            SkillPrimitive(
                name="scrub", display_name="擦法", technique="scrub",
                category=PrimitiveCategory.LINEAR,
                z_offset=0.0, default_duration=5.0, force_level=0.3,
                multi_cycle=True,
                joint_delta=[0.03, -0.05, 0.04, 0.0, 0.0],
                description="往返直线推擦, 5个周期",
            ),

            # ── 弧线类 ──
            SkillPrimitive(
                name="arc_L", display_name="弧线左扫", technique="arc_L",
                category=PrimitiveCategory.LINEAR,
                z_offset=-0.015, default_duration=3.0, force_level=0.3,
                multi_cycle=True,
                joint_delta=[0.01, -0.02, 0.04, 0.0, 0.10],
                description="cosine弧线扫过背部(左旋), 5个插值点",
            ),
            SkillPrimitive(
                name="arc_R", display_name="弧线右扫", technique="arc_R",
                category=PrimitiveCategory.LINEAR,
                z_offset=-0.015, default_duration=3.0, force_level=0.3,
                multi_cycle=True,
                joint_delta=[0.01, -0.02, 0.04, 0.0, -0.10],
                description="cosine弧线扫过背部(右旋)",
            ),
        ]

        for prim in prims:
            self._primitives[prim.name] = prim

    # ── Public API ──

    def get(self, name: str) -> Optional[SkillPrimitive]:
        """按名称查询原语. 不存在返回None."""
        return self._primitives.get(name)

    def require(self, name: str) -> SkillPrimitive:
        """按名称查询原语. 不存在抛出KeyError."""
        prim = self._primitives.get(name)
        if prim is None:
            available = list(self._primitives.keys())
            raise KeyError(
                f"Unknown skill primitive: '{name}'. "
                f"Available: {available}"
            )
        return prim

    def by_category(self, category: PrimitiveCategory) -> List[SkillPrimitive]:
        """按类别筛选原语."""
        return [p for p in self._primitives.values()
                if p.category == category]

    def list_all(self) -> List[str]:
        """列出所有原语名称."""
        return list(self._primitives.keys())

    def register(self, prim: SkillPrimitive):
        """注册自定义原语 (覆盖同名已有原语)."""
        self._primitives[prim.name] = prim

    @property
    def count(self) -> int:
        return len(self._primitives)


# ═══════════════════════════════════════════════════════════════
# 按摩区域配置 (纯数据, 无ROS依赖)
# ═══════════════════════════════════════════════════════════════

# 7个标准按摩区域 — 与 massage_body_params.yaml 保持一致
ZONE_ORDER = ["C7", "shoulder", "upper", "mid", "lower_th", "lumbar", "sacrum"]

# 区域中文名
ZONE_NAMES: Dict[str, str] = {
    "C7": "C7隆椎",
    "shoulder": "肩胛带",
    "upper": "上背(T1-8)",
    "mid": "中背(T9-L1)",
    "lower_th": "下胸",
    "lumbar": "腰椎",
    "sacrum": "骶骨",
}

# 端点区域 (需要双臂交叉越障)
ENDPOINT_ZONES = {"C7", "sacrum"}

# 安全躯干区 (不交叉, 臂在各自半背操作)
SAFE_ZONES = ["shoulder", "upper", "mid", "lower_th", "lumbar"]

# 膀胱经穴位列表
ACUPOINT_NAMES = [
    "BL11_大杼", "BL13_肺俞", "BL15_心俞", "BL17_膈俞",
    "BL18_肝俞", "BL23_肾俞", "BL25_大肠俞",
]
