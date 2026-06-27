#!/usr/bin/env python3
"""Path Generator — 背部曲面模型 + 13种手法 Cartesian 路径生成.

Layer 3: 背部曲面模型 (Catmull-Rom 样条 → 参数化曲面 S(u,v))
Layer 4: 轨迹生成 (13种手法 → 8种 Cartesian 路径图元)

用法:
    surface = BackSurfaceModel(body_params)
    gen = PathGenerator(surface)
    poses = gen.generate({"zone": "C7", "position": "C", "technique": "press"})
    # poses: list[Pose] — 可直接传给 MoveIt2 Cartesian Path 规划

参考:
  - 中医推拿学 (第10版) — 手法分类与力学特征
  - Catmull & Rom (1974) — A Class of Local Interpolating Splines
  - MoveIt2 /compute_cartesian_path — Cartesian 路径规划接口
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Point, Pose, Quaternion


# ═══════════════════════════════════════════════════════════════
# Catmull-Rom Spline 脊柱曲线
# ═══════════════════════════════════════════════════════════════

class CatmullRomSpline:
    """Catmull-Rom 样条插值器 — 从 N 个控制点生成光滑脊柱曲线 C(u).

    Catmull-Rom 保证曲线通过所有控制点，且 C¹ 连续。
    使用弧长参数化，保证 u ∈ [0,1] 均匀对应曲线距离。
    """

    def __init__(self, control_points: List[Tuple[float, float]]):
        """
        Args:
            control_points: [(x0, z0), (x1, z1), ...]
        """
        if len(control_points) < 2:
            raise ValueError(f"Need at least 2 control points, got {len(control_points)}")
        self._pts = control_points
        self._n = len(control_points)

        # 预计算弧长参数化表
        self._table: List[Tuple[float, float, float, float]] = []  # (u_seg, arc, x, z)
        self._total_arc: float = 0.0
        self._build_arc_table(1000)

    def _build_arc_table(self, samples: int):
        """Build arc-length parameterization table."""
        self._table = []
        total = 0.0
        du = 1.0 / samples

        for i in range(samples + 1):
            u_seg = i * du
            x, z = self._evaluate_raw(u_seg)
            if i > 0:
                prev_x, prev_z = self._table[-1][2], self._table[-1][3]
                total += math.sqrt((x - prev_x) ** 2 + (z - prev_z) ** 2)
            self._table.append((u_seg, total, x, z))

        self._total_arc = total

    def _evaluate_raw(self, u: float) -> Tuple[float, float]:
        """Evaluate Catmull-Rom at uniform segment parameter u ∈ [0,1].

        P(t) = 0.5 * [ (2P₁)
                     + (-P₀ + P₂) t
                     + (2P₀ - 5P₁ + 4P₂ - P₃) t²
                     + (-P₀ + 3P₁ - 3P₂ + P₃) t³ ]
        """
        n = self._n
        float_idx = u * (n - 1)
        i = int(float_idx)
        if i >= n - 1:
            i = n - 2
        t = float_idx - i

        p0 = self._pts[max(0, i - 1)]
        p1 = self._pts[i]
        p2 = self._pts[min(n - 1, i + 1)]
        p3 = self._pts[min(n - 1, i + 2)]

        t2 = t * t
        t3 = t2 * t

        # Basis weights
        w0 = -0.5 * t + t2 - 0.5 * t3
        w1 = 1.0 - 2.5 * t2 + 1.5 * t3
        w2 = 0.5 * t + 2.0 * t2 - 1.5 * t3
        w3 = -0.5 * t2 + 0.5 * t3

        x = w0 * p0[0] + w1 * p1[0] + w2 * p2[0] + w3 * p3[0]
        z = w0 * p0[1] + w1 * p1[1] + w2 * p2[1] + w3 * p3[1]
        return (x, z)

    def evaluate(self, u: float) -> Tuple[float, float]:
        """Evaluate at arc-length parameter u ∈ [0,1].

        Returns (x, z) in world coordinates.
        """
        u = max(0.0, min(1.0, u))
        target = u * self._total_arc

        # Binary search in arc table
        lo, hi = 0, len(self._table) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self._table[mid][1] < target:
                lo = mid + 1
            else:
                hi = mid
        return (self._table[lo][2], self._table[lo][3])

    def derivative(self, u: float, eps: float = 0.0001) -> Tuple[float, float]:
        """Numerical derivative dC/du at arc-length parameter u."""
        u = max(eps, min(1.0 - eps, u))
        x1, z1 = self.evaluate(u - eps)
        x2, z2 = self.evaluate(u + eps)
        return ((x2 - x1) / (2 * eps), (z2 - z1) / (2 * eps))

    @property
    def total_arc_length(self) -> float:
        return self._total_arc

    @property
    def control_points(self) -> List[Tuple[float, float]]:
        return list(self._pts)


# ═══════════════════════════════════════════════════════════════
# 背部曲面模型
# ═══════════════════════════════════════════════════════════════

class BackSurfaceModel:
    """背部参数化曲面 S(u,v).

    S(u,v) = ( x(u),  v·w(u),  z(u) − d_c·v² )

    where:
        u ∈ [0,1]  — 沿脊柱 (0=头部, 1=骶骨)
        v ∈ [-1,1] — 横向 (-1=左边缘, 0=脊柱线, 1=右边缘)
        w(u)       — 横向半宽 (12段线性插值)
        d_c        — 背部横向曲率深度 (可配, 默认 0.025m)
    """

    # 7 个按摩区域 → u 坐标范围
    ZONE_U_RANGES: Dict[str, Tuple[float, float]] = {
        "C7":       (0.00, 0.08),
        "shoulder": (0.08, 0.18),
        "upper":    (0.18, 0.38),
        "mid":      (0.38, 0.52),
        "lower_th": (0.52, 0.64),
        "lumbar":   (0.64, 0.82),
        "sacrum":   (0.82, 1.00),
    }

    def __init__(self, body_params: dict):
        """
        Args:
            body_params: massage_body_params.yaml 加载的字典
        """
        self._cfg = body_params
        self._curvature_depth = body_params.get("curvature_depth", 0.025)

        # 构建脊柱样条
        segments = body_params.get("body_segments", [])
        if len(segments) < 2:
            raise ValueError(f"Need at least 2 body_segments, got {len(segments)}")

        spine_pts = [(seg["x"], seg["z"]) for seg in segments]
        self._spine = CatmullRomSpline(spine_pts)
        self._half_widths = [seg["half_w"] for seg in segments]

        # 构建穴位映射
        self._acupoint_map = self._build_acupoint_map(body_params)

    # ── 穴位映射 ──

    def _build_acupoint_map(self, cfg: dict) -> dict:
        """将穴位 x 坐标投影到脊柱曲线上，得到 (u, v_L, v_R)。"""
        acupoints = cfg.get("acupoints", [])
        y_offset = cfg.get("acupoint_y_offset", 0.04)

        mapping = {}
        for idx, acu in enumerate(acupoints):
            u = self._x_to_u(acu["x"])
            w = self.half_width(u)
            v = y_offset / w if w > 0 else 0.3
            mapping[idx] = {
                "name": acu["name"],
                "u": u,
                "v_L": -v,
                "v_R": v,
                "x": acu["x"],
                "z": acu["z"],
            }
        return mapping

    def _x_to_u(self, x: float) -> float:
        """将世界 X 坐标投影到脊柱曲线上，返回弧长参数 u。"""
        best_u = 0.5
        best_err = float("inf")
        for i in range(1000):
            u = i / 999.0
            sx, _ = self._spine.evaluate(u)
            err = abs(sx - x)
            if err < best_err:
                best_err = err
                best_u = u
        return best_u

    # ── Public API ──

    def evaluate(self, u: float, v: float) -> Tuple[float, float, float]:
        """计算曲面坐标 S(u,v) → (x, y, z) 世界坐标。"""
        u = max(0.0, min(1.0, u))
        v = max(-1.0, min(1.0, v))

        x_u, z_u = self._spine.evaluate(u)
        w = self.half_width(u)
        return (x_u, v * w, z_u - self._curvature_depth * v * v)

    def normal(self, u: float, v: float) -> Tuple[float, float, float]:
        """曲面法向量 — 指向体外 (大致 +Z 方向).

        n = normalize(∂S/∂v × ∂S/∂u)
        """
        u = max(0.001, min(0.999, u))
        v = max(-0.999, min(0.999, v))

        dx_du, dz_du = self._spine.derivative(u)
        dw_du = self._half_width_derivative(u)
        w = self.half_width(u)

        # ∂S/∂u
        dS_du = (dx_du, v * dw_du, dz_du)
        # ∂S/∂v
        dS_dv = (0.0, w, -2.0 * self._curvature_depth * v)

        # Cross: dS/dv × dS/du
        nx = dS_dv[1] * dS_du[2] - dS_dv[2] * dS_du[1]
        ny = dS_dv[2] * dS_du[0] - dS_dv[0] * dS_du[2]
        nz = dS_dv[0] * dS_du[1] - dS_dv[1] * dS_du[0]

        length = math.sqrt(nx * nx + ny * ny + nz * nz)
        if length < 1e-10:
            return (0.0, 0.0, 1.0)
        return (nx / length, ny / length, nz / length)

    def tangent_u(self, u: float, v: float) -> Tuple[float, float, float]:
        """脊柱方向切向量 ∂S/∂u (归一化)。"""
        dx_du, dz_du = self._spine.derivative(u)
        dw_du = self._half_width_derivative(u)

        tx, ty, tz = dx_du, v * dw_du, dz_du
        length = math.sqrt(tx * tx + ty * ty + tz * tz)
        if length < 1e-10:
            return (1.0, 0.0, 0.0)
        return (tx / length, ty / length, tz / length)

    def half_width(self, u: float) -> float:
        """横向半宽 w(u) — 12段线性插值。"""
        u = max(0.0, min(1.0, u))
        n = len(self._half_widths)
        float_idx = u * (n - 1)
        i = int(float_idx)
        if i >= n - 1:
            return self._half_widths[-1]
        t = float_idx - i
        return self._half_widths[i] * (1 - t) + self._half_widths[i + 1] * t

    def _half_width_derivative(self, u: float, eps: float = 0.0001) -> float:
        u = max(eps, min(1.0 - eps, u))
        return (self.half_width(u + eps) - self.half_width(u - eps)) / (2 * eps)

    def zone_urange(self, zone: str) -> Tuple[float, float]:
        """获取按摩区域的 u 参数范围。"""
        return self.ZONE_U_RANGES.get(zone, (0.0, 1.0))

    def acupoint_uv(self, acu_idx: int, side: str = "L") -> Tuple[float, float]:
        """获取穴位的 (u, v) 参数坐标。

        Args:
            acu_idx: 穴位索引 (0-6)
            side: "L" (左侧, v<0) 或 "R" (右侧, v>0)
        """
        info = self._acupoint_map.get(acu_idx, {})
        u = info.get("u", 0.5)
        v = info.get("v_L", -0.3) if side == "L" else info.get("v_R", 0.3)
        return (u, v)

    @property
    def spine(self) -> CatmullRomSpline:
        return self._spine


# ═══════════════════════════════════════════════════════════════
# Pose 构造工具
# ═══════════════════════════════════════════════════════════════

def _quaternion_from_axes(
    x_axis: Tuple[float, float, float],
    z_axis: Tuple[float, float, float],
) -> Quaternion:
    """从 X 轴和 Z 轴方向构造四元数 (Y = Z × X)。

    末端坐标系:
        Z = 指向体内 (按摩接触方向，≈ -法向量)
        X = 沿脊柱方向 (头部→骶骨)
        Y = 右手定则
    """
    # 归一化 Z
    z_len = math.sqrt(z_axis[0]**2 + z_axis[1]**2 + z_axis[2]**2)
    if z_len < 1e-10:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    zx, zy, zz = z_axis[0] / z_len, z_axis[1] / z_len, z_axis[2] / z_len

    # Gram-Schmidt: X ⟂ Z
    x_len = math.sqrt(x_axis[0]**2 + x_axis[1]**2 + x_axis[2]**2)
    if x_len < 1e-10:
        # Fallback: pick arbitrary X ⟂ Z
        if abs(zx) < 0.9:
            tx, ty, tz = 1.0, 0.0, 0.0
        else:
            tx, ty, tz = 0.0, 1.0, 0.0
    else:
        tx, ty, tz = x_axis[0] / x_len, x_axis[1] / x_len, x_axis[2] / x_len

    dot = tx * zx + ty * zy + tz * zz
    tx -= dot * zx
    ty -= dot * zy
    tz -= dot * zz
    x_len2 = math.sqrt(tx**2 + ty**2 + tz**2)
    if x_len2 < 1e-10:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    xx, xy, xz = tx / x_len2, ty / x_len2, tz / x_len2

    # Y = Z × X
    yx = zy * xz - zz * xy
    yy = zz * xx - zx * xz
    yz = zx * xy - zy * xx

    # 旋转矩阵 → 四元数
    # R = [X|Y|Z] (column-major)
    m00, m10, m20 = xx, xy, xz
    m01, m11, m21 = yx, yy, yz
    m02, m12, m22 = zx, zy, zz

    tr = m00 + m11 + m22
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif m00 > m11 and m00 > m22:
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def _make_pose(
    position: Tuple[float, float, float],
    normal: Tuple[float, float, float],   # 曲面法向量 (指向体外)
    tangent: Tuple[float, float, float],  # 脊柱切向量 (头部→骶骨)
    z_offset: float = 0.0,                # 深度偏移 (负=压入体内)
) -> Pose:
    """构造按摩末端 Pose.

    末端 Z 轴 = -normal (指向体内, 按摩接触方向)
    末端 X 轴 = tangent_u (沿脊柱)
    位置沿 -normal 方向偏移 z_offset (z_offset<0 表示压入体内)
    """
    # 位置偏移: + normal * z_offset
    #   z_offset < 0 (press) → 沿 -normal 方向 → 压入体内 ✓
    #   z_offset > 0 (hover)  → 沿 +normal 方向 → 抬高离开体表 ✓
    px = position[0] + normal[0] * z_offset
    py = position[1] + normal[1] * z_offset
    pz = position[2] + normal[2] * z_offset

    # 末端 Z 轴 = -normal (指向体内)
    z_axis = (-normal[0], -normal[1], -normal[2])
    x_axis = tangent

    quat = _quaternion_from_axes(x_axis, z_axis)

    pose = Pose()
    pose.position = Point(x=px, y=py, z=pz)
    pose.orientation = quat
    return pose


# ═══════════════════════════════════════════════════════════════
# 路径图元 (Path Primitives)
# ═══════════════════════════════════════════════════════════════

def _path_stationary(u: float, v: float, n_pts: int = 1) -> List[Tuple[float, float]]:
    """驻点 — hover/press/deep_press/vibrate/release."""
    return [(u, v)] * max(n_pts, 1)


def _path_line(u_start: float, u_end: float, v: float,
               n_pts: int = 30) -> List[Tuple[float, float]]:
    """直线 — 沿 u 方向等间距采样 (推法)."""
    return [(u_start + (u_end - u_start) * i / max(n_pts - 1, 1), v)
            for i in range(n_pts)]


def _path_oscillate(u_start: float, u_end: float, v: float,
                    cycles: int = 3, n_pts: int = 60) -> List[Tuple[float, float]]:
    """往复振荡 — 沿 u 方向来回 (擦法/摩法).

    每个 cycle: 前进 (u_start→u_end) + 返回 (u_end→u_start).
    """
    points = []
    pts_per_half = n_pts // (cycles * 2)
    for c in range(cycles):
        # Forward
        for i in range(pts_per_half):
            t = i / max(pts_per_half - 1, 1)
            u = u_start + (u_end - u_start) * t
            points.append((u, v))
        # Backward
        for i in range(pts_per_half):
            t = i / max(pts_per_half - 1, 1)
            u = u_end - (u_end - u_start) * t
            points.append((u, v))
    return points[:n_pts]


def _path_circle(u_center: float, v_center: float,
                 radius: float = 0.012, n_pts: int = 36,
                 n_cycles: int = 3) -> List[Tuple[float, float]]:
    """圆周 — 在 (u,v) 切平面内画圆 (按揉)."""
    total_pts = n_pts * n_cycles
    return [(u_center + radius * math.cos(2 * math.pi * i / n_pts),
             v_center + radius * math.sin(2 * math.pi * i / n_pts))
            for i in range(total_pts)]


def _path_sinusoid(u_start: float, u_end: float, v: float,
                   amplitude_v: float = 0.015, wavelength: float = 0.06,
                   n_pts: int = 80) -> List[Tuple[float, float]]:
    """正弦波 — 沿 u 行进 + 横向正弦摆动 (滚揉法)."""
    points = []
    for i in range(n_pts):
        u = u_start + (u_end - u_start) * i / max(n_pts - 1, 1)
        v_osc = v + amplitude_v * math.sin(2 * math.pi * u / wavelength)
        points.append((u, v_osc))
    return points


def _path_spiral(u_center: float, v_center: float,
                 max_radius: float = 0.03, n_pts: int = 72,
                 n_turns: int = 3) -> List[Tuple[float, float]]:
    """阿基米德螺旋 — 从中心向外扩展 (摩法)."""
    return [(u_center + max_radius * t * math.cos(2 * math.pi * n_turns * t),
             v_center + max_radius * t * math.sin(2 * math.pi * n_turns * t))
            for t in (i / max(n_pts - 1, 1) for i in range(n_pts))]


def _path_discrete(u_start: float, u_end: float, v: float,
                   spacing: float = 0.025,
                   n_pulses: int = 1) -> List[Tuple[float, float]]:
    """离散点列 — 沿 u 等间距分布 (击法)."""
    u_range = u_end - u_start
    if u_range <= 0:
        return [(u_start, v)] * n_pulses
    n_pts = max(2, int(u_range / spacing) + 1)
    result = []
    for i in range(n_pts):
        u = u_start + u_range * i / (n_pts - 1)
        for _ in range(max(n_pulses, 1)):
            result.append((u, v))
    return result


# ═══════════════════════════════════════════════════════════════
# 手法 → 路径图元 + 参数配置
# ═══════════════════════════════════════════════════════════════

# 手法配置: z_offset (深度偏移, 负=压入) + 路径图元 + 参数
TECHNIQUE_CONFIG: Dict[str, dict] = {
    # ── 驻点类 ──
    "hover":      {"z_offset":  0.040, "primitive": "stationary", "params": {}},
    "press":      {"z_offset": -0.015, "primitive": "stationary", "params": {}},
    "deep_press": {"z_offset": -0.030, "primitive": "stationary", "params": {}},
    "release":    {"z_offset":  0.040, "primitive": "stationary", "params": {}},
    "vibrate":    {"z_offset":  0.000, "primitive": "stationary", "params": {"n_pts": 10}},

    # ── 圆周类 ──
    "knead_L": {"z_offset": -0.010, "primitive": "circle",
                "params": {"radius": 0.012, "n_pts": 36, "n_cycles": 3}},
    "knead_R": {"z_offset": -0.010, "primitive": "circle",
                "params": {"radius": 0.012, "n_pts": 36, "n_cycles": 3}},

    # ── 往复振荡类 ──
    "rub_L":  {"z_offset": 0.0, "primitive": "oscillate",
               "params": {"cycles": 3, "n_pts": 60}},
    "rub_R":  {"z_offset": 0.0, "primitive": "oscillate",
               "params": {"cycles": 3, "n_pts": 60}},
    "scrub":  {"z_offset": 0.0, "primitive": "oscillate",
               "params": {"cycles": 5, "n_pts": 100}},

    # ── 正弦波类 ──
    "roll":   {"z_offset": 0.0, "primitive": "sinusoid",
               "params": {"amplitude_v": 0.015, "wavelength": 0.06, "n_pts": 80}},

    # ── 离散点列类 ──
    "tap":    {"z_offset": -0.005, "primitive": "discrete",
               "params": {"spacing": 0.025, "n_pulses": 2}},
    "strike": {"z_offset": -0.010, "primitive": "discrete",
               "params": {"spacing": 0.030, "n_pulses": 1}},

    # ── 波浪类 (v5.0 新增) ──
    "wave":   {"z_offset": -0.005, "primitive": "oscillate",
               "params": {"cycles": 3, "n_pts": 24}},
}

# position → v 侧偏
POSITION_V_MAP: Dict[str, float] = {
    "L": -0.6,
    "C":  0.0,
    "R":  0.6,
}


# ═══════════════════════════════════════════════════════════════
# Path Generator — 主接口
# ═══════════════════════════════════════════════════════════════

class PathGenerator:
    """按摩路径生成器 — 阶段定义 + 背部曲面 → Cartesian 位姿序列.

    用法:
        surface = BackSurfaceModel(body_params)
        gen = PathGenerator(surface)
        poses = gen.generate({"zone": "C7", "position": "C", "technique": "press"})
    """

    def __init__(self, surface: BackSurfaceModel):
        self._surface = surface

    def generate(self, stage_def: dict) -> List[Pose]:
        """为单个按摩阶段生成 Cartesian 位姿序列.

        Args:
            stage_def: {"zone"/"acupoint", "position", "technique"}
                       来自 massage_stages.yaml 的 left/right 字段

        Returns:
            list[Pose] — 可用于 MoveIt2 Cartesian Path 规划
        """
        technique = stage_def.get("technique", "hover")
        position = stage_def.get("position", "C")

        if technique not in TECHNIQUE_CONFIG:
            raise ValueError(
                f"Unknown technique: {technique}. "
                f"Available: {list(TECHNIQUE_CONFIG.keys())}"
            )

        cfg = TECHNIQUE_CONFIG[technique]
        z_offset = cfg["z_offset"]

        # 确定 (u,v) 参数域
        if "acupoint" in stage_def:
            # 穴位模式: 精确 (u,v)
            acu_idx = stage_def["acupoint"]
            side = "L" if position == "L" else "R"
            u_center, v_center = self._surface.acupoint_uv(acu_idx, side)
            u_start = u_end = u_center
        else:
            # 区域模式: zone → u_range, position → v
            zone = stage_def.get("zone", "mid")
            u_start, u_end = self._surface.zone_urange(zone)
            u_center = (u_start + u_end) / 2.0
            v_center = POSITION_V_MAP.get(position, 0.0)

        # 生成 (u,v) 采样点
        uv_points = self._generate_uv_points(
            cfg["primitive"],
            u_start, u_end, u_center, v_center,
            cfg["params"],
        )

        # (u,v) → Pose
        return [self._uv_to_pose(u, v, z_offset) for u, v in uv_points]

    def _generate_uv_points(self, primitive: str,
                            u_start: float, u_end: float,
                            u_center: float, v_center: float,
                            params: dict) -> List[Tuple[float, float]]:
        """分发到具体路径图元函数。"""
        if primitive == "stationary":
            return _path_stationary(u_center, v_center, **params)
        elif primitive == "line":
            return _path_line(u_start, u_end, v_center, **params)
        elif primitive == "oscillate":
            return _path_oscillate(u_start, u_end, v_center, **params)
        elif primitive == "circle":
            return _path_circle(u_center, v_center, **params)
        elif primitive == "sinusoid":
            return _path_sinusoid(u_start, u_end, v_center, **params)
        elif primitive == "spiral":
            return _path_spiral(u_center, v_center, **params)
        elif primitive == "discrete":
            return _path_discrete(u_start, u_end, v_center, **params)
        else:
            return [(u_center, v_center)]

    def _uv_to_pose(self, u: float, v: float, z_offset: float) -> Pose:
        """(u,v) 参数坐标 → 按摩末端 Pose."""
        pos = self._surface.evaluate(u, v)
        normal = self._surface.normal(u, v)
        tangent = self._surface.tangent_u(u, v)
        return _make_pose(pos, normal, tangent, z_offset)

    @property
    def surface(self) -> BackSurfaceModel:
        return self._surface
