# 当前生产机械臂网格（2026-08-27 11:43 机械交付）

本目录基于机械 2026-08-27 重新导出的 SolidWorks URDF 包和同批 STEP，
不混用历史图纸或历史网格。所有网格单位均为米，Xacro 缩放均为 `1 1 1`。

机械插件导出的刚体归属存在一处已确认错误：`Link4.STL` 只有 J4 附近的
小组件，而约 320 mm 的长小臂被放进 `_Link5.STL`。这会使长小臂错误地
绕 J5 转动，并让 J3 看起来只带动很小的零件。

当前 `Link4.STL`、`Link5.STL` 已从同批 STEP 重新分组：

- `Link4`：J4 输出端、完整长小臂、J5 电机定子及其上游壳体；
- `Link5`：J5 输出转轴、轴承盖、J6 限位与 J6 电机定子；
- 坐标不使用历史手填原点，而是读取同批 SolidWorks URDF 的 J1～J6
  `origin xyz/rpy`，换算到各关节 `q=0` 的局部坐标系；
- 可复现工具：`src/fruit_picking_arm/tools/step_xcaf_to_robot_meshes.py`
  的 `--reference-urdf` 模式；
- STEP 原文件 SHA256：
  `3EE25040E1A5CB3C782282DDF8FF4A082D32737723EF411026FBAAB5E855EEB7`。

交付包：`src/机械臂总装配体 (2).zip`

交付包 SHA256：
`582B591E31555A277DBDA9D17C8CE2015B527757DA0F7C33A5D10C8548631CE4`

独立核对 STEP：`src/机械臂总装配体（stp）.zip`

STEP 包 SHA256：
`CF375EB548ABABFD6A24B3BCEDFA2192272EAB16A8CF2DE22F68D1B79C83A394`

## 当前网格哈希

- `base_link.STL`: `02D7CC924C27AFB71FF48B21EBC3C96DC0E9C1636D7B30377A4166F3B922FE61`
- `Link1.STL`: `690C5030B6E9AA392D099A92735CA1E02C17E49C160CCC0E310490249F50D251`
- `Link2.STL`: `32A979F73E2793B099E21BBBCD868E0259BB6E74A96359F7A37611AECF4BD635`
- `Link3.STL`: `7D929DB48D58B15AF51EC884C422AF149D16ED7B29A4CB93979F22CE4C71D5F4`
- `Link4.STL`（同批 STEP 刚体归属修复）: `EDBA6D6673924084186C3D0A8F180293C3B57D8CE9675D39D4023D5B0C5B37B8`
- `Link5.STL`（同批 STEP 刚体归属修复）: `767833E991CCD06585AEC7C9AFAF88D43D514840E6AD006E45F268018BAFC20A`
- `Link6.STL`（移除误归入 J6 的整套夹爪）: `52E44F228C999D3EDF44AB3EF6A9DD06E3BED6879A60EC212A428709C16C7A73`

## J6/夹爪分离修复

旧分类器只识别夹指零件号 `FAEF86`，没有识别夹爪总成名
`FAE4M86M+D`，因此夹爪电机、壳体和内部连杆被错误合并进
`Link6.STL`。2026-08-27 修复分类器后，从同批 STEP 和同批机械 URDF
关节坐标重新生成且只替换 `Link6.STL`：

- J6 叶子零件数从 35 降为 28；
- 夹爪总成 11 个叶子零件全部从 J6 分离；
- `Link4.STL`、`Link5.STL` 重生成哈希与生产文件完全一致，生产目录中的
  Link3/Link5 哈希也在替换前后复核未改变；
- 修复工具仍只负责网格刚体分组，不重新推导 J1～J6 运动学。

## 四指夹爪网格与端点运动学

`../gripper_current/` 由机械交付的同坐标系闭合/张开 STEP 生成：

- 闭合 STEP SHA256：
  `A7C5413DA7E481D396F5760720687D96E2E4EBD55CE057DFA8E2FE3426B777DA`；
- 张开 STEP SHA256：
  `A895636BBA5340A006905EED80CD2D02D67E741397AF91AF307A0140D837589D`；
- 3 个固定零件、1 个中心滑块、4 个驱动连杆、8 个夹指叶子零件，
  输出为 10 个非空活动/固定 STL；
- 闭合到张开的刚体变换拟合残差小于 `1.5e-13 mm`；
- J6 到 `GRIPPER_MOUNT`：`xyz = 0.0422 0 0 m`；
- `GRIPPER_MOUNT` 到 TCP：`xyz = 0 0 0.124 m`；
- 完整数值、源文件哈希和端点说明见
  `../gripper_current/KINEMATICS.json`；可复现工具为
  `src/fruit_picking_arm/tools/build_actual_gripper_meshes.py`。

真实丝杆/连杆的中间运动是非线性的，仅凭两份静态 STEP 无法恢复完整运动
函数。当前 URDF 保证闭合、张开两端准确，中间姿态用 mimic 线性插值，只用于
显示和保守碰撞近似；没有把最后一次命令伪装成实测夹爪反馈。

## 仍需后续确认

1. SolidWorks 插件仍把多个 lower/upper 反写，并把 J3/J6 行程写短；代码侧按已确认机械标准修正，不能直接采用交付 URDF 的 limit 数值。
2. J4 导出轴为 `0.99996 0 0.0087265`，约倾斜 `0.5°`，仍需机械确认是否设计值。
3. J6 与 GRIPPER_MOUNT 的轴向装配方向已由图纸几何对齐；因为四指机构近似
   四重对称，绕工具轴的坐标相位仍应在机械坐标轴截图到齐后做最终复核。
4. 夹爪中间姿态是两端 STEP 的线性 mimic 近似；如需精确动态碰撞，机械仍需
   提供丝杆位移到各活动刚体角度的曲线或多个中间配置。
5. 当前碰撞使用完整三角网格，后续应生成经验证的简化凸碰撞网格。
6. Link4/Link5 及夹爪活动体的视觉和碰撞刚体归属已修复，但修复后的质量、质心、
   惯量还没有由 SolidWorks 按相同刚体分组重新导出；这不影响 RViz 和
   MoveIt 几何规划，但在启用动力学仿真前必须重新校核。
