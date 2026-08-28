# SolidWorks 到 URDF/MoveIt 的可验证流程

## 结论

生产 URDF 不再由 STEP 零件名称、人工关节原点和肉眼截图拼接。STEP 用于
独立核对几何；关节刚体归属、轴线、装配坐标系和机械零位必须来自完整
SolidWorks 总装的配合关系与明确基准。本项目 2026-08-27 起只接受下述新坐标
契约，旧的 J1 ±165°、J2 144°、J3 ±90° 等定义全部作废。

| 关节 | CAD/机械零点 | URDF 有效范围 |
|---|---|---|
| J1 | 总装坐标系 +X 为 0 | `[-90°, +90°]` |
| J2 | 紧贴限位块为 0 | `[0°, +145°]` |
| J3 | 紧贴限位块为 0 | `[-180°, 0°]` |
| J4 | J4 坐标系 +Z 为 0 | `[-165°, +165°]` |
| J5 | J5 坐标系 +X 为 0 | `[-90°, +90°]` |
| J6 | J6 坐标系 +X 为 0 | `[-180°, +180°]` |

正方向由 URDF `<axis>` 定义。URDF 永远要求 `lower <= upper`；把上下限倒写
不是表达反转方向的方法。

项目原工具 `tools/step_xcaf_to_robot_meshes.py` 只保留为旧 STL 的审计/复现
工具。它不读取配合，不能证明 URDF 或 MoveIt 运动学正确，默认会拒绝运行。

## 机械交付要求

SolidWorks Pack and Go 必须同时满足：

1. 总装存在并保存配置 `初始位置`，J1～J6 均位于上表约定的 `0 rad` 姿态。
2. 总装层级存在 `Origin_global`、`J1坐标系`～`J6坐标系` 以及
   `TCP坐标系`（或 `TCP_URDF`）。
3. `Origin_global` 使用 ROS 约定：+X 正前、+Y 向左、+Z 向上。
4. 每个 `J*坐标系` 原点位于实际旋转轴中心；单独的参考轴 J1～J6 指定
   URDF `<axis>` 的正向。
5. `TCP坐标系` 位于真实工具中心，方向由机械和视觉共同确认。
6. 从 `初始位置` 配置另存 `初始位置.STEP`，用于与提取结果做独立几何核对。
7. 限位表明确写出每轴下限、上限、单位、正方向，并与上表一致，不从模型
   截图反推。

只有默认装配、只有 STEP，或者缺少 TCP，均不满足生产抓取模型的交付条件。

## 开源提取器

使用 `jsk-ros-pkg/solidworks_urdf_exporter2`（命令/包名 `sw2robot`）。提取阶段
通过 SolidWorks COM 读取组件、配合、坐标系、参考轴、质量属性和网格，所以
必须在安装 SolidWorks 的 Windows 电脑运行。后续编辑、构建和验证不需要
SolidWorks。

在 SolidWorks 电脑上安装 Python 3.12+ 后：

```powershell
git clone https://github.com/jsk-ros-pkg/solidworks_urdf_exporter2.git
cd solidworks_urdf_exporter2
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e .

.\.venv\Scripts\python.exe -m sw2robot.exporter.extract `
  "D:\交付目录\机械臂总装配体.SLDASM" `
  -o "D:\交付目录\sw2robot_output" `
  -n fruit_arm `
  --configuration "初始位置" `
  --visible
```

也可以安装项目发布页的 Windows 预编译 `sw2robot-web`，在浏览器界面选择
同一总装和 `初始位置` 配置。无论使用命令行还是界面，都要交付整个
`sw2robot_output/fruit_arm` 目录，不能只交一个 URDF。

## 项目侧验收

收到输出目录后先执行：

```powershell
python src/fruit_picking_arm/tools/solidworks_urdf_pipeline.py validate `
  D:\交付目录\sw2robot_output\fruit_arm\graph.json `
  --configuration "初始位置"
```

若使用经典 `solidworks_urdf_exporter`，还必须验收整个导出包：

```powershell
python src/fruit_picking_arm/tools/solidworks_urdf_pipeline.py audit-urdf `
  D:\交付目录\robot\urdf\robot.urdf
```

该命令会额外拒绝不存在/空 STL、非单位关节轴、反向上下限、与新机械契约
不一致的行程以及损坏的父子 link 图。ROS 自带的 `check_urdf` 只检查 XML
树，不能发现 84 字节、0 三角面的空 STL，也不会拒绝倒写的上下限。

## 2026-08-27 机械交付审计

`src/urdf.zip` 不能直接进入生产：

- `Link3.STL` 与 `_Link5.STL` 都只有 84 字节、0 个三角面；导出日志也记录
  这两次 STL 修正失败。
- J1/J3/J4/J5/J6 的 `lower` 和 `upper` 被倒写。
- J3 被写成 `0 → -90°`，而新标准是 `-180° → 0`。
- J6 被写成 `+90° → -90°`，而新标准是 `-180° → +180°`。
- 总装已有 `初始位置`、`Origin_global`、J1～J6 参考轴及六个关节坐标系，
  但没有可见的 `TCP坐标系/TCP_URDF`。

经典导出器对“把整个子装配作为 link 组件”的空 STL 问题已有公开报告：
<https://github.com/ros/solidworks_urdf_exporter/issues/88>。机械端应展开子装配，
给 Link3 和 Link5 选择实际叶子零件后重新导出，不能只选择子装配根节点。

验收器会拒绝以下情况：

- 提取的不是 `初始位置` 配置；
- 缺少任一 `Origin_global/J1坐标系...J6坐标系/TCP坐标系`；
- 组件变换不是有限的刚体变换；
- 配合图中不足六根有效单位轴线。

通过后再完成 link 合并、关节命名、限位、惯量和 ROS 2 description 导出。
最终验收至少包括：

1. 在机械零位和每轴两个端点，共 13 个姿态比较实车与 RViz。
2. 用量角器/水平仪只作独立复核，不再用它反向“调到看起来相同”。
3. 逐姿态比较法兰/TCP 的实测位置与 FK，记录最大位置和姿态误差。
4. MoveIt 碰撞体、惯量和 TCP 分别验收；外观正确不代表碰撞/运动学正确。
5. 串口 `/joint_states` 只发布同一定义的机械关节角，不在视觉侧重复加零偏。

## 与旧错误的区别

正确的 URDF 关节变换为：

```text
parent_from_joint = inverse(world_from_parent_link) * world_from_joint
```

网格必须转换到所属 link 的局部坐标：

```text
link_from_mesh = inverse(world_from_link) * world_from_mesh
```

旧转换器只做公共 CAD 换轴和手填平移，没有读取每个子装配的旋转，也没有从
配合建立刚体集合；随后又在 Xacro 中叠加人工 `rpy`。这会让网格在某些姿态
看似接近，而真实 FK、TCP 和碰撞体仍然错误，直接影响 MoveIt 规划与抓取。
