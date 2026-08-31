# 自研机械臂 CAD/URDF 模型

## 当前状态

当前生产模型来自机械 2026-08-27 的“初始位置”SolidWorks URDF/STEP 交付，
不是 JAKA 官方机械臂模型。J1～J6 原点和轴采用机械插件导出结果，机械限位按
双方确认的新协议在代码侧修正；Link4/Link5 的错误刚体归属和 J6 混入夹爪的
历史分类错误已经修复。交付来源、哈希和剩余校核项记录在
`fruit_arm_description/meshes/production_current/PROVENANCE.md`。

> 重要：STEP 只能用于修复/核对网格，不能单独重新推导关节运动学。J4 导出轴
> 的约 0.5° 倾斜、各活动体惯量和精确夹爪中间运动仍需后续机械数据确认。

- 机械臂关节：`joint_1` ～ `joint_6`
- ROS 基座：`base_link`
- CAD 坐标适配：`cad_base_link`
- 机械臂连杆：`arm_link_1` ～ `arm_link_6`
- 机械法兰：`tool_flange`
- 抓取 TCP：`gripper_tcp`
- 仿真夹爪关节：`left_finger_joint`、`right_finger_joint`

机械臂生产网格位于 `meshes/production_current`，夹爪网格位于
`meshes/gripper_current`；文件名不是 ROS link 名称。

## 文件布局

```text
src/fruit_picking_arm/
├── tools/step_xcaf_to_robot_meshes.py       # STEP 网格刚体分组/修复工具
├── tools/build_actual_gripper_meshes.py     # 闭合/张开 STEP 生成四指夹爪
├── test/run_custom_arm_gazebo_e2e.sh        # Gazebo/控制器/MoveIt 闭环测试
├── test/run_custom_arm_pick_e2e.sh          # 4 个水果识别抓取分类测试
├── test/run_custom_arm_state_probe.sh       # MoveIt 合法待机位探针
└── docs/CUSTOM_ARM_CAD_MODEL.md              # 本文档

src/moveit_resources-ros2/
├── fruit_arm_description/
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── meshes/
│       ├── production_current/               # 当前机械臂生产网格与来源记录
│       └── gripper_current/                  # 四指夹爪网格和 KINEMATICS.json
└── fruit_arm_moveit_config/config/
    ├── fruit_arm_macro.xacro                 # 6 轴运动学、碰撞和惯性
    └── fruit_picking_arm.urdf.xacro         # 默认引用 fruit_arm_macro
```

运行包为 `fruit_picking_arm`，MoveIt 配置包为 `fruit_arm_moveit_config`，网格描述包为 `fruit_arm_description`。

## 坐标系契约

```text
world                         工位/规划全局坐标
└── base_link                 400×400 mm 底座底面的几何中心，+X 向前、+Y 向左、+Z 向上
    └── cad_base_link         SolidWorks 总装内部原点，仅供模型内部使用
        └── arm_link_1 ... arm_link_6
            └── tool_flange  GRIPPER_MOUNT，J6 +X 方向 42.2 mm
                ├── gripper_base
                └── gripper_tcp  夹取中心，沿 MOUNT +Z 方向 124 mm
```

`world -> base_link` 是机械臂在工位中的安装位姿，由 URDF xacro 参数 `base_xyz`、`base_rpy` 唯一维护。当前实车令二者重合。`base_link` 的原点位于底座 400×400 mm 外轮廓底面的水平中心，不是 J1 轴心；J1 轴心相对它约为 `[-5, +17, +82] mm`。J1 为 0° 时机械臂的竖直运动平面是公共 `X-Z` 平面，实际工作台一侧为 `+X` 正前方，`+Y` 为左侧。桌子、传送带、料框、D455 和 MoveIt 目标使用 `world`；方案 B 将来若补齐无 frame-id 的 C 板水果坐标协议，必须固定使用 `base_link`；应用代码不得使用内部 `cad_base_link`。

生产 CAD 网格原点相对该公共测量基准有偏置，因此固定接头为
`base_link -> cad_base_link = [+0.269625726, +0.039718900, +0.190511596] m`，绕 Z 轴旋转 `-90°`。这个内部适配只负责把网格、机械正前方和关节装配放回正确位置，不能再用于现场量尺。

## CAD 到 ROS 坐标转换

历史转换输出保持毫米并在 URDF 缩放；当前 `production_current` 和
`gripper_current` 网格已经转换为米，URDF 使用 `scale="1 1 1"`。

下式只描述生成网格时 SolidWorks 原始几何到网格局部坐标的离线转换，不能
当成运行时 `base_link` 的量尺公式。运行时公共基座方向只由上一节的
`base_link -> cad_base_link` 固定接头定义。

坐标轴映射为：

```text
ROS x = CAD z - 274.626 mm
ROS y = CAD x - 22.719 mm
ROS z = CAD y + 172.512 mm
```

STEP AP203/AP214 文件不能替代 SolidWorks 配合关系、机械零位、编码器零偏和
厂家关节限位。下表是历史模型的推定值，只用于定位旧模型问题，不再作为新
URDF 的输入：

| 关节 | ROS 轴 | 相对父连杆原点 xyz（m） |
|---|---|---|
| `joint_1` | `0 0 1` | `0 0 0.040975` |
| `joint_2` | `0 1 0` | `-0.048101 0.056781 0.094025` |
| `joint_3` | `0 1 0` | `0.399412 -0.015781 0.184581` |
| `joint_4` | `1 0 0` | `-0.041455 -0.032800 0.057441` |
| `joint_5` | `0 1 0` | `-0.372174 -0.034450 -0.004890` |
| `joint_6` | `-0.834467 0 -0.551058` | `-0.026883 0.054950 -0.018957` |

历史模型曾在 J2、J3、J5 关节原点中加入 `-119.196838°`、
`+65.196838°`、`+33.439625°` 等人工固定旋转。这些数值把关节中心连线、
CAD 保存姿态和机械零位混在一起，并非从命名关节坐标系或 SolidWorks 配合
自动导出，因此全部标记为“未验证”；不能继续以某个极限姿态是否水平来反算
或修补其中一个角。

## 重新生成网格

转换工具依赖 Python 的 OpenCascade/OCP（当前开发机已安装）。在 Windows PowerShell 中运行：

```powershell
python src/fruit_picking_arm/tools/step_xcaf_to_robot_meshes.py `
  artifacts/cad_source/stp/robot_arm_ascii_v1.STEP `
  src/moveit_resources-ros2/fruit_arm_description/meshes/visual `
  --manifest src/moveit_resources-ros2/fruit_arm_description/meshes/cad_mesh_manifest.json `
  --unsafe-legacy-inferred-frames
```

该命令仅复现旧网格，参数名中的 `unsafe` 是有意的：工具不读取 mates，按名字
推测刚体归属，并使用历史人工 link 原点。它不能生成或验证运动学。生产转换
必须执行 `SOLIDWORKS_URDF_WORKFLOW.md` 中的 mate-aware 流程。

## 构建、查看和验证

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  fruit_arm_description fruit_arm_moveit_config fruit_picking_arm \
  --symlink-install
source install/setup.bash
```

运行方案 A 的完整 Gazebo 水果识别抓取仿真：

```bash
ros2 launch fruit_picking_arm architecture_a_sim.launch.py
```

首次检查场景时建议先关闭自动抓取：

```bash
ros2 launch fruit_picking_arm architecture_a_sim.launch.py \
  run_task:=false start_image_view:=false
```

自动闭环测试：

```bash
bash src/fruit_picking_arm/test/run_custom_arm_gazebo_e2e.sh "$PWD"
bash src/fruit_picking_arm/test/run_custom_arm_pick_e2e.sh "$PWD"
```

第一项验证 URDF、8 个关节、Gazebo 控制器和 MoveIt/KDL 基础闭环；第二项必须看到
`CUSTOM_ARM_PICK_E2E_PASS`，并验证 4 个水果全部完成识别、IK、OMPL 规划、物理约束
夹持、抬升搬运、重力释放及好坏分框；测试还会查询 Gazebo 实体最终坐标，防止用瞬移
或只打印成功日志冒充抓取。

## 实车前必须由机械/电控确认

当前模型可以用于视觉、串口、MoveIt 和仿真流程调试，但不能把推定参数直接作为实车安全参数。实车联调前还需要提供并回填：

1. 六个关节的真实旋转轴方向、机械零位与编码器零偏。
2. 每个关节的软限位、硬限位、最大速度、最大加速度和允许力矩。
3. 各连杆质量、质心和惯性张量，或可计算这些数据的带材料 CAD。
4. J6→MOUNT 的绕工具轴坐标相位最终复核（位置 42.2 mm 已提供）。
5. 如需精确中间姿态，提供丝杆位移到四夹指/驱动连杆角度的曲线或多个中间配置。
6. 相机到机器人基座的眼在手外标定结果。

当前夹爪已经拆为固定座、中心滑块、四个驱动连杆和四个双零件夹指。闭合与
张开两端严格来自机械 STEP；真实机构非线性，因此两端之间的 URDF mimic
运动仅为显示/碰撞近似。对外仍保留 `left_finger_joint`、
`right_finger_joint` 命令接口以兼容现有串口控制，但它们不是伪造的物理直线
夹指，也不会被当作实测夹爪反馈。

## 许可证和交付边界

- 机械臂网格由用户放入本项目的图纸生成，描述包标记为 `Proprietary`。
- 转换过程中没有复制外部自瞄项目的源代码，也没有修改该项目。
- 企业交付前应由项目方确认原始 CAD、夹爪和电机外形数据的商业使用与再分发权。
- Python/OCP 只用于离线生成网格；ROS 运行和最终设备不需要安装 OCP。
