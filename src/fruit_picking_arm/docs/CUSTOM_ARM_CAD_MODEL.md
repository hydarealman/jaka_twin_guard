# 自研机械臂 CAD/URDF 模型

## 当前状态

当前生产模型来自项目图纸/STEP 总装配体，不是 JAKA 官方机械臂模型。运行包和坐标系已按实际用途命名：

- 机械臂关节：`joint_1` ～ `joint_6`
- ROS 基座：`base_link`
- CAD 坐标适配：`cad_base_link`
- 机械臂连杆：`arm_link_1` ～ `arm_link_6`
- 机械法兰：`tool_flange`
- 抓取 TCP：`gripper_tcp`
- 仿真夹爪关节：`left_finger_joint`、`right_finger_joint`

网格文件仍叫 `Link_00.STL` ～ `Link_06.STL`，因为它们是 CAD 导出物名称；这不再是 ROS link 名称。

## 文件布局

```text
src/fruit_picking_arm/
├── tools/step_xcaf_to_robot_meshes.py       # STEP 装配体分组和 STL 生成工具
├── test/run_custom_arm_gazebo_e2e.sh        # Gazebo/控制器/MoveIt 闭环测试
├── test/run_custom_arm_pick_e2e.sh          # 4 个水果识别抓取分类测试
├── test/run_custom_arm_state_probe.sh       # MoveIt 合法待机位探针
└── docs/CUSTOM_ARM_CAD_MODEL.md              # 本文档

src/moveit_resources-ros2/
├── fruit_arm_description/
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── meshes/
│       ├── cad_mesh_manifest.json            # 229 个实例的分组审计记录
│       └── visual/Link_00.STL ... Link_06.STL
└── fruit_arm_moveit_config/config/
    ├── fruit_arm_macro.xacro                 # 6 轴运动学、碰撞和惯性
    └── fruit_picking_arm.urdf.xacro         # 默认引用 fruit_arm_macro
```

运行包为 `fruit_picking_arm`，MoveIt 配置包为 `fruit_arm_moveit_config`，网格描述包为 `fruit_arm_description`。

## 坐标系契约

```text
world                         工位/规划全局坐标
└── base_link                 机械臂物理安装基准，+X 向前、+Y 向左、+Z 向上
    └── cad_base_link         仅吸收 CAD 前向与 ROS 前向之间的 Rz(pi)
        └── arm_link_1 ... arm_link_6
            └── tool_flange  机械法兰
                ├── gripper_base
                └── gripper_tcp  指尖中心，沿法兰 -Z 方向 86 mm
```

`world -> base_link` 是机械臂在工位中的安装位姿，由 URDF xacro 参数 `base_xyz`、`base_rpy` 唯一维护。桌子、传送带、料框、D455 和 MoveIt 目标使用 `world`；无 frame-id 的 C 板水果坐标协议固定使用 `base_link`；应用代码不得使用内部 `cad_base_link`。

## CAD 到 ROS 坐标转换

原图纸单位为毫米。生成的二进制 STL 保持毫米单位，URDF 使用 `scale="0.001 0.001 0.001"` 转成米。

坐标轴映射为：

```text
ROS x = CAD z - 274.626 mm
ROS y = CAD x - 22.719 mm
ROS z = CAD y + 172.512 mm
```

STEP AP203 文件不保存 SolidWorks 配合关系、编码器零偏和厂家关节限位。当前关节中心和轴由轴、法兰、电机安装结构的几何中心推定：

| 关节 | ROS 轴 | 相对父连杆原点 xyz（m） |
|---|---|---|
| `joint_1` | `0 0 1` | `0 0 0.040975` |
| `joint_2` | `0 1 0` | `-0.048101 0.056781 0.094025` |
| `joint_3` | `0 1 0` | `0.399412 -0.015781 0.184581` |
| `joint_4` | `1 0 0` | `-0.041455 -0.032800 0.057441` |
| `joint_5` | `0 1 0` | `-0.372174 -0.034450 -0.004890` |
| `joint_6` | `-0.834467 0 -0.551058` | `-0.026883 0.054950 -0.018957` |

STEP 网格按 CAD 装配姿态导出，而机械零位定义为：J2 大臂处于最低机械
限位、J3大小臂垂直、J4 Logo 外壳基准面、J5 的 J6/J4 roll 轴平行。因此 URDF
不能把 CAD 展示姿态直接当作六轴全零姿态。名义模型在 J2、J3、J5
关节原点中分别加入 `-125.196838°`、`+65.196838°`、`+33.439625°`
的固定旋转。J4 使用 CAD 原始姿态（固定旋转为 `0°`），已经过实车外观核对。
J2、J3、J5 可由上表的中心线/轴线数据复算，具体公式见
`MECHANICAL_SAFETY_LIMITS.md`。`xyz`、`axis`、编码器正方向及传动比例仍须
通过实车逐轴核对。

## 重新生成网格

转换工具依赖 Python 的 OpenCascade/OCP（当前开发机已安装）。在 Windows PowerShell 中运行：

```powershell
python src/fruit_picking_arm/tools/step_xcaf_to_robot_meshes.py `
  artifacts/cad_source/stp/robot_arm_ascii_v1.STEP `
  src/moveit_resources-ros2/fruit_arm_description/meshes/visual `
  --manifest src/moveit_resources-ros2/fruit_arm_description/meshes/cad_mesh_manifest.json
```

工具不修改原始图纸。它读取 XCAF 装配层级，按刚体分成七段，并输出约 42 MB 的等精度二进制 STL。OpenCascade 报出的少量 `null triangulation` 面来自原 STEP 中的坏边；主体 229 个零件实例均已分组，详情见 manifest。

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
4. 实测 `tool_flange -> gripper_tcp`，并替换当前图纸值 86 mm。
5. 真实四指夹爪的驱动自由度、开合行程和控制协议。
6. 相机到机器人基座的眼在手外标定结果。

目前完整 CAD 作为外观网格，实时碰撞使用简化盒体/圆柱；这能显著提高 Gazebo 和 MoveIt 稳定性。夹爪控制仍沿用已验证的双平移关节仿真抽象，CAD 夹爪主体保留在末端外观中。获得真实夹爪运动图纸后，再把四指机构拆成准确的活动连杆。

## 许可证和交付边界

- 机械臂网格由用户放入本项目的图纸生成，描述包标记为 `Proprietary`。
- 转换过程中没有复制外部自瞄项目的源代码，也没有修改该项目。
- 企业交付前应由项目方确认原始 CAD、夹爪和电机外形数据的商业使用与再分发权。
- Python/OCP 只用于离线生成网格；ROS 运行和最终设备不需要安装 OCP。
