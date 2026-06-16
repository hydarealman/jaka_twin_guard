# JAKA C5 Description

JAKA C5 协作机器人 URDF 模型描述包。包含单臂 6 自由度机械臂的 URDF 定义和 STL 碰撞/视觉网格文件。

## 用途

此包作为双机械臂防碰撞系统 ([`dual_arm_jaka_c5_moveit_config`](../dual_arm_jaka_c5_moveit_config/)) 的模型依赖，也可作为单臂 JAKA C5 的独立 URDF 描述使用。

## 文件结构

```
jaka_c5_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── urdf/
│   └── jaka_c5.urdf              # 单臂 URDF（无 gazebo/transmission）
└── meshes/
    └── jaka_c5_meshes/
        ├── Link_00.STL            # 基座
        ├── Link_01.STL            # J1 输出连杆
        ├── Link_02.STL            # J2 输出连杆（大臂）
        ├── Link_03.STL            # J3 输出连杆
        ├── Link_04.STL            # J4 输出连杆（前臂）
        ├── Link_05.STL            # J5 输出连杆
        └── Link_06.STL            # J6 输出连杆（末端）
```

## URDF 结构

```
world (root)
  │
  └─[fixed]── Link_00 (基座)
                │
              joint_1 [revolute, Z]  ±360°
                │
              Link_01
                │
              joint_2 [revolute, Z']  -85°~+265°
                │
              Link_02 (430mm)
                │
              joint_3 [revolute, Z]  ±175°
                │
              Link_03
                │
              joint_4 [revolute, Z]  -85°~+265°
                │
              Link_04 (368.5mm)
                │
              joint_5 [revolute, Z']  ±360°
                │
              Link_05
                │
              joint_6 [revolute, Z']  ±360°
                │
              Link_06 (末端)
```

## 关键参数

| 参数 | 值 |
|------|-----|
| 自由度 | 6 |
| 总质量 | ~92.9 kg |
| 额定速度 | 3.14 rad/s (所有关节) |
| 网格格式 | ASCII STL |
| 网格来源 | JAKA 官方 `jaka_ros2` 项目 |

## 与原始 URDF 的区别

来自 `jaka_ros2` 项目的原始 URDF 做了以下简化：

1. **移除 `<gazebo>` 插件** — 本文档仅用于 MoveIt 运动规划
2. **移除 `<transmission>` 元素** — 由 `ros2_control` XACRO 宏单独管理
3. **添加 `<collision>` 几何体** — 每个视觉网格同时用作碰撞几何体
4. **使用 `package://` 路径** — 网格引用 `package://jaka_c5_description/meshes/...`

## 使用方法

### XACRO 宏引用

在使用此包的 MoveIt 配置中，通过 XACRO 宏参数化实例化：

```xml
<xacro:include filename="$(find jaka_c5_description)/urdf/jaka_c5.urdf"/>
```

或作为宏引用网格：

```xml
<mesh filename="package://jaka_c5_description/meshes/jaka_c5_meshes/Link_01.STL"/>
```

### 依赖声明

```xml
<!-- package.xml -->
<exec_depend>jaka_c5_description</exec_depend>
```
