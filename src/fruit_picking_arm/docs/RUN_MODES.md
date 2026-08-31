# 当前运行模式

> 本文以 `scripts/single_arm/*.sh`、`launch/*.launch.py` 和当前串口协议源码为准。
> 四个架构 launch 文件仍保留，但目前只有方案 A 是已经跑通的实车控制链路。

## 方案 A：上位机 MoveIt 规划

### 仿真

```bash
bash scripts/single_arm/start_architecture_a_sim.sh
```

该入口只使用 Gazebo、MoveIt 和仿真控制器，不打开真实串口。直接调用
`architecture_a_sim.launch.py` 时 `run_task` 默认是 `true`；只看场景和规划可传
`run_task:=false`。

### 实车基础联调

```bash
bash scripts/single_arm/start_architecture_a_real.sh
```

当前脚本默认启动 D455、持续感知、手眼 TF、真实 `/joint_states`、MoveIt、C++
串口控制器和 RViz，但保持 `run_task:=false`，不会自动抓取。若只需感知，显式使用：

```bash
JAKA_START_ROBOT_STACK=false bash scripts/single_arm/start_architecture_a_real.sh
```

### 任意目标 Plan/Execute

```bash
bash scripts/single_arm/start_architecture_a_real_plan_execute.sh
```

用于验证当前实车姿态、MoveIt 规划和上位机轨迹下发。先点 `Plan`，检查整条轨迹，
再点 `Execute`；`Execute` 会向真实电控发送轨迹。

### 水果分阶段人工验收

```bash
bash scripts/single_arm/start_architecture_a_real_fruit_plan_execute.sh
```

程序锁定一个稳定水果并依次发布：

```text
pregrasp → open → grasp → close → lift → bin hover
         → release → retract → home
```

每个机械臂阶段都必须重新点击 `Plan`、检查、再点击 `Execute`。夹爪在对应机械臂
阶段成功后自动开/闭；它是无位置、力和接触传感器的开环二值执行器，完成只表示
电控动作周期结束，不表示已验证夹住。场景中最多保留 5 个苹果，任务一次锁定并处理
一个；其余苹果继续作为 MoveIt 碰撞球。预抓取/抓取对每个目标最多尝试 8 个保持
竖直套取方向的腕部 yaw；全部 IK 失败时跳过该物理位置并尝试其他水果。

### 自动实车任务

```bash
bash scripts/single_arm/start_architecture_a_real_run.sh
```

该入口无 RViz 和调试窗口，设置 `start_robot_stack:=true`、`run_task:=true`，直接运行
行为树。只有人工分阶段验收、箱体测量、桌面感知、软硬限位、急停和电控保护全部
通过后才能使用。

所有方案 A 实车入口使用 `scene_params_real.yaml`。桌面顶面由新鲜 D455 RGB-D
估计；两个分拣箱和 `home_pose` 来自实测配置。仿真只使用
`scene_params_sim.yaml`，两者不得互相复制。

实车箱体参数统一采用外轮廓：`center.x/y/z` 是整个箱体外包络的几何中心，
`size.x/y/z` 是外部长、宽、高。因此箱底和上沿为：

```text
bottom_z = center.z - size.z / 2
top_z    = center.z + size.z / 2
```

若箱子放在 `base_link` 的 z=0 地面上，填写 `center.z = size.z / 2`。`size.x`
沿公共 +X，`size.y` 沿公共 +Y；当前箱体不支持额外 yaw，长边平行 X。MoveIt
用底板和四面墙建立碰撞体，并依据壁厚、水果半径和安全间隙选择箱内落点。

关闭任一方案 A 实车入口：

```bash
bash scripts/single_arm/stop_architecture_a_real.sh
```

## 方案 B：C 板规划（保留的未来并行方案）

方案 B 的 launch、感知输出、工作空间门控和串口桥文件必须保留。设计目标是上位机
只发送 `base_link` 下的水果三维目标，C 板负责抓取姿态、IK、轨迹、夹爪和分拣。

当前固定长度 AA55 协议的 `MessageType` 没有 `FRUIT_TARGET`，`ControlLink` 也没有
可用的 `send_fruit_target()`，因此方案 B 目前不是可运行/可验收的实车方案。不要用
`architecture_b_real.launch.py` 驱动实车，也不要把现有测试解释为方案 B 端到端完成。

## launch 的安全默认值

直接调用 `architecture_a_real.launch.py` 时，默认：

```text
start_perception=false
start_robot_stack=false
run_task=false
model_license_approved=false
```

因此日常操作使用上述脚本。直接 launch 只适合清楚每个参数含义的开发调试。

## 坐标和相机档位

- `world`：方案 A 的工位/MoveIt 规划坐标。
- `base_link`：底座 400×400 mm 外轮廓底面的水平中心，+X 向前、+Y 向左、+Z 向上。
- 当前实车 `world → base_link` 为单位变换，数值相同但语义不能混用。
- `cad_base_link` 是内部 CAD 适配坐标，现场量尺和应用代码都不能使用。
- 当前 WSL/USBIP 实车 RGB-D 默认 `424×240@15`；只有同步和无超时验收通过后才覆盖。

机械限位、坐标方向和上电检查见 `MECHANICAL_SAFETY_LIMITS.md`；线缆协议见
`SERIAL_CONTROL_PROTOCOL.md`。
