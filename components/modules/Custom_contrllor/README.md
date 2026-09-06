# 机械臂控制代码必要整改要求

本文依据当前 H7 工程的实际代码整理，只列出接入视觉六轴轨迹所必需的控制层整改。本轮不要求重构已经能够手控运行的电机控制代码。

## 1. 审查结论

下列现有实现可以保留：

- `J1_MAP_K/J1_MAP_D` 至 `GR_MAP_K/GR_MAP_D` 的电机角度与关节角度映射；
- `HAND_J1`～`HAND_G` 的现有顺序，其中 `HAND_G` 继续作为 J6 使用；
- `__SET_JOINT_ANGLE()` 的本地软限位；
- `hand_task_get_feedback()` 和达妙电机现有位置模式；
- `dm_set_pos()` 当前按电机 ID 设置的固定速度，继续原样服务遥控和旧控制路径；
- PE13/PE9 现有互斥控制和 `hand_claw_poll()` 的 700 ms 非阻塞脉冲。

视觉接入前只需要完成下面四项整改：

1. 打通独立的视觉控制模式；
2. 提供六轴目标和反馈适配接口；
3. 提供失联/故障时的本地停止接口；
4. 修复堵转检测中已经确认的条件错误。

## 2. 必须整改：视觉控制模式没有真正执行

代码证据：

- `application/hand/hand_task.h:46` 已定义 `HAND_MODE_CUSTOM_CTRL`；
- `application/hand/hand_task_interface.c:373`、`:379` 的进入代码被注释，目前遥控器模式只会选择 IDLE、RC_CTRL 或 RC2_CTRL；
- `application/hand/hand_task_interface.c:500-518` 的 `hand_task_set_output()` 没有 `HAND_MODE_CUSTOM_CTRL` 分支，因此即使外部把模式写成 CUSTOM，也会落入默认 `__hand_nonforce()`；
- `application/hand/hand_task_interface.c:1122-1133` 的旧 `__hand_custom_ctrl()` 读取 `CC_handler`，它属于旧自定义遥控器数据，不是视觉轨迹目标。

必要修改：

- 复用 `HAND_MODE_CUSTOM_CTRL` 作为视觉自动模式，不新增另一套模式枚举；
- 在 `hand_task_set_output()` 增加明确的 `HAND_MODE_CUSTOM_CTRL` 分支；
- CUSTOM 分支使用新的六轴目标缓冲，不再读取旧 `CC_handler`；
- 只有人工允许、机械臂初始化结束且六个电机均在线、无堵转时才能进入；
- 遥控器退出授权档位、通信失联或本地故障时立即退出 CUSTOM；
- 进入 CUSTOM 的第一拍先把目标设为当前反馈角度，避免沿用旧目标造成跳动。

人工授权具体使用哪个遥控器档位由电控决定，不要求重写现有 RC_CTRL/RC2_CTRL 逻辑。

## 3. 必须整改：提供最小控制适配接口

独立串口模块不能直接访问 `hand_task_handler_ptr`、电机对象或 GPIO。控制层只需提供以下最小接口，函数名可以按现有工程习惯调整：

```c
bool hand_visual_enable(void);
void hand_visual_disable(void);
bool hand_visual_is_ready(void);

bool hand_visual_set_joint_target(const float joint_rad[6]);
bool hand_visual_get_joint_feedback(
    float joint_rad[6],
    float joint_velocity_rad_s[6]);

void hand_visual_controlled_stop(void);
bool hand_visual_stop_complete(void);
bool hand_visual_fault_active(void);

bool hand_visual_claw_command(uint8_t action); /* 1=open, 2=close, 3=stop */
```

接口约定：

- 六轴顺序固定为 `HAND_J1, HAND_J2, HAND_J3, HAND_J4, HAND_J5, HAND_G`；
- 接口角度使用当前控制代码内部的关节角度单位 rad；
- `set_joint_target()` 一次提交六轴，串口任务只能写目标缓冲，不能逐轴直接写 `joint_angle[]`；
- HandTask 每个 1 ms 周期把六轴目标一次性复制到局部变量，再统一调用现有 `__SET_JOINT_ANGLE()`；
- 目标缓冲的读写使用临界区、双缓冲或版本号，不能在六轴只更新一半时被 HandTask 读取；
- 超出当前 `min_joint_angle[]/max_joint_angle[]` 的目标直接返回 false，不允许仅静默截断后继续报告成功；
- 反馈角度直接使用 `feedback_joint_angle[]`；
- 反馈速度由现有 `feedback_motor_speed[]` 按对应 `MAP_K` 换算为关节速度，不把目标速度当反馈速度。

ROS 关节零位与电控内部关节零位的方向/偏置转换放在独立串口适配模块中。现有 `J*_MAP_K/J*_MAP_D` 不要求修改，也不能在两处重复换算。

## 4. 必须整改：CUSTOM 分支六轴采用同一种目标更新方式

现有 `__hand_move2_subctrl()` 中：

- J1、J2、J3、J5 直接调用 `__SET_JOINT_ANGLE()`；
- J4、HAND_G 使用 `__hand_motor_go_setting_angle()` 再做一层渐进处理。

视觉端发送的轨迹已经包含严格时间和插值目标，因此 CUSTOM 模式不能继续调用这套混合更新函数，否则六轴会出现不同步和额外延迟。

CUSTOM 分支只需对六轴统一执行现有的：

```c
__SET_JOINT_ANGLE(HAND_J1, target[0]);
__SET_JOINT_ANGLE(HAND_J2, target[1]);
__SET_JOINT_ANGLE(HAND_J3, target[2]);
__SET_JOINT_ANGLE(HAND_J4, target[3]);
__SET_JOINT_ANGLE(HAND_J5, target[4]);
__SET_JOINT_ANGLE(HAND_G,  target[5]);
```

然后继续复用 `hand_task_output()` 中现有的关节到电机映射。视觉轨迹分支把同一插值拍的目标速度换算成达妙位置-速度模式的动态速度上限；遥控和旧控制路径仍调用未修改的 `dm_set_pos()`。

## 5. 必须整改：READY、故障和受控停止

`hand_visual_is_ready()` 至少同时检查：

- `hand_task_init()` 已经执行结束；
- 当前已获得人工授权；
- 六轴 `toe_is_error(TOE_J1...TOE_G)` 均为 false；
- 六轴没有 `toe_is_stall()`；
- 当前没有执行 RC_CTRL、RC2_CTRL 或固定动作。

`hand_visual_fault_active()` 至少汇总现有掉线和堵转状态。

`hand_visual_controlled_stop()` 的首版实现不需要增加复杂减速器，可以采用现有能力：

1. 禁止继续接受视觉目标；
2. 将六轴目标一次性设置为当前 `feedback_joint_angle[]`；
3. 退出 CUSTOM，进入现有 IDLE 保持模式；
4. 等目标和反馈均稳定后由 `hand_visual_stop_complete()` 返回 true。

出现电机掉线或堵转后不得自动恢复视觉轨迹；必须丢弃旧目标，重新人工授权后才能再次进入 CUSTOM。

## 6. 必须修复：J4/J6 堵转判断当前不会按设计工作

代码证据：

- `application/detect_task/detect_task.c:352` 把堵转参数表声明为 `uint16_t`，但表中使用了 `2.5、1.2、0.8、0.4`，小数会被截断；
- J4 的 `0.8` 和 J6 的 `0.4` 会变成 0；
- `application/detect_task/detect_task.c:476-502` 把“阈值等于 0”的处理写在“阈值不等于 0”的分支内部，该分支永远不能执行。

这是现有代码错误，不是视觉新增功能。要求做最小修复：

- 参数表改为能保存小数的类型，或分别给浮点字段赋值；
- 将 `speed_slope_threshold == 0` 的判断移到正确的并列分支；
- 使用当前六轴做一次堵转测试，确认 `toe_is_stall(0...5)` 均能置位；
- 堵转置位后 CUSTOM 必须退出且不能继续更新目标。

不要求借此重写整个 `detect_task`。

## 7. 夹爪只需要暴露现有开环动作

现有代码已经做到：

- `hand_claw_open()` 先同时拉低 PE13/PE9，再拉高 PE13；
- `hand_claw_close()` 先同时拉低 PE13/PE9，再拉高 PE9；
- `hand_claw_poll()` 在 HandTask 中每 1 ms 调用，700 次后拉低输出；
- 没有使用阻塞式 `HAL_Delay(700)`。

因此不要求重写夹爪控制。只需要让 `hand_visual_claw_command()` 安全地复用现有动作：

- 串口任务只提交 OPEN/CLOSE/STOP 请求；
- HandTask 消费请求并调用现有函数，避免跨任务同时操作夹爪状态变量；
- STOP 将 PE13、PE9 同时拉低并回到 IDLE；
- 现有 700 ms 脉冲先保留；串口模块约 1500 ms 后报告“动作周期完成但未验证抓取”；
- 不增加力、压力、开度或水果检测逻辑。

## 8. 首轮接入不要求整改的内容

为避免影响已经实车手控验证过的代码，本轮明确不要求：

- 重命名 `HAND_G`、`dm_gripper`；
- 修改 `J*_MAP_K/J*_MAP_D`；
- 修改现有六轴软限位数值；
- 重写 `dm_set_pos()` 或达妙 CAN 发送周期（视觉分支在上层调用已有的 `pos_speed_ctrl()`，不改变旧函数）；
- 重构 RC_CTRL、RC2_CTRL 和固定动作；
- 重写 J1～J6 初始化/寻零流程；
- 把夹爪 700 ms 脉冲强制改成 1500 ms；
- 增加夹爪传感器或抓取成功判断。

但电控必须把以下现有参数提供给视觉端，用于保持两侧约束一致：

| 轴 | 电控名称 | 电机 ID | 当前关节下限(rad) | 当前关节上限(rad) |
|---|---|---:|---:|---:|
| J1 | HAND_J1 | 1 | -1.57 | 1.57 |
| J2 | HAND_J2 | 2 | -2.00 | -0.613 |
| J3 | HAND_J3 | 3 | -6.00 | -2.93 |
| J4 | HAND_J4 | 6 | -2.93 | 2.85 |
| J5 | HAND_J5 | 5 | -1.57 | 1.57 |
| J6 | HAND_G | 4 | -3.14 | 3.14 |

当前视觉串口接收上限为 `[0.42, 0.49, 0.49, 1.29, 1.50, 1.50] rad/s`，上位机正常加速度上限为 `0.75 rad/s^2`。这些是依据现有映射和电机固定速度上限得到的保守值，仍需通过单轴、空载、负载测试校准；调整视觉专用速度计算时不得修改遥控使用的 `dm_set_pos()`。

## 9. 最小验收顺序

1. 原有 RC_CTRL、RC2_CTRL 和夹爪手控回归测试，行为不得改变；
2. 电机不上使能，验证六轴反馈顺序、方向和零位；
3. 人工授权进入/退出 CUSTOM，进入时目标等于当前反馈；
4. 单轴小角度低速目标，验证现有软限位仍生效；
5. 六轴同步小轨迹，检查目标缓冲没有半更新；
6. 通信失联、遥控器取消授权、电机掉线和堵转时退出 CUSTOM；
7. OPEN/CLOSE/STOP 请求复用现有 700 ms 夹爪脉冲；
8. 最后再进行六轴空载和水果负载测试。

完成以上项目后，控制层才具备接收视觉轨迹的最小条件。其余代码优化不属于本轮必要整改。
