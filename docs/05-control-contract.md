# 训练与部署的控制接口

模型能够成功加载，不代表控制接口已经一致。关节顺序、单位、零位、观测归一化和动作含义必须共同匹配。

## 1. 61 维 actor observation

| Python slice | 维度 | 内容 | 单位/参考 |
| --- | ---: | --- | --- |
| `[0:3]` | 3 | IMU gyro | body frame，rad/s |
| `[3:6]` | 3 | projected gravity | 机器人坐标中的重力方向 |
| `[6:20]` | 14 | `q_rotor − HOME` | 输出等效角度，rad |
| `[20:34]` | 14 | `qdot_rotor` | 输出等效速度，rad/s |
| `[34:48]` | 14 | `last_action` | 上一策略原始动作 |
| `[48:51]` | 3 | velocity command | vx、vy、wz |
| `[51:55]` | 4 | head command | neck_pitch、head_pitch、head_yaw、head_roll |
| `[55:61]` | 6 | body command | x、y、z、roll、pitch、yaw |

未启用的命令仍保留对应槽位，不通过删维度适配。61D 是 actor 输入，critic 可以拥有额外的训练信息，不能将 critic 维度误认为 ONNX 输入。

## 2. 关节顺序

```text
0  left_hip_yaw       7  head_yaw
1  left_hip_roll      8  head_roll
2  left_hip_pitch     9  right_hip_yaw
3  left_knee         10  right_hip_roll
4  left_ankle        11  right_hip_pitch
5  neck_pitch        12  right_knee
6  head_pitch        13  right_ankle
```

所有 14D q、qdot、action 和目标数组使用此顺序。总线电机 ID 按名称映射，不能将 ID 0–13 的列表直接当作 policy 数组。

## 3. action、HOME 与 last_action

```text
q_target = HOME + action_scale × action
```

该配置 `action_scale=1`。`HOME` 是训练默认站姿；不是标定 q=0，也不是某一帧实际反馈。部署继续执行原目标限位与安全检查。

`last_action` 保存上一策略输出，既不是 `q_target`，也不是 `q_target-q_rotor`。把 tracking error 填进这 14 维会改变网络的物理输入含义。

键盘逐次修改速度 command；不对 policy action 做滤波。当前 head/leg 目标低通、额外 target slew 和额外 PD projection 均关闭。

## 4. 主动与被动状态

仿真每个电机对应主动 rotor hinge 和被动 backlash hinge。主动关节接受 actuator torque；被动 hinge 无独立 actuator，通过约束和力学耦合影响连杆。

policy 读取 active q/qdot，不读取 passive，也不读取 active+passive。原始 XML 中被动关节可能与主动关节交错，因此代码通过名称和 servo helper 选择主动关节，不能硬编码实体中全部 joint 的前 14 项。

当前 MJCF 自带被动传动关节，保留成功训练版本的参数。本版不使用旧任务的每 episode 传动间隙随机化事件。

## 5. Hybrid + BAM M4

| 参数 | 配置 |
| --- | --- |
| Kp / Kd 协议值 | 15 / 0 |
| SDK Kp / Kd | 15/1280000 / 0 |
| 齿比、M4 ratio | 288.35、1.0 |
| policy / physics | 50 Hz / 5 ms；decimation=4 |
| 仿真限幅 | 固定 ±0.6 Nm，不按电压缩放 |
| command lag DR | 3–6 physics step |
| 拟合 command delay | 0.0029063016859105155s，另按物理步做分数延迟插值 |
| 电气模型 / voltage DR | 无 / 无 |

输出 PD 请求为 `gear_ratio² × ratio × (kp_sdk × (goal-q) - kd_sdk × qdot)`，本配置等效刚度约 0.97436 Nm/rad。M4 在此基础上计算载荷相关库仑/Stribeck 摩擦及粘性摩擦。公式与参数见[执行器说明](07_engineering_notes/pd_vs_bam.md)。

JSON 的拟合 `max_torque≈0.4586` 保留作为辨识记录，仿真采用用户选定的独立 0.6 Nm 限幅。Pi 直接发送固件 Hybrid 目标和增益，没有另行设置 0.6 Nm 硬件限幅。

## 6. 真机反馈来源

rotor 位置与协议速度来自同一回包；完成协议缩放后使用标定 direction/zero 映射。qdot 不用位置差分估计。

OutPos 仍写入日志，便于比较输出侧与转子侧差异；它不进入 61D policy，也不作为启动时目标坐标对齐来源。

## 7. ONNX contract

导出模型包含 observation normalizer。部署 loader 校验输入输出维度、观测项顺序、HOME、动作缩放、增益以及本 task 的 contract/dynamics metadata。

```text
input  [1, 61]
output [1, 14]
contract: microduck_s288_velocity_61d_v1
```

接口不匹配时应重新核对模型与配置，不通过删除校验来使错误模型进入控制循环。

<!-- TODO: 插入14关节policy顺序与总线ID的对应示意图 -->
