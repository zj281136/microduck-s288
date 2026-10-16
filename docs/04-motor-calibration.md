# 先让鸭子认清自己的关节

电机接上以后，程序还需要知道三件事：它装在哪个关节、模型里的零位对应哪个编码器读数，以及朝哪边转算正方向。这就是标定要做的事情。

这一步值得慢慢做。零位和方向弄清楚，后面站立、行走出了问题，才能把它们从排查清单里划掉。

这版使用 **rotor 位置和协议速度**，零位捕获也选择 rotor。下面依次读反馈、捕获零位，再做方向测试。

所有命令都在 `software/deployment` 中执行，先按[安装说明](03_software_setup/environment_setup.md)准备好环境。请先把鸭子支撑稳：零增益读反馈时，电机不会帮它站住。

整机总线通信检查覆盖 15 个舵机（ID 0–14）；下面的零位、direction 和标定文件仍只服务于 RL 的 14 个关节（ID 0–13），不将第 15 个舵机加入策略数组。

## 1. 先对一下电机编号

| 总线 ID | 关节 | 分组 |
| --- | --- | --- |
| 0 | head_roll | head |
| 1 | head_yaw | head |
| 2 | head_pitch | head |
| 3 | neck_pitch | head |
| 4 | right_hip_yaw | right |
| 5 | left_hip_yaw | left |
| 6 | left_hip_roll | left |
| 7 | left_hip_pitch | left |
| 8 | left_knee | left |
| 9 | left_ankle | left |
| 10 | right_hip_roll | right |
| 11 | right_hip_pitch | right |
| 12 | right_knee | right |
| 13 | right_ankle | right |

这张表可以放在装配台旁边查：ID是总线里找电机的地址，后面的关节名则说明它装在哪里。策略数组有另一套顺序，见[控制接口](05-control-contract.md)，两者不要直接按数字相配。

电机 ID 设置参考[宇树 J288 / S288 官方快速入门](https://support.unitree.com/home/zh/J288-S288%20Servo/Quick_Start)。先逐颗连接，按上表写入目标 ID，再回读确认并贴标签；本章的标定命令不负责修改电机 ID。

## 2. 找到通信板的串口

```bash
ls -l /dev/serial/by-id/
```

选择对应 USB 控制器的设备。下面使用 `/dev/ttyACM0` 作为示例；设备名变化时应修改命令中的 `--port`。同一串口只允许一个标定或部署进程使用。

## 3. 先读反馈，暂时不让它主动运动

可以先读头部这组电机，确认通信板和线束都连到了预期位置：

```bash
.venv/bin/python deploy.py probe \
  --port /dev/ttyACM0 --ids 0 1 2 3 --samples 3
```

这组读数正常后，再读整机全部 15 颗：

```bash
.venv/bin/python deploy.py probe \
  --port /dev/ttyACM0 --samples 3
```

probe 发送 `Kp=0、Kd=0、torque=0、speed=0` 请求以获得反馈，不让电机主动跟踪位置。它仍然会打开串口并发送协议帧，支架必须承重。

每条 JSON 对应一次电机请求，主要字段如下：

| 字段 | 含义 |
| --- | --- |
| `id` | 回包对应电机 |
| `raw_pos` / `raw_speed` | 协议原始转子位置、速度整数 |
| `raw_OutPos` | 输出端编码器 13 位计数 |
| `q_rotor_equiv_rad` | 转子位置折算到输出等效角度，尚未套用模型标定 |
| `qdot_rotor_equiv_rad_s` | 协议速度折算到输出等效角速度 |
| `q_out_rad` | 输出端编码器角度 |
| `torque_nm` / `voltage` | 协议换算后的反馈力矩、电压 |
| `tx_s` / `rx_s` | 请求前、读反馈后的单调时钟时间 |
| `motor_error` / `extra_error` | 电机回报错误状态 |

应看到请求 ID 与反馈一致，主要字段为有限数值，错误码为零。超时、CRC 或 ID 错误需要先解决，不能用不完整反馈继续标定。TX/RX 时间描述主机请求窗口，不等于编码器内部独立采样时间。

## 4. 日志里的原始数值怎么读

以当前解析源码为准，齿比 `g=288.35`：

```text
rotor_equiv_position = raw_pos × 2π / 32768 / g
rotor_equiv_velocity = raw_speed / 2.56 × 2π / g
output_position      = raw_OutPos × 2π / 8192
```

位置和速度原始字段使用不同的缩放。不能简单把两个整数都除以同一个常数，也不能把已经换算好的 `MotorState.pos_rad` 或 `speed_rads` 再除一次齿比。

这些值是传感器坐标；进入 policy 前还要应用零位、direction 和模型参考位置。

## 5. 给每个关节记下零位

### 5.1 参考姿态

将待标定关节摆到 **MuJoCo 模型 q=0** 对应的机械姿态。它不等于站立 HOME，也不等于电机上电零点。

零位姿态照片与局部舵盘示意图：**待完善**。需要先确认实际结构与模型定义一致，不能通过让机器人“看起来站直”来替代。

### 5.2 首次整机捕获

```bash
.venv/bin/python deploy.py calibrate \
  --port /dev/ttyACM0 \
  --output teleop/scripts/microduck_s288_calibration.json \
  --group all --capture-seconds 3
```

脚本会检查所选电机、等待回车，然后采集多帧 rotor 位置。采样期间保持关节不动。记录采用圆周统计，避免读数跨过角度边界时普通平均产生错误。

输出中的 `encoder_home_rad` 是传感器参考，`capture_std_rad` 描述采样离散程度，`sample_count` 是有效样本数。明显不稳定的捕获应先检查支撑和通讯，再重新采集；本教程没有将任意固定阈值作为通用验收标准。

默认分组为 `head`，首次整机标定必须显式写 `--group all`。仅换头后可以改为 `--group head`；`--motor-id 3` 可以单独选择 neck_pitch，并覆盖 group 选择。

已有文件会先备份；局部捕获保留未选关节记录。首次捕获完成后，direction 仍可能为空，需要继续方向标定。

## 6. 再确认正反方向——这一步真的会动

```bash
.venv/bin/python deploy.py calibrate \
  --port /dev/ttyACM0 \
  --output teleop/scripts/microduck_s288_calibration.json \
  --group all --calibrate-directions
```

脚本一次驱动一个关节，从当前反馈位置开始执行小幅正编码器方向运动，并询问该方向是否与 MuJoCo 正方向相符。不能仅凭“向前/向后”的文字判断左右镜像关节，应与对应模型轴比较。

标定测试默认使用较小增益和偏移，独立于 RL 的完整增益配置。不要将运行时增益直接替换到该测试中。

已有 `direction=+1/-1` 的关节会跳过测试并保留原值。若需要重新检查，应先备份标定，再核对并处理对应条目；重复运行命令不代表所有旧 direction 都会被覆盖。

## 7. 坐标映射

```text
q_model = model_home + direction × wrap_pi(q_sensor − encoder_home)
qdot_model = direction × rotor_equiv_velocity
```

这里 `q_sensor` 取 rotor 等效位置，`direction` 为 ±1。当前标定参考为模型 q=0，`model_home_rad` 不用于存放 RL HOME。

同一组映射必须同时用于观测和目标转换。策略 q 选择 rotor 后，再额外使用首次 OutPos/rotor 差值转换目标，会造成新的坐标偏移；本配置使用纯 rotor 目标路径。

## 8. 做完后，再过一遍这几项

- 参与 RL 的 14 个关节均有有效零位和 direction，名称与 ID 一一对应。
- q=0 参考姿态下，模型位置应接近零；正向小幅运动方向正确。
- 标定文件属于当前机器人，不从另一台机器复制零位。
- 重新安装舵盘、替换电机或修改机械零位后，重新检查相关关节。

`microduck_s288_calibration.json` 是本机运行数据，默认被 Git 忽略。底层实现位于 `uniarm_l1_bus.py`、`microduck_288.py` 和 `microduck_s288.py`。带 J288 名称的标定基础模块由 S288 入口复用，对外仍使用本章的 S288 命令。

<!-- TODO: 插入 MuJoCo q=0 的正视、侧视和各关节零位局部照片 -->
<!-- TODO: 插入 rotor 标定的实际终端输出及有效JSON字段示例，隐去个体零位 -->
