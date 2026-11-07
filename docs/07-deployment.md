# Raspberry Pi：运行 S288 行走策略

本版包含作者已测试的 `model_4999.pt` 对应 ONNX 和部署代码。对外入口统一为 `deploy.py keyboard`，模型名为 `microduck_s288_velocity.onnx`。

## 安装并离线检查

在 Pi 上进入本仓目录：

```bash
cd software/deployment
bash setup_pi.sh
.venv/bin/python deploy.py check
```

`check` 校验 `[1,61] → [1,14]`、HOME、rotor 反馈、Kp/Kd、M4 metadata 和模型哈希，不打开串口或 IMU。实际个体标定文件不随代码发布，请按[标定教程](04-motor-calibration.md)完成本机标定，保存为 `teleop/scripts/microduck_s288_calibration.json`。

## 启动

托好机器人或使用防坠支撑，确认本机 IMU 方向与零位；下面命令会驱动电机：

```bash
.venv/bin/python deploy.py keyboard \
  --port /dev/ttyACM0 --i2c-address 0x6a \
  --stand-ramp-seconds 12 --arm-ground-test
```

程序保留启动和操作提示，静止完成 IMU 校准，经 12 秒 HOME 过渡后，以零速度指令进入策略。默认持续运行至退出；可以加 `--duration 3` 做短时策略检查。串口路径和 IMU 地址按实际设备修改。

| 按键 | 指令变化 |
| --- | --- |
| W / S | 每次前后速度 ±0.1 m/s，范围 [-0.4, 0.4] |
| A / D | 每次转速 ±0.1 rad/s，范围 [-1, 1] |
| X / 空格 | 速度指令归零，策略仍运行 |
| Ctrl+C | 退出并执行停机处理 |

指令会保持，松键不会归零；终端自动重复按键也会产生增量。head command 固定零，头部关节仍由策略输出控制。启动后终端只保留指令输出，CSV 日志继续记录，stderr 异常仍可见。

## 与成功部署一致的参数

- Kp wire=15、Kd=0，14 个 RL 关节，rotor 位置和协议速度反馈；不加动作低通。
- 50 Hz 目标周期，保持时钟相位；ONNX 单线程，避免小网络推理线程竞争。
- 关节速度保护 **11 rad/s**；跟踪误差和机身角速度保护默认关闭。
- 倾角 35°、温度和通信异常保护保留。退出会撤去增益，需要支撑接住机器人。
- ±0.6 Nm 是训练限幅；直接 Hybrid 固件 PD 部署没有另行设置相同的硬件力矩上限。

新机器需要确认供电、接线、标定和传感器。作者机器的行走结果不能替代另一台机器的验收。

## 日志和离线回归

日志目录：`teleop/scripts/logs/s288_velocity/`。CSV 包含关节、IMU、目标、动作和时序；可从这里检查实际周期和超时情况。

```bash
.venv/bin/python tests/check_velocity_policy.py
```

该检查仅加载模型、验证增益/保护/时钟设置、键盘增量和终端输出行为，不启动硬件。

`deploy.py probe` 和 `deploy.py calibrate` 仍提供总线检测和标定。probe 会发零增益请求；方向标定会运动。接口约定见[控制接口](05-control-contract.md)。
