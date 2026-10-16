# Raspberry Pi 部署代码

本目录包含 S288 rotor-only 61D 策略运行所需的项目源码：电机协议、模型坐标映射、IMU、标定、实时控制循环、键盘输入及 ONNX。可单独复制到 Pi，训练环境不参与运行。

## 安装与模型检查

```bash
cd software/deployment
bash setup_pi.sh
.venv/bin/python deploy.py check
```

`setup_pi.sh` 创建本目录 `.venv` 并安装运行依赖。`check` 校验模型接口、contract 和哈希，不打开硬件。

## 统一入口

| 模式 | 功能 | 是否访问硬件 |
| --- | --- | --- |
| check | 加载模型并核对 manifest | 否 |
| probe | 零增益反馈采集 | 是，发送总线请求 |
| calibrate | 捕获零位或执行方向标定 | 是，方向标定会运动 |
| stand | 固定 command 的站立/RL运行 | 是，会运动 |
| keyboard | 键盘控制 | 是，会运动 |

```bash
.venv/bin/python deploy.py keyboard \
  --port /dev/ttyACM0 --i2c-address 0x6a \
  --stand-ramp-seconds 12 --max-speed 5 --print-rate 1 --arm-ground-test
```

运动前完成本机标定并使用防坠保护。键盘命令会保持，W/S 为 ±0.20 m/s，A/D 为 ±2 rad/s，X/空格停止，Ctrl+C 退出。

## 代码结构

```text
setup_pi.sh                 环境安装
deploy.py                   入口分发，不改变原控制逻辑
teleop/scripts/             模型加载、标定、实时循环与键盘
teleop/scripts/models/      配套 ONNX、来源与哈希
teleop/robot_control/arm/    S288控制、校准映射、串口与CRC
teleop/robot_control/imu/    LSM6DSV16X
```

S288 对外入口是 `deploy_microduck_s288_v0_backlash_dr_real.py` 和对应 keyboard 脚本。带 J288 名称的模块是继承的内部基础代码，保留名字以维持依赖兼容，不表示另一套公开训练配置。

完整步骤见[部署教程](../../docs/07-deployment.md)和[通讯标定教程](../../docs/04-motor-calibration.md)。
