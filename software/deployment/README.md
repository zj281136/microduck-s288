# Microduck S288 Deployment

Pi 可独立运行此目录，包含驱动、IMU、标定、50 Hz 控制循环及配套归一化 ONNX。

```bash
bash setup_pi.sh
.venv/bin/python deploy.py check
.venv/bin/python deploy.py keyboard --port /dev/ttyACM0 --i2c-address 0x6a --stand-ramp-seconds 12 --arm-ground-test
```

最后一条会驱动电机；先完成本机标定并准备支撑。W/S 每次 ±0.1 m/s，A/D 每次 ±0.1 rad/s，X/空格归零，Ctrl+C 退出。

Kp15/Kd0，关节速度保护 11 rad/s；跟踪误差和机身角速度保护默认关闭。启动提示保留，运行时仅显示指令，CSV 和 stderr 保留。

对外策略入口：`teleop/scripts/deploy_microduck_s288_velocity.py`。J288/M1 名称的模块为共享驱动或模型加载基础，不是另一套公开策略。

[完整部署教程](../../docs/07-deployment.md) · [标定](../../docs/04-motor-calibration.md)
