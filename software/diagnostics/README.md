# 诊断能力与边界

| 工具 | 已实现内容 | 不代表什么 |
| --- | --- | --- |
| diagnostics/read_imu_i2c.py | 独立 I2C 原始加速度、角速度、温度和读取速率 | 不控制电机，不做坐标转换和姿态估计 |
| tools/check_release.py | 源码语法、链接、文件/hash检查 | 不检查机器人 |
| software/tools/inspect_onnx.py | protobuf张量/元数据/hash检查，无推理 | 不检查机械或闭环稳定 |
| deployment/deploy.py check | 通过ORT加载模型/loader检查 | 不打开电机/IMU，不保证硬件安全 |
| deployment/deploy.py probe | 零增益命令和原始回包字段 | 会访问并发送总线帧，不是完全只读设备扫描 |
| deployment/deploy.py calibrate --calibrate-directions | 已有单关节小角度交互方向测试 | 确实运动，不是自动安全验证 |

I2C 原始读取工具已收录，但本次未连接实物执行。机器人坐标与数据率验收、长时通信质量统计、单关节跟随验收尚无本公开目录端到端验证；具体缺口见hardware_validation。不要把未执行的项输出为PASS。
