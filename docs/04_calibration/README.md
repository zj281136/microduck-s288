# 标定与诊断顺序

1. [电控连接](../03-electronics.md)与系统节点识别。
2. [电机ID、原始值、零位及方向](../04-motor-calibration.md)。ID修改工具TODO，不提供未经验证的写入脚本。
3. [逐项验收矩阵](hardware_validation.md)：单关节、IMU、通信质量、受保护STAND。
4. [首次部署](../07-deployment.md)。

`configs/`中的空值模板仅用于描述格式，不自动加载，也不提供任何通用机器零位。

[单关节、IMU与通信测试说明](testing.md)区分既有工具的真实能力、未完成的独立检查及配图位置。
