# Sim2real 对齐

本版统一使用 rotor 位置和协议速度、相同 HOME、61D 观测、14D action、Kp15/Kd0及无动作滤波。50Hz控制循环保持时钟相位，ONNX使用单线程。

训练与实际环境仍可能因标定、供电、IMU、接触和装配而不同。实际反馈、command与时序以部署CSV为准。

[接口](../05-control-contract.md) · [本次验证](../10-local-validation.md)
