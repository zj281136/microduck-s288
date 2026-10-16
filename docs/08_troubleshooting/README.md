# 按现象排查

详细步骤见[故障排查](../08-troubleshooting.md)，验收/停止条件见[硬件验收](../04_calibration/hardware_validation.md)。

| 分类 | 现象 | 优先顺序 | 停止条件 |
| --- | --- | --- | --- |
| 硬件 | 控制器不识别/低电压 | 节点→供电→线缆/接头→板卡 | 极性不清、发热或电压异常 |
| 标定 | 方向错、偏离HOME | q=0定义→ID映射→rotor来源→direction→HOME | 不明确正方向/意外运动 |
| 通讯 | 掉线、CRC、ID冲突 | 端口占用→ID→供电→线束长度/拓扑→控制器 | 错误持续，停止运动保存证据 |
| 部署 | ONNX维度错/无法站立 | task/contract→normalizer→校准→IMU→target/dynamics | 倾倒、超速或闭环振荡 |

不以放宽max-speed、following error或容错次数作为第一步。未验证的新硬件不能只凭模型check通过进入行走。
