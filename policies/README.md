# 预训练策略与兼容性

[manifest.yaml](manifest.yaml)是公共策略目录，模型文件保持原部署路径，不再复制第二份ONNX。无GPU路线可直接使用仓库内的同版本ONNX，但仍需本机硬件核对和标定。

| 配置 | 策略 | 观测 | 电机反馈 | 状态 |
| --- | --- | --- | --- | --- |
| 原鸭头S288 V0 BacklashDR | model9999配套ONNX | 61D rotor | raw转子位置+协议速度，经缩放/零位/方向转换 | 文件/历史接口记录存在；独立复刻验收待完成 |
| 刚性旧V0、XL330、J288、75D、蜥蜴头 | 不适用本策略 | 不应直接替换 | 配方不同 | 不纳入本发布 |

ONNX路径：`software/deployment/teleop/scripts/models/microduck_s288_v0_backlash_dr_rotor_pd_m1.onnx`（相对仓库根目录）。PT保留在原训练工作区，公开下载地址TODO；无PT不影响预训练部署，但无法仅凭ONNX恢复原训练状态。

manifest记录I/O名称、opset、dtype、归一化、关节顺序、增益、校准版本、源文件hash及验证等级。hardware_revision/完整deployment_revision为null，表示硬件尚无正式修订编号，不应自行命名为已认证Rev1。

BAM limit是仿真配置，不等于真实固件设置。metadata里的joint_stiffness/damping可能是BAM底层actuator占位参数，实际SDK增益以pd_config和S288模型为准。

模型、同名JSON sidecar与本YAML目录都应在升级时同步审查；hash一致不保证物理硬件一致。
