# 部署策略

配套文件：`microduck_s288_v0_backlash_dr_rotor_pd_m1.onnx`。

| 属性 | 值 |
| --- | --- |
| task | 原鸭头 S288 V0 BacklashDR rotor-only 61D |
| checkpoint | model_9999 |
| 输入 / 输出 | `[1,61]` / `[1,14]` |
| observation normalizer | 已包含在 ONNX 中 |
| SDK Kp / Kd | `1.3545e-4` / `3.74e-6` |
| 仿真力矩限幅 | 0.55 Nm @ 12 V |

来源、模型标识和 SHA256 记录于同名 manifest。模型 loader 校验 contract 和接口；`deploy.py check` 额外核对 manifest 哈希。

替换模型时需同时核对 task、关节顺序、HOME、归一化和增益，并更新 manifest。每台机器人仍需独立标定，模型文件不包含通用机械零位。
