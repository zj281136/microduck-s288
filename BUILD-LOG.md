# 版本记录

## S288 V0 BacklashDR rotor-only 61D

- 原鸭头，整机 15 个 S288，其中 14 个关节参与 RL。
- policy 位置/速度来自 rotor 侧，OutPos 保留诊断用途。
- M1 BAM、匹配 SDK 增益及物理被动传动间隙。
- 配套训练 checkpoint 标识：`2026-10-13_18-11-27_s288_v0_model6250_recipe_backlash_dr_2to4deg/model_9999.pt`。
- 包含 61→14 ONNX、Pi 控制、键盘、通讯和标定源码。

## 文档与资源

机械制作过程照片、打印文件版本、硬件采购及独立复现记录待完善。新增资料应与本配置对应，并注明测试条件和实际结果。

模型兼容性和检查结果见[验证记录](docs/10-local-validation.md)。
