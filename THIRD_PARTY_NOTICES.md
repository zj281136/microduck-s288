# 版权、来源与第三方资源

## 训练代码

训练环境基于 [Pollen Robotics Microduck RL](https://github.com/pollen-robotics/microduck_rl) 的 S288 适配代码，保留 Apache-2.0 LICENSE。代码保留所选 task 的依赖构造链、原始 NaN guard 和 passive-joint 导出处理。

[mjlab](https://github.com/mujocolab/mjlab)、MuJoCo/Warp、rsl_rl 和 [Rhoban BAM](https://github.com/Rhoban/bam) 作为依赖，按各自许可证使用。BAM 的具体来源与版本由 `uv.lock` 记录。

## 部署代码

部署实现基于 UniArmL1 的串口驱动及 Microduck S288 适配。Unitree Robotics 的 Apache-2.0 LICENSE 保留于 `software/deployment/LICENSE`。部分共享基础模块仍使用 J288 文件名。

补充贡献者与原始文件的更细粒度版权来源记录：待完善。

## 机械资产

MJCF 和网格基于上游 Microduck RL 及 S288 适配模型，模型中的 Onshape 来源注释保留。代码许可证不自动决定所有机械设计、网格和商标的授权范围；机械资源应分别记录来源与适用许可。

教程新增引用 [laotawayer/microduck v0.1.0 机械源文件](https://github.com/laotawayer/microduck/releases/tag/v0.1.0)，压缩包未复制到本仓库。外链源文件的使用与再分发遵循[来源项目 LICENSE.md](https://github.com/laotawayer/microduck/blob/main/LICENSE.md)；后续自行适配和新增机械资料另行记录来源与许可。

## 教程与资料组织

教程资料组织参考 [fanhao375/microduck-replica](https://github.com/fanhao375/microduck-replica)。本仓教程依据 S288 配置及实现独立整理，不包含该项目的正文、装配图、图片或 CAD 副本。

## 模型文件

配套 ONNX 来自本仓对应训练配置。公开文件中的 run_path 元数据采用相对 checkpoint 标识；计算图和模型权重保持不变。模型标识、参数与哈希记录在同名 manifest 中。

## IMU 模块资料

`hardware/imu/lsm6dsv16x/` 收录用户提供的 ST 芯片数据手册和模块原理图；`assets/images/imu/` 收录原始模块尺寸图及从原理图 PDF 渲染的预览。原文件中的署名和声明保持不变；模块图纸未附独立许可文本，未将其纳入项目代码的 Apache-2.0 授权。文件用途和来源记录见该目录 README。

`software/diagnostics/read_imu_i2c.py` 来自用户提供的独立 I2C 检测脚本，保留原内容并改名，不含 STM32 HAL 工程或 ST C 驱动副本。

## 宇树电机总线接线说明与图片

安装篇和电气说明按用户提供的段落摘录了[宇树官方「多电机强化学习部署例程」](https://support.unitree.com/home/zh/J288-S288%20Servo/Servo_learning_routine)第 1.3.1 节的总线连接建议，并在 `assets/images/motors/` 保存该节两张原图。文字以引文形式标注，图片保留原内容，来源为宇树科技；这些文档图片未纳入本项目代码的 Apache-2.0 授权。原图地址及文件哈希见图片目录 README。
