# 工程现状与路径盘点

检查日期：2026-10-16。检查范围为公开目录及同工作区内明确关联的训练、部署、STL和硬件项目；未搜索整个用户主目录、未连接Pi。

## 路径约定

为避免公开个人绝对路径，使用工作区相对名称：`RL_ROOT=microduck_rl/`、`BUS_ROOT=UniArmL1/`、`PUBLIC_ROOT=microduck-s288-open-source/`。下表中的源路径相对RL_ROOT或BUS_ROOT，目标路径相对PUBLIC_ROOT。

## 状态摘要

| 项目 | 观察 | 影响 |
| --- | --- | --- |
| 训练仓 | 基准commit `4be48df27ddab354d0ecdc75cb21fe717d094f79`，工作树非干净 | 基准commit不能完整代表S288本地修改，需保留文件级hash |
| 部署仓 | 基准commit `cae412dfd98342e4f33098350dc0821c91e91d8f`，工作树非干净 | 不覆盖、reset或批量重命名原文件 |
| 公开目录 | 已有中文教程、单task训练包、部署代码和ONNX；未初始化Git | 在此增量整理，不从旧仓直接全量复制 |
| 其他硬件 | UniArmL1硬件目录有机械臂STEP/STL | 属于机械臂，不能作为S288鸭子CAD发布 |
| 外部STL目录 | 四件蜥蜴头STL | 不属原鸭头公开配置，不纳入 |
| SolidWorks/STEP | 在限定相关路径未发现可确认的S288鸭子完整装配工程 | 待提供；不能用机械臂STEP或仿真网格替代 |
| 厂家工具/示例 | 有Unitree Motor Assistant、IMU示例项目 | 不复制二进制工具/固件，许可及必要性待审查 |

## 源路径到公开路径

| 类别 | 现有源文件 | 目标位置/唯一入口 | 复用方式 | 缺口或发布风险 |
| --- | --- | --- | --- | --- |
| 训练 | RL_ROOT/src/mjlab_microduck/tasks/microduck_s288_v0_backlash_dr_env_cfg.py | software/training/src/mjlab_microduck/tasks/同名文件 | 已收录单task依赖闭包 | 新环境完整训练未验收 |
| MDP | RL_ROOT/src/mjlab_microduck/tasks/mdp.py | software/training/src/mjlab_microduck/tasks/mdp.py | 复用所选配方依赖 | 不新增reward/控制模型 |
| 执行器 | RL_ROOT/src/mjlab_microduck/actuator/s288_bam_m1.py、params/s288_m1.json | software/training/src/mjlab_microduck/actuator/ | 复用实现/参数 | 数据分发来源待审核 |
| 仿真 | RL_ROOT/src/mjlab_microduck/robot/microduck/robot_walk_s288_v1.xml及assets | 同结构保留；simulation/README.md索引 | 通过现有spec factory动态加passive | 网格许可与打印可用性分别确认 |
| HOME/质量 | RL_ROOT/src/mjlab_microduck/robot/microduck_constants.py、weight_s288.md | training/robot及hardware/measured_part_weights.md | 保持数值，记录参数来源 | 模型质量不等于整机称重 |
| checkpoint | RL_ROOT/logs/rsl_rl/velocity_s288_v0_backlash_dr_rotor_pd_m1/2026-10-13_18-11-27_s288_v0_model6250_recipe_backlash_dr_2to4deg/model_9999.pt | weights/说明，后续Release附件 | 现有文件4843573 bytes；不移动/复制公开 | 保留SHA256；下载地址待完善 |
| ONNX | BUS_ROOT/teleop/scripts/models/microduck_s288_v0_backlash_dr_rotor_pd_m1.onnx | software/deployment/teleop/scripts/models/ | 已有公开副本；仅run_path元数据去个人路径 | 与原字节hash不同，计算图一致性有历史记录 |
| 导出 | RL_ROOT/scripts/export_s288_v0_backlash_dr.py → scripts/export.py | software/training/scripts/export.py | 精简为单task官方runner导出 | 精简导出入口端到端待验证 |
| 真机入口 | BUS_ROOT/teleop/scripts/deploy_microduck_s288_v0_backlash_dr_real.py、keyboard_real.py | software/deployment/deploy.py | 薄分发器保留原路径和实现 | 会驱动电机，需显式arm参数 |
| 串口驱动 | BUS_ROOT/teleop/robot_control/arm/uniarm_l1_bus.py、constants.py、utiles.py | deployment/teleop/robot_control/arm/ | 原实现复用 | 保留版权；不增加容错或改变单位 |
| 标定 | BUS_ROOT/teleop/scripts/calibrate_microduck_s288_rotor.py及基础模块 | deployment/teleop/scripts/；configs/calibration.example.json | 实际工具+不可运行空值模板 | 个体标定JSON不公开 |
| 方向显示 | BUS_ROOT/teleop/scripts/view_microduck_joint_direction.py | 盘点保留；未作公开推荐入口 | 现脚本依赖原工作区scene路径 | 独立包路径与STAND前提未验证 |
| IMU | BUS_ROOT/teleop/robot_control/imu/lsm6dsv16x.py | deployment/teleop/robot_control/imu/ | 原驱动复用 | 安装方向/引脚图缺失；独立诊断工具待验证 |
| 日志/报告 | RL_ROOT/analysis、outputs及BUS_ROOT/teleop/scripts/logs | engineering notes引用脱敏结论；不复制原日志 | 证据索引先记录范围和限制 | 原始日志含私有路径、网络/设备信息需筛选 |
| BOM/装配 | PUBLIC_ROOT/BOM.md、hardware/*.csv、docs/01–04 | 保留旧链接，新增hardware/bom及章节导航 | 增量扩充字段/来源 | 型号、针序、价格、装配图仍缺 |
| 第三方 | 两仓LICENSE、uv.lock、模型Onshape注释 | NOTICE.md、THIRD_PARTY_NOTICES.md | 保留原许可与修改说明 | 未完成逐文件权属核验，不能声称全部可再分发 |

## 操作证据等级

1. **文件/源码核实**：通过读取实现或文件确认入口、参数和语义，不表示实际运行。
2. **历史记录**：工作区已有导出、配置diff、加载或部署记录，本阶段未复跑。
3. **待核验**：新包装、干净环境安装、原始硬件装配资料或新验证流程。

具体命令状态见[操作证据表](VERIFIED_OPERATIONS.md)。本阶段只读盘点和文件输出，不执行依赖安装、模型推理、物理step、训练或设备操作。
