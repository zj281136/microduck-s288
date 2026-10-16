# 兼容性与验证记录

参考配置：原鸭头 S288 V0 BacklashDR rotor-only 61D。记录日期：2026-10-15。

## 1. 已完成的检查

| 检查项 | 结果 |
| --- | --- |
| MicroDuck task 注册 | 本包仅注册一个公开 task |
| 环境配置 | 与对应参考 task 递归比较一致 |
| 编译模型 | 14 actuator、14 active hinge、14 passive hinge、1 free root |
| 模型质量 | 733.3 g |
| 关键物理数组 | body 质量/位姿/CoM/惯量、joint、damping、armature、transmission 一致 |
| 依赖锁一致性 | `uv lock --check --offline` 通过 |
| ONNX loader | 61→14、contract、HOME、action scale 和增益校验通过 |
| ONNX 计算图 | 与配套原始导出图和权重一致 |
| 统一部署入口 | `deploy.py check` 加载与哈希核对通过 |
| 静态文件检查 | Python/shell 语法、相对链接、单 task 注册和模型哈希通过 |

上述配置比较在现有 Python 3.12 依赖环境中进行，使用独立导入路径确认加载本包源码。锁文件检查验证项目声明与锁一致，不等于完成干净机器依赖安装。

## 2. 模型标识

```text
microduck_s288_v0_backlash_dr_rotor_pd_m1.onnx
SHA256: b04484f1432d6199bc7c9e898fc52229462cdd55d32bdf4c39e11da6ae482f09
```

模型源于同 task 的 model_9999。公开文件的 run_path 元数据采用相对标识；图与权重未变化。

## 3. 检查范围

静态模型编译、配置比较和模型加载不包含闭环 rollout。当前记录没有对精简包重新训练，也没有完成全新机器上的安装和真机复现；这些工作列于[资料完善计划](09-author-checklist.md)。独立导出脚本还需要结合后续公开 checkpoint 验证完整导出流程。

部署检查不打开电机或 IMU，因此不能验证电气连接、实机标定、实际频率、温升和行走稳定性。

继承构造链可能提示 actuator 表达式也匹配到 site；编译结果确认 14 个 actuator 均为 joint transmission。检查时保留该 warning，不通过变更配方来隐藏提示。

## 4. 可重复执行的静态检查

在仓库根目录执行：

```bash
python tools/check_release.py
```

该工具检查源码语法、相对文档链接、task 注册数量、模型 manifest 哈希，以及意外收录的真实标定/PT。它不依赖 MuJoCo，也不执行任何真机动作。

Pi 运行环境还可执行：

```bash
cd software/deployment
.venv/bin/python deploy.py check
```

## 2026-10-16 资料整理范围

以下内容属于历史资料整理的检查记录，不应当作当前机器已通过硬件验收。文档维护不重新执行模型加载、仿真、训练或真机测试；现有程序的适用范围见操作证据表。
