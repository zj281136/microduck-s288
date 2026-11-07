# S288 Hybrid BAM：M1–M6

这里收录本次 S288 Hybrid 辨识的 **M1–M6 六套完整参数**、辨识时使用的数学模型/控制律/延迟仿真源码，以及训练/独立验证误差表。原始参数保持不变，没有重新拟合。

当前公开行走任务 `Mjlab-Velocity-Flat-S288-MicroDuck` 使用 **M4**；M1/M2/M3/M5/M6 供对比研究，上传这些参数不会改变训练任务的执行器。

## 参数下载

| 模型 | 参数 JSON | 机械摩擦结构 |
| --- | --- | --- |
| M1 | [m1.json](parameters/m1.json) | Coulomb + viscous |
| M2 | [m2.json](parameters/m2.json) | M1 + Stribeck |
| M3 | [m3.json](parameters/m3.json) | 载荷相关 Coulomb + viscous |
| M4 | [m4.json](parameters/m4.json) | 载荷相关 Coulomb/Stribeck + viscous |
| M5 | [m5.json](parameters/m5.json) | 区分电机侧/外部侧的载荷相关摩擦 + Stribeck |
| M6 | [m6.json](parameters/m6.json) | M5 + 二次载荷耦合 |

## 模型和控制律

- [M1–M6 完整数学模型](reference/bam/model.py)
- [S288 Hybrid 控制律](reference/bam/unitree/actuator.py)
- [本次不使用电压缩放的 Hybrid 配置](reference/bam/unitree/identification12/model.py)
- [单轴仿真与分数目标延迟](reference/bam/simulate.py)
- [训练使用的 M4 mjlab adapter](../training/src/mjlab_microduck/actuator/s288_bam_m4.py)
- [训练使用的同一套 M4 参数](../training/src/mjlab_microduck/actuator/params/s288_hybrid_m4.json)
- [训练/独立验证误差表](model_comparison.csv) · [辨识条件与源码哈希](identification.json)

`reference/` 是可单独加载这六套 S288 模型的最小源码包。数学模型、S288 控制律和 Simulator 保留辨识时快照；基础 scalar/backend 及消息显示 helpers 来自训练锁定的 Rhoban BAM；registry 仅收录 `s288_hybrid12`。保留原版权和 [Apache-2.0 LICENSE](LICENSE)。不是完整的采集/重拟合工具链，原始电机采集日志未收录。

## 条件和单位

用于本次12V供电场景；**辨识台架电源设定值为11.5V**，不能把它写成实测电压精确12.0V。采集/处理100Hz；Kp协议值10/15/20/25/30/40，Kd=0，齿比288.35，位置与速度采用rotor坐标并换算成输出等效量。每条轨迹重复两次，repeat1训练、repeat2独立验证。

角度rad、速度rad/s、时间s、输出力矩Nm、armature为输出关节坐标下的kg·m²；viscous为Nm·s/rad，Stribeck速度为rad/s，alpha/ratio为无量纲。不同阶次的载荷摩擦系数按源码公式使用。

`q_offset` 是台架零位参数，不是14个机器人关节的标定值。JSON中的 `max_torque` 是辨识值；当前行走训练另设固定±0.6Nm限幅。此Hybrid模型不使用电气模型或动态电压限制。

## 加载示例

在已安装依赖的训练目录执行；这条命令只加载模型，不访问硬件：

```bash
cd software/training
PYTHONPATH=../bam/reference uv run python -c 'from bam.model import load_model; m=load_model("../bam/parameters/m4.json"); print(m.name, m.compute_frictions(0.2, -0.4, 0.1))'
```

将路径中的 `m4` 换成 `m1`–`m6` 可加载其他模型。仅在研究参考模型时设置此PYTHONPATH；常规RL训练沿用其锁定依赖和M4 adapter。

本次离线检查：六套参数均成功加载；在0、0.1、-0.5、8rad/s下摩擦预算及粘性项均为有限非负值，Hybrid控制律可计算，voltage_scale固定为1。未访问电机。
