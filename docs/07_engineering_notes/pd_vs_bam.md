# S288 Hybrid + BAM M4

本版结构为 `Hybrid 控制律 → M4 机械模型 → 机器人动力学`。机械参数来自随代码附带的 `s288_hybrid_m4.json`，不重新拟合，不混入先前 2S 的电气参数。

令 `u` 为电机输出力矩，`e` 为外部载荷，`v` 为输出等效角速度：

```text
S = exp(-(abs(v)/dtheta_stribeck)^alpha)
L = abs(e-u)
dry_friction = friction_base + load_friction_base*L
             + S*(friction_stribeck + load_friction_stribeck*L)
```

dry friction 再乘原有 DR friction_scale；viscous 项由 `friction_viscous` 给出。外部载荷采用 BAM 的上一物理步估计，并排除自身干摩擦约束。拟合 armature 按输出关节坐标写入，不再乘减速比。`q_offset` 是台架摆臂重力零位，不写成所有机器人关节的偏置。

Kp wire=15 → SDK `15/1280000`，Kd=0；控制律只做一次 `288.35² × ratio` 映射。M4 ratio=1。目标包含原有命令时延 DR 和拟合的分数物理步延迟。

固定输出限幅 ±0.6 Nm；无 XL330 电气模型、反电动势补偿、Vlim/R/Kt/Ke 模型或动态电压限制。共享的 M1 基类仅复用模型编辑、摩擦写入和 DR；运行时的控制、载荷依赖 Stribeck 摩擦及延迟由 M4 子类实现。

模型描述的是本次辨识和训练条件，不能推导所有电机个体或所有供电条件的性能。Pi 直接固件 PD 不等于另外编程了相同的硬件力矩上限。

[接口](../05-control-contract.md) · [仿真](../../simulation/README.md)
