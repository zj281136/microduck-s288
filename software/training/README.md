# Microduck S288 Velocity

唯一任务：`Mjlab-Velocity-Flat-S288-MicroDuck`。

```bash
uv sync --locked
uv run python -m mjlab.scripts.train Mjlab-Velocity-Flat-S288-MicroDuck --env.scene.num-envs 64 --agent.max-iterations 5
uv run python -m mjlab.scripts.train Mjlab-Velocity-Flat-S288-MicroDuck --env.scene.num-envs 4096 --agent.max-iterations 5000
```

Hybrid + M4，Kp15/Kd0，固定 ±0.6 Nm，无电气模型。完整说明、play、resume 和归一化 ONNX 导出见[训练教程](../../docs/TRAIN_GUIDE.md)。

执行器复用共享 adapter 的模型编辑、摩擦写入和 DR 方法；`s288_bam_m1.py` 是保留的内部基类，实际注册并执行的是 `S288BamM4Actuator`，不会退化成 M1。

CPU 配置回归：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --with pytest pytest tests/ -q`。
