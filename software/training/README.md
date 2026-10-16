# S288 rotor-only 强化学习训练

本包注册一个 MicroDuck task：

```text
Mjlab-Velocity-Flat-S288-v0-BacklashDR-RotorOnly61D-MicroDuck
```

## 安装与训练

```bash
cd software/training
uv sync --locked
uv run python -m mjlab.scripts.train \
  Mjlab-Velocity-Flat-S288-v0-BacklashDR-RotorOnly61D-MicroDuck \
  --env.scene.num-envs 4096
```

运行环境为 Python 3.12、Linux 和匹配 CUDA 依赖的 NVIDIA GPU。Pi 只安装部署依赖，不安装此训练环境。

`src/mjlab_microduck/tasks/` 包含配方、MDP、runner 和观测约定；`actuator/` 包含 S288 M1 BAM；`robot/` 包含 HOME、模型工厂、MJCF 和网格。共享配置仅保留本 task 构造链需要的部分。

模型输出用于仿真 play；部署前需通过官方 runner 路径导出带归一化的 ONNX，并同步维护 manifest。详细命令见[训练与导出教程](../../docs/06-training.md)。
