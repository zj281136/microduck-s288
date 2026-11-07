# Microduck S288 行走训练

公开任务：`Mjlab-Velocity-Flat-S288-MicroDuck`。本版对应已在作者机器人上测试的 `model_4999.pt`，统一了公开命名，没有调整这次成功训练的物理或奖励配方。

## 环境安装

在 Linux / NVIDIA CUDA 训练电脑上运行，使用 Python 3.12：

```bash
cd software/training
uv sync --locked
TASK_ID=Mjlab-Velocity-Flat-S288-MicroDuck
```

## 先做短测试，再正式训练

```bash
uv run python -m mjlab.scripts.train "$TASK_ID" \
  --env.scene.num-envs 64 --agent.max-iterations 5

uv run python -m mjlab.scripts.train "$TASK_ID" \
  --env.scene.num-envs 4096 --agent.max-iterations 5000
```

默认 seed 42，使用 TensorBoard，无需 W&B 账号。日志位于 `logs/rsl_rl/microduck_s288_velocity/<run>/`。

```bash
uv run tensorboard --logdir logs/rsl_rl/microduck_s288_velocity --port 6006
```

## Play 和继续训练

用自己的实际 checkpoint 路径替换示例：

```bash
uv run python -m mjlab.scripts.play "$TASK_ID" \
  --checkpoint-file /path/to/model_4999.pt --num-envs 1 --viewer native

uv run python -m mjlab.scripts.train "$TASK_ID" \
  --env.scene.num-envs 4096 --agent.resume True \
  --agent.load-run YOUR_RUN_DIRECTORY --agent.load-checkpoint model_2500.pt \
  --agent.max-iterations 5000
```

`load-run` 是本任务日志目录下的运行文件夹名。`max-iterations` 是本次追加的迭代数，不是累计迭代终点。仓库附带部署 ONNX；继续训练和 play 需要 PT，PT 没有提交到 Git。

## 导出带归一化的 ONNX

```bash
uv run python scripts/export.py \
  --checkpoint-file /path/to/model_4999.pt \
  --onnx-file ../deployment/teleop/scripts/models/my_walk.onnx
```

脚本调用 mjlab runner，导出 `actor(normalizer(obs))`，核对 61→14 维接口并附加 S288 metadata。不要只转换 MLP 权重。

内置部署入口固定校验已发布策略的 SHA256。自行训练后，需要在完成数值对比和仿真检查后，同步更新 ONNX、同名 manifest 以及 `deploy_microduck_s288_velocity.py` 中的模型哈希。不能仅重命名其他任务的模型来绕过匹配检查。

## 配方与验证

- Hybrid 控制律 + 完整 BAM M4；Kp wire=15、Kd=0、齿比 288.35。
- 仿真力矩固定 ±0.6 Nm，12V 使用条件，不含电气模型或电压 DR。
- 原鸭头 MJCF，保留原部件、质量、惯量和被动传动关节。
- actor 61D / critic 76D，14D action，scale=1，50 Hz，无动作低通。
- 奖励为本次成功版本：upright std=`sqrt(0.5)`、weight=2；airtime 0.125–0.300s、weight=3；腿部限位邻近惩罚；超过 8 rad/s 的速度惩罚及原权重课程。
- 头部指令和范围课程开启；部署键盘入口的 head command 固定零。
- PPO、命令分布、DR、termination 和其他课程保留；完整配置见 `tests/fixtures/velocity_baseline.json`，奖励表见下方链接。

[奖励与课程](05_reinforcement_learning/recipe.md) · [控制接口](05-control-contract.md) · [本次验证](10-local-validation.md)
