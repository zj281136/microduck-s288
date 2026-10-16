# 04 · 训练教程：让仿真里的鸭子先学会走

如果你现在的目标是先让实物动起来，可以跳过本篇，用仓库附带的 ONNX。自己训练适合想研究步态、调整奖励，或者已经改了结构、需要重新匹配策略的情况。

训练可以理解为反复练习：策略尝试动作，仿真计算结果，奖励告诉它哪些行为更接近目标。我们先沿用当前配方，把环境、训练、观察和导出完整走一遍，再考虑修改参数。

## 1. 在训练电脑上安装环境

训练在 Linux 和合适的 NVIDIA CUDA GPU 环境下进行，Python 要求为 3.12。树莓派负责运行策略，不承担这里的大规模仿真。

准备好 `uv`，从仓库根目录进入：

```bash
cd software/training
uv sync --locked
```

**本篇后面的命令都在 `software/training` 中执行。** 项目用 `pyproject.toml` 和 `uv.lock` 记录依赖，先按锁文件安装，不要把另一份 Microduck 工程的虚拟环境复制过来。

安装后确认模块与 GPU：

```bash
uv run python -c "import mjlab_microduck.tasks; import torch; print(torch.__version__); print(torch.version.cuda); print(torch.cuda.is_available())"
```

最后一项应为 `True` 才继续 GPU 训练。若为 `False`，先查驱动和 PyTorch 安装；aarch64 的 CUDA 包来源已经写进项目配置，别随手删掉看起来重复的 torch 固定依赖。

当前配方的 task 名称较长，先存成一个终端变量，后面便于重复使用：

```bash
TASK_ID=Mjlab-Velocity-Flat-S288-v0-BacklashDR-RotorOnly61D-MicroDuck
```

新开终端后要重新执行这行。这个 task 对应原鸭头 S288 V0、M1 BAM、传动间隙随机化和 rotor-only 61 维观测。不要用其他机械版本的 checkpoint 混着试。

## 2. 先跑一个小测试

正式开长训练前，先用 64 个并行环境跑 5 次迭代：

```bash
uv run python -m mjlab.scripts.train "$TASK_ID" \
  --env.scene.num-envs 64 \
  --agent.max-iterations 5
```

这一轮不是为了学会走路，而是确认模型能构建、GPU 能运行、奖励项能计算、观测维度正确，并且没有 NaN。首次启动可能需要编译相关内核，先看终端输出，区分初始化和报错。

只有这一轮正常结束，才增加规模。若显存不足，可以减少并行数；若是关节或观测错误，需要修正配置，不能删掉对应项来绕开。

## 3. 再开始正式训练

```bash
uv run python -m mjlab.scripts.train "$TASK_ID" \
  --env.scene.num-envs 4096
```

当前默认上限为 10000 iterations，4096 是常用并行规模，不是所有 GPU 都能承担的固定要求。训练所需时间与显存取决于设备和环境配置，这里不预估一个未经测量的完成时间。

产物保存在：

```text
logs/rsl_rl/velocity_s288_v0_backlash_dr_rotor_pd_m1/<run>/
```

保留 checkpoint、配置快照和事件文件。checkpoint 通常是 `model_XXXX.pt`，它用于继续训练、仿真查看和导出，不是直接给树莓派加载的文件。

## 4. 看曲线，也看它到底在做什么

另开终端，进入相同训练目录后启动 TensorBoard：

```bash
uv run --with tensorboard tensorboard \
  --logdir logs/rsl_rl/velocity_s288_v0_backlash_dr_rotor_pd_m1 \
  --host 127.0.0.1 --port 6006
```

在训练机本机浏览器打开 `http://127.0.0.1:6006`；远程使用时可通过 SSH 转发端口。

总奖励上升值得高兴，但还要同时看主任务跟踪项、episode 长度和平滑项。策略可能只是更会“少动少扣分”，并没有学会前进。还要检查每个惩罚项的加权贡献应为非正，避免符号写反后反而奖励抖动或碰撞。

选一个已经保存的 checkpoint，打开仿真看看：

```bash
uv run python -m mjlab.scripts.play "$TASK_ID" \
  --checkpoint-file /path/to/model_XXXX.pt \
  --num-envs 1 --viewer native
```

把路径替换成实际文件；新终端中也要先设置 `TASK_ID`。重点看零速能否站住、起步有没有明显冲击、左右转向是否正常，以及头部和脚掌有没有靠不自然的姿态维持平衡。记录实际现象，比只写“reward 很高”更有用。

## 5. 导出时，把归一化一起带走

仿真 play 正常后，用项目脚本导出到**新文件名**，保留原来的配套策略：

```bash
uv run python scripts/export.py \
  --checkpoint-file /path/to/model_XXXX.pt \
  --onnx-file ../deployment/teleop/scripts/models/my_walk.onnx
```

导出脚本会创建对应环境、加载 runner 与 checkpoint，再调用 mjlab 的导出接口。输出应包含训练使用的观测归一化，输入为 `[1,61]`，输出为 `[1,14]`。不要只把网络 MLP 手工转成 ONNX：play 会使用归一化，漏掉它的 ONNX 仍可能在真机表现完全不同。

当前精简导出入口已能从源码核对流程，但还缺独立端到端验收记录。本次文档整理也没有启动训练或重新导出，不能把上面的命令理解成所有平台都已验证通过。

## 6. 新文件准备好以后，还不能只看“能加载”

先保存新模型的文件哈希：

```bash
sha256sum ../deployment/teleop/scripts/models/my_walk.onnx
```

同时记录 task、checkpoint、配置、依赖版本、HOME、关节顺序和验证结果。配套 ONNX 旁边的 `.manifest.json` 是来源记录的格式参考；新模型要填写自己的 checkpoint 和哈希，不能沿用旧模型的数值误差或验证结论。

验证按三个层次做：

1. **文件接口正确。** 核对维度、观测项顺序、metadata、HOME、增益和动作缩放。
2. **同输入得到近似同输出。** 用同一批 61 维观测比较含 normalizer 的原策略和 ONNX，保存误差统计。
3. **ONNX 在闭环里正常。** 在匹配的仿真中实际用 ONNX Runtime 控制机器人，检查站立、起步和转向。

第 2 项的新模型通用验证工具、第 3 项可直接推荐的匹配闭环命令，目前还需要补齐。**PT play 通过不能代替 ONNX 闭环验证。** 在这些验证完成前，先保留新模型作离线研究，不急着替换真机正在用的策略。

还有一个检查范围要记住：`deploy.py check` 检查的是仓库默认内置模型，不接受 `--onnx` 来验证任意新路径。新模型通过上述验证后，真机入口可用 `--onnx` 显式选择，具体见[部署教程](07-deployment.md)。
