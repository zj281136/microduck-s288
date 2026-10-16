# 训练自己的步态

想试不同的步态、了解鸭子怎样学会平衡，可以从这里开始。现有模型、执行器和reward配方已经放在代码里，先沿用同一套配置训练，比较起来更清楚。

如果只是准备跑起配套策略，可以先去[真机部署](07-deployment.md)，不用在这一步等待训练。

## 1. 在训练电脑上准备环境

这里使用Linux、合适的NVIDIA CUDA环境和Python3.12。依赖记录在 `pyproject.toml` 与 `uv.lock` 中；Pi另装轻量运行环境。

```bash
cd software/training
uv sync --locked
uv run python -c "import mjlab_microduck.tasks"
```

`uv sync --locked` 要求项目依赖与锁文件一致。BAM 的 git 来源、protobuf/ONNX 覆盖规则及 aarch64 CUDA 路由已经记录在项目配置中，不需要在系统 Python 中逐个手装训练依赖。

本包沿用 `mjlab_microduck` 名称，应放在独立环境中。成功 import 只能证明模块可加载，不能证明 GPU 驱动和整套运行环境均满足要求。

## 2. 找到这只S288对应的task

```text
Mjlab-Velocity-Flat-S288-v0-BacklashDR-RotorOnly61D-MicroDuck
```

此 task 继承行走配方，并使用 S288 M1 BAM、原鸭头和物理间隙 DR。共享参考模型和帮助函数只作为构造链依赖，不单独注册其他 MicroDuck 训练版本。

## 3. 开始一轮训练

```bash
uv run python -m mjlab.scripts.train \
  Mjlab-Velocity-Flat-S288-v0-BacklashDR-RotorOnly61D-MicroDuck \
  --env.scene.num-envs 4096
```

`num-envs` 控制并行环境数量。4096 是该配方的常用规模，所需显存受模型、GPU 和框架版本影响。资源不足时应调整环境数，而非删除执行器或观测逻辑来降低开销。

默认训练 10000 iterations，使用 TensorBoard。训练产物位于：

```text
logs/rsl_rl/velocity_s288_v0_backlash_dr_rotor_pd_m1/<run>/
```

其中 checkpoint、配置快照与事件文件应一起保留。只保留一个 PT 文件会降低后续复现时确认参数的能力。

## 4. 跑起来后，看看它在学什么

```bash
uv run --with tensorboard tensorboard \
  --logdir logs/rsl_rl/velocity_s288_v0_backlash_dr_rotor_pd_m1 \
  --host 127.0.0.1 --port 6006
```

本机浏览器访问 `http://127.0.0.1:6006`。远程训练机可通过 SSH 转发该端口，避免直接暴露服务。

至少同时观察：任务跟踪项、episode 长度、动作平滑项和总 reward。总 reward 上升不一定表示低速起步或转向已经学好；策略可能只在某些 command 范围表现正常。日志中的加权 reward 也不等于原始物理误差。

## 5. 打开play，亲眼看看动作

```bash
uv run python -m mjlab.scripts.play \
  Mjlab-Velocity-Flat-S288-v0-BacklashDR-RotorOnly61D-MicroDuck \
  --checkpoint-file /path/to/model_9999.pt \
  --num-envs 1 --viewer native
```

将路径替换成同 task 的 checkpoint。`python -m` 避免目录移动后旧 console script 的绝对 shebang 失效，但虚拟环境本身仍建议重新创建，不应复制到另一台机器使用。

play 会使用配方中的 play 配置。比较仿真和实机时，需要另外核对 command、初始姿态、随机推力和其他 DR，不能只看同一 checkpoint 名称。

## 6. 选好checkpoint，再导出ONNX

```bash
uv run python scripts/export.py \
  --checkpoint-file /path/to/model_9999.pt \
  --onnx-file ../deployment/teleop/scripts/models/microduck_s288_v0_backlash_dr_rotor_pd_m1.onnx
```

导出流程会创建对应环境、加载匹配 runner/checkpoint，再通过 mjlab runner 的官方接口输出 `actor(normalizer(obs))`。导出脚本核对 actor 项顺序和 `[1,61] → [1,14]` 维度，并附加本 task metadata。

不能只抽取 MLP 权重手动转换，否则 ONNX 可能缺少训练使用的归一化。in-sim play 通常会自动使用 normalizer，不能用“play 正常”证明手动 ONNX 正确。

## 7. 更新部署 manifest

仓库内置 ONNX 带有同名 `.manifest.json`。重新导出后需要同步更新其模型来源和 SHA256；否则 `deploy.py check` 会因哈希不符而拒绝，防止模型被静默替换。

```bash
sha256sum ../deployment/teleop/scripts/models/microduck_s288_v0_backlash_dr_rotor_pd_m1.onnx
```

manifest 的 `onnx_sha256` 填写新哈希，并更新 checkpoint、iteration 及相关验证记录。仅改哈希不会修复错误的模型来源；task contract、归一化与控制参数仍需匹配。

## 8. 权重发布与复现资料

ONNX 是 Pi 推理产物，PT 用于继续训练和仿真 play，两者用途不同。仓库默认忽略 `*.pt`，较大的训练权重可作为 Release 附件，并附 task、配置、依赖版本、模型哈希及验证记录。

验证范围见 [兼容性与验证](10-local-validation.md)。新训练结果应先检查零速度、低速起步和转向，再进入真机阶段。

<!-- TODO: 插入同版本训练配置、TensorBoard曲线和play截图，注明checkpoint -->

<!-- TODO: 插入训练输出目录、checkpoint文件和参数快照的对应截图 -->
