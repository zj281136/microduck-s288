# 从 checkpoint 到 ONNX 与真机

## 1. 两种模型格式

训练保存的是 PT checkpoint，包含策略权重、归一化状态以及训练所需的其他信息。ONNX 是用于部署推理的导出格式，不是训练过程直接替代 checkpoint 的文件。

- PT：用于同 task 的 play、训练恢复和导出。
- ONNX：由 ONNX Runtime 推理，用于 Pi 运行和对应仿真验证。

本仓公开策略为原鸭头 V0 BacklashDR rotor-only61D，不能用其他观测或机械配置的 checkpoint 替换。

## 2. 导出前核对

确认 task、checkpoint、关节顺序、HOME、SDK增益、action scale 和归一化来自同一配置。已有的 `model_9999` manifest 可作为格式参考，不能将其中哈希复制给新文件。

<!-- TODO: 插入 checkpoint、配置文件和 ONNX 的关系示意图 -->

## 3. 使用已有导出入口

在 `software/training` 中人工执行：

```bash
uv run python scripts/export.py \
  --checkpoint-file /path/to/model_9999.pt \
  --onnx-file ../deployment/teleop/scripts/models/NEW_POLICY.onnx
```

导出使用 mjlab runner 接口，不手工剥离 MLP。`EmpiricalNormalization` 应进入图中。新文件名可保留现有预训练策略，避免静默替换。

该命令会创建环境和加载 checkpoint；精简导出入口的独立端到端验收仍为 `TODO`。本教程记录实现路径，不把源码存在等同于已完成所有平台验证。

## 4. 当前配套 ONNX 接口

| 项目 | 值 |
| --- | --- |
| opset | 18 |
| 输入 | `obs`，float32，`[1,61]` |
| 输出 | `actions`，float32，`[1,14]` |
| 额外历史帧输入 | 无；last_action已在61D中 |
| RNN隐藏状态输入 | 无 |
| observation normalizer | 已烘焙到ONNX |
| 动作语义 | HOME + action_scale × action |

完整维度顺序见[控制接口](../05-control-contract.md)。这里的结构是当前配套模型的信息，新导出文件仍需重新核对。

## 5. 验证分为三个层次

### 文件与接口

核对张量名称、shape/dtype、metadata、SHA256及manifest。文件加载成功不能证明反馈来源、标定和机械状态正确。

### 同输入数值一致性

使用同一批61D观测比较原策略与ONNX输出，原策略必须包含对应normalizer。历史同版本记录为128帧、最大绝对差约1.56e-7；这是已有记录，不是对任意新模型的保证。

新模型应保留观测来源、数量、差值统计和阈值依据。离线数值检查不应接硬件，也不等同于闭环稳定性验证。

### ONNX闭环仿真

需要在匹配的MuJoCo环境中实际使用ONNX Runtime运行，而不是仅play PT。应保持关节选择、被动传动、command时序、action处理、归一化与部署一致。

当前可作为唯一公开配置推荐的已验证 ONNX/MuJoCo 闭环命令：`TODO`。不得把不同实验版本脚本或PT play结果写成已通过。

<!-- TODO: 插入同输入PT/ONNX误差图，注明模型哈希和观测来源 -->
<!-- TODO: 插入同版本ONNX闭环仿真的零速、起步和转向截图/曲线 -->

## 6. 模型与元信息一起更新

每份新策略至少记录：硬件/机械版本、task、checkpoint标识、ONNX SHA256、joint顺序、obs/action顺序、normalizer、HOME、频率、增益、动作缩放、标定格式和部署入口。

查看文件哈希：

```bash
sha256sum ../deployment/teleop/scripts/models/NEW_POLICY.onnx
```

`policies/manifest.yaml`与同名JSON sidecar是现有记录位置。当前统一 `deploy.py check`检查默认内置模型，不应将它通过当作任意新路径也已验证。

## 7. 进入真机部署

完成接口、数值与所需闭环验证后，显式选择新ONNX，并使用本机rotor标定。先受保护零速站立，再固定低速；不得为通过测试先修改Kp/Kd、过滤target或提高超速阈值。

详细命令和停止流程见[真机部署](../07-deployment.md)。
