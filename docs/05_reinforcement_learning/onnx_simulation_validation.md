# ONNX 导出与仿真验证

## 现有链路

[训练/导出教程](../06-training.md)说明checkpoint与ONNX的不同用途。导出使用官方mjlab runner，将normalizer写入图。当前模型opset18，`obs:float32[1,61] → actions:float32[1,14]`，无额外输入历史栈或RNN隐藏状态；last_action属于61D内部字段。

## 纯文件检查（不运行推理）

在software/training目录，已有依赖环境下人工执行：

```bash
uv run python ../tools/inspect_onnx.py   --model ../deployment/teleop/scripts/models/microduck_s288_v0_backlash_dr_rotor_pd_m1.onnx
```

工具只读取ONNX protobuf与JSON manifest，输出真实形状、opset、metadata和hash；不创建ORT session、物理模型或设备。

## 相同输入的数值对照（人工执行，新增入口待验证）

`software/tools/compare_policy_outputs.py`复用原离线对照的rsl_rl MLPModel/normalizer接口。输入必须是可信checkpoint、匹配ONNX及真实61D顺序的`.npy (N,61)`观测。运行后会执行CPU推理，不接硬件；本阶段不运行。

```bash
uv run python ../tools/compare_policy_outputs.py   --checkpoint /path/to/model_9999.pt   --onnx ../deployment/teleop/scripts/models/microduck_s288_v0_backlash_dr_rotor_pd_m1.onnx   --observations /path/to/observations.npy
```

历史sidecar记录128观测最大误差约1.56e-7，属于此前对照，不能标记为本新入口已通过。无已公开匹配PT/观测时该步骤被输入缺失阻塞。只加载可信PT，torch.load不是不可信文件沙箱。

## ONNX Runtime + MuJoCo闭环

当前明确列为P1缺口。原工作区有多种旧版本CPU/MJLab脚本，尚未确认一个入口同时匹配此单task的模型、主动rotor观测、被动DR、command时序和目标路径，故不直接复制成“已验证”工具。

验收应至少包括零速、固定小vx、ramp与step命令；记录真实ONNX输入/action/target、active q/qdot、passive角、IMU和是否终止。`play PT`成功不能当作ONNX闭环验证，单次输出一致也不能证明闭环稳定。

不得用构造大残差、额外滤波或改PD来让验证看起来通过。
