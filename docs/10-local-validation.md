# 本版验证记录

日期：2026-11-07。任务：`Mjlab-Velocity-Flat-S288-MicroDuck`；策略来源：成功行走训练的 `model_4999.pt`。

| 检查 | 结果 |
| --- | --- |
| 独立环境安装 | 新建 `.venv`，`uv sync --locked` 成功 |
| 配置比较 | 训练和play配置与原任务递归比较一致；仅修改名称、文件路径和默认日志后端 |
| PPO与日志 | seed42、全部算法参数不变；TensorBoard与成功运行保存的agent.yaml一致 |
| 机器人编译 | 14 actuator，14主动关节和14被动关节，质量741.5g |
| 物理数组 | 质量、惯量、CoM、关节、阻尼、armature、transmission、碰撞数组逐项hash一致 |
| CPU回归 | 冻结配置和模型数组、M4载荷相关Stribeck摩擦预算测试通过 |
| GPU短训练 | 64环境、5轮完成；NaN termination=0，所有奖励项正常计算 |
| 正式checkpoint导出 | 使用本包runner/export.py加载model_4999成功，归一化包含在ONNX中 |
| 图与权重 | 原Pi模型、公开ONNX和新包重新导出ONNX的graph/权重序列化相同 |
| ONNX对比 | 128组输入，公开模型/原Pi模型/新导出模型输出差值均为0 |
| PyTorch/ONNX | 同128组输入，最大误差1.0132789611816406e-6，阈值1e-5 |
| 离线部署 | 61→14、metadata、HOME、gains、模型hash、50Hz设置、保护阈值、键盘及输出行为通过 |

公开ONNX SHA256：`d0a8cad7b8a77ab8e20f35a680f59e2d50ac336a9f248f7d1b54013330e92eee`。

模型权重与成功版本相同，仅规范化 contract/dynamics 名称及去掉run_path中的个人路径，因此整个ONNX文件的SHA256发生变化。checkpoint SHA256和文件清单记录于根目录PROVENANCE.json和SOURCE_MANIFEST.json。

CPU数值对比不等于闭环实机测试。原始策略已由作者在其机器人上测试并反馈行走效果良好；本次打包过程没有启动电机、修改Pi或在另一台机器上验证。50Hz是目标频率，实际周期仍取决于Pi负载、串口和传感器时序。

编译时可能提示actuator表达式也匹配site；实际14个actuator均为joint transmission。本次保留原配置，没有为隐藏warning而改变模型。

## 重复检查

仓库根目录：

```bash
python tools/check_release.py
```

训练目录：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --with pytest pytest tests/ -q
```

部署目录（使用已安装运行依赖的Python）：

```bash
.venv/bin/python deploy.py check
.venv/bin/python tests/check_velocity_policy.py
```

依赖安装、短训练与导出命令见[训练教程](TRAIN_GUIDE.md)。以上离线检查不打开串口或IMU；新机器的接线、标定、供电及步态需要单独验收。
