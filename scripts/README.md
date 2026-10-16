# 既有操作的薄包装

这些脚本不改控制逻辑，不注入新的参数，不运行隐式测试。所有命令以仓库根目录为当前目录；创建/安装环境与设备操作需人工执行。

| 脚本 | 委托入口 | 状态/副作用 |
| --- | --- | --- |
| setup.sh deployment/training | 原setup_pi.sh / uv sync --locked | 安装依赖、可能下载；不改系统权限/固件 |
| check_hardware.sh | ls设备节点 | 不打开设备；不是安全检测 |
| calibrate.sh 参数 | deploy.py calibrate | 会访问电机；方向测试会运动 |
| deploy.sh 无参数 | deploy.py check | 加载ORT但不打开硬件 |
| deploy.sh stand/keyboard 参数 | 同名既有入口 | 必须显式arm；会运动 |
| train.sh --run 参数 | 同task mjlab.scripts.train | 开始真实训练；本阶段不执行 |
| export_onnx.sh 参数 | 既有精简export.py | 创建环境/导出文件；完整流程待验证 |

包装脚本仅检查shell语法和静态路径，尚未逐一执行。模型检查不等于硬件验收。详细命令以路线A/B和原始教程为准。
