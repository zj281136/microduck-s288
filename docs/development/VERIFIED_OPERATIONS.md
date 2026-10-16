# 操作证据与命令状态

“历史记录”不等于本整理阶段重新执行；“源码核实”不等于在独立机器验证成功。

| 操作 | 真实入口 | 证据状态 | 本阶段是否执行 |
| --- | --- | --- | --- |
| 原训练 | RL_ROOT的mjlab.scripts.train + 唯一task | 有model9999及训练产物 | 否 |
| 原导出 | RL_ROOT/scripts/export_s288_v0_backlash_dr.py | 原ONNX/manifest及verify_export记录 | 否 |
| 当前精简导出 | software/training/scripts/export.py | 源码使用官方runner，完整导出待验证 | 否 |
| 静态发布检查 | python tools/check_release.py | 上次检查有输出；本阶段将复查纯文件项 | 仅静态 |
| 部署模型加载 | software/deployment/deploy.py check | 历史loader/哈希记录；会加载ORT但不打开设备 | 否 |
| 零增益反馈 | deploy.py probe | 实际SDK命令路径已核对，公开probe包装独立实机验证待完成 | 否 |
| rotor零位/方向 | deploy.py calibrate | 原工具有使用记录；方向测试确实运动 | 否 |
| STAND/键盘 | deploy.py stand/keyboard | 对应原入口有部署记录；不能据此保证其他机器可行走 | 否 |
| 同步 | tools/sync_pi.sh | 包装源码存在；不自动连接Pi | 否 |
| ONNX+MuJoCo | 待完成匹配单task入口 | 旧研究脚本跨版本；未认定为本公开配置的已验证流程 | 否 |

历史记录定位：RL_ROOT/outputs/s288_v0_backlash_dr_deployment/、outputs/s288_open_source_packaging/，以及相关S288报告。这些目录不整体打包公开，以免传播私有数据或混入其他实验模型。
