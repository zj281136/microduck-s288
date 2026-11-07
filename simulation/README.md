# S288 仿真模型

唯一训练模型：`software/training/src/mjlab_microduck/robot/microduck/robot_walk_s288_duck.xml`，由 `get_s288_spec()` 加载，EntityCfg 应用 HOME、碰撞参数和 Hybrid/M4 执行器。

它保留成功训练时的头部、腿部、传动关节、质量、惯量、碰撞体与质心。模型总质量 741.5g；包含单独的104g电池、21g Pi和23.7g嘴部舵机记账。这些是保留的仿真数值，不是对任何复刻整机的重新称重。

没有旧版 NPF550 工厂或额外电池替换逻辑；不添加旧任务的传动间隙 DR。M4 参数和固定0.6Nm限幅由当前配置给出。

[控制接口](../docs/05-control-contract.md) · [训练教程](../docs/TRAIN_GUIDE.md) · [参数来源](parameter_sources.csv)
