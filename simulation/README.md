# 仿真模型唯一入口

模型文件不搬移或复制成第二份。训练task通过`software/training/src/mjlab_microduck/robot/microduck_constants.py`中的`get_s288_v0_backlash_dr_spec()`构造最终模型。

1. 载入同目录`microduck/robot_walk_s288_v1.xml`及assets。
2. 应用V0电池155g和头部Pi18g记账。
3. 为14个受控关节添加passive backlash hinge。
4. 在task factory中固定本版本SDK Kp/Kd和0.55Nm限幅，并添加episode间隙DR。

直接打开基础XML不能代替最终训练spec。参数来源见[parameter_sources.csv](parameter_sources.csv)，精确观测见[控制接口](../docs/05-control-contract.md)，训练启动见[训练教程](../docs/06-training.md)。

本阶段未编译/运行仿真。历史静态编译结果见验证记录；匹配ONNX的闭环MuJoCo运行流程仍待验证。
