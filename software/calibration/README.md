# 标定工具导航

现有入口保持`software/deployment/deploy.py calibrate`，基础脚本在`teleop/scripts/calibrate_microduck_s288_rotor.py`。实际采样/方向测试由既有实现完成；未修改增益或零位定义。

标定模板在configs/calibration.example.json；它包含null而不能用于运行。步骤、运动提示和停止条件见[标定教程](../../docs/04-motor-calibration.md)。
