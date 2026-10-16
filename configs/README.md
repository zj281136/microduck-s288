# 公共配置模板

这些文件是参数记录模板，不是新增的运行时配置加载器；现有代码仍使用Python配置及CLI。`documentation_only: true`表示YAML不会被deploy.py自动读取。null表示缺少信息，不能解释为自动检测。

`calibration.example.json`展示实际format_version=1、14个RL关节的标定结构（整机第15个舵机不在此模板内）。encoder_home/direction均为null，故意不可用于运动。真实标定应由工具生成`microduck_s288_calibration.json`，不能把模板改名后直接部署。

参数的来源/不确定性见simulation/parameter_sources.csv，电机ID与policy顺序是不同排列。
