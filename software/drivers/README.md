# 驱动路径

保持已验证模块原位，不复制另一套驱动：

- 电机协议/CRC：`software/deployment/teleop/robot_control/arm/uniarm_l1_bus.py`。
- 协议常量/单位：同目录`constants.py`、`utiles.py`。
- 机器人坐标/限位：同目录`microduck_288.py`、`microduck_s288.py`。
- IMU：`software/deployment/teleop/robot_control/imu/lsm6dsv16x.py`。

本目录仅提供导航。驱动安装见部署requirements；制造商工具/固件不随仓库重新分发。
