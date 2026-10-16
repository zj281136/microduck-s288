# LSM6DSV16X 模块参考资料

主教程已经把有用的接线和安装信息写进[软件安装教程第 5 节](../../../docs/03_software_setup/environment_setup.md#5-安装-imu并通过-i2c-与树莓派通信)，首次制作沿着教程阅读即可。

| 文件 | 用途与来源 |
| --- | --- |
| [芯片数据手册](lsm6dsv16x-datasheet.pdf) | ST LSM6DSV16X，DS13510 Rev 4，198 页；查芯片接口、量程和寄存器 |
| [模块原理图](module-schematic.pdf) | 用户提供的 SCH_Schematic_LSM6DSV.pdf，V1.0，更新日期 2025-04-08；查 H3/H2 接口、稳压器和上拉电阻 |
| [模块尺寸图](../../../assets/images/imu/lsm6dsv16x-module-dimensions.png) | 用户提供的原始尺寸图；15 × 18 mm，并标有传感器轴向 |
| [原理图预览](../../../assets/images/imu/lsm6dsv16x-module-schematic.png) | 从上述单页原理图 PDF 渲染，供教程内嵌显示 |
| [独立 I2C 读取脚本](../../../software/diagnostics/read_imu_i2c.py) | 用户提供的 test_lsm6dsv16x_i2c.py，原样收录并改名；仅访问 IMU |

原始 PDF 和尺寸图保持内容不变。本目录不是新的 IMU 固件，也不是 imu_to_dxl 工程；树莓派运行 Python I2C 驱动即可。随资料附带的 STM32 SPI 示例及编译产物不属于当前 Pi I2C 路线，未收录。

作者已确认本项目实物用 Pi 物理 1 脚（3.3 V）连接模块 H3.1 VCC。原理图中 VCC 接板载 LDO 输入，内部 `3V3` 是另一节点；当前未记录实测 LDO 输出电压。图片和原理图不代替机器人上的安装位置照片。

这些文件来自用户提供的模块资料，原理图与尺寸图未附单独许可文本；保留原图标记，不将其归为本项目 Apache-2.0 代码。芯片手册保留 ST 原有声明。独立脚本与图纸的上游下载地址未随本地资料提供，未补写推测来源。
