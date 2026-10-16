# 03 · 系统、代码与标定：让程序认清这台机器人

进入这一篇之前，先完成安装篇的电脑 USB 舵机接线测试、按图组装和装后的整机通信复测。Pi 和 IMU 从这个阶段才开始安装与配置。

这一篇做完以后，Pi 应该能接替电脑读到整机 15 颗电机，并能读取 IMU、加载策略；随后完成本机关节零位和方向标定，再进入整机站立。

先把“烧录”说清楚：这里需要给树莓派的存储卡写操作系统，再安装 Python 程序。ONNX 是由树莓派读取的模型文件，不需要烧进电机。S288 的 ID 设置已放在安装篇，并附宇树官方快速入门入口。固件版本和刷写工具的本项目记录仍待补充，安装树莓派代码不会自动更改电机固件或 ID。

## 1. 给树莓派准备系统和工程

使用适配 Pi Zero 2 W 的系统镜像，写入存储卡，并配置好联网与登录方式。**本项目经过核实的系统镜像版本、Python 版本和写卡截图还没整理出来**，因此这一步目前不是一套已经完成验收的写卡教程。最终版本需要同时确认 ONNX Runtime 能安装，串口和 I2C 能访问。

系统启动后，在树莓派终端记录环境：

```bash
python3 --version
uname -m
cat /etc/os-release
```

把下载的工程复制到树莓派，保留目录结构。只复制 `software/deployment/` 也可以，这里面包含运行程序和配套 ONNX；不要只拿走 `deploy.py`，它还会调用子目录中的模块。

下面默认你从仓库根目录开始。先进入部署目录：

```bash
cd software/deployment
```

**本篇余下命令都在这个目录执行。** 如果你单独复制了部署文件夹，就进入那个实际文件夹，不再重复追加 `software/deployment`。

## 2. 安装运行环境，先只检查模型

```bash
bash setup_pi.sh
```

脚本会创建 `.venv` 虚拟环境、安装依赖，然后检查默认 ONNX。它不会启动电机。树莓派只需要 NumPy、ONNX Runtime、pyserial、smbus2 等运行依赖，不需要安装训练用的 PyTorch 和 CUDA。

安装成功后，也可以单独重复检查：

```bash
.venv/bin/python deploy.py check
```

输出应包含下面这些字段，另有模型路径和 SHA256：

```json
{
  "input_shape": [1, 61],
  "output_shape": [1, 14],
  "position_source": "rotor",
  "hardware_opened": false
}
```

这表示模型及其记录可以通过当前加载器检查；它还没有打开电机或 IMU。安装失败时，先保存完整报错，并对照刚才记录的系统、架构和 Python 版本。尤其要看 ONNX Runtime 是否有匹配的软件包，不要混用另一台电脑的 `.venv`。

## 3. 找到串口和 I2C

```bash
ls -l /dev/serial/by-id/
ls -l /dev/i2c-*
```

第一条用于找到 USB 电机控制器，第二条查看 I2C 设备节点。若没有对应设备，先解决系统配置、连接或权限，再进行下一步。I2C 的启用和用户权限设置，应随最终系统镜像补成准确步骤。

后面用 `/dev/ttyACM0` 示范，执行时换成你的设备；长期使用更适合填 `/dev/serial/by-id/...` 路径。同一时间只运行一个占用电机串口的程序。

IMU 当前参考配置为 I2C 总线 1、地址 `0x6a`，实际地址和电平仍要与所用模块一致。设备节点出现不代表轴向正确。

## 4. 先读电机，看看线和地址对不对

这里默认已经按安装篇的关节表逐颗设置好 ID。还没有编号的话，先按[宇树官方快速入门](https://support.unitree.com/home/zh/J288-S288%20Servo/Quick_Start)完成设置，再回来读反馈。

把机器人放到能承重的支架上。下面的 probe 使用零增益、零转矩请求读取反馈，电机不会主动撑住机器人。

先读头部四颗，范围小一点，出错时容易定位：

```bash
.venv/bin/python deploy.py probe \
  --port /dev/ttyACM0 --ids 0 1 2 3 --samples 3
```

正常后再读全部 15 颗：

```bash
.venv/bin/python deploy.py probe \
  --port /dev/ttyACM0 --samples 3
```

每条 JSON 对应一次请求。先看 `id` 是否正确，`motor_error` 和 `extra_error` 是否为零，再看位置、速度和电压有没有正常数值。遇到超时、CRC 或 ID 错误，停在这里查电源、地址和线束，不带着缺失反馈继续标定。

读数里有两类位置：`q_rotor_equiv_rad` 是转子折算到输出侧的角度，`q_out_rad` 是输出端编码器角度。当前策略使用前者；它还没有经过本机零位校正，所以此时数值不为零并不奇怪。

整机总线通信检查覆盖 15 个舵机（ID 0–14）；下面的零位、direction 和标定文件仍只服务于 RL 的 14 个关节（ID 0–13），不将第 15 个舵机加入策略数组。

## 5. 捕获零位：告诉程序“这个姿态就是 q=0”

先将关节摆到 **MuJoCo 模型 q=0** 对应的机械姿态。这里最容易出错：把鸭子扶成看起来正常的站姿，并不等于找到了模型零位。训练 HOME 本身就带有弯腿和颈部角度。

**各关节 q=0 的实物照片和舵盘对照图目前待补。** 没有准确参考时，不能靠目测站直来完成这一步。

姿态确认后，首次整机捕获使用：

```bash
.venv/bin/python deploy.py calibrate \
  --port /dev/ttyACM0 \
  --output teleop/scripts/microduck_s288_calibration.json \
  --group all --capture-seconds 3
```

程序会检查电机、等待你按回车，再采集多帧 rotor 位置。采集时保持关节不动。注意显式写 `--group all`，因为默认只选择头部。

结果保存在 `teleop/scripts/microduck_s288_calibration.json`。其中 `encoder_home_rad` 记录传感器参考，`capture_std_rad` 反映采样离散程度，`sample_count` 是有效样本数。读数明显漂动时，先查支撑或通信，再重新捕获。

已有标定文件会先备份，局部重采会保留未选择的关节。换电机、重装舵盘后，需要重新检查受影响的关节。

## 6. 确认方向：这一步电机会动

零位说明“从哪里算起”，方向说明“往哪边算正”。两者都正确，策略的关节目标才有意义。

```bash
.venv/bin/python deploy.py calibrate \
  --port /dev/ttyACM0 \
  --output teleop/scripts/microduck_s288_calibration.json \
  --group all --calibrate-directions
```

程序一次测试一个关节，以较小增益做小幅运动，再询问它是否与模型正方向一致。保持机体受支撑，给测试关节留出空间；判断时对照模型轴，不能把左右镜像关节都简单理解成“往前就是正”。

已经有 `direction=+1/-1` 的条目会保留并跳过。重复运行这条命令不代表旧方向全都重测了；需要重测时先备份，再检查对应记录。

完成后，确认参与 RL 的 14 个关节都有零位与有效 direction。在模型 q=0 姿态下，映射后的位置应接近零；小幅正向运动也应与模型对应。不要复制其他机器的标定文件。

## 7. 安装并单独验证 I2C IMU

![LSM6DSV16X 实际安装近照](../assets/images/imu/lsm6dsv16x-installed.jpg)

这版 LSM6DSV16X 使用 I2C 总线 1，地址为 `0x6a` 或 `0x6b`，按实际扫描结果选择。当前驱动没有自动换轴，传感器 +X/+Y/+Z 必须分别对应机器人前/左/上，并固定在与躯干刚性连接的位置。

完整的安装、I2C 接线、系统配置与独立读数命令已经补在[软件安装教程第 5 节](03_software_setup/environment_setup.md#5-安装-imu并通过-i2c-与树莓派通信)。先读取芯片 ID，再看静止与缓慢倾斜时的 gyro、accel 和 gravity。独立检测不控制电机，不需要通过启动站立程序来测试 IMU。

到这里，模型加载、15 颗电机通信反馈、14 个 RL 关节的零位、方向和 IMU 检查都应有实际结果。接下来可以跳过训练，直接读[部署教程](07-deployment.md)使用配套策略；想自己训练则继续读[训练教程](TRAIN_GUIDE.md)。
