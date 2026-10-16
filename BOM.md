# 备料：做一只鸭子，需要买些什么？

先把电机、电控、供电、线束和机械件分开清点。大件容易记，螺丝、接头和线长也值得在下单前看一遍，省得装到一半再补一单。

这张表对应原鸭头S288 V0。型号、价格和链接没核实的地方保留`TODO`，不会先填一个估计数；未知价格也不计作零成本。

<!-- TODO: 插入全部物料平铺照片，按编号标出型号、接头和数量 -->

## 1. 采购表

币种：`TODO`；价格采集日期：`TODO`。在型号、紧固件和供电路径未选定前，不给出整机总价。

| 编号 | 名称 | 型号 | 数量 | 单价 | 小计 | 购买链接 | 替代条件 | 必需性 | 对应步骤 | 备注 |
| --- | --- | --- | ---: | --- | --- | --- | --- | --- | --- | --- |
| M01 | S288电机 | S288 | 15 | TODO | TODO | TODO | TODO | 必需 | legs/head | 已有记录；具体规格需复核 |
| C01 | 树莓派Zero2W | Zero 2 W | 1 | TODO | TODO | TODO | TODO | 必需 | body/head | 已有记录；具体规格需复核 |
| C02 | USB电机通信板 | 宇树官方，UniArm L1 同款 | 1 | TODO | TODO | 与 S288 电机同一购买入口（链接待补） | 使用对应官方板卡 | 必需 | wiring | 作者确认实物所用；固件版本与接口标注待补 |
| I01 | IMU模块 | LSM6DSV16X | 1 | TODO | TODO | TODO | TODO | 必需 | IMU | 已有记录；具体规格需复核 |
| P01 | 航模锂电池 | 3S LiPo | 1 | TODO | TODO | TODO | 按实际容量、放电能力与接口匹配 | 必需 | power | 作者确认使用3S；品牌、容量、C数与接头待补，模型重量不等于电池规格 |
| P02 | 台架电源 | TODO | 1 | TODO | TODO | TODO | TODO | 备选路径 | power | 与电池为备选动力路径，不同时计最低成本 |
| P03 | Pi供电降压模块 | 12 V 转 5 V | 1 | TODO | TODO | TODO | 输入覆盖所用3S电池电压范围，5 V输出 | 必需 | power | 作者确认接法；具体型号、额定输出电流与接头待补 |
| P04 | 电源开关/保护/急停 | TODO | TODO | TODO | TODO | TODO | TODO | 必需 | power | 规格/型号待核实 |
| W04 | 电源Y型分线 | 电池一分二，接口匹配电池、通信板和降压模块 | 1 | TODO | TODO | TODO | 核对极性、线径与载流能力 | 必需 | power | 一路接电机通信板，一路接Pi降压模块；具体接头待补 |
| W01 | 电机线束/连接器 | TODO | TODO | TODO | TODO | TODO | TODO | 必需 | wiring | 规格/型号待核实 |
| W02 | USB连接线 | TODO | 1 | TODO | TODO | TODO | TODO | 必需 | controller | 规格/型号待核实 |
| W03 | I2C线与转接件 | TODO | TODO | TODO | TODO | TODO | TODO | 必需 | IMU | 规格/型号待核实 |
| S01 | 系统存储卡 | TODO | 1 | TODO | TODO | TODO | TODO | 必需 | software | 规格/型号待核实 |
| F01 | 原鸭头S288打印件 | TODO | 1 | TODO | TODO | TODO | TODO | 必需 | mechanical | 规格/型号待核实 |
| B01 | 脚部螺丝 | TODO | 6 | TODO | TODO | TODO | TODO | 必需 | feet | 数量源于重量记录，直径/长度/头型TODO |
| B02 | 小腿螺丝 | TODO | 8 | TODO | TODO | TODO | TODO | 必需 | lower legs | 数量源于重量记录，直径/长度/头型TODO |
| B03 | 大腿螺丝 | TODO | 16 | TODO | TODO | TODO | TODO | 必需 | upper legs | 数量源于重量记录，直径/长度/头型TODO |
| B04 | 躯干螺丝 | TODO | 24 | TODO | TODO | TODO | TODO | 必需 | trunk | 数量源于重量记录，直径/长度/头型TODO |
| B05 | 颈部螺丝 | TODO | 12 | TODO | TODO | TODO | TODO | 必需 | neck | 数量源于重量记录，直径/长度/头型TODO |
| B06 | 头部螺丝 | TODO | 11 | TODO | TODO | TODO | TODO | 必需 | head | 数量源于重量记录，直径/长度/头型TODO |
| B07 | 轴承/嵌件/螺母/垫片 | TODO | TODO | TODO | TODO | TODO | TODO | 必需 | mechanical | 规格/型号待核实 |
| T01 | 装配工具 | TODO | TODO | TODO | TODO | TODO | TODO | 必需 | assembly | 规格/型号待核实 |
| T02 | 打印/固定耗材 | TODO | TODO | TODO | TODO | TODO | TODO | 必需 | assembly | 规格/型号待核实 |
| T03 | 支撑与防坠架 | TODO | 1 | TODO | TODO | TODO | TODO | 必需 | calibration/deployment | 规格/型号待核实 |
| O01 | 备用电机/线束 | TODO | TODO | TODO | TODO | TODO | TODO | 可选 | maintenance | 规格/型号待核实 |

## 2. 机械件和重量

部件名称、原重量记录及安装状态见[零件重量](hardware/measured_part_weights.md)。可编辑CAD、可打印STL、零件数量和紧固件规格需共同对应同一装配版本。

| 头部部件 | 重量记录 | 说明 |
| --- | ---: | --- |
| top_head_shell | 30.9g | 原鸭头上壳 |
| bottom_head_shell | 26.7g | 原鸭头下壳 |
| face_part | 6.1g | 原配置记录 |
| noenoeil | 2.6g | 原配置记录 |
| motor_support | 8.7g | 电机支撑件 |

整机采购 15 个舵机，其中 14 个关节参与当前 RL。第 15 个舵机不计入 RL 观测、动作和标定模板；其他机械件按对应装配图清点，历史重量不代表当前安装状态。

## 3. 接口与现成线束

| 线束 | 接口 | 长度 | 是否可直接购买 | 是否需焊接/压接 |
| --- | --- | --- | --- | --- |
| 电机总线 | TODO | TODO | TODO | TODO |
| USB控制器连接 | TODO | TODO | TODO | TODO |
| I2C模块连接 | TODO | TODO | TODO | TODO |
| 电池/动力分配 | TODO | TODO | TODO | TODO |
| Pi供电 | TODO | TODO | TODO | TODO |

成品板卡路线不等于所有接头天然匹配。线束型号核实后应给出可直接购买的替代条件，而不是默认裸线拼接。

## 4. 采购复核

- 电机完整后缀、固件、额定条件与安装接口匹配。
- 螺丝直径、长度和头型依装配图确认；数量记录不能代替规格。
- 电池与台架电源是不同使用路径，核对电压、电流和连接器。
- 打印件必须是S288匹配版本，不能凭外观替换XL330承力件。
- 价格记录币种与日期，链接失效时按接口和额定条件找替代，不能只按商品名称。

具体装配与接线见[机械教程](docs/02-assembly.md)和[电气教程](docs/03-electronics.md)。
