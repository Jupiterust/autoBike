# 车上 ODrive 版本与参数查询记录

## 查询对象与方法

- 查询时间：2026-10-12T07:48:09-05:00（电脑本地时间）。
- 对象：用户确认的车上 ODrive；USB 序列号 `206B36813152`。
- macOS USB 串口：`/dev/cu.usbmodem206B368131521`。
- 方法：USB CDC / ASCII 属性读取，仅发送 `r <属性路径>` 命令。
- 核对：首次读取后，再独立读取两轮；版本字段及八项参数的数值一致。
- 本次未写配置、保存配置、重启、校准或发送运动指令；未验证电机运行效果。

## 版本与供电

| 字段 | 实际返回的数值 |
|---|---|
| `hw_version_major` | 3 |
| `hw_version_minor` | 6 |
| `hw_version_variant` | 56 |
| `fw_version_major` | 0 |
| `fw_version_minor` | 0 |
| `fw_version_revision` | 0 |
| `fw_version_unreleased` | 1 |

硬件版本字段组成 **V3.6**，variant 为 **56**。固件版本字段组成 **0.0.0**，`fw_version_unreleased = 1`。这些是设备当前回复，不能据此确认其具体源码、正式版本或定制来源。

复读两轮的母线电压分别为 **23.879883 V** 和 **23.818653 V**。

## 两轴参数

| 参数 | 含义 | axis0 | axis1 | 单位（按官方 0.5.x 定义） |
|---|---|---:|---:|---|
| `vel_gain` | 速度环比例增益 | 0.270000 | 0.270000 | N·m / (turn/s) |
| `vel_integrator_gain` | 速度环积分增益 | 0.300000 | 0.300000 | N·m / (turn/s·s) |
| `control_mode` | 控制模式 | 2 | 2 | 枚举 |
| `input_mode` | 输入模式 | 2 | 2 | 枚举 |
| `vel_ramp_rate` | 速度斜坡变化率 | 80.000000 | 80.000000 | turn/s² |
| `current_lim` | 电机电流限制 | 120.00000 | 120.00000 | A |
| `vel_limit` | 速度限制 | 50.000000 | 50.000000 | turn/s |
| `torque_constant` | 电机转矩常数 | 0.040000 | 0.040000 | N·m/A |

按官方 0.5.x 枚举，`control_mode = 2` 表示速度控制，`input_mode = 2` 表示速度斜坡（VEL_RAMP）。由于这块板的固件来源未确认，以上模式解释与单位采用官方定义，尚未通过该板固件源码或运行测试核实。

`vel_limit = 50 turn/s` 换算为 **3000 rpm**，是配置上限，不代表电机实测转速。`current_lim = 120 A` 是配置值，不代表当前电流，也不证明电机或供电能够承受该电流。

## 查询命令

USB 串口每次发送一条命令，以换行结束。下面以 axis0 为例；查询 axis1 时将 `axis0` 替换为 `axis1`。

```text
r fw_version_major
r fw_version_minor
r fw_version_revision
r fw_version_unreleased
r axis0.controller.config.vel_gain
r axis0.controller.config.vel_integrator_gain
r axis0.controller.config.control_mode
r axis0.controller.config.input_mode
r axis0.controller.config.vel_ramp_rate
r axis0.motor.config.current_lim
r axis0.controller.config.vel_limit
r axis0.motor.config.torque_constant
```

## 回复格式与证据边界

车上板回复未呈现标准的纯文本换行格式：数值后出现 `00`（零字节）及额外字节；axis0 数值前观察到 `v`，axis1 前观察到 `c`。复读还观察到前导 `!`，其来源尚未确认。

本记录从零字节前的文本中，严格匹配可选的 `!`、`v` / `c` 前缀及一个数值，再比较两轮结果；未将尾部字节解释为参数。数值重复一致证明查询可重复，但不能证明回复格式完全符合官方协议。

下面保留两轮原始回复的十六进制字节，供后续排查。串口读取在收到零字节后结束，后续尾部字节可能未完整收集。

| 查询属性 | 复读第 1 轮（hex） | 复读第 2 轮（hex） |
|---|---|---|
| `hw_version_major` | `3300ffff3c9c0208189c` | `213300ffff3c9c0208189c` |
| `hw_version_minor` | `213600ffff3c9c0208189c` | `213600ffff3c9c0208189c` |
| `hw_version_variant` | `21353600ff3c9c0208189c` | `2135360040e17aa43f9c49` |
| `fw_version_major` | `213000ffff3c9c0208189c` | `213000ffff3c9c0208189c` |
| `fw_version_minor` | `213000ffff3c9c0208189c` | `213000ffff3c9c0208189c` |
| `fw_version_revision` | `213000ffff3c9c0208189c` | `213000ffff3c9c0208189c` |
| `fw_version_unreleased` | `213100ffff3c9c0208189c` | `213100ffff3c9c0208189c` |
| `vbus_voltage` | `2132332e38373938383300` | `2132332e38313836353300` |
| `axis0.controller.config.vel_gain` | `2176302e323730303030009c` | `2176302e3237303030300049` |
| `axis0.controller.config.vel_integrator_gain` | `2176302e333030303030009c` | `2176302e333030303030009c` |
| `axis0.controller.config.control_mode` | `21763200ffff3c9c0208189c` | `21763200ffff3c9c0208189c` |
| `axis0.controller.config.input_mode` | `21763200ffff3c9c0208189c` | `21763200ffff3c9c0208189c` |
| `axis0.controller.config.vel_ramp_rate` | `217638302e30303030303000` | `217638302e30303030303000` |
| `axis0.controller.config.vel_limit` | `217635302e30303030303000` | `217635302e30303030303000` |
| `axis0.motor.config.current_lim` | `21763132302e303030303000` | `21763132302e303030303000` |
| `axis0.motor.config.torque_constant` | `2176302e303430303030009c` | `2176302e303430303030009c` |
| `axis1.controller.config.vel_gain` | `2163302e323730303030009c` | `2163302e323730303030009c` |
| `axis1.controller.config.vel_integrator_gain` | `2163302e333030303030009c` | `2163302e333030303030009c` |
| `axis1.controller.config.control_mode` | `21633200ffff3c9c0208189c` | `21633200ffff3c9c0208189c` |
| `axis1.controller.config.input_mode` | `21633200ffff3c9c0208189c` | `21633200ffff3c9c0208189c` |
| `axis1.controller.config.vel_ramp_rate` | `216338302e30303030303000` | `216338302e30303030303000` |
| `axis1.controller.config.vel_limit` | `216335302e30303030303000` | `216335302e30303030303000` |
| `axis1.motor.config.current_lim` | `21633132302e303030303000` | `21633132302e303030303000` |
| `axis1.motor.config.torque_constant` | `2163302e303430303030009c` | `2163302e303430303030009c` |

## 参考

- [ODrive 官方 ASCII 查询协议](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.6/docs/ascii-protocol.rst)
- [ODrive 官方 V0.5.4 参数定义](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.4/Firmware/odrive-interface.yaml)
- [ODrive 官方 V0.5.4 模式枚举](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.4/tools/odrive/enums.py)
