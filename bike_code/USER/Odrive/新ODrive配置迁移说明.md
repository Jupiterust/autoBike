# 新 ODrive 已知配置迁移

来源：本目录 `车上ODrive版本与参数查询记录.md`。目标 USB 串口：`/dev/cu.usbmodem2064396658471`，新板报告硬件 3.6、固件 0.5.1，unreleased=1。

## 已完成

两轴均恢复以下旧记录值：

| 参数 | axis0 / axis1 |
|---|---:|
| controller.config.vel_gain | 0.27 |
| controller.config.vel_integrator_gain | 0.30 |
| controller.config.control_mode | 2 |
| controller.config.input_mode | 2 |
| controller.config.vel_ramp_rate | 80 |
| motor.config.current_lim | 120 |
| controller.config.vel_limit | 50 |
| motor.config.torque_constant | 0.04 |

UART 保持启用，波特率从 115200 改为 460800，与当前 STM32 USART3 源码一致；该波特率来自当前源码，不是旧板完整备份。

写入前备份了上述参数和部分状态；逐项写入后回读核对，发送保存命令，随后重启并再次回读。18 项目标配置均匹配，持久化验证通过。重启命令导致 USB 断开，发送脚本因此报 Device not configured；随后已成功重连并验证。

## 校准与上电流程恢复结果

后续在仓库 `../../../odrive/老.txt`（2024-11-27）及 `config_8192cpr_20PP.json` 中找到两轴 20 极对、8192 CPR 的记录。旧文本比 JSON 更新，且速度增益与近期旧板读回值一致，启动流程采用该文本的最终设置，不整包覆盖其他旧配置。

两轴极对数由新板默认的 7 修正为 20。两轴电机电阻/电感校准及编码器偏移校准均通过。保存的两轴启动设置如下：

| 参数 | axis0 / axis1 |
|---|---:|
| motor.config.pre_calibrated | 1 |
| encoder.config.pre_calibrated | 0 |
| config.startup_motor_calibration | 0 |
| config.startup_encoder_index_search | 0 |
| config.startup_encoder_offset_calibration | 1 |
| config.startup_closed_loop_control | 1 |

含义：保留已经测得的电机电阻、电感，每次上电重新校准编码器偏移，随后进入闭环。不是每次重新测电机电阻、电感。

实际重启验证：两轴由状态 7（编码器偏移校准）进入状态 8（闭环），motor.is_calibrated=1、encoder.is_ready=1；motor.error、encoder.error、controller.error 均为 0。两轴速度指令和速度反馈读回均为 0。重启后再次复核上表启动选项均已保留。

## 过程中的异常与证据边界

- 默认 7 极对时，axis0 编码器校准报错误 2（CPR/极对数匹配失败），修正到 20 极对后通过。
- 首次启动验证时 axis1 也报错误 2，扫描反馈为 2031，预期约 3277。用户随后告知曾意外卡住后轮；该现象与校准受阻一致，但不单凭此排除接线等其他因素。
- axis1 随后单独复测通过；两轴同时校准也通过，扫描反馈分别为 3267、3235，采样母线电压约 24.42–24.55 V。未放宽默认 2% 校准误差阈值。
- 轴级错误返回仍带异常尾缀，例如正常时 `0d`、编码器错误时 `256d`；未将整个文本当数值。子错误和实际状态单独检查。
- 已验证新板校准与软件重启后的自动启动；尚未验证实际整车平衡、负载运行或物理断电重启。旧板不能 USB 枚举的问题尚未定位到具体元件。
- 120 A 是按要求恢复的旧配置上限，不是本次实测电流或新板额定能力验证。未整包迁移旧 JSON 的制动、再生电流、保护等参数。

逐条迁移命令及备份见 `新ODrive配置迁移记录.json`；校准、错误、重试与上电状态记录见 `新ODrive校准记录.json`。
