# autoBike

飞轮（反作用轮）自平衡自行车，含整车固件、遥控器固件与调参上位机。

车体靠一个反作用飞轮产生反力矩维持横滚平衡，同时用舵机主动转向辅助扶正，
后轮驱动前进。两台电机（飞轮 / 后轮）由 ODrive 驱动。

## 目录

| 目录 | 内容 |
|---|---|
| `bike_code/` | 整车固件，STM32F427IIHx，CubeMX + HAL，工程名 `Fly_Dreams` |
| `Remote/` | 手持遥控器固件，STM32F103C8，标准外设库，nRF24L01 / 串口 |
| `odrive/`, `my_odrive/` | 两台电机的 ODrive 配置与记录 |
| `PID_Param_Con/` | 串级 PID 调参经验笔记 |

## 控制结构

串级 PID：**角度环 → 角速度环 → 飞轮转速环**，另有转向舵机环与后轮环。

控制周期 2.5 ms（400 Hz），但**不由定时器驱动** —— `balance()` 挂在 CH100 IMU
的串口接收完成回调上，每收到一帧 IMU 数据跑一次，所以环路频率等于 IMU 输出率。
四个环用同一个回调里的自增计数器分频，计数单位是 IMU 帧而非毫秒。

保护：横滚角偏离平衡点超过 5° 立即切断两台电机。

## 构建

Mac / Linux 用 GCC：

```bash
cd bike_code
make          # -> build/Fly_Dreams.{elf,hex,bin}
make flash    # 经 CMSIS-DAP 用 openocd 烧录
```

需要 `arm-none-eabi-gcc` 与 `openocd`。

Windows 下也可用 Keil MDK（`bike_code/MDK-ARM/Fly_Dreams.uvprojx`，AC5）或
EIDE 工程。**新增源文件时三处的文件列表都要同步**，否则某一边会静默漂移。

## 调参遥测

固件把控制环内部量以二进制帧发出（`0xAA55` + 长度 + payload + CRC16-CCITT），
走 USART6 @460800，200 Hz。上位机 `bike_code/pid_tuning_bridge.py` 收帧、校验、
落 CSV 并自动算指标（真实环频、丢帧、飞轮饱和占比、各环误差、内环 P/I/D 分项
占比、微分项抖动）。

```bash
python pid_tuning_bridge.py --list
python pid_tuning_bridge.py --port /dev/tty.usbserial-XXXX --baud 460800 --capture 6
```

调参流程与安全注意事项见 `bike_code/CLAUDE_TUNING_BRIEF.md`，
当前进度见 `bike_code/TUNING_PROGRESS.md`。

> ⚠️ 调参务必先上支架或系绳。飞轮高速旋转储有可观动能。

## 源码编码

`bike_code/USER/` 与 `bike_code/Core/` 下的 C/H 文件是 **GBK 编码**（中文注释）、
CRLF 行尾。读取需 `iconv -f GBK -t UTF-8`；GCC 构建已带 `-finput-charset=GBK`，
不要去掉 —— 否则注释里以 `0x5C` 结尾的 GBK 字节对会被预处理器当成续行符。
