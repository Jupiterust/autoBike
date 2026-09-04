# 飞轮平衡自行车 — PID 调参交接说明书（给工程目录里的 Claude Code）

> 把本文件 + `pid_tuning_bridge.py` 一起放进 STM32 工程根目录。
> 建议再把本文件的「一、目标与工作流」「六、安全流程」两节要点复制进工程的 `CLAUDE.md`，
> 这样每次会话 Claude Code 都会自动带上背景。

---

## 一、目标与工作流

用户在用「AI 看数据 + 看代码 → 改代码/参数 → 烧录 → 再看数据」的闭环调这台**飞轮/反作用轮自平衡自行车**的**串级 PID**。当前已能自平衡，正在加飞轮/配重，需要重新整定。

闭环里各角色：

- **固件（STM32 + Keil）**：按下面「三、线协议」把调参需要的动态量通过串口二进制发出来。
- **桥接层 `pid_tuning_bridge.py`（Mac 上位机）**：实时收帧、校验、落 CSV、按秒抓窗口并自动算指标。
- **你（Claude Code）**：读桥接产出的 `logs/telem_*.csv` + `logs/summary_*.json`，结合固件源码定位问题，给出「改哪个参数 / 改哪段代码」的具体建议或直接改。
- **用户**：负责烧录，以及提供静态参数（见「五」）。

**关键约定：不实时灌流。** 400Hz 数据不会逐帧进对话。用户抓一段窗口（一次受扰、一次摔倒），你分析那一段的 CSV + summary。时序基准一律用**车上的 `t_ms` / `loop_cnt`**，不要用 PC 接收时刻（串口缓冲会抹平时序）。

---

## 二、你要做的固件改动（第一阶段，最重要）

现状：遥测是 15 个 float 的 ValuePack，但其中 13 个是**静态 PID 增益**，动态量只有 `imu.rol` 和 `set_speed1` —— 三个环里两个是瞎的。要改成发下面这套动态量。

**任务清单（请只做遥测，暂不动控制逻辑，便于隔离验证）：**

1. 在工程里新增一个遥测模块（如 `telemetry.c/.h`），实现：
   - 一个和「三、线协议」`SCHEMA` **逐字段、逐类型对齐**的打包函数 `telem_pack()`；
   - CRC16-CCITT（0x1021, init 0xFFFF），C 参考实现见 `pid_tuning_bridge.py` 顶部注释；
   - 通过现有串口发送（复用当前 ValuePack 用的那个 UART/USB-CDC 即可）。
2. 在 `balance()` 里：维护 `loop_cnt`（每次自增），取 `HAL_GetTick()` 做 `t_ms`，在**控制计算完成、限幅之后**采集所有字段并发送。
3. 加降采样宏 `TELEM_DECIM`（如 2 → 每 2 个控制周期发 1 帧 = 200Hz），减带宽、且和上位机对齐（上位机会自动识别每帧 `loop_cnt` 增量）。
4. 加编译开关 `#define TELEM_ENABLE 1`，方便随时关掉。
5. **务必新增而非改动这些量**：`set_speed0_raw`（限幅前）和 `set_speed0`（限幅后）都要发——两者相减才能算饱和深度/占比，这是自平衡车头号失效模式，只看限幅后永远不知道飞轮憋了多少。
6. 把内环 P/I/D **分项**（`kp*Bias`、`ki*∫`、`kd*ΔBias`）分别发出来（字段 `inner_p/i/d`）。这一步价值最高：一眼看出谁在主导输出、D 项是否在放大噪声。

改完请在回复里列出：你新增/改动了哪些文件与函数、`SCHEMA` 是否与固件打包完全一致、串口波特率、`TELEM_DECIM` 取值。

> ⚠️ 单位/含义要与用户确认（见「五」），尤其 `now_speed0` 的 `enc/2.5ms`、`set_speed0` 的 `turn/s` 限幅值、`fly_gain` 的 ±限幅阈值。这些不确认，指标算得出但解读会错。

---

## 三、线协议（固件与上位机的唯一契约）

一帧（小端）：

```
[0xAA][0x55]        帧头 magic
[LEN]               payload 字节数 (uint8)
[payload ...]       按 SCHEMA 顺序打包
[CRC16_L][CRC16_H]  CRC16-CCITT 覆盖 (LEN + payload)，小端
```

payload 第 0 字段固定 `ver:uint8 == PROTO_VERSION`（当前 =1）。增删字段就改 `SCHEMA` 并 `PROTO_VERSION+1`，两侧同步。

字段顺序 / 类型（= `pid_tuning_bridge.py` 里的 `SCHEMA`，payload 共 **73 字节**）：

| # | 字段 | 类型 | 单位 | 说明 |
|---|------|------|------|------|
| 0 | ver | uint8 | — | 协议版本，必须 =1 |
| 1 | loop_cnt | uint32 | — | balance() 自增序号，精确丢帧检测 |
| 2 | t_ms | uint32 | ms | HAL_GetTick()，算真实环频 |
| 3 | imu_rol | float | deg | 角度环实际值 |
| 4 | angle_target | float | deg | 角度环设定值（合成后） |
| 5 | imu_vx | float | deg/s | 角速度环实际值 |
| 6 | pwm_x | float | deg/s | 角速度环设定值 |
| 7 | set_speed0_raw | float | turn/s | 飞轮指令（限幅前） |
| 8 | set_speed0 | float | turn/s | 飞轮指令（限幅后） |
| 9 | now_speed0 | float | enc/2.5ms | 速度环实际值 |
| 10 | pwm_accel | float | deg | 速度环输出 |
| 11 | fly_gain | float | deg | 饱和解脱项 |
| 12 | servo_zhongzhi_gain | float | deg | 转向补偿 |
| 13 | servo_ctl | float | — | 打舵量（限幅后） |
| 14 | servo_lim | float | — | 当前允许打舵斜率 |
| 15 | inner_p | float | — | kp*Bias |
| 16 | inner_i | float | — | ki*∫Bias |
| 17 | inner_d | float | — | kd*ΔBias |
| 18 | crc_err_cnt | uint16 | — | IMU 链路累计 CRC 错 |
| 19 | flags | uint16 | bitfield | M0/M1/restart/upper/饱和/CRC错 |

波特率建议 ≥ 921600（USB-CDC 可无视）。73B payload、200Hz ≈ 125 kbps，余量充足。

---

## 四、桥接层用法（上位机，用户在 Mac 跑）

```bash
pip install pyserial
python pid_tuning_bridge.py --list                       # 看串口名
python pid_tuning_bridge.py --port /dev/tty.usbmodemXXXX --baud 921600 --log      # 持续录
python pid_tuning_bridge.py --port /dev/tty.usbmodemXXXX --baud 921600 --capture 6 # 抓 6 秒 + 摘要
python pid_tuning_bridge.py --analyze logs/telem_20260904_153000.csv               # 离线重算
```

产出：`logs/telem_*.csv`（逐帧原始，带列名）+ `logs/summary_*.json`（算好的指标）。
**你分析时读这两个文件**，不要让用户把原始流贴进对话。

---

## 五、需要用户补充的静态信息（不必走串口，直接告诉即可）

用这份清单向用户逐项确认；有了它你才能把指标翻译成物理结论、给对的参数方向。

**机械 / 物理**
- 飞轮质量 m、半径 r、厚度；飞轮转动惯量 J_fly（或让我按 J=½·m·r² 估）
- 车+摆整体质量、质心高度 h、绕接触点/铰链的转动惯量估计
- 几何：轴距、飞轮安装位置与朝向（绕哪个轴产生反力矩）

**飞轮电机 / 驱动（ODrive?）**
- ODrive 型号/固件版本、电机 KV、极对数、编码器类型与 CPR
- 电流限幅、速度限幅（`set_speed0` 的 turn/s 上限就是它）
- `now_speed0` 的 `enc/2.5ms` → turn/s 换算系数（编码器一圈计数、周期）

**转向舵机**
- 舵机型号、打舵机械限幅、`servo_zhongzhi` 标定表的来源与是否已验证

**IMU**
- 型号、量程、输出频率；`imu_rol`/`imu_vx` 的定义与正方向；是否已有姿态融合（互补/卡尔曼）及其参数

**控制结构（最关键）**
- 三个环怎么串的：谁的输出 = 谁的设定值（确认是 角度→角速度→飞轮转速）
- 每个环当前的 kp/ki/kd、积分限幅、微分是否加了低通
- 控制频率（确认 2.5ms = 400Hz）、`balance()` 由谁触发（定时器中断？）
- `angle_target` 的合成公式各分项含义（`angular_zero`/`PWM_accel`/`Servo_zhongzhi_Gain`/`Fly_Gain`）

**当前串口现状**
- 端口名、波特率、现在 ValuePack 的确切字节布局、用的是不是匿名/山外之类上位机（决定改动方式）

**目标 / 症状**
- 现在的表现：静态能站多久、受扰恢复情况、加飞轮后新出现的问题
- 想达到的指标（抗扰角度、恢复时间等）

---

## 六、安全流程（务必遵守，平衡车会伤人/自伤）

1. **先上支架或系绳**再通电调参；飞轮高转速有动能，手/线远离。
2. 每次只改**一个环、一个参数、小步长**（典型 ±20%），改完抓一窗对比，别一次动多个。
3. 整定顺序：**先内后外**（先角速度/内环稳，再角度环，最后飞轮速度环的偏置回中），外环别在内环没稳时调。
4. 每次改动前记下旧值；`summary.json` 里若「饱和占比」或「D 项抖动」变差，立即回滚。
5. 飞轮**长期贴限幅**时，不要靠加增益硬顶——先降增益或提升飞轮能力（转速/惯量），否则会越调越飞。
6. 固件保留失控保护（超角度断电、看门狗），AI 调参不替代硬件层保护。

---

## 七、可直接粘贴给项目内 Claude Code 的启动指令

> 这是一台飞轮反作用轮自平衡自行车（STM32+Keil，串级 PID：角度→角速度→飞轮转速，2.5ms/400Hz，飞轮用 ODrive）。
> 请阅读工程根目录的 `CLAUDE_TUNING_BRIEF.md` 与 `pid_tuning_bridge.py`。
> 第一步：按其中「二、固件改动」和「三、线协议」，新增遥测模块，把 SCHEMA 里的动态量按二进制帧发出来（只加遥测，先别动控制逻辑），改完列出改动文件、确认 SCHEMA 与固件打包字节对齐、告诉我波特率和 TELEM_DECIM。
> 之后我会用桥接层抓数据，把 `logs/summary_*.json` 和 `logs/telem_*.csv` 给你，你结合源码分析并给出调参改动。
> 静态参数我按「五」逐项补充。开工前先问我清单「五」里你最需要先确认的 3 项。
