# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for a self-balancing, single-track "smart bike": a reaction-flywheel balance system with active steering, on an STM32F427IIHx (Cortex-M4, target name `Fly_Dreams`). This directory (`bike_code`) is one part of a monorepo whose git root is one level up (`smartBike/bike/`); siblings you may need for context:

- `../Remote` — handheld remote controller firmware (separate STM32F103C8 project, Standard Peripheral Library not HAL; nRF24L01 wireless link and/or USART).
- `../odrive`, `../my_odrive` — ODrive motor-controller JSON configs/notes for the two motors (flywheel + rear wheel).
- `../PID_Param_Con` — free-text notes on cascaded-PID tuning experience for this bike.

There is no top-level README; this CLAUDE.md is the primary map of the codebase.

## Source encoding — read this before editing anything under `USER/` or `Core/`

C/H sources there are **GBK-encoded** (Chinese comments), not UTF-8, and most also use **CRLF** line endings.

- To read/search: `iconv -f GBK -t UTF-8 USER/Balane/Balane.c`. Plain `cat`/`Read` shows mojibake; `grep` needs `-a` on these files or it silently matches nothing.
- **Never use the `Edit`/`Write` tools on them.** Edit round-trips content as UTF-8 and silently re-encodes the *whole* file on save — a 3-line change came back as a 230-line diff with every Chinese comment in the file corrupted. Edit via raw bytes instead: Python opened `'rb'`/`'wb'` doing an exact `bytes.replace()`, matching ASCII-only substrings so you never type the Chinese bytes yourself.
- Verify after any such edit: `data.decode('gbk')` still succeeds, and `git diff --stat` shows only the lines you meant to touch.
- The GCC build passes `-finput-charset=GBK` so a GBK byte pair ending in `0x5C` inside a comment isn't misread by cpp as a line continuation. Don't drop that flag.

## Build / flash

Two parallel build systems target the same sources. **The GCC/Make one works natively on macOS and is the one to use here.**

```bash
make              # -> build/Fly_Dreams.{elf,hex,bin}, prints size
make flash        # openocd program+verify+reset over CMSIS-DAP
make clean
```

Requires `arm-none-eabi-gcc` (Homebrew cask `gcc-arm-embedded`) and `openocd` on PATH; both are installed at `/opt/homebrew/bin`. `.vscode/tasks.json` wraps these, and `.vscode/launch.json` has a cortex-debug/OpenOCD config against `build/Fly_Dreams.elf`.

The Makefile, `STM32F427XX_FLASH.ld` and `startup_stm32f427xx.s` were generated once by STM32CubeMX from `Fly_Dreams.ioc` and hand-merged: **CubeMX does not know about the hand-added `USER/` tree** (where all the control logic lives), so `USER/**/*.c` and its include paths were added to `C_SOURCES`/`C_INCLUDES` manually. Adding a new source file means editing `C_SOURCES` by hand — and, if the Keil project should stay buildable, adding it there too. Do not regenerate the Makefile from CubeMX without re-merging those lines.

Three Keil(AC5)→GCC compatibility edits are already applied in `USER/` and must survive future edits: `<cmath>` in `A_include.h` guarded behind `#ifdef __cplusplus`, and the `const unsigned short {TX,RX}_PACK_BYTE_SIZE` array-size declarations in `A_Remote.c`/`A_blue.c` converted to `#define`s (C forbids VLA-sized file-scope arrays). `uart.c` still contains AC5-only `#pragma import(__use_no_semihosting)`, which GCC ignores harmlessly.

**Flashing quirks** (already handled — don't re-diagnose): the probe is a "Horco CMSIS-DAP" clone (`0xfaed:0x4870`), not a genuine DAPLink. Its CMSIS-DAPv2/bulk channel never responds on any host or OS, so the HID backend must be forced (`cmsis_dap_backend hid`, see `openocd-horco.cfg`), and even the HID connect handshake fails intermittently at random. All failures happen before any write starts, so the `flash` target just retries up to 8 times.

**Keil / EIDE (Windows only):** `MDK-ARM/Fly_Dreams.uvprojx` (AC5, pack `Keil.STM32F4xx_DFP.3.1.1`) and a mirrored EIDE project at `bikecode_eide/.eide/eide.yml`. AC5 is Windows-only, which is why the GCC build exists. `keilkill.bat` deletes Keil build artifacts. Changes to the source list must be mirrored across Makefile / uvprojx / eide.yml or one build silently drifts.

No test suite exists (embedded control firmware; validation is on real hardware).

## Git hygiene — the working tree is noisy by construction

- The intended ignore files are misnamed `bike.gitignore` / `remote.gitignore` (not `.gitignore`), so git ignores nothing. Compiled Keil artifacts (`.o .crf .axf .hex .map .dep`), JLink logs and per-user `.uvguix.*` UI state are all tracked.
- Nearly every tracked file currently shows as modified purely because the working tree is CRLF and HEAD is LF (~210 files, ~2000 lines each of pure churn). To see what actually changed:
  ```bash
  git diff --ignore-cr-at-eol --numstat -- bike_code | awk '$1!=0||$2!=0'
  ```
- Stage deliberately. Never `git add -A` here.

## Architecture

### Entry point and top-level loop

`Core/Src/main.c` does CubeMX init (GPIO/DMA/TIM2/3/8/12, USART2/3/6, UART7/8; `MX_CAN2_Init()` is present but commented out — CAN is wired but unused), then calls `USER/Init/Init.c: Sys_All_Init()`, which never returns and holds the actual application loop:

1. Inits ODrive state and PID params (`param_init`), starts the flywheel encoder (TIM8), the beeper PWM (TIM12), the servo (TIM2, via `Servo_Init`), USART6 bluetooth RX-DMA, USART2 and UART7 RX-IT, and the CH100 IMU on UART8 RX-DMA. `HAL_TIM_Base_Start_IT(&htim3)` is **deliberately** commented out — see the control-loop trigger below.
2. Blocks on the onboard key, then runs a ~31 s beeping "hold it upright" calibration countdown, toggles `param.M0_Flag` (balance enable) and sets `upper_Flag = 1`.
3. `while(1)`: further key presses toggle balance / set `restart_flag` for a re-stand buffer, and one of two mutually exclusive paths runs, selected at compile time by `#define Button_Only` in `USER/vofa/vofa.h`:
   - **defined** (current default): `vofa_apply()` / `vofa_con()` — VOFA+ debug/tuning path.
   - **undefined**: `uart_data_treating()` / `data_treating()` — the ValuePack path for the remote/upper computer.

   This flag also gates the UART7 RX branch in `uart.c`, so it silently disables one whole control path. Check it first when the bike "ignores the remote" or "ignores VOFA".

### The control loop is IMU-frame-driven, not timer-driven

`USER/uart/uart.c: HAL_UART_RxCpltCallback` is the single RX dispatch point for the whole system, and it is what actually runs the controller:

- `huart8` (CH100 IMU) → `CH100_Rec()` then **`balance()`** — so the control loop runs once per IMU frame (82 bytes @ 460800, ~2.5 ms), and its rate is set by the IMU's output rate.
- `huart2` → `Upper()` when `upper_Flag`, then re-arms RX-IT.
- `huart7` → `vofa_get()` under `Button_Only`, then re-arms RX-IT.

`USER/Time/Time.c: HAL_TIM_PeriodElapsedCallback` *also* calls `balance()` on TIM3 (2 ms), which is why TIM3's IT start is commented out — enabling it would run the controller from two sources at once. If you need a fixed-rate loop, disable the UART8 call site, don't just start TIM3.

### `balance()` — cascaded PID (`USER/Balane/Balane.c`)

Four coupled loops at different sub-rates, divided down by free-running counters inside the one callback. **The counters count IMU frames, not milliseconds** (`cnt>=6` angle loop, `cnt1>=60` flywheel-speed loop, `M1_cnt1>=40` rear wheel), so changing the IMU rate rescales every loop.

- **Steering servo** — clamps `Servo_Ctl` to ±`Servo_Delta`, applies a flywheel-speed-dependent authority limit (`calculateServoLim`) and a slew limit (`Steer_Speed_Limit`), then drives the servo only when `param.M0_Flag && !restart_flag`; otherwise centers it and forces `upper_Flag = 0`.
- **Angle loop** (`X_balance_Control`) — IMU roll vs. a dynamically biased zero point (`param.angular_zero` + flywheel-speed accel bias + `Servo_Gain()` steering-offset compensation + `Fly_Spped_Zero_Gain()`), damped by `imu.vx`.
- **Angular-velocity loop** (`Angle_Velocity`) — turns the angle-loop output into a flywheel speed setpoint, clamped by `fly_wheel_rate_limit` (18).
- **Flywheel speed loop** (`Velocity_Control`) — outer loop on measured flywheel speed, feeding the accel bias back into the angle loop's zero point.
- **Rear wheel** — `M1_Ctl` plus a servo-angle-dependent gain (`calculateGain`), only when `param.M1_Flag`.
- **Safety cutout**: `if (fabs(error_zero) > 5) { param.M0_Flag = 0; param.M1_Flag = 0; }` — 5° of roll error kills both motors. `Balane.c` carries a commented-out `> 10` variant used while tuning.

`param` (`paramTypeDef`, `Balane.h`) holds every gain plus `angular_zero`, and is what tuning sessions edit. `param_init()` has two gain sets in source — a commented-out "有风" (windy) set and the active "无风" (no-wind) one; VOFA keys `'2'`/`'3'` switch between them live. Windy needs a larger angle-loop gain and smaller angular-rate/speed gains (see the newest commit message and `../PID_Param_Con`).

`USER/PID/PID.c` is not a generic PID library — it's the named, purpose-specific control laws used only by `balance()`, plus lookup-table helpers (`Servo_ZERO[20]`, indexed by `Servo_Ctl` in 10-unit steps).

### Sensors and actuators

- **IMU** — `USER/ch100/CH100.c`, CH100 AHRS on `huart8` (DMA, fixed 82-byte 0x5A/0xA5 frames + CRC16), fills the global `imu_t imu`. `imu.rol`/`imu.pit` are assigned from the offsets the source labels Roll/Pitch **swapped**; `imu.rol` is what the balance loop uses, so don't "fix" the naming without checking sign/axis on hardware. `imu.vx` is low-pass filtered (`ALPHA 0.03`) on receipt.
- **Flywheel speed** — TIM8 encoder mode; `Read_Encoder()` reads `TIM8->CNT`, zeroes it, and 3-sample-averages into `odrive.now_speed0`. So flywheel speed is *counts per IMU frame*, not a physical unit, and it does **not** come from the ODrive: `odrive_feedback()`/`odrive_analyze_speed()` exist but are never called.
- **Steering servo** — `USER/Servo/Servo.c`, **TIM2 CH1 on PA0** (20 ms period), written directly via `TIM2->CCR1`. `Servo_Center_Mid 180` / `Servo_Delta 80` in `Servo.h` are specific to the current horn/linkage geometry. (TIM12 CH1 on PH6 is the **beeper**, not the servo — the `BEEP_ON`/`BEEP_OFF` macros in `A_include.h`.)
- **Motors (2× ODrive: flywheel = axis 0, rear wheel = axis 1)** — `USER/Odrive/Odrive.c`. `Odrive.h` defines a full CANOpen-style protocol and `Odrive.c` implements `odrive_speed_ctrl`/`HAL_CAN_RxFifo0MsgPendingCallback` over `hcan2`, but CAN is disabled at `main()` and unused. The live path is `odrive_speed_ctl()`, which hand-formats ODrive's native ASCII `"v <axis> ±NN.NN\r\n"` into a fixed 12-byte DMA transmit on `huart3` — note it only formats 2 integer + 2 fractional digits, so |speed| ≥ 100 wraps silently.

### Host / remote / debug links

Five UARTs, five different protocols:

| Handle | Baud | Role |
|---|---|---|
| `huart3` | 460800 | ODrive ASCII protocol (TX only; RX not used) |
| `huart8` | 460800 | CH100 IMU, DMA — **drives the control loop** |
| `huart6` | 460800 | Bluetooth telemetry/tuning, `USER/A_blue` ValuePack; also the target of `printf` (`fputc` writes `USART6->DR` directly) |
| `huart7` | 115200 | Remote link (`USER/A_Remote` ValuePack) / VOFA input; also the target of `uart_printf()` (DMA) |
| `huart2` | 115200 | "Upper computer", `USER/upper/upper.c` |

- `upper.c: Upper()` parses a 5-byte `STX(0xA5) speed servo checksum ETX(0x5A)` frame; byte 1 is a steering command (0 = auto left/right sweep, 1–5 = discrete increments, 3 = center), byte 2 an index into a fixed rear-wheel speed table (0–3.0).
- `A_blue.c` and `A_Remote.c` implement the same "ValuePack" scheme (`0xa5` … checksum … `0x5a`, configurable bool/byte/short/int/float counts) **twice**, with different macro and type names (`TX_BOOL` vs `TX_BOOL_NUM`, `Tx_Pack` vs `TxPack`, `read_ValuePack` vs `readValuePack`). They are not shared code — a protocol fix in one does not apply to the other.
- `vofa.c` — VOFA+ path. `vofa_get()` parses ASCII `(key,value)` off huart7; `vofa_apply()` dispatches on the key (`'2'`/`'3'` load the no-wind/windy gain sets, `'4'`/`'5'`/`'7'` run servo sweeps via `servo_con`, `'6'` nudges `angular_zero` by +0.01). Large blocks of alternative key mappings are commented out in place — when a VOFA key "does nothing", check which block is live.

### Key globals

- `param` (`Balane.h`) — all gains + `angular_zero`; `M0_Flag` = balance enabled, `M1_Flag` = rear wheel enabled.
- `odrive` (`Odrive.h`) — `set_speed0/1` commanded, `now_speed0/1` filtered feedback (0 = flywheel, 1 = rear wheel).
- `imu` (`CH100.h`) — IMU outputs consumed by `balance()`.
- `Servo_Ctl` / `Servoduty` / `M1_Ctl` — live actuator values written by the protocol handlers, consumed by `balance()`.
- `upper_Flag` / `restart_flag` — gate whether external control input and the balance loop may act; both are cleared from inside `balance()` on fault, so they can change under a handler's feet.
