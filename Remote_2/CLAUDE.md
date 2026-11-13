# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`Remote_2` is the second-generation **handheld remote firmware** for the autoBike
reaction-flywheel self-balancing bike: STM32F103C8T6 (Cortex-M3, 72 MHz),
STM32CubeMX 6.15 + HAL, Makefile/GCC toolchain.

**It is currently a bare CubeMX skeleton.** `main()` is `HAL_Init()` +
`MX_*_Init()` + an empty `while(1)`; every `USER CODE BEGIN/END` block is empty;
there is no application code, no `USER/` tree, no drivers of its own. The whole
image is 8 KB. Everything below about *behaviour* describes what this firmware
has to become, sourced from the two projects it sits between.

The git root is one level up (`smartBike/bike/`), and `Remote_2/` is **untracked** —
nothing in it has ever been committed. Siblings:

- `../Remote` — the v1 handset, same physical hardware, Standard Peripheral
  Library + register-level `sys_*` helpers under Keil. It is the functional
  spec for this port, not code to copy verbatim.
- `../bike_code` — the bike firmware (STM32F427, `Fly_Dreams`). It has its own,
  much more detailed CLAUDE.md; read it before touching anything on the bike side.
- `../README.md` — project overview (Chinese).

## Build

```bash
make          # -> build/Remote_2.{elf,hex,bin}, prints size
make flash    # openocd program+verify+reset over CMSIS-DAP, 8 retries
make clean
```

Verified working with `arm-none-eabi-gcc` 15.3 and `openocd` 0.12 from
`/opt/homebrew/bin`. `build/` is covered by the repo-root `.gitignore`.

The `flash` target and `openocd-horco.cfg` were added by hand — the CMSIS-DAP
probe is a "Horco" clone whose CMSIS-DAPv2/bulk channel never responds, so the
HID backend has to be forced (`cmsis_dap_backend hid`) and even the HID connect
handshake fails at random. All failures happen before any write starts, hence
the retry loop. Same story as `../bike_code`; details in its CLAUDE.md.

**The Makefile is CubeMX-generated**, so regenerating from the .ioc wipes the
`flash` target and the `.PHONY` line. Re-add them (they sit just above
`# clean up`).

`.vscode/` has `tasks.json` (⇧⌘B → `build`), `launch.json` (F5 → cortex-debug
over OpenOCD, `preLaunchTask: build`), `c_cpp_properties.json` for IntelliSense,
and `extensions.json` recommending `ms-vscode.cpptools` + `marus25.cortex-debug`.
`settings.json` here pins UTF-8 + CRLF — do **not** copy bike_code's
`settings.json`, which forces `gb18030` on C/H files for its GBK sources.

## CubeMX regeneration

`Remote_2.ioc` is set to `TargetToolchain=Makefile`, so regenerating rewrites
`Core/`, `Drivers/`, `Makefile`, `STM32F103XX_FLASH.ld` and the startup file.

- Application code must live inside `USER CODE BEGIN/END` markers, or better, in
  an out-of-tree `USER/` directory the way `../bike_code` does it.
- If you add a `USER/` tree, `C_SOURCES` and `C_INCLUDES` in the Makefile have to
  be edited by hand **and re-merged after every CubeMX regeneration** — CubeMX
  does not know the tree exists. bike_code hit exactly this and documents it.
- `.mxproject` is CubeMX's record of the last generated file list; don't hand-edit it.

## Source encoding — the rule here is the opposite of bike_code's

Everything under `Remote_2/` is plain **ASCII with CRLF**. Normal `Read`/`Edit`
are safe on these files.

The files you will be *porting from* are not: `../Remote/**` and
`../bike_code/USER/**` are **GBK** (Chinese comments), CRLF. Read them with
`iconv -f GBK -t UTF-8`; `grep` needs `-a`; never use `Edit`/`Write` on them —
it silently re-encodes the whole file. See `../bike_code/CLAUDE.md`.

This Makefile has no `-finput-charset` flag (bike_code's does, with `=GBK`).
Keep new sources here UTF-8 or ASCII; do not paste GBK bytes into them.

## Hardware: what the .ioc configures, and what the same pin did in v1

72 MHz from an 8 MHz HSE ×9. LSE on PC14/PC15 is **new** in v2 — it costs the
handset the PC14 button. `stm32f1xx_hal_msp.c` calls
`__HAL_AFIO_REMAP_SWJ_NOJTAG()`, freeing PB3/PA15 for buttons the same way v1's
`JTAG_Set(JTAG_SWD_DISABLE)` did; SWD stays on PA13/PA14.

**Buttons** — ten `GPIO_MODE_INPUT` + `GPIO_PULLUP` pins, no EXTI (so they must be
polled/debounced from the timer tick). v1's `../Remote/Hardware/key.h` names the
same pins:

| Pin | v1 name | v1 position |
|---|---|---|
| PB9 | KEY0 | 左上 |
| PA11 | KEY1 | 右上 |
| PB8 | KEY2 | 左左 |
| PB7 | KEY3 | 左右 |
| PA15 | KEY4 | 右左 |
| PB3 | KEY5 | 右右 |
| PB6 | KEY6 | 左下 |
| PA0 | KEY8 | 中右 |
| PA12 | KEY9 | 右下 |
| PC13 | — | *(v1 used PC13 as the beeper; v1's KEY7 中左 was PC14, now LSE)* |

Two pin roles to verify against the physical handset before writing key code:
PC13 (beeper in v1, a pulled-up input in the .ioc), and PA0 — v1 configured it
**pull-down** and tested it active-high (`if(KEY8 == 1)`), while the .ioc gives it
a pull-up like every other key.

**SPI1** — master, PA5/PA6/PA7, mode 0, soft NSS, prescaler 16 → 4.5 Mbit/s.
**Unused, and `MX_SPI1_Init()` is commented out**: the OLED on this board turned
out to be a 4-pin I2C module, not SPI, and it takes PA5 as SCL and PA7 as SDA.
See `USER/oled`. Only PA6 (MISO) is left free.

**SPI2** — master, PB13/PB14/PB15, mode 0, soft NSS, prescaler 2 → **18 Mbit/s**.
This is the nRF24L01: v1 bit-banged the same three pins, with a comment claiming
hardware SPI was unusable on this handset (`//#define NFR_SPI  //硬件SPI 遥控器c6t6不支持`);
the .ioc assumes otherwise. Two problems to resolve: 18 Mbit/s is above the
nRF24L01's 10 Mbit/s SPI ceiling, and v1's CSN=PB10 / IRQ=PB11 / CE=PB12 now
collide with USART3 — CSN and IRQ need new pins.

**TIM1** — update interrupt enabled, PSC 71 / ARR 999 → **1 ms**. v1 ran key
scanning and the control tick from TIM3 at 20 ms, and its `Key_Scan()` hard-codes
`Key_time += 20` per call for long-press detection, so that logic must be
re-scaled or divided down before reuse.

**USARTs** — USART1 PA9/PA10, USART2 PA2/PA3, USART3 PB10/PB11, all 115200 8N1,
RX interrupt enabled, TX on DMA1 channels 4 / 7 / 2 respectively. v1 had only
USART1 (115200, RXNE interrupt, DMA TX commented out), and **USART1 is the wire
to the bike**. USART2/USART3 are new and unassigned.

Not configured in the .ioc, but present in v1: LEDs on PB4/PB5, beeper on PC13,
ADC joysticks + battery sense (v1's `Button_Only` build already bypassed those).

## `USER/` — application code

The Makefile's `C_SOURCES`/`C_INCLUDES` were extended by hand to reach this
tree; CubeMX does not know it exists, so **re-merge those lines after every
regeneration** (same trap as `../bike_code`).

### `USER/oled` — SSD1306 128x64, soft I2C

The 江协科技 OLED library **V2.0 (2024.10.20)**, ported from the standard
peripheral library to HAL. Not the copy in `../Remote/Hardware` — that one is an
older version and never worked properly on this hardware.

Wiring: **SCL = PA5, SDA = PA7.**

Those are `SPI1_SCK` and `SPI1_MOSI` in the .ioc, so `OLED_GPIO_Init()` takes
both over as open-drain outputs and **`MX_SPI1_Init()` is commented out in
`main.c`** — that line lives outside the USER CODE markers, so a CubeMX
regeneration puts it back and the display dies.

Only four things differ from the upstream library, all in the `引脚配置`
section at the top of `OLED.c`:

- `#include "stm32f10x.h"` → `#include "main.h"`.
- `OLED_W_SCL()` / `OLED_W_SDA()` → `HAL_GPIO_WritePin`, plus `OLED_I2C_Delay()`.
- `OLED_GPIO_Init()` → HAL, open-drain, `HAL_Delay(100)` instead of the busy loop.
- `OLED_Probe()` — **new**. Upstream sends the ninth clock and ignores the ACK
  bit entirely, so a blank panel gives you nothing to go on. This one drives the
  address byte by hand and reads SDA back on the ninth clock: 1 = something
  answered at 0x78, 0 = bus, power or address is wrong.

  It first checks that SDA sits **high** with both lines released. That test is
  not decoration: while SDA was briefly mis-assigned to an unconnected pin, the
  floating input read low and was indistinguishable from a real ACK, so the
  probe reported success with no display attached. No pull-up, no bus — report
  the fault instead of a false pass.

Open-drain is not a style choice: the SSD1306 pulls SDA low itself during each
ACK, so a push-pull master would fight it. Nothing pulls the lines up on the
MCU side — an F1 has no usable internal pull-up while the pin is an output — so
the bus depends on the resistors fitted on the OLED module.

`OLED_I2C_NOPS` (8) pads each half bit period. With no padding at all, 72 MHz
through `HAL_GPIO_WritePin` clocks the bus near 2 MHz, past what an SSD1306 is
specified for. Raise it if the panel shows noise, lower it if refresh is slow.

Encoding note: these four files were **converted GBK → UTF-8** on the way in, so
they match the rest of Remote_2 and need no `-finput-charset` flag. Do not copy
newer upstream versions in without converting them first.

### `USER/key`, `USER/led` — 按键与 LED

按键 8 个，全部上拉输入、按下为低；GPIO 由 `MX_GPIO_Init()` 配好，`key_init()`
只清软件状态，不重复配置（避免和 .ioc 出现两份互相矛盾的设置）。

```
左边  上 PB9   左 PB8   右 PB7   下 PB6
右边  上 PA11  左 PA15  右 PB3   下 PA12
LED   LED0 = PB4      LED1 = PB5
```

`PB3` / `PA15` / `PB4` 都要靠 `HAL_MspInit()` 里的 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`
从 JTAG 手里放出来。`led.h` 的 `LED_ACTIVE_HIGH` 默认 0（低电平点亮）。

扫描挂在 **TIM1 更新中断（1 ms）**上，`main.c` 的 `HAL_TIM_PeriodElapsedCallback`
里调 `key_tick()` + `led_tick()`。600 ms 长按，之后 100 ms 连发。

消抖**按下和松开分开**：`KEY_DEBOUNCE_PRESS_MS` 15、`KEY_DEBOUNCE_RELEASE_MS` 25。

松开方向**故意不放长**。这块板子的触点其实不抖（实测 10 次点按正好 10 个 DOWN +
10 个 UP），而认松开越慢，`key_is_down()` 就多撑越久，等于把"按住时长"凭空拉长，
反而更容易触发 `remote.c` 的连发：松开消抖 20 ms 时按 10 下发 13 帧，改成 50 ms 后
变成 20 帧——每次点按都被撑过了连发门槛。真正的修法是给连发加首次延迟，不是加消抖。

屏上 `N`（按下次数）和 `B`（按键帧数）一起看：点按 10 下应该 N=10、B=10。
`B > N` 是点按被当成了按住（调大 `REMOTE_REPEAT_DELAY_MS`），`N` 大于实际按的次数
才是触点回弹（调大 `KEY_DEBOUNCE_RELEASE_MS`）。

事件走**环形队列**，不是 v1 那种 `Key_Flag[]` 标志位数组：整屏刷 OLED 要十几
毫秒，标志位在这个节奏下必然把两次按下并成一次，而且置位在中断、清位在主循环，
天生有竞态。

### `USER/bt`, `USER/remote` — 蓝牙遥控链路

`REMOTE_PROTOCOL.md`（仓库根目录也有一份，内容相同）是**权威规格**，从整车固件
生成，改动以那份为准。

- `bt.c` — USART1 (PA9/PA10) 115200 8N1 的纯字节管道。发送走环形缓冲 +
  DMA1_Channel4，`bt_write()` 不阻塞。缓冲不够时**整帧丢弃**而不是写半帧：
  半帧会让整车多毁一帧（虽然下一个 `'('` 会重新同步）。手柄**只发不收**，
  没有接收通路。
- `remote.c` — 协议层。帧是 ASCII `"(id,)"`，**逗号不能省**：整车解析器在收集
  id 的状态里不把 `')'` 当结束符，`(U)` 会让 id 变成 `"U)"` 并卡住状态机，那一帧
  被静默丢掉。

按键 → 协议 id 的映射就是 `remote.c` 顶上那一张表，要改只改那里：

| 按键 | id | 含义 |
|---|:--:|---|
| 左 上 PB9 | `U` | 加速 / 菜单上一项 |
| 左 左 PB8 | `L` | 左转 / 值减一步 |
| 左 右 PB7 | `R` | 右转 / 值加一步 |
| 左 下 PB6 | `D` | 减速 / 菜单下一项 |
| 右 上 PA11 | `M` | 模式切换 |
| 右 下 PA12 | `K` | 停车回中 / 执行菜单项 |
| 右 左 PA15 / 右 右 PB3 | — | 空闲 |

左边一整块是方向键，协议要的 U/D/L/R 四个方向都在左手；右手只留 `M`（模式）和
`K`（OK/停车）两个动作键。`K` 对应 v1 遥控器的"中值"键，语义一致。

时序：按下瞬间发一帧，**按住超过 400 ms** 后每 100 ms 重发（整车侧一帧 = 一步，
没有长按概念；首次延迟是为了让点按只发一帧），
**心跳 `(H,)` 每 150 ms 无条件发**。整车 500 ms 收不到任何帧就切动力（平衡环
不动）。协议层不用 `key.c` 的 `KEY_EV_REPEAT`：那个要按住 600 ms 才开始连发，
而协议要求 100 ms。

**绝对不能发 id `'0'`~`'7'`** —— 那是 VOFA+ 上位机调参用的，会绕过模式机，而且
不刷新心跳定时器。

屏上的测试页把每个键**实际会发的 id** 画在方框里（未分配画 `-`），所以对着屏
就能核对映射，不用回去翻代码。底下两行是 `TX`(总帧数) / `ID`(最后一帧) /
`HB`(心跳数) / `P`(发送队列积压) / `D`(丢帧数)。`P` 长期不为 0 说明 DMA 没在
排空——通常是波特率或 DMA 通道配错了。

### Verifying the link without a logic analyser

发送环形缓冲是普通全局变量，调试器可以直接读回真正排进去的字节：

```bash
arm-none-eabi-nm build/Remote_2.elf | grep -E "tx_ring|stat_frames|stat_heartbeats"
openocd -f openocd-horco.cfg -f target/stm32f1x.cfg \
  -c init -c halt -c "dump_image /tmp/ring.bin 0x20000728 64" -c resume -c exit
```

上一次实测：字节是 `28 48 2C 29` = `(H,)`，和 REMOTE_PROTOCOL.md 7.2 节逐字节
一致；心跳 6.0 Hz（167 ms，协议要求 100~200 ms）；`stat_tx_bytes` 正好等于
帧数 × 4；`tx_busy = 0`、`tx_head == tx_tail`，说明 DMA 完成回调这条路是通的。

### Verifying the display without looking at it

The frame buffer is a plain global, so the debugger can read back exactly what
the panel was told to show — much better evidence than squinting at a photo:

```bash
arm-none-eabi-nm build/Remote_2.elf | grep OLED_DisplayBuf   # -> 0x20000278
openocd -f openocd-horco.cfg -f target/stm32f1x.cfg \
  -c init -c halt -c "dump_image /tmp/fb.bin 0x20000278 1024" -c resume -c exit
```

then render the 1024 bytes as text (page-major, bit 0 = top pixel of the page):

```python
fb = open('/tmp/fb.bin','rb').read()
for page in range(8):
    for bit in range(8):
        print(''.join('#' if (fb[page*128+x] >> bit) & 1 else '.' for x in range(128)))
```

`main.c` currently holds a staged bring-up test: `OLED_Probe()` into `oled_ack`,
a full-screen fill, then a text page and a `TICK` counter with a marching block
so a frozen MCU is distinguishable from a working one. As of the last run on
hardware: `oled_ack = 1`, the counter advances, and the rendered buffer matches
the intended layout.

## The protocol this firmware has to speak

**`REMOTE_PROTOCOL.md`, in this directory, is the authoritative spec** — 756
lines generated from the bike's firmware, quoting the actual parser and
dispatcher inline. An identical copy sits at the repo root. Read it rather than
any summary; a summary here would be a second source of truth and would drift.

The one-paragraph version, only so you know what you are looking at: the handset
is **transmit-only**, sending ASCII `(id,)` frames at 115200 8N1 over Bluetooth
SPP into the bike's UART7. Ids are `M` `U` `D` `L` `R` `K` (physical buttons,
never actions — the bike decides what a press means in its current mode, so the
handset holds no state and cannot desync), plus a mandatory `(H,)` heartbeat.
The comma is mandatory. Digit ids `'0'`–`'7'` belong to the VOFA+ PC tuning tool
and must never be sent. No ACK, no retry, no checksum — repetition is the
reliability mechanism.

Note that `../bike_code/USER/ui/*` and `USER/vofa/*` have uncommitted local
changes. If `REMOTE_PROTOCOL.md` and the bike's source ever disagree, the source
wins — the doc says so itself.
