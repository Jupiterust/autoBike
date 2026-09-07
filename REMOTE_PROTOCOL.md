# Remote Control Protocol - Self-Balancing Bike

Everything a second MCU needs in order to act as the **handheld remote (handset)**
for this self-balancing bike, with the authoritative firmware source quoted inline.
This file is self-contained: you do **not** need the bike's repository to
implement the handset.

- Bike MCU: STM32F427IIHx, project name `Fly_Dreams`.
- Handset role: **transmit only.** It sends short ASCII frames over a Bluetooth
  serial link. It never has to receive or parse anything.
- Document generated from the firmware on 2026-09-08. Where this document and
  the quoted code disagree, the code wins.

---

## 1. Physical layer

| Property | Value |
|---|---|
| Bike-side peripheral | `UART7` |
| Pins | `PE8` = UART7_TX, `PE7` = UART7_RX (`GPIO_AF8_UART7`) |
| **Baud rate** | **115200** |
| Frame | **8 data bits, no parity, 1 stop bit (8N1)** |
| Flow control | **none** |
| Oversampling | 16 |
| Link | Bluetooth SPP serial module on each end |

Bike-side peripheral init, verbatim from `Core/Src/usart.c`:

```c
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
```

**The handset must run 115200 8N1 and its Bluetooth module must be paired with
the module attached to the bike's UART7.** Nothing else about the radio matters
to this protocol - as far as the firmware is concerned it is a transparent byte
pipe.

### 1.1 The telemetry stream is a DIFFERENT link - ignore it

The bike also emits a high-rate binary telemetry stream, but that is a
**separate, unrelated UART**:

| | Command link (this document) | Telemetry link (not this document) |
|---|---|---|
| Peripheral | `UART7` | `USART6` |
| Pins | PE8 / PE7 | PG14 / PG9 |
| Baud | 115200 | 460800 |
| Direction used | bike **receives** | bike **transmits** |
| Payload | ASCII `(id,value)` | binary frames, `0xAA55` + LEN + payload + CRC16 |

The handset has nothing to do with USART6. Do not try to receive or decode it.

### 1.2 The handset is transmit-only

The handset only needs a UART **TX** path. It does not have to open, arm or
service an RX path.

The bike's UART7 is configured `UART_MODE_TX_RX`, and the firmware contains a
`uart_printf()` helper that would transmit on UART7 by DMA - but **that function
is currently never called anywhere in the firmware**, so in practice the bike
sends nothing back on this link. If your Bluetooth module surfaces incoming
bytes anyway (module status text, echo, future firmware), just discard them.
No reply, ACK or NAK exists in this protocol.

---

## 2. Frame format

An ASCII frame:

```
( id , value )
```

- `(` - start of frame
- `id` - the command field. **Only its first byte is used** by the dispatcher.
- `,` - separator. **Mandatory** (see 2.3).
- `value` - the payload field. May be empty.
- `)` - end of frame; the frame is handed to the application.

No line terminator is required. Bytes that arrive while the parser is idle are
silently discarded, so you *may* send `\r\n` between frames if that makes
terminal testing easier - it costs nothing and is ignored.

### 2.1 The parser, verbatim

This is the authoritative parser, `vofa_get()` in `USER/vofa/vofa.c`. It is
called once per received byte from the UART7 RX interrupt.

```c
/* Frame parser for "(id,value)", called once per byte from the UART7 RX ISR.
 *
 * Two defects fixed against the original:
 *  1. neither index was bounds checked, so a stream without a ')' - noise on
 *     the line, or a handset resetting mid-frame - ran straight off the end of
 *     the 256 byte buffers and corrupted whatever globals followed them;
 *  2. the indices were only reset in vofa_apply(), after the frame had been
 *     consumed.  A frame arriving while vofa_flag was still set appended to
 *     the previous one instead of replacing it, and a dropped ')' left the
 *     state machine stuck forever.  '(' now always starts a clean frame,
 *     wherever it turns up, and newest-frame-wins.
 */
void vofa_get(uint8_t byte)
{
	static uint8_t vofa_state;

	if (byte == '(')                    /* always restarts, in any state */
	{
		vofa_index[0] = 0;
		vofa_index[1] = 0;
		vofa_state = 1;
		return;
	}

	if (vofa_state == 1)
	{
		if (byte == ',')
			vofa_state = 2;
		else if (vofa_index[0] < VOFA_BUFF_SIZE - 1)
			vofa_buff[0][vofa_index[0]++] = byte;
	}
	else if (vofa_state == 2)
	{
		if (byte == ')')
		{
			vofa_buff[0][vofa_index[0]] = '\0';
			vofa_buff[1][vofa_index[1]] = '\0';
			vofa_state = 0;
			vofa_flag = 1;
		}
		else if (vofa_index[1] < VOFA_BUFF_SIZE - 1)
			vofa_buff[1][vofa_index[1]++] = byte;
	}
}
```

with, from `USER/vofa/vofa.h`:

```c
#define VOFA_BUFF_SIZE 		256
#define VOFA_COUNT	2

extern uint8_t vofa_buff[VOFA_COUNT][VOFA_BUFF_SIZE];
extern volatile uint8_t vofa_flag;
extern volatile uint8_t vofa_index[VOFA_COUNT];
```

### 2.2 State machine, step by step

The parser has three states. `vofa_state` is `static` and starts at 0.

| State | Meaning | On `'('` | On `','` | On `')'` | On any other byte |
|:--:|---|---|---|---|---|
| 0 | idle, between frames | reset both indices, go to **1** | discard | discard | discard |
| 1 | collecting `id` | reset both indices, stay at **1** | go to **2** | *appended to `id`* (see 2.3) | append to `id` if room |
| 2 | collecting `value` | reset both indices, go to **1** | append to `value` | NUL-terminate both fields, set `vofa_flag = 1`, go to **0** | append to `value` if room |

Three consequences worth designing around:

1. **`'('` always restarts a frame, in any state.** A frame interrupted by
   noise, or by the handset resetting mid-transmission, cannot wedge the parser:
   the next `'('` starts a clean frame. Your handset does not need to send any
   resynchronisation preamble.
2. **Newest frame wins.** The indices are reset when a frame *starts*, not when
   the application consumes it. If the bike's main loop has not yet handled the
   previous frame, a new frame simply replaces it; frames never concatenate.
3. **Fields are bounded.** Each field is capped at `VOFA_BUFF_SIZE - 1` = **255
   bytes**; excess bytes are dropped, not written past the buffer. Real frames
   are 4-12 bytes, so this is a safety net, not a budget.

### 2.3 The comma is mandatory - `(U)` does NOT work

In state 1 the byte `')'` has no special meaning: it falls through to the
"append to `id`" branch. So the byte sequence `(`,`U`,`)`:

- leaves `id` = `"U)"`,
- leaves the parser stuck in state 1,
- **never sets `vofa_flag`, so the frame is silently dropped.**

The parser recovers on the next `'('`, but that frame is lost. **Always send the
comma.** For commands that carry no payload, send `(U,)` - an empty value field
is perfectly legal.

### 2.4 Only the first byte of `id` is read

The dispatcher switches on `vofa_buff[0][0]`. `(U,)`, `(UP,)` and `(Uxyz,)` all
dispatch as `'U'`. Rely on this only as tolerance, never as a feature: **send
exactly one character in the `id` field.**

### 2.5 How bytes reach the parser

From `USER/uart/uart.c` - this is the single RX dispatch point for the whole
system:

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == (&huart8))
    {
        CH100_Rec(); 
				balance();			
    }
    else if (huart == (&huart2))
    {
				if(upper_Flag == 1)Upper(USART_RX_BUF1[0]);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)USART_RX_BUF1, USART_RX_LEN1);
    }
   else if (huart == (&huart7))
    {
        //HAL_UART_Transmit_DMA(&huart7,USART_RX_BUF1,USART_RX_LEN1);

			
		#ifdef Button_Only
        //vofa
		vofa_get(USART_RX_BUF2[0]);
        #else
		//原串口处理
		//data_analysis(USART_RX_BUF2[0]);
        #endif
		
		HAL_UART_Receive_IT(&huart7, (uint8_t *)USART_RX_BUF2, USART_RX_LEN2);
    }
```

Notes for the handset author:

- The `#ifdef Button_Only` branch is the live one: `Button_Only` is defined
  unconditionally in `USER/vofa/vofa.h`, so received bytes do reach
  `vofa_get()`. The `#else` arm (whose Chinese comment reads "legacy serial
  handling") is dead code. **If anyone ever removes that `#define`, this entire
  remote protocol goes dead** - the UART7 bytes would be discarded.
- `USART_RX_LEN2` is **1**: the bike arms a **one byte** interrupt receive, feeds
  that byte to `vofa_get()`, then re-arms. There is no RX DMA and no RX FIFO
  beyond the peripheral's own register.
- `UART7_IRQn` runs at NVIC preemption priority **2**, while `UART8_IRQn` (the
  IMU, which runs the 2.5 ms balance control loop inside its ISR) runs at
  priority **0** and therefore preempts it. A long control tick can in principle
  delay the UART7 ISR past one byte time (86.8 us at 115200) and lose a byte.
- This is why the protocol is designed to be **idempotent and repeated** rather
  than acknowledged: a corrupted frame costs one lost button step, and the
  `'('` restart rule means the following frame parses correctly. Keep sending.

### 2.6 Frames are ignored while the bike is in its boot sequence

The frame parser runs in an interrupt and is always active, but the *dispatcher*
(`vofa_apply()`) is only called from the bike's main loop, which does not start
until the boot sequence finishes. During these three blocking stretches remote
commands are parsed but never acted on:

1. waiting for the first press of the bike's on-board **KEY0** button
   (the bike's screen shows `Press KEY0 to start`),
2. a **31 second** "hold the bike upright" calibration countdown
   (screen shows `Calib hold upright` / `NN s left`),
3. a **2 second** re-stand pause after KEY0 is pressed again mid-run
   (screen shows `re-stand`).

The handset should simply keep sending its heartbeat throughout; nothing special
is required.

---

## 3. Command table

### 3.1 Authoritative copy, verbatim from `USER/ui/ui.h`

```c
/*
 * ui.h - OLED pages, the drive/menu mode machine and the remote command map.
 *
 * Everything here runs from the main loop in Init.c only.  Nothing in this
 * module may be called from balance() (UART8 RX ISR, 2.5 ms control loop):
 * a full OLED refresh is milliseconds of SPI and would wreck the balance.
 *
 * Remote protocol: the handset sends "(id,value)" frames on huart7, parsed by
 * the existing vofa_get(); vofa_apply() routes every non-digit id here.  The
 * handset sends PHYSICAL BUTTONS, never actions - this module decides what a
 * button means in the current mode, so the handset cannot get out of sync and
 * needs no state of its own.  Legacy VOFA+ digit keys '0'-'7' are untouched.
 *
 *   id   button   drive mode                       menu mode
 *   ---  -------  -------------------------------  ---------------------
 *   'M'  MODE     -> menu                          -> drive
 *   'U'  up       M1_Ctl += 0.1                    previous item
 *   'D'  down     M1_Ctl -= 0.1                    next item
 *   'L'  left     Servo_Ctl -= UI_SERVO_STEP       value -= step
 *   'R'  right    Servo_Ctl += UI_SERVO_STEP       value += step
 *   'K'  OK       stop: M1_Ctl=0, M1_Flag=0,       run the action item
 *                 steering ramps back to centre    (LOAD calm / LOAD wind)
 *   'H'  (none)   heartbeat, ~10 Hz from the handset, both modes
 *   'V'  (none)   set the selected menu item to value, clamped
 *   'I'  (none)   jump to menu item number value
 *
 * One frame is one step; press-and-hold is the handset repeating the same id.
 */
```

### 3.2 Summary

The handset transmits **physical buttons**, never actions. The bike decides what
a button means in the current mode. **The handset holds no mode state of its own
and therefore cannot get out of sync with the bike.**

| id | Handset button | Uses `value`? | Drive mode | Menu mode |
|:--:|---|:--:|---|---|
| `M` | MODE | no | switch to menu mode | switch to drive mode |
| `U` | up | no | `M1_Ctl += 0.1` (forward / faster) | previous menu item |
| `D` | down | no | `M1_Ctl -= 0.1` (reverse / slower) | next menu item |
| `L` | left | no | `Servo_Ctl -= 5` (steer left) | selected value `-= step` |
| `R` | right | no | `Servo_Ctl += 5` (steer right) | selected value `+= step` |
| `K` | OK | no | **stop**: `M1_Ctl = 0`, `M1_Flag = 0`, steering ramps back to centre | run the selected action item (`LOAD calm` / `LOAD wind`); no effect on ordinary value items |
| `H` | *(none)* | no | heartbeat only - see section 4 | same |
| `I` | *(none)* | **yes** | jump to menu item number | jump to menu item number |
| `V` | *(none)* | **yes** | set selected item to an absolute value | same - **but see 3.4: not in the current build** |

Limits enforced on the bike side (the handset does not need to track them):

- `M1_Ctl` is clamped to **[-3.0, +3.0]**, step 0.1.
- `Servo_Ctl` is clamped to **[-80, +80]** (`Servo_Delta` = 80), step 5.
- Menu values are clamped per item, see 3.5.
- `U` / `D` **refuse to spin the rear wheel while the balance loop is off**
  (`param.M0_Flag == 0`), which normally means the bike is lying down. The frame
  is still accepted and still counts as a heartbeat; it just does nothing.

### 3.3 Digit ids `'0'`-`'7'` are a different, coexisting command set

The same UART7 link is also used by a **VOFA+** PC tuning tool, which sends the
same `(id,value)` frame format with **single-digit ids**. Those are handled
before the remote dispatcher ever sees them:

| digit id | effect |
|:--:|---|
| `0` | `angular_zero -= 0.01` |
| `1` | toggle `upper_Flag`; when disabling it also sets `M1_Flag = 1`, `M1_Ctl = 1.5` |
| `2` | load the "no wind" PID gain set |
| `3` | load the "windy" PID gain set |
| `4` | start servo ramp toward `+Servo_Delta` |
| `5` | start servo ramp toward `-Servo_Delta` |
| `6` | `angular_zero += 0.01` |
| `7` | start servo ramp back to centre |

**The handset must not send digit ids.** They bypass the mode machine, are not
range-checked the way the menu is, and - importantly - **they do not refresh the
heartbeat timer** (see section 4), because they never reach `ui_command()`.

Routing is done by the `default:` arm of the dispatcher in `vofa_apply()`
(`USER/vofa/vofa.c`); everything that is not a digit id goes to the remote
handler:

```c
				default:
						/* Everything that is not a legacy VOFA+ digit key is a
						 * remote button; ui.h holds the id table. */
						ui_command((char)vofa_buff[0][0], (const char *)vofa_buff[1]);
						break;
```

### 3.4 `V` is NOT compiled into the current firmware

`'V'` is guarded by a build switch that is currently **off**:

```c
/* USER/ui/ui.h */
#define UI_ENABLE_SET_CMD 0
```

With this at 0 the `case 'V':` arm does not exist. A `(V,...)` frame reaches the
dispatcher's `default:` arm, refreshes the heartbeat timer, and is otherwise
**silently ignored**.

It was switched off deliberately: `'V'` is the only user of `atof()`, which
drags in newlib's `strtod`/`dtoa` and measured **+5604 bytes of flash**, and a
6-button handset cannot enter a float anyway. If you want absolute assignment
from a serial terminal, set the switch to 1 and rebuild the bike firmware.
`'I'` is unaffected and always available.

### 3.5 Menu items (for `I` and for context)

`I` takes a **0-based** index into this table. Valid range **0 to 12**; out of
range values are ignored. Verbatim from `USER/ui/ui.c`:

```c
static const menu_item_t MENU[] =
{
    { "av_kp", &param.angular_v_kp,          0.0f,  5.0f,   0.05f,   MENU_ACT_NONE },
    { "av_ki", &param.angular_v_ki,          0.0f,  1.0f,   0.005f,  MENU_ACT_NONE },
    { "av_kd", &param.angular_v_kd,          0.0f,  5.0f,   0.05f,   MENU_ACT_NONE },
    { "an_kp", &param.angular_kp,          -20.0f,  0.0f,   0.1f,    MENU_ACT_NONE },
    { "an_ki", &param.angular_ki,           -2.0f,  0.0f,   0.01f,   MENU_ACT_NONE },
    { "an_kd", &param.angular_kd,           -5.0f,  0.0f,   0.05f,   MENU_ACT_NONE },
    { "fw_kp", &param.fly_wheel_speed_kp,   -1.0f,  0.0f,   0.005f,  MENU_ACT_NONE },
    { "fw_ki", &param.fly_wheel_speed_ki,   -0.5f,  0.0f,   0.002f,  MENU_ACT_NONE },
    { "fw_kd", &param.fly_wheel_speed_kd,   -0.5f,  0.5f,   0.002f,  MENU_ACT_NONE },
    { "zero",  &param.angular_zero,         -5.0f,  5.0f,   0.01f,   MENU_ACT_NONE },
    { "sgain", &Servo_Gain_K,                0.0f,  0.01f,  0.0001f, MENU_ACT_NONE },
    { "LOADcalm", NULL, 0.0f, 0.0f, 0.0f, MENU_ACT_LOAD_CALM },
    { "LOADwind", NULL, 0.0f, 0.0f, 0.0f, MENU_ACT_LOAD_WIND },
};
```

| index | name | min | max | step |
|:--:|---|--:|--:|--:|
| 0 | `av_kp` | 0.0 | 5.0 | 0.05 |
| 1 | `av_ki` | 0.0 | 1.0 | 0.005 |
| 2 | `av_kd` | 0.0 | 5.0 | 0.05 |
| 3 | `an_kp` | -20.0 | 0.0 | 0.1 |
| 4 | `an_ki` | -2.0 | 0.0 | 0.01 |
| 5 | `an_kd` | -5.0 | 0.0 | 0.05 |
| 6 | `fw_kp` | -1.0 | 0.0 | 0.005 |
| 7 | `fw_ki` | -0.5 | 0.0 | 0.002 |
| 8 | `fw_kd` | -0.5 | 0.5 | 0.002 |
| 9 | `zero` | -5.0 | 5.0 | 0.01 |
| 10 | `sgain` | 0.0 | 0.01 | 0.0001 |
| 11 | `LOAD calm` | *action item - press OK* | | |
| 12 | `LOAD wind` | *action item - press OK* | | |

Items 3 to 7 have a maximum of exactly `0.0` on purpose: those gains must stay
negative in the present control law, and letting one cross zero would turn its
loop into positive feedback.

### 3.6 The dispatcher, verbatim

`ui_command()` in `USER/ui/ui.c`:

```c
void ui_command(char id, const char *value)
{
#if UI_ENABLE_SET_CMD
    const menu_item_t *m;
#endif
    int n;

    /* Any frame counts as a sign of life, including ones this build ignores. */
    rc_last_ms = HAL_GetTick();
    rc_seen    = 1;
    rc_link_ok = 1;

    switch (id)
    {
    case 'H':                   /* heartbeat: the timestamp above is the point */
        return;                 /* no redraw - it would defeat the 10 Hz cap   */

    case 'M':
        ui_mode = (ui_mode == UI_MODE_DRIVE) ? UI_MODE_MENU : UI_MODE_DRIVE;
        ui_rows_invalidate();   /* the whole page changed, not just its values */
        break;

    case 'U':
        if (ui_mode == UI_MODE_DRIVE)
            ui_drive_speed(+UI_M1_STEP);
        else if (menu_sel > 0)
            menu_sel--;
        break;

    case 'D':
        if (ui_mode == UI_MODE_DRIVE)
            ui_drive_speed(-UI_M1_STEP);
        else if ((uint8_t)(menu_sel + 1) < MENU_N)
            menu_sel++;
        break;

    case 'L':
        if (ui_mode == UI_MODE_DRIVE)
            ui_drive_steer(-UI_SERVO_STEP);
        else
            ui_menu_nudge(-1);
        break;

    case 'R':
        if (ui_mode == UI_MODE_DRIVE)
            ui_drive_steer(+UI_SERVO_STEP);
        else
            ui_menu_nudge(+1);
        break;

    case 'K':
        if (ui_mode == UI_MODE_DRIVE)
            ui_drive_stop();
        else
            ui_menu_action();
        break;

#if UI_ENABLE_SET_CMD
    case 'V':                   /* absolute set of the selected item */
        m = &MENU[menu_sel];
        if (ui_mode == UI_MODE_MENU && m->var && value && value[0])
            *m->var = clampf((float)atof(value), m->min, m->max);
        break;
#endif

    case 'I':                   /* jump to menu item */
        if (value && value[0])
        {
            n = atoi(value);
            if (n >= 0 && n < (int)MENU_N)
                menu_sel = (uint8_t)n;
        }
        break;

    default:
        return;                 /* unknown id: no state change, no redraw */
    }

    ui_request_redraw();
}
```

Two details that matter to the handset:

- The heartbeat timestamp is refreshed at the **top** of the function, before
  the `switch`. **Any** frame with a non-digit id keeps the link alive -
  including ids this build does not implement, which fall through to `default:`.
- `'H'` returns early, deliberately **without** requesting a screen redraw. A
  10 Hz heartbeat would otherwise force the OLED to repaint 10 times a second
  and defeat its refresh budget.

---

## 4. Timing contract

### 4.1 Heartbeat - REQUIRED

**The handset must transmit `(H,)` every 100-200 ms, continuously, in every
mode, whether or not any button is pressed.**

### 4.2 Failsafe on link loss

If the bike receives **no frame of any kind for 500 ms**, it cuts the drive:

```c
static void ui_rc_check(void)
{
    if (!rc_seen || !rc_link_ok)
        return;
    if ((uint32_t)(HAL_GetTick() - rc_last_ms) < UI_RC_TIMEOUT_MS)
        return;

    rc_link_ok    = 0;
    M1_Ctl        = 0.0f;
    param.M1_Flag = 0;
    servo_con     = 1;
    ui_request_redraw();
}
```

Precisely:

- `M1_Ctl = 0`, `param.M1_Flag = 0` - the rear wheel stops.
- `servo_con = 1` - the steering ramps smoothly back to centre.
- **`param.M0_Flag` is NOT touched - the balance loop keeps running.**
  Dropping balance on a radio glitch would put the bike on the ground, which is
  worse than anything a lost link can cause. Balance stays owned by the bike's
  on-board KEY0 button and by the control loop's own 5-degree tilt cutout.
- The failsafe fires **once** per link loss (`rc_link_ok` latches), and the next
  frame received re-arms the link.
- It never fires before the first frame is received, so a bike operated with no
  handset at all is unaffected.

The bike's screen shows the link state on the drive page: `--` = no frame ever
received, `RC` = link up, `??` = link lost.

### 4.3 Button repeat - there is no long-press

The bike implements **one frame = one step**. There is no press-and-hold
detection, no auto-repeat and no key-down/key-up distinction on the bike side.

**Press-and-hold is implemented entirely in the handset: while a button is held,
retransmit that button's id at about 10 Hz.** A single press must send exactly
one frame.

Recommended handset timing:

| Event | Behaviour |
|---|---|
| Button pressed (edge) | send its frame immediately, once |
| Button held | keep resending the same id every ~100 ms |
| Button released | stop |
| Always, in parallel | send `(H,)` every 100-200 ms |

At 10 Hz repeat this gives, on the bike: `M1_Ctl` +-1.0 per second, and
`Servo_Ctl` centre to full lock in about 1.6 s.

Debounce in the handset. A bouncing contact that emits several frames per press
will step the value several times.

### 4.4 Bandwidth

A frame is 4-6 bytes; at 115200 8N1 one byte takes 86.8 us, so a frame takes
under 0.6 ms on the wire. Worst case (heartbeat at 10 Hz plus one button held at
10 Hz) is about 80 bytes/s against a link capacity of 11520 bytes/s -
under 1%. There is no need to pace frames beyond the rates above, and no reason
to batch them.

---

## 5. `value` field format

| id | `value` content | Parsed with | Empty allowed? |
|:--:|---|---|---|
| `M` `U` `D` `L` `R` `K` `H` | ignored entirely | - | yes - send empty |
| `I` | decimal integer, 0-12 | `atoi()` | **no** - an empty value is ignored |
| `V` | floating point, e.g. `-7.35` | `atof()` | **no** - and not compiled in, see 3.4 |

**Canonical form for a command with no payload: `(id,)`.**

`(id,0)` is also accepted and parses to `value = "0"`, which every value-less
command ignores. Use whichever is easier; `(id,)` is one byte shorter. What is
**not** acceptable is omitting the comma - see 2.3.

`value` is a NUL-terminated C string when the dispatcher sees it. Leading and
trailing whitespace is **not** stripped; `atoi`/`atof` skip leading whitespace
themselves but do not tolerate a leading `+` followed by space, so send a clean
token with no padding.

---

## 6. What the handset firmware has to do

Minimum viable handset:

1. **UART TX at 115200 8N1** into the Bluetooth serial module. RX not required.
2. **Pair** the handset's Bluetooth module with the bike's UART7 module, in
   transparent SPP mode.
3. **Read 6 buttons**, debounced: `UP`, `DOWN`, `LEFT`, `RIGHT`, `MODE`, `OK`.
4. **On a button press edge**, transmit that button's frame once:

   | Button | Frame |
   |---|---|
   | MODE | `(M,)` |
   | UP | `(U,)` |
   | DOWN | `(D,)` |
   | LEFT | `(L,)` |
   | RIGHT | `(R,)` |
   | OK | `(K,)` |

5. **While a button stays held**, retransmit its frame every ~100 ms.
6. **Independently and always**, transmit `(H,)` every 100-200 ms. Never stop,
   including while a button is held, and including at power-on before the first
   press.
7. **Never send ids `'0'`-`'7'`.**
8. Do not implement acknowledgements, retries, sequence numbers, or checksums:
   the protocol has none. Repetition is the reliability mechanism.
9. Optional: `(I,<n>)` to jump directly to menu item `n` (0-12).

Suggested handset transmit loop, in pseudocode:

```c
/* called every 10 ms */
static uint32_t last_hb_ms;
static uint32_t last_rep_ms;

uint32_t now = millis();

/* 1. edge-triggered single step */
for (each button b)
    if (pressed_edge(b))
        uart_send(frame_for(b));            /* e.g. "(U,)" */

/* 2. ~10 Hz repeat while held */
if (now - last_rep_ms >= 100) {
    last_rep_ms = now;
    for (each button b)
        if (is_held(b) && !pressed_edge(b))
            uart_send(frame_for(b));
}

/* 3. heartbeat, unconditional */
if (now - last_hb_ms >= 150) {
    last_hb_ms = now;
    uart_send("(H,)");
}
```

---

## 7. Byte-level examples

All bytes are plain 7-bit ASCII. No terminator is sent or expected.

### 7.1 "Up" button - accelerate / previous menu item

```
ASCII : (  U  ,  )
hex   : 28 55 2C 29
```

4 bytes. In drive mode this adds 0.1 to `M1_Ctl` (clamped at +3.0) and enables
the rear wheel, provided the balance loop is on. In menu mode it moves the
selection up one item.

### 7.2 Heartbeat

```
ASCII : (  H  ,  )
hex   : 28 48 2C 29
```

4 bytes, sent every 100-200 ms forever. Refreshes the 500 ms failsafe timer.
Causes no screen redraw and changes no state.

### 7.3 Jump to menu item 9 (`zero` = `angular_zero`)

```
ASCII : (  I  ,  9  )
hex   : 28 49 2C 39 29
```

5 bytes. Selects menu index 9. A following `(R,)` then adds one step (0.01) to
`angular_zero`, and `(L,)` subtracts one.

### 7.4 Absolute assignment - only if `UI_ENABLE_SET_CMD` is rebuilt as 1

```
ASCII : (  V  ,  -  7  .  3  5  )
hex   : 28 56 2C 2D 37 2E 33 35 29
```

9 bytes. In menu mode, sets the currently selected item to -7.35, clamped to
that item's min/max. **In the firmware as shipped today this frame is accepted
as a heartbeat and otherwise ignored** - see section 3.4.

### 7.5 A realistic one-second slice

Holding UP in drive mode while the heartbeat runs, roughly:

```
(H,)(U,)(U,)(H,)(U,)(U,)(U,)(H,)(U,)(U,)(H,)(U,)(U,)(U,)(H,)...
```

Order does not matter, interleaving is fine, and no delimiter between frames is
needed. If you prefer readable logs you may insert `\r\n` between frames; the
bike discards bytes that arrive between frames.

---

## 8. Quick reference card

```
Link      UART7, 115200 8N1, no flow control, over Bluetooth SPP
Direction handset -> bike only; handset needs no RX
Frame     "(" id "," value ")"      comma MANDATORY, no terminator
Ids       M=mode U=up D=down L=left R=right K=ok H=heartbeat
          I=<0..12> jump to menu item      V=<float> (disabled in build)
Never     send ids '0'-'7' (VOFA+ PC tuning uses those)
Heartbeat "(H,)" every 100-200 ms, always
Repeat    one frame = one step; ~10 Hz resend while a button is held
Failsafe  no frame for 500 ms -> M1_Ctl=0, M1_Flag=0, steering re-centres
          (the balance loop is deliberately left running)
```
