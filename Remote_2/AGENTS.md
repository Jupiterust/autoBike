# Repository Guidelines

## Project Structure & Module Organization

`Remote_2` is handheld remote firmware for a self-balancing bike, targeting the STM32F103C8T6 with STM32 HAL and a Cortex-M3 toolchain.

- `Core/Src/` and `Core/Inc/`: CubeMX-generated initialization, interrupts, and application entry point.
- `USER/`: application modules: `key` scanning, `led` control, `oled` display, `bt` UART/DMA transport, and `remote` protocol handling. Display fonts and graphics live in `USER/oled/OLED_Data.c`.
- `Drivers/`: bundled STM32 HAL and CMSIS dependencies; avoid unrelated vendor edits.
- `Remote_2.ioc`: CubeMX configuration; the root linker script and startup assembly define memory layout and boot code.
- `REMOTE_PROTOCOL.md`: command framing and bike-side behavior. `build/` holds generated artifacts.

## Build, Test, and Development Commands

Install GNU Arm Embedded tools (`arm-none-eabi-*`); flashing also requires OpenOCD and a connected CMSIS-DAP probe.

- `make`: build `build/Remote_2.elf`, `.hex`, and `.bin`, and report image size.
- `make GCC_PATH=/path/to/bin`: build using an explicit toolchain directory.
- `make clean`: remove build outputs.
- `make flash`: build, program, verify, and reset hardware using `openocd-horco.cfg`; retries probe connection failures up to eight times.

## Coding Style & Naming Conventions

Use four-space indentation and braces on separate lines in `USER/` code. Follow existing module-prefixed snake_case names (`remote_task`, `key_poll`), uppercase constants (`KEY_COUNT`), and `_t` type suffixes. Preserve local style in generated files. Use UTF-8/ASCII and preserve existing line endings; editor settings specify CRLF. No formatter or linter is configured; compilation enables `-Wall`.

## Testing Guidelines

No application test framework or coverage threshold is configured. Rebuild and resolve new warnings. Validate changes on hardware: display updates, button mapping, ten taps producing ten button frames, held-key repeats, heartbeat transmission, and DMA queue draining. Include observed results when changing timing or transport behavior.

## Commit & Pull Request Guidelines

The parent repository's history uses short, descriptive subjects, often in Chinese, naming the subsystem and behavior changed; no fixed Conventional Commits pattern appears. Keep commits focused. PR descriptions should explain behavior, affected modules, build results, hardware validation, and related issues. Include display photos when changing the UI.

## CubeMX & Protocol Constraints

Keep generated-file edits inside `USER CODE` blocks. After regeneration, restore `USER/` Makefile entries and the flash target, and ensure `MX_SPI1_Init()` remains disabled because OLED software I2C uses PA5/PA7. Preserve mandatory commas in `(id,)` frames and heartbeat timing; never transmit tuning IDs `0`–`7`.
