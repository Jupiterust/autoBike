# Repository Guidelines

## Project Structure & Module Organization

This directory is the ODrive motor interface for STM32F427 smart-bike firmware. The firmware root is `../..` (`bike_code/`); the Git root is its parent, `bike/`.

- `Odrive.c` and `Odrive.h`: motor commands, feedback, and interface declarations.
- `Core/Inc`, `Core/Src`: CubeMX peripheral configuration and entry point.
- `USER/`: application modules, including `Balane`, `PID`, `Servo`, `upper`, `ui`, and display drivers; font assets are under `oled` and `tft`.
- `Drivers/`: HAL/CMSIS dependencies; `tests/`: host tests and stubs.
- `build/`: generated binaries; `logs/`: tuning records.

## Build, Test, and Development Commands

Run commands from `bike_code/` (`cd ../..` from this directory).

- `make -j4`: build `build/Fly_Dreams.{elf,hex,bin}` using ARM GCC. Supply `GCC_PATH=/path/to/bin` when necessary.
- `make clean`: remove generated build outputs.
- `sh tests/run_upper_tests.sh`: test host protocol and remote dispatch.
- `sh tests/run_ui_tests.sh`: test UI variants and display drawing/cache behavior.
- `make flash`: program, verify, and reset connected hardware through OpenOCD/CMSIS-DAP HID.

GCC/Make is authoritative. Register added sources and include directories in `C_SOURCES` and `C_INCLUDES`; preserve them during CubeMX regeneration.

## Coding Style & Naming Conventions

Match surrounding C code. Use four spaces for new code, module-prefixed functions such as `odrive_init`, uppercase macros, and paired `.c`/`.h` files. Preserve established names, including `Balane`.

Inspect encoding before editing: legacy sources, including ODrive files, use CRLF and GBK comments. Preserve encoding and line endings; retain `-finput-charset=GBK`. Avoid bulk reformatting. `bikecode_eide/.clang-format` specifies four spaces and no tabs.

## Testing Guidelines

Host tests use C assertions, HAL stubs, AddressSanitizer, and UndefinedBehaviorSanitizer; they require `cc` and `iconv`. Add regressions in `tests/test_<module>.c`, with `test_<behavior>` functions. Run both scripts for protocol/UI changes and rebuild firmware. No numerical coverage threshold is specified. Report host verification separately from on-board validation.

## Commit & Pull Request Guidelines

Recent commits use concise Chinese subjects describing behavior changes. Stage explicit paths and preserve unrelated work. PRs should identify affected modules, behavior changes, relevant issues, and validation results. Include display photos for UI changes and update communication documentation for protocol changes.

## Architecture Precautions

`Sys_All_Init()` owns the foreground loop; UART8 IMU reception triggers `balance()`. Keep display work outside interrupts and preserve the disabled TIM3 control trigger. Verify current source against older notes before changing control behavior.
