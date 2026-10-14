# Repository Guidelines

## Project Structure & Module Organization

This directory contains STM32F427 smart-bike firmware; the Git root is one level above.

- `Core/Inc` and `Core/Src`: CubeMX peripheral setup and entry point.
- `USER/`: application modules, including `Balane`, `PID`, `Servo`, `Odrive`, `upper`, `ui`, and display drivers. Font assets live under `oled` and `tft`.
- `Drivers/`: vendor HAL/CMSIS code.
- `tests/`: host C tests and hardware stubs.
- `build/`: generated firmware and test binaries; `logs/`: tuning records.

## Build, Test, and Development Commands

Run from `bike_code/`. Firmware builds require Make and `arm-none-eabi-gcc`; host tests require `cc` and `iconv`.

- `make -j4`: build `build/Fly_Dreams.{elf,hex,bin}`. Set `GCC_PATH=/path/to/bin` if needed.
- `make clean`: remove build outputs.
- `sh tests/run_upper_tests.sh`: test host protocol and remote dispatch.
- `sh tests/run_ui_tests.sh`: test UI configurations and TFT drawing/cache behavior.
- `make flash`: program, verify, and reset connected hardware using OpenOCD/CMSIS-DAP HID.

GCC/Make is authoritative; Keil/EIDE projects are historical. Register new sources in `C_SOURCES` and directories in `C_INCLUDES`; preserve these additions when regenerating CubeMX files.

## Coding Style & Naming Conventions

Match surrounding C style; use four spaces for new code, module-prefixed functions such as `upper_task`, uppercase macros, and paired `.c`/`.h` files. Preserve existing names, including `Balane`.

Many legacy sources use GBK and CRLF. Inspect encoding before editing, preserve bytes/line endings, and retain `-finput-charset=GBK`. Avoid bulk reformatting. `bikecode_eide/.clang-format` specifies four spaces and no tabs; no repository-wide formatting gate is configured.

## Testing Guidelines

Tests use C assertions, HAL stubs, AddressSanitizer, and UndefinedBehaviorSanitizer. Add regressions to `tests/test_<module>.c`, using `test_<behavior>` functions where practical. Run both scripts for protocol/UI changes and rebuild firmware. No numerical coverage threshold is configured. Report host results separately from on-board validation.

## Commit & Pull Request Guidelines

Recent commits use short, descriptive Chinese subjects; Conventional Commits prefixes are not required. Keep changes focused and stage explicit paths, preserving unrelated work.

PRs should describe behavior changes, affected modules, linked issues when applicable, and validation commands/results. Include display photos for UI changes and update communication documentation for protocol changes.

## Architecture Precautions

`Sys_All_Init()` owns the foreground loop; UART8 IMU reception triggers `balance()`. Keep display work outside interrupts and preserve the disabled TIM3 control trigger. Check current source and tests against older notes in `CLAUDE.md` and tuning documents.
