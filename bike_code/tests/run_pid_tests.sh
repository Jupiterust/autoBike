#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build/test_pid
# Convert build copies only; production PID sources remain GBK/CRLF.
iconv -f GBK -t UTF-8 USER/PID/PID.c > build/test_pid/PID.c
iconv -f GBK -t UTF-8 USER/PID/PID.h > build/test_pid/PID.h
# calculateGain() already uses abs(float); suppress that unrelated Clang
# diagnostic only for the production translation unit under test.
cc -std=c99 -Wall -Wextra -Werror -Wno-absolute-value \
    -fsanitize=address,undefined \
    -Ibuild/test_pid -Itests/stubs -IUSER/upper -IUSER/vofa -IUSER/telemetry \
    -include USER/telemetry/telemetry.h \
    -c build/test_pid/PID.c -o build/test_pid/PID.o
cc -std=c99 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Ibuild/test_pid -Itests/stubs -IUSER/upper -IUSER/vofa -IUSER/telemetry \
    tests/test_pid.c build/test_pid/PID.o -lm -o build/test_pid/test_pid
build/test_pid/test_pid
