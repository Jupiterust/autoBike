#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build
# Mechanical encoding conversion for the host compiler, not a source edit.
iconv -f GBK -t UTF-8 USER/vofa/vofa.c > build/test_vofa.c
iconv -f GBK -t UTF-8 USER/ch100/CH100.c > build/test_ch100.c
cc -std=c99 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs -IUSER/upper -IUSER/vofa \
    USER/upper/upper.c build/test_vofa.c build/test_ch100.c tests/test_upper.c \
    -lm -o build/test_upper
build/test_upper
