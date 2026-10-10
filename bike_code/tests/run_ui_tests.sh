#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build
iconv -f GBK -t UTF-8 USER/vofa/vofa.c > build/test_vofa.c
for panel in 1 0; do
    for perf in 1 0; do
        cc -std=c99 -Wall -Wextra -Werror -fsanitize=address,undefined \
            -DTEST_REAL_UI -DDISPLAY_DRIVER="$panel" -DUI_SHOW_PERF="$perf" \
            -Itests/stubs -IUSER/upper -IUSER/vofa -IUSER/ui -IUSER/disp \
            USER/upper/upper.c USER/ui/ui.c build/test_vofa.c tests/test_ui.c \
            -lm -o "build/test_ui-$panel-$perf"
        "build/test_ui-$panel-$perf"
    done
done
cc -std=c99 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs -IUSER/disp -IUSER/tft \
    USER/disp/disp.c tests/test_display.c -o build/test_display
build/test_display

