# Firmware host tests

Run from the repository root:

```console
python tests/host/run.py
```

Requires Python 3 and GCC on `PATH`. On Windows the runner also checks
`C:/msys64/ucrt64/bin/gcc.exe`. No Python packages or STM32 toolchain are needed.
Every invocation rebuilds; all artifacts go under ignored `Debug/host_tests/`.
A compiler error or failed check produces a nonzero exit status.

The executable builds the real startup self-test, command dispatcher, inputs,
outputs, and Modbus modules. `test_release.c` includes the real `app/release.c`
once, so its boundary checks use the module's actual timing constants.

Only the HAL header is replaced. Production GPIO labels come from the real
`Core/Inc/main.h`; test expectations use independently fixed physical pins
from the confirmed wiring. Release channels use EN PA15 / DIR PE10 and
EN PA10 / DIR PD0, with contact inputs PD1 and PA8 respectively. This makes
swapped production label definitions observable. Ports A through F have
separate input and output registers.
Output writes are mirrored to input reads. Tests set input levels and advance
the virtual millisecond clock directly, without sleeping. HAL reset starts at
1000 ms, clears GPIO, counters and UART capture, and sets the link down. Axis
commands return `ACK_RESULT_OK` and are counted instead of driving hardware.

Boot checks run `selftest_run()` with both contacts closed and both open,
checking its result, command-register cleanup and absence of motor writes or
axis commands. A further 50 ms release step catches delayed startup movement.
The HAL counts SET writes to legacy lock, release EN/DIR, and mast motor pins,
release EN rising edges, and direction writes while the matching EN is high.
Only failed UART assertions and the self-test summary are printed.

Host-only command checks exercise the real dispatcher with counted axis stubs:
an invalid GOTO axis index returns `BAD_ARG` without calling an axis command,
and STOP for a valid axis executes successfully even while the link is down.

These tests check firmware logic and GPIO sequencing; electrical behavior and
actual travel still require hardware verification.
