#!/usr/bin/env python3
"""Always rebuild and run the firmware host tests using Python and GCC."""

from pathlib import Path
import os
import shutil
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[2]
    compiler = shutil.which("gcc")
    if compiler is None:
        windows_gcc = Path("C:/msys64/ucrt64/bin/gcc.exe")
        if windows_gcc.is_file():
            compiler = str(windows_gcc)
    if compiler is None:
        print("GCC not found; install gcc and add it to PATH.", file=sys.stderr)
        return 1

    build = root / "Debug" / "host_tests"
    build.mkdir(parents=True, exist_ok=True)
    executable = build / ("host_tests.exe" if os.name == "nt" else "host_tests")
    includes = [
        "tests/host/include", "Core/Inc", "app", "modbus", "vendor", "tests",
        "bootloader/Inc", "third_party/mcuboot/boot/bootutil/include",
    ]
    sources = [
        "tests/host/host_hal.c", "tests/host/test_main.c",
        "tests/host/test_boot.c", "tests/host/test_command.c",
        "tests/host/test_release.c", "tests/selftest.c",
        "app/command.c", "app/outputs.c", "app/inputs.c", "app/fw_update.c",
        "tests/host/test_fw_update.c",
        "third_party/mcuboot/boot/bootutil/src/bootutil_public.c",
        "modbus/crc16.c", "modbus/pdu.c", "modbus/regfile.c", "modbus/slave.c",
    ]
    command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-g", "-O0"]
    command.extend("-I" + str(root / path) for path in includes)
    command.extend(str(root / path) for path in sources)
    command.extend(["-o", str(executable)])
    print("Building host tests...", flush=True)
    result = subprocess.run(command, cwd=root)
    if result.returncode != 0:
        return result.returncode
    return subprocess.run([str(executable)], cwd=root).returncode


if __name__ == "__main__":
    sys.exit(main())
