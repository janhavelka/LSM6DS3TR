#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
IDF_MAIN_DIR = ROOT / "examples" / "idf" / "basic" / "main"

MANDATORY_COMMANDS = [
    "help",
    "version",
    "ver",
    "scan",
    "begin",
    "init",
    "drv",
    "health",
    "drv1",
    "cfg",
    "settings",
    "refresh",
    "verbose",
    "read",
    "raw",
    "accel",
    "gyro",
    "temp",
    "status",
    "tsread",
    "steps",
    "fifo",
    "fifo_read",
    "stream",
    "odrxl",
    "odrg",
    "fsxl",
    "fsg",
    "apm",
    "gpm",
    "gsleep",
    "reset",
    "boot",
    "alpf2",
    "aslope",
    "a6d",
    "glpf1",
    "ghpf",
    "ghpfmode",
    "ts",
    "tshr",
    "tsreset",
    "pedo",
    "sigmot",
    "tilt",
    "wtilt",
    "stepreset",
    "ofswt",
    "offset",
    "fifo_mode",
    "fifo_odr",
    "fifo_xl",
    "fifo_g",
    "fifo_th",
    "fifo_temp",
    "fifo_step",
    "fifo_stop",
    "fifo_high",
    "cal",
    "calxl",
    "calg",
    "biasxl",
    "biasg",
    "biasreset",
    "probe",
    "recover",
    "whoami",
    "id",
    "wusrc",
    "tapsrc",
    "6dsrc",
    "funcsrc1",
    "funcsrc2",
    "wtstatus",
    "shub",
    "selftest",
    "stress",
    "stress_mix",
    "rreg",
    "wreg",
    "dump",
]

REQUIRED_IDF_TOKENS = [
    'extern "C" void app_main',
    '#include "driver/i2c_master.h"',
    "i2c_new_master_bus",
    "i2c_master_bus_add_device",
    "i2c_master_probe",
    "i2c_master_transmit",
    "i2c_master_transmit_receive",
    "esp_timer_get_time",
    "vTaskDelay",
    "getchar()",
    "char input[",
    "cachedConfigDirty",
    "parseU32Range",
    "parseFloat",
    "FIFO_READ_MAX",
    "STRESS_COUNT_MAX",
]

FORBIDDEN_PATTERNS = [
    r"ArduinoCompat",
    r"IdfArduinoCompat",
    r"Arduino\.h",
    r"Wire\.h",
    r"\bString\b",
    r"\bSerial\b",
    r"\bTwoWire\b",
    r"01_basic_bringup_cli/main\.cpp",
    r"driver/i2c\.h",
    r"i2c_cmd_link",
    r"i2c_driver_install",
]


def fail(message: str) -> None:
    print(f"IDF example contract FAILED: {message}")
    raise SystemExit(1)


def read(path: pathlib.Path, label: str) -> str:
    if not path.exists():
        fail(f"missing {label}: {path.as_posix()}")
    return path.read_text(encoding="utf-8", errors="replace")


def main() -> int:
    main_text = read(IDF_MAIN_DIR / "main.cpp", "native ESP-IDF main")
    cmake_text = read(IDF_MAIN_DIR / "CMakeLists.txt", "ESP-IDF main CMake")
    combined = main_text + "\n" + cmake_text

    for token in REQUIRED_IDF_TOKENS:
      if token not in combined:
        fail(f"required native ESP-IDF token missing: {token}")

    for component in ("LSM6DS3TR", "esp_driver_i2c", "esp_timer", "freertos"):
        if re.search(rf"\b{re.escape(component)}\b", cmake_text) is None:
            fail(f"IDF CMake missing required component '{component}'")

    for pattern in FORBIDDEN_PATTERNS:
        if re.search(pattern, combined):
            fail(f"forbidden Arduino/legacy token present: {pattern}")

    for stale in ("ArduinoCompat.cpp", "Arduino.h", "Wire.h"):
        if (IDF_MAIN_DIR / stale).exists():
            fail(f"stale compatibility file remains: {stale}")

    for command in MANDATORY_COMMANDS:
        if re.search(rf'"{re.escape(command)}"', main_text) is None:
            fail(f"native CLI missing command '{command}'")

    print("IDF example contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
