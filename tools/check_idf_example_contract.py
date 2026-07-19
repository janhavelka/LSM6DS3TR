#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ARDUINO_MAIN = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"
IDF_MAIN_DIR = ROOT / "examples" / "idf" / "basic" / "main"

COMMANDS = [
    "help",
    "version",
    "status",
    "bind",
    "unbind",
    "probe",
    "configure",
    "sample",
    "reset",
    "boot",
    "recover",
    "reconcile",
    "powerdown",
    "selftest",
    "calxl",
    "calg",
    "purge",
    "cancel",
    "rreg",
    "wreg",
    "dump",
]

REQUIRED_IDF_TOKENS = [
    'extern "C" void app_main',
    '#include "driver/i2c_master.h"',
    "i2c_new_master_bus",
    "i2c_master_bus_add_device",
    "i2c_master_transmit",
    "i2c_master_transmit_receive",
    "esp_timer_get_time",
    "vTaskDelay",
    "getchar()",
    "O_NONBLOCK",
    "clearerr(stdin)",
    "char input[",
    "OperationTiming",
    "#include <cinttypes>",
    'accepted token=%" PRIu64',
    'sample sequence=%" PRIu64',
    'result token=%" PRIu64',
    "startConfigure",
    "startSample",
    "imu.poll(now, 1)",
    "takeResult",
    "cancelActiveJob",
    "SensorAddress::SA0_GND",
    "bool inputOverflow = false",
    "input line too long; discarded",
    "inputOverflow = true",
    "validQuantity",
    "validMode",
    "expected sample [all|accel|gyro|temp] [ready|direct]",
    "selftest [5..100]",
    "samples < 5U",
    "samples == 0U",
    "extra != nullptr",
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
    r"requestMeasurement",
    r"cachedConfigDirty",
    r"runSelfTest",
]


def fail(message: str) -> None:
    print(f"IDF example contract FAILED: {message}")
    raise SystemExit(1)


def read(path: pathlib.Path, label: str) -> str:
    if not path.exists():
        fail(f"missing {label}: {path.as_posix()}")
    return path.read_text(encoding="utf-8", errors="replace")


def command_set(text: str) -> set[str]:
    return {command for command in COMMANDS if re.search(rf'"{re.escape(command)}"', text)}


def main() -> int:
    main_text = read(IDF_MAIN_DIR / "main.cpp", "native ESP-IDF main")
    cmake_text = read(IDF_MAIN_DIR / "CMakeLists.txt", "ESP-IDF main CMake")
    arduino_text = read(ARDUINO_MAIN, "Arduino owner-safe example")
    combined = main_text + "\n" + cmake_text

    for token in REQUIRED_IDF_TOKENS:
        if token not in combined:
            fail(f"required native ESP-IDF token missing: {token}")
    for component in ("LSM6DS3TR", "esp_driver_i2c", "esp_timer", "freertos"):
        if re.search(rf"\b{re.escape(component)}\b", cmake_text) is None:
            fail(f"IDF CMake missing required component '{component}'")
    if '"../../../.."' in cmake_text:
        fail("IDF example must consume only the component's exported public headers")
    for pattern in FORBIDDEN_PATTERNS:
        if re.search(pattern, combined):
            fail(f"forbidden Arduino/legacy token present: {pattern}")
    for stale in ("ArduinoCompat.cpp", "Arduino.h", "Wire.h"):
        if (IDF_MAIN_DIR / stale).exists():
            fail(f"stale compatibility file remains: {stale}")

    idf_commands = command_set(main_text)
    arduino_commands = command_set(arduino_text)
    if idf_commands != set(COMMANDS):
        fail(f"native CLI command set incomplete: {sorted(set(COMMANDS) - idf_commands)}")
    if arduino_commands != idf_commands:
        fail("Arduino and ESP-IDF owner-safe command sets differ")
    if re.search(
        r'else if \(strcmp\(command, "sample"\).*?'
        r'!validQuantity \|\| !validMode \|\| extra != nullptr',
        main_text,
        re.DOTALL,
    ) is None:
        fail("native sample command must reject invalid or extra arguments")
    if re.search(
        r'else if \(strcmp\(command, "calxl"\).*?'
        r'samples == 0U\)\) \|\|\s*extra != nullptr',
        main_text,
        re.DOTALL,
    ) is None:
        fail("native calibration commands must reject zero or extra arguments")

    print("IDF example contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
