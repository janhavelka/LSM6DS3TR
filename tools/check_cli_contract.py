#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ARDUINO_MAIN = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"

REQUIRED_COMMON = ["BoardConfig.h", "I2cTransport.h"]

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

OWNER_SAFE_TOKENS = [
    "DriverConfig",
    "OperationTiming",
    "OperationToken",
    "#include <cinttypes>",
    'accepted token=%" PRIu64',
    'sample sequence=%" PRIu64',
    'result token=%" PRIu64',
    "startProbe",
    "startConfigure",
    "startSample",
    "poll(now",
    "takeResult",
    "cancelActiveJob",
    "hardwareStateMayHaveChanged",
    "convertSample",
    "diagnosticReadRegister",
    "SensorAddress::SA0_GND",
    "INPUT_CHARS_PER_LOOP",
    "bool inputOverflow = false",
    "input line too long; discarded",
    "inputOverflow = true",
    "validQuantity",
    "validMode",
    "expected sample [all|accel|gyro|temp] [ready|direct]",
    "selftest [5..100]",
    "samples < 5U",
    "samples == 0U",
    "zeroArgumentCommand",
    "argument1 != nullptr",
    "argument2 != nullptr",
    "argument3 != nullptr",
    "Serial.flush()",
]

FORBIDDEN_V1_TOKENS = [
    "device.begin(",
    "device.tick(",
    "requestMeasurement",
    "cachedConfigDirty",
    "readAllRaw",
    "runSelfTest",
    "configureFifo",
]


def fail(message: str) -> None:
    print(f"CLI contract FAILED: {message}")
    raise SystemExit(1)


def main() -> int:
    common_dir = ROOT / "examples" / "common"
    if not ARDUINO_MAIN.exists():
        fail(f"missing Arduino example: {ARDUINO_MAIN.as_posix()}")
    for name in REQUIRED_COMMON:
        if not (common_dir / name).exists():
            fail(f"missing example helper: {name}")

    text = ARDUINO_MAIN.read_text(encoding="utf-8", errors="replace")
    for command in COMMANDS:
        if re.search(rf'"{re.escape(command)}"', text) is None:
            fail(f"owner-safe command '{command}' is missing")
    for token in OWNER_SAFE_TOKENS:
        if token not in text:
            fail(f"owner-safe token '{token}' is missing")
    for token in FORBIDDEN_V1_TOKENS:
        if token in text:
            fail(f"removed v1 API token remains: {token}")

    if "char input[" not in text or "String " in text:
        fail("Arduino CLI must use a fixed character buffer")
    if "POLL_TRANSACTION_BUDGET = 1" not in text:
        fail("Arduino example must demonstrate one-transaction owner polling")
    if "serviced < INPUT_CHARS_PER_LOOP" not in text:
        fail("Arduino console work must be bounded per owner-loop iteration")
    if "if (inputOverflow)" not in text:
        fail("Arduino CLI must discard an entire overlength input line")
    if re.search(
        r'else if \(strcmp\(command, "sample"\).*?'
        r'!validQuantity \|\| !validMode \|\| argument3 != nullptr',
        text,
        re.DOTALL,
    ) is None:
        fail("Arduino sample command must reject invalid or extra arguments")
    if re.search(
        r'else if \(strcmp\(command, "calxl"\).*?'
        r'samples == 0U\)\) \|\|\s*argument2 != nullptr',
        text,
        re.DOTALL,
    ) is None:
        fail("Arduino calibration commands must reject zero or extra arguments")
    if re.search(
        r'selftest.*?samples < 5U\)\) \|\|\s*argument2 != nullptr',
        text,
        re.DOTALL,
    ) is None:
        fail("Arduino self-test command must reject extra arguments")
    for command in ("purge", "rreg", "wreg", "dump"):
        block = re.search(
            rf'else if \(strcmp\(command, "{command}"\).*?\n  \}} else',
            text,
            re.DOTALL,
        )
        if block is None or "argument" not in block.group(0) or "!= nullptr" not in block.group(0):
            fail(f"Arduino {command} command must reject extra arguments")

    print("CLI contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
