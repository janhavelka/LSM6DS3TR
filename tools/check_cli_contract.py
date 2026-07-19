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

    print("CLI contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
