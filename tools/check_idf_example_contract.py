#!/usr/bin/env python3
"""Enforce native ESP-IDF CLI parity and framework-boundary contracts."""

from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ARDUINO_MAIN = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"
IDF_MAIN_DIR = ROOT / "examples" / "idf" / "basic" / "main"
PROFILE_HELPER = ROOT / "examples" / "common" / "ProfileCli.h"

COMMANDS = {
    "help", "?", "version", "ver", "status", "diag", "job", "result",
    "bind", "unbind", "cancel", "scan", "addr", "freq", "profile", "cfg",
    "settings", "probe", "configure", "sample", "reset", "boot", "recover",
    "reconcile", "powerdown", "selftest", "calxl", "calg", "purge", "rreg",
    "wreg", "dump", "stress", "stress_mix",
}

PROFILE_FIELDS = {
    "xl_odr", "xl_fs", "xl_power", "xl_lpf2", "xl_slope_hp", "xl_6d_lpf",
    "g_odr", "g_fs", "g_power", "g_lpf1", "g_hpf", "g_hpf_mode",
    "g_sleep", "bdu", "offset_weight", "offset", "fifo", "interrupts",
}

REQUIRED_IDF_TOKENS = (
    'extern "C" void app_main',
    '#include "driver/i2c_master.h"',
    '#include "ProfileCli.h"',
    "i2c_new_master_bus",
    "i2c_master_bus_add_device",
    "i2c_master_bus_rm_device",
    "i2c_master_probe",
    "i2c_master_transmit",
    "i2c_master_transmit_receive",
    "esp_timer_get_time",
    "vTaskDelay",
    "getchar()",
    "O_NONBLOCK",
    "clearerr(stdin)",
    "char input[",
    "OperationTiming",
    "OperationToken",
    "DeviceProfile stagedProfile",
    "profile_cli::setField",
    "device.poll(now, POLL_TRANSACTION_BUDGET)",
    "device.takeResult",
    "device.cancelActiveJob",
    "SessionKind::SCAN",
    "SessionKind::STRESS",
    "SessionKind::STRESS_MIX",
    "validateSessionResult",
    "result.state != OperationState::SUCCEEDED",
    "result.hardwareStateMayHaveChanged",
    "last_error present=",
    "mismatch present=",
    "job poll token=",
    "scan summary attempted=",
    "transport_fail_delta=",
    "calibrationTimeoutMs",
    "odrPeriodUs(odr)",
    "fflush(stdout)",
)

FORBIDDEN_PATTERNS = (
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
    r"std::vector",
    r"std::string",
    r"\bmalloc\s*\(",
    r"\bcalloc\s*\(",
    r"\brealloc\s*\(",
)


def fail(message: str) -> None:
    print(f"IDF example contract FAILED: {message}")
    raise SystemExit(1)


def read(path: pathlib.Path, label: str) -> str:
    if not path.exists():
        fail(f"missing {label}: {path.as_posix()}")
    return path.read_text(encoding="utf-8", errors="replace")


def handled_commands(text: str) -> set[str]:
    return set(re.findall(r'strcmp\(command,\s*"([^"]+)"\)', text))


def main() -> int:
    main_text = read(IDF_MAIN_DIR / "main.cpp", "native ESP-IDF main")
    cmake_text = read(IDF_MAIN_DIR / "CMakeLists.txt", "ESP-IDF main CMake")
    arduino_text = read(ARDUINO_MAIN, "Arduino owner-safe example")
    helper_text = read(PROFILE_HELPER, "shared typed profile helper")
    combined = main_text + "\n" + cmake_text

    for token in REQUIRED_IDF_TOKENS:
        if token not in combined:
            fail(f"required native ESP-IDF token missing: {token}")
    for component in (
        "LSM6DS3TR", "esp_driver_i2c", "esp_timer", "freertos", "heap",
        "spi_flash",
    ):
        if re.search(rf"\b{re.escape(component)}\b", cmake_text) is None:
            fail(f"IDF CMake missing required component '{component}'")
    if '"../../../common"' not in cmake_text:
        fail("IDF example must include only the narrow shared-example helper path")
    if '"../../../.."' in cmake_text:
        fail("IDF example must not add the repository root as an include path")
    for pattern in FORBIDDEN_PATTERNS:
        if re.search(pattern, combined):
            fail(f"forbidden Arduino/dynamic-allocation token present: {pattern}")
    for stale in ("ArduinoCompat.cpp", "Arduino.h", "Wire.h"):
        if (IDF_MAIN_DIR / stale).exists():
            fail(f"stale compatibility file remains: {stale}")

    idf_commands = handled_commands(main_text)
    arduino_commands = handled_commands(arduino_text)
    if idf_commands != COMMANDS:
        fail(
            "native CLI command mismatch: "
            f"missing={sorted(COMMANDS - idf_commands)} "
            f"extra={sorted(idf_commands - COMMANDS)}"
        )
    if arduino_commands != idf_commands:
        fail(
            "Arduino/ESP-IDF command mismatch: "
            f"arduino_only={sorted(arduino_commands - idf_commands)} "
            f"idf_only={sorted(idf_commands - arduino_commands)}"
        )
    for field in PROFILE_FIELDS:
        if f'"{field}"' not in helper_text or field not in main_text:
            fail(f"typed profile field '{field}' is not shared and documented")

    if "POLL_TRANSACTION_BUDGET = 1" not in main_text:
        fail("native CLI must demonstrate one-transaction driver polling")
    if "serviced < INPUT_CHARS_PER_LOOP" not in main_text:
        fail("native console input work must be bounded per loop")
    if not re.search(
        r"if \(c == '\\n'\).*?inputOverflow = false;\s*return;",
        main_text,
        re.DOTALL,
    ):
        fail("native console must return after servicing one complete command")
    if not re.search(
        r"serviceInput\(\);\s*serviceOperation\(nowMs\(\)\);", main_text
    ):
        fail("native owner loop must sample time after command admission")
    if not re.search(
        r"bool ownerMutationBlocked\(\).*?session\.kind != SessionKind::NONE.*?"
        r"device\.operationActive\(\).*?device\.resultPending\(\)",
        main_text,
        re.DOTALL,
    ):
        fail("all native owner/session states must gate mutations")

    replace_start = main_text.find("HandleReplacement replaceDeviceHandle")
    normal_replace = main_text.find(
        "i2c_master_dev_handle_t previousDevice", replace_start
    )
    add_index = main_text.find("replacement.status = addDevice", normal_replace)
    remove_index = main_text.find(
        "replacement.status = i2c_master_bus_rm_device(previousDevice)",
        replace_start,
    )
    commit_index = main_text.find("i2c.device = candidate", remove_index)
    if min(replace_start, normal_replace, add_index, remove_index, commit_index) < 0:
        fail("native I2C handle replacement evidence is incomplete")
    if not add_index < remove_index < commit_index:
        fail("candidate handle must be added before old-handle removal and commit")
    if "replacement.originalPreserved = true" not in main_text:
        fail("failed handle replacement must expose preserved-owner evidence")
    if "i2c.orphanDevice = candidate" not in main_text:
        fail("failed candidate cleanup must remain tracked for a bounded retry")

    for marker in (
        "frequency != 100000U && frequency != 400000U",
        "address != 0x6AU && address != 0x6BU",
        "result.state != OperationState::SUCCEEDED",
        "result.hardwareStateMayHaveChanged",
        "Session job reported zero transactions",
        'kind == SessionKind::STRESS_MIX\n             ? "mixed"',
        "lastResultAvailable = false",
        "periodMs * static_cast<uint64_t>(request.samples) * 3U + 5000U",
        "expected sample [all|accel|gyro|temp] [ready|direct]",
        "expected profile [show|validate|defaults|apply|set <field> <value...>]",
        "expected selftest [5..100]",
        "expected calxl [samples [x y z [max_p2p_g]]]",
        "expected calg [samples [max_p2p_dps]]",
        "Stress: stress_mix [1..10000]",
    ):
        if marker not in main_text:
            fail(f"native CLI safety/arity marker missing: {marker}")

    print("IDF example contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
