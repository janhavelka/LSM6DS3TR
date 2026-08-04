#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ARDUINO_MAIN = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"
PROFILE_HELPER = ROOT / "examples" / "common" / "ProfileCli.h"

REQUIRED_COMMON = ["BoardConfig.h", "I2cTransport.h", "ProfileCli.h"]

COMMANDS = [
    "help",
    "?",
    "version",
    "ver",
    "status",
    "diag",
    "job",
    "result",
    "bind",
    "unbind",
    "scan",
    "addr",
    "freq",
    "profile",
    "cfg",
    "settings",
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
    "stress",
    "stress_mix",
]

PROFILE_FIELDS = [
    "xl_odr", "xl_fs", "xl_power", "xl_lpf2", "xl_slope_hp",
    "xl_6d_lpf", "g_odr", "g_fs", "g_power", "g_lpf1", "g_hpf",
    "g_hpf_mode", "g_sleep", "bdu", "offset_weight", "offset", "fifo",
    "interrupts",
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
    "selectedAddress",
    "selectedFrequencyHz",
    "INPUT_CHARS_PER_LOOP",
    "MAX_COMMAND_TOKENS",
    "MAX_STRESS_COUNT = 10000",
    "bool inputOverflow = false",
    "input line too long; discarded",
    "inputOverflow = true",
    "tokenize(line, tokens, count)",
    "parseSampleArguments",
    "profile_cli::setField",
    "DeviceProfile stagedProfile",
    "SessionKind::SCAN",
    "SessionKind::STRESS",
    "SessionKind::STRESS_MIX",
    "startNextSessionOperation",
    "validateSessionResult",
    "transport::wireProbe",
    "transport::setWireFrequency",
    "expected sample [all|accel|gyro|temp] [ready|direct]",
    "selftest [5..100]",
    "samples < 5U",
    "samples == 0U",
    "last_error present=",
    "mismatch present=",
    "settle_remaining_ms=",
    "job poll token=",
    "profile locked",
    "scan summary attempted=",
    "transport_fail_delta=",
    "Session job reported zero transactions",
    "calibrationTimeoutMs",
    "odrPeriodUs(odr)",
    "restore_code=",
    "Values: xl_odr=",
    "Values: g_odr=",
    "Serial.flush()",
]

PROFILE_HELPER_TOKENS = [
    "DeviceProfile candidate = profile",
    "const Status valid = validateProfile(candidate)",
    "profile = candidate",
    "parseSigned",
    "parseGyroHpfMode",
    "inline bool equal",
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
    helper = PROFILE_HELPER.read_text(encoding="utf-8", errors="replace")
    for command in COMMANDS:
        if re.search(rf'"{re.escape(command)}"', text) is None:
            fail(f"owner-safe command '{command}' is missing")
    for token in OWNER_SAFE_TOKENS:
        if token not in text:
            fail(f"owner-safe token '{token}' is missing")
    for field in PROFILE_FIELDS:
        if f'"{field}"' not in helper or field not in text:
            fail(f"typed profile field '{field}' is not shared and documented")
    for token in PROFILE_HELPER_TOKENS:
        if token not in helper:
            fail(f"profile helper safety token '{token}' is missing")
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
    if not re.search(
        r"bool ownerMutationBlocked\(\).*?session\.kind != SessionKind::NONE.*?"
        r"device\.operationActive\(\).*?device\.resultPending\(\)",
        text,
        re.DOTALL,
    ):
        fail("all application and driver ownership states must gate mutations")
    if "count >= MAX_COMMAND_TOKENS" not in text:
        fail("bounded tokenizer must reject tokens beyond maximum arity")
    if "frequency != 100000U && frequency != 400000U" not in text:
        fail("runtime frequency selection must remain within the chip's I2C limits")
    if not re.search(
        r'if \(!requireOwnerIdle\("freq"\)\) return;\s*if \(!busReady\)', text
    ):
        fail("runtime frequency mutation must require an initialized owner bus")
    if "address != 0x6AU && address != 0x6BU" not in text:
        fail("runtime address selection must remain within the SA0 contract")
    if "periodMs * static_cast<uint64_t>(request.samples) * 3U + 5000U" not in text:
        fail("calibration deadline must cover every supported ODR/sample count")
    if not re.search(
        r"pendingToken = token;\s*lastPoll = \{\};\s*lastPoll\.status = status;",
        text,
    ):
        fail("newly accepted jobs must not expose stale prior poll evidence")
    for marker in (
        "count != 2U || !parseUnsigned(tokens[1], 2048",
        "count != 2U || !parseUnsigned(tokens[1], 0xFF",
        "count != 3U || !parseUnsigned(tokens[1], 0xFF",
        "count != 3U || !parseUnsigned(tokens[1], 0xFF, reg)",
    ):
        if marker not in text:
            fail(f"strict command arity marker missing: {marker}")

    print("CLI contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
