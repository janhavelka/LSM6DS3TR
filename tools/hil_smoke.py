#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib
import re
import statistics
import sys
import time
from dataclasses import dataclass, field
from typing import Iterable, Sequence

DEFAULT_BAUD = 115200
DEFAULT_COMMAND_TIMEOUT_S = 15.0
SELFTEST_TIMEOUT_S = 120.0
SOAK_DURATION_S = 8 * 60 * 60
SOAK_SIMPLE_TIMEOUT_S = 5.0
SOAK_STRESS_TIMEOUT_S = 30.0
SOAK_RECOVER_TIMEOUT_S = 30.0
PROMPT_RE = re.compile(r"(?:^|\r?\n)>\s*$")

DEFAULT_COMMANDS = [
    "version",
    "scan",
    "probe",
    "settings",
    "health",
    "whoami",
    "status",
    "raw",
    "fifo",
    "selftest",
]

STATUS_TOKENS = {
    "OK",
    "NOT_INITIALIZED",
    "INVALID_CONFIG",
    "INVALID_PARAM",
    "I2C_ERROR",
    "I2C_NACK_ADDR",
    "I2C_NACK_DATA",
    "I2C_TIMEOUT",
    "I2C_BUS",
    "I2C_BUSY",
    "DEVICE_NOT_FOUND",
    "CHIP_ID_MISMATCH",
    "TIMEOUT",
    "BUSY",
    "IN_PROGRESS",
    "MEASUREMENT_NOT_READY",
    "SELF_TEST_FAIL",
    "FIFO_EMPTY",
    "FIFO_OVERRUN",
    "CALIBRATION_UNSTABLE",
    "CALIBRATION_ORIENTATION",
    "OFFLINE",
}

FAILURE_TOKENS = STATUS_TOKENS - {"OK"}
ANSI_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


@dataclass(frozen=True)
class CommandSpec:
    test_id: str
    feature: str
    command: str
    expected: tuple[str, ...] = ()
    allowed_failures: tuple[str, ...] = ()
    timeout_s: float | None = None
    expected_unknown: bool = False
    notes: str = ""


@dataclass
class Classification:
    tokens: set[str]
    failures: set[str]
    missing_expected: list[str] = field(default_factory=list)
    selftest_failures: int = 0
    selftest_skips: int = 0
    unknown_command: bool = False

    def ok(self) -> bool:
        return (
            not self.failures
            and not self.missing_expected
            and self.selftest_failures == 0
            and self.selftest_skips == 0
            and not self.unknown_command
        )


@dataclass
class StepResult:
    test_id: str
    feature: str
    command: str
    expected: str
    observed: str
    elapsed_s: float
    outcome: str
    notes: str
    tokens: list[str]
    failures: list[str]
    prompt_seen: bool


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def clean_one_line(text: str, limit: int = 180) -> str:
    clean = strip_ansi(text).replace("\r", "")
    lines = [line.strip() for line in clean.splitlines() if line.strip() and line.strip() != ">"]
    if not lines:
        return "(no output)"
    joined = " | ".join(lines[:4])
    if len(lines) > 4:
        joined += " | ..."
    if len(joined) > limit:
        return joined[: limit - 3] + "..."
    return joined


def markdown_cell(text: str) -> str:
    return text.replace("|", "\\|").replace("\n", "<br>")


def as_text_tuple(values: Sequence[str] | str) -> tuple[str, ...]:
    if isinstance(values, str):
        return (values,)
    return tuple(values)


def classify_output(
    text: str,
    allowed_failures: Iterable[str] = (),
    expected: Sequence[str] | str = (),
    expected_unknown: bool = False,
) -> Classification:
    clean = strip_ansi(text)
    allowed = set(allowed_failures)
    tokens = set(re.findall(r"\b[A-Z][A-Z0-9_]+\b", clean)) & STATUS_TOKENS
    failures = (tokens & FAILURE_TOKENS) - allowed
    selftest_failures = 0
    selftest_skips = 0

    match = re.search(r"Selftest result:.*?fail=\D*(\d+).*?skip=\D*(\d+)", clean, re.DOTALL)
    if match is not None:
        selftest_failures = int(match.group(1))
        selftest_skips = int(match.group(2))

    missing_expected = [token for token in as_text_tuple(expected) if token not in clean]
    unknown = "Unknown command" in clean and not expected_unknown

    return Classification(
        tokens=tokens,
        failures=failures,
        missing_expected=missing_expected,
        selftest_failures=selftest_failures,
        selftest_skips=selftest_skips,
        unknown_command=unknown,
    )


def run_parser_self_test() -> None:
    ok = classify_output("Status: OK\nSelftest result: pass=4 fail=0 skip=0\n")
    bad = classify_output("Status: I2C_TIMEOUT\nSelftest result: pass=1 fail=1 skip=0\n")
    allowed = classify_output("Status: FIFO_EMPTY\n", allowed_failures={"FIFO_EMPTY"})
    expected = classify_output("WHO_AM_I = 0x6A\n", expected=("WHO_AM_I = 0x6A",))
    missing = classify_output("WHO_AM_I = 0x00\n", expected=("WHO_AM_I = 0x6A",))
    expected_unknown = classify_output("Unknown command: 'bogus'\n", expected_unknown=True)
    if not ok.ok():
        raise SystemExit("HIL parser self-test FAILED: OK sample classified as failure")
    if bad.ok() or "I2C_TIMEOUT" not in bad.failures or bad.selftest_failures != 1:
        raise SystemExit("HIL parser self-test FAILED: failure sample not detected")
    if not allowed.ok():
        raise SystemExit("HIL parser self-test FAILED: allowed failure token not honored")
    if not expected.ok() or missing.ok():
        raise SystemExit("HIL parser self-test FAILED: expected text handling failed")
    if not expected_unknown.ok():
        raise SystemExit("HIL parser self-test FAILED: expected unknown command handling failed")
    print("HIL parser self-test PASSED")


def prompt_seen(text: str) -> bool:
    return PROMPT_RE.search(strip_ansi(text).rstrip()) is not None


def read_until_idle_or_prompt(serial_port, timeout_s: float, idle_s: float, stop_on_prompt: bool) -> str:
    deadline = time.monotonic() + timeout_s
    last_data = time.monotonic()
    chunks: list[bytes] = []

    while time.monotonic() < deadline:
        data = serial_port.read(4096)
        if data:
            chunks.append(data)
            last_data = time.monotonic()
            if stop_on_prompt:
                text = b"".join(chunks).decode("utf-8", errors="replace")
                if prompt_seen(text):
                    break
            continue
        if chunks and not stop_on_prompt and (time.monotonic() - last_data) >= idle_s:
            break

    return b"".join(chunks).decode("utf-8", errors="replace")


def smoke_plan() -> list[CommandSpec]:
    plan: list[CommandSpec] = []
    for index, command in enumerate(DEFAULT_COMMANDS, start=1):
        timeout = SELFTEST_TIMEOUT_S if command == "selftest" else None
        expected = ("Selftest result:",) if command == "selftest" else ()
        plan.append(CommandSpec(f"SMK-{index:02d}", "smoke", command, expected=expected, timeout_s=timeout))
    return plan


def validation_plan(stress_count: int) -> list[CommandSpec]:
    plan = [
        CommandSpec("BAS-001", "connectivity", "help", expected=("CLI Help",)),
        CommandSpec("BAS-002", "connectivity", "version", expected=("Version:", "Commit:")),
        CommandSpec("BAS-003", "connectivity", "scan", expected=("Scan complete", "6A")),
        CommandSpec("BAS-004", "identity", "probe", expected=("Status:", "OK")),
        CommandSpec("BAS-005", "state", "settings", expected=("Driver state:", "READY")),
        CommandSpec("BAS-006", "state", "health", expected=("State:", "READY")),
        CommandSpec("BAS-007", "state", "drv1", expected=("READY",)),
        CommandSpec("BAS-008", "identity", "whoami", expected=("WHO_AM_I = 0x6A", "match=")),
        CommandSpec("BAS-009", "identity", "id", expected=("WHO_AM_I = 0x6A",)),
        CommandSpec("DAT-001", "data", "status", expected=("STATUS_REG",)),
        CommandSpec("DAT-002", "data", "raw", expected=("Raw:",)),
        CommandSpec("DAT-003", "data", "read", expected=("Sample:", "g", "dps", "C")),
        CommandSpec("DAT-004", "data", "accel", expected=("Accel",)),
        CommandSpec("DAT-005", "data", "gyro", expected=("Gyro",)),
        CommandSpec("DAT-006", "data", "temp", expected=("Temp",)),
        CommandSpec("CFG-001", "configuration", "odrxl", expected=("odrxl:",)),
        CommandSpec("CFG-002", "configuration", "odrg", expected=("odrg:",)),
        CommandSpec("CFG-003", "configuration", "fsxl", expected=("Accel FS:",)),
        CommandSpec("CFG-004", "configuration", "fsg", expected=("Gyro FS:",)),
        CommandSpec("CFG-005", "configuration", "apm", expected=("apm:",)),
        CommandSpec("CFG-006", "configuration", "gpm", expected=("gpm:",)),
        CommandSpec("CFG-007", "configuration", "gsleep", expected=("Gyro sleep:",)),
        CommandSpec("CFG-008", "configuration", "fsxl 2", expected=("Status:", "OK")),
        CommandSpec("CFG-009", "configuration", "fsxl 4", expected=("Status:", "OK")),
        CommandSpec("CFG-010", "configuration", "fsxl 8", expected=("Status:", "OK")),
        CommandSpec("CFG-011", "configuration", "fsxl 16", expected=("Status:", "OK")),
        CommandSpec("CFG-012", "configuration", "fsxl 2", expected=("Status:", "OK")),
        CommandSpec("CFG-013", "configuration", "fsg 125", expected=("Status:", "OK")),
        CommandSpec("CFG-014", "configuration", "fsg 250", expected=("Status:", "OK")),
        CommandSpec("CFG-015", "configuration", "fsg 500", expected=("Status:", "OK")),
        CommandSpec("CFG-016", "configuration", "fsg 1000", expected=("Status:", "OK")),
        CommandSpec("CFG-017", "configuration", "fsg 2000", expected=("Status:", "OK")),
        CommandSpec("CFG-018", "configuration", "fsg 250", expected=("Status:", "OK")),
        CommandSpec("CFG-019", "configuration", "odrxl 12.5", expected=("Status:", "OK")),
        CommandSpec("CFG-020", "configuration", "odrxl 104", expected=("Status:", "OK")),
        CommandSpec("CFG-021", "configuration", "odrxl 6660", expected=("Status:", "OK")),
        CommandSpec("CFG-022", "configuration", "odrxl 1.6", expected=("Status:", "OK")),
        CommandSpec("CFG-023", "configuration", "odrxl 104", expected=("Status:", "OK")),
        CommandSpec("CFG-024", "configuration", "apm hp", expected=("Status:", "OK")),
        CommandSpec("CFG-025", "configuration", "odrg 12.5", expected=("Status:", "OK")),
        CommandSpec("CFG-026", "configuration", "odrg 104", expected=("Status:", "OK")),
        CommandSpec("CFG-027", "configuration", "odrg 6660", expected=("Status:", "OK")),
        CommandSpec("CFG-028", "configuration", "odrg 104", expected=("Status:", "OK")),
        CommandSpec("FLT-001", "filters", "alpf2 1", expected=("Status:", "OK")),
        CommandSpec("FLT-002", "filters", "alpf2 0", expected=("Status:", "OK")),
        CommandSpec("FLT-003", "filters", "aslope 1", expected=("Status:", "OK")),
        CommandSpec("FLT-004", "filters", "aslope 0", expected=("Status:", "OK")),
        CommandSpec("FLT-005", "filters", "a6d 1", expected=("Status:", "OK")),
        CommandSpec("FLT-006", "filters", "a6d 0", expected=("Status:", "OK")),
        CommandSpec("FLT-007", "filters", "glpf1 1", expected=("Status:", "OK")),
        CommandSpec("FLT-008", "filters", "glpf1 0", expected=("Status:", "OK")),
        CommandSpec("FLT-009", "filters", "ghpf 1", expected=("Status:", "OK")),
        CommandSpec("FLT-010", "filters", "ghpfmode 3", expected=("Status:", "OK")),
        CommandSpec("FLT-011", "filters", "ghpfmode 0", expected=("Status:", "OK")),
        CommandSpec("FLT-012", "filters", "ghpf 0", expected=("Status:", "OK")),
        CommandSpec("FNC-001", "embedded functions", "ts 1", expected=("Status:", "OK")),
        CommandSpec("FNC-002", "embedded functions", "tsread", expected=("Timestamp",)),
        CommandSpec("FNC-003", "embedded functions", "tshr 1", expected=("Status:", "OK")),
        CommandSpec("FNC-004", "embedded functions", "tsreset", expected=("Status:", "OK")),
        CommandSpec("FNC-005", "embedded functions", "tshr 0", expected=("Status:", "OK")),
        CommandSpec("FNC-006", "embedded functions", "pedo 1", expected=("Status:", "OK")),
        CommandSpec("FNC-007", "embedded functions", "steps", expected=("Step",)),
        CommandSpec("FNC-008", "embedded functions", "stepreset", expected=("Status:", "OK")),
        CommandSpec("FNC-009", "embedded functions", "pedo 0", expected=("Status:", "OK")),
        CommandSpec("FNC-010", "embedded functions", "sigmot 1", expected=("Status:", "OK")),
        CommandSpec("FNC-011", "embedded functions", "sigmot 0", expected=("Status:", "OK")),
        CommandSpec("FNC-012", "embedded functions", "tilt 1", expected=("Status:", "OK")),
        CommandSpec("FNC-013", "embedded functions", "tilt 0", expected=("Status:", "OK")),
        CommandSpec("FNC-014", "embedded functions", "wtilt 1", expected=("Status:", "OK")),
        CommandSpec("FNC-015", "embedded functions", "wtilt 0", expected=("Status:", "OK")),
        CommandSpec("CAL-001", "calibration", "ofswt 16", expected=("Status:", "OK")),
        CommandSpec("CAL-002", "calibration", "offset -4 7 12", expected=("Status:", "OK")),
        CommandSpec("CAL-003", "calibration", "offset", expected=("Offset:",)),
        CommandSpec("CAL-004", "calibration", "offset 0 0 0", expected=("Status:", "OK")),
        CommandSpec("CAL-005", "calibration", "ofswt 1", expected=("Status:", "OK")),
        CommandSpec("CAL-006", "calibration", "biasxl", expected=("Accel bias:",)),
        CommandSpec("CAL-007", "calibration", "biasg", expected=("Gyro bias:",)),
        CommandSpec("CAL-008", "calibration", "biasxl 0.001 0.002 -0.003", expected=("Accel bias set:",)),
        CommandSpec("CAL-009", "calibration", "biasg 0.1 -0.2 0.3", expected=("Gyro bias set:",)),
        CommandSpec("CAL-010", "calibration", "biasreset", expected=("biases cleared",)),
        CommandSpec("FIFO-001", "fifo", "fifo_mode bypass", expected=("Status:", "OK")),
        CommandSpec("FIFO-002", "fifo", "fifo_odr 104", expected=("Status:", "OK")),
        CommandSpec("FIFO-003", "fifo", "fifo_xl 1", expected=("Status:", "OK")),
        CommandSpec("FIFO-004", "fifo", "fifo_g 1", expected=("Status:", "OK")),
        CommandSpec("FIFO-005", "fifo", "fifo_th 8", expected=("Status:", "OK")),
        CommandSpec("FIFO-006", "fifo", "fifo_temp 1", expected=("Status:", "OK")),
        CommandSpec("FIFO-007", "fifo", "fifo_step 1", expected=("Status:", "OK")),
        CommandSpec("FIFO-008", "fifo", "fifo_stop 0", expected=("Status:", "OK")),
        CommandSpec("FIFO-009", "fifo", "fifo_high 0", expected=("Status:", "OK")),
        CommandSpec("FIFO-010", "fifo", "fifo_mode cont", expected=("Status:", "OK")),
        CommandSpec("FIFO-011", "fifo", "fifo", expected=("FIFO",)),
        CommandSpec("FIFO-012", "fifo", "fifo_read 8", allowed_failures=("FIFO_EMPTY",), expected=("FIFO",)),
        CommandSpec("FIFO-013", "fifo", "fifo_mode bypass", expected=("Status:", "OK")),
        CommandSpec("SRC-001", "diagnostics", "wusrc", expected=("0x",)),
        CommandSpec("SRC-002", "diagnostics", "tapsrc", expected=("0x",)),
        CommandSpec("SRC-003", "diagnostics", "6dsrc", expected=("0x",)),
        CommandSpec("SRC-004", "diagnostics", "funcsrc1", expected=("0x",)),
        CommandSpec("SRC-005", "diagnostics", "funcsrc2", expected=("0x",)),
        CommandSpec("SRC-006", "diagnostics", "wtstatus", expected=("0x",)),
        CommandSpec("SRC-007", "diagnostics", "shub 12", expected=("2E:",)),
        CommandSpec("REG-001", "register access", "rreg 0x0F", expected=("[0x0F] = 0x6A",)),
        CommandSpec("REG-002", "register access", "dump 0x10 32", expected=("10:",)),
        CommandSpec("REG-003", "register access", "wreg 0x73 0x00", expected=("Status:", "OK")),
        CommandSpec("RST-001", "reset/recovery", "reset", expected=("Status:", "OK"), timeout_s=30.0),
        CommandSpec("RST-002", "reset/recovery", "health", expected=("State:", "READY")),
        CommandSpec("RST-003", "reset/recovery", "boot", expected=("Status:", "OK"), timeout_s=30.0),
        CommandSpec("RST-004", "reset/recovery", "refresh", expected=("Status:", "OK")),
        CommandSpec("RST-005", "reset/recovery", "recover", expected=("Status:", "OK"), timeout_s=30.0),
        CommandSpec("INV-001", "invalid input", "bogus_command", expected=("Unknown command",), expected_unknown=True),
        CommandSpec("INV-002", "invalid input", "odrxl bad", expected=("Invalid ODR token",)),
        CommandSpec("INV-003", "invalid input", "odrg 1.6", expected=("INVALID_PARAM",), allowed_failures=("INVALID_PARAM",)),
        CommandSpec("INV-004", "invalid input", "fsxl 32", expected=("Expected fsxl",)),
        CommandSpec("INV-005", "invalid input", "rreg 0x100", expected=("Invalid register",)),
        CommandSpec("INV-006", "invalid input", "dump 0x10 0", expected=("Invalid dump range",)),
        CommandSpec("INV-007", "invalid input", "fifo_th 2048", expected=("Expected fifo_th",)),
        CommandSpec("STRESS-001", "stress", f"stress {stress_count}", expected=("Stress Summary", "Errors:"), timeout_s=120.0),
        CommandSpec("STRESS-002", "stress", f"stress_mix {stress_count}", expected=("Mixed Stress Summary", "Errors:"), timeout_s=120.0),
        CommandSpec("SELF-001", "self-test", "selftest", expected=("Selftest result:",), timeout_s=SELFTEST_TIMEOUT_S),
        CommandSpec("POST-001", "post-check", "settings", expected=("Driver state:", "READY")),
        CommandSpec("POST-002", "post-check", "health", expected=("State:", "READY")),
    ]
    return plan


def targeted_plan(stress_count: int) -> list[CommandSpec]:
    return [
        CommandSpec("TGT-BOOT-001", "baseline", "version", expected=("Version:", "Commit:")),
        CommandSpec("TGT-BOOT-002", "baseline", "health", expected=("State:", "READY")),
        CommandSpec("TGT-BOOT-003", "baseline", "whoami", expected=("WHO_AM_I = 0x6A", "match=")),
        CommandSpec("TGT-POLL-001", "poll/manual", "job status", expected=("Job status:",)),
        CommandSpec("TGT-POLL-002", "poll/manual", "job auto 0", expected=("Automatic tick polling: no", "manual=yes")),
        CommandSpec(
            "TGT-POLL-003",
            "poll/direct",
            "job start direct",
            expected=("Status:", "IN_PROGRESS", "Job start:"),
            allowed_failures=("IN_PROGRESS",),
        ),
        CommandSpec(
            "TGT-POLL-004",
            "poll/direct",
            "job get",
            expected=("MEASUREMENT_NOT_READY",),
            allowed_failures=("IN_PROGRESS", "MEASUREMENT_NOT_READY"),
        ),
        CommandSpec(
            "TGT-POLL-005",
            "poll/direct-budget-zero",
            "job poll 0 1",
            expected=("budget=0 -> IN_PROGRESS", "busy=yes"),
            allowed_failures=("IN_PROGRESS",),
        ),
        CommandSpec(
            "TGT-POLL-006",
            "poll/direct-budget-one",
            "job poll 1 1",
            expected=("budget=1 -> OK", "Status: OK"),
            allowed_failures=("IN_PROGRESS",),
        ),
        CommandSpec("TGT-POLL-007", "poll/direct", "job getraw", expected=("Cached raw:",)),
        CommandSpec("TGT-POLL-008", "poll/direct", "job get", expected=("Sample:", "Status: OK")),
        CommandSpec(
            "TGT-POLL-009",
            "poll/sample",
            "job run sample 1 40 2",
            expected=("Job run: kind=sample", "Status: OK"),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=20.0,
        ),
        CommandSpec("TGT-POLL-010", "poll/sample", "job getraw", expected=("Cached raw:",)),
        CommandSpec(
            "TGT-POLL-011",
            "poll/sample-high-budget",
            "job run sample 255 3",
            expected=("Job run: kind=sample", "Status: OK"),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=20.0,
        ),
        CommandSpec(
            "TGT-POLL-012",
            "poll/busy",
            "job start direct",
            expected=("Status:", "IN_PROGRESS"),
            allowed_failures=("IN_PROGRESS",),
        ),
        CommandSpec(
            "TGT-POLL-013",
            "poll/busy",
            "job start refresh",
            expected=("BUSY",),
            allowed_failures=("IN_PROGRESS", "BUSY"),
        ),
        CommandSpec(
            "TGT-POLL-014",
            "poll/busy",
            "job poll 1 3",
            expected=("Status: OK",),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=20.0,
        ),
        CommandSpec(
            "TGT-POLL-015",
            "poll/invalid",
            "job start fifo 0",
            expected=("INVALID_PARAM",),
            allowed_failures=("INVALID_PARAM",),
        ),
        CommandSpec(
            "TGT-POLL-016",
            "poll/invalid",
            "job start calxl 0",
            expected=("INVALID_PARAM",),
            allowed_failures=("INVALID_PARAM",),
        ),
        CommandSpec("TGT-POLL-017", "poll/invalid", "job start bogus", expected=("Invalid job kind",)),
        CommandSpec("TGT-POLL-018", "poll/invalid", "job poll 999", expected=("Invalid poll budget",)),
        CommandSpec(
            "TGT-POLL-019",
            "poll/budget-zero",
            "job run direct 0 3 0",
            expected=("Job run: kind=direct budget=0", "IN_PROGRESS"),
            allowed_failures=("IN_PROGRESS",),
        ),
        CommandSpec(
            "TGT-POLL-020",
            "poll/recover-progress",
            "job poll 1 3",
            expected=("Status: OK",),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=20.0,
        ),
        CommandSpec(
            "TGT-POLL-021",
            "poll/refresh",
            "job run refresh 1 30",
            expected=("Job run: kind=refresh", "Status: OK"),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=20.0,
        ),
        CommandSpec(
            "TGT-POLL-022",
            "poll/refresh",
            "job run refresh 255 3",
            expected=("Job run: kind=refresh", "Status: OK"),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=20.0,
        ),
        CommandSpec(
            "TGT-POLL-023",
            "poll/reset",
            "job run reset 1 60 2",
            expected=("Job run: kind=reset", "Status: OK"),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=30.0,
        ),
        CommandSpec(
            "TGT-POLL-024",
            "poll/boot",
            "job run boot 1 60 2",
            expected=("Job run: kind=boot", "Status: OK"),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=30.0,
        ),
        CommandSpec("TGT-POLL-025", "poll/post", "job auto 1", expected=("Automatic tick polling: yes",)),
        CommandSpec("TGT-FEAT-001", "data", "status", expected=("STATUS_REG",)),
        CommandSpec("TGT-FEAT-002", "data", "raw", expected=("Raw:",)),
        CommandSpec("TGT-FEAT-003", "data", "read", expected=("Sample:", "g", "dps", "C")),
        CommandSpec("TGT-FEAT-004", "data", "accel", expected=("Accel",)),
        CommandSpec("TGT-FEAT-005", "data", "gyro", expected=("Gyro",)),
        CommandSpec("TGT-FEAT-006", "data", "temp", expected=("Temp",)),
        CommandSpec("TGT-FS-001", "full-scale", "fsxl 2", expected=("Status:", "OK")),
        CommandSpec("TGT-FS-002", "full-scale", "fsxl 4", expected=("Status:", "OK")),
        CommandSpec("TGT-FS-003", "full-scale", "fsxl 8", expected=("Status:", "OK")),
        CommandSpec("TGT-FS-004", "full-scale", "fsxl 16", expected=("Status:", "OK")),
        CommandSpec("TGT-FS-005", "full-scale", "fsxl 2", expected=("Status:", "OK")),
        CommandSpec("TGT-FS-006", "full-scale", "fsg 125", expected=("Status:", "OK")),
        CommandSpec("TGT-FS-007", "full-scale", "fsg 250", expected=("Status:", "OK")),
        CommandSpec("TGT-FS-008", "full-scale", "fsg 500", expected=("Status:", "OK")),
        CommandSpec("TGT-FS-009", "full-scale", "fsg 1000", expected=("Status:", "OK")),
        CommandSpec("TGT-FS-010", "full-scale", "fsg 2000", expected=("Status:", "OK")),
        CommandSpec("TGT-FS-011", "full-scale", "fsg 250", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-001", "odr", "odrxl 0", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-002", "odr", "odrxl 1.6", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-003", "odr", "odrxl 12.5", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-004", "odr", "odrxl 26", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-005", "odr", "odrxl 52", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-006", "odr", "odrxl 104", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-007", "odr", "odrxl 208", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-008", "odr", "odrxl 416", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-009", "odr", "odrxl 833", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-010", "odr", "odrxl 1660", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-011", "odr", "odrxl 3330", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-012", "odr", "odrxl 6660", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-013", "odr", "odrxl 104", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-014", "odr", "odrg 0", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-015", "odr", "odrg 12.5", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-016", "odr", "odrg 26", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-017", "odr", "odrg 52", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-018", "odr", "odrg 104", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-019", "odr", "odrg 208", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-020", "odr", "odrg 416", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-021", "odr", "odrg 833", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-022", "odr", "odrg 1660", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-023", "odr", "odrg 3330", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-024", "odr", "odrg 6660", expected=("Status:", "OK")),
        CommandSpec("TGT-ODR-025", "odr", "odrg 104", expected=("Status:", "OK")),
        CommandSpec("TGT-PWR-001", "power", "apm lpn", expected=("Status:", "OK")),
        CommandSpec("TGT-PWR-002", "power", "apm hp", expected=("Status:", "OK")),
        CommandSpec("TGT-PWR-003", "power", "gpm lpn", expected=("Status:", "OK")),
        CommandSpec("TGT-PWR-004", "power", "gpm hp", expected=("Status:", "OK")),
        CommandSpec("TGT-PWR-005", "power", "gsleep 1", expected=("Status:", "OK")),
        CommandSpec("TGT-PWR-006", "power", "gsleep 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-001", "filters", "alpf2 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-002", "filters", "aslope 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-003", "filters", "a6d 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-004", "filters", "glpf1 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-005", "filters", "ghpf 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-006", "filters", "ghpfmode 3", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-007", "filters", "alpf2 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-008", "filters", "aslope 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-009", "filters", "a6d 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-010", "filters", "glpf1 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-011", "filters", "ghpfmode 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FLT-012", "filters", "ghpf 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-001", "embedded-functions", "ts 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-002", "embedded-functions", "tsread", expected=("Timestamp",)),
        CommandSpec("TGT-FNC-003", "embedded-functions", "tshr 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-004", "embedded-functions", "tsreset", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-005", "embedded-functions", "tshr 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-006", "embedded-functions", "pedo 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-007", "embedded-functions", "steps", expected=("Step",)),
        CommandSpec("TGT-FNC-008", "embedded-functions", "stepreset", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-009", "embedded-functions", "pedo 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-010", "embedded-functions", "sigmot 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-011", "embedded-functions", "sigmot 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-012", "embedded-functions", "tilt 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-013", "embedded-functions", "tilt 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-014", "embedded-functions", "wtilt 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FNC-015", "embedded-functions", "wtilt 0", expected=("Status:", "OK")),
        CommandSpec("TGT-CAL-001", "calibration", "ofswt 16", expected=("Status:", "OK")),
        CommandSpec("TGT-CAL-002", "calibration", "offset -4 7 12", expected=("Status:", "OK")),
        CommandSpec("TGT-CAL-003", "calibration", "offset", expected=("Offset:",)),
        CommandSpec("TGT-CAL-004", "calibration", "offset 0 0 0", expected=("Status:", "OK")),
        CommandSpec(
            "TGT-CAL-005",
            "calibration",
            "job run calxl 1 40 2 1",
            expected=("Job run: kind=calxl", "Status: OK"),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=20.0,
        ),
        CommandSpec(
            "TGT-CAL-006",
            "calibration",
            "job run calg 1 40 2 1",
            expected=("Job run: kind=calg", "Status: OK"),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=20.0,
        ),
        CommandSpec("TGT-CAL-007", "calibration", "biasxl", expected=("Accel bias:",)),
        CommandSpec("TGT-CAL-008", "calibration", "biasg", expected=("Gyro bias:",)),
        CommandSpec("TGT-CAL-009", "calibration", "biasreset", expected=("biases cleared",)),
        CommandSpec("TGT-FIFO-001", "fifo", "fifo_mode bypass", expected=("Status:", "OK")),
        CommandSpec("TGT-FIFO-002", "fifo", "fifo_odr 104", expected=("Status:", "OK")),
        CommandSpec("TGT-FIFO-003", "fifo", "fifo_xl 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FIFO-004", "fifo", "fifo_g 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FIFO-005", "fifo", "fifo_th 8", expected=("Status:", "OK")),
        CommandSpec("TGT-FIFO-006", "fifo", "fifo_temp 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FIFO-007", "fifo", "fifo_step 1", expected=("Status:", "OK")),
        CommandSpec("TGT-FIFO-008", "fifo", "fifo_stop 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FIFO-009", "fifo", "fifo_high 0", expected=("Status:", "OK")),
        CommandSpec("TGT-FIFO-010", "fifo", "fifo_mode cont", expected=("Status:", "OK")),
        CommandSpec("TGT-FIFO-011", "fifo", "fifo", expected=("FIFO",)),
        CommandSpec(
            "TGT-FIFO-012",
            "fifo",
            "job run fifo 1 20 2 8",
            expected=("Job run: kind=fifo", "Status: OK"),
            allowed_failures=("IN_PROGRESS",),
            timeout_s=20.0,
        ),
        CommandSpec("TGT-FIFO-013", "fifo", "fifo_read 8", allowed_failures=("FIFO_EMPTY",), expected=("FIFO",)),
        CommandSpec("TGT-FIFO-014", "fifo", "fifo_mode bypass", expected=("Status:", "OK")),
        CommandSpec("TGT-SRC-001", "sources", "wusrc", expected=("0x",)),
        CommandSpec("TGT-SRC-002", "sources", "tapsrc", expected=("0x",)),
        CommandSpec("TGT-SRC-003", "sources", "6dsrc", expected=("0x",)),
        CommandSpec("TGT-SRC-004", "sources", "funcsrc1", expected=("0x",)),
        CommandSpec("TGT-SRC-005", "sources", "funcsrc2", expected=("0x",)),
        CommandSpec("TGT-SRC-006", "sources", "wtstatus", expected=("0x",)),
        CommandSpec("TGT-SRC-007", "sources", "shub 12", expected=("2E:",)),
        CommandSpec("TGT-REG-001", "registers", "rreg 0x0F", expected=("[0x0F] = 0x6A",)),
        CommandSpec("TGT-REG-002", "registers", "dump 0x00 64", expected=("00:", "30:")),
        CommandSpec("TGT-REG-003", "registers", "wreg 0x73 0x00", expected=("Status:", "OK")),
        CommandSpec(
            "TGT-INV-001",
            "invalid-input",
            "odrg 1.6",
            expected=("INVALID_PARAM",),
            allowed_failures=("INVALID_PARAM",),
        ),
        CommandSpec("TGT-INV-002", "invalid-input", "dump 0x10 0", expected=("Invalid dump",)),
        CommandSpec("TGT-INV-003", "invalid-input", "rreg 0x100", expected=("Invalid register",)),
        CommandSpec("TGT-STRESS-001", "stress", f"stress {stress_count}", expected=("Stress Summary", "Errors:"), timeout_s=120.0),
        CommandSpec("TGT-STRESS-002", "stress", f"stress_mix {stress_count}", expected=("Mixed Stress Summary", "Errors:"), timeout_s=120.0),
        CommandSpec("TGT-SELF-001", "self-test", "selftest", expected=("Selftest result:",), timeout_s=SELFTEST_TIMEOUT_S),
        CommandSpec("TGT-POST-001", "post-check", "refresh", expected=("Status:", "OK")),
        CommandSpec("TGT-POST-002", "post-check", "settings", expected=("Driver state:", "READY")),
        CommandSpec("TGT-POST-003", "post-check", "health", expected=("State:", "READY")),
    ]


def benchmark_plan(count: int) -> list[CommandSpec]:
    return [
        CommandSpec("BENCH-001", "benchmark", "health", expected=("State:", "READY")),
        CommandSpec("BENCH-002", "benchmark", f"stress {count}", expected=("Stress Summary", "Errors:"), timeout_s=180.0),
        CommandSpec("BENCH-003", "benchmark", f"stress_mix {count}", expected=("Mixed Stress Summary", "Errors:"), timeout_s=180.0),
        CommandSpec("BENCH-004", "benchmark", "health", expected=("State:", "READY")),
    ]


def soak_cycle(stress_count: int) -> list[CommandSpec]:
    return [
        CommandSpec("SOAK-RAW", "soak/data", "raw", expected=("Raw:",), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-READ", "soak/data", "read", expected=("Sample:", "g", "dps", "C"), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-ACCEL", "soak/data", "accel", expected=("Accel",), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-GYRO", "soak/data", "gyro", expected=("Gyro",), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-TEMP", "soak/data", "temp", expected=("Temp",), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-STATUS", "soak/diagnostics", "status", expected=("STATUS_REG",), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-FIFO", "soak/fifo", "fifo", expected=("FIFO",), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-PROBE", "soak/diagnostics", "probe", expected=("Status:", "OK"), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-HEALTH", "soak/diagnostics", "health", expected=("State:", "READY"), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-CFG", "soak/config", "settings", expected=("Driver state:", "READY"), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-SRC1", "soak/diagnostics", "funcsrc1", expected=("0x",), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-TS", "soak/functions", "tsread", expected=("Timestamp"), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-STEPS", "soak/functions", "steps", expected=("Step"), timeout_s=SOAK_SIMPLE_TIMEOUT_S),
        CommandSpec("SOAK-STRESS", "soak/stress", f"stress {stress_count} quiet", expected=("RESULT stress", "errors=0", "state=READY"), timeout_s=SOAK_STRESS_TIMEOUT_S),
        CommandSpec("SOAK-MIX", "soak/stress", f"stress_mix {stress_count} quiet", expected=("RESULT stress_mix", "fail=0", "state=READY"), timeout_s=SOAK_STRESS_TIMEOUT_S),
        CommandSpec("SOAK-RECOVER", "soak/recovery", "recover", expected=("Status:", "OK"), timeout_s=SOAK_RECOVER_TIMEOUT_S),
    ]


def select_plan(args: argparse.Namespace) -> list[CommandSpec]:
    if args.command:
        return [
            CommandSpec(f"CMD-{index:03d}", "custom", command)
            for index, command in enumerate(args.command, start=1)
        ]
    if args.benchmark:
        return benchmark_plan(args.benchmark_count)
    if args.suite == "targeted":
        return targeted_plan(args.stress_count)
    if args.suite == "validation":
        return validation_plan(args.stress_count)
    return smoke_plan()


def toggle_reset(port, settle_s: float) -> None:
    port.dtr = False
    port.rts = True
    time.sleep(0.05)
    port.rts = False
    time.sleep(settle_s)


def open_serial(args: argparse.Namespace):
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit("pyserial is required for live HIL: python -m pip install pyserial") from exc

    return serial.Serial(args.port, args.baud, timeout=args.read_timeout)


def discard_serial_input(port, args: argparse.Namespace, transcript: list[str], label: str) -> None:
    transcript.append(f"\n<<< discard pending serial: {label} >>>\n")
    try:
        port.reset_input_buffer()
    except Exception:
        pass
    time.sleep(args.command_delay)
    try:
        pending = getattr(port, "in_waiting", 0)
        if pending:
            discarded = port.read(pending).decode("utf-8", errors="replace")
            if discarded:
                transcript.append(discarded)
    except Exception:
        pass


def run_step(port, spec: CommandSpec, args: argparse.Namespace, transcript: list[str]) -> StepResult:
    timeout_s = spec.timeout_s if spec.timeout_s is not None else args.command_timeout
    command_start = time.monotonic()
    prompt_recovered = False
    transcript.append(f"\n>>> {spec.command}\n")
    port.write((spec.command + "\n").encode("utf-8"))
    port.flush()
    output = read_until_idle_or_prompt(port, timeout_s, args.idle_timeout, args.stop_on_prompt)

    if not output.strip() and args.retry_no_prompt > 0:
        for _ in range(args.retry_no_prompt):
            port.write(b"\n")
            port.flush()
            output += read_until_idle_or_prompt(port, timeout_s, args.idle_timeout, args.stop_on_prompt)
            if output.strip():
                break

    output_written = False
    if output.strip() and args.require_prompt and not prompt_seen(output):
        transcript.append(output)
        output_written = True
        for attempt in range(args.retry_missing_prompt):
            transcript.append(f"\n<<< missing prompt retry {attempt + 1} >>>\n")
            port.write(b"\n")
            port.flush()
            extra = read_until_idle_or_prompt(
                port, args.prompt_retry_timeout, args.idle_timeout, args.stop_on_prompt
            )
            output += extra
            transcript.append(extra)
            if prompt_seen(output):
                prompt_recovered = True
                break
        if args.late_prompt_grace > 0 and not prompt_seen(output):
            transcript.append("\n<<< late prompt drain >>>\n")
            extra = read_until_idle_or_prompt(
                port, args.late_prompt_grace, args.idle_timeout, args.stop_on_prompt
            )
            output += extra
            transcript.append(extra)
            if prompt_seen(output):
                prompt_recovered = True
        if args.resync_on_prompt_loss and not prompt_seen(output):
            transcript.append("\n<<< prompt resync: health >>>\n")
            port.write(b"health\n")
            port.flush()
            extra = read_until_idle_or_prompt(
                port, args.prompt_retry_timeout, args.idle_timeout, args.stop_on_prompt
            )
            output += extra
            transcript.append(extra)
            prompt_recovered = prompt_seen(output)

    elapsed_s = time.monotonic() - command_start
    if not output_written:
        transcript.append(output)

    allowed = set(args.allow_failure) | set(spec.allowed_failures)
    classification = classify_output(output, allowed, spec.expected, spec.expected_unknown)
    saw_prompt = prompt_seen(output)
    if not output.strip():
        outcome = "UNKNOWN"
        notes = "No serial output captured"
    elif args.require_prompt and not saw_prompt:
        outcome = "FAIL"
        notes = "Prompt not seen before timeout"
    elif classification.ok():
        outcome = "PASS"
        notes = spec.notes
        if prompt_recovered:
            notes = (notes + "; " if notes else "") + "prompt recovered with health resync"
    else:
        outcome = "FAIL"
        reasons: list[str] = []
        if classification.failures:
            reasons.append(f"failures={sorted(classification.failures)}")
        if classification.missing_expected:
            reasons.append(f"missing={classification.missing_expected}")
        if classification.selftest_failures or classification.selftest_skips:
            reasons.append(
                f"selftest_fail={classification.selftest_failures} "
                f"selftest_skip={classification.selftest_skips}"
            )
        if classification.unknown_command:
            reasons.append("unknown command")
        notes = "; ".join(reasons)

    expected_values = as_text_tuple(spec.expected)
    expected = ", ".join(expected_values) if expected_values else "No failure tokens"
    if spec.allowed_failures:
        expected += f"; allowed failures: {', '.join(spec.allowed_failures)}"

    return StepResult(
        test_id=spec.test_id,
        feature=spec.feature,
        command=spec.command,
        expected=expected,
        observed=clean_one_line(output),
        elapsed_s=elapsed_s,
        outcome=outcome,
        notes=notes,
        tokens=sorted(classification.tokens),
        failures=sorted(classification.failures),
        prompt_seen=saw_prompt,
    )


def print_result(result: StepResult, verbose: bool) -> None:
    status = result.outcome
    print(f"{result.test_id:>10} {status:<7} {result.elapsed_s:7.3f}s  {result.command}")
    if verbose:
        print(f"  observed: {result.observed}")
        if result.notes:
            print(f"  notes: {result.notes}")


def run_plan(args: argparse.Namespace, plan: list[CommandSpec]) -> tuple[int, str, list[StepResult], list[str]]:
    transcript: list[str] = []
    results: list[StepResult] = []

    with open_serial(args) as port:
        if args.reset_before:
            toggle_reset(port, args.reset_settle)

        boot_text = ""
        if args.settle_timeout > 0:
            boot_text = read_until_idle_or_prompt(
                port, args.settle_timeout, args.idle_timeout, args.stop_on_prompt
            )
            transcript.append("=== BOOT/SETTLE ===\n")
            transcript.append(boot_text)
            if args.verbose and boot_text.strip():
                print(boot_text, end="" if boot_text.endswith("\n") else "\n")

        failed = False
        for spec in plan:
            attempts = 0
            while True:
                try:
                    result = run_step(port, spec, args, transcript)
                    break
                except Exception as exc:  # serial exceptions differ by platform.
                    attempts += 1
                    if attempts > args.reconnect_attempts:
                        result = StepResult(
                            test_id=spec.test_id,
                            feature=spec.feature,
                            command=spec.command,
                            expected=", ".join(spec.expected),
                            observed="Serial exception",
                            elapsed_s=0.0,
                            outcome="FAIL",
                            notes=str(exc),
                            tokens=[],
                            failures=[],
                            prompt_seen=False,
                        )
                        break
                    time.sleep(args.reconnect_delay)
                    port.close()
                    port.open()

            results.append(result)
            print_result(result, args.verbose)
            if result.outcome == "FAIL":
                failed = True
                if args.stop_on_fail:
                    break
            if args.command_delay > 0:
                time.sleep(args.command_delay)

    return (1 if failed else 0), boot_text, results, transcript


def try_soak_resync(port, args: argparse.Namespace, transcript: list[str], sequence: int) -> tuple[bool, list[StepResult]]:
    resync_results: list[StepResult] = []
    for attempt in range(1, args.reconnect_attempts + 1):
        transcript.append(f"\n<<< serial reconnect attempt {attempt} >>>\n")
        try:
            port.close()
            time.sleep(args.reconnect_delay)
            port.open()
            time.sleep(args.reconnect_delay)
            discard_serial_input(port, args, transcript, f"soak resync {sequence}-{attempt}")
            spec = CommandSpec(
                test_id=f"SOAK-RESYNC-{sequence:06d}-{attempt}",
                feature="soak/reconnect",
                command="health",
                expected=("State:", "READY"),
                timeout_s=10.0,
            )
            result = run_step(port, spec, args, transcript)
        except Exception as exc:
            result = StepResult(
                test_id=f"SOAK-RESYNC-{sequence:06d}-{attempt}",
                feature="soak/reconnect",
                command="health",
                expected="State:, READY",
                observed="Serial reconnect exception",
                elapsed_s=0.0,
                outcome="FAIL",
                notes=str(exc),
                tokens=[],
                failures=[],
                prompt_seen=False,
            )
        resync_results.append(result)
        if result.outcome == "PASS":
            return True, resync_results
    return False, resync_results


def run_soak(args: argparse.Namespace) -> tuple[int, str, list[StepResult], list[str]]:
    transcript: list[str] = []
    results: list[StepResult] = []
    cycle = soak_cycle(args.stress_count)
    duration_s = args.soak_duration
    deadline = time.monotonic() + duration_s
    iteration = 0
    failed = False

    with open_serial(args) as port:
        if args.reset_before:
            toggle_reset(port, args.reset_settle)
        boot_text = read_until_idle_or_prompt(port, args.settle_timeout, args.idle_timeout, args.stop_on_prompt)
        transcript.append("=== BOOT/SETTLE ===\n")
        transcript.append(boot_text)

        while time.monotonic() < deadline:
            for template in cycle:
                if time.monotonic() >= deadline:
                    break
                iteration += 1
                spec = CommandSpec(
                    test_id=f"{template.test_id}-{iteration:06d}",
                    feature=template.feature,
                    command=template.command,
                    expected=template.expected,
                    allowed_failures=template.allowed_failures,
                    timeout_s=template.timeout_s,
                    expected_unknown=template.expected_unknown,
                    notes=template.notes,
                )
                result = run_step(port, spec, args, transcript)
                resync_results: list[StepResult] = []
                needs_resync = (
                    result.outcome == "UNKNOWN"
                    or (result.outcome == "FAIL" and "Prompt not seen" in result.notes)
                )
                recovered_stream = False
                if needs_resync:
                    recovered_stream, resync_results = try_soak_resync(
                        port, args, transcript, iteration
                    )
                    if recovered_stream:
                        result.outcome = "UNKNOWN"
                        result.notes = (
                            result.notes
                            + "; serial stream resynced with bounded health check"
                        )

                results.append(result)
                print_result(result, args.verbose)
                for resync_result in resync_results:
                    results.append(resync_result)
                    print_result(resync_result, args.verbose)

                if needs_resync and not recovered_stream:
                    failed = True
                    return 1, boot_text, results, transcript
                if result.outcome == "FAIL":
                    failed = True
                    return 1, boot_text, results, transcript
                if args.max_steps and len(results) >= args.max_steps:
                    return 0, boot_text, results, transcript
                if args.command_delay > 0:
                    time.sleep(args.command_delay)

    return (1 if failed else 0), boot_text, results, transcript


def summarize_results(results: Sequence[StepResult]) -> dict[str, int]:
    counts = {"PASS": 0, "FAIL": 0, "UNKNOWN": 0, "NOT_RUN": 0}
    for result in results:
        counts[result.outcome] = counts.get(result.outcome, 0) + 1
    return counts


def timing_summary(results: Sequence[StepResult]) -> str:
    if not results:
        return "No command timing captured."
    values = [r.elapsed_s for r in results]
    mean = statistics.fmean(values)
    return (
        f"count={len(values)}, min={min(values):.3f}s, mean={mean:.3f}s, "
        f"max={max(values):.3f}s"
    )


def write_markdown_report(
    path: pathlib.Path,
    args: argparse.Namespace,
    started: dt.datetime,
    ended: dt.datetime,
    boot_text: str,
    results: Sequence[StepResult],
    transcript_path: pathlib.Path | None,
) -> None:
    counts = summarize_results(results)
    lines = [
        f"# LSM6DS3TR HIL Runner Output - {args.port}",
        "",
        f"- Started: {started.isoformat()}",
        f"- Ended: {ended.isoformat()}",
        f"- Suite: {args.suite}",
        f"- Port: {args.port}",
        f"- Baud: {args.baud}",
        f"- Result counts: PASS={counts.get('PASS', 0)}, FAIL={counts.get('FAIL', 0)}, "
        f"UNKNOWN={counts.get('UNKNOWN', 0)}, NOT_RUN={counts.get('NOT_RUN', 0)}",
        f"- Timing: {timing_summary(results)}",
    ]
    if transcript_path is not None:
        lines.append(f"- Transcript: `{transcript_path.as_posix()}`")
    lines.extend(
        [
            "",
            "## Boot Excerpt",
            "",
            "```text",
            strip_ansi(boot_text).strip()[:4000],
            "```",
            "",
            "## Results",
            "",
            "| Test ID | Feature | Command | Expected | Observed | Elapsed | Result | Notes |",
            "|---|---|---|---|---|---:|---|---|",
        ]
    )
    for result in results:
        lines.append(
            "| "
            + " | ".join(
                [
                    markdown_cell(result.test_id),
                    markdown_cell(result.feature),
                    markdown_cell(f"`{result.command}`"),
                    markdown_cell(result.expected),
                    markdown_cell(result.observed),
                    f"{result.elapsed_s:.3f}s",
                    result.outcome,
                    markdown_cell(result.notes),
                ]
            )
            + " |"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_json(path: pathlib.Path, results: Sequence[StepResult]) -> None:
    payload = [result.__dict__ for result in results]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serial HIL runner for the LSM6DS3TR CLI.")
    parser.add_argument("--port", help="Serial port, for example COM26 or /dev/ttyUSB0.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--suite", choices=("smoke", "validation", "targeted", "soak"), default="smoke")
    parser.add_argument("--command", action="append", help="Command to run; repeat to override suites.")
    parser.add_argument("--allow-failure", action="append", default=[], choices=sorted(FAILURE_TOKENS))
    parser.add_argument("--dry-run", action="store_true", help="Print command plan without opening serial.")
    parser.add_argument("--parser-self-test", action="store_true", help="Run built-in output classifier checks.")
    parser.add_argument("--classify-file", type=pathlib.Path, help="Classify saved CLI output and exit.")
    parser.add_argument("--report-md", type=pathlib.Path, help="Write a Markdown result table.")
    parser.add_argument("--json-out", type=pathlib.Path, help="Write raw step results as JSON.")
    parser.add_argument("--transcript-file", type=pathlib.Path, help="Write raw serial transcript.")
    parser.add_argument("--verbose", action="store_true", help="Print observed output summaries.")
    parser.add_argument("--benchmark", action="store_true", help="Run sample-rate benchmark commands.")
    parser.add_argument("--benchmark-count", type=int, default=500)
    parser.add_argument("--stress-count", type=int, default=50)
    parser.add_argument("--soak-duration", type=float, default=SOAK_DURATION_S)
    parser.add_argument("--max-steps", type=int, default=0, help="Optional hard cap for soak/debug runs.")
    parser.add_argument("--timeout-s", type=float, dest="command_timeout", default=DEFAULT_COMMAND_TIMEOUT_S)
    parser.add_argument("--command-timeout", type=float, dest="command_timeout_alias")
    parser.add_argument("--settle-timeout", type=float, default=2.0)
    parser.add_argument("--boot-settle-s", type=float, dest="boot_settle_alias")
    parser.add_argument("--idle-timeout", type=float, default=0.25)
    parser.add_argument("--read-timeout", type=float, default=0.1)
    parser.add_argument("--command-delay", type=float, default=0.05)
    parser.add_argument("--retry-no-prompt", type=int, default=1)
    parser.add_argument("--retry-missing-prompt", type=int, default=1)
    parser.add_argument("--prompt-retry-timeout", type=float, default=5.0)
    parser.add_argument("--late-prompt-grace", type=float, default=0.75)
    parser.add_argument("--resync-on-prompt-loss", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stop-on-prompt", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-prompt", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stop-on-fail", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--reset-before", action="store_true")
    parser.add_argument("--reset-settle", type=float, default=2.0)
    parser.add_argument("--reconnect-attempts", type=int, default=1)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    args = parser.parse_args()
    if args.command_timeout_alias is not None:
        args.command_timeout = args.command_timeout_alias
    if args.boot_settle_alias is not None:
        args.settle_timeout = args.boot_settle_alias
    return args


def main() -> int:
    args = parse_args()

    if args.parser_self_test:
        run_parser_self_test()

    if args.classify_file is not None:
        text = args.classify_file.read_text(encoding="utf-8", errors="replace")
        classification = classify_output(text, args.allow_failure)
        print(f"tokens={sorted(classification.tokens)}")
        print(f"failures={sorted(classification.failures)}")
        print(f"selftest_fail={classification.selftest_failures}")
        print(f"selftest_skip={classification.selftest_skips}")
        print(f"unknown={classification.unknown_command}")
        return 0 if classification.ok() else 1

    plan = select_plan(args)

    if args.dry_run:
        print("HIL dry-run command plan:")
        if args.suite == "soak" and not args.command and not args.benchmark:
            print(f"  suite: soak for {args.soak_duration:.0f}s")
            for spec in soak_cycle(args.stress_count):
                print(f"  - {spec.command}")
        else:
            for spec in plan:
                print(f"  - {spec.test_id}: {spec.command}")
        print("HIL dry-run PASSED")
        return 0

    if not args.port:
        raise SystemExit("--port is required unless --dry-run or --classify-file is used")

    started = dt.datetime.now().astimezone()
    if args.suite == "soak" and not args.command and not args.benchmark:
        status, boot_text, results, transcript = run_soak(args)
    else:
        status, boot_text, results, transcript = run_plan(args, plan)
    ended = dt.datetime.now().astimezone()

    counts = summarize_results(results)
    print(
        "HIL summary: "
        f"PASS={counts.get('PASS', 0)} "
        f"FAIL={counts.get('FAIL', 0)} "
        f"UNKNOWN={counts.get('UNKNOWN', 0)} "
        f"NOT_RUN={counts.get('NOT_RUN', 0)}"
    )
    print(f"HIL timing: {timing_summary(results)}")

    transcript_path = args.transcript_file
    if transcript_path is not None:
        transcript_path.parent.mkdir(parents=True, exist_ok=True)
        transcript_path.write_text("".join(transcript), encoding="utf-8", errors="replace")

    if args.report_md is not None:
        write_markdown_report(args.report_md, args, started, ended, boot_text, results, transcript_path)

    if args.json_out is not None:
        write_json(args.json_out, results)

    return status


if __name__ == "__main__":
    sys.exit(main())
