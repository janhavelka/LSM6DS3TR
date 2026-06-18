#!/usr/bin/env python3
from __future__ import annotations

import argparse
import pathlib
import re
import sys
import time
from dataclasses import dataclass
from typing import Iterable

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
    "DEVICE_NOT_FOUND",
    "CHIP_ID_MISMATCH",
    "TIMEOUT",
    "BUSY",
    "IN_PROGRESS",
    "MEASUREMENT_NOT_READY",
    "SELF_TEST_FAIL",
    "FIFO_EMPTY",
    "OFFLINE",
}

FAILURE_TOKENS = STATUS_TOKENS - {"OK"}
ANSI_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


@dataclass
class Classification:
    tokens: set[str]
    failures: set[str]
    selftest_failures: int = 0
    selftest_skips: int = 0
    unknown_command: bool = False

    def ok(self) -> bool:
        return (
            not self.failures
            and self.selftest_failures == 0
            and self.selftest_skips == 0
            and not self.unknown_command
        )


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def classify_output(text: str, allowed_failures: Iterable[str] = ()) -> Classification:
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

    return Classification(
        tokens=tokens,
        failures=failures,
        selftest_failures=selftest_failures,
        selftest_skips=selftest_skips,
        unknown_command="Unknown command" in clean,
    )


def run_parser_self_test() -> None:
    ok = classify_output("Status: OK\nSelftest result: pass=4 fail=0 skip=0\n")
    bad = classify_output("Status: I2C_TIMEOUT\nSelftest result: pass=1 fail=1 skip=0\n")
    allowed = classify_output("Status: FIFO_EMPTY\n", allowed_failures={"FIFO_EMPTY"})
    if not ok.ok():
        raise SystemExit("HIL parser self-test FAILED: OK sample classified as failure")
    if bad.ok() or "I2C_TIMEOUT" not in bad.failures or bad.selftest_failures != 1:
        raise SystemExit("HIL parser self-test FAILED: failure sample not detected")
    if not allowed.ok():
        raise SystemExit("HIL parser self-test FAILED: allowed failure token not honored")
    print("HIL parser self-test PASSED")


def read_idle(serial_port, timeout_s: float, idle_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    last_data = time.monotonic()
    chunks: list[bytes] = []

    while time.monotonic() < deadline:
        data = serial_port.read(4096)
        if data:
            chunks.append(data)
            last_data = time.monotonic()
            continue
        if chunks and (time.monotonic() - last_data) >= idle_s:
            break

    return b"".join(chunks).decode("utf-8", errors="replace")


def run_live(args: argparse.Namespace, commands: list[str]) -> int:
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit("pyserial is required for live HIL: python -m pip install pyserial") from exc

    allowed = set(args.allow_failure)
    with serial.Serial(args.port, args.baud, timeout=args.read_timeout) as port:
        if args.settle_timeout > 0:
            boot_text = read_idle(port, args.settle_timeout, args.idle_timeout)
            if boot_text.strip():
                print(boot_text, end="" if boot_text.endswith("\n") else "\n")

        failed = False
        for command in commands:
            print(f">>> {command}")
            port.write((command + "\n").encode("utf-8"))
            port.flush()
            output = read_idle(port, args.command_timeout, args.idle_timeout)
            print(output, end="" if output.endswith("\n") else "\n")
            classification = classify_output(output, allowed)
            if not classification.ok():
                failed = True
                print(
                    "HIL command FAILED: "
                    f"tokens={sorted(classification.tokens)} "
                    f"failures={sorted(classification.failures)} "
                    f"selftest_fail={classification.selftest_failures} "
                    f"selftest_skip={classification.selftest_skips} "
                    f"unknown={classification.unknown_command}"
                )

    return 1 if failed else 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serial HIL smoke runner for the LSM6DS3TR CLI.")
    parser.add_argument("--port", help="Serial port, for example COM7 or /dev/ttyUSB0.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--command", action="append", help="Command to run; repeat to override defaults.")
    parser.add_argument("--allow-failure", action="append", default=[], choices=sorted(FAILURE_TOKENS))
    parser.add_argument("--dry-run", action="store_true", help="Print command plan without opening serial.")
    parser.add_argument("--parser-self-test", action="store_true", help="Run built-in output classifier checks.")
    parser.add_argument("--classify-file", type=pathlib.Path, help="Classify saved CLI output and exit.")
    parser.add_argument("--settle-timeout", type=float, default=2.0)
    parser.add_argument("--command-timeout", type=float, default=15.0)
    parser.add_argument("--idle-timeout", type=float, default=0.25)
    parser.add_argument("--read-timeout", type=float, default=0.1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    commands = args.command if args.command else DEFAULT_COMMANDS

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

    if args.dry_run:
        print("HIL dry-run command plan:")
        for command in commands:
            print(f"  - {command}")
        print("HIL dry-run PASSED")
        return 0

    if not args.port:
        raise SystemExit("--port is required unless --dry-run or --classify-file is used")

    return run_live(args, commands)


if __name__ == "__main__":
    sys.exit(main())
