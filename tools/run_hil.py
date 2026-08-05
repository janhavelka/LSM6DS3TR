#!/usr/bin/env python3
"""Repeatable targeted HIL campaign for the owner-safe Arduino example.

The runner deliberately drives only the example's public CLI. It never reaches
around the application-owned transport, and it keeps raw evidence outside the
repository unless the caller explicitly selects a repository path. Long soak
testing uses the low-output device-side owner harness and ``run_owner_soak.py``;
streaming thousands of CLI result records is not a reliable native-USB soak.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import re
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable

try:
    import serial
    from serial.tools import list_ports
except ImportError as error:  # pragma: no cover - depends on the HIL host
    serial = None  # type: ignore[assignment]
    list_ports = None  # type: ignore[assignment]
    SERIAL_IMPORT_ERROR: ImportError | None = error
else:
    SERIAL_IMPORT_ERROR = None

SERIAL_EXCEPTION = serial.SerialException if serial is not None else OSError


ROOT = pathlib.Path(__file__).resolve().parents[1]
EXPECTED_LIBRARY_VERSION = str(
    json.loads((ROOT / "library.json").read_text(encoding="utf-8"))["version"]
)
RESULT_RE = re.compile(
    r"result token=(\d+) kind=(\w+) state=(\w+) "
    r"transactions=(\d+)/(\d+) changed=(yes|no) "
    r"started_ms=(\d+) completed_ms=(\d+)"
)
SAMPLE_RE = re.compile(
    r"sample sequence=(\d+) generation=(\d+) valid=0x([0-9A-Fa-f]+) "
    r"fresh=0x([0-9A-Fa-f]+) read_ms=(\d+) quality=(\w+) "
    r"xl_fs_g=([^\s]+) g_fs_dps=([^\s]+)"
)
RAW_SAMPLE_RE = re.compile(
    r"raw accel=(-?\d+),(-?\d+),(-?\d+) "
    r"gyro=(-?\d+),(-?\d+),(-?\d+) temp=(-?\d+)"
)
STATUS_CODE_RE = re.compile(r"status=(\d+) detail=(-?\d+) message=([^\r\n]+)")
TRANSPORT_RE = re.compile(r"transport ok=(\d+) fail=(\d+) last_error=(\d+)")
BUS_RE = re.compile(
    r"bus ready=(yes|no) selected_address=0x([0-9A-Fa-f]{2}) bound_address=([^\s]+) "
    r"frequency_hz=(\d+) timeout_ms=(\d+)"
)
BUS_INIT_RE = re.compile(
    r"bus_init code=(\d+) detail=(-?\d+) message=([^\r\n]+)"
)
DRIVER_STATE_RE = re.compile(
    r"bound=(yes|no) active=(yes|no) result_pending=(yes|no) "
    r"config=(\w+) generation=(\d+) valid_after=(\d+) settle_remaining_ms=(\d+)"
)
LAST_ERROR_RE = re.compile(
    r"last_error present=(yes|no) code=(\d+) detail=(-?\d+) "
    r"message=(.*?) time_ms=(\d+) age_ms=(\d+)"
)
MISMATCH_RE = re.compile(
    r"mismatch present=(yes|no) register=0x([0-9A-Fa-f]{2}) "
    r"expected=0x([0-9A-Fa-f]{2}) observed=0x([0-9A-Fa-f]{2})"
)
SCAN_SUMMARY_RE = re.compile(
    r"scan summary attempted=(\d+) found=(\d+) failures=(\d+) "
    r"cancelled=(yes|no) elapsed_ms=(\d+)"
)
STRESS_SUMMARY_RE = re.compile(
    r"(stress|stress_mix) summary requested=(\d+) completed=(\d+) "
    r"ok=(\d+) fail=(\d+) cancelled=(yes|no) aborted=(yes|no) "
    r"elapsed_ms=(\d+) transport_ok_delta=(\d+) transport_fail_delta=(\d+) "
    r"probes=(\d+) reconciles=(\d+) samples=(\d+)"
)
PLATFORM_RE = re.compile(
    r"platform arduino=([^\s]+) esp-idf=([^\s]+) "
    r"flash_bytes=(\d+) psram_bytes=(\d+)"
)
EXPECTED_ARDUINO_CORE = "3.3.11"
EXPECTED_ESP_IDF = "v5.5.5"
EXPECTED_FLASH_BYTES = 4 * 1024 * 1024
EXPECTED_PSRAM_BYTES = 2 * 1024 * 1024


class HilFailure(RuntimeError):
    pass


@dataclass
class JobResult:
    command: str
    token: int
    kind: str
    state: str
    transactions: int
    transaction_limit: int
    changed: bool
    status_code: int
    status_message: str
    started_ms: int
    completed_ms: int
    output: str


class SerialCli:
    def __init__(
        self, port: str, baud: int, raw_log: pathlib.Path, assert_dtr: bool = False
    ) -> None:
        self.port = port
        self.baud = baud
        self.assert_dtr = assert_dtr
        self.raw_log = raw_log.open("a", encoding="utf-8", newline="\n")
        self.serial: serial.Serial | None = None
        self.last_command_at = 0.0
        self.open()

    def log(self, direction: str, text: str) -> None:
        stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        for line in text.replace("\r", "").splitlines() or [""]:
            self.raw_log.write(f"{stamp} {direction} {line}\n")
        self.raw_log.flush()

    def open(self) -> None:
        endpoint = serial.Serial()
        endpoint.port = self.port
        endpoint.baudrate = self.baud
        endpoint.timeout = 0.05
        endpoint.write_timeout = 2.0
        endpoint.rtscts = False
        endpoint.dsrdtr = False
        # Configure these before open. Setting the pyserial defaults after open
        # can hold GPIO0 active and reset an ESP32-S3 into its ROM downloader.
        endpoint.dtr = self.assert_dtr
        endpoint.rts = False
        endpoint.open()
        self.serial = endpoint
        self.log(
            "HOST",
            f"opened {self.port} baud={self.baud} "
            f"dtr={int(self.assert_dtr)} rts=0",
        )

    def close(self) -> None:
        self.close_endpoint()
        self.raw_log.close()

    def close_endpoint(self) -> None:
        if self.serial is not None:
            try:
                self.serial.close()
            except serial.SerialException as error:
                self.log("HOST", f"serial close failed: {error}")
            self.serial = None

    def send(self, command: str) -> None:
        if self.serial is None:
            raise HilFailure("serial endpoint is closed")
        guard_remaining = 0.50 - (time.monotonic() - self.last_command_at)
        if guard_remaining > 0:
            time.sleep(guard_remaining)
        self.log("TX", command)
        self.serial.write((command + "\n").encode("ascii"))
        self.serial.flush()
        self.last_command_at = time.monotonic()

    def read_until(
        self,
        done: Callable[[str], bool],
        timeout_s: float,
        quiet_s: float = 0.12,
    ) -> str:
        if self.serial is None:
            raise HilFailure("serial endpoint is closed")
        started = time.monotonic()
        last_data = started
        matched_at: float | None = None
        raw = bytearray()
        while time.monotonic() - started < timeout_s:
            available = self.serial.in_waiting
            if available > 0:
                chunk = self.serial.read(available)
            elif raw and matched_at is None:
                # Windows can briefly report zero between fragments of one
                # native-USB packet. A bounded continuation read is safe after
                # a response starts; avoid issuing/cancelling reads while idle.
                chunk = self.serial.read(1)
            else:
                chunk = b""
            now = time.monotonic()
            if chunk:
                raw.extend(chunk)
                last_data = now
                text = raw.decode("utf-8", errors="replace").replace("\r", "")
                if done(text) and matched_at is None:
                    matched_at = now
            if matched_at is not None and now - last_data >= quiet_s:
                break
            if not chunk:
                time.sleep(0.005)
        text = raw.decode("utf-8", errors="replace").replace("\r", "")
        if text:
            self.log("RX", text)
        if not done(text):
            raise HilFailure(f"timeout waiting for response; received: {text!r}")
        return text

    def drain(self, timeout_s: float = 0.5) -> str:
        return self.read_until(lambda text: bool(text), timeout_s)

    def exchange(self, command: str, expected: str, timeout_s: float = 2.0) -> str:
        self.send(command)
        return self.read_until(lambda text: expected in text, timeout_s)

    def job(self, command: str, timeout_s: float = 35.0) -> JobResult:
        self.send(command)
        token: int | None = None

        def correlated_result(
            text: str, accepted_end: int, expected_token: int
        ) -> re.Match[str] | None:
            matches = [
                match
                for match in RESULT_RE.finditer(text, accepted_end)
                if int(match.group(1)) == expected_token
            ]
            return matches[-1] if matches else None

        def terminal(text: str) -> bool:
            nonlocal token
            accepted_matches = list(re.finditer(r"accepted token=(\d+)", text))
            if not accepted_matches:
                return False
            accepted = accepted_matches[-1]
            token = int(accepted.group(1))
            result = correlated_result(text, accepted.end(), token)
            if result is None:
                return False
            result_block = text[result.start():]
            if STATUS_CODE_RE.search(result_block) is None:
                return False
            kind = result.group(2)
            state = result.group(3)
            if kind == "sample" and state == "succeeded":
                return SAMPLE_RE.search(result_block) is not None
            if kind == "probe" and state == "succeeded":
                return "who_am_i=" in result_block
            if kind in ("configure", "recover", "reconcile") and state == "succeeded":
                return "valid_after_ms=" in result_block
            if kind == "selftest":
                return "restore_code=" in result_block
            if kind == "calibration" and state == "succeeded":
                return "samples=" in result_block
            if kind == "purge":
                return "truncated=" in result_block
            return True

        output = self.read_until(terminal, timeout_s)
        if token is None:
            raise HilFailure(f"{command!r} was not accepted: {output!r}")
        accepted_matches = list(re.finditer(r"accepted token=(\d+)", output))
        if not accepted_matches:
            raise HilFailure(f"{command!r} was not accepted: {output!r}")
        accepted = accepted_matches[-1]
        token = int(accepted.group(1))
        result_match = correlated_result(output, accepted.end(), token)
        if result_match is None:
            raise HilFailure(f"malformed terminal result for {command!r}: {output!r}")
        result_block = output[result_match.start():]
        status_match = STATUS_CODE_RE.search(result_block)
        if status_match is None:
            raise HilFailure(f"missing correlated status for {command!r}: {output!r}")
        return JobResult(
            command=command,
            token=token,
            kind=result_match.group(2),
            state=result_match.group(3),
            transactions=int(result_match.group(4)),
            transaction_limit=int(result_match.group(5)),
            changed=result_match.group(6) == "yes",
            status_code=int(status_match.group(1)),
            status_message=status_match.group(3).strip(),
            started_ms=int(result_match.group(7)),
            completed_ms=int(result_match.group(8)),
            output=output,
        )


def require(condition: bool, message: str) -> None:
    if not condition:
        raise HilFailure(message)


def require_success(result: JobResult, expected_kind: str) -> None:
    require(result.kind == expected_kind, f"{result.command}: wrong kind {result.kind}")
    require(result.state == "succeeded", f"{result.command}: state={result.state}")
    require(result.status_code == 0, f"{result.command}: {result.status_message}")
    require(result.transactions > 0, f"{result.command}: no physical transaction")
    require(
        result.transactions <= result.transaction_limit,
        f"{result.command}: transaction limit exceeded",
    )
    require(
        result.completed_ms >= result.started_ms,
        f"{result.command}: completion timestamp precedes admission",
    )
    if expected_kind in {"probe", "sample", "reconcile", "calibration"}:
        require(not result.changed, f"{result.command}: read-only job reported change")
    if expected_kind in {
        "configure", "reset", "boot", "recover", "powerdown", "selftest"
    }:
        require(result.changed, f"{result.command}: write job omitted change evidence")


def parse_status_code(output: str) -> int:
    matches = list(STATUS_CODE_RE.finditer(output))
    if not matches:
        raise HilFailure(f"missing status code in {output!r}")
    return int(matches[-1].group(1))


def parse_transport(output: str) -> tuple[int, int, int]:
    match = TRANSPORT_RE.search(output)
    if match is None:
        raise HilFailure(f"missing transport diagnostics in {output!r}")
    return tuple(int(match.group(index)) for index in range(1, 4))


def parse_bus(output: str) -> tuple[bool, int, str, int, int]:
    match = BUS_RE.search(output)
    if match is None:
        raise HilFailure(f"missing bus diagnostics in {output!r}")
    return (
        match.group(1) == "yes",
        int(match.group(2), 16),
        match.group(3),
        int(match.group(4)),
        int(match.group(5)),
    )


def require_last_transport_error(output: str) -> None:
    match = LAST_ERROR_RE.search(output)
    require(match is not None and match.group(1) == "yes",
            "last transport error evidence was not exposed")
    assert match is not None
    require(int(match.group(2)) != 0,
            "transport error code was not exposed")
    # The detail field is context-specific. Zero is meaningful for a read
    # length mismatch because it records that the transport returned no bytes.
    require(bool(match.group(4).strip()),
            "transport error message was not exposed")
    require(int(match.group(5)) > 0,
            "transport error timestamp was not exposed")


def parse_driver_state(output: str) -> tuple[str, str, str, str, int, int, int]:
    match = DRIVER_STATE_RE.search(output)
    if match is None:
        raise HilFailure(f"missing driver state diagnostics in {output!r}")
    return (
        match.group(1), match.group(2), match.group(3), match.group(4),
        int(match.group(5)), int(match.group(6)), int(match.group(7)),
    )


def parse_profile(output: str, label: str = "staged") -> dict[str, str]:
    fields: dict[str, str] = {}
    prefix = f"profile {label} "
    for line in output.replace("\r", "").splitlines():
        if not line.startswith(prefix):
            continue
        for key, value in re.findall(r"([a-zA-Z0-9_]+)=([^\s]+)", line[len(prefix):]):
            fields[key] = value
    required = {
        "xl_odr", "xl_fs_g", "xl_power", "xl_lpf2", "xl_slope_hp",
        "xl_6d_lpf", "g_odr", "g_fs_dps", "g_power", "g_lpf1", "g_hpf",
        "g_hpf_mode", "g_sleep", "bdu", "offset_weight_mg", "offset_x",
        "offset_y", "offset_z", "fifo", "interrupts",
    }
    if not required.issubset(fields):
        raise HilFailure(f"incomplete {label} profile in {output!r}")
    return fields


def parse_stress_summary(output: str, expected_kind: str) -> dict[str, int | bool]:
    matches = list(STRESS_SUMMARY_RE.finditer(output))
    if not matches:
        raise HilFailure(f"missing stress summary in {output!r}")
    match = matches[-1]
    if match.group(1) != expected_kind:
        raise HilFailure(f"wrong stress summary kind {match.group(1)}")
    result: dict[str, int | bool] = {
        "requested": int(match.group(2)),
        "completed": int(match.group(3)),
        "ok": int(match.group(4)),
        "fail": int(match.group(5)),
        "cancelled": match.group(6) == "yes",
        "aborted": match.group(7) == "yes",
        "transport_ok": int(match.group(9)),
        "transport_fail": int(match.group(10)),
        "probes": int(match.group(11)),
        "reconciles": int(match.group(12)),
        "samples": int(match.group(13)),
    }
    return result


def require_stress_success(output: str, kind: str, requested: int) -> dict[str, int | bool]:
    summary = parse_stress_summary(output, kind)
    require(summary["requested"] == requested, f"{kind}: wrong requested count")
    require(summary["completed"] == requested, f"{kind}: incomplete run")
    require(summary["ok"] == requested, f"{kind}: success count mismatch")
    require(summary["fail"] == 0, f"{kind}: failures reported")
    require(not summary["cancelled"], f"{kind}: unexpectedly cancelled")
    require(not summary["aborted"], f"{kind}: unexpectedly aborted")
    require(summary["transport_ok"] >= requested,
            f"{kind}: too few successful physical transactions")
    require(summary["transport_fail"] == 0, f"{kind}: transport failure")
    return summary


def command_rejected(cli: SerialCli, command: str, expected: str) -> None:
    output = cli.exchange(command, expected)
    require("accepted token=" not in output, f"invalid command was accepted: {command}")
    statuses = list(STATUS_CODE_RE.finditer(output))
    if statuses:
        require(int(statuses[-1].group(1)) != 0,
                f"invalid command reported success: {command}")


def run_invalid_matrix(cli: SerialCli) -> int:
    checks = 0
    before_status = cli.exchange("status", "transport ok=")
    before_transport = parse_transport(before_status)
    before_bus = parse_bus(before_status)
    before_driver = parse_driver_state(before_status)
    before_profile = parse_profile(cli.exchange("profile", "profile locked"))
    zero_argument = (
        ("help", "help"), ("?", "help"), ("version", "version"),
        ("ver", "version"), ("status", "status"), ("diag", "diag"),
        ("bind", "bind"), ("unbind", "unbind"), ("cancel", "cancel"),
        ("scan", "scan"), ("result", "result"), ("cfg", "cfg"),
        ("settings", "settings"), ("probe", "probe"),
        ("configure", "configure"), ("reset", "reset"), ("boot", "boot"),
        ("recover", "recover"), ("reconcile", "reconcile"),
        ("powerdown", "powerdown"),
    )
    for command, expected_name in zero_argument:
        command_rejected(cli, f"{command} extra", f"expected {expected_name}")
        checks += 1
    cases = (
        ("job nope", "expected job"),
        ("job current extra", "expected job"),
        ("addr 0x69", "expected addr"),
        ("addr 0x6c", "expected addr"),
        ("addr -1", "expected addr"),
        ("addr 0x6a extra", "expected addr"),
        ("freq 0", "expected freq"),
        ("freq 100001", "expected freq"),
        ("freq 400001", "expected freq"),
        ("freq -1", "expected freq"),
        ("freq 100000 extra", "expected freq"),
        ("profile show extra", "expected profile"),
        ("profile set", "expected profile"),
        ("profile set unknown 1", "status="),
        ("profile set xl_odr 1.7", "status="),
        ("profile set xl_fs 3", "status="),
        ("profile set xl_power turbo", "status="),
        ("profile set xl_lpf2 2", "status="),
        ("profile set xl_slope_hp 1", "status="),
        ("profile set xl_6d_lpf 1", "status="),
        ("profile set g_odr 1.6", "status="),
        ("profile set g_fs 126", "status="),
        ("profile set g_power turbo", "status="),
        ("profile set g_lpf1 2", "status="),
        ("profile set g_hpf 1", "status="),
        ("profile set g_hpf_mode 0.1", "status="),
        ("profile set g_sleep 2", "status="),
        ("profile set bdu 0", "status="),
        ("profile set offset_weight 2", "status="),
        ("profile set offset -128 0 0", "status="),
        ("profile set offset 0 0 128", "status="),
        ("profile set offset 0 0", "status="),
        ("profile set offset 0 0 0 0", "status="),
        ("profile set fifo 1", "status="),
        ("profile set interrupts 1", "status="),
        ("sample nope", "expected sample"),
        ("sample all nope", "expected sample"),
        ("sample all ready extra", "expected sample"),
        ("selftest 4", "expected selftest"),
        ("selftest 101", "expected selftest"),
        ("selftest 5 extra", "expected selftest"),
        ("calxl 0", "expected calxl"),
        ("calxl 1001", "expected calxl"),
        ("calxl 5 extra", "expected calxl"),
        ("calxl 5 0 0", "expected calxl"),
        ("calxl 5 0 0 0", "status="),
        ("calxl 5 0 0 1 0", "expected calxl"),
        ("calg 0", "expected calg"),
        ("calg 1001", "expected calg"),
        ("calg 5 extra", "expected calg"),
        ("calg 5 0 0", "expected calg"),
        ("purge", "expected purge"),
        ("purge 0", "expected purge"),
        ("purge 2049", "expected purge"),
        ("purge 1 extra", "expected purge"),
        ("rreg", "expected rreg"),
        ("rreg 256", "expected rreg"),
        ("rreg 0x0f extra", "expected rreg"),
        ("wreg 0x12", "expected wreg"),
        ("wreg 256 0", "expected wreg"),
        ("wreg 0x12 0 1", "expected wreg"),
        ("dump 0x20 0", "expected dump"),
        ("dump 0x20 33", "expected dump"),
        ("dump 0x20 1 extra", "expected dump"),
        ("stress 0", "expected stress"),
        ("stress 10001", "expected stress"),
        ("stress 1 nope", "expected stress"),
        ("stress 1 all nope", "expected stress"),
        ("stress 1 all ready extra", "expected stress"),
        ("stress_mix 0", "expected stress_mix"),
        ("stress_mix 10001", "expected stress_mix"),
        ("stress_mix 1 extra", "expected stress_mix"),
    )
    for command, expected in cases:
        command_rejected(cli, command, expected)
        checks += 1
    command_rejected(cli, "not-a-command", "unknown command")
    checks += 1
    cli.send("x" * 140)
    overflow = cli.read_until(lambda text: "input line too long; discarded" in text, 2.0)
    require("unknown command" not in overflow, "overflow tail was parsed as a command")
    checks += 1
    after_status = cli.exchange("status", "transport ok=")
    require(parse_transport(after_status) == before_transport,
            "invalid command matrix performed I2C")
    require(parse_bus(after_status) == before_bus,
            "invalid command matrix changed owner bus state")
    require(parse_driver_state(after_status) == before_driver,
            "invalid command matrix changed driver lifecycle/provenance")
    after_profile = parse_profile(cli.exchange("profile", "profile locked"))
    require(after_profile == before_profile,
            "invalid profile command mutated the staged profile")
    return checks


def parse_sample(
    result: JobResult, expected_mask: int, ready: bool
) -> dict[str, int | str]:
    require_success(result, "sample")
    match = SAMPLE_RE.search(result.output)
    if match is None:
        raise HilFailure(f"missing converted sample in {result.output!r}")
    sample = {
        "sequence": int(match.group(1)),
        "generation": int(match.group(2)),
        "valid": int(match.group(3), 16),
        "fresh": int(match.group(4), 16),
        "read_ms": int(match.group(5)),
        "quality": match.group(6),
        "xl_fs_g": match.group(7),
        "g_fs_dps": match.group(8),
    }
    require(sample["valid"] == expected_mask, f"wrong valid mask in {result.command}")
    require(sample["generation"] > 0, f"missing sample generation in {result.command}")
    require(result.started_ms <= sample["read_ms"] <= result.completed_ms,
            f"sample timestamp is outside its operation in {result.command}")
    if ready:
        require(sample["fresh"] == expected_mask, f"wrong fresh mask in {result.command}")
    else:
        require(sample["fresh"] == 0, f"direct sample claimed freshness in {result.command}")
    expected_quality = "ready_checked" if ready else "direct_unverified"
    require(sample["quality"] == expected_quality,
            f"wrong sample quality in {result.command}")
    require(sample["xl_fs_g"] == "2" and sample["g_fs_dps"] == "250",
            f"wrong default full-scale provenance in {result.command}")
    require(RAW_SAMPLE_RE.search(result.output) is not None,
            f"raw sample evidence missing in {result.command}")
    converted_records = (
        (0x01, r"accel_ug x=-?\d+ y=-?\d+ z=-?\d+", "acceleration"),
        (0x02, r"gyro_udps x=-?\d+ y=-?\d+ z=-?\d+", "angular rate"),
        (0x04, r"temperature_mC=-?\d+", "temperature"),
    )
    for mask, pattern, label in converted_records:
        if expected_mask & mask:
            require(re.search(pattern, result.output) is not None,
                    f"converted {label} evidence missing in {result.command}")
    return sample


def update_ranges(ranges: dict[str, list[int]], output: str) -> None:
    patterns = {
        "accel_x_ug": r"accel_ug x=(-?\d+)",
        "accel_y_ug": r"accel_ug x=-?\d+ y=(-?\d+)",
        "accel_z_ug": r"accel_ug x=-?\d+ y=-?\d+ z=(-?\d+)",
        "gyro_x_udps": r"gyro_udps x=(-?\d+)",
        "gyro_y_udps": r"gyro_udps x=-?\d+ y=(-?\d+)",
        "gyro_z_udps": r"gyro_udps x=-?\d+ y=-?\d+ z=(-?\d+)",
        "temperature_mC": r"temperature_mC=(-?\d+)",
    }
    for name, pattern in patterns.items():
        match = re.search(pattern, output)
        if match is None:
            continue
        value = int(match.group(1))
        bounds = ranges.setdefault(name, [value, value])
        bounds[0] = min(bounds[0], value)
        bounds[1] = max(bounds[1], value)
        if name.startswith("accel_"):
            require(abs(value) <= 2_100_000, f"acceleration outside configured full scale: {value}")
        elif name.startswith("gyro_"):
            require(abs(value) <= 251_000_000, f"gyro outside configured full scale: {value}")
        else:
            require(-40_000 <= value <= 85_000, f"temperature outside rating: {value}")


def run_targeted(
    cli: SerialCli,
    summary: dict[str, object],
    expected_flash_bytes: int = EXPECTED_FLASH_BYTES,
    expected_psram_bytes: int = EXPECTED_PSRAM_BYTES,
    run_accel_calibration: bool = True,
) -> None:
    print("HIL targeted: metadata and startup", flush=True)
    version = cli.exchange("version", "platform arduino=")
    expected_banner = f"LSM6DS3TR {EXPECTED_LIBRARY_VERSION}"
    require(
        expected_banner in version,
        f"wrong firmware/library version: {version}",
    )
    platform_match = PLATFORM_RE.search(version)
    require(platform_match is not None, "missing platform version metadata")
    assert platform_match is not None
    require(
        platform_match.group(1) == EXPECTED_ARDUINO_CORE,
        f"wrong Arduino core version {platform_match.group(1)}",
    )
    require(
        platform_match.group(2).startswith(EXPECTED_ESP_IDF),
        f"wrong ESP-IDF version {platform_match.group(2)}",
    )
    require(
        int(platform_match.group(3)) == expected_flash_bytes,
        f"wrong flash size {platform_match.group(3)}",
    )
    require(
        int(platform_match.group(4)) == expected_psram_bytes,
        f"PSRAM was not initialized: {platform_match.group(4)} bytes",
    )
    summary["platform"] = {
        "arduino": platform_match.group(1),
        "esp_idf": platform_match.group(2),
        "flash_bytes": int(platform_match.group(3)),
        "psram_bytes": int(platform_match.group(4)),
    }
    cli.exchange("help", "All jobs use absolute deadlines")
    cli.exchange("?", "All jobs use absolute deadlines")
    cli.exchange("ver", "platform arduino=")
    status = cli.exchange("status", "transport ok=")
    require("bound=yes" in status, "driver was not bound at baseline")
    bus = parse_bus(status)
    require(bus[0], "application-owned I2C bus was not ready")
    bus_init = BUS_INIT_RE.search(status)
    require(bus_init is not None, "missing retained bus initialization status")
    assert bus_init is not None
    require(int(bus_init.group(1)) == 0,
            f"bus initialization failed: {bus_init.group(3)}")
    require(bus[1] == 0x6A and bus[2] == "0x6A", "wrong initial address")
    require(bus[3] == 400000, "wrong initial I2C frequency")
    require(bus[4] == 50, "wrong transport timeout")
    require(LAST_ERROR_RE.search(status) is not None, "full last-error diagnostics missing")
    require(MISMATCH_RE.search(status) is not None, "mismatch diagnostics missing")
    baseline_transport = parse_transport(status)
    require("address selected=0x6A bound=yes" in
            cli.exchange("addr", "address selected="),
            "address query was incomplete")
    require("frequency_hz=400000 bus_ready=yes" in
            cli.exchange("freq", "frequency_hz="),
            "frequency query was incomplete")
    cli.exchange("cfg", "profile locked")
    cli.exchange("settings", "profile locked")

    print("HIL targeted: application-owned bus scan", flush=True)
    scan_output = cli.exchange("scan", "scan summary", timeout_s=3.0)
    scan_match = SCAN_SUMMARY_RE.search(scan_output)
    require(scan_match is not None, "scan summary malformed")
    assert scan_match is not None
    require(int(scan_match.group(1)) == 2, "scan did not attempt both SA0 addresses")
    require(int(scan_match.group(2)) == 1, "scan did not find exactly the fixture address")
    require(int(scan_match.group(3)) == 0, "scan reported a bus failure")
    require("scan address=0x6A ack=yes" in scan_output, "0x6A did not ACK scan")
    require("scan address=0x6B ack=no" in scan_output, "0x6B scan result missing")

    print("HIL targeted: lifecycle, identity, busy, and cancellation", flush=True)
    cli.exchange("unbind", "unbound (zero I2C)")
    require("bound=no" in cli.exchange("status", "transport ok="), "unbind failed")
    unbound_probe = cli.exchange("probe", "status=")
    require(parse_status_code(unbound_probe) != 0, "unbound probe reported success")
    require("accepted token=" not in unbound_probe, "unbound probe was accepted")
    require(parse_status_code(cli.exchange("bind", "status=")) == 0, "bind failed")
    before_rebind = parse_transport(cli.exchange("status", "transport ok="))
    require(parse_status_code(cli.exchange("bind", "status=")) == 0,
            "same-binding no-op failed")
    after_rebind = parse_transport(cli.exchange("status", "transport ok="))
    require(after_rebind == before_rebind, "same-binding no-op performed I2C")
    probe = cli.job("probe")
    require_success(probe, "probe")
    require("address=0x6A who_am_i=0x6A" in probe.output, "identity/address mismatch")
    configure = cli.job("configure")
    require_success(configure, "configure")

    print("HIL targeted: runtime frequency and address ownership", flush=True)
    require(parse_status_code(cli.exchange("freq 100000", "configuration_preserved=yes")) == 0,
            "100 kHz change failed")
    require(parse_bus(cli.exchange("diag", "mismatch present="))[3] == 100000,
            "100 kHz selection was not reported")
    require_success(cli.job("sample all ready"), "sample")
    require(parse_status_code(cli.exchange("freq 400000", "configuration_preserved=yes")) == 0,
            "400 kHz restore failed")
    require_success(cli.job("sample all ready"), "sample")
    require(parse_status_code(cli.exchange("addr 0x6b", "profile_apply_required=yes")) == 0,
            "0x6B selection failed")
    absent_probe = cli.job("probe")
    require(absent_probe.state == "failed" and absent_probe.status_code != 0,
            "absent 0x6B probe did not fail explicitly")
    failed_diag = cli.exchange("diag", "mismatch present=")
    require_last_transport_error(failed_diag)
    require(parse_status_code(cli.exchange("addr 0x6a", "profile_apply_required=yes")) == 0,
            "0x6A restore failed")
    require(parse_transport(cli.exchange("status", "transport ok="))[1] == 0,
            "address rebind did not reset passive diagnostics")
    require_success(cli.job("probe"), "probe")
    require_success(cli.job("configure"), "configure")
    no_cancel = cli.exchange("cancel", "status=")
    require(parse_status_code(no_cancel) != 0, "idle cancel reported success")

    cli.send("selftest 100")
    cli.send("job current")
    cli.send("sample all ready")
    cli.send("cancel")
    busy_cancel = cli.read_until(
        lambda text: "kind=selftest state=cancelled" in text and "status=" in text,
        8.0,
    )
    require("accepted token=" in busy_cancel, "self-test was not admitted")
    require("job driver bound=yes active=yes" in busy_cancel,
            "active job progress was not observable")
    require("job poll token=" in busy_cancel and "transactions=" in busy_cancel,
            "poll transaction progress was not exposed")
    require("Operation is active" in busy_cancel, "busy request was not rejected")
    require("message=Operation cancelled" in busy_cancel, "cancellation was not terminal")
    require("changed=yes" in busy_cancel, "partial self-test effect was not reported")
    cached_result = cli.exchange("result", "job last available=yes")
    require("kind=selftest state=cancelled" in cached_result,
            "last taken result was not retained by the CLI")
    job_last = cli.exchange("job last", "job last available=yes")
    require("kind=selftest state=cancelled" in job_last,
            "job last did not expose the cached terminal result")
    cancelled_status = cli.exchange("status", "transport ok=")
    require("config=unknown" in cancelled_status,
            "cancelled changed operation retained false provenance")
    require_success(cli.job("configure"), "configure")

    print("HIL targeted: staged typed profile controls and verified application", flush=True)
    initial_profile = parse_profile(cli.exchange("profile", "profile locked"))
    require(initial_profile["xl_odr"] == "104" and initial_profile["g_odr"] == "104",
            "unexpected profile defaults")
    require(parse_status_code(cli.exchange("profile validate", "status=")) == 0,
            "valid staged profile failed validation")
    for command in (
        "profile set xl_odr 208",
        "profile set xl_fs 4",
        "profile set xl_power lp",
        "profile set g_fs 500",
        "profile set xl_lpf2 1",
        "profile set xl_slope_hp 0",
        "profile set xl_6d_lpf off",
        "profile set g_odr 208",
        "profile set g_power lp",
        "profile set g_power hp",
        "profile set g_lpf1 on",
        "profile set g_hpf false",
        "profile set g_hpf_mode 0.260",
        "profile set g_sleep off",
        "profile set bdu true",
        "profile set offset_weight 16",
        "profile set offset 1 -2 3",
        "profile set fifo 0",
        "profile set interrupts false",
    ):
        require(parse_status_code(cli.exchange(command, "configure_required=yes")) == 0,
                f"profile setter failed: {command}")
    staged = parse_profile(cli.exchange("profile show", "profile locked"))
    require(staged["xl_fs_g"] == "4" and staged["g_fs_dps"] == "500",
            "full-scale profile setters were not staged")
    require(staged["xl_odr"] == "208" and staged["g_odr"] == "208" and
            staged["xl_power"] == "lp" and staged["g_power"] == "hp",
            "ODR/power profile setters were not staged")
    require(staged["xl_lpf2"] == "on" and staged["g_lpf1"] == "on",
            "filter profile setters were not staged")
    require(staged["g_hpf_mode"] == "0.260", "HPF mode field was not staged")
    require(staged["offset_weight_mg"] == "16" and staged["offset_x"] == "1" and
            staged["offset_y"] == "-2" and staged["offset_z"] == "3",
            "offset profile setters were not staged")
    require(staged["xl_slope_hp"] == "off" and staged["xl_6d_lpf"] == "off" and
            staged["g_hpf"] == "off" and staged["bdu"] == "on" and
            staged["fifo"] == "off" and staged["interrupts"] == "off",
            "fixed production-invariant setters were not staged safely")
    require_success(cli.job("profile apply"), "configure")
    applied = cli.exchange("profile", "profile locked")
    require("profile verified available=yes matches_staged=yes" in applied,
            "applied profile did not become verified")
    require_success(cli.job("sample all ready"), "sample")

    cli.exchange("profile defaults", "profile defaults staged")
    for command in (
        "profile set xl_odr 208",
        "profile set xl_power lp",
        "profile set xl_odr 1.6",
    ):
        require(parse_status_code(cli.exchange(command, "configure_required=yes")) == 0,
                f"valid low-power transition failed: {command}")
    low_power = parse_profile(cli.exchange("profile", "profile locked"))
    require(low_power["xl_odr"] == "1.6" and low_power["xl_power"] == "lp",
            "1.6 Hz low-power profile was not staged")
    rejected_transition = cli.exchange("profile set xl_power hp", "status=")
    require(parse_status_code(rejected_transition) != 0,
            "invalid 1.6 Hz high-performance transition was accepted")
    unchanged = parse_profile(cli.exchange("profile", "profile locked"))
    require(unchanged["xl_odr"] == "1.6" and unchanged["xl_power"] == "lp",
            "rejected cross-field update partially mutated the profile")
    cli.exchange("profile defaults", "profile defaults staged")
    require_success(cli.job("profile apply"), "configure")

    print("HIL targeted: every sample quantity and readiness mode", flush=True)
    ranges: dict[str, list[int]] = {}
    previous_sequence = 0
    for quantity, mask in (("all", 0x07), ("accel", 0x01), ("gyro", 0x02), ("temp", 0x04)):
        for mode in ("ready", "direct"):
            result = cli.job(f"sample {quantity} {mode}")
            sample = parse_sample(result, mask, mode == "ready")
            require(sample["sequence"] > previous_sequence, "sample sequence did not advance")
            previous_sequence = sample["sequence"]
            update_ranges(ranges, result.output)

    print("HIL targeted: cooperative stress sessions and cancellation", flush=True)
    stress_ready = cli.exchange("stress 3 all ready", "stress summary", timeout_s=12.0)
    summary["stress_ready"] = require_stress_success(stress_ready, "stress", 3)
    stress_direct = cli.exchange("stress 4 accel direct", "stress summary", timeout_s=12.0)
    summary["stress_direct"] = require_stress_success(stress_direct, "stress", 4)
    default_count = cli.exchange("stress accel ready", "stress summary", timeout_s=20.0)
    summary["stress_default_count"] = require_stress_success(
        default_count, "stress", 100
    )
    mixed = cli.exchange("stress_mix 8", "stress_mix summary", timeout_s=20.0)
    mixed_summary = require_stress_success(mixed, "stress_mix", 8)
    require(mixed_summary["probes"] == 2 and mixed_summary["reconciles"] == 2 and
            mixed_summary["samples"] == 4,
            "stress_mix did not execute the deterministic non-destructive rotation")
    summary["stress_mix"] = mixed_summary

    cli.send("stress 10000 temp ready")
    cli.send("job current")
    cli.send("cancel")
    cancelled_stress = cli.read_until(
        lambda text: "stress summary" in text and "cancelled=yes" in text,
        8.0,
    )
    cancelled_summary = parse_stress_summary(cancelled_stress, "stress")
    require(cancelled_summary["cancelled"], "stress cancel did not stop the session")
    require(cancelled_summary["completed"] < cancelled_summary["requested"],
            "cancelled stress unexpectedly ran to completion")
    require(cancelled_summary["fail"] == 0,
            "cancellation was incorrectly counted as a stress failure")
    require("job session=stress" in cancelled_stress,
            "stress job progress was not observable")
    require_success(cli.job("sample all ready"), "sample")

    print("HIL targeted: diagnostics, invalidation, and reconciliation", flush=True)
    who = cli.exchange("rreg 0x0f", "0x0F = 0x6A")
    require(parse_status_code(who) == 0, "WHO_AM_I diagnostic read failed")
    dump = cli.exchange("dump 0x20 14", "status=0")
    require(len(re.findall(r"\b[0-9A-F]{2}\b", dump.splitlines()[-1])) == 14,
            "diagnostic dump length mismatch")
    ctrl = cli.exchange("rreg 0x10", "0x10 =")
    ctrl_match = re.search(r"0x10 = 0x([0-9A-Fa-f]{2})", ctrl)
    require(ctrl_match is not None, "CTRL1_XL read did not return a byte")
    ctrl_value = int(ctrl_match.group(1), 16)
    changed_ctrl = ctrl_value ^ 0x04
    require(parse_status_code(cli.exchange(f"wreg 0x10 0x{changed_ctrl:02x}", "status=")) == 0,
            "safe diagnostic config write failed")
    invalidated = cli.exchange("status", "transport ok=")
    require("config=unknown" in invalidated, "diagnostic write did not invalidate provenance")
    gated = cli.exchange("sample all direct", "status=")
    require(parse_status_code(gated) != 0 and "accepted token=" not in gated,
            "sample bypassed unknown-configuration gate")
    mismatched = cli.job("reconcile")
    require(mismatched.kind == "reconcile" and mismatched.state == "failed" and
            mismatched.status_code != 0,
            "reconcile did not report the injected register mismatch")
    mismatch_diag = cli.exchange("diag", "mismatch present=")
    mismatch_match = MISMATCH_RE.search(mismatch_diag)
    require(mismatch_match is not None and mismatch_match.group(1) == "yes",
            "register mismatch evidence was not exposed")
    assert mismatch_match is not None
    require(int(mismatch_match.group(2), 16) == 0x10,
            "wrong mismatch register evidence")
    require(int(mismatch_match.group(3), 16) == ctrl_value and
            int(mismatch_match.group(4), 16) == changed_ctrl,
            "mismatch expected/observed evidence was incorrect")
    require_success(cli.job("configure"), "configure")
    cleared_mismatch = MISMATCH_RE.search(cli.exchange("diag", "mismatch present="))
    require(cleared_mismatch is not None and cleared_mismatch.group(1) == "no",
            "successful configure did not clear mismatch evidence")

    print("HIL targeted: maintenance and complete lifecycle procedures", flush=True)
    self_test = cli.job("selftest")
    summary["self_test"] = {
        "state": self_test.state,
        "status": self_test.status_code,
        "message": self_test.status_message,
        "details": self_test.output.splitlines()[-1] if self_test.output else "",
    }
    require_success(self_test, "selftest")
    require("restore_code=0" in self_test.output,
            "self-test did not restore configuration")

    calibration_commands = [
        ("calg_default", "calg"),
        ("calg_custom", "calg 16 5"),
    ]
    if run_accel_calibration:
        calibration_commands.extend((
            ("calxl_default", "calxl"),
            ("calxl_custom", "calxl 16 0 0 1 0.1"),
        ))
    else:
        summary["calxl_default"] = {
            "skipped": True,
            "reason": "validated +Z gravity fixture not supplied",
        }
        summary["calxl_custom"] = {
            "skipped": True,
            "reason": "validated +Z gravity fixture not supplied",
        }
    for key, command in calibration_commands:
        calibration = cli.job(command)
        summary[key] = {
            "state": calibration.state,
            "status": calibration.status_code,
            "message": calibration.status_message,
            "output": calibration.output.splitlines()[-1] if calibration.output else "",
        }
        require_success(calibration, "calibration")
    purge = cli.job("purge 1")
    require_success(purge, "purge")
    require("discarded=0" in purge.output, "bypass FIFO unexpectedly contained data")

    for command, kind in (
        ("powerdown", "powerdown"),
        ("configure", "configure"),
        ("reset", "reset"),
        ("boot", "boot"),
        ("recover", "recover"),
        ("reconcile", "reconcile"),
    ):
        result = cli.job(command)
        require_success(result, kind)
        if command == "powerdown":
            powered_down = cli.exchange("status", "transport ok=")
            require("config=unconfigured" in powered_down,
                    "powerdown did not publish confirmed unconfigured state")
            gated = cli.exchange("sample all direct", "status=")
            require(parse_status_code(gated) != 0, "powerdown did not gate sampling")

    final_status = cli.exchange("status", "transport ok=")
    summary["targeted_transport_before"] = baseline_transport
    final_transport = parse_transport(final_status)
    require(final_transport[1] == 0, "targeted campaign recorded transport failures")
    summary["targeted_transport_after"] = final_transport
    summary["targeted_ranges"] = ranges
    summary["targeted_last_sequence"] = previous_sequence
    required_ranges = {
        "accel_x_ug", "accel_y_ug", "accel_z_ug", "gyro_x_udps",
        "gyro_y_udps", "gyro_z_udps", "temperature_mC",
    }
    require(required_ranges.issubset(ranges),
            "targeted samples did not publish every converted physical range")

    print("HIL targeted: strict invalid-input matrix", flush=True)
    summary["invalid_input_checks"] = run_invalid_matrix(cli)


def port_present(name: str) -> bool:
    if list_ports is None:
        return False
    return any(info.device.upper() == name.upper() for info in list_ports.comports())


def esptool_command() -> list[str]:
    configured_core = os.environ.get("PLATFORMIO_CORE_DIR")
    if configured_core:
        core_dir = pathlib.Path(configured_core)
    else:
        user_profile = pathlib.Path(os.environ.get("USERPROFILE", pathlib.Path.home()))
        core_dir = user_profile / ".platformio"

    if os.name == "nt":
        scripts_dir = core_dir / "penv" / "Scripts"
        executable = scripts_dir / "esptool.exe"
        python = scripts_dir / "python.exe"
    else:
        scripts_dir = core_dir / "penv" / "bin"
        executable = scripts_dir / "esptool"
        python = scripts_dir / "python"
    if executable.is_file():
        return [str(executable)]

    for package_root in ("packages", "tools"):
        script = core_dir / package_root / "tool-esptoolpy" / "esptool.py"
        if python.is_file() and script.is_file():
            return [str(python), str(script)]
    raise HilFailure(
        "PlatformIO esptool was not found; check PLATFORMIO_CORE_DIR or omit "
        "--watchdog-reset"
    )


def watchdog_reset(port: str, raw_log: pathlib.Path, chip: str) -> None:
    command = esptool_command() + [
        "--chip", chip, "--port", port, "--baud", "115200",
        "--before", "default-reset", "--after", "watchdog-reset",
        "--no-stub", "chip-id",
    ]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=20, check=False)
    with raw_log.open("a", encoding="utf-8") as handle:
        handle.write(completed.stdout)
        handle.write(completed.stderr)
    evidence = completed.stdout + completed.stderr
    reset_reported = "Hard resetting with a watchdog" in evidence
    known_windows_disconnect = (
        os.name == "nt" and "A serial exception error occurred" in evidence
    )
    chip_label = "ESP32-S2" if chip == "esp32s2" else "ESP32-S3"
    if (chip_label not in evidence or not reset_reported or
            (completed.returncode != 0 and not known_windows_disconnect)):
        raise HilFailure(f"watchdog reset did not execute: {evidence}")
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if port_present(port):
            # Windows publishes the ESP32 native USB COM name before the
            # replacement endpoint is fully stable. Opening during that window
            # can lose later packets even though the first command succeeds.
            time.sleep(2.5)
            if not port_present(port):
                continue
            return
        time.sleep(0.1)
    raise HilFailure(f"{port} did not re-enumerate after watchdog reset")


def default_path(name: str) -> pathlib.Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    requested = pathlib.Path(name)
    return pathlib.Path(tempfile.gettempdir()) / (
        f"lsm6ds3tr_{requested.stem}_{stamp}{requested.suffix}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--chip", choices=("esp32s2", "esp32s3"), default="esp32s3")
    parser.add_argument(
        "--assert-dtr",
        action="store_true",
        help="assert DTR while open (required by the ESP32-S2 TinyUSB fixture)",
    )
    parser.add_argument(
        "--expected-flash-bytes", type=int, default=EXPECTED_FLASH_BYTES
    )
    parser.add_argument(
        "--expected-psram-bytes", type=int, default=EXPECTED_PSRAM_BYTES
    )
    parser.add_argument(
        "--skip-accel-calibration",
        action="store_true",
        help=(
            "skip accelerometer calibration when no validated +Z gravity "
            "fixture is available; the summary records the omitted coverage"
        ),
    )
    parser.add_argument("--watchdog-reset", action="store_true")
    parser.add_argument("--raw-log", type=pathlib.Path)
    parser.add_argument("--summary", type=pathlib.Path)
    return parser.parse_args()


def main() -> int:
    if serial is None:
        raise SystemExit(
            "pyserial is required: python -m pip install pyserial"
        ) from SERIAL_IMPORT_ERROR
    args = parse_args()
    raw_log = args.raw_log or default_path("hil_raw.log")
    summary_path = args.summary or default_path("hil_summary.json")
    raw_log.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    raw_log.write_text("", encoding="utf-8")
    summary: dict[str, object] = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "port": args.port,
        "baud": args.baud,
        "chip": args.chip,
        "assert_dtr": args.assert_dtr,
        "expected_flash_bytes": args.expected_flash_bytes,
        "expected_psram_bytes": args.expected_psram_bytes,
        "skip_accel_calibration": args.skip_accel_calibration,
        "raw_log": str(raw_log.resolve()),
        "result": "running",
    }
    cli: SerialCli | None = None
    try:
        if args.watchdog_reset:
            watchdog_reset(args.port, raw_log, args.chip)
        cli = SerialCli(args.port, args.baud, raw_log, args.assert_dtr)
        try:
            cli.drain(2.0)
        except HilFailure:
            pass
        run_targeted(
            cli,
            summary,
            args.expected_flash_bytes,
            args.expected_psram_bytes,
            not args.skip_accel_calibration,
        )
        summary["result"] = "passed"
        print("HIL campaign PASSED", flush=True)
        return_code = 0
    except (HilFailure, SERIAL_EXCEPTION, subprocess.SubprocessError) as error:
        summary["result"] = "failed"
        summary["failure"] = str(error)
        print(f"HIL campaign FAILED: {error}", file=sys.stderr, flush=True)
        return_code = 1
    finally:
        summary["completed_utc"] = datetime.now(timezone.utc).isoformat()
        if cli is not None:
            cli.close()
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"raw log: {raw_log.resolve()}", flush=True)
        print(f"summary: {summary_path.resolve()}", flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
