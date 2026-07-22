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
    raise SystemExit("pyserial is required: python -m pip install pyserial") from error


RESULT_RE = re.compile(
    r"result token=(\d+) kind=(\w+) state=(\w+) "
    r"transactions=(\d+)/(\d+) changed=(yes|no)"
)
SAMPLE_RE = re.compile(
    r"sample sequence=(\d+) generation=(\d+) valid=0x([0-9A-Fa-f]+) "
    r"fresh=0x([0-9A-Fa-f]+) read_ms=(\d+)"
)
STATUS_CODE_RE = re.compile(r"status=(\d+) detail=(-?\d+) message=([^\r\n]+)")
TRANSPORT_RE = re.compile(r"transport ok=(\d+) fail=(\d+) last_error=(\d+)")


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
    output: str


class SerialCli:
    def __init__(self, port: str, baud: int, raw_log: pathlib.Path) -> None:
        self.port = port
        self.baud = baud
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
        endpoint.dtr = False
        endpoint.rts = False
        endpoint.open()
        self.serial = endpoint
        self.log("HOST", f"opened {self.port} baud={self.baud} dtr=0 rts=0")

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

        def terminal(text: str) -> bool:
            nonlocal token
            accepted = re.search(r"accepted token=(\d+)", text)
            if accepted is not None:
                token = int(accepted.group(1))
            if token is None or re.search(
                rf"result token={token}\b.*\nstatus=", text
            ) is None:
                return False
            result = RESULT_RE.search(text)
            if result is None:
                return False
            kind = result.group(2)
            state = result.group(3)
            if kind == "sample" and state == "succeeded":
                return SAMPLE_RE.search(text) is not None
            if kind == "probe" and state == "succeeded":
                return "who_am_i=" in text
            if kind in ("configure", "recover", "reconcile") and state == "succeeded":
                return "valid_after_ms=" in text
            if kind == "selftest":
                return "restore=" in text
            if kind == "calibration" and state == "succeeded":
                return "samples=" in text
            if kind == "purge":
                return "truncated=" in text
            return True

        output = self.read_until(terminal, timeout_s)
        if token is None:
            raise HilFailure(f"{command!r} was not accepted: {output!r}")
        result_match = RESULT_RE.search(output)
        status_matches = list(STATUS_CODE_RE.finditer(output))
        if result_match is None or not status_matches:
            raise HilFailure(f"malformed terminal result for {command!r}: {output!r}")
        if int(result_match.group(1)) != token:
            raise HilFailure(f"result identity mismatch for {command!r}")
        return JobResult(
            command=command,
            token=token,
            kind=result_match.group(2),
            state=result_match.group(3),
            transactions=int(result_match.group(4)),
            transaction_limit=int(result_match.group(5)),
            changed=result_match.group(6) == "yes",
            status_code=int(status_matches[-1].group(1)),
            status_message=status_matches[-1].group(3).strip(),
            output=output,
        )


def require(condition: bool, message: str) -> None:
    if not condition:
        raise HilFailure(message)


def require_success(result: JobResult, expected_kind: str) -> None:
    require(result.kind == expected_kind, f"{result.command}: wrong kind {result.kind}")
    require(result.state == "succeeded", f"{result.command}: state={result.state}")
    require(result.status_code == 0, f"{result.command}: {result.status_message}")
    require(
        result.transactions <= result.transaction_limit,
        f"{result.command}: transaction limit exceeded",
    )


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


def command_rejected(cli: SerialCli, command: str, expected: str) -> None:
    output = cli.exchange(command, expected)
    require("accepted token=" not in output, f"invalid command was accepted: {command}")


def run_invalid_matrix(cli: SerialCli) -> int:
    checks = 0
    zero_argument = (
        "help", "version", "status", "bind", "unbind", "cancel", "probe",
        "configure", "reset", "boot", "recover", "reconcile", "powerdown",
    )
    for command in zero_argument:
        command_rejected(cli, f"{command} extra", f"expected {command}")
        checks += 1
    cases = (
        ("sample nope", "expected sample"),
        ("sample all nope", "expected sample"),
        ("sample all ready extra", "expected sample"),
        ("selftest 4", "expected selftest"),
        ("selftest 101", "expected selftest"),
        ("selftest 5 extra", "expected selftest"),
        ("calxl 0", "expected calibration"),
        ("calxl 1001", "expected calibration"),
        ("calxl 5 extra", "expected calibration"),
        ("calg 0", "expected calibration"),
        ("calg 1001", "expected calibration"),
        ("calg 5 extra", "expected calibration"),
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
    )
    for command, expected in cases:
        command_rejected(cli, command, expected)
        checks += 1
    command_rejected(cli, "not-a-command", "unknown command")
    checks += 1
    cli.send("x" * 140)
    overflow = cli.read_until(lambda text: "input line too long; discarded" in text, 2.0)
    require("unknown command" not in overflow, "overflow tail was parsed as a command")
    return checks + 1


def parse_sample(result: JobResult, expected_mask: int, ready: bool) -> dict[str, int]:
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
    }
    require(sample["valid"] == expected_mask, f"wrong valid mask in {result.command}")
    if ready:
        require(sample["fresh"] == expected_mask, f"wrong fresh mask in {result.command}")
    else:
        require(sample["fresh"] == 0, f"direct sample claimed freshness in {result.command}")
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


def run_targeted(cli: SerialCli, summary: dict[str, object]) -> None:
    print("HIL targeted: metadata and startup", flush=True)
    version = cli.exchange("version", "LSM6DS3TR 2.0.0")
    require("LSM6DS3TR 2.0.0" in version, "wrong firmware/library version")
    cli.exchange("help", "All jobs use absolute deadlines")
    status = cli.exchange("status", "transport ok=")
    require("bound=yes" in status, "driver was not bound at baseline")
    baseline_transport = parse_transport(status)

    print("HIL targeted: lifecycle, identity, busy, and cancellation", flush=True)
    cli.exchange("unbind", "unbound (zero I2C)")
    require("bound=no" in cli.exchange("status", "transport ok="), "unbind failed")
    unbound_probe = cli.exchange("probe", "status=")
    require(parse_status_code(unbound_probe) != 0, "unbound probe reported success")
    require("accepted token=" not in unbound_probe, "unbound probe was accepted")
    require(parse_status_code(cli.exchange("bind", "status=")) == 0, "bind failed")
    before_rebind = parse_transport(cli.exchange("status", "transport ok="))
    require(parse_status_code(cli.exchange("bind", "status=")) == 0,
            "idempotent duplicate bind failed")
    after_rebind = parse_transport(cli.exchange("status", "transport ok="))
    require(after_rebind == before_rebind, "idempotent duplicate bind performed I2C")
    probe = cli.job("probe")
    require_success(probe, "probe")
    require("address=0x6A who_am_i=0x6A" in probe.output, "identity/address mismatch")
    configure = cli.job("configure")
    require_success(configure, "configure")
    no_cancel = cli.exchange("cancel", "status=")
    require(parse_status_code(no_cancel) != 0, "idle cancel reported success")

    cli.send("selftest 100")
    cli.send("sample all ready")
    cli.send("cancel")
    busy_cancel = cli.read_until(
        lambda text: "kind=selftest state=cancelled" in text and "status=" in text,
        8.0,
    )
    require("accepted token=" in busy_cancel, "self-test was not admitted")
    require("Operation is active" in busy_cancel, "busy request was not rejected")
    require("message=Operation cancelled" in busy_cancel, "cancellation was not terminal")
    require("changed=yes" in busy_cancel, "partial self-test effect was not reported")
    cancelled_status = cli.exchange("status", "transport ok=")
    require("config=unknown" in cancelled_status,
            "cancelled changed operation retained false provenance")
    require_success(cli.job("configure"), "configure")

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

    print("HIL targeted: diagnostics, invalidation, and reconciliation", flush=True)
    who = cli.exchange("rreg 0x0f", "0x0F = 0x6A")
    require(parse_status_code(who) == 0, "WHO_AM_I diagnostic read failed")
    dump = cli.exchange("dump 0x20 14", "status=0")
    require(len(re.findall(r"\b[0-9A-F]{2}\b", dump.splitlines()[-1])) == 14,
            "diagnostic dump length mismatch")
    ctrl = cli.exchange("rreg 0x12", "0x12 =")
    ctrl_match = re.search(r"0x12 = 0x([0-9A-Fa-f]{2})", ctrl)
    require(ctrl_match is not None, "CTRL3_C read did not return a byte")
    ctrl_value = int(ctrl_match.group(1), 16)
    require(parse_status_code(cli.exchange(f"wreg 0x12 0x{ctrl_value:02x}", "status=")) == 0,
            "idempotent diagnostic write failed")
    invalidated = cli.exchange("status", "transport ok=")
    require("config=unknown" in invalidated, "diagnostic write did not invalidate provenance")
    gated = cli.exchange("sample all direct", "status=")
    require(parse_status_code(gated) != 0 and "accepted token=" not in gated,
            "sample bypassed unknown-configuration gate")
    require_success(cli.job("reconcile"), "reconcile")

    print("HIL targeted: maintenance and complete lifecycle procedures", flush=True)
    self_test = cli.job("selftest 5")
    summary["self_test"] = {
        "state": self_test.state,
        "status": self_test.status_code,
        "message": self_test.status_message,
        "details": self_test.output.splitlines()[-1] if self_test.output else "",
    }
    # A fixture or individual device can legitimately fail the stimulus limits,
    # but the procedure must restore a known configuration before we continue.
    require("restore=0" in self_test.output, "self-test did not restore configuration")

    for command in ("calg 16", "calxl 16"):
        calibration = cli.job(command)
        summary[command.split()[0]] = {
            "state": calibration.state,
            "status": calibration.status_code,
            "message": calibration.status_message,
            "output": calibration.output.splitlines()[-1] if calibration.output else "",
        }
        require(calibration.state in ("succeeded", "failed"), "calibration did not terminate")
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
    summary["targeted_transport_after"] = parse_transport(final_status)
    summary["targeted_ranges"] = ranges
    summary["targeted_last_sequence"] = previous_sequence

    print("HIL targeted: strict invalid-input matrix", flush=True)
    summary["invalid_input_checks"] = run_invalid_matrix(cli)


def port_present(name: str) -> bool:
    return any(info.device.upper() == name.upper() for info in list_ports.comports())


def watchdog_reset(port: str, raw_log: pathlib.Path) -> None:
    user_profile = pathlib.Path(os.environ.get("USERPROFILE", pathlib.Path.home()))
    pio_python = user_profile / ".platformio" / "penv" / "Scripts" / "python.exe"
    esptool = user_profile / ".platformio" / "packages" / "tool-esptoolpy" / "esptool.py"
    if not pio_python.exists() or not esptool.exists():
        raise HilFailure(
            "PlatformIO esptool was not found; omit --watchdog-reset or install PlatformIO"
        )
    command = [
        str(pio_python), str(esptool), "--chip", "esp32s3", "--port", port,
        "--baud", "115200", "--before", "default_reset", "--after",
        "watchdog_reset", "--no-stub", "chip_id",
    ]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=20, check=False)
    with raw_log.open("a", encoding="utf-8") as handle:
        handle.write(completed.stdout)
        handle.write(completed.stderr)
    evidence = completed.stdout + completed.stderr
    if completed.returncode != 0 or "Hard resetting with a watchdog" not in evidence:
        raise HilFailure(f"watchdog reset did not execute: {evidence}")
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if port_present(port):
            # Windows publishes the ESP32-S3 native USB COM name before the
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
    parser.add_argument("--port", default="COM26")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--watchdog-reset", action="store_true")
    parser.add_argument("--raw-log", type=pathlib.Path)
    parser.add_argument("--summary", type=pathlib.Path)
    return parser.parse_args()


def main() -> int:
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
        "raw_log": str(raw_log.resolve()),
        "result": "running",
    }
    cli: SerialCli | None = None
    try:
        if args.watchdog_reset:
            watchdog_reset(args.port, raw_log)
        cli = SerialCli(args.port, args.baud, raw_log)
        try:
            cli.drain(2.0)
        except HilFailure:
            pass
        run_targeted(cli, summary)
        summary["result"] = "passed"
        print("HIL campaign PASSED", flush=True)
        return_code = 0
    except (HilFailure, serial.SerialException, subprocess.SubprocessError) as error:
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
