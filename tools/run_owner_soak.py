#!/usr/bin/env python3
"""Capture and validate the low-output ESP32-S3 owner-soak firmware."""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone

try:
    import serial
except ImportError as error:  # pragma: no cover - depends on the HIL host
    raise SystemExit("pyserial is required: python -m pip install pyserial") from error

from run_hil import HilFailure, watchdog_reset


RESULT_RE = re.compile(
    r"HIL_SOAK_(PROGRESS|PASS|FAIL) elapsed_ms=(\d+) samples=(\d+) "
    r"sequence=(\d+) generation=(\d+) operation_failures=(\d+) "
    r"contract_failures=(\d+) transport_ok=(\d+) transport_fail=(\d+) "
    r"probes=(\d+) reconciles=(\d+) temp_min_mC=(-?\d+) "
    r"temp_max_mC=(-?\d+) accel_abs_max_ug=(\d+) gyro_abs_max_udps=(\d+)"
)


def default_path(name: str) -> pathlib.Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    requested = pathlib.Path(name)
    return pathlib.Path(tempfile.gettempdir()) / (
        f"lsm6ds3tr_{requested.stem}_{stamp}{requested.suffix}"
    )


def open_safe(port: str, baud: int) -> serial.Serial:
    endpoint = serial.Serial()
    endpoint.port = port
    endpoint.baudrate = baud
    endpoint.timeout = 0
    endpoint.write_timeout = 2.0
    endpoint.rtscts = False
    endpoint.dsrdtr = False
    endpoint.dtr = False
    endpoint.rts = False
    endpoint.open()
    return endpoint


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM26")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--expected-seconds", type=float, default=3600.0)
    parser.add_argument("--timeout-margin-seconds", type=float, default=120.0)
    parser.add_argument("--raw-log", type=pathlib.Path)
    parser.add_argument("--summary", type=pathlib.Path)
    args = parser.parse_args()
    if args.expected_seconds <= 0 or args.timeout_margin_seconds <= 0:
        parser.error("duration and timeout margin must be positive")
    return args


def main() -> int:
    args = parse_args()
    raw_path = args.raw_log or default_path("owner_soak_raw.log")
    summary_path = args.summary or default_path("owner_soak_summary.json")
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    raw_path.write_text("", encoding="utf-8")
    summary: dict[str, object] = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "port": args.port,
        "baud": args.baud,
        "expected_seconds": args.expected_seconds,
        "raw_log": str(raw_path.resolve()),
        "result": "running",
        "progress": [],
    }
    endpoint: serial.Serial | None = None
    raw = raw_path.open("a", encoding="utf-8", newline="\n")
    try:
        raw.close()
        watchdog_reset(args.port, raw_path)
        raw = raw_path.open("a", encoding="utf-8", newline="\n")
        endpoint = open_safe(args.port, args.baud)
        deadline = time.monotonic() + args.expected_seconds + args.timeout_margin_seconds
        last_record_at = time.monotonic()
        buffer = ""
        final: dict[str, int | str] | None = None
        while time.monotonic() < deadline:
            available = endpoint.in_waiting
            if available == 0:
                if time.monotonic() - last_record_at > 75.0:
                    raise HilFailure("no owner-soak progress for 75 seconds")
                time.sleep(0.02)
                continue
            chunk = endpoint.read(available).decode("utf-8", errors="replace")
            buffer += chunk.replace("\r", "")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if not line:
                    continue
                stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                raw.write(f"{stamp} RX {line}\n")
                raw.flush()
                print(line, flush=True)
                if (line.startswith("HIL_SOAK_ABORT") or
                        line.startswith("HIL_START_FAILURE") or
                        line.startswith("HIL_OPERATION_FAILURE") or
                        line.startswith("HIL_CONTRACT_FAILURE")):
                    raise HilFailure(line)
                match = RESULT_RE.fullmatch(line)
                if match is None:
                    continue
                record: dict[str, int | str] = {
                    "marker": match.group(1).lower(),
                    "elapsed_ms": int(match.group(2)),
                    "samples": int(match.group(3)),
                    "sequence": int(match.group(4)),
                    "generation": int(match.group(5)),
                    "operation_failures": int(match.group(6)),
                    "contract_failures": int(match.group(7)),
                    "transport_ok": int(match.group(8)),
                    "transport_fail": int(match.group(9)),
                    "probes": int(match.group(10)),
                    "reconciles": int(match.group(11)),
                    "temp_min_mC": int(match.group(12)),
                    "temp_max_mC": int(match.group(13)),
                    "accel_abs_max_ug": int(match.group(14)),
                    "gyro_abs_max_udps": int(match.group(15)),
                }
                progress = summary["progress"]
                assert isinstance(progress, list)
                if progress:
                    previous = progress[-1]
                    if (record["elapsed_ms"] <= previous["elapsed_ms"] or
                            record["samples"] <= previous["samples"] or
                            record["sequence"] <= previous["sequence"]):
                        raise HilFailure("non-monotonic owner-soak progress")
                progress.append(record)
                last_record_at = time.monotonic()
                if record["marker"] in ("pass", "fail"):
                    final = record
                    break
            if final is not None:
                break
        if final is None:
            raise HilFailure("owner-soak terminal record timed out")
        minimum_elapsed = int(args.expected_seconds * 1000.0)
        minimum_samples = int(args.expected_seconds * 5.0)
        minimum_maintenance = max(
            0, int((args.expected_seconds - 0.001) // 300.0)
        )
        if (final["marker"] != "pass" or final["elapsed_ms"] < minimum_elapsed or
                final["samples"] < minimum_samples or
                final["operation_failures"] != 0 or final["contract_failures"] != 0 or
                final["transport_fail"] != 0 or final["generation"] == 0 or
                final["transport_ok"] < final["samples"] or
                final["probes"] < minimum_maintenance or
                final["reconciles"] != final["probes"] or
                final["accel_abs_max_ug"] < 100_000):
            raise HilFailure(f"owner-soak terminal validation failed: {final}")
        summary["result"] = "passed"
        summary["final"] = final
        return_code = 0
        print("Owner-soak capture PASSED", flush=True)
    except (HilFailure, serial.SerialException, subprocess.SubprocessError) as error:
        summary["result"] = "failed"
        summary["failure"] = str(error)
        return_code = 1
        print(f"Owner-soak capture FAILED: {error}", file=sys.stderr, flush=True)
    finally:
        summary["completed_utc"] = datetime.now(timezone.utc).isoformat()
        if endpoint is not None:
            endpoint.close()
        if not raw.closed:
            raw.close()
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"raw log: {raw_path.resolve()}", flush=True)
        print(f"summary: {summary_path.resolve()}", flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
