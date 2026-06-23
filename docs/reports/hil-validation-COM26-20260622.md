# LSM6DS3TR HIL Validation - COM26 - 20260622

Date/time: 2026-06-22 20:17 to 2026-06-23 04:46, Europe/Prague (UTC+02:00)

This report records the HIL work performed on the connected fixture. It is evidence-based only; it does not claim production readiness.

## Setup

| Item | Value |
|---|---|
| Repository | `C:\Users\Honza\Documents\Projects\LSM6DS3TR` |
| Branch / commit | `main` / `f10e8e4502956792971acdec4e677f9f59ef2917` |
| Initial git status | Clean before HIL/tooling edits |
| Final dirty status | `M tools/hil_smoke.py`, `?? docs/reports/` |
| OS | Microsoft Windows NT 10.0.26200.0 |
| Python | 3.12.10 |
| PlatformIO | Core 6.1.18; PIO reported an obsolete-core warning |
| Target environment | `esp32s3dev`, Arduino framework, `esp32-s3-devkitc-1` |
| Also built | `esp32s2dev`, Arduino framework, `esp32-s2-saola-1` |
| Serial port / baud | `COM26` / 115200 |
| Detected USB device | ESP32-S3, MAC `64:e8:33:73:a1:54`, USB VID:PID `303A:1001` |
| I2C fixture | SDA GPIO8, SCL GPIO9, 400 kHz, 50 ms transaction timeout |
| Detected I2C endpoints | `0x6A` LSM6DS3TR-C, plus `0x40` unknown fixture device |
| Device identity | `WHO_AM_I = 0x6A`, expected `0x6A` |
| Electrical assumptions | 3.3 V reference wiring already present; no power-cycle, disconnect, line-hold, overtemperature, shock, or unsafe physical stimulus tests were performed |

## Commands Run

```powershell
pio run -e esp32s3dev
pio run -e esp32s3dev -t upload --upload-port COM26
python tools\hil_smoke.py --port COM26 --baud 115200 --suite smoke --timeout-s 20 --transcript-file docs\reports\artifacts\hil-COM26-20260622-smoke-transcript.txt --report-md docs\reports\artifacts\hil-COM26-20260622-smoke-results.md --json-out docs\reports\artifacts\hil-COM26-20260622-smoke-results.json
python tools\hil_smoke.py --port COM26 --baud 115200 --suite validation --stress-count 50 --reset-before --settle-timeout 5 --timeout-s 20 --transcript-file docs\reports\artifacts\hil-COM26-20260622-validation-transcript.txt --report-md docs\reports\artifacts\hil-COM26-20260622-validation-results.md --json-out docs\reports\artifacts\hil-COM26-20260622-validation-results.json
python tools\hil_smoke.py --port COM26 --baud 115200 --benchmark --benchmark-count 500 --reset-before --settle-timeout 5 --timeout-s 20 --transcript-file docs\reports\artifacts\hil-COM26-20260622-benchmark-transcript.txt --report-md docs\reports\artifacts\hil-COM26-20260622-benchmark-results.md --json-out docs\reports\artifacts\hil-COM26-20260622-benchmark-results.json
python tools\hil_smoke.py --port COM26 --baud 115200 --suite soak --soak-duration 28800 --stress-count 50 --reset-before --settle-timeout 5 --timeout-s 20 --reconnect-attempts 3 --reconnect-delay 1 --transcript-file docs\reports\artifacts\hil-COM26-20260622-soak-8h-transcript.txt --report-md docs\reports\artifacts\hil-COM26-20260622-soak-8h-results.md --json-out docs\reports\artifacts\hil-COM26-20260622-soak-8h-results.json
python tools\hil_smoke.py --port COM26 --baud 115200 --command raw --command health --settle-timeout 0 --timeout-s 10 --transcript-file docs\reports\artifacts\hil-COM26-20260623-post-soak-raw-transcript.txt --report-md docs\reports\artifacts\hil-COM26-20260623-post-soak-raw-results.md --json-out docs\reports\artifacts\hil-COM26-20260623-post-soak-raw-results.json
```

Final repository checks:

```powershell
python tools\hil_smoke.py --dry-run --parser-self-test
python tools\check_core_timing_guard.py
python tools\check_cli_contract.py
python tools\check_idf_example_contract.py
python tools\check_chip_docs_coverage.py
pio test -e native
pio run -e esp32s3dev
pio run -e esp32s2dev
pio pkg pack
git diff --check
```

The package archive `LSM6DS3TR-1.1.0.tar.gz` was removed after `pio pkg pack`.

## Tooling Changes

`tools/hil_smoke.py` was extended in place. The runner now has:

- explicit smoke, validation, benchmark, and soak suites
- per-step test IDs, expected-token classification, elapsed time, JSON output, Markdown output, and transcript output
- configurable serial port, baud, command timeout, idle timeout, boot settle time, command delay, and soak duration
- parser self-test and dry-run modes
- bounded missing-prompt retry and bounded serial reconnect/resync during soak
- 8-hour soak default duration

Relevant implementation points: `SOAK_DURATION_S` at `tools/hil_smoke.py:18`, validation suite at `tools/hil_smoke.py:217`, step execution at `tools/hil_smoke.py:403`, reconnect resync at `tools/hil_smoke.py:554`, soak loop at `tools/hil_smoke.py:591`, and CLI arguments at `tools/hil_smoke.py:741`.

Two early soak attempts exposed serial prompt loss:

| Attempt | Result | Evidence | Action |
|---|---:|---|---|
| Early soak 1 | 489 PASS, 1 FAIL | `settings` output truncated; immediate follow-up `health` passed | Added missing-prompt retry |
| Early soak 2 | 554 PASS, 1 FAIL | `funcsrc1` output truncated; immediate follow-up `health` and `funcsrc1` passed | Added bounded reconnect/resync |

Artifacts were preserved as `docs/reports/artifacts/hil-COM26-20260622-soak-earlyfail*` and `docs/reports/artifacts/hil-COM26-20260622-soak-earlyfail2*`.

## Summary

| Test Set | Scope | Expected Result | Observed Result | Count Summary | Result |
|---|---|---|---|---:|---|
| Build/flash | `esp32s3dev` firmware on `COM26` | Build and upload succeed | Build and upload succeeded; ESP32-S3 detected | 1 upload | PASS |
| Boot/control | Reset and `version` | Boot transcript and prompt | ESP ROM boot, CLI banner, I2C scan, init READY | 1 command | PASS |
| Smoke | Default HIL commands plus self-test | No unexpected failure tokens | All commands passed, self-test passed | 10 PASS | PASS |
| Validation | Broad CLI feature matrix | Command expectations met | All command expectations met | 120 PASS | PASS |
| Benchmark | `stress 500`, `stress_mix 500` | Complete with health READY | `stress 500` 4.797 s; `stress_mix 500` 0.218 s | 4 PASS | PASS |
| Soak precheck | 100-step soak subset | No failures | No failures | 100 PASS | PASS |
| 8-hour soak | Mixed command cycle for 28800 s | Finish without unrecovered link/driver failure | Completed 8:01:11.604, no hard failures, repeated recoverable serial prompt losses | 41261 PASS, 293 UNKNOWN, 0 FAIL | COMPLETED WITH UNKNOWN |
| Post-soak read | `raw`, `health` | Live I2C still works | Raw read and health passed | 2 PASS | PASS |
| Native tests | Unity native suite | All tests pass | 97/97 passed | 97 PASS | PASS |
| Static contracts | Core/CLI/IDF/docs checks | All pass | All passed | 4 PASS | PASS |
| Target builds | `esp32s3dev`, `esp32s2dev` | Both build | Both succeeded | 2 PASS | PASS |
| Package check | `pio pkg pack` | Archive builds | Archive built, then removed | 1 PASS | PASS |

Full per-step result tables:

- `docs/reports/artifacts/hil-COM26-20260622-smoke-results.md`
- `docs/reports/artifacts/hil-COM26-20260622-validation-results.md`
- `docs/reports/artifacts/hil-COM26-20260622-benchmark-results.md`
- `docs/reports/artifacts/hil-COM26-20260622-soak-precheck2-results.md`
- `docs/reports/artifacts/hil-COM26-20260622-soak-8h-results.md`
- `docs/reports/artifacts/hil-COM26-20260623-post-soak-raw-results.md`

Raw serial transcripts are in the matching `*-transcript.txt` files. The 8-hour transcript is `docs/reports/artifacts/hil-COM26-20260622-soak-8h-transcript.txt`.

## Detailed HIL Table

| Test ID | Feature Area | Command / Step | Expected | Observed | Elapsed | Result | Notes |
|---|---|---|---|---|---:|---|---|
| BOOT-001 | boot/control | reset before `version` | ESP boot and CLI prompt | ESP ROM boot, I2C scan, driver init READY, prompt responsive | 0.109 s command | PASS | `docs/reports/artifacts/hil-COM26-20260622-reset-version-transcript.txt` |
| SMK-010 | self-test | `selftest` | all safe self-test checks pass | accel/gyro self-test flow passed | 1.312 s | PASS | Full output in smoke transcript |
| VAL-ALL | validation | 120-step validation suite | identity, config, reads, FIFO, source regs, reset/recover, invalid input, stress all classified | 120 PASS, 0 FAIL | max 1.312 s | PASS | Full 120-row table in validation results artifact |
| BENCH-002 | benchmark | `stress 500` | ODR-paced stress completes | PASS, 4.797 s | 4.797 s | PASS | Approx. 104.2 reads/s at this CLI path |
| BENCH-003 | benchmark | `stress_mix 500` | mixed blocking operations complete | PASS, 0.218 s | 0.218 s | PASS | Approx. 2293.6 ops/s at this CLI path |
| SOAK-8H | soak | mixed cycle for 28800 s | no unrecovered communication or driver failure | 41261 PASS, 293 UNKNOWN, 0 FAIL | 8:01:11.604 | COMPLETED WITH UNKNOWN | Unknowns were serial prompt losses with successful bounded health resync |
| POST-RAW | post-soak | `raw`, `health` | live I2C and health after soak | raw read passed; health READY, consecutive failures 0 | max 0.110 s | PASS | Final live I2C check |

## Timing And Sampling

Short benchmark:

- `stress 500`: 4.797 s, approx. 104.2 converted reads/s through the CLI path.
- `stress_mix 500`: 0.218 s, approx. 2293.6 mixed operations/s through the CLI path.

8-hour soak:

- Start: 2026-06-22T20:44:56.255919+02:00
- End: 2026-06-23T04:46:07.860369+02:00
- Actual elapsed: 8:01:11.604
- Total step records: 41554
- PASS / FAIL / UNKNOWN / NOT_RUN: 41261 / 0 / 293 / 0
- Min / mean / max command elapsed: 0.093 s / 0.622 s / 125.188 s
- Worst latency: 125.188 s on a recoverable `stress_mix 50` prompt-loss timeout plus resync
- Final post-soak raw sample: `ax=75 ay=-313 az=16700 | gx=19 gy=-186 gz=-113 | t=1595`
- Final health after soak raw check: READY, consecutive failures 0

8-hour command mix:

| Command | Total | PASS | UNKNOWN |
|---|---:|---:|---:|
| `raw` | 2579 | 2578 | 1 |
| `read` | 2579 | 2576 | 3 |
| `accel` | 2579 | 2579 | 0 |
| `gyro` | 2579 | 2579 | 0 |
| `temp` | 2579 | 2579 | 0 |
| `status` | 2579 | 2578 | 1 |
| `fifo` | 2579 | 2578 | 1 |
| `probe` | 2579 | 2574 | 5 |
| `health` | 2870 | 2869 | 1 |
| `settings` | 2579 | 2540 | 39 |
| `funcsrc1` | 2579 | 2554 | 25 |
| `tsread` | 2579 | 2579 | 0 |
| `steps` | 2579 | 2494 | 85 |
| `stress 50` | 2579 | 2571 | 8 |
| `stress_mix 50` | 2579 | 2457 | 122 |
| `recover` | 2578 | 2576 | 2 |

The passed `stress 50` commands represent at least 128550 ODR-paced converted sample reads per active accel/gyro channel. Passed `stress_mix 50` commands represent 122850 mixed operation iterations. Commands classified `UNKNOWN` are not counted as successful samples.

## Failures And Anomalies

- No LSM6DS3TR I2C failures were observed in the final HIL evidence. Driver health after the soak showed READY, consecutive failures 0, total failures 0, and total success 1719277.
- The host serial session repeatedly lost or truncated command output during the long soak. The runner classified these as `UNKNOWN`, reopened the serial port, and required a `health` command to pass before continuing.
- The final 8-hour soak therefore completed with unknown communication events, not as a clean all-pass soak.
- Two early soak attempts were stopped and preserved before the bounded reconnect behavior was added.
- `0x40` was visible on the I2C bus but was not identified or exercised.

## Limitations And Not Run

| Area | Status | Reason |
|---|---|---|
| Electrical fault injection | NOT RUN | No safe fixture for disconnect, NACK forcing, SCL/SDA hold, brownout, overtemperature, or mechanical stimulus |
| Power-cycle behavior | NOT RUN | No controlled power relay or safe power-cycle fixture |
| Alternate IMU address `0x6B` | NOT RUN | Hardware detected only at `0x6A`; CLI config is hard-coded to `0x6A` |
| Safe absent-device behavior | NOT RUN | Would require disconnect/fault fixture |
| Public staged/chunked APIs via serial | NOT RUN | CLI lacks direct commands for `poll(maxInstructions)`, `startSoftReset()`, `startBoot()`, `startRefreshCachedConfig()`, `startFifoDrain()`, and staged calibration |
| `end()` / `UNINIT` HIL | NOT RUN | CLI has `begin` but no command that leaves the device ended/uninitialized |
| ESP-IDF live hardware example | NOT RUN | Native IDF contract was checked, but IDF toolchain/live flash was not run in this session |
| Full Cartesian config matrix | NOT RUN | Replaced by boundary/smoke matrix to keep HIL bounded |
| Unsafe threshold/alert stimulus | NOT RUN | No safe physical stimulus fixture |

## Fixes Implemented

| Area | File | Fix | Verification |
|---|---|---|---|
| HIL tooling | `tools/hil_smoke.py` | Added suite model, report/JSON/transcript output, elapsed timing, expected-token classification, benchmark mode, 8-hour soak mode, dry-run/parser self-test, bounded prompt retry, and bounded reconnect/resync | Parser self-test, dry-run, smoke, validation, benchmark, soak precheck, and 8-hour soak artifacts |

No core library behavior was changed in this pass. The library audit findings below need design/test decisions before changing public API behavior.

## Library And Example Audit

| Severity | Finding | Evidence | Risk | Simplest Safe Fix | Native Test | HIL Regression | Implemented |
|---|---|---|---|---|---|---|---|
| High | Async measurement deadlines and sample timestamps depend on optional `Config::nowMs`, not the `poll(nowMs)` clock | `requestMeasurement()` uses `_nowMs()` for `_pollDeadlineMs` at `src/LSM6DS3TR.cpp:594`; poll-completed samples stamp `_sampleTimestampMs = _nowMs()` at `src/LSM6DS3TR.cpp:2643`; `_nowMs()` fallback is at `src/LSM6DS3TR.cpp:2647` | With no time hook, tick/poll users can get immediate timeout or sample timestamp 0 | Initialize deadlines/timestamps from the explicit poll clock, or require `nowMs` for APIs that need time | `nowMs=nullptr`, request measurement, `poll(100000, 2)`, expect OK and timestamp 100000 | Add serial command for async request/poll and run after uptime >10 s | No |
| High | Combined measurements can report inactive-axis stale data as success | Combined request path starts at `src/LSM6DS3TR.cpp:575`; all-register raw read stamps sample at `src/LSM6DS3TR.cpp:1367`; `getMeasurement()` returns converted cached sample at `src/LSM6DS3TR.cpp:1268` | If accel or gyro is powered down, output registers may hold old values but API has no validity flags | Require both sensors active for combined fresh measurements, or add validity flags and mark inactive fields invalid | Configure gyro power-down, seed old gyro registers, request combined sample, assert rejection or invalid gyro flag | CLI command to power down one axis then request combined sample | No |
| Medium | Cached getters can return an old sample while a new request is pending | `getMeasurement()` only checks cached sample state at `src/LSM6DS3TR.cpp:1268`; request starts at `src/LSM6DS3TR.cpp:575` | Callers may treat stale data as completion of a pending request | Return `MEASUREMENT_NOT_READY` from requested-sample getters while a request is pending, or name cached-sample APIs explicitly | Cache sample A, request sample B, call getter before completion; expect not-ready | Async CLI command: request, immediate get, then poll to completion | No |
| Medium | Blocking bias capture can monopolize application loop for large sample counts | Public blocking capture declarations at `include/LSM6DS3TR/LSM6DS3TR.h:372` and `:394`; loops at `src/LSM6DS3TR.cpp:2772` and `:2828` | `cal 10000` style usage can block for a long time at low ODR | Cap blocking capture tightly or document it as diagnostic-only and prefer staged capture | Assert large blocking sample counts are rejected or explicitly opt-in | HIL staged calibration command with loop responsiveness checks | No |
| Medium | `end()` writes hardware power-down commands and ignores failures | `void LSM6DS3TR::end()` at `src/LSM6DS3TR.cpp:481` | Application cannot know whether shutdown I2C writes failed | Make `end()` state-only and add fallible `powerDown()`/`shutdown()`, or record/report shutdown failure before clearing state | Inject write failure during shutdown and assert observable status through new API | Safe fault fixture or fake I2C path in native only | No |
| Medium | Serial HIL cannot exercise important public staged APIs | Public poll/staged docs in README around `README.md:269`; Arduino CLI only uses `requestMeasurement()` internally behind stress state at `examples/01_basic_bringup_cli/main.cpp:1062` and `:2229` | Public instruction-budget contracts rely on native tests only, not hardware | Add narrow commands for request, poll budget, staged reset/boot/refresh/FIFO/calibration status | Existing native poll tests plus command contract update | HIL matrix for `poll 0/1/2`, busy, timeout, completion | No |
| Low | `Err::OFFLINE` exists but offline gates return `BUSY` | Offline enum in `include/LSM6DS3TR/Status.h`; gates noted in `src/LSM6DS3TR.cpp:551` and `:2574` | Programmatic status handling is less obvious from headers alone | Document `BUSY` as the offline API return or switch gates to `OFFLINE` with compatibility note | Force offline and assert chosen status contract | HIL only with safe NACK/fault fixture | No |
| Low | README ESP-IDF representative command drift | README shows `fifo_mode cont` at `README.md:222`; IDF parser uses different/native command handling around `examples/idf/basic/main/main.cpp:231` | Users may copy Arduino command syntax into IDF example | Correct README or add IDF parser alias | Static docs/contract check | IDF live CLI smoke if IDF toolchain available | No |

## Final Verification Results

| Command | Result |
|---|---|
| `python tools\hil_smoke.py --dry-run --parser-self-test` | PASS |
| `python tools\check_core_timing_guard.py` | PASS |
| `python tools\check_cli_contract.py` | PASS |
| `python tools\check_idf_example_contract.py` | PASS |
| `python tools\check_chip_docs_coverage.py` | PASS |
| `pio test -e native` | PASS, 97/97 |
| `pio run -e esp32s3dev` | PASS |
| `pio run -e esp32s2dev` | PASS |
| `pio pkg pack` | PASS, archive removed |
| `git diff --check` | PASS, only CRLF warning for `tools/hil_smoke.py` |

## Final Assessment

The connected LSM6DS3TR-C at `0x6A` responded correctly across the smoke, validation, benchmark, and post-soak checks. The 8-hour soak completed without driver offline state, I2C failure counters, or unrecovered serial loss. However, because 293 soak steps were classified `UNKNOWN` due to host serial prompt/output loss, the soak result is not a clean pass. The serial transport/tooling behavior should be stabilized or the soak should be rerun with a more reliable monitor path before using this as release evidence.
