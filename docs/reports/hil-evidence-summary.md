# HIL Evidence Summary

> **Version scope:** This retained evidence was produced with the version 1.x
> API and one Arduino fixture. It remains useful chip/fixture history, but it
> does not validate the version 2 owner-scheduled lifecycle, cancellation,
> result identity, configuration provenance, or fault-injection contracts.
> Version 2 requires new HIL before field acceptance.

This file is the compact retained evidence after removing generated prompt files,
raw serial transcripts, JSON result dumps, Markdown result tables, stdout/stderr
captures, and PID files. The deleted files were runner artifacts, not source
inputs needed to build or test the library.

## Fixture

- Board: ESP32-S3 on COM26, Arduino bring-up CLI.
- IMU: LSM6DS3TR-C at I2C address `0x6A`; `WHO_AM_I = 0x6A`.
- Bus: SDA GPIO8, SCL GPIO9, 400 kHz, 50 ms transport timeout.
- Extra I2C endpoint observed: `0x40`, not identified or exercised.

## Retained Results

| Date | Run | Result | Notes |
|---|---:|---|---|
| 2026-06-22 | Smoke | 10 PASS, 0 FAIL | CLI, identity, data/status/FIFO, self-test. |
| 2026-06-22 | Broad validation | 120 PASS, 0 FAIL | CLI feature matrix including config, reads, FIFO, source registers, reset/recover, invalid input, and stress. |
| 2026-06-22 to 2026-06-23 | 8 h soak | 41261 PASS, 0 FAIL, 293 UNKNOWN | UNKNOWN records were host serial prompt/output capture losses; bounded health resync passed and final health was READY with consecutive failures 0. |
| 2026-06-23 | Post-soak live read | 2 PASS, 0 FAIL | `raw` and `health`; driver READY. |
| 2026-06-23 | Targeted poll/features | Initial 136 PASS, 8 FAIL; follow-up targeted checks passed | The 8 initial failures were expected ready-deadline timeouts from splitting a ready-checked job across slow manual serial commands, not crashes. Follow-up direct/poll/reset/boot/cancel probes passed. |
| 2026-06-23 | Production closeout targeted | 145 PASS, 0 FAIL | Covered production closeout behavior after API/docs fixes. |
| 2026-06-23 to 2026-06-24 | Requested 20 h intensive soak | Stopped after about 7 h 33 m: 39825 PASS, 1 FAIL, 29 UNKNOWN | Culprit was serial/CLI framing: the failed `stress_mix 500` result appeared in the next command capture; follow-up health was READY with total failures 0. |
| 2026-06-24 | Quiet stress check | 3 PASS, 0 FAIL | `stress 50 quiet`, `stress_mix 500 quiet`, `health`. |
| 2026-06-24 | 30 min quiet soak | 2095 PASS, 0 FAIL, 0 UNKNOWN | 18 prompt recoveries remained visible in timing/notes; stress payloads reported `errors=0`, `fail=0`, `health_fail=0`, `state=READY`. |
| 2026-06-24 | 10 min bounded quiet soak | 1102 PASS, 0 FAIL, 0 UNKNOWN | Final release HIL run. Seven prompt recoveries were noted, bounded to about 11 s for simple commands and 36 s for one `stress_mix`; no driver health failures were observed. |

## Historical Version 1 Culprit And Fix

The failed 20 h run was caused by host serial/CLI prompt framing, not by a
driver or sensor failure. Evidence:

- missed `Mixed Stress Summary` appeared in the following command capture;
- follow-up `health` reported READY with consecutive failures 0 and total
  failures 0;
- quiet stress output removed large progress/summary streams;
- bounded HIL reruns completed with 0 FAIL and 0 UNKNOWN.

The version 1 implementation used these mitigations:

- Arduino CLI supports `stress [N] [quiet]` and `stress_mix [N] [quiet]`,
  emitting single-line `RESULT ...` records.
- Soak HIL uses quiet stress commands.
- Soak step timeouts are bounded: 5 s for simple commands, 30 s for stress
  commands, 30 s for recover.
- Prompt recovery is visible in HIL notes instead of being hidden.
- Reconnect/resync drains stale serial input before the health probe.

## Final Non-HIL Verification

Before the version 1 cleanup/tag, these checks passed at that revision:

- `python tools/check_cli_contract.py`
- `python tools/check_idf_example_contract.py`
- the then-current `hil_smoke.py` parser self-test and dry-run for its v1 soak
  suite (the incompatible runner is intentionally not retained in version 2)
- `pio test -e native` - 113/113 test cases passed
- `pio run -e esp32s3dev`
- `pio run -e esp32s2dev`
- `git diff --check` - no whitespace errors; PowerShell reported only CRLF normalization warnings

## Remaining Release Gates

These were not covered by the retained HIL evidence:

- physical bus fault injection: SDA/SCL hold, forced NACK, timeout, brownout;
- IMU power-cycle or disconnect/reconnect while MCU stays alive;
- absent-device behavior on target wiring;
- alternate address `0x6B`;
- live native ESP-IDF HIL;
- a longer soak with independent serial-link evidence and no prompt-recovery
  notes, if that is required as a final field gate.
