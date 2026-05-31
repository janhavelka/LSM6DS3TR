# Prompt 03 — Reset/boot, ODR/filter settling, freshness, scaling, and self-test prep

Work on reset/boot correctness, ODR/power-mode constraints, filter settling/sample-discard rules, `tick()` semantics, cached-data freshness, raw-to-unit scaling, and self-test support/procedure. Spawn subagents: `reset-boot-agent`, `odr-settling-agent`, `freshness-agent`, `scaling-agent`, `self-test-agent`.

## Context

The app note says power-up boot is about 15 ms before accel/gyro enter power-down. It defines accel/gyro ODR/power modes, settling/sample discard guidance, data-ready behavior, and gyroscope HPF/LPF cutoff tables. The datasheet documents self-test mode selection in CTRL5_C and self-test output-change conditions.

## Implement

1. Align `softReset()` and `boot()` with local datasheet/app-note guidance. Use configured `nowMs`; return `RESET_TIMEOUT`/`BOOT_TIMEOUT` distinctly. Do not claim hardware-verified completion.
2. Validate ODR/power-mode constraints:
   - accel 1.6 Hz is low-power only;
   - low-power/normal/high-performance constraints documented and enforced;
   - gyro sleep/power-down behavior documented.
3. Add filter settling/sample-discard documentation or API support. Do not silently treat first samples after ODR/mode/filter changes as stable if datasheet says discard.
4. Clarify freshness:
   - `tick(nowMs)` must use timestamp or be documented/renamed in compatibility-safe way;
   - cached measurement has timestamp/fresh/stale status;
   - APIs promising a new sample return `DATA_NOT_READY` if no sample is ready;
   - latest/cache APIs are explicitly latest/stale-capable.
5. Centralize accel/gyro sensitivity and raw conversion tables. Test every accel FS and gyro FS including ±125 dps special case.
6. Temperature tests: raw `0 -> 25 °C`, positive raw, negative raw, rounding/overflow.
7. Self-test:
   - If bounded, implement diagnostic self-test that preserves/restores baseline, gathers baseline/test samples, compares per-axis deltas to datasheet limits, and marks dirty if restore fails.
   - If not bounded, document a hardware self-test procedure and return `UNSUPPORTED` for public request.

## Tests

Add/extend tests for reset/boot timeouts, missing `nowMs`, ODR constraint validation, stale cache, repeated get without tick, `DATA_NOT_READY`, all scaling ranges, temperature scaling, BLE protection, and self-test restore/failure behavior if implemented.

## Validate

```bash
python tools/check_core_framework_guard.py
python tools/check_core_timing_guard.py
python -m platformio test -e native
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
```

Commit message:

```text
core: harden LSM6DS3TR-C timing freshness and scaling contracts
```


---

## Required finish for this prompt

Run the requested checks, then:

```bash
git diff --check
git status --short
git add <changed files>
git commit -m "<requested commit message>"
git push -u origin HEAD
```

If push/sync fails, report the exact failure and local commit hash. Final response must include branch, commit, files changed, tests/checks run with exact results, and what remains next.
