# Prompt 01 — Core framework neutrality, timebase, status diagnostics

Work only on removing framework leakage from core, enforcing timebase contracts, and improving status/probe/begin diagnostics. Spawn subagents: `framework-neutrality-agent`, `timing-contract-agent`, `status-agent`, `fault-test-agent`.

## Context

The audit found `src/LSM6DS3TR.cpp` includes `<Arduino.h>` and `_nowMs()` falls back to `millis()`. It also found status vocabulary is too narrow and some probe/begin transport errors are collapsed.

## Implement

1. Remove `Arduino.h` and `millis()` fallback from core. Keep Arduino time/Wire only in examples/adapters.
2. For APIs needing deadlines/polling/reset/boot/calibration timing, require configured `Config::nowMs` or return `INVALID_CONFIG` before side effects. APIs not needing time should still work without `nowMs`.
3. Add/update a guard such as `tools/check_core_framework_guard.py` that fails on core use of `Arduino.h`, `Wire.h`, `String`, `millis(`, `micros(`, `delay(`, `yield(`, `vTaskDelay`, ESP-IDF/FreeRTOS headers, `Serial`.
4. Keep or update `tools/check_core_timing_guard.py`.
5. Expand status vocabulary append-only where practical: `DATA_NOT_READY`, `FIFO_EMPTY`, `FIFO_FULL`, `FIFO_OVERRUN`, `UNSUPPORTED`, `PARTIAL_APPLY`, `HW_STATE_DIRTY`, `DEGRADED`, `OFFLINE`, `SHORT_READ`, `RESET_TIMEOUT`, `BOOT_TIMEOUT`, `BAD_BANK_STATE`. Preserve existing numeric compatibility if possible.
6. Tighten `probe()` and `begin()`:
   - wrong WHO_AM_I distinct from absent device;
   - address NACK/data NACK/timeout/bus/short read preserved where transport supports it;
   - `probe()` stays raw and no-health-side-effect;
   - invalid config rejected before bus traffic.
7. Add tests for no-clock behavior, missing callbacks, zero timeout, NACK, timeout, bus error, short read, wrong WHO_AM_I, probe no-health-side-effect, and begin error mapping.
8. Add a minimal no-Arduino/native compile check if feasible.

## Validate

```bash
python tools/check_core_framework_guard.py
python tools/check_core_timing_guard.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
```

Commit message:

```text
core: make LSM6DS3TR-C core framework-neutral
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
