# Prompt 04 — FIFO dataset model, FIFO status/overrun, bounded drain

Work only on FIFO. Spawn subagents: `fifo-datasheet-agent`, `fifo-api-agent`, `fifo-latency-agent`, `fifo-tests-agent`.

## Context

The audit found FIFO 4th dataset support incomplete, `_buildFifoCtrl4()` not modeling `DEC_DS3_FIFO`/`DEC_DS4_FIFO`, and word-by-word FIFO drain causing long blocking bus occupancy. The device has 4 kB FIFO, FIFO_CTRL1..5, FIFO_STATUS1..4, FIFO_DATA_OUT_L/H, and special step/timestamp/temperature FIFO interactions.

## Implement

1. Define explicit typed FIFO dataset model:
   - accel;
   - gyro;
   - third dataset;
   - fourth dataset;
   - temperature;
   - timestamp;
   - step/step timestamp if supported.
2. Reject mutually exclusive or unsupported configurations precisely.
3. Encode FIFO_CTRL1..5 only from typed config; avoid partially modeled magic values.
4. Parse FIFO_STATUS1..4 fully: unread count high bits, watermark, overrun, full, empty, pattern.
5. Surface `FIFO_EMPTY`, `FIFO_OVERRUN`, `FIFO_FULL`, `DATA_NOT_READY` where appropriate.
6. Add bounded drain API:
   - caller-owned buffer;
   - max words/bytes per call;
   - no heap allocation;
   - returns count read;
   - stops on empty/overrun/transport error;
   - handles partial reads.
7. Keep one-word read only as simple/diagnostic or document its latency.
8. Document FIFO bus-latency lower bounds for 4 kB at 100 kHz and 400 kHz, and current/bounded API behavior. Do not claim 1 MHz unless local docs and hardware support it.
9. Midpoint failure in FIFO config must use dirty-state from Prompt 02.

## Tests

Add tests for FIFO_CTRL3/4/5 encodings, invalid combinations, status parsing edge cases, overrun/full/empty, bounded drain count, partial read, FIFO config midpoint failure dirty state, and no dynamic allocation.

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
core: correct LSM6DS3TR-C FIFO dataset and drain handling
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
