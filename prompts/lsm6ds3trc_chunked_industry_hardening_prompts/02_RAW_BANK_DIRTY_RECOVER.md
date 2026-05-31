# Prompt 02 — Raw register safety, bank safety, partial-apply dirty state, recover/resync

Work only on protected register access, embedded-function bank safety, dirty state, partial multi-register writes, and recover/resync. Spawn subagents: `raw-api-safety-agent`, `bank-state-agent`, `partial-apply-agent`, `recover-resync-agent`, `fault-matrix-agent`.

## Context

The audit found public `writeRegisterValue()` can write `FUNC_CFG_ACCESS`, `CTRL3_C`, and `CTRL4_C`. That can leave the device in embedded-function bank, clear BDU/IF_INC, change BLE, trigger reset/boot, or set `I2C_disable`. The audit also found `_applyConfig()` and `configureFifo()` can partially apply register writes without surfacing dirty hardware state.

## Implement

1. Split raw access into safe diagnostic reads and explicit expert writes. Default public raw writes must reject protected registers: bank selection, bus-control, reset/boot, BDU, IF_INC, BLE, managed config, FIFO controls, interrupt routing controls.
2. Rejected writes return precise `UNSUPPORTED`/`PROTECTED_REGISTER` style status, not generic failure.
3. If expert writes remain, make them clearly named diagnostic/unsafe; they must set `HW_STATE_DIRTY`, preserve original status, and require `recover()`/resync before normal trusted measurement APIs.
4. Add safe bank helper: enter bank, perform operation, always attempt return to user bank; if restore fails, mark dirty/bad-bank.
5. Add dirty-state fields/API/snapshot: dirty flag, last dirtying status, failing register/phase in `detail` where practical.
6. On midpoint failure in any multi-register path, mark dirty and preserve original error; cache rollback must not be treated as hardware rollback.
7. `recover()`/`resync()` clears dirty only after WHO_AM_I, bank state, CTRL3_C safety bits, CTRL4_C I2C bit, and managed config are verified or reapplied.
8. Measurement APIs should fail with `HW_STATE_DIRTY` or equivalent while dirty, unless a documented safe latest-cache path exists.

## Tests

Add tests for:

- raw writes to `FUNC_CFG_ACCESS`, `CTRL3_C`, `CTRL4_C` rejected by default;
- expert writes mark dirty;
- failed bank restore marks dirty/bad-bank;
- `_applyConfig()` failure at each write position;
- `configureFifo()` failure at each write position;
- dirty state visible;
- recover failure keeps dirty;
- recover success clears dirty;
- normal reads blocked while dirty.

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
core: protect LSM6DS3TR-C register state and track dirty hardware
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
