# LSM6DS3TR I2C Uniformization Prompt

Repository: `LSM6DS3TR`

Absolute path: `C:\Users\Honza\Documents\Projects\LSM6DS3TR`

## Execution Rules

You are working inside this single repository. Implement this prompt directly;
do not repeat the cross-repository audit.

You may spawn subagents for read-only inspection of APIs, tests, I2C
transactions, docs, and diagnostics. Keep final judgment, edits, and
verification in the main agent.

Prefer simple, robust, readable code. Before adding code, inspect whether
existing code can be simplified, reused, tightened, or deleted.

Preserve dirty user changes. Do not commit unless explicitly asked.

## Common Uniformization Target

Apply this shared I2C library contract: injected non-owning transport, `Status` returns, cache-only `getSettings(SettingsSnapshot&) const`, active `probe()`/diagnostics named explicitly, `DriverState` with `state()` and `driverState()`, `isOnline()`, `lastOkMs()`, `lastErrorMs()`, `lastError()`, `consecutiveFailures()`, `totalFailures()`, and `totalSuccess()`.

Keep the common `Err` vocabulary append-only where missing: `OK`, `NOT_INITIALIZED`, `INVALID_CONFIG`, `INVALID_PARAM`, `I2C_ERROR`, `I2C_NACK_ADDR`, `I2C_NACK_DATA`, `I2C_TIMEOUT`, `I2C_BUS`, `DEVICE_NOT_FOUND`, `TIMEOUT`, `BUSY`, and `IN_PROGRESS`. Preserve LSM6DS3TR-specific self-test, FIFO, chip-ID, and measurement codes.

Uniformization is not a new base class or framework. Make only local, source-compatible additions and tests.

## Current State

- Public lifecycle and health are in `include\LSM6DS3TR\LSM6DS3TR.h`: `SettingsSnapshot` at line 110, `begin()`, `probe()`, `recover()`, `driverState()` at line 206, `lastOkMs()` at line 217, and `getSettings(SettingsSnapshot&)` at line 278.
- Native tests cover example transport error mapping and lifecycle behavior; `pio test -e native` passed 86 tests.
- Status model includes `SELF_TEST_FAIL`, `FIFO_EMPTY`, and `OFFLINE`, which are device-specific and should not be flattened.
- No explicit HIL runner was found.

## Best Sources To Adapt

- Use SHT3x for active diagnostic naming and health separation: `SHT3x-main\include\SHT3x\SHT3x.h:192-198`, `:340-358`.
- Use SSD1315 only as a bounded state-machine example if FIFO reads are ever chunked; do not copy framebuffer concepts.
- Use BME280/PCA9555 dirty-state patterns only for register writes that can actually leave cached configuration uncertain.

## Implementation Tasks

1. Preserve existing health API names and device-specific error codes.
2. Audit public self-test, FIFO, and configuration write paths. Any long-running wait must be timeout-bounded and visible as `TIMEOUT` or a more specific existing error.
3. Confirm `getSettings(SettingsSnapshot&) const` is bus-silent. If any status snapshot path probes hardware, rename that path to an active name such as `readSettings()` and keep the cache-only snapshot separate.
4. If configuration writes can partially reach hardware while cache remains stale, add a dirty/uncertain diagnostic modeled after BME280 or PCA9555. If the current code already updates cache only after confirmed writes, document that no dirty API is needed.
5. Add a HIL runner only if the repo already has a maintained serial CLI for WHO_AM_I, self-test, sample, FIFO, and health commands. If present, cover the common minimum contract: `version`, `scan`, `probe`, `settings`, `health`, failure-token classification, and dry-run/parser test support. Otherwise document HIL as not automated.

## API Changes Required

- None expected unless task 4 finds a real dirty-state gap.

## Simplifications Before Adding Code

- Prefer tightening existing self-test/FIFO code over adding new wrapper layers.
- Do not add a generic "sensor diagnostics" class.

## Tests To Add Or Update

- Native tests for timeout-bounded self-test/FIFO behavior if gaps are found.
- Native test proving `getSettings()` is bus-silent.
- HIL parser tests only if a real runner is added.

## Commands To Run

- `pio test -e native`
- `pio run -e esp32s3dev`

## Constraints And Non-Goals

- Do not hide self-test failures behind generic `I2C_ERROR`.
- Do not add bus ownership, global `Wire`, or hidden retry loops.
- Preserve distinct timeout, address NACK/device-not-found, data NACK, bus, chip-ID, self-test, FIFO, and measurement statuses. Do not use `DEVICE_NOT_FOUND` for timeout/data/bus failures.

## Risks And Open Questions

- Open: whether IMU self-test requires a live HIL fixture before adding any stronger field-readiness claims.
