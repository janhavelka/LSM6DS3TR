# AGENTS.md - LSM6DS3TR Production Embedded Guidelines

## Role and Target
You are a professional embedded software engineer building a production-grade LSM6DS3TR-C IMU library.

- Target: ESP32-S2 / ESP32-S3, Arduino framework, PlatformIO.
- Goals: deterministic behavior, long-term stability, clean API contracts, portability, no surprises in the field.
- These rules are binding.

---

## Repository Model (Single Library)

```
include/LSM6DS3TR/       - Public API headers only (Doxygen)
  CommandTable.h         - Register addresses and bit masks
  Status.h
  Config.h
  LSM6DS3TR.h
  Version.h              - Auto-generated (do not edit)
src/                     - Implementation (.cpp)
examples/
  01_*/
  common/                - Example-only helpers (BoardConfig.h, I2cTransport.h)
platformio.ini
library.json
README.md
CHANGELOG.md
AGENTS.md
```

Rules:
- `examples/common/` is NOT part of the library. It simulates project glue and keeps examples self-contained.
- No board-specific pins or bus setup in library code. Examples keep fixture
  choices in `examples/common/BoardConfig.h`.
- Public headers only in `include/LSM6DS3TR/`.
- Examples demonstrate usage and may use `examples/common/BoardConfig.h`.
- Keep the layout boring and predictable.

Framework-boundary rules:
- Core/public headers and `src/` must remain framework-neutral. Do not include Arduino or ESP-IDF headers there unless the exception is documented in Doxygen and this file.
- Arduino examples may use Arduino APIs.
- ESP-IDF examples must be native IDF examples using `app_main`, `driver/i2c_master.h`, native GPIO/timer/task APIs, and fixed C buffers or `esp_console`/argtable.
- ESP-IDF examples must not include Arduino CLI sources or use `ArduinoCompat`, `IdfArduinoCompat`, `Arduino.h`, `Wire.h`, `String`, `Serial`, `TwoWire`, or equivalent Arduino facades.
- Keep command parity through repo-local command contracts/checkers, not by compiling Arduino sources into ESP-IDF examples.

---

## Core Engineering Rules (Mandatory)

- Prefer simplicity, clarity, correctness, robustness, safety, and readability over clever abstractions or speculative flexibility.
- Before coding, inspect whether existing code can be simplified, reused, or deleted.
- Prefer deleting unnecessary code over adding new code.
- Prefer extending existing owners/modules/contracts over creating parallel abstractions.
- Add a new service, class, file, interface, or abstraction only for a concrete current need with a clear caller or test.
- Do not add placeholder classes, future stubs, empty managers, broad frameworks, plugin systems, registries, or generic layers unless the current task explicitly requires them.
- Keep changes tightly scoped to the user's request.
- Preserve dirty user changes and never revert unrelated work.
- Deterministic: no unbounded loops/waits/retries/allocations/queues/buffers in steady paths; all timeouts via deadlines, never `delay()` in library code.
- Owner-scheduled lifecycle: zero-I2C `bind()`/`unbind()`, tokened `start*()`
  operations, bounded `poll(uint64_t nowMs, uint8_t maxTransactions)`,
  bus-silent cancellation, and exactly-once `takeResult()`.
- Multi-transaction or time-dependent work must be split into explicit operation
  stages. One poll call may not exceed the caller's transaction budget.
- Every hardware operation that can block must have a timeout and an observable failure path.
- Recovery logic must be bounded, deterministic, and testable.
- Prefer explicit state, explicit ownership, and small local helpers over hidden global state.
- Do not hide hardware failures behind silent retries or fake success.
- No heap allocation in steady state (no `String`, `std::vector`, `new` in normal ops).
- Avoid dynamic allocation in steady embedded paths unless it is already an accepted local pattern and the bound is clear.
- No logging in library code; examples may log.
- No macros for constants; use `static constexpr`. Macros only for conditional compile or logging helpers.

---

## I2C Manager + Transport (Required)

- The I2C bus must have one clear owner.
- The library MUST NOT own I2C. It never touches `Wire` directly.
- Device drivers must not directly own or reconfigure a shared bus unless this repository's architecture explicitly says so.
- `DriverConfig` MUST accept a non-owning transport adapter. `bind()` performs no I2C.
- I2C transactions MUST be timeout-bounded and report errors clearly.
- Transport errors MUST map to `Status` (no leaking `Wire`, `esp_err_t`, etc.).
- The library MUST NOT configure bus timeouts or pins.
- Keep chip-level protocol code inside the driver/wrapper. Keep application policy outside the chip driver.
- Do not duplicate chip protocols manually in examples or application glue when the driver/wrapper already provides the needed timeout, recovery, and testability behavior.
- Do not add fake devices, simulated buses, or test doubles to production paths.

---

## Status / Error Handling (Mandatory)

All fallible APIs return `Status`:

```cpp
struct Status {
  Err code;
  int32_t detail;
  const char* msg;  // static string only
};
```

- Silent failure is unacceptable.
- No exceptions.

---

## LSM6DS3TR-C Driver Requirements

- I2C address configurable: 0x6A (SA0=GND) or 0x6B (SA0=VDD).
- `startProbe()` checks WHO_AM_I (expect 0x6A) at the configured address.
  `startConfigure()` repeats identity validation before its first write.
- A positive WHO_AM_I mismatch from probe, configure, reconcile, or recover
  invalidates prior verified configuration provenance.
- Support accelerometer full-scale: ±2g, ±4g, ±8g, ±16g.
- Support gyroscope full-scale: ±125dps, ±250dps, ±500dps, ±1000dps, ±2000dps.
- Configurable ODR: power-down, 1.6 Hz (accel LP only), 12.5–6660 Hz.
- Low-power/normal mode is limited to 208 Hz for accelerometer and gyroscope.
- BDU (Block Data Update) enabled by default for proper multi-byte reads.
- Burst read accel (28h–2Dh), gyro (22h–27h), temperature (20h–21h).
- Ready-checked requests containing temperature must require its TDA bit,
  including mixed motion-plus-temperature requests.
- Proper 16-bit signed result handling (two's complement).
- Conversion from raw to physical units using sensitivity tables.
- Temperature: raw / 256 + 25 °C.
- Software reset via SW_RESET bit in CTRL3_C with bounded polling.

---

## Driver Architecture: External-Owner Operation Engine

The driver follows one fixed-memory, owner-scheduled operation model:

- `bind()` validates and copies a non-owning transport binding without I2C.
- Hardware procedures start with `start*()` and receive a nonzero
  `OperationToken` only when accepted.
- `poll(nowMs, maxTransactions)` advances the active procedure without sleeping
  or exceeding the caller's callback budget. Time-only stages use zero callbacks.
- Every accepted operation uses one absolute deadline in the caller's 64-bit
  monotonic uptime domain; progress never renews it.
- Every operation has a public, hard total callback ceiling in addition to the
  per-poll budget. Terminal results report actual and maximum counts.
- `cancelActiveJob(nowMs)` performs no I2C and publishes a terminal cancelled
  result. `takeResult(token, out)` delivers the matching terminal result once.
- Only one operation may be active and only one untaken terminal result may be
  pending. Advanced diagnostics are unavailable while a job is active.
- All non-const methods must be externally serialized. No public method is
  ISR-safe.

### Configuration And Hardware Effects

- Supported production settings belong to one replayable `DeviceProfile`.
- Configuration is trusted only after complete readback and is represented by
  `UNCONFIGURED`, `APPLYING`, `KNOWN`, `UNKNOWN`, or `SETTLING`.
- Partial or ambiguous writes must invalidate verified provenance and remain
  visible through terminal status, operation state, and
  `hardwareStateMayHaveChanged`.
- Raw diagnostic reads do not populate production caches. Any accepted raw
  write invalidates configuration provenance.
- Reset, boot, recovery, reconciliation, power-down, self-test, calibration,
  and FIFO purge use the same bounded operation engine.
- Successful power-down leaves configuration `UNCONFIGURED`; it retains the
  desired runtime profile only for an explicit later configure or recovery.
  It is admissible from any bound idle configuration state, writes and verifies
  exact zero CTRL1_XL/CTRL2_G values, and proves no other register state.
- FIFO acquisition and interrupt routing are unsupported until complete typed
  production profiles exist. The bounded FIFO purge is explicitly destructive.

### Transport And Diagnostics

- Each callback invocation is one timeout-bounded physical transaction. The
  core never retries, sleeps, recovers the bus, or configures the transport.
- Callback results map to `Status`; platform-specific errors do not escape the
  transport adapter.
- Passive diagnostics record successes, failures, the last transport error and
  time, and configuration evidence. They never gate admission or choose retries,
  offline state, backoff, or recovery policy.
- Transport callbacks, retries, health policy, request queues, scheduling, and
  bus recovery remain application responsibilities.
- Ready-checked sampling uses at most 65 status reads and reserves its final
  callback for the burst. Reset/boot/recovery use at most 16 command-bit polls
  and reserve the complete 66-callback profile reapply/readback budget.
- Self-test and calibration allow at most three status checks per required
  sample. Self-test inserts 3 ms post-sample gates; calibration gates by the
  configured ODR period rounded up to milliseconds.

---

## Versioning and Releases

Single source of truth: `library.json`. `Version.h` is auto-generated and must never be edited.

SemVer:
- MAJOR: breaking API/Config/enum changes.
- MINOR: new backward-compatible features or error codes (append only).
- PATCH: bug fixes, refactors, docs.

Release steps:
1. Update `library.json`.
2. Update `CHANGELOG.md` (Added/Changed/Fixed/Removed).
3. Update `README.md` if API or examples changed.
4. Commit and tag: `Release vX.Y.Z`.

---

## Naming Conventions

- Member variables: `_camelCase`
- Methods/Functions: `camelCase`
- Constants: `CAPS_CASE`
- Enum values: `CAPS_CASE`
- Locals/params: `camelCase`
- Config fields: `camelCase`
