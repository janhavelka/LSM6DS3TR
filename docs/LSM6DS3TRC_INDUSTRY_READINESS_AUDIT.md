# LSM6DS3TR-C Industry-Readiness Audit

Date: 2026-05-31  
Repository: `C:\Users\HonzovoSpectre\Documents\Projects\LSM6DS3TR`  
Branch: `exploration/lsm6ds3trc-industry-readiness`  
Audit mode: exploration/report-only; no production driver fixes were implemented.

## Executive Summary

The repository is more mature than a bring-up prototype. It has an injected I2C transport, explicit `Status` values, health tracking, bounded reset polling, native tests, PlatformIO builds for ESP32-S2/S3, local datasheet/application-note references, and CI that exercises the main quality gates.

It is not yet ready to be described as an industry-ready production IMU library. The main blockers are not small polish issues: the core implementation is still Arduino-bound, raw register access can break hidden device state, multi-register configuration paths can partially apply hardware changes without surfacing dirty state, FIFO 4th-dataset behavior is incomplete, and there is no hardware validation evidence. Those issues can produce field surprises even if the current test suite passes.

The recommended next step is a hardening branch, not immediate release. The driver should first close the P0 findings in this audit, then add fault-injection and hardware validation coverage before making a production-readiness claim.

## Readiness Classification

Classification: **Engineering-grade with major gaps**.

Rationale: the project has a coherent architecture and useful test/build automation, but it still has several high-severity correctness and portability gaps. It is suitable for continued engineering hardening and controlled bench testing. It is not yet a production-grade, long-term-stability library for deployed products.

## Scope Reviewed

Reviewed areas:

- Public API headers under `include/LSM6DS3TR/`.
- Core implementation in `src/LSM6DS3TR.cpp`.
- Example glue under `examples/common/` and `examples/01_basic_bringup_cli/`.
- Native tests under `test/`.
- Project metadata: `library.json`, `platformio.ini`, `.github/workflows/ci.yml`.
- Local extracted device documentation under `docs/`, including datasheet and application-note markdown extracts.
- Existing quality tools under `tools/` and `scripts/`.

Explicitly not performed:

- No hardware execution against a real LSM6DS3TR-C device.
- No production fixes.
- No public API redesign.
- No release/tag work.

## Datasheet / Documentation Sources Found

Local primary sources:

- `docs/LSM6DS3TR-C_datasheet.pdf`
- `docs/pdf-extracted-md/LSM6DS3TR-C_datasheet.md`
- `docs/LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.pdf`
- `docs/pdf-extracted-md/LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.md`
- `LSM6DS3TR_imu_implementation_manual.md`
- Topic extracts under `docs/extracted-md/`

Key local references used:

- Device identity, I2C address, CS/I2C wiring, and IF_INC behavior: `docs/pdf-extracted-md/LSM6DS3TR-C_datasheet.md:1424`, `:1471`, `:1483`, `:1496`, `:1866`.
- CTRL3_C reset/BDU/IF_INC and CTRL4_C I2C disable: `docs/pdf-extracted-md/LSM6DS3TR-C_datasheet.md:2414`, `:2461`.
- Output registers, status bits, FIFO status/data, and temperature formula: `docs/pdf-extracted-md/LSM6DS3TR-C_datasheet.md:2829`, `:2848`, `:2860`, `:3096`, `:1003`.
- FIFO 4 kB, datasets, 4th dataset, step/timestamp/temperature FIFO behavior: `docs/pdf-extracted-md/LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.md:3029`, `:3163`, `:3882`, `:3913`.
- Reset/boot application-note sequence: `docs/pdf-extracted-md/LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.md:1946`.

## Device-Specific Facts Confirmed

- WHO_AM_I expected value is `0x6A`.
- I2C address is `0x6A` when SA0 is low and `0x6B` when SA0 is high.
- I2C mode requires CS held high.
- I2C Fast Mode is documented at 400 kHz; local docs do not establish 1 MHz Fast Mode Plus support.
- CTRL3_C contains `BOOT`, `BDU`, `IF_INC`, and `SW_RESET`.
- BDU and IF_INC are required for stable multi-byte register reads.
- Accelerometer full-scale encodings cover +/-2g, +/-4g, +/-8g, +/-16g.
- Gyroscope full-scale encodings cover +/-125 dps, +/-250 dps, +/-500 dps, +/-1000 dps, +/-2000 dps.
- Temperature conversion is `raw / 256 + 25 degC`.
- FIFO is 4 kB and can contain multiple datasets; 4th dataset handling is special and must be configured deliberately.

## Scorecard

| Area | Result | Notes |
| --- | --- | --- |
| Repository structure | PASS | Public headers, source, examples, tests, docs, and metadata are separated in a conventional PlatformIO library layout. |
| Injected transport | PASS | Core does not call `Wire` directly; examples own the Arduino `Wire` adapter. |
| Framework neutrality | FAIL | Core includes `<Arduino.h>` and falls back to `millis()` in `src/LSM6DS3TR.cpp:8` and `:1918`. |
| Status-based errors | PARTIAL | Fallible APIs return `Status`, but vocabulary is too narrow and some transport failures are collapsed. |
| Health tracking model | PARTIAL | Tracked wrappers centralize health updates, but begin/probe/recover semantics and offline visibility need tightening. |
| Deterministic timing | PARTIAL | Poll loops are bounded, but some public calls can perform large I2C bursts or thousands of FIFO transactions. |
| Datasheet register correctness | PARTIAL | Core register map and sensitivities are mostly aligned; FIFO 4th dataset and unsafe raw access are not. |
| Cache/hardware consistency | FAIL | Multi-register apply paths can leave hardware partially changed while software cache rolls back or appears valid. |
| Tests | PARTIAL | Native tests pass and are useful; important fault-injection and hardware-facing cases are missing. |
| ESP32-S2/S3 PlatformIO build | PASS | Both configured Arduino targets build successfully. |
| ESP-IDF portability | FAIL | There is no pure ESP-IDF build path and core still depends on Arduino symbols. |
| Documentation | PARTIAL | Extensive docs exist, but production claims, hardware caveats, freshness contracts, and FIFO limits need correction. |
| Hardware validation | UNKNOWN | No hardware validation matrix or run evidence was found. |

## What Is Strong

- The library uses a managed synchronous model instead of hidden background tasks or unbounded async behavior.
- I2C ownership is correctly placed outside the library; `examples/common/I2cTransport.h` is example glue.
- Health tracking is centralized in tracked transport wrappers in `src/LSM6DS3TR.cpp:1665-1846`.
- `probe()` uses raw I2C and does not update health, matching its diagnostic purpose.
- `recover()` uses tracked operations after initialization, so recovery failures affect health state.
- Native tests and PlatformIO builds are already wired into local and CI workflows.
- Version generation is automated from `library.json`, and `Version.h` was verified current.
- The example CLI surfaces a lot of operational state and is useful for bench bring-up.

## High-Severity Findings

### H1. Core implementation is Arduino-bound

Severity: High

Evidence:

- `src/LSM6DS3TR.cpp:8` includes `<Arduino.h>`.
- `_nowMs()` in `src/LSM6DS3TR.cpp:1918-1922` falls back to `millis()` when no `nowMs` callback is configured.
- `platformio.ini` only defines Arduino environments; `idf.py --version` was unavailable locally.

Impact:

The public architecture says the library is transport-injected and portable, but the implementation still requires Arduino symbols. That blocks a pure ESP-IDF consumer and makes timing behavior depend on hidden framework state if `Config::nowMs` is omitted.

Recommended remediation:

- Remove `Arduino.h` from core implementation.
- Require `Config::nowMs` for operations that need deadlines, or provide a framework-neutral compile-time adapter outside core.
- Keep Arduino-specific time and Wire glue in examples or optional adapters only.

Suggested tests:

- Add a non-Arduino compile target for core sources.
- Add a config validation test proving missing `nowMs` is rejected when deadlines are required.
- Add a CI guard that fails if core includes `Arduino.h`, `Wire.h`, `String`, or framework delay APIs.

### H2. Raw public register writes can break hidden device state

Severity: High

Evidence:

- `writeRegisterValue()` exposes raw register writes in `src/LSM6DS3TR.cpp:1454-1497`.
- `isValidPublicRegisterAddress()` only checks the register is at or below `REG_Z_OFS_USR` in `src/LSM6DS3TR.cpp:227`.
- `FUNC_CFG_ACCESS`, `CTRL3_C`, and `CTRL4_C` are within the allowed range.
- Datasheet references show `FUNC_CFG_ACCESS` changes bank access, CTRL3_C contains `IF_INC`, and CTRL4_C contains `I2C_disable`.

Impact:

A caller can accidentally leave the device in the embedded-function bank, disable I2C, change endianness, clear IF_INC, or mutate BDU/reset behavior. The driver cache may still look valid while future reads and writes address a different bank or no longer communicate.

Recommended remediation:

- Split raw register access into safe diagnostic APIs with explicit allowlists.
- Block writes to bank-selection, bus-control, reset, IF_INC, BDU, BLE, and managed configuration registers unless a deliberate expert mode is enabled.
- After expert writes, mark cache/hardware state dirty and require `recover()` or a full `begin()`-style resync.

Suggested tests:

- Fault-injection tests for rejected writes to `FUNC_CFG_ACCESS`, `CTRL3_C`, and `CTRL4_C`.
- Tests proving expert writes set a dirty flag and prevent stale managed reads until recovery.
- Tests proving cache refresh returns a specific dirty/unsupported status if bank state is unsafe.

### H3. Multi-register configuration can partially apply without surfacing dirty hardware state

Severity: High

Evidence:

- `_applyConfig()` writes CTRL registers sequentially in `src/LSM6DS3TR.cpp:1847-1889`.
- `configureFifo()` writes FIFO_CTRL1..FIFO_CTRL5 sequentially in `src/LSM6DS3TR.cpp:1256-1307`.
- Cache rollback exists in some paths, but hardware rollback is not guaranteed.
- Existing `Err` values in `include/LSM6DS3TR/Status.h:10-30` do not include `PARTIAL_APPLY` or `HW_STATE_DIRTY`.

Impact:

A transport failure midway through configuration can leave the IMU running with partially changed ODR, scale, FIFO mode, or interrupts while the driver reports only a generic I2C failure. Applications cannot decide whether to trust subsequent samples without a full recovery strategy.

Recommended remediation:

- Introduce explicit dirty-state tracking for partial apply failures.
- Return a status such as `PARTIAL_APPLY` with the failing register or phase in `detail`.
- Gate measurement APIs while dirty unless the driver successfully refreshes and validates the managed register set.

Suggested tests:

- Inject failures at every register write in `_applyConfig()` and `configureFifo()`.
- Assert state/cache/health behavior after each injected midpoint failure.
- Verify `recover()` clears dirty state only after hardware is revalidated.

### H4. FIFO 4th dataset support is incomplete and can misrepresent device behavior

Severity: High

Evidence:

- `_buildFifoCtrl4()` in `src/LSM6DS3TR.cpp:1998-2003` only emits `STOP_ON_FTH` and `ONLY_HIGH_DATA` style controls; it does not model `DEC_DS3_FIFO` or `DEC_DS4_FIFO` dataset decimation/source controls.
- AN5130 distinguishes 4th dataset, step/timestamp FIFO, and temperature FIFO controls.
- Local app-note extract indicates temperature FIFO depends on `TIMER_PEDO_FIFO_EN = 0`, while step/timestamp FIFO uses that path differently.

Impact:

Applications using FIFO for temperature, step counter, timestamp, or mixed datasets can get an incomplete or misleading stream interpretation. This is especially risky because FIFO bugs often show up as plausible but incorrectly sequenced data.

Recommended remediation:

- Define an explicit FIFO dataset model: accel, gyro, 3rd dataset, and 4th dataset source.
- Prevent mutually exclusive step/timestamp/temperature configurations.
- Document and test FIFO pattern decoding for every supported dataset combination.

Suggested tests:

- Unit tests for FIFO_CTRL3/4/5 encodings across every supported FIFO dataset mode.
- Validation tests rejecting mutually exclusive temperature and step/timestamp FIFO settings.
- Hardware tests reading known FIFO patterns with accel-only, gyro-only, accel+gyro, and 4th-dataset modes.

### H5. Reset and boot behavior do not fully match application-note sequencing

Severity: High

Evidence:

- `softReset()` and `boot()` are bounded in `src/LSM6DS3TR.cpp:1363-1445`.
- AN5130 documents reset/boot preconditions and wait behavior around power-down and boot flows.
- `boot()` currently polls the BOOT bit, but local docs do not prove this is a reliable completion signal for every mode.

Impact:

Bounded polling is good, but incorrect sequencing around reset/boot can leave the device in an undefined operating state. This matters in field recovery paths where reset is used after bus errors or brownout-like behavior.

Recommended remediation:

- Align reset and boot flows with AN5130 preconditions.
- Document exactly what completion condition is used and why.
- If BOOT polling remains ambiguous, classify completion confidence as hardware-validated only after bench evidence.

Suggested tests:

- Unit tests for reset/boot timeout paths.
- Fault-injection tests for CTRL3_C read/write failures during reset and boot.
- Hardware tests measuring reset and boot completion across ODRs and power modes.

### H6. Public status vocabulary is too narrow for production diagnostics

Severity: High

Evidence:

- `include/LSM6DS3TR/Status.h:10-30` lacks statuses for `DATA_NOT_READY`, `FIFO_OVERRUN`, `UNSUPPORTED`, `PARTIAL_APPLY`, `HW_STATE_DIRTY`, `DEGRADED`, and `OFFLINE`; wrong WHO_AM_I is represented as `CHIP_ID_MISMATCH`.
- `begin()` and `probe()` distinguish wrong WHO_AM_I, but collapse some transport failures during identity reads to broad device-not-found style results in `src/LSM6DS3TR.cpp:294-382` and `:442-455`.

Impact:

Production applications need to distinguish absent device, wrong silicon, temporary data-not-ready, FIFO overrun, unsupported configuration, degraded health, and dirty hardware state. Existing `CHIP_ID_MISMATCH` covers wrong silicon, but broader errors still force callers into guesswork and make field telemetry less useful.

Recommended remediation:

- Append new `Err` enum values without breaking existing numeric values.
- Preserve transport status codes through `detail` where practical.
- Return specific statuses for wrong WHO_AM_I, not-ready data, FIFO overrun/full conditions, offline state, and partial apply.

Suggested tests:

- Contract tests for every new status.
- Begin/probe tests separating NACK, timeout, short read, and wrong WHO_AM_I.
- FIFO status tests proving overrun/full/watermark are surfaced when APIs require them.

### H7. FIFO drain can be extremely blocking and is not documented as such

Severity: High

Evidence:

- FIFO is 4 kB by datasheet.
- `readFifoStatus()` reads status registers in `src/LSM6DS3TR.cpp:1317-1336`.
- `readFifoWord()` performs a status read and a FIFO data read per word in `src/LSM6DS3TR.cpp:1339-1361`.
- At high FIFO depth, draining word-by-word requires about two I2C transactions per successful FIFO word, plus the caller's initial/terminal status checks.

Impact:

A public API that appears to read one FIFO word safely can produce long blocking windows when used to drain a full FIFO in a loop. At 400 kHz, the current transaction shape can consume hundreds of milliseconds of bus time before application overhead.

Recommended remediation:

- Add an explicit burst FIFO drain API with a caller-provided fixed buffer and max-word limit.
- Document worst-case bus occupancy for word-by-word FIFO reads.
- Provide a bounded chunking pattern for applications that must preserve real-time responsiveness.

Suggested tests:

- Timing-budget tests around maximum configured FIFO chunk sizes.
- Fault injection for partial FIFO burst reads.
- Hardware timing captures at 100 kHz and 400 kHz.

### H8. No hardware validation evidence was found

Severity: High

Evidence:

- CI and local tests build and run without hardware.
- No validation matrix, captured register dumps, logic-analyzer traces, or board-specific test record was found.

Impact:

For an IMU driver, native tests are necessary but insufficient. Electrical wiring, bus pullups, CS/SA0 state, timing, reset behavior, FIFO drain behavior, and interrupt routing must be validated on real boards before production use.

Recommended remediation:

- Add a hardware validation document with board, sensor marking, voltage, bus speed, pullups, commit hash, and test results.
- Capture register dumps after begin/recover/reset/FIFO configurations.
- Keep logic-analyzer traces for reset, WHO_AM_I, burst reads, and FIFO drain.

Suggested tests:

- See the hardware validation matrix below.
- Run the same matrix on ESP32-S2 and ESP32-S3 boards.
- Repeat at both supported I2C addresses.

## Medium-Severity Findings

### M1. `tick(uint32_t nowMs)` ignores its timestamp parameter

Severity: Medium

Evidence:

- `tick()` takes `nowMs` but explicitly ignores it in `src/LSM6DS3TR.cpp:385-386`.
- It can perform a status read and then `_readRawAll()` in `src/LSM6DS3TR.cpp:397` and `:413`.

Impact:

The lifecycle advertises caller-driven time, but `tick()` does not use the supplied time for scheduling or health timestamps. It may also block for multiple I2C operations, which should be documented.

Recommended remediation:

- Either remove the unused parameter from future breaking APIs or use it consistently for timestamps/scheduling.
- Document the maximum expected I2C transaction count per `tick()`.

Suggested tests:

- Verify `tick(nowMs)` updates health timestamps with the supplied value.
- Add a no-transport-call case when data-ready polling is disabled.

### M2. Driver object appears copyable/movable

Severity: Medium

Evidence:

- `include/LSM6DS3TR/LSM6DS3TR.h` defines a stateful class with transport callbacks, caches, and health fields, but no deleted copy/move operations were found.

Impact:

Implicit copies of a driver object can duplicate state and callback context unexpectedly. In embedded code this often leads to two objects believing they own the same logical device/session.

Recommended remediation:

- Delete copy constructor, copy assignment, move constructor, and move assignment unless a well-defined copy contract is intentionally designed.

Suggested tests:

- Add compile-time tests proving the driver is not copy-constructible or move-constructible.

### M3. Example Arduino transport loses some timeout detail

Severity: Medium

Evidence:

- `examples/common/I2cTransport.h:21-35` maps Wire result codes.
- `examples/common/I2cTransport.h:40-47` only applies timeouts that fit the underlying API range.
- `examples/common/I2cTransport.h:113-117` maps short read to generic I2C error.

Impact:

The example is not library core, but users are likely to copy it. Collapsing timeout or short-read detail weakens diagnostics in production sketches.

Recommended remediation:

- Preserve timeout and short-read detail in the adapter `Status`.
- Clamp or reject unsupported timeout values explicitly.

Suggested tests:

- Example-adapter unit tests for timeout too large, short read, NACK, and read timeout.

### M4. Config validation coverage and time-source contract need tightening

Severity: Medium

Evidence:

- `begin()` validates the core I2C callbacks and timeout before bus traffic.
- Test coverage should still explicitly cover one-callback-missing and zero-timeout cases.
- `nowMs` remains optional because core falls back to Arduino `millis()`, which weakens the deadline contract for framework-neutral production use.

Impact:

Validation is mostly in the right place, but insufficient tests and an optional framework-dependent time source leave the no-surprises API contract weaker than it should be.

Recommended remediation:

- Keep rejecting invalid transport callback/timeout configs before bus traffic.
- Add targeted tests for callback-pair and timeout validation.
- Require or explicitly document a framework-neutral time source for deadline-bearing operations.

Suggested tests:

- Missing write callback.
- Missing write-read callback.
- Missing context where required by adapter contract.
- Zero timeout and extreme timeout values.

### M5. Gyroscope high-pass-filter enum labels appear misleading

Severity: Medium

Evidence:

- `include/LSM6DS3TR/Config.h:86-91` labels gyro HPF cutoff options.
- Datasheet extracted references around `docs/pdf-extracted-md/LSM6DS3TR-C_datasheet.md:2576-2580` use device-specific cutoff selections tied to ODR.

Impact:

Mislabelled filter options can produce incorrect application tuning even if register bits are written correctly.

Recommended remediation:

- Rename enum values or documentation to match datasheet semantics.
- If cutoff depends on ODR, document it as a selection code instead of a fixed Hz value.

Suggested tests:

- Encoding tests for each HPF enum value.
- Documentation review against datasheet table names.

### M6. Measurement freshness contract is unclear

Severity: Medium

Evidence:

- `getMeasurement()` and cached measurement docs imply freshness, while `tick()` only updates cache when status data-ready bits are observed.
- Direct read APIs and cached APIs have different freshness semantics.

Impact:

Applications may treat cached data as newly sampled when it is actually the last successful poll result.

Recommended remediation:

- Add timestamp and freshness status to cached measurement APIs, or clearly document stale cache behavior.
- Return `DATA_NOT_READY` from APIs that promise a new sample but none is available.

Suggested tests:

- Data-ready false path.
- Repeated `getMeasurement()` without new data.
- Health transition after stale-cache reads versus actual transport failures.

### M7. Documentation can imply SPI or broader interface support than the library provides

Severity: Medium

Evidence:

- The datasheet covers SPI and I2C, while the library architecture is I2C-only.
- README/manual excerpts discuss device interfaces and implementation details in ways that need sharper library scope language.

Impact:

Users may expect SPI support or assume documented device features are implemented when only some I2C-managed features are supported.

Recommended remediation:

- State plainly that this library is currently I2C-only.
- Separate device capability documentation from implemented library capability.

Suggested tests:

- Documentation checklist test or lint that README/API docs contain "I2C-only" scope language.

### M8. CI coverage is good but still misses hidden files and ESP-IDF contract checks

Severity: Medium

Evidence:

- `.github/workflows/ci.yml` builds Arduino PlatformIO environments and runs native tests/guards.
- `python tools/check_idf_example_contract.py` failed locally because the file does not exist.
- Default `rg --files` did not show hidden CI files; they were inspected separately.

Impact:

CI currently supports the Arduino PlatformIO story, but it does not prove framework neutrality or ESP-IDF adapter readiness.

Recommended remediation:

- Add a real ESP-IDF/core-portability check or remove stale references to missing checks.
- Keep CI explicit about hidden-file scanning if contract tools depend on `.github`.

Suggested tests:

- Add a no-Arduino compile job.
- Add a CI step that runs any documented contract-check scripts and fails if referenced scripts are missing.

## Low-Severity Findings

### L1. `Config.h` says SA0=VDD instead of SA0=VDD_IO

Severity: Low

Evidence:

- `include/LSM6DS3TR/Config.h:136` documents address `0x6B` as `SA0=VDD`.
- Datasheet pin/power references distinguish VDD and VDD_IO.

Impact:

This is a documentation precision issue, but incorrect voltage-domain wording can cause bad wiring guidance.

Recommended remediation:

- Change public docs to say SA0 high is tied to VDD_IO or logic high in the I/O voltage domain.

Suggested tests:

- Documentation review against datasheet pin table.

### L2. Public header uses `#undef DISABLED`

Severity: Low

Evidence:

- `include/LSM6DS3TR/Config.h:103` conditionally undefines `DISABLED`.

Impact:

This avoids macro collisions but mutating consumer macro state from a public header is surprising.

Recommended remediation:

- Rename enum values or isolate compatibility handling so public headers do not alter user macros.

Suggested tests:

- Compile test with a consumer-defined `DISABLED` macro.

### L3. README and manual need stronger hardware caveats

Severity: Low

Evidence:

- Current docs mention bring-up and configuration, but the audit found missing emphasis on CS high for I2C, VDD_IO domain, pullups, address strap, and no hardware validation record.

Impact:

Users may wire boards incorrectly or over-trust unvalidated paths.

Recommended remediation:

- Add a hardware assumptions section to README.
- Mark example CLI as bring-up/diagnostic, not production glue.

Suggested tests:

- Documentation checklist for CS, SA0, VDD_IO, pullups, bus speed, and sensor marking.

### L4. Scaling tables are fragmented across docs and implementation

Severity: Low

Evidence:

- Sensitivity tables are implemented in `src/LSM6DS3TR.cpp:256-281`.
- Documentation references scaling in several places.

Impact:

The implementation appears mostly correct, but duplicated documentation increases drift risk.

Recommended remediation:

- Keep one public table in API docs and point README/manual to it.
- Include units and raw-to-physical formulas together.

Suggested tests:

- Unit tests covering every accel and gyro full-scale conversion.

### L5. Generated package artifact should not be left in the worktree

Severity: Low

Evidence:

- `python -m platformio pkg pack` generated `LSM6DS3TR-1.1.0.tar.gz`.
- The artifact was removed after verification.

Impact:

Generated release artifacts can pollute exploration branches if not removed.

Recommended remediation:

- Ensure package artifacts are ignored or generated only in CI output directories.

Suggested tests:

- Check `git status --short` after packaging in release workflows.

## LSM6DS3TR-C Correctness Checklist

| Item | Result | Evidence / Notes |
| --- | --- | --- |
| Configurable I2C address `0x6A` / `0x6B` | PASS | Config validation and docs support both addresses. |
| WHO_AM_I check expects `0x6A` | PASS | `begin()` reads WHO_AM_I before applying config. |
| Core does not touch `Wire` directly | PASS | No core `Wire` usage found; example adapter owns `Wire`. |
| Core is framework-neutral | FAIL | Core includes Arduino and uses `millis()` fallback. |
| Fallible APIs return `Status` | PASS | API follows status-return model. |
| Status vocabulary production-complete | FAIL | Missing several diagnostic and state statuses. |
| No library logging | PASS | Logging is in examples. |
| No `delay()` in library code | PASS | Timing guard passed; delays found are in examples. |
| No unbounded reset polling | PASS | Reset/boot loops have deadlines. |
| Reset/boot sequence fully app-note aligned | UNKNOWN | Needs deeper validation and hardware evidence. |
| BDU enabled by default | PASS | Config/apply path manages CTRL3_C. |
| IF_INC enabled for burst reads | PASS | Config/apply path manages CTRL3_C. |
| IF_INC protected from user raw writes | FAIL | Public raw writes can mutate CTRL3_C. |
| BLE protected from user raw writes | FAIL | Public raw writes can mutate CTRL3_C. |
| I2C_disable protected from user raw writes | FAIL | Public raw writes can mutate CTRL4_C. |
| Accel FS +/-2/4/8/16g | PASS | Encodings and sensitivity table present. |
| Gyro FS +/-125/250/500/1000/2000 dps | PASS | Encodings and sensitivity table present. |
| ODR power-down and 12.5-6660 Hz | PASS | Encodings present. |
| Accel 1.6 Hz low-power-only constraint | PARTIAL | Encoding exists; mode constraint needs stronger validation/docs. |
| Burst accel read `0x28-0x2D` | PASS | Raw burst helper exists. |
| Burst gyro read `0x22-0x27` | PASS | Raw burst helper exists. |
| Burst temperature read `0x20-0x21` | PASS | Raw burst helper exists. |
| Signed 16-bit two's complement handling | PASS | Conversion uses signed raw interpretation. |
| Temperature formula `raw / 256 + 25` | PASS | Implemented. |
| FIFO status parsing | PARTIAL | Basic fields parsed; high-bit/edge coverage needs tests. |
| FIFO 4th dataset support | FAIL | Temperature/step/timestamp interaction incomplete. |
| Interrupt routing support | PARTIAL | Encodings exist; hardware behavior not validated. |
| Managed health tracking through wrappers | PASS | `_updateHealth()` is called in tracked wrappers. |
| Probe does not update health | PASS | Probe uses raw path. |
| Recover tracks failures | PASS | Recover uses tracked read/apply after initialization. |
| Partial-apply dirty state | FAIL | No explicit `PARTIAL_APPLY` / `HW_STATE_DIRTY` contract. |
| Hardware validation | UNKNOWN | No evidence found. |

## API Latency / Blocking Model table

| API / Path | Blocking shape | Risk | Recommendation |
| --- | --- | --- | --- |
| `begin(const Config&)` | WHO_AM_I read plus config writes; roughly 17 I2C transactions depending on config. | Medium during startup. | Document startup bus budget and fail phase detail. |
| `tick(uint32_t nowMs)` | Status read, and optionally accel/gyro/temp burst reads. | Medium; ignores `nowMs`. | Use timestamp or remove from future API; document max transactions. |
| `readAccel()` | One burst read. | Low. | Acceptable as managed synchronous call. |
| `readGyro()` | One burst read. | Low. | Acceptable as managed synchronous call. |
| `readTemperature()` | One burst read. | Low. | Acceptable as managed synchronous call. |
| `readMeasurement()` / raw-all path | Multiple burst reads. | Medium. | Document transaction count and timeout effect. |
| `configureFifo()` | Sequential FIFO control writes. | High on midpoint failure. | Add dirty-state status and rollback/resync contract. |
| `readFifoStatus()` | One status burst read. | Low. | Acceptable. |
| `readFifoWord()` | Status read plus FIFO data read per word. | High when called in a drain loop. | Add chunked/burst drain API. |
| `refreshCachedConfig()` | About 11 register reads plus offset/FIFO reads. | Medium. | Document and use after expert writes/recovery only. |
| `softReset()` | CTRL3_C write plus bounded polling. | Medium. | Align app-note sequence and test timeout. |
| `boot()` | CTRL3_C write plus bounded polling. | Medium. | Validate completion signal on hardware. |
| `captureAccelBias()` / `captureGyroBias()` | Bounded busy polling and repeated sample reads. | High for application responsiveness. | Document as calibration-only, not steady-state path. |

## FIFO Bus-Latency Analysis

The device FIFO is 4 kB. A full FIFO is 4096 payload bytes, or 2048 16-bit words. The current public `readFifoWord()` shape reads status and then reads one FIFO word. A near-full drain loop therefore performs about two I2C transactions per successful FIFO word, plus whatever initial and terminal status checks the caller uses.

Ideal continuous payload lower bound:

| Bus speed | 4096-byte payload at 9 bits/byte | Notes |
| --- | ---: | --- |
| 100 kHz | about 369 ms | Payload only; excludes addressing/start/stop overhead. |
| 400 kHz | about 92 ms | Datasheet-supported Fast Mode. |
| 1 MHz | about 37 ms | Not established by local datasheet; should not be claimed supported. |

Current word-by-word API rough lower bound:

| Bus speed | Approximate lower bound | Notes |
| --- | ---: | --- |
| 100 kHz | about 1.8 s or more | Includes repeated per-word status/data transactions, still excludes software overhead. |
| 400 kHz | about 0.45 s or more | Too long for many real-time loops. |
| 1 MHz | about 0.18 s or more | Not a supported operating claim from local docs. |

Conclusion: FIFO support needs a bounded burst/chunk drain API with caller-owned buffers and explicit max words per call. The existing one-word API can remain for diagnostics or simple polling, but it should be documented as unsuitable for draining a large FIFO in latency-sensitive applications.

## Partial-State / Cache Consistency Assessment

The driver keeps software caches for managed config and status, which is the right direction for a production API. The gap is failure atomicity. The IMU does not provide transaction-level rollback across multiple register writes, so the driver must treat midpoint failures as a distinct hardware state.

Current risks:

- `_applyConfig()` can update some control registers before failing on a later write.
- `configureFifo()` can update only part of FIFO_CTRL1..FIFO_CTRL5.
- Public raw writes can mutate managed registers or bank/bus state and then attempt a cache refresh.
- Cache rollback is not equivalent to hardware rollback.
- No explicit dirty state tells applications that a full revalidation is required.

Required production contract:

- On midpoint write failure, enter a dirty or degraded state with a specific status.
- Measurement APIs should fail while dirty unless a documented resync succeeds.
- `recover()` should validate WHO_AM_I, bank state, CTRL3_C safety bits, CTRL4_C bus bit, and all managed config before returning READY.
- Expert raw writes should either be outside the production API or force a dirty/resync path.

## Tests and Build Coverage

Commands run locally:

| Command | Result |
| --- | --- |
| `git status --short` | No output at audit start; worktree was clean. |
| `git checkout -b exploration/lsm6ds3trc-industry-readiness` | Succeeded. |
| `git branch --show-current` | `exploration/lsm6ds3trc-industry-readiness`. |
| `git remote -v` | `origin https://github.com/janhavelka/LSM6DS3TR.git` for fetch/push. |
| `python --version` | `Python 3.13.12`. |
| `python -m platformio --version` | `PlatformIO Core, version 6.1.19`. |
| `rg --files` | Repository files discovered; hidden CI file inspected separately. |
| `rg -n "Arduino\.h\|Wire\.h\|driver/i2c\|freertos\|String\|delay\(\|vTaskDelay\|WHO_AM_I\|0x0F\|0x6A\|CTRL3_C\|BDU\|IF_INC\|FIFO\|STATUS_REG" include src examples test tools docs` | Found core Arduino dependency, example Arduino/Wire usage, and device/register references. |
| `python tools/check_core_timing_guard.py` | `Core timing guard PASSED`. |
| `python tools/check_cli_contract.py` | `CLI contract PASSED`. |
| `python scripts/generate_version.py check` | `Version.h` up to date. |
| `python tools/check_idf_example_contract.py` | Failed: file not found. |
| `idf.py --version` | Failed: command not recognized. |
| `python -m platformio test -e native` | PASSED, 68 test cases, duration about 24.5 s. |
| `python -m platformio run -e esp32s3dev` | SUCCESS, RAM 22496/327680, Flash 412926/1310720. |
| `python -m platformio run -e esp32s2dev` | SUCCESS, RAM 36896/327680, Flash 403113/1310720. |
| `python -m platformio pkg pack` | Succeeded, wrote `LSM6DS3TR-1.1.0.tar.gz`; artifact removed after verification. |

Present coverage:

- Native tests compile and pass.
- Arduino PlatformIO builds pass for ESP32-S2 and ESP32-S3.
- Timing guard catches obvious forbidden timing constructs.
- CLI contract check passes.
- Version generation check passes.

Missing or weak coverage:

- Reset and boot timeout/error tests.
- Full failure-injection matrix for every register write in multi-register apply paths.
- Tests that BDU and IF_INC are preserved against raw writes.
- Full accel/gyro conversion tests for every full-scale value.
- FIFO high-bit/status edge cases, overrun/full behavior, and 4th dataset combinations.
- Config validation tests for missing callbacks and zero/extreme timeout.
- Copy/move prevention compile tests.
- ESP-IDF or no-Arduino compile job.
- Hardware validation tests.

## ESP-IDF Port Assessment

Current status: **not ESP-IDF ready**.

Reasons:

- Core includes Arduino headers and calls `millis()` as fallback.
- No ESP-IDF build target or example was found.
- `idf.py` is not installed in the local environment used for this audit.
- The documented or implied ESP-IDF contract checker is missing.
- No ESP-IDF transport adapter, mutex/locking contract, or bus-timeout mapping was found.

To become ESP-IDF ready:

- Remove Arduino dependencies from core.
- Add a pure C++ core compile target.
- Add an ESP-IDF example transport adapter outside the library core.
- Define callback threading/locking expectations.
- Add CI or documented local validation for ESP-IDF.

## Arduino / PlatformIO Assessment

Current status: **usable for Arduino PlatformIO engineering builds, not yet production-hardened**.

Strengths:

- `esp32s2dev` and `esp32s3dev` builds succeeded.
- The example keeps board pins and `Wire` setup outside library core.
- The CLI example provides useful diagnostic visibility.

Risks:

- The example adapter may be copied into production despite being diagnostic glue.
- Timeout detail can be degraded in the adapter.
- Example delays and logging are acceptable for examples but should be clearly separated from production guidance.

Recommended documentation change:

- Label `examples/common/` as example-only board glue and diagnostic transport code.
- Provide a short production-adapter checklist for Arduino users.

## Documentation Assessment

Documentation is extensive and useful, but it needs stricter production language.

Needed corrections:

- Say SA0 high is VDD_IO/logical high, not simply VDD.
- State I2C mode requires CS held high.
- State the library is I2C-only unless/until SPI is implemented.
- Avoid implying every datasheet feature is implemented.
- Clarify `tick()` and cached measurement freshness semantics.
- Document FIFO word-by-word latency and the lack of production-grade full-FIFO drain support.
- Add a hardware validation matrix and keep it tied to commit hashes.
- Distinguish example diagnostics from production integration code.

## Hardware Validation Needed

Minimum validation matrix before production-readiness claim:

| Area | Required validation |
| --- | --- |
| Boards | At least one ESP32-S2 and one ESP32-S3 board. |
| Device identity | Record sensor marking, WHO_AM_I, address strap, CS state. |
| Voltage/wiring | Record VDD, VDD_IO, SA0, CS, pullup values, bus length. |
| Bus speeds | Validate 100 kHz and 400 kHz; do not claim 1 MHz without datasheet-backed proof and hardware evidence. |
| Startup | Power-on begin, wrong address, missing device, wrong WHO_AM_I simulation if possible. |
| Reset/boot | Soft reset and boot flows with logic-analyzer captures. |
| Accel scales | All full-scale settings with static orientation sanity checks. |
| Gyro scales | All full-scale settings with stationary zero/noise sanity checks and rotation spot checks. |
| ODRs | Power-down, low ODR, common ODRs, and high ODRs with data-ready behavior. |
| Temperature | Room-temperature sanity and raw formula verification. |
| FIFO | Empty, watermark, full, overrun, accel-only, gyro-only, accel+gyro, and 4th dataset modes. |
| Interrupts | Data-ready and FIFO interrupts on INT1/INT2, polarity, latch/open-drain settings. |
| Fault recovery | Bus NACK/timeout injection, recover behavior, offline threshold behavior. |
| Long run | Multi-hour continuous read at selected ODRs with failure counters logged. |

Evidence to retain:

- Commit hash.
- Full config dump after `begin()`.
- Register dump after each major mode change.
- Logic-analyzer traces for WHO_AM_I, burst reads, reset, and FIFO drain.
- Serial logs with health counters and timestamps.

## Recommended Implementation Plan

### P0 — Must fix before production claim

1. Remove Arduino dependency from core and make time-source requirements explicit.
2. Add dirty-state handling for partial multi-register writes.
3. Restrict or redesign raw public register writes around bank/bus/managed registers.
4. Expand `Err` statuses for production diagnostics while preserving backward compatibility.
5. Fix FIFO 4th-dataset configuration semantics and validation.
6. Align reset/boot behavior with AN5130 and validate on hardware.
7. Add a hardware validation matrix and run it on ESP32-S2/S3.

### P1 — Should fix before release/merge

1. Delete copy/move operations for the driver class.
2. Clarify `tick()` semantics and measurement freshness.
3. Add full fault-injection tests for config/FIFO apply midpoint failures.
4. Improve Arduino example transport timeout/short-read detail.
5. Add full-scale conversion tests for every accel/gyro range.
6. Correct gyro HPF enum labels/docs.
7. Update README/API docs with I2C-only scope, CS high, VDD_IO, bus-speed, and example-only caveats.

### P2 — Nice hardening / later

1. Add an ESP-IDF adapter example after core is framework-neutral.
2. Add a bounded FIFO burst/chunk drain API.
3. Add documentation lint/checklist tooling for production claims.
4. Centralize sensitivity and ODR tables in one documented source.
5. Add release artifact hygiene around package generation.

## Proposed Future Hardening Branch

`hardening/lsm6ds3trc-industry-readiness`

Suggested branch scope:

- Implement P0 fixes first.
- Keep API-breaking work grouped and versioned intentionally.
- Add tests with each behavioral fix.
- Do not merge to a release branch until hardware validation evidence exists.

## Final Verdict

Do not implement production fixes directly on this exploration branch. Use this branch as the audit artifact and open a separate hardening branch for code changes.

The project is a strong engineering base, but it should not claim industry readiness yet. The biggest risks are portability mismatch, unsafe expert register access, partial hardware state after failures, FIFO dataset correctness, and lack of hardware validation. Fix those first, then use the hardware validation matrix to decide whether the library is ready for production deployment.
