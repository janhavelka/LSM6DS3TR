# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Completed Doxygen contracts for the owner-safe public types, helpers,
  operations, result provenance, and diagnostic boundary.
- Made undocumented public API, missing parameter documentation, and Doxygen
  documentation errors fail both clean local generation and CI.
- Consolidated ESP-IDF guidance into `docs/IDF_PORT.md`, removed the duplicate
  implementation note, included the maintained guide in package exports, and
  clarified documentation validation and ownership language across the README
  and contributor guidance.
- Added a documentation map that distinguishes maintained contracts from
  source extracts, generated pages, and historical hardware evidence.

## [2.0.0] - 2026-07-19

### Added

- Zero-I2C `bind()`/`unbind()` lifecycle with an application-owned
  `DriverConfig` transport binding.
- One fixed-memory operation model for probe, configuration, sampling, reset,
  boot, recovery, reconciliation, power-down, self-test, calibration, and
  destructive FIFO purge.
- Absolute 64-bit operation deadlines, caller-selected transport budgets,
  hard total callback ceilings, bus-silent cancellation, 64-bit nonzero
  operation tokens, and exactly-once terminal result delivery.
- Atomic raw sample results with validity/freshness masks, a 64-bit sequence,
  configuration generation, read uptime, and immutable full-scale provenance.
- Explicit configuration states, verified profile readback, settling gates,
  mismatch diagnostics, and ambiguous/partial hardware-effect reporting.
- Allocation-free fixed-unit conversion, sensitivity, timing, validation, and
  calibration helpers independent of mutable driver state.
- Real ESP-IDF 5.4.0 CI builds for ESP32-S2 and ESP32-S3.

### Changed

- Replaced the managed synchronous/offline driver with an externally scheduled
  owner-safe driver. Applications own bus locking, retries, health, backoff,
  and recovery policy.
- Production configuration is one replayable `DeviceProfile`. Version 2
  supports polling snapshots and requires FIFO plus interrupts disabled.
- Reset, boot, self-test, and calibration are staged operations with visible
  waits and restoration outcomes.
- Self-test and calibration enforce three-check per-sample readiness limits and
  zero-I2C sampling cadence; self-test reserves its full restoration budget
  and explicitly wakes then restores a configured sleeping gyro.
- Poll progress now includes cumulative callback use and its hard ceiling.
  Self-test exposes a failed primary status at a bus-silent poll boundary before
  a later poll begins restoration.
- Public callback ceilings are probe 2, configure 68, sample 66, reset/boot 88,
  recover 87, reconcile 35, and power-down 8 worst case (6 from the main bank).
  Self-test remains `16 * (samples + 5) + 80`, calibration is `4 * samples`,
  and FIFO purge is `maxWords + 5`.
- Ready-checked sampling spaces unsuccessful status reads by the slowest
  requested cadence instead of a fixed 1 ms loop. Motion follows its configured
  ODR; temperature follows the AN5130 12.5/26/52 Hz cases and remains available
  from a non-power-down sleeping gyro.
- Calibration requires an explicit expected acceleration vector instead of
  assuming a Z-up product installation.
- Diagnostic raw register access is one-transaction, excluded during active
  jobs, and invalidates configuration provenance on accepted writes. Writes
  now enforce per-register masks and reject reserved bits or illegal encodings
  before I2C.
- Arduino and native ESP-IDF examples now demonstrate the same compact
  token/start/poll/cancel/take command surface with fixed input buffers.
- The examples reject entire overlength command lines and validate sample and
  calibration arguments exactly instead of executing truncated/defaulted input.
- PlatformIO Core, pioarduino platform archive, and CI ESP-IDF version are
  pinned. The IDF component supports the ESP-IDF 5.4 line.
- Library packages use an explicit minimal export whitelist, with archive
  contents enforced by a CI contract check.

### Fixed

- Prevented stale results from being attributed to a later request through
  token correlation and exactly-once consumption.
- Prevented cached raw samples from being converted with a newer full-scale
  setting by carrying scale provenance in each sample.
- Prevented interpreted measurements while configuration is unknown or sensor
  output is settling.
- Enforced BDU for managed multi-byte samples and independent validity of
  acceleration, angular-rate, and temperature fields, including TDA readiness
  for every request containing temperature.
- Enforced the 208 Hz low-power/normal ceiling for both sensors and made
  power-down available from any bound idle state while proving only exact zero
  accelerometer/gyro ODR registers and leaving configuration unconfigured.
- Partitioned total callback limits so sampling reserves its burst and
  reset/boot/recovery reserve a complete profile replay/readback budget.
- Made configure validate WHO_AM_I before its first write and made positive
  chip-ID mismatches from probe, configure, reset, boot, reconcile, recovery,
  and FIFO purge invalidate prior verified provenance. Reconcile checks
  identity before managed readback.
- Added main-register-bank prechecks before identity-dependent operations.
  Reset/boot verify an explicit bank clear; power-down does so conditionally
  when its precheck finds an alternate bank, then validates identity before its
  ODR writes. Public ceilings include these callbacks.
- Added the required bus-silent boot/reset inaccessible interval and verified
  profile replay/readback.
- Rejected non-finite or invalid calibration inputs and removed the public
  `DISABLED`, `LOW`, and `HIGH` macro collisions.
- Rejected gyroscope bias calibration while the verified gyro is sleeping,
  without performing I2C.
- Corrected the sensor-sync register addresses to `TIME_FRAME=0x04` and
  `RES_RATIO=0x05`. Sensor-sync and DRDY-pulse controls are now managed and
  verified as zero; diagnostic writes invalidate provenance for reconciliation.
- Corrected gyro high-pass cutoff names/encodings as register facts and made
  high-pass production profiles explicitly unsupported because authoritative
  settling tables exclude them. Also restricted offset and snapshot-profile
  combinations whose register effects were not safely represented.
- Made settling calculations account for documented accelerometer discard
  counts and gyro turn-on/filter delay instead of using a coarse fixed bound.
- Made self-test use at least five averaged samples per phase, temporarily
  remove user-offset effects, establish the required opposite-sensor modes,
  preserve the exact original profile, and reserve the corrected restoration
  ceiling.
- Made conversion reject internally inconsistent raw provenance while carrying
  sample quality into converted output.
- Made FIFO purge verify device identity and IF_INC/BDU before consuming data,
  and treat an overrun's zero encoded unread count as full/data-lost state; it
  also verifies the main bank and its public ceiling is `maxWords + 5`.
- Made passive diagnostic time identify the last transport error specifically,
  rather than moving on successful transfers.
- Synchronized `library.json`, `Version.h`, `idf_component.yml`, and Doxygen
  project versions from one source.
- Removed unrelated TunnelMonitor dependency-pin generation from the library's
  version metadata script.

### Removed

- Synchronous `begin()`/`end()`/`recover()`, `tick()`, synchronous setters and
  reads, cached measurement getters, driver-owned `OFFLINE` admission policy,
  and blocking self-test/calibration APIs.
- The incomplete FIFO acquisition/configuration surface and raw interrupt/event
  configuration claims. FIFO purge remains explicitly destructive maintenance.
- Stale broad CLI helpers and version 1 command-contract requirements.
- The version 1-only HIL runner; its retained evidence is historical and a new
  version 2 physical test campaign is required.

### Documentation

- Re-audited every TunnelMonitor suitability finding against the version 2
  API and the authoritative local TunnelMonitor-node ownership/capacity
  contracts.
- Documented operation classes, transaction/deadline behavior, concurrency,
  ISR restrictions, cancellation, ambiguous effects, provenance, migration,
  and remaining product/HIL decisions.

## [1.2.0] - 2026-06-25

### Added

- ESP-IDF component metadata (`CMakeLists.txt`, `idf_component.yml`) for the
  framework-neutral driver core.
- Native ESP-IDF basic example using `app_main`, fixed-buffer CLI input, and
  ESP-IDF `driver/i2c_master.h` callbacks.
- IDF example contract check covering native command coverage, required ESP-IDF
  components, native I2C symbols, and absence of Arduino compatibility facades.
- `Err::I2C_BUSY`, `Err::FIFO_OVERRUN`, `Err::CALIBRATION_UNSTABLE`, and
  `Err::CALIBRATION_ORIENTATION`.
- `CalibrationLimits` in `Config` plus blocking/staged calibration quality
  checks that reject unstable or wrongly oriented captures without updating bias.
- Core `SelfTestResult` and `runSelfTest()` API for bounded accelerometer and
  gyroscope self-test.

### Changed

- `library.json` now advertises both Arduino and ESP-IDF framework support.
- Removed the private platform time shim; the core now relies on injected
  `Config::nowMs` and otherwise uses 0 for timestamps.
- Staged sample jobs now arm ready deadlines from the first positive-budget
  `poll(nowMs, ...)` that executes the status-read step, and poll-completed
  samples use the raw-burst poll timestamp.
- FIFO convenience reads now report overrun explicitly and treat zero unread
  words as empty even if the empty flag is not set.
- Arduino and ESP-IDF CLI `selftest` commands now call the core self-test API.

### Fixed

- Transport callback `IN_PROGRESS` leakage is normalized to `I2C_BUSY` and
  tracked as a transport failure.
- README transport callback sample now applies `timeoutMs` and points to the
  tested example transport helper.

### Documentation

- Documented the staged poll clock contract, `poll(..., 0)` no-progress
  behavior, CLI job grammar, job cancel caveat, HIL evidence, and remaining
  release gates.

## [1.1.0] - 2026-05-17

### Added

- `SettingsSnapshot`, `getSettings()`, `settings()`, `driverState()`, `hasSample()`, `sampleTimestampMs()`, and `sampleAgeMs()` for cache-only diagnostics.
- `StatusReg` / `readStatus(StatusReg&)` and `SensorHubData` / `readSensorHub()` for decoded status and sensor-hub output readback.
- `Err::CONVERSION_NOT_READY` alias and `Status::is(Err)` for cross-library status handling.
- CLI commands for `begin`, `whoami` / `id`, `shub [N]`, decoded `status`, and expanded FIFO status output.
- CLI diagnostics for `steps`, `funcsrc1`, `funcsrc2`, and `wtstatus`, including decoded pedometer, embedded-function, wrist-tilt, sensor-hub, and slave-NACK source flags.
- Command-table masks for `FUNC_SRC1`, `FUNC_SRC2`, and `WRIST_TILT_IA` source-bit decoding.
- Native coverage proving latched `OFFLINE` blocks normal I2C operations without touching the bus while explicit recovery/reset paths remain available.
- Native coverage asserting exact datasheet `CTRL10_C` writes for timestamp, pedometer, significant motion, tilt, wrist tilt, and step-counter reset.
- README documentation for the single-threaded, non-ISR driver contract and explicit recovery model.

### Changed

- Doxyfile inputs now include the implementation manual while keeping extracted
  math-heavy design tips out of generated API docs.
- Reference documentation now uses human-readable vendor PDF names and separates compact IMU notes from full PDF/application-note extractions under `docs/extracted-md/` and `docs/pdf-extracted-md/`.
- Explicit recovery/reset bypass internals now use the shared `ScopedOfflineI2cAllowance` / `_reassertOfflineLatch()` procedure so failed recovery attempts that begin from `OFFLINE` keep the latch asserted.
- Hardened cached configuration setters so invalid enum values are rejected before I2C and cached state rolls back on failed writes.
- `begin()` now clears stale health, config, feature, and sample state before validation, and normalizes `offlineThreshold = 0` to one.
- `IN_PROGRESS` statuses are health-neutral.
- Documented direct-register access bounds and bounded calibration/reset polling behavior.
- Health behavior is now standardized on latched `OFFLINE`: normal public I2C operations return `BUSY` with `Driver is offline; call recover()` and do not touch I2C until `recover()` succeeds.
- CLI `odrxl` now bridges power-mode transitions automatically when moving into or out of accel-only 1.6 Hz low-power operation.
- CLI `stress` now uses independent data-ready polling for active accel/gyro channels, reports per-channel sample rates, stops reading channels once their target is reached, and keeps progress counts tied to the requested target.
- CLI stress progress keeps coloring limited to `ok=` and `fail=` values.

### Fixed

- Made accel/gyro bias capture exit with `TIMEOUT` even if the injected millisecond source stalls while data-ready never asserts.
- Made `recover()` record chip-ID mismatches in health tracking and made reset/boot raw polling failures update health.
- Rejected out-of-range public raw register addresses and invalid register blocks before touching the bus.
- Corrected `CTRL10_C` embedded-function bit positions so timestamp, pedometer, significant motion, tilt, wrist tilt, and step reset write the datasheet-defined values.
- Fixed pedometer enable behavior observed on hardware: `pedo 1` now enables the durable step counter path instead of setting the wrong control bits.
- Improved invalid ODR/power-mode error messages for accel and gyro low-power constraints.
- Removed the async combined-measurement restriction from CLI `stress` by reading active ready channels independently, so mismatched accel/gyro ODRs can be stress-tested.
- Prevented temperature-ready events from masking accel/gyro stress timeouts.

## [1.0.0] - 2026-04-06

### Added

- Initial full-featured release of the LSM6DS3TR-C IMU driver library
- Managed synchronous driver with 4-state health tracking (UNINIT, READY, DEGRADED, OFFLINE)
- Transport-agnostic I2C interface via function pointer callbacks
- Full register map in `CommandTable.h`
- Accelerometer support: +/-2g, +/-4g, +/-8g, +/-16g full-scale, 1.6 Hz to 6.66 kHz ODR
- Gyroscope support: +/-125, +/-250, +/-500, +/-1000, +/-2000 dps full-scale, 12.5 Hz to 6.66 kHz ODR
- Temperature sensor readout (256 LSB/degC, +25 degC offset)
- Blocking direct-read API (`readAccelRaw`, `readGyroRaw`, `readTemperatureRaw`, `readAllRaw`)
- Non-blocking measurement API via `requestMeasurement()` + `tick()` + `getMeasurement()`
- Burst read of all 14 bytes (temp + gyro + accel) in a single I2C transaction
- Raw-to-physical-unit conversion helpers for g, dps, and degC
- Runtime configuration of ODR and full-scale via `setAccelOdr()` / `setGyroOdr()` / `setAccelFs()` / `setGyroFs()`
- Managed support for timestamp control, step counter access, embedded motion features, FIFO configuration, source-register reads, and direct register/block access
- Software bias calibration API: `setAccelBias()` / `setGyroBias()` / `accelBias()` / `gyroBias()`
- Blocking at-rest bias capture via `captureAccelBias(samples)` and `captureGyroBias(samples)` with bounded data-ready polling
- In-place bias correction helpers: `correctAccel()` / `correctGyro()`
- Software reset with bounded polling
- Probe (raw, no health tracking) and recover (tracked) diagnostics
- Block Data Update (BDU) and auto-increment (IF_INC) enabled by default
- Example CLI application (`01_basic_bringup_cli`) with calibration commands, FIFO control, register diagnostics, source-register reads, stress tests, and continuous streaming
- Common example helper headers and repository contract checks aligned with the other production I2C libraries in this workspace
- Native host tests with FakeBus coverage for lifecycle, validation, timestamp/FIFO helpers, direct-register refresh, failed-begin recovery, and bias workflows
- `generate_version.py` for automatic `Version.h` generation

### Changed

- Standardized the bringup CLI around the workspace family conventions with broader command coverage, version output, register dump/read/write commands, mixed stress testing, and real hardware self-test flow
- Converted CLI sample output to one line per sample and aligned converted CLI reads with software bias correction
- Expanded README coverage to document managed features, CLI usage, validation rules, and the managed-vs-raw split for advanced chip functionality
- Cleaned repository documentation links so they resolve correctly on GitHub instead of using machine-specific local paths

### Fixed

- Enforced documented accelerometer and gyroscope ODR versus power-mode constraints
- Enforced documented dependencies for timestamp, embedded functions, FIFO use, and async combined measurements
- Fixed `recover()` after a failed `begin()` by copying stored config before retrying initialization
- Added sensor-sync register constants (corrected in 2.0.0).
- Made public headers safe to include from Arduino translation units by removing the `DISABLED` macro collision around `FifoDecimation`

[Unreleased]: https://github.com/janhavelka/LSM6DS3TR/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/janhavelka/LSM6DS3TR/compare/v1.2.0...v2.0.0
[1.2.0]: https://github.com/janhavelka/LSM6DS3TR/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/janhavelka/LSM6DS3TR/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/janhavelka/LSM6DS3TR/releases/tag/v1.0.0
