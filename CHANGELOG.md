# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- ESP-IDF component metadata (`CMakeLists.txt`, `idf_component.yml`) for the
  framework-neutral driver core.
- Pure ESP-IDF basic example that reuses the Arduino bringup CLI source through
  an example-local `Serial` / `String` / `TwoWire` compatibility facade backed
  by the v6 `driver/i2c_master.h` API.
- IDF example contract check covering shared CLI source inclusion, required
  ESP-IDF components, native I2C symbols, and stale example wording.
- Private platform time shim so the core can compile under Arduino, ESP-IDF,
  and native tests without including `Arduino.h` in the driver implementation.

### Changed

- `library.json` now advertises both Arduino and ESP-IDF framework support.
- Core timing guard now allows Arduino timing only inside the private timing
  shim instead of the main driver implementation.

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
- Fixed `REG_SENSOR_SYNC_TIME_FRAME` register address to `0x02`
- Made public headers safe to include from Arduino translation units by removing the `DISABLED` macro collision around `FifoDecimation`

[Unreleased]: https://github.com/janhavelka/LSM6DS3TR/compare/v1.1.0...HEAD
[1.1.0]: https://github.com/janhavelka/LSM6DS3TR/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/janhavelka/LSM6DS3TR/releases/tag/v1.0.0
