# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/janhavelka/LSM6DS3TR/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/janhavelka/LSM6DS3TR/releases/tag/v1.0.0
