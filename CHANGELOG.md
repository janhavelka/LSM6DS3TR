# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Managed support for timestamp control, step counter access, embedded motion features, FIFO configuration, source-register reads, and direct register/block access
- Common example helper headers and repo contract checks aligned with the stronger sibling I2C libraries in this workspace
- Native coverage for power-mode validation, timestamp and FIFO helpers, direct-register refresh behavior, command-table corrections, and failed-begin recovery

### Changed

- Standardized the bringup CLI around the workspace family conventions with broader command coverage, version output, register dump/read/write commands, mixed stress testing, and real hardware self-test flow
- Expanded README coverage to document managed features, CLI usage, validation rules, and the managed-vs-raw split for advanced chip functionality

### Fixed

- Enforced documented accelerometer and gyroscope ODR versus power-mode constraints
- Enforced documented dependencies for timestamp, embedded functions, FIFO use, and async combined measurements
- Fixed `recover()` after a failed `begin()` by copying stored config before retrying initialization
- Fixed `REG_SENSOR_SYNC_TIME_FRAME` register address to `0x02`
- Made public headers safe to include from Arduino translation units by removing the `DISABLED` macro collision around `FifoDecimation`

## [1.0.0] - 2026-04-04

### Added

- Initial release of LSM6DS3TR-C IMU driver library
- Managed synchronous driver with 4-state health tracking (UNINIT, READY, DEGRADED, OFFLINE)
- Transport-agnostic I2C interface via function pointer callbacks
- Full register map in CommandTable.h
- Accelerometer support: ±2g/±4g/±8g/±16g full-scale, 1.6 Hz to 6.66 kHz ODR
- Gyroscope support: ±125/±250/±500/±1000/±2000 dps full-scale, 12.5 Hz to 6.66 kHz ODR
- Temperature sensor readout (256 LSB/°C, 25°C offset)
- Blocking direct-read API (readAccelRaw, readGyroRaw, readTemperatureRaw, readAllRaw)
- Non-blocking measurement API via requestMeasurement() + tick() + getMeasurement()
- Burst read of all 14 bytes (temp + gyro + accel) in single I2C transaction
- Raw-to-physical-unit conversion helpers (g, dps, °C)
- Runtime configuration of ODR and full-scale via setAccelOdr/setGyroOdr/setAccelFs/setGyroFs
- Software reset with bounded polling
- Probe (raw, no health tracking) and recover (tracked) diagnostics
- Block Data Update (BDU) and auto-increment (IF_INC) enabled by default
- Native host tests with FakeBus pattern
- Example CLI application (01_basic_bringup_cli)
- generate_version.py for automatic Version.h generation

[Unreleased]: https://github.com/janhavelka/LSM6DS3TR/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/janhavelka/LSM6DS3TR/releases/tag/v1.0.0
