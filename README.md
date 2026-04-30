# LSM6DS3TR-C Driver Library

Production-grade LSM6DS3TR-C 6-axis IMU driver for ESP32-S2 / ESP32-S3 using the Arduino framework and PlatformIO.

## Overview

This library follows the same managed synchronous pattern used by the stronger I2C libraries in this workspace:

- injected I2C transport, no direct `Wire` dependency in library code
- explicit `Status` return values on every fallible operation
- 4-state health tracking: `UNINIT`, `READY`, `DEGRADED`, `OFFLINE`
- deterministic library behavior with bounded polling only
- blocking register and sample reads, plus a tick-driven non-blocking measurement path
- no steady-state heap allocation and no logging inside the library

The implementation is cross-checked against the repository's device documentation:

- [LSM6DS3TR_imu_implementation_manual.md](LSM6DS3TR_imu_implementation_manual.md)
- [docs/datasheet_LSM6DS3TR-C.pdf](docs/datasheet_LSM6DS3TR-C.pdf)
- [docs/application_note.pdf](docs/application_note.pdf)

## Managed Feature Coverage

The managed API covers the most practical runtime features of the chip:

- WHO_AM_I probe and chip ID verification
- accelerometer full-scale: `+/-2g`, `+/-4g`, `+/-8g`, `+/-16g`
- gyroscope full-scale: `+/-125`, `+/-250`, `+/-500`, `+/-1000`, `+/-2000 dps`
- accelerometer and gyroscope ODR selection
- accelerometer and gyroscope power-mode selection
- gyro sleep enable
- accelerometer LPF2, slope/high-pass, and low-pass-on-6D controls
- gyroscope LPF1, HPF enable, and HPF mode controls
- timestamp enable, high-resolution mode, read, and reset
- pedometer, significant motion, tilt, and wrist tilt enable
- step counter read/reset and step timestamp read
- accelerometer user offsets and offset weight selection
- software bias calibration for accel (1-point, Z-up) and gyro (zero-rate-level) with at-rest capture
- automatic bias correction in `getMeasurement()`; manual helpers `correctAccel()` / `correctGyro()`
- FIFO configuration, status readout, and FIFO word reads
- raw sample reads, converted sample helpers, and tick-driven async measurement
- direct single-register and block register access for advanced diagnostics
- source register reads for wake-up, tap, 6D, embedded function, and wrist-tilt status

Advanced interrupt routing, threshold tuning, tap/wake/free-fall configuration, and sensor-hub flows remain available through the raw register APIs and the CLI register commands rather than dedicated managed wrappers.

## Important Runtime Rules

The driver enforces several device constraints from the datasheet and application note:

- accelerometer `1.6 Hz` is allowed only in `LOW_POWER_NORMAL`
- accelerometer `LOW_POWER_NORMAL` supports only `POWER_DOWN`, `1.6`, `12.5`, `26`, `52`, `104`, and `208 Hz`
- gyroscope `LOW_POWER_NORMAL` supports only `POWER_DOWN`, `12.5`, `26`, `52`, `104`, and `208 Hz`
- gyroscope `1.6 Hz` is rejected
- embedded functions require accelerometer ODR `>= 26 Hz`
- timestamp requires at least one active sensor
- FIFO configuration requires BDU enabled
- async combined measurements require BDU and matching active accel/gyro ODR
- managed setters validate enum values before I2C and keep cached state unchanged if a write fails
- software-reset, boot, and calibration waits use bounded polling; transport failures during raw reset/boot polling are recorded in driver health

## Installation

### PlatformIO

```ini
lib_deps =
  https://github.com/janhavelka/LSM6DS3TR.git
```

### Manual

Copy [include/LSM6DS3TR](include/LSM6DS3TR) and [src](src) into your project.

## Quick Start

```cpp
#include <Wire.h>
#include "LSM6DS3TR/LSM6DS3TR.h"

LSM6DS3TR::Status myWrite(uint8_t addr, const uint8_t* data, size_t len,
                          uint32_t timeoutMs, void* user) {
  auto* wire = static_cast<TwoWire*>(user);
  wire->beginTransmission(addr);
  wire->write(data, len);
  const uint8_t rc = wire->endTransmission(true);
  if (rc == 0) return LSM6DS3TR::Status::Ok();
  if (rc == 2) return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_NACK_ADDR, "addr nack", rc);
  if (rc == 3) return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_NACK_DATA, "data nack", rc);
  if (rc == 4) return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_BUS, "bus error", rc);
  if (rc == 5) return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_TIMEOUT, "timeout", rc);
  return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, "wire error", rc);
}

LSM6DS3TR::Status myWriteRead(uint8_t addr, const uint8_t* tx, size_t txLen,
                              uint8_t* rx, size_t rxLen, uint32_t timeoutMs, void* user) {
  auto* wire = static_cast<TwoWire*>(user);
  wire->beginTransmission(addr);
  wire->write(tx, txLen);
  if (wire->endTransmission(false) != 0) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, "write phase failed");
  }
  if (wire->requestFrom(addr, static_cast<uint8_t>(rxLen)) != rxLen) {
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, "short read");
  }
  for (size_t i = 0; i < rxLen; ++i) {
    rx[i] = static_cast<uint8_t>(wire->read());
  }
  return LSM6DS3TR::Status::Ok();
}

uint32_t myNowMs(void*) {
  return millis();
}

LSM6DS3TR::LSM6DS3TR imu;

void setup() {
  Wire.begin(8, 9);
  Wire.setClock(400000);

  LSM6DS3TR::Config cfg;
  cfg.i2cWrite = myWrite;
  cfg.i2cWriteRead = myWriteRead;
  cfg.i2cUser = &Wire;
  cfg.nowMs = myNowMs;
  cfg.i2cAddress = 0x6A;
  cfg.odrXl = LSM6DS3TR::Odr::HZ_104;
  cfg.odrG = LSM6DS3TR::Odr::HZ_104;
  cfg.fsXl = LSM6DS3TR::AccelFs::G_2;
  cfg.fsG = LSM6DS3TR::GyroFs::DPS_250;

  const LSM6DS3TR::Status st = imu.begin(cfg);
  if (!st.ok()) {
    // handle error
  }
}

void loop() {
  imu.tick(millis());

  LSM6DS3TR::RawMeasurement raw;
  if (imu.readAllRaw(raw).ok()) {
    const LSM6DS3TR::Axes accel = imu.convertAccel(raw.accel);
    const LSM6DS3TR::Axes gyro = imu.convertGyro(raw.gyro);
    const float tempC = imu.convertTemperature(raw.temperature);
    (void)accel;
    (void)gyro;
    (void)tempC;
  }
}
```

## CLI Example

The main example is [examples/01_basic_bringup_cli/main.cpp](examples/01_basic_bringup_cli/main.cpp). It exposes:

- probe, recover, health, version, and I2C bus scan commands
- raw and scaled sample reads
- full managed configuration coverage for ODR, full-scale, power modes, filters, timestamp, embedded functions, offsets, and FIFO
- register read, write, and dump commands for advanced tuning
- source-register inspection commands
- async stress and mixed-operation stress commands
- hardware self-test flow for accelerometer and gyroscope
- software bias calibration and continuous streaming mode with one line per sample
- converted CLI sample reads (`read`, `accel`, `gyro`, `stream`) that respect the currently configured software bias values

Representative commands:

```text
probe
cfg
odrxl 104
gpm lpn
ts 1
pedo 1
offset -4 7 12
cal 200
biasxl
biasg
stream
fifo_mode cont
fifo_odr 104
fifo_read 8
rreg 0x10
wreg 0x58 0x8E
dump 0x10 32
selftest
```

## Example Helpers

Files under [examples/common](examples/common) are example-only glue, not part of the library API:

- `BoardConfig.h`
- `BuildConfig.h`
- `Log.h`
- `I2cTransport.h`
- `TransportAdapter.h`
- `BusDiag.h`
- `CliShell.h`
- `HealthView.h`
- `I2cScanner.h`
- `CommandHandler.h`

## Direct Register Access

Use the raw register APIs when you need chip features that are intentionally left at register level:

- `readRegisterValue()`
- `writeRegisterValue()`
- `readRegisterBlock()`
- `refreshCachedConfig()`

Public raw access is limited to the main user-register window through `Z_OFS_USR`; zero-length, oversized, and wrapping block reads are rejected before the bus is touched. Writes to managed configuration registers refresh the cached configuration after the write succeeds.

This is the intended path for interrupt routing, threshold registers, advanced tap/wake/free-fall setup, and sensor-hub experimentation.

## Building And Validation

```bash
pio test -e native
pio run -e esp32s3dev
pio run -e esp32s2dev
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
```

## Repository Notes

- Public headers live in [include/LSM6DS3TR](include/LSM6DS3TR)
- Implementation lives in [src/LSM6DS3TR.cpp](src/LSM6DS3TR.cpp)
- Version metadata is generated into [include/LSM6DS3TR/Version.h](include/LSM6DS3TR/Version.h) from [library.json](library.json)
- `examples/common` is not installed as part of the library
- The library never configures I2C pins or owns the bus
