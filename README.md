# LSM6DS3TR-C IMU Driver Library

Production-grade LSM6DS3TR-C 6-axis IMU I2C driver for ESP32-S2 / ESP32-S3 (Arduino framework, PlatformIO).

## Features

- **Injected I2C transport** — no Wire dependency in library code
- **4-state health tracking** — UNINIT → READY → DEGRADED → OFFLINE
- **Manual recovery** via `recover()` — application controls retry strategy
- **Blocking direct-read API** and non-blocking measurement via `tick()`
- **Burst read** of all 14 sensor bytes in a single I2C transaction
- **Raw-to-physics conversion** — g, dps, °C
- **Runtime reconfiguration** of ODR and full-scale
- **Software reset** with bounded polling
- **Zero heap allocation** in steady state
- **No logging** in library code

## Installation

### PlatformIO (recommended)

```ini
lib_deps =
  https://github.com/janhavelka/LSM6DS3TR.git
```

### Manual

Copy `include/LSM6DS3TR/` and `src/` into your project.

## Quick Start

```cpp
#include <Wire.h>
#include "LSM6DS3TR/LSM6DS3TR.h"

// --- Transport callbacks (map Wire errors → LSM6DS3TR::Status) ---
LSM6DS3TR::Status myWrite(uint8_t addr, const uint8_t* data, size_t len,
                          uint32_t timeoutMs, void* user) {
  auto* wire = static_cast<TwoWire*>(user);
  wire->beginTransmission(addr);
  wire->write(data, len);
  uint8_t err = wire->endTransmission(true);
  if (err == 0) return LSM6DS3TR::Status::Ok();
  if (err == 2) return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_NACK_ADDR,
                                                  "NACK on address");
  return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, "Wire error",
                                   static_cast<int32_t>(err));
}

LSM6DS3TR::Status myWriteRead(uint8_t addr, const uint8_t* wData, size_t wLen,
                               uint8_t* rData, size_t rLen,
                               uint32_t timeoutMs, void* user) {
  auto* wire = static_cast<TwoWire*>(user);
  wire->beginTransmission(addr);
  wire->write(wData, wLen);
  if (wire->endTransmission(false) != 0)
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, "Write phase failed");
  size_t got = wire->requestFrom(addr, rLen);
  if (got != rLen)
    return LSM6DS3TR::Status::Error(LSM6DS3TR::Err::I2C_ERROR, "Short read");
  for (size_t i = 0; i < rLen; ++i) rData[i] = wire->read();
  return LSM6DS3TR::Status::Ok();
}

uint32_t myNowMs(void*) { return millis(); }

// --- Setup & Loop ---
LSM6DS3TR::LSM6DS3TR imu;

void setup() {
  Wire.begin(8, 9);
  Wire.setClock(400000);

  LSM6DS3TR::Config cfg;
  cfg.i2cWrite     = myWrite;
  cfg.i2cWriteRead = myWriteRead;
  cfg.i2cUser      = &Wire;
  cfg.nowMs        = myNowMs;
  cfg.i2cAddress   = 0x6A;       // SA0 = GND
  cfg.odrXl = LSM6DS3TR::Odr::HZ_104;
  cfg.odrG  = LSM6DS3TR::Odr::HZ_104;
  cfg.fsXl  = LSM6DS3TR::AccelFs::G_2;
  cfg.fsG   = LSM6DS3TR::GyroFs::DPS_250;

  LSM6DS3TR::Status st = imu.begin(cfg);
  if (!st.ok()) { /* handle error */ }
}

void loop() {
  imu.tick(millis());

  LSM6DS3TR::RawMeasurement raw;
  if (imu.readAllRaw(raw).ok()) {
    LSM6DS3TR::Axes accel = imu.convertAccel(raw.accel);   // g
    LSM6DS3TR::Axes gyro  = imu.convertGyro(raw.gyro);     // dps
    float tempC           = imu.convertTemperature(raw.temperature);
  }
}
```

## Architecture

### Driver State Machine

```
begin() success    any I2C failure    failures >= threshold
  ─────────────> READY ──────────> DEGRADED ──────────> OFFLINE
                   ^                  │                    │
                   └──────────────────┘────────────────────┘
                       success (recover or normal op)

  end() -> UNINIT (from any state)
```

### Transport Wrapper Layers

```
Public API (readAllRaw, setAccelOdr, etc.)
    │
Register helpers (readRegs, writeRegs)
    │
TRACKED wrappers (_i2cWriteReadTracked, _i2cWriteTracked)
    │  ← _updateHealth() called here ONLY
RAW wrappers (_i2cWriteReadRaw, _i2cWriteRaw)
    │
Transport callbacks (Config::i2cWrite, i2cWriteRead)
```

## API Reference

### Lifecycle

| Method | Description |
|--------|-------------|
| `begin(config)` | Initialize driver, validate config, read WHO_AM_I, configure device |
| `tick(nowMs)` | Process pending measurement requests |
| `end()` | Shutdown, power down sensors, reset state |

### Measurement

| Method | Description |
|--------|-------------|
| `readAllRaw(out)` | Blocking burst read of temp + gyro + accel (14 bytes) |
| `readAccelRaw(out)` | Blocking read of accelerometer only (6 bytes) |
| `readGyroRaw(out)` | Blocking read of gyroscope only (6 bytes) |
| `readTemperatureRaw(out)` | Blocking read of temperature (2 bytes) |
| `requestMeasurement()` | Start non-blocking measurement (returns IN_PROGRESS) |
| `getMeasurement(out)` | Get converted measurement after tick() completes |
| `convertAccel(raw)` | Raw → g conversion |
| `convertGyro(raw)` | Raw → dps conversion |
| `convertTemperature(raw)` | Raw → °C conversion |

### Configuration

| Method | Description |
|--------|-------------|
| `setAccelOdr(odr)` | Set accelerometer output data rate |
| `setGyroOdr(odr)` | Set gyroscope output data rate |
| `setAccelFs(fs)` | Set accelerometer full-scale |
| `setGyroFs(fs)` | Set gyroscope full-scale |
| `softReset()` | Reset device registers, re-apply config |

### Health & Diagnostics

| Method | Description |
|--------|-------------|
| `state()` | Current DriverState (UNINIT/READY/DEGRADED/OFFLINE) |
| `isOnline()` | true if READY or DEGRADED |
| `probe()` | Raw WHO_AM_I check (no health tracking) |
| `recover()` | Tracked WHO_AM_I check + config re-apply |
| `lastError()` | Most recent error Status |
| `consecutiveFailures()` | Failures since last success |
| `totalSuccess()` / `totalFailures()` | Lifetime counters |

### Health Monitoring Example

```cpp
if (imu.state() == LSM6DS3TR::DriverState::OFFLINE) {
  LSM6DS3TR::Status st = imu.recover();
  if (st.ok()) {
    // Back online
  }
}

// Query health stats
Serial.printf("State: %d, Consecutive failures: %u, Total OK: %lu\n",
              static_cast<int>(imu.state()),
              imu.consecutiveFailures(),
              static_cast<unsigned long>(imu.totalSuccess()));
```

## Sensor Specifications

### Accelerometer

| Full-Scale | Sensitivity | ODR Range |
|------------|-------------|-----------|
| ±2 g | 0.061 mg/LSB | 1.6 Hz – 6.66 kHz |
| ±4 g | 0.122 mg/LSB | 12.5 Hz – 6.66 kHz |
| ±8 g | 0.244 mg/LSB | 12.5 Hz – 6.66 kHz |
| ±16 g | 0.488 mg/LSB | 12.5 Hz – 6.66 kHz |

### Gyroscope

| Full-Scale | Sensitivity | ODR Range |
|------------|-------------|-----------|
| ±125 dps | 4.375 mdps/LSB | 12.5 Hz – 6.66 kHz |
| ±250 dps | 8.75 mdps/LSB | 12.5 Hz – 6.66 kHz |
| ±500 dps | 17.50 mdps/LSB | 12.5 Hz – 6.66 kHz |
| ±1000 dps | 35 mdps/LSB | 12.5 Hz – 6.66 kHz |
| ±2000 dps | 70 mdps/LSB | 12.5 Hz – 6.66 kHz |

### Temperature

- Resolution: 256 LSB/°C
- Offset: 0 LSB at 25°C
- Formula: `T(°C) = raw / 256 + 25`

## I2C Address

| SA0 Pin | Address |
|---------|---------|
| GND | 0x6A |
| VDD | 0x6B |

## Examples

| Directory | Description |
|-----------|-------------|
| `01_basic_bringup_cli/` | Interactive CLI for testing all driver features |

### Example Helpers (`examples/common/`)

Not part of the library. These simulate project-level glue and keep examples self-contained:

| File | Description |
|------|-------------|
| `BoardConfig.h` | Board-specific pin/frequency defaults (SDA=8, SCL=9, 400 kHz) |
| `BuildConfig.h` | Compile-time LOG_LEVEL configuration |
| `Log.h` | Serial logging macros (LOGE/LOGW/LOGI/LOGD/LOGT) |
| `I2cTransport.h` | Wire-based I2C transport adapter with error mapping |
| `I2cScanner.h` | I2C bus scanner utility |

## Behavioral Contracts

1. **Threading model:** Single-threaded; call `tick()` and all API from the same context. Not ISR-safe.
2. **Timing model:** `tick()` does bounded work (one status read + one burst read max)
3. **Resource ownership:** I2C bus owned by application; library uses injected callbacks
4. **Memory behavior:** All allocation in `begin()`; zero allocation in steady state
5. **Error handling:** All fallible APIs return `Status`; check with `st.ok()`

## Building & Testing

```bash
# ESP32-S3 build
pio run -e esp32s3dev

# ESP32-S2 build
pio run -e esp32s2dev

# Native host tests
pio test -e native
```

## Documentation

- [CHANGELOG.md](CHANGELOG.md) — Release history
- [CONTRIBUTING.md](CONTRIBUTING.md) — Contribution guidelines
- [docs/datasheet_LSM6DS3TR-C.pdf](docs/datasheet_LSM6DS3TR-C.pdf) — LSM6DS3TR-C datasheet (DocID030071)
- [docs/application_note.pdf](docs/application_note.pdf) — Application note AN5130
- `docs/design_tips/` — Calibration, tilt, dead reckoning, noise analysis design tips

## License

MIT License. See [LICENSE](LICENSE).
