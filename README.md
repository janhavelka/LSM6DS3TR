# LSM6DS3TR-C Driver Library

Production-grade LSM6DS3TR-C 6-axis IMU driver for ESP32-S2 / ESP32-S3 using Arduino/PlatformIO or ESP-IDF.

## Overview

This library follows the same managed synchronous pattern used by the stronger I2C libraries in this workspace:

- injected I2C transport, no direct `Wire` dependency in library code
- framework-neutral core with Arduino and ESP-IDF application-owned transports
- explicit `Status` return values on every fallible operation
- 4-state health tracking: `UNINIT`, `READY`, `DEGRADED`, `OFFLINE`
- deterministic library behavior with bounded polling only
- blocking register and sample reads, plus a tick-driven non-blocking measurement path
- no steady-state heap allocation and no logging inside the library

The implementation is cross-checked against the repository's device documentation:

- [LSM6DS3TR_imu_implementation_manual.md](LSM6DS3TR_imu_implementation_manual.md)
- [docs/LSM6DS3TR-C_datasheet.pdf](docs/LSM6DS3TR-C_datasheet.pdf)
- [docs/LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.pdf](docs/LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.pdf)

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
- decoded source-register constants for step, significant-motion, tilt, wrist-tilt,
  sensor-hub, and slave-NACK diagnostics
- accelerometer user offsets and offset weight selection
- software bias calibration for accel (1-point, Z-up) and gyro (zero-rate-level) with at-rest capture
- configurable calibration stillness/orientation limits that reject bad captures before updating bias
- automatic bias correction in `getMeasurement()`; manual helpers `correctAccel()` / `correctGyro()`
- FIFO configuration, status readout, FIFO word reads, and explicit FIFO overrun reporting
- bounded core accelerometer/gyroscope self-test via `runSelfTest()`
- raw sample reads, converted sample helpers, and tick-driven async measurement
- direct single-register and block register access for advanced diagnostics
- source register reads for wake-up, tap, 6D, embedded function, and wrist-tilt status

Advanced interrupt routing, threshold tuning, tap/wake/free-fall configuration,
and deeper sensor-hub setup remain available through the raw register APIs and
the CLI register commands. Basic sensor-hub output readback is available through
`SensorHubData` and `readSensorHub()`.

## Important Runtime Rules

The driver enforces several device constraints from the datasheet and application note:

- accelerometer `1.6 Hz` is allowed only in `LOW_POWER_NORMAL`
- accelerometer `LOW_POWER_NORMAL` supports only `POWER_DOWN`, `1.6`, `12.5`, `26`, `52`, `104`, and `208 Hz`
- gyroscope `LOW_POWER_NORMAL` supports only `POWER_DOWN`, `12.5`, `26`, `52`, `104`, and `208 Hz`
- gyroscope `1.6 Hz` is rejected
- embedded functions require accelerometer ODR `>= 26 Hz`
- timestamp requires at least one active sensor
- pedometer step events are short pulses unless interrupt latching/routing is
  configured; the step counter is the durable pedometer readback
- FIFO configuration requires BDU enabled
- FIFO convenience reads return `FIFO_EMPTY` when no unread words are available and `FIFO_OVERRUN` when the FIFO overrun flag is observed
- async combined measurements require BDU and matching active accel/gyro ODR
- managed setters validate enum values before I2C and keep cached state unchanged if a write fails
- transport callbacks must be synchronous and timeout-bounded; driver-owned staged jobs are the only APIs that return `IN_PROGRESS`
- `Config::offlineThreshold = 0` is normalized to one; failed `begin()` clears
  cached config, feature flags, samples, and health before validation.
- software-reset, boot, and calibration waits use bounded polling; reset/boot timeouts and transport failures during raw polling are recorded in driver health

## Thread, ISR, And Recovery Model

- The driver is single-threaded: call public APIs from one task or loop context, or serialize access externally.
- Do not call I2C-backed APIs from ISRs. Use an interrupt only to set an application flag, then call the driver from normal task context.
- `OFFLINE` is latched. Normal public I2C operations return `BUSY` with `Driver is offline; call recover()` and do not touch the bus.
- `probe()` remains a raw diagnostic check and does not update health counters. `recover()`, `softReset()`, and `boot()` are explicit recovery/reset paths and may access I2C while offline.

## Installation

### PlatformIO

```ini
lib_deps =
  https://github.com/janhavelka/LSM6DS3TR.git
```

### Manual

Copy [include/LSM6DS3TR](include/LSM6DS3TR) and [src](src) into your project.

### ESP-IDF

Use this repository as an ESP-IDF component with `EXTRA_COMPONENT_DIRS` or the
ESP Component Registry metadata in `idf_component.yml`. The component builds
only [src/LSM6DS3TR.cpp](src/LSM6DS3TR.cpp) and public headers. Applications
own the I2C bus and provide `Config::i2cWrite`, `Config::i2cWriteRead`, and
optionally `Config::nowMs`.

## Quick Start

```cpp
#include <Wire.h>
#include "LSM6DS3TR/LSM6DS3TR.h"

LSM6DS3TR::Status myWrite(uint8_t addr, const uint8_t* data, size_t len,
                          uint32_t timeoutMs, void* user) {
  auto* wire = static_cast<TwoWire*>(user);
  wire->setTimeOut(timeoutMs);
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
  wire->setTimeOut(timeoutMs);
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

The full Arduino example transport in
[examples/common/I2cTransport.h](examples/common/I2cTransport.h) clamps
`timeoutMs` for `Wire`, maps Wire error codes to `Status`, and keeps transport
policy outside the driver.

## CLI Example

The main example is [examples/01_basic_bringup_cli/main.cpp](examples/01_basic_bringup_cli/main.cpp). It exposes:

- probe, recover, health, version, and I2C bus scan commands
- raw and scaled sample reads
- full managed configuration coverage for ODR, full-scale, power modes, filters, timestamp, embedded functions, offsets, and FIFO
- register read, write, and dump commands for advanced tuning
- source-register inspection commands
- ODR-paced independent data-ready stress and mixed-operation stress commands
- core `runSelfTest()` hardware self-test flow for accelerometer and gyroscope
- software bias calibration and continuous streaming mode with one line per sample
- converted CLI sample reads (`read`, `accel`, `gyro`, `stream`) that respect the currently configured software bias values
- decoded `status` output, expanded FIFO flags, `begin`, `whoami` / `id`, and
  `shub [1..12]` for sensor-hub output bytes

## ESP-IDF Example

[examples/idf/basic](examples/idf/basic) is a native ESP-IDF project with
`app_main()`, fixed-buffer CLI input, and `driver/i2c_master.h` callbacks. It
does not compile Arduino sources or provide Arduino compatibility facades. The
driver core remains framework-neutral and receives transport/timing callbacks
through `Config`.

Representative commands:

```text
probe
cfg
status
whoami
shub 12
odrxl 104
gpm lpn
ts 1
pedo 1
steps
funcsrc1
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
stress 100
stress_mix 100
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
- `cachedConfigDirty()`

Public raw access is limited to the main user-register window through `Z_OFS_USR`; zero-length, oversized, and wrapping block reads are rejected before the bus is touched. Writes to managed configuration registers refresh the cached configuration after the write succeeds.

This is the intended path for interrupt routing, threshold registers, advanced tap/wake/free-fall setup, and sensor-hub experimentation.
If a cache-affecting write or refresh fails, `cachedConfigDirty()` reports that local configuration mirrors may not match the chip until `refreshCachedConfig()` or `recover()` succeeds.

## Poll-Chunked Execution

`poll(nowMs, maxInstructions)` advances at most the requested number of I2C
instructions. One register read, register write, or contiguous burst read counts
as one instruction. CPU decode and conversion work does not count.

- `requestMeasurement()` schedules a status-gated sample job.
- `requestMeasurement(false)` schedules a direct raw burst without a status read.
- `tick(nowMs)` delegates to `poll(nowMs, 1)`.
- `pollBusy()` reports active sample, reset, boot, refresh, FIFO-drain, or staged
  calibration jobs.
- `lastPollStatus()` reports the latest poll progress or terminal error.
- `startSoftReset()`, `startBoot()`, `startRefreshCachedConfig()`,
  `startFifoDrain(maxWords)`, `startAccelBiasCapture(samples)`, and
  `startGyroBiasCapture(samples)` schedule chunked diagnostic/configuration jobs.

With `maxInstructions = 1`, a ready status-gated sample takes two polls: status
read first, raw burst second. With `maxInstructions >= 2`, both can complete in
one `poll()` call when data is ready.

`poll(nowMs, 0)` is an explicit no-progress call: it returns `IN_PROGRESS` for
an active job, does not touch I2C, and does not arm ready deadlines. For staged
sample jobs, the `nowMs` argument passed to the first positive-budget readiness
poll starts the ready deadline, and the `nowMs` argument used for the raw burst
becomes `sampleTimestampMs()`. Direct blocking reads such as `readAllRaw()` use
`Config::nowMs` for timestamps when provided and `0` when no hook is configured.

Ready-checked sample jobs can legitimately timeout if a caller manually splits
`job start sample` and positive-budget `job poll` calls across slow host/serial
interaction. For instruction-budget diagnostics use direct jobs
(`requestMeasurement(false)` / `job start direct`) or `job run sample ...`.

CLI job grammar:

```text
job status
job auto <0|1>
job start <sample|direct|reset|boot|refresh|fifo|calxl|calg> [arg]
job poll <budget 0..255> [count 1..1000] [delayMs 0..1000]
job run <kind> <budget> [limit 1..1000] [delayMs] [arg]
job get
job getraw
job cancel
```

`job cancel` is CLI manual-polling cleanup only. It does not cancel a partially
applied driver job; active staged jobs are expected to complete or fail on later
`tick()` / `poll()` calls. `end()` is the hard lifecycle reset.

## TunnelMonitor Fit

See [docs/tunnelmonitor_fit_report.md](docs/tunnelmonitor_fit_report.md) for the API classification and status taxonomy decisions used when integrating behind a queue-owned I2C task.

## Diagnostics And Snapshots

Cache-only diagnostics include `SettingsSnapshot`, `getSettings()`,
`settings()`, `driverState()`, `hasSample()`, `sampleTimestampMs()`, and
`sampleAgeMs(nowMs)`. `StatusReg` and `readStatus(StatusReg&)` expose decoded
accelerometer, gyroscope, and temperature data-ready flags without requiring
callers to decode `STATUS_REG` manually.

The bring-up CLI also decodes `FUNC_SRC1`, `FUNC_SRC2`, and `WRIST_TILT_IA`.
`steps` prints pedometer enable state, accelerometer ODR, the step counter,
step timestamp, and the relevant `FUNC_SRC1` step flags. A zero `FUNC_SRC1`
with a changing step counter is valid: without latched/routed interrupts, step
source bits can clear before a manual CLI read observes them.

## Building And Validation

```bash
pio test -e native
pio run -e esp32s3dev
pio run -e esp32s2dev
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python tools/hil_smoke.py --dry-run --parser-self-test
```

When ESP-IDF is installed, build the IDF example from
[examples/idf/basic](examples/idf/basic):

```bash
idf.py set-target esp32s3
idf.py build
idf.py set-target esp32s2
idf.py build
```

## HIL Evidence And Release Gates

Current evidence in this repository:

- native Unity tests cover lifecycle, validation, health, chunked polling, FIFO,
  calibration quality rejection, self-test, and register helpers
- Arduino PlatformIO builds are maintained for ESP32-S3 and ESP32-S2
- broad COM26 Arduino HIL validation on 2026-06-22 covered smoke, 120-command
  validation, benchmark, post-soak live read, and an 8-hour soak that completed
  with 0 hard failures and recoverable prompt-capture UNKNOWNs
- targeted COM26 HIL on 2026-06-23 covered poll budgets `0`, `1`, and `255`,
  reset/boot retry, job cancel/manual-mode behavior, FIFO/config/source-register
  commands, calibration, self-test, and short stress
- closeout targeted COM26 HIL on 2026-06-23 passed 145/145 checks with the
  final behavior summarized in
  [docs/reports/hil-evidence-summary.md](docs/reports/hil-evidence-summary.md)
- release closeout HIL on 2026-06-24 completed a 30-minute quiet soak
  (2095 PASS, 0 FAIL, 0 UNKNOWN) and a final bounded 10-minute quiet soak
  (1102 PASS, 0 FAIL, 0 UNKNOWN); remaining prompt recoveries are documented in
  [docs/reports/hil-evidence-summary.md](docs/reports/hil-evidence-summary.md)

Remaining gates before claiming final field readiness:

- physical bus fault injection: hold SDA/SCL low or force NACK/timeout
- IMU power-cycle or disconnect/reconnect while the MCU stays alive
- absent-device behavior on the target wiring
- alternate address `0x6B`
- live native ESP-IDF HIL, not only Arduino HIL and IDF build/contract checks
- longer soak with independent serial-link evidence and no prompt-recovery notes
  if that is required as a final field gate

## Hardware-In-Loop Smoke

`tools/hil_smoke.py` drives the maintained serial CLI; it does not add fake devices or simulated buses to production paths. The default command set covers `version`, `scan`, `probe`, `settings`, `health`, `whoami`, `status`, `raw`, `fifo`, and `selftest`.

```bash
python tools/hil_smoke.py --dry-run --parser-self-test
python tools/hil_smoke.py --suite targeted --port COM7 --reset-before --stress-count 50
```

The runner classifies visible status/error tokens such as `I2C_TIMEOUT`,
`I2C_NACK_ADDR`, `I2C_BUSY`, `DEVICE_NOT_FOUND`, `CHIP_ID_MISMATCH`,
`SELF_TEST_FAIL`, `FIFO_EMPTY`, `FIFO_OVERRUN`, `CALIBRATION_UNSTABLE`,
`CALIBRATION_ORIENTATION`, and `OFFLINE`. A live self-test still requires a
suitable stationary fixture before making field-readiness claims.

## Repository Notes

- Public headers live in [include/LSM6DS3TR](include/LSM6DS3TR)
- Implementation lives in [src/LSM6DS3TR.cpp](src/LSM6DS3TR.cpp)
- Version metadata is generated into [include/LSM6DS3TR/Version.h](include/LSM6DS3TR/Version.h) from [library.json](library.json)
- `examples/common` is not installed as part of the library
- The library never configures I2C pins or owns the bus
- ESP-IDF I2C setup and GPIO choices live in examples/application code only
