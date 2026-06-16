# LSM6DS3TR TunnelMonitor Fit Report

## Steady-Path APIs

- Poll-chunked sample acquisition: `requestMeasurement()`, `poll(nowMs, 1)`,
  `pollBusy()`, `measurementReady()`, `getRawMeasurement()`, and
  `lastPollStatus()`.
- Raw integer reads: `readAccelRaw()`, `readGyroRaw()`, `readTemperatureRaw()`, `readAllRaw()`.
- Cached sample access: `getRawMeasurement()` after a completed request.
- Local conversion helpers: `convertAccel()`, `convertGyro()`, `convertTemperature()`.
- Local software-bias helpers: `setAccelBias()`, `accelBias()`, `setGyroBias()`,
  `gyroBias()`, `correctAccel()`, `correctGyro()`.

## Convenience Or Diagnostic APIs

- `tick()` is compatibility sugar for `poll(nowMs, 1)`.
- `getMeasurement()` is a convenience conversion wrapper over the cached raw sample.
- `captureAccelBias()` and `captureGyroBias()` are diagnostic calibration helpers.
  They are bounded by sample count, data-ready deadlines, and a per-sample poll cap,
  but they may still issue many blocking I2C transactions in one call.
- `startAccelBiasCapture()` and `startGyroBiasCapture()` are chunked diagnostic
  calibration jobs; they are budgeted but still not steady firmware sample paths.
- `probe()`, `recover()`, `softReset()`, `boot()`, synchronous
  `refreshCachedConfig()`, direct register access, FIFO configuration, feature
  setters, and synchronous cache refresh are diagnostic or configuration APIs, not
  steady polling APIs.
- `startSoftReset()`, `startBoot()`, `startRefreshCachedConfig()`, and
  `startFifoDrain(maxWords)` are chunked diagnostic/configuration jobs.

## Instruction Budget Decisions

- One register read, register write, or contiguous burst read is one instruction.
- Delay gates and CPU decode/conversion math consume zero instructions.
- With `maxInstructions = 1`, status-gated sample acquisition takes one poll for
  `STATUS_REG` and a later poll for the raw output burst.
- With `maxInstructions >= 2`, a ready status-gated sample may complete status
  plus raw burst in one `poll()` call.
- A not-ready status read stops the current `poll()` call even when budget remains,
  so repeated readiness polling is visible across calls.
- FIFO drain reads FIFO status once, then drains each FIFO word as a separate
  instruction.

## Status Taxonomy Decisions

- WHO_AM_I transport failures from `begin()` and `probe()` preserve the transport
  status (`I2C_TIMEOUT`, `I2C_BUS`, `I2C_NACK_ADDR`, `I2C_NACK_DATA`, or
  `I2C_ERROR`) and `detail`. Only a successful read with the wrong ID returns
  `CHIP_ID_MISMATCH`.
- `Err::OFFLINE` is appended for explicit offline precondition failures. The current
  explicit offline path is `requestMeasurement()`, which now returns `OFFLINE`
  instead of overloading `BUSY`.
- Cache-affecting setters commit local mirrors only after successful writes. Partial
  multi-register failures and failed direct writes to managed registers set
  `cachedConfigDirty()`, signalling that the application should refresh or recover
  before trusting cached configuration mirrors.
- `refreshCachedConfig()` stages all decoded register values locally, validates them,
  and commits only after the full refresh succeeds.
