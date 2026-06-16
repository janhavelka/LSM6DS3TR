#pragma once

#include <Wire.h>

#include "BoardConfig.h"
#include "I2cScanner.h"

namespace bus_diag {
inline void scan() {
  i2c_scanner::ScanOptions options;
  options.scanTimeoutMs = board::I2C_TIMEOUT_MS;
  options.restoreTimeoutMs = board::I2C_TIMEOUT_MS;
  options.restoreClockHz = board::I2C_FREQ_HZ;
  options.sda = board::I2C_SDA;
  options.scl = board::I2C_SCL;
  options.recoverOnTimeout = true;
  i2c_scanner::scan(Wire, options);
}
}  // namespace bus_diag
