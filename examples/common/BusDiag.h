#pragma once

#include <Wire.h>

#include "I2cScanner.h"

namespace bus_diag {
inline void scan() {
  i2c_scanner::scan(Wire);
}
}  // namespace bus_diag
