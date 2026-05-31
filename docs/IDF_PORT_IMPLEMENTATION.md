# LSM6DS3TR ESP-IDF Port Implementation

Implementation status:
- `src/PlatformTime.h` was removed; the core no longer includes Arduino or
  ESP-IDF timing headers.
- `examples/idf/basic/main/main.cpp` owns the native fixed-buffer CLI and the
  ESP-IDF I2C callback glue.
- The ESP-IDF CMake target compiles only native IDF sources.
- Compatibility files (`Arduino.h`, `Wire.h`, `ArduinoCompat.cpp`) are not part
  of the IDF example.

The command contract is enforced by `tools/check_idf_example_contract.py`.
When adding a CLI command to the Arduino bring-up example, add the matching
native IDF command or explicitly document why parity is not applicable.
