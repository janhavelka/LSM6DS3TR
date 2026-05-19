# LSM6DS3TR ESP-IDF Port Implementation Notes

Date: 2026-05-19.
Branch: `feature/lsm6ds3tr-idf-port`.

## Scope

- Kept the public `Config` callback contract unchanged.
- Removed direct `Arduino.h` use from `src/LSM6DS3TR.cpp`.
- Kept `src/PlatformTime.h` private:
  - Arduino and native tests with an Arduino stub use `millis()`.
  - ESP-IDF uses `esp_timer_get_time() / 1000`.
  - Applications can still override timing with `Config::nowMs`.
- Added ESP-IDF component metadata for the framework-neutral core.
- Made `examples/idf/basic` run the same user-visible CLI as the Arduino
  bringup example.

## Shared CLI Approach

The ESP-IDF example compiles `examples/01_basic_bringup_cli/main.cpp` directly.
That keeps command names, aliases, help sections, arguments, output formatting,
colors, prompts, health/error reporting, diagnostics, reset/recover/probe,
self-test, stress/demo flows, FIFO, source registers, and raw register access
aligned with Arduino.

The IDF project provides example-local compatibility files:

- `examples/idf/basic/main/Arduino.h`
- `examples/idf/basic/main/Wire.h`
- `examples/idf/basic/main/ArduinoCompat.cpp`

These files are not library API. They only supply the tiny `Serial`, `String`,
timing/GPIO, and `TwoWire` surface that the shared CLI expects.

## ESP-IDF I2C Glue

The compatibility `TwoWire` facade uses ESP-IDF v6 `driver/i2c_master.h`:

- `i2c_new_master_bus()` owns bus creation in the example.
- `i2c_master_bus_add_device()` creates per-address device handles lazily.
- `i2c_master_transmit()` backs write transactions.
- `i2c_master_transmit_receive()` backs register reads with a repeated-start
  style transaction.
- `i2c_master_receive()` backs receive-only transactions.
- `i2c_master_probe()` backs CLI I2C scans.

The core driver still receives only `Config::i2cWrite` and
`Config::i2cWriteRead`; it does not see `Wire`, ESP-IDF bus handles, pins, GPIO,
tasks, logging, or terminal I/O.

## Files Added Or Updated

- `CMakeLists.txt`
- `idf_component.yml`
- `src/PlatformTime.h`
- `examples/idf/basic/CMakeLists.txt`
- `examples/idf/basic/main/CMakeLists.txt`
- `examples/idf/basic/main/main.cpp`
- `examples/idf/basic/main/Arduino.h`
- `examples/idf/basic/main/Wire.h`
- `examples/idf/basic/main/ArduinoCompat.cpp`
- `examples/idf/basic/README.md`
- `tools/check_idf_example_contract.py`
- `docs/IDF_PORT.md`

## Validation Status

- Contract checks cover core timing separation, Arduino CLI structure, and IDF
  shared-CLI inclusion.
- Native tests cover framework-neutral driver behavior.
- Hardware validation is still required for both I2C addresses, live samples,
  reset/recover/probe, FIFO, embedded functions, self-test, stress workflows,
  bus scan, and fault injection.
