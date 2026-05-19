# LSM6DS3TR ESP-IDF Port Implementation Notes

Date: 2026-05-19.
Branch: `feature/lsm6ds3tr-idf-port`.

## Scope

- Kept the public `Config` callback contract unchanged.
- Removed the direct `Arduino.h` include from `src/LSM6DS3TR.cpp`.
- Added `src/PlatformTime.h` as a private timing shim:
  - Arduino and native tests with an Arduino stub use `millis()`.
  - ESP-IDF uses `esp_timer_get_time() / 1000`.
  - Applications can still override timing with `Config::nowMs`.
- Added ESP-IDF component metadata for the core library.
- Added a pure ESP-IDF basic I2C example under `examples/idf/basic`.

## Files Added

- `CMakeLists.txt`
- `idf_component.yml`
- `src/PlatformTime.h`
- `examples/idf/basic/CMakeLists.txt`
- `examples/idf/basic/main/CMakeLists.txt`
- `examples/idf/basic/main/main.cpp`
- `examples/idf/basic/README.md`

## Architecture

- The library core remains I2C-callback based and does not own `Wire`, an
  ESP-IDF I2C bus, pins, pull-ups, tasks, or interrupts.
- The ESP-IDF example owns `i2c_master_bus_handle_t` and
  `i2c_master_dev_handle_t`, then injects bounded blocking callbacks into
  `LSM6DS3TR::Config`.
- The IDF callback maps `ESP_ERR_TIMEOUT` to `Err::I2C_TIMEOUT`,
  `ESP_ERR_INVALID_ARG` to `Err::INVALID_PARAM`, and other transfer failures to
  `Err::I2C_ERROR` with the raw `esp_err_t` in `Status::detail`.
- Transfer timeouts are clamped before passing them to ESP-IDF's signed
  timeout parameter so a large value cannot become an infinite wait.

## Remaining Validation

- `idf.py` is not on PATH in this shell, so native ESP-IDF builds for `esp32s3`
  and `esp32s2` are pending.
- Hardware smoke remains pending for both I2C addresses, sensor samples, reset,
  FIFO paths, interrupt-driven application loops, and missing-device failure.
