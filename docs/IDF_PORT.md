# LSM6DS3TR ESP-IDF v6.0.1 Port Notes

Last updated: 2026-05-19.

This file existed before the port work as an untracked documentation-only audit.
It has been incorporated as the active ESP-IDF port note instead of being
deleted.

Official ESP-IDF references:

- I2C master driver: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html
- Build system and components: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html
- ESP-IDF v6 peripheral migration notes: https://docs.espressif.com/projects/esp-idf/en/release-v6.0/esp32c3/migration-guides/release-6.x/6.0/peripherals.html

## Current State

- The driver core is framework-neutral C++17 and builds as an ESP-IDF component.
- Public API remains in `include/LSM6DS3TR/`; implementation remains in `src/`.
- The core never touches `Wire`, ESP-IDF I2C handles, GPIO pins, terminal I/O,
  logging, FreeRTOS tasks, or board-specific configuration.
- `Config` remains the portability boundary with `i2cWrite`, `i2cWriteRead`,
  `i2cUser`, `nowMs`, and `timeUser`.
- `src/PlatformTime.h` is the only private Arduino/ESP-IDF time shim. It uses
  `millis()` under Arduino and `esp_timer_get_time() / 1000` under ESP-IDF.
- `CMakeLists.txt` and `idf_component.yml` describe the core component.
- `examples/idf/basic` is a normal ESP-IDF project.

## CLI Parity

The ESP-IDF example compiles the same bringup CLI source as Arduino:

- shared source: `examples/01_basic_bringup_cli/main.cpp`
- ESP-IDF entrypoint: `examples/idf/basic/main/main.cpp`
- compatibility facade:
  - `examples/idf/basic/main/Arduino.h`
  - `examples/idf/basic/main/Wire.h`
  - `examples/idf/basic/main/ArduinoCompat.cpp`

This keeps these user-visible behaviors aligned across Arduino and ESP-IDF:

- command names and aliases
- help sections and command descriptions
- argument defaults, ranges, and error messages
- ANSI colors, status lines, prompts, and output structure
- probe, recover, reset, boot, health, and version reporting
- diagnostics for WHO_AM_I, status, source registers, sensor-hub bytes, FIFO,
  steps, and raw register access
- calibration, self-test, streaming, ODR-paced stress, and mixed stress flows

## ESP-IDF I2C Glue

The example-local `TwoWire` facade is backed by ESP-IDF v6
`driver/i2c_master.h`:

- `i2c_new_master_bus()` creates the bus in the example.
- `i2c_master_bus_add_device()` lazily creates per-address handles.
- `i2c_master_transmit()` implements writes.
- `i2c_master_transmit_receive()` implements register reads.
- `i2c_master_receive()` implements receive-only reads.
- `i2c_master_probe()` implements CLI bus scans.

The shared CLI still injects callbacks through `Config` via
`examples/common/I2cTransport.h`. The core driver only sees the callback
contract and framework-neutral `Status` values.

## Build Files

Core component:

```cmake
idf_component_register(
  SRCS "src/LSM6DS3TR.cpp"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES esp_timer
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
```

IDF example main component:

```cmake
idf_component_register(
  SRCS
    "main.cpp"
    "ArduinoCompat.cpp"
    "../../../01_basic_bringup_cli/main.cpp"
  INCLUDE_DIRS "." "../../../.."
  REQUIRES LSM6DS3TR esp_driver_i2c esp_driver_gpio esp_timer freertos
)
```

## Validation Checklist

Run locally where tools are installed:

```bash
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
pio test -e native
pio run -e esp32s3dev
pio run -e esp32s2dev
```

Run from `examples/idf/basic` with ESP-IDF v6.0.1:

```bash
idf.py set-target esp32s3
idf.py build
idf.py set-target esp32s2
idf.py build
```

Hardware validation still needed:

- both I2C addresses, `0x6A` and `0x6B`
- clean missing-device and wrong-ID failures
- accel, gyro, temperature, and combined burst reads
- ODR, full-scale, power-mode, filter, timestamp, pedometer, tilt, wrist-tilt,
  offset, and FIFO setters
- raw register read/write/dump safeguards
- soft reset, boot, probe, recover, and health transitions
- self-test, stream, calibration, `stress`, and `stress_mix`
- scan and timeout/NACK recovery behavior on real wiring faults
