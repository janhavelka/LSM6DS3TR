# LSM6DS3TR ESP-IDF Port

The ESP-IDF example is a native IDF application in `examples/idf/basic`.
It does not compile Arduino example sources and does not provide Arduino
compatibility facades.

Native boundaries:
- Entry point: `app_main()`.
- I2C: `driver/i2c_master.h` with `i2c_new_master_bus()`,
  `i2c_master_bus_add_device()`, `i2c_master_probe()`,
  `i2c_master_transmit()`, and `i2c_master_transmit_receive()`.
- CLI input: fixed C buffer using `getchar()`.
- Timing: `esp_timer_get_time()` is injected through `Config::nowMs`.
- Scheduling: FreeRTOS task delay/yield is used only in the example.
- Forbidden in IDF examples: `Arduino.h`, `Wire.h`, `String`, `Serial`,
  `TwoWire`, `ArduinoCompat`, `IdfArduinoCompat`, and including
  `examples/01_basic_bringup_cli/main.cpp`.

The driver core remains framework-neutral. If `Config::nowMs` is not supplied,
core timestamps use 0 rather than calling platform timing APIs.

Run the static contract check after touching the IDF example:

```sh
python tools/check_idf_example_contract.py
python tools/check_core_timing_guard.py
```
