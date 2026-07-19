# LSM6DS3TR ESP-IDF Port

The core is a framework-neutral ESP-IDF component. `idf_component.yml`
supports the ESP-IDF 5.4 line, and CI compiles the native example with exactly
ESP-IDF 5.4.0 for ESP32-S2 and ESP32-S3.

The application owns all ESP-IDF integration:

- entry point: `app_main()`;
- I2C bus/device handles: `driver/i2c_master.h`;
- monotonic operation time: `esp_timer_get_time()` converted to 64-bit ms;
- task scheduling and yields: FreeRTOS;
- non-blocking console input so CLI traffic cannot stall operation polling;
- pins, bus rate, per-transfer timeout, locking, retries, and recovery;
- fixed CLI buffers and result publication.

The component receives only synchronous, timeout-bounded transport callbacks.
`bind()` does no I2C. The owner starts a tokened operation, calls
`poll(nowMs, budget)`, cancels through the bus-silent cancellation API when
needed, and takes the matching terminal result exactly once. Each operation
also has a hard total callback ceiling reported in its terminal result.

The native example must not include `Arduino.h`, `Wire.h`, `String`, `Serial`,
`TwoWire`, Arduino compatibility facades, or Arduino CLI sources.

Run the repository guards after changing the example:

```sh
python tools/check_idf_example_contract.py
python tools/check_cli_contract.py
python tools/check_core_timing_guard.py
```

Local compilation requires an ESP-IDF 5.4 installation:

```sh
cd examples/idf/basic
idf.py set-target esp32s3
idf.py build
```

Use `esp32s2` for the other supported target. A static contract check is not a
compiler result; CI's two target jobs provide the actual IDF compile evidence.
