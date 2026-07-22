# LSM6DS3TR ESP-IDF Port

The core is a framework-neutral ESP-IDF component. `idf_component.yml`
supports the ESP-IDF 5.4 line, and CI compiles the native example with exactly
ESP-IDF 5.4.4 for ESP32-S2 and ESP32-S3.

The component target compiles `src/LSM6DS3TR.cpp` as C++17 and exports only
the public `include` directory. Package version, public version constants,
component version, and Doxygen project version are synchronized from
`library.json`.

The application owns all ESP-IDF integration:

- entry point: `app_main()`;
- I2C bus/device handles: `i2c_new_master_bus()` and
  `i2c_master_bus_add_device()` from `driver/i2c_master.h`;
- synchronous physical attempts: `i2c_master_transmit()` and
  `i2c_master_transmit_receive()`;
- monotonic operation time: `esp_timer_get_time()` converted to 64-bit ms;
- task scheduling, yields, and logging: FreeRTOS and native ESP-IDF facilities;
- non-blocking console input so CLI traffic cannot stall operation polling;
- pins, bus rate, per-transfer timeout, locking, retries, and recovery;
- fixed CLI buffers and result publication.

The component receives only synchronous, timeout-bounded transport callbacks.
Each callback performs exactly one physical attempt: it neither retries nor
recovers the bus, and it maps `esp_err_t` into the library's framework-neutral
`Status`. The owner may serialize access inside the callback, but the lock wait
must remain within the supplied transfer timeout.

`bind()` does no I2C. The owner starts a tokened operation, calls
`poll(nowMs, budget)`, cancels through the bus-silent cancellation API when
needed, and takes the matching terminal result exactly once. Each operation
also has a hard total callback ceiling reported in its terminal result.

The native example must not include `Arduino.h`, `Wire.h`, `String`, `Serial`,
`TwoWire`, Arduino compatibility facades, or Arduino CLI sources.
The Arduino and native ESP-IDF examples expose the same compact command set;
repository guards enforce parity without compiling Arduino sources into the
IDF component.

Run the repository guards after changing the example:

```sh
python tools/check_idf_example_contract.py
python tools/check_cli_contract.py
python tools/check_core_timing_guard.py
python tools/build_docs.py
```

Local compilation requires an ESP-IDF 5.4 installation:

```sh
cd examples/idf/basic
idf.py set-target esp32s3
idf.py build
```

Use `esp32s2` for the other supported target. A static contract check is not a
compiler result; CI's two target jobs provide the actual IDF compile evidence.
