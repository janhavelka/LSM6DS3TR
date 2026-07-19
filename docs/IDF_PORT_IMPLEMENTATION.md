# LSM6DS3TR ESP-IDF Port Implementation

The component target compiles `src/LSM6DS3TR.cpp` as C++17 and exports only
the public `include` directory. The core contains no ESP-IDF timing, GPIO,
task, logging, or I2C ownership.

`examples/idf/basic/main/main.cpp` owns:

- `i2c_new_master_bus()` and `i2c_master_bus_add_device()`;
- synchronous `i2c_master_transmit()` and
  `i2c_master_transmit_receive()` callbacks;
- `esp_timer_get_time()` as the single 64-bit uptime domain;
- one-transaction owner polls;
- a non-blocking stdin descriptor and bounded per-loop console service;
- token correlation and exactly-once terminal result consumption;
- fixed-buffer CLI input and FreeRTOS yielding.

The Arduino and native-IDF examples expose the same compact version 2 command
surface. Command parity is checked as a repository contract, not achieved by
compiling Arduino sources or compatibility facades into the IDF project.

The manifest accepts ESP-IDF `>=5.4.0,<5.5.0`. CI fixes its build image to
5.4.0 and compiles both supported targets. Package version, public version
header, and Doxygen project version are generated/checked from `library.json`.
