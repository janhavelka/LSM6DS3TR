# LSM6DS3TR ESP-IDF Basic Example

Native ESP-IDF bring-up CLI for LSM6DS3TR-C.

- Entry point: `app_main()`
- I2C driver: ESP-IDF `driver/i2c_master.h`
- CLI input: fixed C buffer using `getchar()`
- Timing: `esp_timer_get_time()` injected through `Config::nowMs`
- No Arduino compatibility facade and no Arduino CLI source inclusion

Run `python tools/check_idf_example_contract.py` from the repository root after
editing this example.
