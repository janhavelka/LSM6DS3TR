# LSM6DS3TR ESP-IDF Basic Example

This example owns the ESP-IDF I2C bus and injects blocking, bounded I2C
callbacks into the framework-neutral driver core.

- Default SDA: GPIO8
- Default SCL: GPIO9
- Default address: `0x6A`
- I2C driver: ESP-IDF v6 `driver/i2c_master.h`

Change the GPIO constants in `main/main.cpp` for your board. The library core
does not configure pins, own the I2C bus, or log.
