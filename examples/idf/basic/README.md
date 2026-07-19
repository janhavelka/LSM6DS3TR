# Native ESP-IDF Owner-Safe Example

This project demonstrates the version 2 external-owner API with native
ESP-IDF 5.4:

- `app_main()` and fixed C buffers;
- `driver/i2c_master.h` transport owned by the application;
- 64-bit monotonic time from `esp_timer_get_time()`;
- non-blocking console input so owner polling is never held behind `getchar()`;
- tokened start/poll/cancel/take lifecycle;
- one transport callback per owner poll;
- no Arduino compatibility facade.

Build locally with ESP-IDF 5.4:

```sh
idf.py set-target esp32s3
idf.py build
```

CI builds both `esp32s2` and `esp32s3` using exactly ESP-IDF 5.4.0.
The example pins GPIO 8/9 and address `0x6A` only as application-owned fixture
defaults; the library owns none of those board choices.
