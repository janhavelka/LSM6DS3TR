# LSM6DS3TR ESP-IDF CLI Example

This ESP-IDF project runs the same bringup CLI as the Arduino example. The
example-local compatibility facade provides the small `Serial`, `String`, and
`TwoWire` surface used by `examples/01_basic_bringup_cli/main.cpp`; the facade's
I2C operations are implemented with ESP-IDF v6 `driver/i2c_master.h`.

- Default SDA: GPIO8
- Default SCL: GPIO9
- Default address: `0x6A`
- I2C driver: ESP-IDF v6 `driver/i2c_master.h`
- CLI prompt: `> `

The core library still owns no I2C bus, pins, tasks, GPIO interrupts, logging,
or terminal I/O. All of that remains example/application glue.

Representative CLI commands:

```text
help
scan
begin
drv
cfg
read
raw
status
whoami
shub 12
odrxl 104
fsg 500
ts 1
pedo 1
steps
funcsrc1
fifo
fifo_mode cont
fifo_odr 104
fifo_read 8
cal 100
selftest
stress 100
stress_mix 100
rreg 0x10
wreg 0x58 0x8E
dump 0x10 32
recover
reset
```

Build from this directory when ESP-IDF is installed:

```bash
idf.py set-target esp32s3
idf.py build
```

Change the GPIO constants in `examples/common/BoardConfig.h` or provide your own
application glue for your board.
