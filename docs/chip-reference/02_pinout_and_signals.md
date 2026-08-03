# Pinout And Signals

> Source confidence: vendor fact. Library status: board-level information;
> host pin ownership remains outside the library.

## Pin Summary

| Pin | Name | Mode 1 function | Mode 2 function | Source |
|---:|---|---|---|---|
| 1 | `SDO/SA0` | SPI SDO; I2C address LSB | Same | Datasheet, p. 20 |
| 2 | `SDx` | Connect to `VDDIO` or GND | I2C master data (`MSDA`) | Datasheet, p. 20 |
| 3 | `SCx` | Connect to `VDDIO` or GND | I2C master clock (`MSCL`) | Datasheet, p. 20 |
| 4 | `INT1` | Programmable interrupt 1 | Same | Datasheet, p. 20 |
| 5 | `VDDIO` | I/O supply | Same | Datasheet, p. 20 |
| 6 | `GND` | Ground | Same | Datasheet, p. 20 |
| 7 | `GND` | Ground | Same | Datasheet, p. 20 |
| 8 | `VDD` | Core supply | Same | Datasheet, p. 20 |
| 9 | `INT2` | Programmable interrupt 2 / DEN | INT2 / DEN / I2C master external sync (`MDRDY`) | Datasheet, p. 20 |
| 10 | `NC` | Leave unconnected | Leave unconnected | Datasheet, p. 20 |
| 11 | `NC` | Leave unconnected | Leave unconnected | Datasheet, p. 20 |
| 12 | `CS` | I2C/SPI mode select | Same | Datasheet, p. 20 |
| 13 | `SCL` | I2C SCL / SPI SPC | Same | Datasheet, p. 20 |
| 14 | `SDA` | I2C SDA / SPI SDI / 3-wire SDO | Same | Datasheet, p. 20 |

## Host Interface Selection

- `CS = 1` enables I2C communication; `CS = 0` selects SPI communication and
  disables I2C. Source: datasheet, pp. 20, 44-45.
- The I2C slave address is `110101x`; `SA0=0` gives `0x6A`, `SA0=1` gives `0x6B`. Source: datasheet, pp. 39-40.
- I2C read/write address bytes are `0xD5`/`0xD4` for SA0 low and `0xD7`/`0xD6` for SA0 high. Source: datasheet, p. 40.

## Hardware Notes

- Use 100 nF ceramic decoupling capacitors near `VDD` and `VDDIO`. Source: datasheet, pp. 20, 45-46.
- `SCL` and `SDA` inputs have no internal pull-up. Source: datasheet, p. 48.
- `SDO/SA0` has no pull-up in the default two-interface configuration; its pull-up is enabled in 3-wire SPI. `SDx`/`SCx` auxiliary-bus pull-ups are controlled by `MASTER_CONFIG.PULL_UP_EN`. Source: datasheet, pp. 47-48, 69.
- `INT1` and `INT2` are forced low by default. `CS` has an internal pull-up enabled by default; setting `CTRL4_C.I2C_disable=1` disables that pull-up together with the I2C interface. Source: datasheet, pp. 47-48, 64.
- Internal 30..50 kohm pull-ups on NC pins 10 and 11 are enabled by default. The vendor disable sequence is `0x00 <- 0x80`, `0x05 <- 0x01`, `0x00 <- 0x00`; it accesses undocumented controls and is therefore board/diagnostic guidance, not a production-profile recipe. Source: datasheet, pp. 47-48.
