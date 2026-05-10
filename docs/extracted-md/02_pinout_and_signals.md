# Pinout And Signals

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

- `CS = 1` selects SPI idle/I2C communication enabled; `CS = 0` selects SPI communication mode and disables I2C. Source: datasheet, p. 20.
- The I2C slave address is `110101x`; `SA0=0` gives `0x6A`, `SA0=1` gives `0x6B`. Source: datasheet, pp. 39-40.
- I2C read/write address bytes are `0xD5`/`0xD4` for SA0 low and `0xD7`/`0xD6` for SA0 high. Source: datasheet, p. 40.

## Hardware Notes

- Use 100 nF ceramic decoupling capacitors near `VDD` and `VDDIO`. Source: datasheet, pp. 20, 45-46.
- `SCL` and `SDA` inputs have no internal pull-up. Source: datasheet, p. 48.
- Internal pull-ups on NC pins 10 and 11 are enabled by default; the datasheet gives a procedure to disable them. Source: datasheet, pp. 47-48.
