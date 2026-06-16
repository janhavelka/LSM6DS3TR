# Initialization, Reset, And Operational Notes

## Basic Bring-Up Sequence

1. Power `VDD` and `VDDIO`, wait for turn-on time. Source: datasheet, p. 24.
2. Select I2C by holding `CS` high or SPI by holding `CS` low. Source: datasheet, p. 20.
3. Read `WHO_AM_I` (`0x0F`) and expect `0x6A`. Source: datasheet, p. 60.
4. For 16-bit output pairs, set `CTRL3_C.BDU=1`; output registers then hold their values until both MSB and LSB are read. Source: datasheet, p. 63.
5. For burst reads, keep `CTRL3_C.IF_INC=1`; register addresses then auto-increment during multi-byte I2C or SPI transfers. Default is enabled. Source: datasheet, p. 63.
6. Configure full scales in `CTRL1_XL` and `CTRL2_G`, then set nonzero ODRs to enable accelerometer and/or gyroscope. Source: datasheet, pp. 61-62.
7. Poll `STATUS_REG` or route data-ready signals to INT pins, then burst-read output registers. Source: datasheet, pp. 59-60, 72-76.

## Reset And Boot Bits

| Action | Bit | Notes | Source |
|---|---|---|---|
| Software reset | `CTRL3_C.SW_RESET` | Set to 1; bit automatically clears. | Datasheet, p. 63 |
| Reboot memory content | `CTRL3_C.BOOT` | Reboots memory content; normal state is 0. | Datasheet, p. 63 |

- Do not set `BOOT` and `SW_RESET` in the same write; run the flows serially. Source: AN5130, reset procedure.
- After `BOOT`, registers may be inaccessible for about 15 ms. Source: AN5130, reset procedure.
- After `SW_RESET`, wait at least the documented short reset interval or poll until `SW_RESET` clears. Source: AN5130, reset procedure.

## Output Handling

- Gyro and accel axis outputs are two's-complement 16-bit values. Source: datasheet, pp. 73-76.
- Default endian places LSB at the lower address (`CTRL3_C.BLE=0`). Source: datasheet, p. 63.
- Apply sensitivity constants from `03_electrical_and_timing.md` based on configured full-scale range. Source: datasheet, p. 21.
- Temperature conversion uses the datasheet temperature sensitivity of 256 LSB/degC and typ. 0 LSB at 25 degC. Source: datasheet, p. 25.
- Burst-read all bytes for a ready output group after DRDY. BDU protects each LSB/MSB pair, but slow per-axis reads must not be treated as one atomic XYZ sample. Source: AN5130, data-ready and BDU notes.
- Latched data-ready flags clear when the corresponding high output byte is read; pulsed DRDY mode does not change the `STATUS_REG.XLDA` and `STATUS_REG.GDA` latch behavior. Source: AN5130, data-ready notes.

## Feature Notes

- Absolute Wrist Tilt requires accelerometer ODR >=26 Hz and setting both `FUNC_EN` and `WRIST_TILT_EN` in `CTRL10_C`. Source: datasheet, p. 17.
- Sensor-hub mode uses Mode 2 pins `SDx/MSDA`, `SCx/MSCL`, and `INT2/MDRDY` plus `MASTER_CONFIG` (`0x1A`) fields such as `MASTER_ON`, `PASS_THROUGH_MODE`, and `PULL_UP_EN`. Source: datasheet, pp. 19-20, 38, 69.
- AN5130 supplies LSM6DS3TR-C feature recipes; the design-tip PDFs contain calibration/tilt/noise math but no replacement register reset values or bit definitions for `0x00..0x7F`.
