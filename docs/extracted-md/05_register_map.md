# Register Map

## Core User Registers

| Address | Name | Type | Default | Notes | Source |
|---:|---|---|---:|---|---|
| `0x01` | `FUNC_CFG_ACCESS` | R/W | `0x00` | Select embedded-function register bank access. | Datasheet, p. 53 |
| `0x06..0x0A` | `FIFO_CTRL1..5` | R/W | `0x00` | FIFO threshold, decimation, ODR, and mode. | Datasheet, pp. 54-58 |
| `0x0B` | `DRDY_PULSE_CFG_G` | R/W | `0x00` | Data-ready pulse mode and wrist-tilt route to INT2. | Datasheet, p. 59 |
| `0x0D` | `INT1_CTRL` | R/W | `0x00` | Route DRDY, FIFO, boot, and motion signals to INT1. | Datasheet, p. 59 |
| `0x0E` | `INT2_CTRL` | R/W | `0x00` | Route DRDY, FIFO, temperature, and step signals to INT2. | Datasheet, p. 60 |
| `0x0F` | `WHO_AM_I` | R | `0x6A` | Fixed identity. | Datasheet, p. 60 |
| `0x10` | `CTRL1_XL` | R/W | `0x00` | Accelerometer ODR, full scale, bandwidth. | Datasheet, p. 61 |
| `0x11` | `CTRL2_G` | R/W | `0x00` | Gyroscope ODR and full scale. | Datasheet, p. 62 |
| `0x12` | `CTRL3_C` | R/W | `0x04` | BOOT, BDU, interrupt polarity/output type, SPI mode, auto-increment, endian, software reset. | Datasheet, pp. 49, 63 |
| `0x13` | `CTRL4_C` | R/W | `0x00` | Gyro sleep, all interrupts on INT1, data-ready mask, I2C disable, gyro LPF1 enable. | Datasheet, p. 64 |
| `0x14` | `CTRL5_C` | R/W | `0x00` | Rounding, DEN polarity, gyro/accel self-test. | Datasheet, pp. 64-65 |
| `0x15` | `CTRL6_C` | R/W | `0x00` | Accelerometer high-performance disable, user-offset weight, gyro LPF1 bandwidth. | Datasheet, p. 66 |
| `0x16` | `CTRL7_G` | R/W | `0x00` | Gyro high-performance disable and high-pass filter. | Datasheet, p. 67 |
| `0x17` | `CTRL8_XL` | R/W | `0x00` | Accelerometer LPF/HPF and 6D filter options. | Datasheet, pp. 67-68 |
| `0x18` | `CTRL9_XL` | R/W | `0x00` | DEN stamping and soft-iron correction enable. | Datasheet, p. 68 |
| `0x19` | `CTRL10_C` | R/W | `0x00` | Embedded function enable bits, including pedometer/tilt/wrist tilt. | Datasheet, p. 69 |
| `0x1E` | `STATUS_REG` | R | output | Temperature, gyro, and accel data-available bits. | Datasheet, p. 72 |
| `0x20..0x2D` | output registers | R | output | Temperature, gyro, and accel output words. | Datasheet, pp. 73-76 |
| `0x3A..0x3F` | FIFO status/data | R | output | FIFO level/status and FIFO data output. | Datasheet, pp. 79-81 |
| `0x58..0x5F` | interrupt config | R/W | `0x00` | Tap, 6D, wake-up, free-fall, routing. | Datasheet, pp. 51, 87-92 |
| `0x73..0x75` | `X/Y/Z_OFS_USR` | R/W | `0x00` | Accelerometer user offset correction. | Datasheet, p. 52 |

## Key Field Encodings

| Field | Encoding | Source |
|---|---|---|
| `CTRL1_XL.FS_XL` | `00` +/-2 g; `01` +/-16 g; `10` +/-4 g; `11` +/-8 g. | Datasheet, p. 61 |
| `CTRL2_G.FS_G` | `00` 245 dps; `01` 500 dps; `10` 1000 dps; `11` 2000 dps; `FS_125=1` selects 125 dps. | Datasheet, p. 62 |
| `CTRL3_C.BDU` | `0` continuous update; `1` output registers not updated until MSB and LSB are read. | Datasheet, p. 63 |
| `CTRL3_C.IF_INC` | `0` disabled; `1` auto-increment enabled. Default is enabled. | Datasheet, p. 63 |
| `CTRL3_C.SW_RESET` | Set to 1 to reset; bit clears automatically. | Datasheet, p. 63 |
| `FIFO_CTRL5.FIFO_MODE` | `000` bypass, `001` FIFO stops when full, `011` continuous-to-FIFO, `100` bypass-to-continuous, `110` continuous overwrite. | Datasheet, p. 58 |

## ODR Encodings

| Field value | Accelerometer `ODR_XL` with `XL_HM_MODE=1` | Accelerometer `ODR_XL` with `XL_HM_MODE=0` | Gyroscope `ODR_G` with `G_HM_MODE=1` | Gyroscope `ODR_G` with `G_HM_MODE=0` | Source |
|---:|---:|---:|---:|---:|---|
| `0000` | Power-down | Power-down | Power-down | Power-down | Datasheet, pp. 61-62 |
| `0001` | 12.5 Hz | 12.5 Hz | 12.5 Hz | 12.5 Hz | Datasheet, pp. 61-62 |
| `0010` | 26 Hz | 26 Hz | 26 Hz | 26 Hz | Datasheet, pp. 61-62 |
| `0011` | 52 Hz | 52 Hz | 52 Hz | 52 Hz | Datasheet, pp. 61-62 |
| `0100` | 104 Hz | 104 Hz | 104 Hz | 104 Hz | Datasheet, pp. 61-62 |
| `0101` | 208 Hz | 208 Hz | 208 Hz | 208 Hz | Datasheet, pp. 61-62 |
| `0110` | 416 Hz | 416 Hz | 416 Hz | 416 Hz | Datasheet, pp. 61-62 |
| `0111` | 833 Hz | 833 Hz | 833 Hz | 833 Hz | Datasheet, pp. 61-62 |
| `1000` | 1.66 kHz | 1.66 kHz | 1.66 kHz | 1.66 kHz | Datasheet, pp. 61-62 |
| `1001` | 3.33 kHz | 3.33 kHz | 3.33 kHz | 3.33 kHz | Datasheet, pp. 61-62 |
| `1010` | 6.66 kHz | 6.66 kHz | 6.66 kHz | 6.66 kHz | Datasheet, pp. 61-62 |
| `1011` | 1.6 Hz low-power-only | 12.5 Hz high-performance | not available | not available | Datasheet, pp. 61-62 |
| `11xx` | not allowed | not allowed | not available | not available | Datasheet, pp. 61-62 |

## Offset Registers

`X_OFS_USR` (`0x73`), `Y_OFS_USR` (`0x74`), and `Z_OFS_USR` (`0x75`) are signed 8-bit two's-complement accelerometer user offsets in range -127..+127. The weight is selected by `CTRL6_C.USR_OFF_W`: 2^-10 g/LSB when `0`, or 2^-6 g/LSB when `1`. Source: datasheet, pp. 66, 96.
