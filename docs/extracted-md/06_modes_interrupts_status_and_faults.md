# Modes, Interrupts, Status, And Faults

## ODR And Power Modes

- Accelerometer and gyroscope ODR fields select both output data rate and operating mode. `0000` powers down the selected sensor. Source: datasheet, pp. 61-62.
- Accelerometer `XL_HM_MODE=1` disables high-performance mode; gyroscope `G_HM_MODE=1` disables high-performance mode. Source: datasheet, pp. 66-67.
- Gyroscope sleep mode is controlled by `CTRL4_C.SLEEP`. Source: datasheet, p. 64.
- After ODR or power-mode changes, discard or mask early samples according to the AN5130 settling guidance; the gyroscope power-down wake path can require about 70 ms before data are usable. Source: AN5130, section 3.2.

## Status Bits

| Register/bit | Meaning | Source |
|---|---|---|
| `STATUS_REG.TDA` | New temperature data available. | Datasheet, p. 72 |
| `STATUS_REG.GDA` | New gyroscope data available. | Datasheet, p. 72 |
| `STATUS_REG.XLDA` | New accelerometer data available. | Datasheet, p. 72 |
| `FIFO_STATUS2.WaterM` | FIFO level reached or exceeded watermark. | Datasheet, p. 80 |
| `FIFO_STATUS2.OVER_RUN` | FIFO completely filled. | Datasheet, p. 80 |
| `FIFO_STATUS2.FIFO_EMPTY` | FIFO contains no data when set. | Datasheet, p. 80 |

## Source And Fault Registers

| Register | Role | Source |
|---|---|---|
| `WAKE_UP_SRC`, `TAP_SRC`, `D6D_SRC` | Wake/free-fall, tap, and 6D orientation event sources. | Datasheet, pp. 70-71 |
| `FUNC_SRC1`, `FUNC_SRC2`, `WRIST_TILT_IA` | Embedded-function, sensor-hub, and wrist-tilt event source flags. | Datasheet, pp. 86-87 |
| `SENS_SYNC_SPI_ERROR_CODE` | Sensor-sync SPI error code. | Datasheet, p. 93 |

## Interrupt Routing

| Register | Routes | Source |
|---|---|---|
| `INT1_CTRL` (`0x0D`) | Step detector, significant motion, FIFO full/overrun/watermark, boot, gyro data-ready, accel data-ready to INT1. | Datasheet, p. 59 |
| `INT2_CTRL` (`0x0E`) | Step delta/count overflow, FIFO full/overrun/watermark, temperature data-ready, gyro data-ready, accel data-ready to INT2. | Datasheet, p. 60 |
| `MD1_CFG` / `MD2_CFG` | Embedded function routes such as inactivity, tap, wakeup, free-fall, 6D, tilt to INT1/INT2. | Datasheet, pp. 91-92 |
| `DRDY_PULSE_CFG_G.DRDY_PULSED` | `0` latched data-ready; `1` pulsed data-ready with 75 us pulses. | Datasheet, p. 59 |

## FIFO

- FIFO threshold is 11 bits across `FIFO_CTRL1.FTH[7:0]` and `FIFO_CTRL2.FTH[10:8]`; minimum resolution is 1 word = 2 bytes. Source: datasheet, pp. 54-55.
- FIFO can include gyro, accelerometer, third/fourth data sets, and optionally temperature. Source: datasheet, pp. 55-58.
- For proper FIFO status/data reads, the datasheet recommends setting `CTRL3_C.BDU=1`. Source: datasheet, pp. 79-81.
- FIFO data are untagged; use `FIFO_PATTERN` and the configured decimation/order to identify stream position. Do not change FIFO configuration while unread data remain, do not read when `FIFO_EMPTY` is set, and drain fast enough to free slots before overflow. Source: AN5130, FIFO reading procedure.

## Self-Test

`CTRL5_C` controls gyro and accelerometer self-test. `ST_G=00` is normal mode, `01` positive sign, `11` negative sign, and `10` is not allowed. `ST_XL=00` is normal mode, `01` positive sign, `10` negative sign, and `11` is not allowed. Source: datasheet, pp. 64-65.

## FIFO Modes

| `FIFO_MODE[2:0]` | Behavior | Source |
|---:|---|---|
| `000` | Bypass; FIFO disabled. | Datasheet, p. 58 |
| `001` | FIFO mode; stops collecting data when full. | Datasheet, p. 58 |
| `011` | Continuous-to-FIFO; continuous mode until trigger is deasserted, then FIFO mode. | Datasheet, p. 58 |
| `100` | Bypass-to-continuous; bypass until trigger is deasserted, then continuous mode. | Datasheet, p. 58 |
| `110` | Continuous; when full, new sample overwrites older sample. | Datasheet, p. 58 |

`FIFO_MODE` values `010`, `101`, and `111` are not assigned in the datasheet table. Source: datasheet, p. 58.
