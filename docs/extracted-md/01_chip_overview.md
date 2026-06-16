# Chip Overview

LSM6DS3TR-C is a system-in-package with a 3-axis digital accelerometer and a 3-axis digital gyroscope. It supports I2C and SPI host interfaces, includes up to 4 kbyte FIFO, and has embedded low-power motion functions. Source: datasheet, pp. 15-16.

## Core Capabilities

| Area | Facts | Source |
|---|---|---|
| Package | 2.5 mm x 3.0 mm x 0.83 mm plastic LGA. | Datasheet, p. 15 |
| Accelerometer full-scale | +/-2 g, +/-4 g, +/-8 g, +/-16 g. | Datasheet, p. 21 |
| Gyroscope full-scale | +/-125, +/-250, +/-500, +/-1000, +/-2000 dps. | Datasheet, pp. 21-22 |
| Accelerometer ODR | 1.6 Hz low-power-only, then 12.5, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664 Hz. | Datasheet, p. 22 |
| Gyroscope ODR | 12.5, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664 Hz. | Datasheet, p. 22 |
| Temperature sensor | 16-bit output, 52 Hz refresh, 256 LSB/degC, typ. 0 LSB at 25 degC. | Datasheet, p. 25 |
| FIFO | Up to 4 kbyte with dynamic allocation of accelerometer and gyroscope data. | Datasheet, pp. 15-16 |
| Embedded functions | Free-fall, wakeup, 6D orientation, tap/double tap, activity/inactivity, pedometer, tilt, significant motion, sensor hub. | Datasheet, p. 16 |

## Device Identity

The `WHO_AM_I` register at `0x0F` is read-only and fixed at `0x6A`. Source: datasheet, p. 60.
