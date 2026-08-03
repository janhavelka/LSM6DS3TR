# Electrical And Timing

> Source confidence: vendor fact unless explicitly marked otherwise. Library
> status: conversion constants are supported; board electrical design remains
> the application and hardware designer's responsibility.

## Electrical Characteristics

| Parameter | Value | Source |
|---|---:|---|
| `Vdd` | 1.71 V to 3.6 V; typ. 1.8 V | Datasheet, p. 24 |
| `Vdd_IO` | min 1.62 V, max `Vdd + 0.1 V` | Datasheet, p. 24 |
| Gyro + accel current, high-performance, ODR 1.6 kHz | typ. 0.90 mA | Datasheet, p. 24 |
| Gyro + accel current, normal mode, ODR 208 Hz | typ. 0.45 mA | Datasheet, p. 24 |
| Gyro + accel current, low-power, ODR 52 Hz | typ. 0.29 mA | Datasheet, p. 24 |
| Accelerometer-only high-performance current | typ. 150 uA below 1.6 kHz, 160 uA at or above 1.6 kHz | Datasheet, p. 24 |
| Accelerometer-only normal current | typ. 85 uA at 208 Hz | Datasheet, p. 24 |
| Accelerometer-only low-power current | typ. 9 uA at 12.5 Hz | Datasheet, p. 24 |
| Gyroscope-only high-performance current | typ. 625 uA at 12.5 Hz through 6.66 kHz | AN5130 Table 7, p. 11 |
| Gyroscope-only normal current | typ. 325 uA at 104 Hz; 430 uA at 208 Hz | AN5130 Table 7, p. 11 |
| Gyroscope-only low-power current | typ. 232/245/270 uA at 12.5/26/52 Hz | AN5130 Table 7, p. 11 |
| Power-down current | typ. 3 uA | Datasheet, p. 24 |
| Turn-on time | typ. 35 ms | Datasheet, p. 24 |
| Digital input high / low | `VIH` min 0.7 x `VDD_IO`; `VIL` max 0.3 x `VDD_IO` | Datasheet, p. 24 |

## Sensitivity Anchors

| Range | Sensitivity | Source |
|---|---:|---|
| Accel +/-2 g | 0.061 mg/LSB | Datasheet, p. 21 |
| Accel +/-4 g | 0.122 mg/LSB | Datasheet, p. 21 |
| Accel +/-8 g | 0.244 mg/LSB | Datasheet, p. 21 |
| Accel +/-16 g | 0.488 mg/LSB | Datasheet, p. 21 |
| Gyro +/-125 dps | 4.375 mdps/LSB | Datasheet, p. 21 |
| Gyro +/-250 dps | 8.75 mdps/LSB | Datasheet, p. 21 |
| Gyro +/-500 dps | 17.50 mdps/LSB | Datasheet, p. 21 |
| Gyro +/-1000 dps | 35 mdps/LSB | Datasheet, p. 21 |
| Gyro +/-2000 dps | 70 mdps/LSB | Datasheet, p. 21 |

Use the declared sensitivities directly. In particular, do not divide the
nominal +/-2000 dps label by the signed-code count to invent 61.03 mdps/LSB;
the characterized transfer function is 70 mdps/LSB. The register-table
245/250 dps naming difference is handled in the
[ambiguity ledger](12_source_ambiguities.md). Source: datasheet, pp. 21, 62.

The temperature channel is characterized with typ. +/-15 degC offset over its
operating range and typ. 500 us stabilization time; it is not a precision
thermometer. Source: datasheet, p. 25.

## Bus Timing

| Interface | Timing facts | Source |
|---|---|---|
| SPI | Max 10 MHz; 100 ns clock cycle; CS setup min 5 ns, CS hold min 20 ns, SDI setup/hold min 5/15 ns, SDO valid max 50 ns, SDO hold min 5 ns, and SDO disable max 50 ns. Values apply to 4-wire and 3-wire SPI. | Datasheet, p. 26 |
| I2C slave standard mode | 0 to 100 kHz. | Datasheet, p. 27 |
| I2C slave fast mode | 0 to 400 kHz. | Datasheet, p. 27 |
| I2C sensor-hub master mode | Fixed generated SCL of 116.3 kHz; the timing table is characterized against Fast-mode limits. | Datasheet Table 8, p. 28 |

Operating temperature range is -40 to +85 degC. Source: datasheet, pp. 22, 24-25.

## Board And Bus Design Boundaries

- This compact note is not sufficient for board layout or absolute maximum design limits. Use the datasheet absolute maximum and package/land-pattern tables before PCB work. Source: datasheet, pp. 29, 105-111.
- I2C lines require external pull-up resistors connected to `Vdd_IO`; pull-up sizing belongs to the board design and bus capacitance budget. Source: datasheet, p. 39.
- The source tables include detailed SPI setup/hold timing and I2C setup/hold/bus-free timing beyond the max bus-rate summary above. Source: datasheet, pp. 26-28.
