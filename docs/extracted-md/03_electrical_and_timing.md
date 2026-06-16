# Electrical And Timing

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

## Bus Timing

| Interface | Timing facts | Source |
|---|---|---|
| SPI | Max 10 MHz; 100 ns clock cycle; values apply to 4-wire and 3-wire SPI. | Datasheet, p. 26 |
| I2C slave standard mode | 0 to 100 kHz. | Datasheet, p. 27 |
| I2C slave fast mode | 0 to 400 kHz. | Datasheet, p. 27 |
| I2C master mode | Fast Mode only; max 400 kHz and source table gives 116.3 kHz min generated SCL. | Datasheet, p. 28 |

Operating temperature range is -40 to +85 degC. Source: datasheet, pp. 22, 24-25.
