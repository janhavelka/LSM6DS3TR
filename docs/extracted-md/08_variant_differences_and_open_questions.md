# Variants And Open Questions

## Naming And Source Priority

| Topic | Note | Source |
|---|---|---|
| Device name | The primary PDF is for `LSM6DS3TR-C`; compact notes keep that suffix when citing datasheet facts. | Datasheet, p. 1 |
| Source priority | Register addresses and bit meanings in these notes come from the Rev. 3 datasheet, not general design-tip PDFs. | Inventory |
| Application note | AN5130 is relevant for feature recipes, but the compact notes do not import its long raw examples. | Inventory |

## Revision Notes

- The source datasheet is Rev. 3, May 2017. Source: datasheet, p. 1.
- Revision history includes updates to the I2C interface section, FIFO section, FIFO reading procedure, `SW_RESET`, `CTRL1_XL`, and `CTRL8_XL`. Source: datasheet, p. 113.

## Open Questions For Implementation

- Gyro range naming differs across source tables: the sensitivity table uses +/-250 dps with 8.75 mdps/LSB, while the `CTRL2_G.FS_G=00` register table labels the range as 245 dps. Conversion code should follow the sensitivity table and keep comments explicit about the naming discrepancy. Source: datasheet, pp. 21, 62.

## Feature Scope Facts

| Area | LSM6DS3TR-C fact | Source |
|---|---|---|
| Basic UI path | I2C address is `0x6A`/`0x6B`; `WHO_AM_I=0x6A`; accel outputs are `0x28..0x2D`; gyro outputs are `0x22..0x27`. | Datasheet, pp. 39-40, 60, 73-76 |
| SPI path | 3-wire and 4-wire SPI are supported up to 10 MHz; 3-wire mode uses `CTRL3_C.SIM=1`. | Datasheet, pp. 26, 41-44, 63 |
| FIFO path | FIFO can store gyro, accel, third/fourth data sets, and temperature; unread FIFO word count is `FIFO_STATUS1` plus `FIFO_STATUS2.DIFF_FIFO[10:8]`. | Datasheet, pp. 54-58, 79-81 |
| Sensor hub | Mode 2 exposes master I2C pins on `SDx`/`SCx`; sensor-hub registers include `MASTER_CONFIG` and `SENSORHUB1_REG..SENSORHUB18_REG`. | Datasheet, pp. 20, 38, 69, 76-85 |
| Embedded functions | Pedometer, wrist tilt, free-fall, wake-up, tap, inactivity, and 6D routing use registers in `0x53..0x5F` and embedded banks selected by `FUNC_CFG_ACCESS`. | Datasheet, pp. 86-98 |

## Embedded Function Bank Anchors

Embedded-bank registers are selected through `FUNC_CFG_ACCESS`; the compact notes only anchor the bank names and library-relevant register groups. Use the raw datasheet extract or PDF before changing embedded function bank bit fields.

| Bank | Register anchors | Role | Source |
|---|---|---|---|
| Bank A | `SLV0_ADD`, `SLV0_SUBADD`, `SLAVE0_CONFIG` | Sensor-hub slave 0 address, subaddress, and transfer configuration. | Datasheet, pp. 99-101 |
| Bank A | `CONFIG_PEDO_THS_MIN`, `SM_THS`, `PEDO_DEB_REG`, `STEP_COUNT_DELTA` | Pedometer and significant-motion thresholds/debounce. | Datasheet, pp. 104-106 |
| Bank A | `MAG_SI_XX` through `MAG_SI_ZZ`, `MAG_OFFX_L` through `MAG_OFFZ_H` | Magnetometer soft-iron and hard-iron correction registers. | Datasheet, pp. 101-104 |
| Bank B | `A_WRIST_TILT_LAT`, `A_WRIST_TILT_THS`, `A_WRIST_TILT_MASK` | Absolute wrist-tilt latency, threshold, and axis mask. | Datasheet, p. 109 |

## Not Documented In Compact Notes

- Full package drawing dimensions and land-pattern coordinates are figure data in the datasheet; use the PDF pages before board-layout work. Source: datasheet, pp. 105-111.
