# LSM6DS3TR-C — Extraction Document

## 1. Source Documents

| # | Document Title | Filename | ST Literature # | Pages | Role |
|---|---------------|----------|-----------------|-------|------|
| 1 | LSM6DS3TR-C iNEMO inertial module: always-on 3D accelerometer and 3D gyroscope — Datasheet | `datasheet_LSM6DS3TR-C.pdf` | DocID030071 Rev 3 (May 2017) | 114 | Primary datasheet |
| 2 | LSM6DS3TR-C Application Note | `application_note.pdf` | AN5130 Rev 1 (Mar 2018) | 109 | Primary application note — programming sequences, filters, interrupts, FIFO, sensor hub, embedded functions |
| 3 | 1-point or 3-point tumble sensor calibration | `calibration_1_or_3_point.pdf` | DT0105 Rev 2 (Jan 2022) | 5 | Design tip — accelerometer offset/gain calibration |
| 4 | Computing tilt measurement and tilt-compensated eCompass | `computing_tilt.pdf` | DT0058 Rev 3 (Jan 2021) | 6 | Design tip — roll/pitch/yaw from accel+mag data |
| 5 | Residual linear acceleration by gravity subtraction to enable dead-reckoning | `dead_reckoning.pdf` | DT0106 Rev 1 (Aug 2018) | 6 | Design tip — linear acceleration extraction & double integration |
| 6 | Noise analysis and identification in MEMS sensors | `noise_analysis.pdf` | DT0064 Rev 1 (Jul 2016) | 6 | Design tip — Allan/Hadamard variance, angular random walk |
| 7 | Exploiting the gyroscope to update tilt measurement and eCompass | `updating_tilt_measurement.pdf` | DT0060 Rev 3 (Jan 2021) | 7 | Design tip — gyro-based orientation update, quaternion implementation |

**Implementation-relevant caveats from secondary documents:**
- Accelerometer 1-point calibration computes offset per axis; 3-point calibration computes both offset and gain per axis. Assumes no cross-axis sensitivity. (DT0105, p1–3)
- Tilt (roll/pitch) from accelerometer: `Phi = Atan2(Gy, Gz)`, `Theta = Atan(-Gx / Gz2)`. For gimbal-lock stability at ±90° pitch, substitute `Gz` with `Gz + Gx*alpha` (alpha = 0.01–0.05). (DT0058, p1–2)
- Tilt cannot be computed from accelerometer if high-g motion is ongoing; accelerometer modulus must be near 1 g. Gyroscope data should be used during such intervals. (DT0058, p2)
- Dead-reckoning by double integration of residual linear acceleration drifts due to offset. Leaky integrator (alpha = 0.9–0.95) and Zero-Velocity-Update (ZVU) help control drift. (DT0106, p4–5)
- Gyroscope Angular Random Walk (ARW): `ARW [deg/√s] = noise density [dps/√Hz]`. Final angle error RMS = ARW × √(time). (DT0064, p5)
- Gyroscope bias must be subtracted before integration. Bias estimated by averaging output when system is stationary. Ts accuracy critical for orientation tracking. (DT0060, p4)
- Quaternion update avoids gimbal-lock singularity present in Euler angle formulation. (DT0060, p3)

---

## 2. Device Identity and Variants

| Field | Value | Source |
|-------|-------|--------|
| Device name | LSM6DS3TR-C | datasheet_LSM6DS3TR-C.pdf, p1 |
| Full title | iNEMO inertial module: always-on 3D accelerometer and 3D gyroscope | datasheet_LSM6DS3TR-C.pdf, p1 |
| Package | LGA-14L | datasheet_LSM6DS3TR-C.pdf, p1 |
| Body size | 2.5 mm × 3 mm × 0.83 mm (typ.) | datasheet_LSM6DS3TR-C.pdf, p1 |
| Pin count | 14 | datasheet_LSM6DS3TR-C.pdf, p20 |
| WHO_AM_I register value | 0x6A (register address 0x0F) | datasheet_LSM6DS3TR-C.pdf, p48 |
| Operating temperature | −40 °C to +85 °C | datasheet_LSM6DS3TR-C.pdf, p1 |
| Packing | Tape & Reel | datasheet_LSM6DS3TR-C.pdf, p1 |
| ST literature number | DocID030071 Rev 3 | datasheet_LSM6DS3TR-C.pdf, p1 |

**Note:** No variant devices (e.g. automotive suffix, SPI-only variant) are mentioned in the source documents for the LSM6DS3TR-C specifically. The datasheet does not reference a "-Q1" or other suffix variant.

---

## 3. High-Level Functional Summary

The LSM6DS3TR-C is a system-in-package featuring a 3D digital accelerometer and a 3D digital gyroscope. Key capabilities:

- **Accelerometer**: ±2/±4/±8/±16 g full scale, ODR from 1.6 Hz (low-power only) to 6.66 kHz (datasheet_LSM6DS3TR-C.pdf, p21–22)
- **Gyroscope**: ±125/±250/±500/±1000/±2000 dps full scale, ODR from 12.5 Hz to 6.66 kHz (datasheet_LSM6DS3TR-C.pdf, p21–22)
- **FIFO**: 4 kbyte buffer with 5 operating modes (Bypass, FIFO, Continuous, Continuous-to-FIFO, Bypass-to-Continuous); stores gyro, accel, external sensor, step counter/timestamp, and temperature data (AN5130, p74)
- **Temperature sensor**: 16-bit, 256 LSB/°C, 0 LSB at 25 °C, max 52 Hz refresh (datasheet_LSM6DS3TR-C.pdf, p25)
- **Timestamp**: 24-bit counter, resolution selectable as 6.4 ms or 25 μs (AN5130, p57)
- **Event detection**: Free-fall, wake-up, 6D/4D orientation, single/double tap, activity/inactivity (datasheet_LSM6DS3TR-C.pdf, p16)
- **Embedded functions**: Pedometer (step detector + step counter), significant motion, relative tilt, absolute wrist tilt — all hardware-implemented at 26 Hz (AN5130, p50)
- **Sensor hub**: I2C master for up to 4 external sensors (Mode 2), with hard-iron/soft-iron magnetometer correction (datasheet_LSM6DS3TR-C.pdf, p16; AN5130, p58)
- **Interfaces**: I2C slave (standard/fast mode, up to 400 kHz), SPI (3-wire and 4-wire, up to 10 MHz), I2C master (116.3 kHz) (datasheet_LSM6DS3TR-C.pdf, p26–28)

Power consumption: 0.90 mA combo high-performance mode; 3 μA power-down mode. (datasheet_LSM6DS3TR-C.pdf, p1, p24)

---

## 4. Interface Summary

### I2C Slave Interface

| Parameter | Value | Source |
|-----------|-------|--------|
| Interface type | I2C slave | datasheet_LSM6DS3TR-C.pdf, p20 |
| Standard mode clock | 0–100 kHz | datasheet_LSM6DS3TR-C.pdf, p27 |
| Fast mode clock | 0–400 kHz | datasheet_LSM6DS3TR-C.pdf, p27 |
| Address format | 7-bit: `110101x`b | datasheet_LSM6DS3TR-C.pdf, p39 |
| SA0 = 0 (SDO/SA0 → GND) | 0x6A | datasheet_LSM6DS3TR-C.pdf, p39 |
| SA0 = 1 (SDO/SA0 → VDD_IO) | 0x6B | datasheet_LSM6DS3TR-C.pdf, p39 |
| SDA type | Open-drain bidirectional | datasheet_LSM6DS3TR-C.pdf, p20 |
| Auto-increment | Enabled by default (IF_INC bit in CTRL3_C = 1) | datasheet_LSM6DS3TR-C.pdf, p50 |
| Sub-address MSB (read) | Set to 1 for auto-increment multi-byte reads | datasheet_LSM6DS3TR-C.pdf, p39 |

### SPI Interface

| Parameter | Value | Source |
|-----------|-------|--------|
| SPI modes | 3-wire and 4-wire | datasheet_LSM6DS3TR-C.pdf, p41 |
| Max clock frequency | 10 MHz | datasheet_LSM6DS3TR-C.pdf, p26 |
| CPOL/CPHA | 00 or 11 (SPI mode 0 or 3) | datasheet_LSM6DS3TR-C.pdf, p41 |
| CS pin | Pin 12; 0 = SPI enabled, 1 = I2C enabled | datasheet_LSM6DS3TR-C.pdf, p20 |
| Read/write bit | bit 0 of first byte: 0 = write, 1 = read | datasheet_LSM6DS3TR-C.pdf, p41 |

### I2C Master Interface (Sensor Hub)

| Parameter | Value | Source |
|-----------|-------|--------|
| Master clock frequency | 116.3 kHz | datasheet_LSM6DS3TR-C.pdf, p28 |
| Max external sensors | 4 | AN5130, p58 |
| Data pins | SDx (pin 2 = MSDA), SCx (pin 3 = MSCL) | datasheet_LSM6DS3TR-C.pdf, p20 |
| Internal pull-up | Configurable via PULL_UP_EN bit in MASTER_CONFIG | AN5130, p60 |
| Trigger signal | Accelerometer DRDY (default) or external via INT2 pin | AN5130, p59 |

---

## 5. Pin Configuration and Package

### Pin Table (LGA-14L)

| Pin # | Name | Mode 1 Function | Mode 2 Function | Source |
|-------|------|-----------------|-----------------|--------|
| 1 | SDO/SA0 | SPI SDO / I2C SA0 (LSB) | SPI SDO / I2C SA0 (LSB) | datasheet_LSM6DS3TR-C.pdf, p20 |
| 2 | SDx | Connect to VDDIO or GND | I2C master data (MSDA) | datasheet_LSM6DS3TR-C.pdf, p20 |
| 3 | SCx | Connect to VDDIO or GND | I2C master clock (MSCL) | datasheet_LSM6DS3TR-C.pdf, p20 |
| 4 | INT1 | Programmable interrupt 1 | Programmable interrupt 1 | datasheet_LSM6DS3TR-C.pdf, p20 |
| 5 | VDDIO | Power supply for I/O pins | Power supply for I/O pins | datasheet_LSM6DS3TR-C.pdf, p20 |
| 6 | GND | Ground | Ground | datasheet_LSM6DS3TR-C.pdf, p20 |
| 7 | GND | Ground | Ground | datasheet_LSM6DS3TR-C.pdf, p20 |
| 8 | VDD | Power supply | Power supply | datasheet_LSM6DS3TR-C.pdf, p20 |
| 9 | INT2 | Interrupt 2 / DEN | Interrupt 2 / DEN / MDRDY | datasheet_LSM6DS3TR-C.pdf, p20 |
| 10 | NC | Leave unconnected | Leave unconnected | datasheet_LSM6DS3TR-C.pdf, p20 |
| 11 | NC | Leave unconnected | Leave unconnected | datasheet_LSM6DS3TR-C.pdf, p20 |
| 12 | CS | I2C/SPI mode select (1=I2C, 0=SPI) | I2C/SPI mode select | datasheet_LSM6DS3TR-C.pdf, p20 |
| 13 | SCL | I2C SCL / SPI SPC | I2C SCL / SPI SPC | datasheet_LSM6DS3TR-C.pdf, p20 |
| 14 | SDA | I2C SDA / SPI SDI / 3-wire SDO | I2C SDA / SPI SDI / 3-wire SDO | datasheet_LSM6DS3TR-C.pdf, p20 |

**NC pin note:** Pins 10–11 have internal pull-ups that can be disabled. Disable procedure: write 80h→FUNC_CFG_ACCESS, then 01h→PEDO_DEB_REG (0x15=05h), then 00h→FUNC_CFG_ACCESS. (datasheet_LSM6DS3TR-C.pdf, p42)

**Decoupling capacitors:** 100 nF recommended on VDD (pin 8) and VDDIO (pin 5). (datasheet_LSM6DS3TR-C.pdf, p20)

---

## 6. Absolute Maximum Ratings and Recommended Operating Conditions

### Absolute Maximum Ratings

| Parameter | Value | Source |
|-----------|-------|--------|
| Supply voltage (VDD) | −0.3 V to 4.8 V | datasheet_LSM6DS3TR-C.pdf, p29 |
| Storage temperature | −40 °C to +125 °C | datasheet_LSM6DS3TR-C.pdf, p29 |
| Mechanical shock (0.2 ms) | 10,000 g | datasheet_LSM6DS3TR-C.pdf, p29 |
| ESD protection (HBM) | 2 kV | datasheet_LSM6DS3TR-C.pdf, p29 |
| Input voltage on control pins | −0.3 V to VDD_IO + 0.3 V | datasheet_LSM6DS3TR-C.pdf, p29 |

**Warning:** Supply voltage on any pin should never exceed 4.8 V. Device is sensitive to mechanical shock and ESD. (datasheet_LSM6DS3TR-C.pdf, p29)

### Recommended Operating Conditions

| Parameter | Min | Typ | Max | Unit | Source |
|-----------|-----|-----|-----|------|--------|
| VDD | 1.71 | 1.8 | 3.6 | V | datasheet_LSM6DS3TR-C.pdf, p24 |
| VDDIO | 1.62 | — | VDD + 0.1 | V | datasheet_LSM6DS3TR-C.pdf, p24 |
| Operating temperature | −40 | — | +85 | °C | datasheet_LSM6DS3TR-C.pdf, p24 |

### Digital I/O Levels (@ VDD = 1.8 V)

| Parameter | Value | Source |
|-----------|-------|--------|
| VIH (input high) | 0.7 × VDD_IO | datasheet_LSM6DS3TR-C.pdf, p24 |
| VIL (input low) | 0.3 × VDD_IO | datasheet_LSM6DS3TR-C.pdf, p24 |
| VOH (output high, IOH = 4 mA) | VDD_IO − 0.2 V | datasheet_LSM6DS3TR-C.pdf, p24 |
| VOL (output low, IOL = 4 mA) | 0.2 V | datasheet_LSM6DS3TR-C.pdf, p24 |

---

## 7. Electrical Characteristics

### Power Supply and Current Consumption (@ VDD = 1.8 V, T = 25 °C)

| Parameter | Test Conditions | Typ | Unit | Source |
|-----------|----------------|-----|------|--------|
| Combo high-performance mode | ODR = 1.6 kHz | 0.90 | mA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Combo normal mode | ODR = 208 Hz | 0.45 | mA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Combo low-power mode | ODR = 52 Hz | 0.29 | mA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Accelerometer HP mode | ODR < 1.6 kHz | 150 | μA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Accelerometer HP mode | ODR ≥ 1.6 kHz | 160 | μA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Accelerometer normal mode | ODR = 208 Hz | 85 | μA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Accelerometer low-power mode | ODR = 12.5 Hz | 9 | μA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Power-down (both sensors) | — | 3 | μA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Turn-on time | — | 35 | ms | datasheet_LSM6DS3TR-C.pdf, p24 |

### Accelerometer Mechanical Characteristics (@ VDD = 1.8 V, T = 25 °C)

| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| Full-scale range | ±2 / ±4 / ±8 / ±16 | g | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity (±2 g) | 0.061 | mg/LSB | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity (±4 g) | 0.122 | mg/LSB | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity (±8 g) | 0.244 | mg/LSB | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity (±16 g) | 0.488 | mg/LSB | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity change vs. temp | ±0.01 | %/°C | datasheet_LSM6DS3TR-C.pdf, p21 |
| Zero-g offset accuracy | ±40 | mg | datasheet_LSM6DS3TR-C.pdf, p21 |
| Zero-g level change vs. temp | ±0.5 | mg/°C | datasheet_LSM6DS3TR-C.pdf, p21 |
| Noise density (HP mode, ≤ ±8 g) | 90 | μg/√Hz | datasheet_LSM6DS3TR-C.pdf, p22 |
| Noise density (HP mode, ±16 g) | 130 | μg/√Hz | datasheet_LSM6DS3TR-C.pdf, p22 |
| RMS noise (normal/LP, ±2 g) | 1.7 | mg RMS | datasheet_LSM6DS3TR-C.pdf, p22 |
| RMS noise (normal/LP, ±4 g) | 2.0 | mg RMS | datasheet_LSM6DS3TR-C.pdf, p22 |
| RMS noise (normal/LP, ±8 g) | 2.4 | mg RMS | datasheet_LSM6DS3TR-C.pdf, p22 |
| RMS noise (normal/LP, ±16 g) | 3.0 | mg RMS | datasheet_LSM6DS3TR-C.pdf, p22 |
| Self-test output change | 90–1700 | mg | datasheet_LSM6DS3TR-C.pdf, p22 |

### Gyroscope Mechanical Characteristics (@ VDD = 1.8 V, T = 25 °C)

| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| Full-scale range | ±125 / ±245 / ±500 / ±1000 / ±2000 | dps | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity (±125 dps) | 4.375 | mdps/LSB | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity (±250 dps) | 8.75 | mdps/LSB | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity (±500 dps) | 17.50 | mdps/LSB | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity (±1000 dps) | 35 | mdps/LSB | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity (±2000 dps) | 70 | mdps/LSB | datasheet_LSM6DS3TR-C.pdf, p21 |
| Sensitivity change vs. temp | ±0.007 | %/°C | datasheet_LSM6DS3TR-C.pdf, p21 |
| Zero-rate level | ±3 | dps | datasheet_LSM6DS3TR-C.pdf, p21 |
| Zero-rate change vs. temp | ±0.05 | dps/°C | datasheet_LSM6DS3TR-C.pdf, p21 |
| Rate noise density (HP mode) | 5 | mdps/√Hz | datasheet_LSM6DS3TR-C.pdf, p21 |
| RMS noise (normal/LP mode) | 75 | mdps | datasheet_LSM6DS3TR-C.pdf, p21 |
| Self-test change (±250 dps) | 20–80 | dps | datasheet_LSM6DS3TR-C.pdf, p22 |
| Self-test change (±2000 dps) | 150–700 | dps | datasheet_LSM6DS3TR-C.pdf, p22 |

### Temperature Sensor Characteristics

| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| Refresh rate | 52 (max) | Hz | datasheet_LSM6DS3TR-C.pdf, p25 |
| Offset | −15 to +15 (0 LSB typ. at 25 °C) | °C | datasheet_LSM6DS3TR-C.pdf, p25 |
| Sensitivity | 256 | LSB/°C | datasheet_LSM6DS3TR-C.pdf, p25 |
| Stabilization time | 500 | μs | datasheet_LSM6DS3TR-C.pdf, p25 |
| ADC resolution | 16 | bit | datasheet_LSM6DS3TR-C.pdf, p25 |

---

## 8. Power Modes and Consumption

### Power Mode Selection

The accelerometer and gyroscope operate independently with separate ODR and power mode settings. (datasheet_LSM6DS3TR-C.pdf, p31)

**Accelerometer power modes** (controlled by `XL_HM_MODE` in FUNC_CFG_ACCESS (01h)):
- `XL_HM_MODE = 0`: High-performance mode for all ODRs (12.5 Hz–6.66 kHz)
- `XL_HM_MODE = 1`: Low-power mode (1.6, 12.5, 26, 52 Hz) and Normal mode (104, 208 Hz)

(datasheet_LSM6DS3TR-C.pdf, p31)

**Gyroscope power modes** (controlled by `G_HM_MODE` in CTRL7_G (16h)):
- `G_HM_MODE = 0`: High-performance mode for all ODRs (12.5 Hz–6.66 kHz)
- `G_HM_MODE = 1`: Low-power mode (12.5, 26, 52 Hz) and Normal mode (104, 208 Hz)

(datasheet_LSM6DS3TR-C.pdf, p31)

**Gyroscope Sleep mode**: Gyroscope maintains bias calibration while consuming less power. Enabled via `SLEEP_G` bit in CTRL4_C (13h). (AN5130, p12)

### Accelerometer ODR Table

| ODR_XL[3:0] | ODR (Hz) | Available Power Modes | Source |
|--------------|----------|-----------------------|--------|
| 0000 | Power-down | — | AN5130, p10 |
| 1011 | 1.6 | Low-power only | AN5130, p10 |
| 0001 | 12.5 | LP / HP | AN5130, p10 |
| 0010 | 26 | LP / HP | AN5130, p10 |
| 0011 | 52 | LP / HP | AN5130, p10 |
| 0100 | 104 | Normal / HP | AN5130, p10 |
| 0101 | 208 | Normal / HP | AN5130, p10 |
| 0110 | 416 | HP | AN5130, p10 |
| 0111 | 833 | HP | AN5130, p10 |
| 1000 | 1.66 k | HP | AN5130, p10 |
| 1001 | 3.33 k | HP | AN5130, p10 |
| 1010 | 6.66 k | HP | AN5130, p10 |

### Gyroscope ODR Table

| ODR_G[3:0] | ODR (Hz) | Available Power Modes | Source |
|------------|----------|-----------------------|--------|
| 0000 | Power-down | — | AN5130, p11 |
| 0001 | 12.5 | LP / HP | AN5130, p11 |
| 0010 | 26 | LP / HP | AN5130, p11 |
| 0011 | 52 | LP / HP | AN5130, p11 |
| 0100 | 104 | Normal / HP | AN5130, p11 |
| 0101 | 208 | Normal / HP | AN5130, p11 |
| 0110 | 416 | HP | AN5130, p11 |
| 0111 | 833 | HP | AN5130, p11 |
| 1000 | 1.66 k | HP | AN5130, p11 |
| 1001 | 3.33 k | HP | AN5130, p11 |
| 1010 | 6.66 k | HP | AN5130, p11 |

### Power Consumption Summary

| Configuration | Current | Source |
|---------------|---------|--------|
| Both sensors power-down | 3 μA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Accel only, LP 12.5 Hz | 9 μA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Accel only, normal 208 Hz | 85 μA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Accel only, HP < 1.6 kHz | 150 μA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Accel only, HP ≥ 1.6 kHz | 160 μA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Combo LP 52 Hz | 0.29 mA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Combo normal 208 Hz | 0.45 mA | datasheet_LSM6DS3TR-C.pdf, p24 |
| Combo HP 1.6 kHz | 0.90 mA | datasheet_LSM6DS3TR-C.pdf, p24 |

---

## 9. Register Map (Main Registers)

All registers are 8-bit. Default values indicated where documented. Address space 00h–75h. (datasheet_LSM6DS3TR-C.pdf, p44–96)

| Addr | Name | Type | Default | Description | Source |
|------|------|------|---------|-------------|--------|
| 01h | FUNC_CFG_ACCESS | R/W | 00h | Access embedded function registers; XL_HM_MODE bit (b4) selects accel power mode | datasheet_LSM6DS3TR-C.pdf, p45 |
| 02h | SENSOR_SYNC_TIME_FRAME | R/W | 00h | Sensor sync configuration | datasheet_LSM6DS3TR-C.pdf, p46 |
| 04h | SENSOR_SYNC_RES_RATIO | R/W | 00h | Sensor sync resolution ratio | datasheet_LSM6DS3TR-C.pdf, p46 |
| 06h | FIFO_CTRL1 | R/W | 00h | FIFO threshold low [FTH_7:0] | datasheet_LSM6DS3TR-C.pdf, p46 |
| 07h | FIFO_CTRL2 | R/W | 00h | FIFO threshold high [FTH_10:8], TIMER_PEDO_FIFO_EN/DRDY, FIFO_TEMP_EN | datasheet_LSM6DS3TR-C.pdf, p47 |
| 08h | FIFO_CTRL3 | R/W | 00h | Gyro/accel FIFO decimation [DEC_FIFO_GYRO, DEC_FIFO_XL] | datasheet_LSM6DS3TR-C.pdf, p47 |
| 09h | FIFO_CTRL4 | R/W | 00h | 3rd/4th data set decimation, STOP_ON_FTH, ONLY_HIGH_DATA | datasheet_LSM6DS3TR-C.pdf, p48 |
| 0Ah | FIFO_CTRL5 | R/W | 00h | FIFO ODR [ODR_FIFO_3:0], FIFO mode [FIFO_MODE_2:0] | datasheet_LSM6DS3TR-C.pdf, p48 |
| 0Bh | DRDY_PULSE_CFG_G | R/W | 00h | DRDY pulsed mode, INT2_WRIST_TILT | datasheet_LSM6DS3TR-C.pdf, p49 |
| 0Dh | INT1_CTRL | R/W | 00h | INT1 pin control (DRDY, FIFO, boot, step, sig-motion) | datasheet_LSM6DS3TR-C.pdf, p49 |
| 0Eh | INT2_CTRL | R/W | 00h | INT2 pin control (DRDY, FIFO, step-delta, step-OV, temp) | datasheet_LSM6DS3TR-C.pdf, p50 |
| 0Fh | WHO_AM_I | R | 6Ah | Device identification | datasheet_LSM6DS3TR-C.pdf, p50 |
| 10h | CTRL1_XL | R/W | 00h | Accel ODR [ODR_XL_3:0], full-scale [FS_XL_1:0], LPF1_BW_SEL | datasheet_LSM6DS3TR-C.pdf, p50 |
| 11h | CTRL2_G | R/W | 00h | Gyro ODR [ODR_G_3:0], full-scale [FS_G_1:0], FS_125 | datasheet_LSM6DS3TR-C.pdf, p51 |
| 12h | CTRL3_C | R/W | 04h | BOOT, BDU, H_LACTIVE, PP_OD, SIM, IF_INC, BLE, SW_RESET | datasheet_LSM6DS3TR-C.pdf, p52 |
| 13h | CTRL4_C | R/W | 00h | DEN_XL_EN, SLEEP_G, INT2_on_INT1, DEN_DRDY_INT1, DRDY_MASK, I2C_disable, LPF1_SEL_G | datasheet_LSM6DS3TR-C.pdf, p53 |
| 14h | CTRL5_C | R/W | 00h | Self-test [ST_XL, ST_G], DEN_LH, rounding | datasheet_LSM6DS3TR-C.pdf, p54 |
| 15h | CTRL6_C | R/W | 00h | TRIG_EN, LVL1_EN, LVL2_EN, XL_HM_MODE (redundant — **unclear**: also at 01h b4) | datasheet_LSM6DS3TR-C.pdf, p55 |
| 16h | CTRL7_G | R/W | 00h | G_HM_MODE, HP_EN_G, HPM_G[1:0], ROUNDING_STATUS, HP_G_RST | datasheet_LSM6DS3TR-C.pdf, p56 |
| 17h | CTRL8_XL | R/W | 00h | LPF2_XL_EN, HPCF_XL[1:0], HP_REF_MODE, INPUT_COMPOSITE, HP_SLOPE_XL_EN, LOW_PASS_ON_6D | datasheet_LSM6DS3TR-C.pdf, p57 |
| 18h | CTRL9_XL | R/W | E0h | DEN stamping axis bits [DEN_X, DEN_Y, DEN_Z], SOFT_EN | datasheet_LSM6DS3TR-C.pdf, p58 |
| 19h | CTRL10_C | R/W | 00h | WRIST_TILT_EN, TIMER_EN, PEDO_EN, TILT_EN, FUNC_EN, PEDO_RST_STEP, SIGN_MOTION_EN | datasheet_LSM6DS3TR-C.pdf, p59 |
| 1Ah | MASTER_CONFIG | R/W | 00h | DRDY_ON_INT1, DATA_VALID_SEL_FIFO, START_CONFIG, PULL_UP_EN, PASS_THROUGH_MODE, IRON_EN, MASTER_ON | datasheet_LSM6DS3TR-C.pdf, p60 |
| 1Bh | WAKE_UP_SRC | R | 00h | FF_IA, SLEEP_STATE_IA, WU_IA, X_WU, Y_WU, Z_WU | datasheet_LSM6DS3TR-C.pdf, p61 |
| 1Ch | TAP_SRC | R | 00h | TAP_IA, SINGLE_TAP, DOUBLE_TAP, TAP_SIGN, X/Y/Z_TAP | datasheet_LSM6DS3TR-C.pdf, p62 |
| 1Dh | D6D_SRC | R | 00h | DEN_DRDY, D6D_IA, ZH, ZL, YH, YL, XH, XL | datasheet_LSM6DS3TR-C.pdf, p63 |
| 1Eh | STATUS_REG | R | 00h | TDA (temp), GDA (gyro), XLDA (accel) data available | datasheet_LSM6DS3TR-C.pdf, p64 |
| 20h–21h | OUT_TEMP_L/H | R | 00h | Temperature output (16-bit, two's complement) | datasheet_LSM6DS3TR-C.pdf, p65 |
| 22h–27h | OUTX/Y/Z_L/H_G | R | 00h | Gyroscope output X, Y, Z (16-bit, two's complement each) | datasheet_LSM6DS3TR-C.pdf, p65–67 |
| 28h–2Dh | OUTX/Y/Z_L/H_XL | R | 00h | Accelerometer output X, Y, Z (16-bit, two's complement each) | datasheet_LSM6DS3TR-C.pdf, p67–69 |
| 2Eh–39h | SENSORHUB1–12_REG | R | 00h | External sensor data (1st & 2nd FIFO data sets for hub) | datasheet_LSM6DS3TR-C.pdf, p69–74 |
| 3Ah | FIFO_STATUS1 | R | 00h | DIFF_FIFO[7:0] — unread word count low | datasheet_LSM6DS3TR-C.pdf, p75 |
| 3Bh | FIFO_STATUS2 | R | 00h | WaterM, OVER_RUN, FIFO_FULL_SMART, FIFO_EMPTY, DIFF_FIFO[10:8] | datasheet_LSM6DS3TR-C.pdf, p75 |
| 3Ch–3Dh | FIFO_STATUS3/4 | R | 00h | FIFO_PATTERN[9:0] — identifies next data in FIFO | datasheet_LSM6DS3TR-C.pdf, p76 |
| 3Eh–3Fh | FIFO_DATA_OUT_L/H | R | 00h | FIFO data output (16-bit) | datasheet_LSM6DS3TR-C.pdf, p77 |
| 40h–42h | TIMESTAMP0–2_REG | R | 00h | Timestamp (24-bit) | datasheet_LSM6DS3TR-C.pdf, p77–78 |
| 49h–4Ah | STEP_TIMESTAMP_L/H | R | 00h | Step timestamp (16-bit) | datasheet_LSM6DS3TR-C.pdf, p83 |
| 4Bh–4Ch | STEP_COUNTER_L/H | R | 00h | Step counter (16-bit unsigned) | datasheet_LSM6DS3TR-C.pdf, p83–84 |
| 4Dh–52h | SENSORHUB13–18_REG | R | 00h | External sensor data (secondary set) | datasheet_LSM6DS3TR-C.pdf, p80–82 |
| 53h | FUNC_SRC1 | R | 00h | STEP_OVERFLOW, STEP_COUNT_DELTA_IA, STEP_DETECTED, TILT_IA, SENSORHUB_END_OP, SI_END_OP, HI_FAIL, SIGN_MOTION_IA | datasheet_LSM6DS3TR-C.pdf, p83 |
| 54h | FUNC_SRC2 | R | 00h | WRIST_TILT_IA, SLAVEx_NACK (x=0..3) | datasheet_LSM6DS3TR-C.pdf, p84 |
| 55h | WRIST_TILT_IA | R | 00h | Wrist tilt axis source (X/Y/Z pos/neg) | datasheet_LSM6DS3TR-C.pdf, p85 |
| 58h | TAP_CFG | R/W | 00h | INTERRUPTS_ENABLE, INACT_EN[1:0], SLOPE_FDS, TAP_X/Y/Z_EN, LIR | datasheet_LSM6DS3TR-C.pdf, p86 |
| 59h | TAP_THS_6D | R/W | 00h | D4D_EN, SIXD_THS[1:0], TAP_THS[4:0] | datasheet_LSM6DS3TR-C.pdf, p87 |
| 5Ah | INT_DUR2 | R/W | 00h | DUR[3:0], QUIET[1:0], SHOCK[1:0] | datasheet_LSM6DS3TR-C.pdf, p88 |
| 5Bh | WAKE_UP_THS | R/W | 00h | SINGLE_DOUBLE_TAP, WK_THS[5:0] | datasheet_LSM6DS3TR-C.pdf, p89 |
| 5Ch | WAKE_UP_DUR | R/W | 00h | FF_DUR5, WAKE_DUR[1:0], TIMER_HR, SLEEP_DUR[3:0] | datasheet_LSM6DS3TR-C.pdf, p89 |
| 5Dh | FREE_FALL | R/W | 00h | FF_DUR[4:0] (bits 7:3), FF_THS[2:0] (bits 2:0) | datasheet_LSM6DS3TR-C.pdf, p90 |
| 5Eh | MD1_CFG | R/W | 00h | INT1 routing: INACT, SINGLE_TAP, WU, FF, DOUBLE_TAP, 6D, TILT, TIMER | datasheet_LSM6DS3TR-C.pdf, p91 |
| 5Fh | MD2_CFG | R/W | 00h | INT2 routing: INACT, SINGLE_TAP, WU, FF, DOUBLE_TAP, 6D, TILT, IRON | datasheet_LSM6DS3TR-C.pdf, p92 |
| 60h | MASTER_CMD_CODE | R/W | 00h | Master command code | datasheet_LSM6DS3TR-C.pdf, p93 |
| 61h | SENS_SYNC_SPI_ERROR_CODE | R/W | 00h | Sensor sync SPI error code | datasheet_LSM6DS3TR-C.pdf, p93 |
| 66h–6Bh | OUT_MAG_RAW_X/Y/Z_L/H | R | 00h | Raw magnetometer data (from sensor hub) | datasheet_LSM6DS3TR-C.pdf, p94 |
| 73h | X_OFS_USR | R/W | 00h | Accelerometer X-axis user offset correction | datasheet_LSM6DS3TR-C.pdf, p95 |
| 74h | Y_OFS_USR | R/W | 00h | Accelerometer Y-axis user offset correction | datasheet_LSM6DS3TR-C.pdf, p95 |
| 75h | Z_OFS_USR | R/W | 00h | Accelerometer Z-axis user offset correction | datasheet_LSM6DS3TR-C.pdf, p96 |

**Key control register bits:**

- **CTRL3_C (12h)**: `IF_INC` (auto-increment, default = 1), `BDU` (block data update, set to 1 for proper reading), `BLE` (big/little-endian), `SW_RESET`, `BOOT`. (datasheet_LSM6DS3TR-C.pdf, p52)
- **CTRL1_XL (10h)**: `ODR_XL[3:0]` (bits 7:4), `FS_XL[1:0]` (bits 3:2): 00=±2g, 01=±16g, 10=±4g, 11=±8g. `LPF1_BW_SEL` (bit 1). (datasheet_LSM6DS3TR-C.pdf, p50)
- **CTRL2_G (11h)**: `ODR_G[3:0]` (bits 7:4), `FS_G[1:0]` (bits 3:2): 00=250dps, 01=500dps, 10=1000dps, 11=2000dps. `FS_125` (bit 1): 1=125 dps. (datasheet_LSM6DS3TR-C.pdf, p51)

---

## 10. Embedded Functions Register Map (Bank A & Bank B)

Embedded function registers are accessed by setting `FUNC_CFG_EN` (bit 7) in FUNC_CFG_ACCESS (01h). Bank A = FUNC_CFG_EN=1, FUNC_CFG_EN_B=0. Bank B = FUNC_CFG_EN=1, FUNC_CFG_EN_B=1. (datasheet_LSM6DS3TR-C.pdf, p97)

**Critical:** Modifications to embedded function registers must be done while the device is in power-down mode. (AN5130, p56)

### Bank A Registers

| Addr | Name | Default | Description | Source |
|------|------|---------|-------------|--------|
| 02h | SLV0_ADD | 00h | Slave 0 I2C address [6:0] + rw_0 bit | AN5130, p61 |
| 03h | SLV0_SUBADD | 00h | Slave 0 register address | AN5130, p61 |
| 04h | SLAVE0_CONFIG | 00h | Slave0_rate[1:0], Aux_sens_on[1:0], Src_mode, Slave0_numop[2:0] | AN5130, p61 |
| 05h | SLV1_ADD | 00h | Slave 1 I2C address [6:0] + r_1 bit | AN5130, p62 |
| 06h | SLV1_SUBADD | 00h | Slave 1 register address | AN5130, p62 |
| 07h | SLAVE1_CONFIG | 00h | Slave1_rate[1:0], write_once, Slave1_numop[2:0] | AN5130, p62 |
| 08h | SLV2_ADD | 00h | Slave 2 I2C address [6:0] + r_2 bit | AN5130, p63 |
| 09h | SLV2_SUBADD | 00h | Slave 2 register address | AN5130, p63 |
| 0Ah | SLAVE2_CONFIG | 00h | Slave2_rate[1:0], Slave2_numop[2:0] | AN5130, p63 |
| 0Bh | SLV3_ADD | 00h | Slave 3 I2C address [6:0] + r_3 bit | AN5130, p64 |
| 0Ch | SLV3_SUBADD | 00h | Slave 3 register address | AN5130, p64 |
| 0Dh | SLAVE3_CONFIG | 00h | Slave3_rate[1:0], Slave3_numop[2:0] | AN5130, p64 |
| 0Eh | DATAWRITE_SRC_MODE_SUB_SLV0 | 00h | Write data / source mode conditioned read address | AN5130, p65 |
| 0Fh | CONFIG_PEDO_THS_MIN | 00h | PEDO_FS (bit 7), ths_min[5:0] (pedometer min threshold) | AN5130, p51 |
| 13h | SM_THS | 06h | SM_THS[7:0] — significant motion threshold (default = 6 steps) | AN5130, p53 |
| 14h | PEDO_DEB_REG | — | DEB_TIME[4:0] (default 13 = ~1040 ms), DEB_STEP[2:0] (default 6 steps) | AN5130, p50 |
| 15h | STEP_COUNT_DELTA | 00h | Step count delta (pedometer interrupt period) | AN5130, p52 |
| 24h–2Ch | MAG_SI_XX through MAG_SI_ZZ | 00h | 3×3 soft-iron transformation matrix (sign-magnitude, 1 LSB = 1/8) | AN5130, p70 |
| 2Dh–32h | MAG_OFFX/Y/Z_L/H | 00h | Hard-iron offset vector (16-bit two's complement, LSB = magnetometer sensitivity) | AN5130, p69 |

### Bank B Registers

| Addr | Name | Default | Description | Source |
|------|------|---------|-------------|--------|
| 50h | A_WRIST_TILT_LAT | 0Fh | WRIST_TILT_TIMER[7:0]: latency, 1 LSB = 40 ms, default = 600 ms | AN5130, p55 |
| 54h | A_WRIST_TILT_THS | 20h | WRIST_TILT_THS[5:0]: threshold, 1 LSB = 15.625 mg, default = 500 mg (30°) | AN5130, p55 |
| 59h | A_WRIST_TILT_Mask | 02h | Axis mask bits: Xpos/Xneg/Ypos/Yneg/Zpos/Zneg (default = X-positive) | AN5130, p55 |

**Important:** Bank B registers (AWT) are reset to defaults every time the accelerometer exits Power-Down mode. They must be reconfigured after every power-on. (AN5130, p56)

---

## 11. Accelerometer Configuration and Filtering

### Full-Scale Selection (FS_XL[1:0] in CTRL1_XL)

| FS_XL[1:0] | Full Scale | Sensitivity (mg/LSB) | Source |
|-------------|------------|---------------------|--------|
| 00 | ±2 g | 0.061 | datasheet_LSM6DS3TR-C.pdf, p21 |
| 10 | ±4 g | 0.122 | datasheet_LSM6DS3TR-C.pdf, p21 |
| 11 | ±8 g | 0.244 | datasheet_LSM6DS3TR-C.pdf, p21 |
| 01 | ±16 g | 0.488 | datasheet_LSM6DS3TR-C.pdf, p21 |

**Note:** FS_XL encoding is non-sequential: `01` = ±16 g, not ±4 g. (datasheet_LSM6DS3TR-C.pdf, p50)

### Filtering Chain

The accelerometer digital chain consists of (AN5130, p13):
1. **Analog anti-aliasing filter**: BW selectable 400 Hz or 1.5 kHz via `BW0_XL` bit in CTRL1_XL
2. **Digital LPF1**: Always active. Bandwidth depends on ODR and `LPF1_BW_SEL` bit
3. **Composite filter**: Configured via `INPUT_COMPOSITE` in CTRL8_XL
4. **LPF2 / HPF**: Additional filtering via `LPF2_XL_EN`, `HPCF_XL[1:0]`, `HP_SLOPE_XL_EN` in CTRL8_XL

### Accelerometer LPF2 Cutoff Frequencies (HPCF_XL[1:0])

| HPCF_XL[1:0] | ODR / divisor | Source |
|---------------|---------------|--------|
| 00 | ODR_XL / 50 | AN5130, p14 |
| 01 | ODR_XL / 100 | AN5130, p14 |
| 10 | ODR_XL / 9 | AN5130, p14 |
| 11 | ODR_XL / 400 | AN5130, p14 |

### Slope Filter

The slope filter computes: `slope(tn) = [acc(tn) - acc(tn-1)] / 2`. Selected by `SLOPE_FDS = 0` in TAP_CFG register (default). Used for wake-up and activity/inactivity detection. (AN5130, p15, p36)

### Accelerometer User Offset Registers

Three 8-bit registers (X_OFS_USR, Y_OFS_USR, Z_OFS_USR at 73h–75h) provide user-programmable offset correction. Weight selectable via `USR_OFF_W` bit in CTRL6_C: 0 = 2^−10 g/LSB (~1 mg/LSB), 1 = 2^−6 g/LSB (~15.6 mg/LSB). (AN5130, p27; datasheet_LSM6DS3TR-C.pdf, p95)

### Turn-On Time

| Transition | Time (typ) | Source |
|------------|-----------|--------|
| Power-down → any active mode | See sample discard tables | AN5130, p19 |
| Boot time after power-on | 15 ms | AN5130, p49 |

Samples to discard after power-on depend on ODR and filter configuration — consult Tables 12–13 in AN5130, p19.

---

## 12. Gyroscope Configuration and Filtering

### Full-Scale Selection

| FS_G[1:0] | FS_125 | Full Scale | Sensitivity (mdps/LSB) | Source |
|-----------|--------|------------|----------------------|--------|
| — | 1 | ±125 dps | 4.375 | datasheet_LSM6DS3TR-C.pdf, p21 |
| 00 | 0 | ±250 dps | 8.75 | datasheet_LSM6DS3TR-C.pdf, p21 |
| 01 | 0 | ±500 dps | 17.50 | datasheet_LSM6DS3TR-C.pdf, p21 |
| 10 | 0 | ±1000 dps | 35 | datasheet_LSM6DS3TR-C.pdf, p21 |
| 11 | 0 | ±2000 dps | 70 | datasheet_LSM6DS3TR-C.pdf, p21 |

### Filtering Chain

The gyroscope digital chain consists of (AN5130, p16):
1. **LPF1**: Optional, enabled by `LPF1_SEL_G` bit in CTRL4_C. Cutoff depends on `FTYPE[1:0]` in CTRL6_C
2. **HPF**: Optional, enabled by `HP_EN_G` bit in CTRL7_G. Cutoff set by `HPM_G[1:0]` in CTRL7_G

### Gyroscope HP Filter Cutoff (HPM_G[1:0])

| HPM_G[1:0] | Cutoff (Hz) | Source |
|------------|-------------|--------|
| 00 | 0.0081 | AN5130, p16 |
| 01 | 0.0324 | AN5130, p16 |
| 10 | 2.07 | AN5130, p16 |
| 11 | 16.32 | AN5130, p16 |

### Gyroscope LPF1 Bandwidth (varies with ODR and FTYPE[1:0])

Consult Table 11 in AN5130, p17 for the full matrix. Example at ODR = 416 Hz:

| FTYPE[1:0] | BW (Hz) | Source |
|------------|---------|--------|
| 00 | 171 | AN5130, p17 |
| 01 | 172 | AN5130, p17 |
| 10 | 63 | AN5130, p17 |
| 11 | 58 | AN5130, p17 |

### Gyroscope Turn-On Time

Gyroscope turn-on from power-down: ~70 ms + samples to discard (varies by ODR). See Tables 14–16 in AN5130, p20–21.

### Gyroscope Sleep Mode

Setting `SLEEP_G = 1` in CTRL4_C puts gyroscope into sleep mode (retains bias calibration, lower power). Wake-up is faster than from full power-down. (AN5130, p12)

---

## 13. FIFO Buffer

### Overview

- **Size**: 4 kbyte (4096 bytes), holds up to 2048 × 16-bit samples (when using single sensor) (AN5130, p74)
- **Data sets**: 4 FIFO data sets of 6 bytes each:
  - 1st: Gyroscope data
  - 2nd: Accelerometer data
  - 3rd: External sensor data (SENSORHUB1–6_REG)
  - 4th: External sensor data (SENSORHUB7–12_REG) **or** step counter + timestamp **or** temperature
  (AN5130, p74)
- **Required bits**: `IF_INC = 1` and `BDU = 1` in CTRL3_C when FIFO is used (AN5130, p74, p78)

### FIFO Modes (FIFO_MODE[2:0] in FIFO_CTRL5)

| FIFO_MODE | Mode | Description | Source |
|-----------|------|-------------|--------|
| 000 | Bypass | FIFO disabled, content cleared | AN5130, p81 |
| 001 | FIFO | Stops collecting when full | AN5130, p81 |
| 011 | Continuous-to-FIFO | Continuous until trigger event, then stops | AN5130, p84 |
| 100 | Bypass-to-Continuous | Bypass until trigger event, then continuous | AN5130, p85 |
| 110 | Continuous | Oldest data overwritten when full | AN5130, p83 |

**Note:** Values 010, 101, 111 are reserved. (AN5130, p78)

### FIFO ODR (ODR_FIFO[3:0] in FIFO_CTRL5)

| ODR_FIFO | Rate | Source |
|----------|------|--------|
| 0000 | FIFO disabled | AN5130, p78 |
| 0001 | 12.5 Hz | AN5130, p78 |
| 0010 | 26 Hz | AN5130, p78 |
| 0011 | 52 Hz | AN5130, p78 |
| 0100 | 104 Hz | AN5130, p78 |
| 0101 | 208 Hz | AN5130, p78 |
| 0110 | 416 Hz | AN5130, p78 |
| 0111 | 833 Hz | AN5130, p78 |
| 1000 | 1.66 kHz | AN5130, p78 |
| 1001 | 3.33 kHz | AN5130, p78 |
| 1010 | 6.66 kHz | AN5130, p78 |

### Decimation Factors (DEC_FIFO_x[2:0])

| Value | Factor | Source |
|-------|--------|--------|
| 000 | Sensor not in FIFO | AN5130, p76 |
| 001 | No decimation | AN5130, p76 |
| 010 | ÷2 | AN5130, p76 |
| 011 | ÷3 | AN5130, p76 |
| 100 | ÷4 | AN5130, p76 |
| 101 | ÷8 | AN5130, p76 |
| 110 | ÷16 | AN5130, p76 |
| 111 | ÷32 | AN5130, p76 |

Applied identically to all four decimation fields: `DEC_FIFO_GYRO`, `DEC_FIFO_XL` (FIFO_CTRL3), `DEC_DS3_FIFO`, `DEC_DS4_FIFO` (FIFO_CTRL4).

### FIFO Trigger Sources

Configured via `DATA_VALID_SEL_FIFO` (MASTER_CONFIG) and `TIMER_PEDO_FIFO_DRDY` (FIFO_CTRL2):

| DATA_VALID_SEL_FIFO | TIMER_PEDO_FIFO_DRDY | Trigger Source | Source |
|---------------------|---------------------|----------------|--------|
| 0 | 0 | Accel/gyro data-ready (limited by ODR_FIFO) | AN5130, p87 |
| 0 | 1 | Step detected | AN5130, p87 |
| 1 | X | Sensor hub end-of-operation | AN5130, p87 |

### FIFO Status Registers

- **DIFF_FIFO[10:0]** (FIFO_STATUS1 + FIFO_STATUS2): Number of 16-bit unread words. Set to 0 on overrun. (AN5130, p79)
- **WaterM** (FIFO_STATUS2 b7): Set when DIFF_FIFO ≥ FTH[10:0] threshold. (AN5130, p79)
- **OVER_RUN** (FIFO_STATUS2 b6): FIFO is full and at least one sample was overwritten. (AN5130, p79)
- **FIFO_FULL_SMART** (FIFO_STATUS2 b5): Next write will make FIFO full. (AN5130, p79)
- **FIFO_EMPTY** (FIFO_STATUS2 b4): FIFO is empty. (AN5130, p79)

### FIFO Pattern (FIFO_PATTERN[9:0])

Data stored without tags. FIFO_STATUS3/4 contain pattern index identifying which axis/sensor will be read next. Pattern repeats based on data set count and decimation ratios. (AN5130, p88)

Example: Gyro + Accel at same ODR → pattern repeats every 6 samples: Gx, Gy, Gz, XLx, XLy, XLz. (AN5130, p89)

### FIFO Reading Procedure

1. Read FIFO_STATUS1/2 to check DIFF_FIFO[10:0]
2. Read FIFO_STATUS3/4 to check FIFO_PATTERN[9:0]
3. Read FIFO_DATA_OUT_L/H to retrieve oldest 16-bit sample
4. Repeat until FIFO_EMPTY = 1

Do not read FIFO_DATA_OUT when FIFO is empty. Read faster than `1 × ODR × 3 × (number of enabled data sets)` to avoid data loss in Continuous mode. (AN5130, p88)

### FIFO Configuration Change Procedure

When changing ODR or FIFO settings while FIFO is active (AN5130, p88):
1. Read all stored data until FIFO is empty
2. Set Bypass mode (FIFO_MODE = 000)
3. Change ODR/decimation/FIFO settings
4. Set desired FIFO mode

### Special FIFO Features

- **STOP_ON_FTH** (FIFO_CTRL4 b7): Limits FIFO depth to FTH threshold. (AN5130, p77)
- **ONLY_HIGH_DATA** (FIFO_CTRL4 b6): Store only MSB (8 bits) of accel+gyro per axis. Data format: AccX_H, GyroX_H, AccY_H, GyroY_H, AccZ_H, GyroZ_H in 1st data set. (AN5130, p93)
- **Step counter + timestamp in FIFO**: Enable via `TIMER_PEDO_FIFO_EN` in FIFO_CTRL2. 6-byte format: TIMESTAMP[15:8], TIMESTAMP[23:16], unused, TIMESTAMP[7:0], STEPS[7:0], STEPS[15:8]. (AN5130, p94)
- **Temperature in FIFO**: Enable via `FIFO_TEMP_EN` in FIFO_CTRL2 (with TIMER_PEDO_FIFO_EN = 0). 6-byte format: unused, unused, TEMP[7:0], TEMP[15:8], unused, unused. (AN5130, p95)

### Continuous-to-FIFO and Bypass-to-Continuous Trigger Events

Both modes switch on the edge of an interrupt signal from one of: significant motion, tilt, step detection, single tap, double tap, free-fall, wake-up, or 6D. Specific INT1/INT2 bits must be set for each event as documented. (AN5130, p84–85)

---

## 14. Interrupt System

### Interrupt Pins

Two programmable interrupt pins (INT1 = pin 4, INT2 = pin 9). Both are push-pull by default; open-drain selectable via `PP_OD` bit in CTRL3_C. Active-high by default; active-low via `H_LACTIVE` bit in CTRL3_C. (datasheet_LSM6DS3TR-C.pdf, p52)

`INT2_on_INT1` bit (CTRL4_C b5): When set to 1, all enabled INT2 signals are OR'd onto INT1 pin. (AN5130, p35)

### INT1 Routing (INT1_CTRL, 0Dh)

| Bit | Signal | Source |
|-----|--------|--------|
| b7 | INT1_STEP_DETECTOR | AN5130, p33 |
| b6 | INT1_SIGN_MOT | AN5130, p33 |
| b5 | INT1_FULL_FLAG | AN5130, p33 |
| b4 | INT1_FIFO_OVR | AN5130, p33 |
| b3 | INT1_FTH | AN5130, p33 |
| b2 | INT1_BOOT | AN5130, p33 |
| b1 | INT1_DRDY_G | AN5130, p33 |
| b0 | INT1_DRDY_XL | AN5130, p33 |

### INT1 Event Routing (MD1_CFG, 5Eh)

| Bit | Signal | Source |
|-----|--------|--------|
| b7 | INT1_INACT_STATE | AN5130, p34 |
| b6 | INT1_SINGLE_TAP | AN5130, p34 |
| b5 | INT1_WU (wake-up) | AN5130, p34 |
| b4 | INT1_FF (free-fall) | AN5130, p34 |
| b3 | INT1_DOUBLE_TAP | AN5130, p34 |
| b2 | INT1_6D | AN5130, p34 |
| b1 | INT1_TILT | AN5130, p34 |
| b0 | INT1_TIMER | AN5130, p34 |

### INT2 Routing (INT2_CTRL, 0Eh)

| Bit | Signal | Source |
|-----|--------|--------|
| b7 | INT2_STEP_DELTA | AN5130, p34 |
| b6 | INT2_STEP_COUNT_OV | AN5130, p34 |
| b5 | INT2_FULL_FLAG | AN5130, p34 |
| b4 | INT2_FIFO_OVR | AN5130, p34 |
| b3 | INT2_FTH | AN5130, p34 |
| b2 | INT2_DRDY_TEMP | AN5130, p34 |
| b1 | INT2_DRDY_G | AN5130, p34 |
| b0 | INT2_DRDY_XL | AN5130, p34 |

### INT2 Event Routing (MD2_CFG, 5Fh)

| Bit | Signal | Source |
|-----|--------|--------|
| b7 | INT2_INACT_STATE | AN5130, p35 |
| b6 | INT2_SINGLE_TAP | AN5130, p35 |
| b5 | INT2_WU | AN5130, p35 |
| b4 | INT2_FF | AN5130, p35 |
| b3 | INT2_DOUBLE_TAP | AN5130, p35 |
| b2 | INT2_6D | AN5130, p35 |
| b1 | INT2_TILT | AN5130, p35 |
| b0 | INT2_IRON | AN5130, p35 |

### Latch Mode

`LIR` bit in TAP_CFG (58h): When set to 1, interrupt signals are latched until the corresponding source register is read. When 0, interrupts are pulsed (~75 μs for embedded functions, 1/ODR for basic events). Latch only takes effect when the interrupt is driven to a pin. (AN5130, p35–36)

### DRDY Signal Configuration

`DRDY_PULSED` bit in DRDY_PULSE_CFG_G (0Bh): When set to 1, data-ready signals are pulsed (~75 μs). When 0, data-ready is latched until output registers are read. (AN5130, p24)

`DRDY_MASK` bit in CTRL4_C: When set to 1, masks DRDY on INT pin until filter settling period completes (invalid samples tagged as 7FFFh/7FFEh/7FFDh). (AN5130, p24)

### Basic Interrupts Enable

`INTERRUPTS_ENABLE` bit in TAP_CFG (58h) must be set to 1 to enable basic interrupts (6D/4D, free-fall, wake-up, tap, inactivity). (AN5130, p35)

---

## 15. Event Detection Features

### Free-Fall Detection

Detects when acceleration on all axes approaches zero simultaneously. (AN5130, p35)

**Configuration registers:**
- `FF_THS[2:0]` in FREE_FALL (5Dh): Threshold (156–500 mg in 8 steps)
- `FF_DUR[5:0]` in FREE_FALL (5Dh, bits 7:3) + WAKE_UP_DUR (5Ch, bit 7): Duration in N/ODR_XL

| FF_THS[2:0] | Threshold (mg) | Source |
|-------------|---------------|--------|
| 000 | 156 | AN5130, p36 |
| 001 | 219 | AN5130, p36 |
| 010 | 250 | AN5130, p36 |
| 011 | 312 | AN5130, p36 |
| 100 | 344 | AN5130, p36 |
| 101 | 406 | AN5130, p36 |
| 110 | 469 | AN5130, p36 |
| 111 | 500 | AN5130, p36 |

**Example sequence** (AN5130, p36):
1. `CTRL1_XL = 60h` (416 Hz, ±2 g)
2. `TAP_CFG = 81h` (enable interrupts + latch)
3. `WAKE_UP_DUR = 00h` (FF_DUR5 = 0)
4. `FREE_FALL = 33h` (threshold 312 mg, duration 6 samples = ~15 ms)
5. `MD1_CFG = 10h` (FF interrupt on INT1)

Status: `FF_IA` bit in WAKE_UP_SRC (1Bh). (AN5130, p35)

### Wake-Up Detection

Detects when filtered acceleration exceeds threshold. Uses slope filter (default) or HP digital filter selected by `SLOPE_FDS` in TAP_CFG. (AN5130, p36)

- `WK_THS[5:0]` in WAKE_UP_THS (5Bh): Threshold, 1 LSB = FS_XL / 2^6
- `WAKE_DUR[1:0]` in WAKE_UP_DUR (5Ch): Duration, 1 LSB = 1/ODR_XL

Status: `WU_IA`, `X_WU`, `Y_WU`, `Z_WU` bits in WAKE_UP_SRC (1Bh). (AN5130, p37)

**Spurious interrupt at startup:** First slope filter output = (1g − 0)/2 = 500 mg, may trigger false wake-up. Solutions: (a) ignore first interrupt, (b) delay driving interrupt to pin, (c) initially use higher ODR then switch. (AN5130, p38)

### 6D/4D Orientation Detection

Detects device orientation in 6 positions (or 4 for portrait/landscape). (AN5130, p38)

- `SIXD_THS[1:0]` in TAP_THS_6D (59h): Threshold angle

| SIXD_THS[1:0] | Threshold (degrees) | Source |
|---------------|---------------------|--------|
| 00 | 80 | AN5130, p39 |
| 01 | 70 | AN5130, p39 |
| 10 | 60 | AN5130, p39 |
| 11 | 50 | AN5130, p39 |

- `D4D_EN` bit in TAP_THS_6D: When 1, Z-axis detection disabled (4D mode). (AN5130, p41)
- `LOW_PASS_ON_6D` in CTRL8_XL: Enable LPF2 for 6D. (AN5130, p39)

Status: `D6D_IA`, `ZH`, `ZL`, `YH`, `YL`, `XH`, `XL` in D6D_SRC (1Dh). (AN5130, p38)

### Single-Tap and Double-Tap Recognition

Uses slope between consecutive samples: `slope(tn) = [acc(tn) - acc(tn-1)] / 2`. Recommended ODR: 416 Hz or 833 Hz. (AN5130, p41)

**Configuration:**
- `TAP_X_EN`, `TAP_Y_EN`, `TAP_Z_EN` in TAP_CFG (58h): Enable per-axis
- `TAP_THS[4:0]` in TAP_THS_6D (59h): Threshold, 1 LSB = FS_XL / 2^5
- `SHOCK[1:0]` in INT_DUR2 (5Ah): Max duration of over-threshold (default 4/ODR, else 8/ODR per LSB)
- `QUIET[1:0]` in INT_DUR2 (5Ah): Quiet time after first tap (default 2/ODR, else 4/ODR per LSB)
- `DUR[3:0]` in INT_DUR2 (5Ah): Max time between two taps for double-tap (default 16/ODR, else 32/ODR per LSB)
- `SINGLE_DOUBLE_TAP` in WAKE_UP_THS (5Bh): 0 = single-tap only, 1 = both enabled

**Note:** Tap threshold (mg) must be higher than wake-up threshold (mg). (AN5130, p44)

Status: `TAP_IA`, `SINGLE_TAP`, `DOUBLE_TAP`, `TAP_SIGN`, `X_TAP`, `Y_TAP`, `Z_TAP` in TAP_SRC (1Ch). (AN5130, p45)

**Single-tap example** (AN5130, p46):
1. `CTRL1_XL = 60h` (416 Hz, ±2 g)
2. `TAP_CFG = 8Eh` (enable interrupts + X/Y/Z tap)
3. `TAP_THS_6D = 89h` (tap threshold = 562.5 mg)
4. `INT_DUR2 = 06h` (Quiet=01, Shock=10)
5. `WAKE_UP_THS = 00h` (single-tap only)
6. `MD1_CFG = 40h` (single-tap on INT1)

**Double-tap example** (AN5130, p46):
1. `CTRL1_XL = 60h`
2. `TAP_CFG = 8Eh`
3. `TAP_THS_6D = 8Ch` (tap threshold = 750 mg)
4. `INT_DUR2 = 7Fh` (DUR=0111, QUIET=11, SHOCK=11)
5. `WAKE_UP_THS = 80h` (single+double-tap enabled)
6. `MD1_CFG = 08h` (double-tap on INT1)

### Activity/Inactivity Detection

Automatically reduces accelerometer ODR to 12.5 Hz (low-power) during inactivity, with configurable gyroscope behavior. (AN5130, p47)

**INACT_EN[1:0] in TAP_CFG:**

| INACT_EN | Accelerometer | Gyroscope | Source |
|----------|---------------|-----------|--------|
| 00 | Disabled | Disabled | AN5130, p47 |
| 01 | ODR → 12.5 Hz LP | Unchanged | AN5130, p47 |
| 10 | ODR → 12.5 Hz LP | Sleep mode | AN5130, p47 |
| 11 | ODR → 12.5 Hz LP | Power-down | AN5130, p47 |

- `WK_THS[5:0]` in WAKE_UP_THS: Shared threshold for activity/inactivity detection
- `SLEEP_DUR[3:0]` in WAKE_UP_DUR: Inactivity duration, 1 LSB = 512/ODR_XL

Status: `SLEEP_STATE_IA` in WAKE_UP_SRC (1Bh). (AN5130, p48)

Max allowed accel ODR for activity/inactivity: 3.3 kHz. (AN5130, p47)

**Example sequence** (AN5130, p48):
1. `CTRL1_XL = 50h` (208 Hz, ±2 g)
2. `CTRL2_G = 40h` (104 Hz, ±250 dps)
3. `WAKE_UP_DUR = 02h` (inactivity: 2 × 512/208 = ~4.92 s)
4. `WAKE_UP_THS = 02h` (62.5 mg)
5. `TAP_CFG = E0h` (enable interrupts, inact = gyro power-down, slope filter)
6. `MD1_CFG = 80h` (activity/inactivity on INT1)

---

## 16. Embedded Functions

All embedded functions work at 26 Hz and require accelerometer ODR ≥ 26 Hz. Enable via `FUNC_EN` bit in CTRL10_C (19h). (AN5130, p50)

### Pedometer (Step Detector + Step Counter)

Enable: `FUNC_EN = 1` and `PEDO_EN = 1` in CTRL10_C. (AN5130, p50)

- **Step counter**: 16-bit unsigned in STEP_COUNTER_H/L (4Ch/4Bh). Not reset on power-down. Reset by `PEDO_RST_STEP = 1` in CTRL10_C.
- **Step detector**: Generates interrupt per detected step
- **Debounce**: 7 consecutive steps (default) required before first interrupt. Configurable via `DEB_STEP[2:0]` in PEDO_DEB_REG (default = 6 steps). Debounce restarts after ~1 s inactivity (configurable via `DEB_TIME[4:0]`, 1 LSB = 80 ms, default = 13 = 1040 ms). (AN5130, p50)
- **Full-scale**: Default ±2 g. Set `PEDO_FS = 1` in CONFIG_PEDO_THS_MIN for ±4 g (with accel FS ≥ ±4 g). (AN5130, p51)
- **Minimum threshold**: `ths_min[5:0]` in CONFIG_PEDO_THS_MIN. 1 LSB = 16 mg (PEDO_FS=0) or 32 mg (PEDO_FS=1). (AN5130, p51)
- **Step timestamp**: STEP_TIMESTAMP_H/L captures TIMESTAMP_REG2/1 at each step. Resolution depends on TIMER_HR. (AN5130, p52)

Interrupt: `INT1_STEP_DETECTOR` in INT1_CTRL, or `STEP_DETECTED` in FUNC_SRC1. Step delta interrupt: `INT2_STEP_DELTA` (triggered every STEP_COUNT_DELTA × 1.6384 s). Step overflow: `INT2_STEP_COUNT_OV` (counter wraps at 2^16 and auto-resets). (AN5130, p51–52)

**Example** (AN5130, p52):
1. `CTRL1_XL = 20h` (26 Hz, ±2 g)
2. `CTRL10_C = 14h` (enable embedded functions + pedometer)
3. `INT1_CTRL = 80h` (step detector on INT1)

### Significant Motion

Detects change of user location based on step count threshold. Enable: `FUNC_EN = 1` and `SIGN_MOTION_EN = 1` in CTRL10_C. (AN5130, p52)

Threshold: `SM_THS[7:0]` in SM_THS register (Bank A, 13h). Default = 6 steps. Must be ≥ pedometer debounce threshold for proper operation. (AN5130, p53)

Interrupt: `INT1_SIGN_MOT` in INT1_CTRL, or `SIGN_MOTION_IA` in FUNC_SRC1. (AN5130, p53)

**Example** (AN5130, p53):
1. `FUNC_CFG_ACCESS = 80h` (Bank A)
2. `SM_THS = 08h` (threshold = 8 steps)
3. `FUNC_CFG_ACCESS = 00h`
4. `CTRL1_XL = 20h` (26 Hz, ±2 g)
5. `CTRL10_C = 05h` (enable functions + significant motion)
6. `INT1_CTRL = 40h` (significant motion on INT1)

### Relative Tilt

Detects when device tilts > 35° from start position. After first tilt interrupt (requires 2 s sustained tilt), subsequent detections are instantaneous. Enable: `FUNC_EN = 1` and `TILT_EN = 1` in CTRL10_C. (AN5130, p53–54)

Interrupt: `INT1_TILT` in MD1_CFG or `INT2_TILT` in MD2_CFG, or `TILT_IA` in FUNC_SRC1. (AN5130, p54)

**Example** (AN5130, p54):
1. `CTRL1_XL = 20h` (26 Hz, ±2 g)
2. `CTRL10_C = 0Ch` (enable functions + tilt)
3. `MD1_CFG = 02h` (tilt on INT1)

### Absolute Wrist Tilt (AWT)

Detects when angle between a selectable semi-axis and horizontal plane exceeds threshold for a minimum latency period. Enable: `FUNC_EN = 1` (implied by bit setting) and `WRIST_TILT_EN = 1` in CTRL10_C. (AN5130, p55)

Default configuration: X-positive axis, threshold 500 mg (30°), latency 600 ms. (AN5130, p55)

**Configurable parameters (Bank B registers):**
- `A_WRIST_TILT_LAT (50h)`: Latency, 1 LSB = 40 ms
- `A_WRIST_TILT_THS (54h)`: Threshold, 1 LSB = 15.625 mg, must be < 64. Angle = 180/π × asin(THS/64)
- `A_WRIST_TILT_Mask (59h)`: Axis selection (Xpos/Xneg/Ypos/Yneg/Zpos/Zneg), OR combination

Interrupt: `INT2_WRIST_TILT` in DRDY_PULSE_CFG (0Bh), or `WRIST_TILT_IA` in FUNC_SRC2. (AN5130, p55)

**Complete AWT configuration procedure** (required every time accel exits power-down; AN5130, p56):
1. `CTRL1_XL = 20h` (26 Hz, ±2 g)
2. `CTRL10_C = 04h` (enable embedded functions)
3. Wait 50 ms
4. `CTRL10_C = 00h` (disable embedded functions)
5. `FUNC_CFG_ACCESS = A0h` (Bank B)
6. Write A_WRIST_TILT_LAT, A_WRIST_TILT_THS, A_WRIST_TILT_Mask
7. `FUNC_CFG_ACCESS = 00h`
8. `CTRL10_C = 84h` (enable functions + AWT)
9. `DRDY_PULSE_CFG = 01h` (AWT on INT2)

### Timestamp

24-bit counter. Enable: `TIMER_EN = 1` in CTRL10_C. Requires at least one sensor active. (AN5130, p57)

Resolution: `TIMER_HR` bit in WAKE_UP_DUR (5Ch):
- TIMER_HR = 0: 1 LSB = 6.4 ms (low-resolution)
- TIMER_HR = 1: 1 LSB = 25 μs (high-resolution)

Wraps at 0xFFFFFF (16,777,215). Reset by writing 0xAA to TIMESTAMP_REG2. Overflow interrupt ~1.638 s before saturation, routed via `INT1_TIMER` in MD1_CFG. (AN5130, p57)

**Note:** Set timestamp resolution before enabling. When switching from low to high resolution, reset timer by writing AAh to TIMESTAMP_REG2. (AN5130, p57)

---

## 17. Sensor Hub (Mode 2)

### Overview

I2C master interface on SDx (MSDA, pin 2) and SCx (MSCL, pin 3). Up to 4 external sensors. Clock: 116.3 kHz. Both accelerometer and gyroscope must not simultaneously be in power-down for sensor hub to function. (AN5130, p58; datasheet_LSM6DS3TR-C.pdf, p28)

Enable: `FUNC_EN = 1` in CTRL10_C, then `MASTER_ON = 1` in MASTER_CONFIG (1Ah). (AN5130, p59)

### MASTER_CONFIG Register (1Ah)

| Bit | Name | Description | Source |
|-----|------|-------------|--------|
| b7 | DRDY_ON_INT1 | Sensor hub data-ready on INT1 (pulsed 150 μs if DRDY_PULSED=1) | AN5130, p59 |
| b6 | DATA_VALID_SEL_FIFO | 0 = accel/gyro trigger FIFO, 1 = sensor hub trigger FIFO | AN5130, p59 |
| b4 | START_CONFIG | 0 = accel DRDY trigger (up to 104 Hz), 1 = INT2 external trigger | AN5130, p59 |
| b3 | PULL_UP_EN | Internal pull-up on SDx/SCx | AN5130, p60 |
| b2 | PASS_THROUGH_MODE | Short-circuit main & aux I2C for direct external sensor access | AN5130, p66 |
| b1 | IRON_EN | Enable hard-iron correction | AN5130, p69 |
| b0 | MASTER_ON | Enable I2C master | AN5130, p60 |

### Slave Configuration

Each slave (0–3) has 3 registers (Bank A):
- `SLVx_ADD`: 7-bit slave address + r/w bit (slave 0: rw_0; slaves 1–3: read enable)
- `SLVx_SUBADD`: Register address on external sensor to read/write
- `SLAVEx_CONFIG`: Decimation rate, number of registers to read (numop[2:0])

Special fields in SLAVE0_CONFIG: `Aux_sens_on[1:0]` (number of external sensors: 00=1, 01=2, 10=3, 11=4), `Src_mode` (source mode conditioned read). (AN5130, p61)

`write_once` bit in SLAVE1_CONFIG: Limits write operations on slave 0 to one occurrence. Requires Aux_sens_on ≠ 00. (AN5130, p62)

Data read from external sensors stored consecutively in SENSORHUB1_REG through SENSORHUB18_REG. SENSORHUB1–6 can be stored in FIFO as 3rd data set; SENSORHUB7–12 as 4th data set. (AN5130, p65)

### NACK Detection

`SLAVEx_NACK` bits in FUNC_SRC2 (54h) indicate communication failure per slave. (AN5130, p60)

### Pass-Through Feature

`PASS_THROUGH_MODE = 1`: Short-circuits main I2C with auxiliary I2C for direct MCU access to external sensors. Recommended for initial external sensor configuration. (AN5130, p66)

Enable procedure (when sensor hub is active; AN5130, p67):
1. `START_CONFIG = 1` (disable internal trigger)
2. Wait ≥ 5 ms
3. `MASTER_ON = 0`
4. `START_CONFIG = 0`
5. `PULL_UP_EN = 0`
6. `PASS_THROUGH_MODE = 1`

Disable: Wait for MCU I2C to finish, then `PASS_THROUGH_MODE = 0`. (AN5130, p67)

**Limitations:** When using internal trigger with pass-through, INT2 must be tied to GND. When using external trigger (START_CONFIG=1), pass-through cannot be used. (AN5130, p66)

### Hard-Iron / Soft-Iron Correction

For external magnetometer connected as slave 0 (Slave0_numop = 6). (AN5130, p69)

| SOFT_EN (CTRL9_XL) | IRON_EN (MASTER_CONFIG) | Mode | Source |
|--------------------|------------------------|------|--------|
| 0 | 0 | No correction | AN5130, p69 |
| 0 | 1 | Hard-iron only | AN5130, p69 |
| 1 | 1 | Hard-iron + soft-iron | AN5130, p69 |

- **Hard-iron offset**: MAG_OFFX/Y/Z_L/H (Bank A). 16-bit two's complement. Sensitivity = external magnetometer LSB/Gauss. (AN5130, p69)
- **Soft-iron matrix**: MAG_SI_XX through MAG_SI_ZZ (Bank A). 8-bit sign-magnitude, 1 LSB = 1/8. Initialize to identity (08h on diagonals) when soft-iron enabled. (AN5130, p70–71)

**Data outputs:**
- Raw magnetometer: OUT_MAG_RAW (66h–6Bh): Uncorrected data from sensor hub
- Calibrated: SENSORHUB1–6_REG (2Eh–33h): SI × (raw − HI)
- Uncalibrated: SENSORHUB13–18_REG (4Dh–52h): SI × (raw − HI) + HI (soft-iron only)

(AN5130, p70)

---

## 18. Temperature Sensor and Self-Test

### Temperature Sensor

Active whenever at least one sensor (accel or gyro) is not in power-down. Output: 16-bit two's complement in OUT_TEMP_L/H (20h/21h). (AN5130, p96; datasheet_LSM6DS3TR-C.pdf, p25)

- Sensitivity: 256 LSB/°C
- Zero level: 0 LSB at 25 °C
- Temperature [°C] = OUT_TEMP / 256 + 25
- Refresh rate: 52 Hz max (12.5 Hz if accel LP 12.5 Hz & gyro off, 26 Hz if accel 26 Hz LP & gyro off)
- Data-ready: `TDA` bit in STATUS_REG (1Eh), routable to INT2 via `INT2_DRDY_TEMP`
- Can be stored in FIFO as 4th data set

(AN5130, p96)

### Example: Temperature Data

| Condition | OUT_TEMP_H (21h) | OUT_TEMP_L (20h) | Source |
|-----------|-----------------|-----------------|--------|
| 0 °C | E7h | 00h | AN5130, p96 |
| 25 °C | 00h | 00h | AN5130, p96 |
| 50 °C | 19h | 00h | AN5130, p96 |

### Accelerometer Self-Test

Enabled by `ST_XL[1:0]` in CTRL5_C (14h): 00 = off, 01 = positive, 10 = negative. (AN5130, p99)

Pass criteria: |output(ST enabled) − output(ST disabled)| must be 90–1700 mg (FS independent, measured at 1 LSB = 0.061 mg at ±2 g). (datasheet_LSM6DS3TR-C.pdf, p22)

### Gyroscope Self-Test

Enabled by `ST_G[1:0]` in CTRL5_C (14h): 00 = off, 01 = positive, 11 = negative. (AN5130, p97)

Pass criteria:
- At ±250 dps: 20–80 dps (measured at 1 LSB = 70 mdps at ±2000 dps)
- At ±2000 dps: 150–700 dps

(datasheet_LSM6DS3TR-C.pdf, p22)

### Boot and Reset

- **Boot time**: 15 ms after power-on (registers not accessible during boot). (AN5130, p49)
- **Reboot**: Set `BOOT = 1` in CTRL3_C. Procedure: gyro to power-down, accel to HP mode, set BOOT, wait 15 ms. (AN5130, p49)
- **SW Reset**: Set `SW_RESET = 1` in CTRL3_C. Takes ~50 μs. Resets all control registers to defaults. Procedure: gyro to power-down, accel to HP mode, set SW_RESET, wait until SW_RESET returns to 0. (AN5130, p49)
- **Do not** set BOOT and SW_RESET simultaneously. Execute sequentially. (AN5130, p49)

---

## 19. Application Design Tips

### Calibration (DT0105)

**1-point tumble calibration**: Place sensor with one axis aligned to gravity (e.g. Z = +1 g). Offsets: Xofs = AccX, Yofs = AccY, Zofs = AccZ − 1. Assumes sensitivity = 1 and no cross-axis error. (DT0105, p2)

**3-point tumble calibration**: Three positions with each axis aligned to gravity in turn. Computes both offset and gain per axis. Offsets by averaging non-stimulated readings; gains from stimulated readings minus offset. (DT0105, p2–3)

For more advanced calibration (cross-axis, 6-point), see referenced DT0053. (DT0105, p4)

### Tilt Measurement (DT0058)

Compute orientation from accelerometer data:
1. Roll: `Phi = Atan2(Gy, Gz)` — for stability near ±90° pitch, use `Gz + Gx*alpha`
2. Pitch: `Theta = Atan(-Gx / Gz2)` where `Gz2 = Gy*sin(Phi) + Gz*cos(Phi)`
3. Yaw (eCompass, requires magnetometer): `Psi = Atan2(By2, Bx3)` using tilt-compensated mag data

Generic tilt from horizontal: `Tilt = atan2(sqrt(Gx² + Gy²), Gz)` (DT0058, p4)

**Gimbal lock**: At Theta = ±90°, roll and yaw describe same axis (one DOF lost). Sum Phi+Psi remains stable. (DT0058, p2)

**High-g constraint**: Tilt from accel only valid when `√(Gx² + Gy² + Gz²) ≈ 1g`. Use gyroscope during high-g. (DT0058, p2)

### Gyroscope Tilt Update (DT0060)

Update orientation by integrating gyroscope angular rate:

**Quaternion method** (preferred, avoids gimbal-lock):
- Quaternion derivative: `Q' = 0.5 × Q ⊗ [0, Wx, Wy, Wz]`
- Update: `Q(t+Ts) = Q(t) + Q'×Ts`, then normalize
- More accurate: quaternion exponentiation `Q(t+Ts) = Q(t) × exp([0,Wx,Wy,Wz]×Ts/2)`

**Euler angle method** (has singularities at Theta = ±90°):
- `Phi' = Wx + Wy*sin(Phi)*tan(Theta) + Wz*cos(Phi)*tan(Theta)`
- `Theta' = Wy*cos(Phi) − Wz*sin(Phi)`
- `Psi' = Wy*sin(Phi)/cos(Theta) + Wz*cos(Phi)/cos(Theta)`

**Critical notes:**
- Subtract gyroscope bias before integration (estimate by averaging when stationary)
- Ts accuracy critical — use highest available ODR (up to 6.66 kHz)
- Mix gyro-derived angles with accel/mag angles: use weighted average with alpha (0 < alpha < 1)

(DT0060, p1–4)

### Dead Reckoning (DT0106)

Compute residual linear acceleration by subtracting rotated gravity from measured acceleration:
- Body frame: `linacc_body = -(acc − rotM × [0;0;1])`
- World frame: `linacc_world = -(rotM' × acc − [0;0;1])`

Dead-reckoning by double integration: velocity = ∫(linacc), position = ∫(velocity). (DT0106, p4)

**Drift control:**
- Leaky integrator: `vel(k) = c*acc(k)*Ts + alpha*vel(k-1)` with alpha = 0.9–0.95
- Zero-Velocity-Update (ZVU): Reset velocity to 0 when accel modulus ≈ 1 g and gyro ≈ 0

(DT0106, p4–5)

### Noise Analysis (DT0064)

Allan variance characterizes MEMS sensor noise:
- **Angular Random Walk (ARW)**: `ARW [deg/√s] = noise density [dps/√Hz]`; final angle error RMS = ARW × √(time). Example: ARW 1°/√s × √(1000 s) = 31.6° RMS. (DT0064, p5)
- Allan deviation slope −1/2 → white frequency noise (gyro ARW)
- Allan deviation slope 0 → flicker (bias instability)
- Allan deviation slope +1/2 → random walk

Overlapping Allan variance (OAVAR) recommended for better confidence, up to m = 10% of data length. (DT0064, p2)

---

## 20. Startup, Boot, and Initialization Sequences

### Power-On Sequence

1. Apply VDD and VDDIO (100 nF decoupling on each)
2. Wait 15 ms for boot to complete (both sensors automatically in power-down after boot)
3. Registers become accessible

(AN5130, p49; datasheet_LSM6DS3TR-C.pdf, p24)

### Basic Accelerometer Startup (Mode 1)

```
1. Write 60h to CTRL1_XL     // ODR = 416 Hz, FS = ±2 g, HP mode
2. Write 01h to INT1_CTRL    // Accel DRDY on INT1
```
(AN5130, p23)

### Basic Accelerometer + Gyroscope Startup

```
1. Write 60h to CTRL1_XL     // Accel: 416 Hz, ±2 g
2. Write 60h to CTRL2_G      // Gyro: 416 Hz, ±250 dps
3. Write 44h to CTRL3_C      // BDU = 1, IF_INC = 1 (default)
```

### Data Reading Using Status Register

```
1. Read STATUS_REG (1Eh)
2. If XLDA = 1, read OUTX_L_XL through OUTZ_H_XL (28h–2Dh)
3. If GDA = 1, read OUTX_L_G through OUTZ_H_G (22h–27h)
4. If TDA = 1, read OUT_TEMP_L/H (20h–21h)
```
(AN5130, p23)

### Data Reading Using DRDY Interrupt

```
1. Configure sensor ODR/FS in CTRL1_XL / CTRL2_G
2. Write 44h to CTRL3_C       // BDU = 1, IF_INC = 1
3. Write 03h to INT1_CTRL     // Accel+Gyro DRDY on INT1
4. On INT1 rising edge, read output registers
```
(AN5130, p24)

### Output Data Interpretation

**Accelerometer** (16-bit two's complement):
- `acceleration [g] = raw_value × sensitivity`
- Example at ±2 g: raw 0x4000 (16384) → 16384 × 0.061 mg = ~999.4 mg ≈ 1 g

**Gyroscope** (16-bit two's complement):
- `angular_rate [dps] = raw_value × sensitivity`
- Example at ±250 dps: raw 0x7080 (28800) → 28800 × 8.75 mdps = 252 dps

**Temperature** (16-bit two's complement):
- `temperature [°C] = raw_value / 256 + 25`

(AN5130, p25–26)

### BDU (Block Data Update) Notes

Set `BDU = 1` in CTRL3_C to prevent output registers from being updated mid-read. Without BDU, the MSB and LSB of a 16-bit value may come from different measurement samples. (AN5130, p25)

### FIFO Basic Setup (Continuous Mode, Accel + Gyro)

```
1. Write 60h to CTRL1_XL     // Accel: 416 Hz, ±2 g
2. Write 60h to CTRL2_G      // Gyro: 416 Hz, ±250 dps
3. Write 44h to CTRL3_C      // BDU = 1, IF_INC = 1
4. Write 09h to FIFO_CTRL3   // No decimation for both gyro and accel
5. Write 66h to FIFO_CTRL5   // ODR_FIFO = 416 Hz, Continuous mode
```
(AN5130, p83)

### Sensor Hub Basic Setup (LIS2MDL Magnetometer, Internal Trigger)

```
 1. Write 80h to FUNC_CFG_ACCESS   // Bank A
 2. Write 3Ch to SLV0_ADD          // LIS2MDL addr 0x1E, write mode
 3. Write 60h to SLV0_SUBADD       // LIS2MDL config register
 4. Write 8Ch to DATAWRITE_SRC_MODE_SUB_SLV0  // Continuous, 100 Hz, temp comp
 5. Write 10h to SLAVE0_CONFIG     // Aux_sens_on ≠ 00
 6. Write 20h to SLAVE1_CONFIG     // write_once
 7. Write 00h to FUNC_CFG_ACCESS   // Exit Bank A
 8. Write 04h to CTRL10_C          // Enable embedded functions
 9. Write 09h to MASTER_CONFIG     // Pull-up, accel DRDY trigger, master on
10. Write 80h to CTRL1_XL          // Accel on (for trigger)
11. Poll FUNC_SRC1 until SENSORHUB_END_OP = 1
    [Then reconfigure SLV0 for read mode with numop=6...]
```
(AN5130, p68)

---

## 21. Addendum — I2C Slave Timing Parameters

The following detailed I2C timing values are required for bus design and signal integrity analysis. (datasheet_LSM6DS3TR-C.pdf, p27, Table 7)

| Symbol | Parameter | Standard Mode Min | Standard Mode Max | Fast Mode Min | Fast Mode Max | Unit |
|--------|-----------|-------------------|-------------------|---------------|---------------|------|
| f(SCL) | SCL clock frequency | 0 | 100 | 0 | 400 | kHz |
| tw(SCLL) | SCL clock low time | 4.7 | — | 1.3 | — | μs |
| tw(SCLH) | SCL clock high time | 4.0 | — | 0.6 | — | μs |
| tsu(SDA) | SDA setup time | 250 | — | 100 | — | ns |
| th(SDA) | SDA data hold time | 0 | 3.45 | 0 | 0.9 | μs |
| th(ST) | START condition hold time | 4 | — | 0.6 | — | μs |
| tsu(SR) | Repeated START setup time | 4.7 | — | 0.6 | — | μs |
| tsu(SP) | STOP condition setup time | 4 | — | 0.6 | — | μs |
| tw(SP:SR) | Bus free time (STOP→START) | 4.7 | — | 1.3 | — | μs |

**Note:** Data based on standard I2C protocol requirement, not tested in production. Measurement points at 0.2×VDD_IO and 0.8×VDD_IO. (datasheet_LSM6DS3TR-C.pdf, p27)

---

## 22. Addendum — SPI Timing Parameters

Full SPI slave timing values for bus design. Values guaranteed at 10 MHz for both 3-wire and 4-wire SPI, based on characterization (not production-tested). (datasheet_LSM6DS3TR-C.pdf, p26, Table 6)

| Symbol | Parameter | Min | Max | Unit |
|--------|-----------|-----|-----|------|
| tc(SPC) | SPI clock cycle | 100 | — | ns |
| fc(SPC) | SPI clock frequency | — | 10 | MHz |
| tsu(CS) | CS setup time | 5 | — | ns |
| th(CS) | CS hold time | 20 | — | ns |
| tsu(SI) | SDI input setup time | 5 | — | ns |
| th(SI) | SDI input hold time | 15 | — | ns |
| tv(SO) | SDO valid output time | — | 50 | ns |
| th(SO) | SDO output hold time | 5 | — | ns |
| tdis(SO) | SDO output disable time | — | 50 | ns |

**Note:** Measurement points at 0.2×VDD_IO and 0.8×VDD_IO for both input and output ports. (datasheet_LSM6DS3TR-C.pdf, p26)

---

## 23. Addendum — I2C Master Timing Parameters

Detailed I2C master timing for sensor hub bus design. The LSM6DS3TR-C master operates at a fixed 116.3 kHz clock in Fast Mode only. (datasheet_LSM6DS3TR-C.pdf, p28, Table 8)

| Symbol | Parameter | I2C Master Value | I2C Fast Mode Min (ref) | Unit |
|--------|-----------|-----------------|------------------------|------|
| f(SCL) | SCL clock frequency | 116.3 | 0 (400 kHz max) | kHz |
| tw(SCLL) | SCL clock low time | 5.86 | 1.3 | μs |
| tw(SCLH) | SCL clock high time | 2.74 | 0.6 | μs |
| — | Data valid time | 3.9 | — | μs |
| — | SDA hold time | ≥0 | 0 | ns |
| — | SDA setup time | ≥100 | 100 | ns |
| tsu(SR) | Repeated START setup time | 1.56 | 0.6 | μs |
| tsu(HD) | Repeated START hold time | 1.56 | 0.6 | μs |
| tsu(SP) | STOP condition setup time | 2.73 | 0.6 | μs |
| tw(SP:SR) | Bus free time (STOP→START) | 21 | 1.3 | μs |

(datasheet_LSM6DS3TR-C.pdf, p28)

---

## 24. Addendum — Register Map Address Corrections and Reserved Ranges

**Address range 43h–48h is RESERVED.** These addresses must not be read/written. The actual step timestamp and step counter register addresses are:

| Addr | Name | Source |
|------|------|--------|
| 43h–48h | RESERVED | datasheet_LSM6DS3TR-C.pdf, p49 (register map) |
| 49h | STEP_TIMESTAMP_L | datasheet_LSM6DS3TR-C.pdf, p83 |
| 4Ah | STEP_TIMESTAMP_H | datasheet_LSM6DS3TR-C.pdf, p83 |
| 4Bh | STEP_COUNTER_L | datasheet_LSM6DS3TR-C.pdf, p83 |
| 4Ch | STEP_COUNTER_H | datasheet_LSM6DS3TR-C.pdf, p84 |

**Note:** This corrects an error in Section 9 above where these registers were listed at 43h–46h instead of 49h–4Ch. The main register map table has been corrected in-place.

---

## 25. Addendum — Accelerometer Turn-On/Off Time Details

### Accelerometer Turn-On/Off Time (LPF2 and HP disabled)

(AN5130, p19, Table 12)

| Starting Mode | Target Mode | Max Turn-On/Off Time |
|---------------|-------------|---------------------|
| Power-Down | Low-Power / Normal | See samples-to-discard table below |
| Power-Down | High-Performance | See samples-to-discard table below |
| Low-Power / Normal | High-Performance | Samples to discard + 1 additional |
| Low-Power / Normal | Low-Power / Normal (ODR change) | See samples-to-discard table below |
| High-Performance | Low-Power / Normal | Samples to discard + 1 additional |
| HP @ ODR ≤ 833 Hz | HP @ ODR ≤ 833 Hz | Samples to discard + 1 additional |
| HP @ ODR ≤ 833 Hz | HP @ ODR > 833 Hz | Samples to discard + 1 additional |
| HP @ ODR > 833 Hz | HP @ ODR ≤ 833 Hz | Samples to discard + 1 additional |
| HP @ ODR > 833 Hz | HP @ ODR > 833 Hz | Discard 5 samples |
| Any active mode | Power-Down | 1 μs |

Settling time is at 99% of final value. (AN5130, p19)

### Accelerometer Samples to Discard (Table 13)

(AN5130, p19–20)

| Target ODR (Hz) | Samples to discard (LPF1_BW_SEL=0, LPF2=0, HP_SLOPE=0) | Samples to discard (LPF1_BW_SEL=1, or HPCF_XL=00 + HP_SLOPE=1) |
|-----------------|--------------------------------------------------------|----------------------------------------------------------------|
| 1.6 (LP) | 0 (first sample correct) | 1 |
| 12.5 (LP) | 0 | 1 |
| 26 (LP) | 0 | 1 |
| 52 (LP) | 0 | 1 |
| 104 (Normal) | 0 | 1 |
| 208 (Normal) | 0 | 1 |
| 12.5 (HP) | 0 | 1 |
| 26 (HP) | 0 | 1 |
| 52 (HP) | 1 | 1 |
| 104 (HP) | 1 | 2 |
| 208 (HP) | 1 | 2 |
| 416 (HP) | 1 | 2 |
| 833 (HP) | 1 | 2 |
| 1666 (HP) | 2 | 2 |
| 3333 (HP) | 3 | 4 |
| 6666 (HP) | 13 | 13 |

---

## 26. Addendum — Gyroscope Turn-On/Off Time Details

### Gyroscope Turn-On/Off Time (HP disabled)

(AN5130, p20, Table 14)

| Starting Mode | Target Mode | Max Turn-On/Off Time |
|---------------|-------------|---------------------|
| Power-Down | Sleep | 70 ms |
| Power-Down | Low-Power / Normal | 70 ms + discard 1 sample |
| Power-Down | High-Performance | 70 ms + see gyro samples-to-discard tables below |
| Sleep | Low-Power / Normal | Discard 1 sample |
| Sleep | High-Performance | See gyro samples-to-discard tables below |
| Low-Power / Normal | High-Performance | Discard 2 samples |
| Low-Power / Normal | LP / Normal (ODR change) | Discard 1 sample |
| High-Performance | Low-Power / Normal | Discard 1 sample |
| High-Performance | HP (ODR change) | Discard 2 samples |
| LP / Normal / HP | Power-Down | 1 μs if both XL and Gyro in PD; 300 μs if XL not in PD |

Settling time is at 99% of final value. (AN5130, p20)

### Gyroscope Samples to Discard — LPF1 Disabled (Table 15)

(AN5130, p21)

| Gyro ODR (Hz) | Samples to Discard |
|--------------|-------------------|
| 12.5 | 2 |
| 26 | 3 |
| 52 | 3 |
| 104 | 3 |
| 208 | 3 |
| 416 | 3 |
| 833 | 3 |
| 1660 | 135 |
| 3330 | 270 |
| 6660 | 540 |

### Gyroscope Samples to Discard — LPF1 Enabled (Table 16)

(AN5130, p21–22)

| Gyro ODR (Hz) | FTYPE=00 | FTYPE=01 | FTYPE=10 | FTYPE=11 |
|--------------|----------|----------|----------|----------|
| 12.5 | 2 | 2 | 2 | 2 |
| 26 | 3 | 3 | 3 | 3 |
| 52 | 3 | 3 | 3 | 3 |
| 104 | 4 | 4 | 4 | 4 |
| 208 | 4 | 4 | 5 | 4 |
| 416 | 5 | 6 | 6 | 5 |
| 833 | 7 | 8 | 9 | 6 |
| 1660 | 135 | 135 | 135 | 135 |
| 3330 | 270 | 270 | 270 | 270 |
| 6660 | 540 | 540 | 540 | 540 |

**Note:** At ODRs ≥ 1.66 kHz, the large sample discard count (135/270/540) dominates and is independent of FTYPE. (AN5130, p21–22)

---

## 27. Addendum — FIFO Pattern Decoding Details

### Pattern Mechanism

Data are stored in the FIFO **without any tag** to maximize sample count. The FIFO_PATTERN[9:0] bits (FIFO_STATUS3/4 registers) indicate which sensor and which axis byte-pair will be read next from FIFO_DATA_OUT_L/H. The pattern counter ranges from 0 to (N−1) where N is the total number of 16-bit samples in one complete pattern cycle, then repeats. (AN5130, p88)

The first sequence always contains data from all enabled FIFO data sets (1st through 4th), ordered from 1st to 4th. Subsequent data within the pattern repeat according to each data set's decimation factor. (AN5130, p88)

### Example 1: Same ODR for Gyro and Accel

**Config:** Gyro ODR = Accel ODR = 104 Hz, DEC_FIFO_GYRO = 001 (none), DEC_FIFO_XL = 001 (none), ODR_FIFO = 104 Hz. (AN5130, p89)

Pattern repeats every **6 samples**: Gx, Gy, Gz, XLx, XLy, XLz

| FIFO_PATTERN[9:0] | Next Reading |
|-------------------|-------------|
| 0 | Gyro X |
| 1 | Gyro Y |
| 2 | Gyro Z |
| 3 | Accel X |
| 4 | Accel Y |
| 5 | Accel Z |

### Example 2: Different ODR for Gyro and Accel

**Config:** Gyro ODR = 208 Hz, Accel ODR = 104 Hz, DEC_FIFO_GYRO = 001, DEC_FIFO_XL = 010 (÷2), ODR_FIFO = 208 Hz. (AN5130, p89)

Pattern repeats every **9 samples**: Gx, Gy, Gz, XLx, XLy, XLz, Gx, Gy, Gz

| FIFO_PATTERN[9:0] | Time | Next Reading |
|-------------------|------|-------------|
| 0 | t0 | Gyro X |
| 1 | t0 | Gyro Y |
| 2 | t0 | Gyro Z |
| 3 | t0 | Accel X |
| 4 | t0 | Accel Y |
| 5 | t0 | Accel Z |
| 6 | t1 | Gyro X |
| 7 | t1 | Gyro Y |
| 8 | t1 | Gyro Z |

### Example 3: Gyro + Accel + Magnetometer at Different ODRs

**Config:** Gyro = 104 Hz, Accel = 208 Hz, Mag = 52 Hz (3rd data set, DEC_DS3 = 100 = ÷4), ODR_FIFO = 208 Hz. (AN5130, p90)

Pattern repeats every **21 samples**:

| FIFO_PATTERN[9:0] | Time | Next Reading |
|-------------------|------|-------------|
| 0 | t0 | Gyro X |
| 1 | t0 | Gyro Y |
| 2 | t0 | Gyro Z |
| 3 | t0 | Accel X |
| 4 | t0 | Accel Y |
| 5 | t0 | Accel Z |
| 6 | t0 | Mag X |
| 7 | t0 | Mag Y |
| 8 | t0 | Mag Z |
| 9 | t1 | Accel X |
| 10 | t1 | Accel Y |
| 11 | t1 | Accel Z |
| 12 | t2 | Gyro X |
| 13 | t2 | Gyro Y |
| 14 | t2 | Gyro Z |
| 15 | t2 | Accel X |
| 16 | t2 | Accel Y |
| 17 | t2 | Accel Z |
| 18 | t3 | Accel X |
| 19 | t3 | Accel Y |
| 20 | t3 | Accel Z |

### FIFO Threshold Resolution

FTH[10:0] resolution is **2 bytes** (1 LSB = 1 × 16-bit sample). Settable range: 0–2047 samples. The WaterM bit in FIFO_STATUS2 goes high when the number of unread samples ≥ FTH. (AN5130, p91)

When STOP_ON_FTH = 1: FIFO depth is limited to FTH level. In FIFO mode, FIFO_FULL_SMART rises when next write would fill the limited FIFO. In Continuous mode, old data is overwritten once threshold is reached. (AN5130, p91–93)

---

## 28. Addendum — Self-Test Procedures (Detailed)

### Gyroscope Self-Test Procedure

The self-test is enabled via `ST_G[1:0]` in CTRL5_C: `01b` = positive, `11b` = negative. When active, output = normal signal + electrostatic test-force. (AN5130, p97)

**Procedure** (from AN5130 Figure 36, p98):

1. Set CTRL2_G = 60h (gyro: 416 Hz, ±250 dps)
2. Set CTRL7_G = 00h (disable HPF)
3. Wait for stable output (800 ms recommended, or discard first samples per Table 14–16)
4. Read 5 consecutive gyro output data sets, average them → NOST (no self-test values for X, Y, Z)
5. Set CTRL5_C = 04h (positive self-test on gyroscope: ST_G = 01b)
6. Wait 60 ms for output stabilization
7. Read 5 consecutive gyro output data sets, average them → ST (self-test values for X, Y, Z)
8. Compute |ST − NOST| for each axis
9. Pass criteria: each axis difference must be within:
   - At ±250 dps (sensitivity 8.75 mdps/LSB): **20–80 dps** → raw difference 2286–9143 LSB
   - At ±2000 dps (sensitivity 70 mdps/LSB): **150–700 dps** → raw difference 2143–10000 LSB
10. Set CTRL5_C = 00h (disable self-test)
11. Set CTRL2_G = 00h (gyro power-down)

(AN5130, p97–98; datasheet_LSM6DS3TR-C.pdf, p22)

### Accelerometer Self-Test Procedure

The self-test is enabled via `ST_XL[1:0]` in CTRL5_C: `01b` = positive, `10b` = negative. (AN5130, p99)

**Procedure** (from AN5130 Figure 37, p100):

1. Set CTRL1_XL = 60h (accel: 416 Hz, ±2 g)
2. Set CTRL8_XL = 00h (disable HP/LPF2)
3. Wait for stable output (100 ms recommended, or discard first samples per Table 12–13)
4. Read 5 consecutive accel output data sets, average them → NOST (no self-test values for X, Y, Z)
5. Set CTRL5_C = 01h (positive self-test on accelerometer: ST_XL = 01b)
6. Wait 100 ms for output stabilization
7. Read 5 consecutive accel output data sets, average them → ST (self-test values for X, Y, Z)
8. Compute |ST − NOST| for each axis
9. Pass criteria: each axis difference must be within **90–1700 mg** (measured at ±2 g sensitivity, 0.061 mg/LSB → raw difference 1475–27869 LSB)
10. Set CTRL5_C = 00h (disable self-test)
11. Set CTRL1_XL = 00h (accel power-down)

(AN5130, p99–100; datasheet_LSM6DS3TR-C.pdf, p22)

---

## 29. Addendum — Block Data Update (BDU) Mechanism

When `BDU = 1` (bit 6 of CTRL3_C, 12h), the output data registers (OUT_TEMP, OUT_G, OUT_XL) are **not updated with new measurement data** until **both** the lower byte (xxxx_L) and upper byte (xxxx_H) of the previous data have been read. This guarantees that the MSB and LSB of any 16-bit output value come from the same measurement sample. (AN5130, p25; datasheet_LSM6DS3TR-C.pdf, p52)

Without BDU (BDU = 0), the registers are updated continuously at the ODR rate. If a multi-byte read spans an update boundary, the lower byte may be from sample N while the upper byte is from sample N+1 (or vice versa), producing a corrupted value. (AN5130, p25)

**BDU is required** when using FIFO (`IF_INC = 1` and `BDU = 1` must both be set). (AN5130, p74, p78)

---

## 30. Addendum — Gyroscope Sleep Mode Details

### Entry Conditions

The gyroscope enters Sleep mode when:
- The gyroscope is NOT in Power-Down mode (an ODR is configured in CTRL2_G), AND
- The `SLEEP_G` (also called `SLEEP`) bit in CTRL4_C (13h) is set to 1

Sleep mode is entered regardless of the selected gyroscope ODR. While in Sleep mode, the driving circuitry that maintains the oscillation of the gyroscope mass remains active. (AN5130, p12)

### Exit Conditions

Clear `SLEEP_G = 0` in CTRL4_C. The gyroscope resumes normal operation at the previously configured ODR. Turn-on from Sleep mode is **drastically faster** than from Power-Down because the driving circuitry was never stopped:
- Sleep → Low-Power/Normal: discard 1 sample only
- Sleep → High-Performance: see gyroscope samples-to-discard table (Section 26)

Compare: Power-Down → any active mode requires 70 ms base time + sample discards. (AN5130, p12, p20, Table 14)

### Activity/Inactivity Automatic Sleep

When `INACT_EN[1:0] = 10` in TAP_CFG (58h), the gyroscope automatically enters Sleep mode during inactivity and wakes when activity is detected. This provides the fastest automatic gyroscope wake-up in response to motion. (AN5130, p47)

---

## 31. Addendum — Boot and Software Reset Details

### Boot Procedure (Reboot Trimming Parameters)

Setting `BOOT = 1` in CTRL3_C reloads trimming parameters from internal non-volatile memory. This does **not** modify the content of control registers — the device operating mode is preserved after boot. The boot takes 15 ms and registers are not accessible during this time. (AN5130, p49)

**Reboot flow:**
1. Set gyroscope to Power-Down (`CTRL2_G = 00h`)
2. Set accelerometer to High-Performance mode (any ODR in CTRL1_XL)
3. Set `BOOT = 1` in CTRL3_C
4. Wait 15 ms

The boot status can be monitored via INT1 pin by setting `INT1_BOOT = 1` in INT1_CTRL. The INT1 signal is high while boot is running and returns low at completion. (AN5130, p49–50)

### Software Reset (SW_RESET)

Setting `SW_RESET = 1` in CTRL3_C resets **all control registers to their default values**. The reset takes approximately 50 μs. The `SW_RESET` bit auto-clears to 0 when reset is complete. (AN5130, p49)

**Reset flow:**
1. Set gyroscope to Power-Down (`CTRL2_G = 00h`)
2. Set accelerometer to High-Performance mode (any ODR in CTRL1_XL)
3. Set `SW_RESET = 1` in CTRL3_C
4. Wait until `SW_RESET` bit reads back as 0 (or wait 50 μs)

**Critical:** BOOT and SW_RESET must **never** be set simultaneously. Execute them sequentially (complete one before starting the other). (AN5130, p49)

---

## 32. Addendum — Accelerometer Analog Anti-Aliasing Filter Bandwidth

The analog anti-aliasing filter bandwidth depends on ODR and power mode. It is active only in High-Performance mode. (AN5130, p13, Table 8)

| Accelerometer ODR | Analog Filter BW |
|-------------------|-----------------|
| ≥ 1666 Hz | 1500 Hz |
| < 1666 Hz | 400 Hz |

The bandwidth can be forced to 400 Hz even at ODR ≥ 1666 Hz by setting `BW0_XL = 1` in CTRL1_XL. (AN5130, p13)

### LPF1 Output Cutoff

The LPF1 filter provides two outputs selectable via `LPF1_BW_SEL` and `INPUT_COMPOSITE` in CTRL8_XL: (AN5130, p13)
- **"ODR/2" output**: BW = ODR/2 in HP mode; 740 Hz in LP/Normal mode
- **"ODR/4" output**: BW = ODR/4 regardless of power mode

---

## 33. Addendum — Additional Current Consumption Details

### Gyroscope-Only Current Consumption

The datasheet (p24) specifies current consumption only for accelerometer-only and combined modes. Gyroscope-only current can be estimated as: (datasheet_LSM6DS3TR-C.pdf, p24)
- Combo HP − Accel HP ≈ 0.90 mA − 0.16 mA ≈ 0.74 mA (gyro contribution at HP)
- Combo Normal − Accel Normal ≈ 0.45 mA − 0.085 mA ≈ 0.365 mA (gyro contribution at normal)

### Power-on Supply Sequence Note

During power-on, pins SCx and SDx should be connected to VDDIO or GND (Mode 1) to avoid undefined states. The device selection between I2C and SPI depends on the CS pin state during power-up. (AN5130, p12; datasheet_LSM6DS3TR-C.pdf, p45)

---

## 34. Addendum — Absolute Maximum Ratings (Complete)

(datasheet_LSM6DS3TR-C.pdf, p29, Table 9)

| Symbol | Parameter | Maximum Value | Unit |
|--------|-----------|--------------|------|
| Vdd | Supply voltage | −0.3 to 4.8 | V |
| TSTG | Storage temperature range | −40 to +125 | °C |
| Sg | Acceleration (0.2 ms half-sine shock) | 10,000 | g |
| ESD | Electrostatic discharge (HBM) | 2 | kV |
| Vin | Input voltage on any control pin (CS, SCL/SPC, SDA/SDI/SDO, SDO/SA0) | −0.3 to VDD_IO + 0.3 | V |

**Warnings:**
- Supply voltage on any pin should **never exceed 4.8 V**. (datasheet_LSM6DS3TR-C.pdf, p29)
- Device is sensitive to mechanical shock — improper handling can cause permanent damage. (datasheet_LSM6DS3TR-C.pdf, p29)
- Device is sensitive to ESD — improper handling can cause permanent damage. (datasheet_LSM6DS3TR-C.pdf, p29)
- Stresses above absolute maximum ratings may cause permanent damage. Exposure to max ratings for extended periods may affect reliability. (datasheet_LSM6DS3TR-C.pdf, p29)
