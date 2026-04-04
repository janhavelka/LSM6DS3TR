# 1-Point or 3-Point Tumble Sensor Calibration
**Source:** calibration_1_or_3_point.pdf | **Doc #:** DT0105 Rev 2 | **Pages:** 5

## Key Takeaways
- 1-point calibration computes only **offset** per axis (assumes unity gain); 3-point calibration computes both **offset and gain**
- Assumes no cross-axis sensitivity; use 6-point tumble (DT0053) for full cross-axis calibration
- 3-point method averages two idle-axis readings per offset for better estimates
- Algorithm generalizes to magnetometers and gyroscopes with appropriate reference stimuli
- Imperfect alignment during calibration directly corrupts computed offsets

## Summary

The design tip presents two lightweight calibration algorithms for 3-axis sensors (primarily accelerometers). Both exploit the known gravity vector as a reference stimulus by placing the sensor in precisely oriented positions.

**1-point calibration** assumes unity gain on all axes. The sensor is placed so one axis aligns with gravity (e.g., Z = +1 g, X = Y = 0 g). Offsets are read directly: the unstimulated axes give their offset as the raw reading, and the stimulated axis offset is the reading minus 1. This is fast but sensitive to gain errors and misalignment.

**3-point calibration** relaxes the unity-gain assumption. The sensor is oriented three times so gravity aligns with each axis in turn (+X, +Y, +Z). Offsets are computed by averaging the two measurements where a given axis should read zero, and gains are derived by subtracting the offset from the stimulated reading.

Both methods are portable to any microcontroller and do not require the MotionFX binary library. They cannot compensate for cross-axis sensitivity — the 6-point tumble calibration (DT0053) or N-point generalization is needed for that. Special equipment (Helmholtz coils, turn tables, gold reference sensors) may be required when calibrating magnetometers or gyroscopes.

## Technical Details

### Sensor Model

$$\begin{bmatrix} Acc_X \\ Acc_Y \\ Acc_Z \end{bmatrix} = \begin{bmatrix} X_{gain} & 0 & 0 \\ 0 & Y_{gain} & 0 \\ 0 & 0 & Z_{gain} \end{bmatrix} \begin{bmatrix} true_X \\ true_Y \\ true_Z \end{bmatrix} + \begin{bmatrix} X_{ofs} \\ Y_{ofs} \\ Z_{ofs} \end{bmatrix}$$

For 1-point calibration, $X_{gain} = Y_{gain} = Z_{gain} = 1$.

### 1-Point Calibration (sensor Z-axis pointing down toward gravity)

| Axis | Formula |
|------|---------|
| X offset | $X_{ofs} = Acc_X$ |
| Y offset | $Y_{ofs} = Acc_Y$ |
| Z offset | $Z_{ofs} = Acc_Z - 1$ |

### 3-Point Calibration

Three positions: gravity along +X, +Y, +Z in turn. Measured data: $(Acc_{X1..3}, Acc_{Y1..3}, Acc_{Z1..3})$.

| Position | AccX | AccY | AccZ |
|----------|------|------|------|
| +X up | Xgain + Xofs | Yofs | Zofs |
| +Y up | Xofs | Ygain + Yofs | Zofs |
| +Z up | Xofs | Yofs | Zgain + Zofs |

**Offsets (averaged from two zero-stimulus readings):**

$$X_{ofs} = \frac{Acc_{X2} + Acc_{X3}}{2}, \quad Y_{ofs} = \frac{Acc_{Y1} + Acc_{Y3}}{2}, \quad Z_{ofs} = \frac{Acc_{Z1} + Acc_{Z2}}{2}$$

**Gains:**

$$X_{gain} = Acc_{X1} - X_{ofs}, \quad Y_{gain} = Acc_{Y2} - Y_{ofs}, \quad Z_{gain} = Acc_{Z3} - Z_{ofs}$$

**Correction applied as:**

$$true_{axis} = \frac{Acc_{axis} - Offset_{axis}}{Gain_{axis}}$$

### Notes on Other Sensors
- **Magnetometer:** Use Helmholtz coils or Earth's magnetic field + gold reference sensor to impose the known reference vector.
- **Gyroscope:** Use a single-axis turn table or step-motor spin table.
- For offset + gain + cross-axis estimation, upgrade to **6-point tumble** (DT0053) or **N-point** calibration.

## Relevance to LSM6DS3TR Implementation

The LSM6DS3TR contains a 3-axis accelerometer and 3-axis gyroscope. In production or field deployment, per-unit offset and gain errors can be removed using these calibration procedures. The 1-point method is suitable for quick offset-only calibration during initialization (place the board flat, Z-down). The 3-point method provides higher accuracy when gain compensation is also needed. For the gyroscope, a turn-table or gold reference is required. Calibration values should be stored in non-volatile memory (e.g., the MB85RC FRAM in this system) and applied to raw readings before any tilt, dead-reckoning, or fusion computation.
