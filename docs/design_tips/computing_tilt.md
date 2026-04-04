# Computing Tilt Measurement and Tilt-Compensated eCompass
**Source:** computing_tilt.pdf | **Doc #:** DT0058 Rev 3 | **Pages:** 6

## Key Takeaways
- Roll (Phi) and Pitch (Theta) are derived from accelerometer data alone; Yaw (Psi) requires tilt-compensated magnetometer data
- Gimbal lock occurs at Pitch = ±90°; the sum Phi + Psi remains stable even in this singularity
- Generic tilt angle relative to horizontal is best computed with `atan2` for full-range accuracy
- Accelerometer-based tilt is only valid when the total acceleration modulus ≈ 1 g (no high-g motion)
- Euler-to-Quaternion conversion is provided for singularity-free downstream processing

## Summary

This design tip derives tilt angles (roll, pitch) from 3-axis accelerometer readings and yaw (heading) from tilt-compensated 3-axis magnetometer readings, following an Euler-angle formulation. It also provides the conversion to quaternion representation.

The roll angle (Phi) is computed first from Gy and Gz. The pitch angle (Theta) uses Gx and a roll-compensated Gz component (Gz2). The yaw angle (Psi) is then obtained from magnetometer data (Bx, By, Bz) after compensating for the computed roll and pitch. Hard-iron offsets must be subtracted and soft-iron effects corrected via matrix multiplication before magnetometer data is used.

A generic tilt angle with respect to the horizontal plane is also derived using dot-product and cross-product identities, with `atan2` recommended for best numerical accuracy across all angles. Optimization strategies for trigonometric functions are discussed: FPU vectorization on Cortex-M4, Horner polynomial approximation on Cortex-M3, and CORDIC for simpler cores.

## Technical Details

### Step 1 — Roll (Phi) from Accelerometer

$$\Phi = \text{atan2}(G_y,\; G_z)$$

Stability fix when $\Theta \approx \pm 90°$: substitute $G_z$ with $G_z + G_x \cdot \alpha$, where $\alpha = 0.01 \text{–} 0.05$.

### Step 2 — Pitch (Theta) from Accelerometer

$$G_{z2} = G_y \sin(\Phi) + G_z \cos(\Phi)$$

$$\Theta = \text{atan}\!\left(\frac{-G_x}{G_{z2}}\right)$$

If $G_{z2} = 0$: $\Theta = -90°$ when $G_x > 0$, $\Theta = +90°$ when $G_x < 0$.

### Step 3 — Yaw (Psi) from Tilt-Compensated Magnetometer

$$B_{y2} = B_z \sin(\Phi) - B_y \cos(\Phi)$$

$$B_{z2} = B_y \sin(\Phi) + B_z \cos(\Phi)$$

$$B_{x3} = B_x \cos(\Theta) + B_{z2} \sin(\Theta)$$

$$\Psi = \text{atan2}(B_{y2},\; B_{x3})$$

### Step 4 — Euler to Quaternion Conversion

$$Q_w = \cos\tfrac{\Phi}{2}\cos\tfrac{\Theta}{2}\cos\tfrac{\Psi}{2} + \sin\tfrac{\Phi}{2}\sin\tfrac{\Theta}{2}\sin\tfrac{\Psi}{2}$$

$$Q_x = \sin\tfrac{\Phi}{2}\cos\tfrac{\Theta}{2}\cos\tfrac{\Psi}{2} - \cos\tfrac{\Phi}{2}\sin\tfrac{\Theta}{2}\sin\tfrac{\Psi}{2}$$

$$Q_y = \cos\tfrac{\Phi}{2}\sin\tfrac{\Theta}{2}\cos\tfrac{\Psi}{2} + \sin\tfrac{\Phi}{2}\cos\tfrac{\Theta}{2}\sin\tfrac{\Psi}{2}$$

$$Q_z = \cos\tfrac{\Phi}{2}\cos\tfrac{\Theta}{2}\sin\tfrac{\Psi}{2} - \sin\tfrac{\Phi}{2}\sin\tfrac{\Theta}{2}\cos\tfrac{\Psi}{2}$$

### Generic Tilt Angle (relative to horizontal)

Using dot-product: $\text{Tilt} = \arccos\!\left(\dfrac{G_z}{\sqrt{G_x^2 + G_y^2 + G_z^2}}\right)$

Using cross-product: $\text{Tilt} = \arcsin\!\left(\dfrac{\sqrt{G_x^2 + G_y^2}}{\sqrt{G_x^2 + G_y^2 + G_z^2}}\right)$

**Best (full-range) formula:**

$$\text{Tilt} = \text{atan2}\!\left(\sqrt{G_x^2 + G_y^2},\; G_z\right)$$

### Singularities and Gimbal Lock
At $\Theta = \pm 90°$, $\Phi$ and $\Psi$ rotate around the same vertical axis (one DOF lost). The sum $\Psi + \Phi$ (when $\Theta = -90°$) or $\Psi - \Phi$ (when $\Theta = +90°$) remains correct.

### Validity Conditions
- **Accelerometer tilt** valid only when $G_x^2 + G_y^2 + G_z^2 \approx 1\;g$ (no linear acceleration).
- **Magnetometer yaw** valid only when $B_x^2 + B_y^2 + B_z^2 \approx B_{earth}$ (no magnetic anomalies, hard/soft-iron compensated).

## Relevance to LSM6DS3TR Implementation

The LSM6DS3TR provides the 3-axis accelerometer data needed for roll and pitch computation. For a full eCompass (yaw), an external magnetometer is required. The roll/pitch formulas should be applied after offset/gain calibration (see DT0105). When high-g motion is detected (modulus ≠ 1 g), the firmware should fall back to gyroscope-based tilt updates (see DT0060). The `atan2`-based generic tilt formula is recommended for applications such as inclinometers or leveling where only the angle from horizontal is needed.
