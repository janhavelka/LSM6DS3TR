# Residual Linear Acceleration and Dead Reckoning
**Source:** dead_reckoning.pdf | **Doc #:** DT0106 Rev 1 | **Pages:** 6

## Key Takeaways
- Residual linear acceleration is obtained by subtracting the rotated gravity vector from raw accelerometer output
- Dead reckoning requires double integration of world-frame linear acceleration; offset errors grow linearly (velocity) and quadratically (position) with time
- A leaky integrator (high-pass filter) suppresses low-frequency drift; typical alpha = 0.9–0.95
- Zero-Velocity Update (ZVU) resets the velocity integrator when the device is stationary (|acc| ≈ 1 g and gyro ≈ 0)
- Orientation must come from the gyroscope (not accelerometer) during motion to avoid corrupting gravity subtraction

## Summary

This design tip covers the computation of residual linear acceleration by subtracting the gravity component from accelerometer readings, then using double integration to estimate velocity and position (dead reckoning). Two reference frames are addressed: the sensor body frame and the world (navigation) frame.

In the body frame, the gravity vector [0, 0, 1] is rotated by the current orientation (rotation matrix from quaternion or Euler angles) and subtracted from the measured acceleration. In the world frame, the measured acceleration is de-rotated (multiplied by the transpose of the rotation matrix) and then the gravity vector is subtracted. The world-frame representation is the one needed for dead reckoning.

The document emphasizes that integration drift is the primary challenge. Spurious offsets from imperfect calibration or orientation errors cause velocity error proportional to time and position error proportional to time². Mitigation strategies include leaky integration and Zero-Velocity Updates (ZVU).

## Technical Details

### Body-Frame Linear Acceleration

$$\text{linacc\_body} = -(acc - R \cdot [0;\;0;\;1])$$

Negative sign because accelerometer output is positive when axis points toward gravity.

### World-Frame Linear Acceleration

$$\text{linacc\_world} = -(R^T \cdot acc - [0;\;0;\;1])$$

Or equivalently: $\text{linacc\_world} = R^T \cdot \text{linacc\_body}$

### Rotation Matrix from Quaternion

```
qw2=qw*qw; qx2=qx*qx; qy2=qy*qy; qz2=qz*qz;
n = 1/(qw2+qx2+qy2+qz2);

m11 = ( qx2-qy2-qz2+qw2)*n;    m12 = 2*(qx*qy+qz*qw)*n;    m13 = 2*(qx*qz-qy*qw)*n;
m21 = 2*(qx*qy-qz*qw)*n;       m22 = (-qx2+qy2-qz2+qw2)*n; m23 = 2*(qy*qz+qx*qw)*n;
m31 = 2*(qx*qz+qy*qw)*n;       m32 = 2*(qy*qz-qx*qw)*n;    m33 = (-qx2-qy2+qz2+qw2)*n;
```

### Rotation Matrix from Euler Angles (NED Frame)

$$R = R_x(\Phi) \cdot R_y(\Theta) \cdot R_z(\Psi)$$

| Element | Formula |
|---------|---------|
| $m_{11}$ | $\cos\Theta\cos\Psi$ |
| $m_{12}$ | $\cos\Theta\sin\Psi$ |
| $m_{13}$ | $-\sin\Theta$ |
| $m_{21}$ | $\sin\Phi\sin\Theta\cos\Psi - \cos\Phi\sin\Psi$ |
| $m_{22}$ | $\sin\Phi\sin\Theta\sin\Psi + \cos\Phi\cos\Psi$ |
| $m_{23}$ | $\sin\Phi\cos\Theta$ |
| $m_{31}$ | $\cos\Phi\sin\Theta\cos\Psi + \sin\Phi\sin\Psi$ |
| $m_{32}$ | $\cos\Phi\sin\Theta\sin\Psi - \sin\Phi\cos\Psi$ |
| $m_{33}$ | $\cos\Phi\cos\Theta$ |

### Simplified Gravity Rotation (Euler-based, body frame)

$$g_x = -\sin\Theta, \quad g_y = \cos\Theta\sin\Phi, \quad g_z = \cos\Theta\cos\Phi$$

$$\text{linacc} = -(acc - [g_x, g_y, g_z])$$

### Quaternion-Based Gravity Rotation

$$g_x = 2(q_x q_z - q_w q_y), \quad g_y = 2(q_w q_x + q_y q_z), \quad g_z = q_w^2 - q_x^2 - q_y^2 + q_z^2$$

### Dead Reckoning with Leaky Integrator

```
acc = acc * 9.81;  // g → m/s²
for i = 2 to N:
    vel_hp[i] = c * (acc[i] * Ts) + alpha * vel_hp[i-1]    // 1st integration
    pos_hp[i] = c * (vel_hp[i] * Ts) + alpha * pos_hp[i-1] // 2nd integration
```

- $\alpha$ = 0.9–0.95 (leaky factor / high-pass)
- $c = 1$ (simple) or $c = (1+\alpha)/2$ (unity gain at high frequencies)
- $T_s$ = sampling interval in seconds

### Zero-Velocity Update (ZVU)

When $|acc| \approx 1\;g$ **and** gyroscope output $\approx 0$: reset $vel_{hp}[i] = [0, 0, 0]$.

- Does not work during constant-velocity motion (false trigger).
- For offline processing, backward integration from ZVU points can be blended with forward integration using a linear weight ramp.

## Relevance to LSM6DS3TR Implementation

The LSM6DS3TR's integrated accelerometer and gyroscope provide all data needed for body-frame and world-frame linear acceleration extraction. The gyroscope is critical — orientation must be updated from gyroscope data (DT0060) during motion, since accelerometer-based tilt (DT0058) is invalid under dynamic acceleration. Calibration of both accelerometer (DT0105) and gyroscope bias is essential to minimize integration drift. For embedded dead reckoning on a resource-constrained MCU, the leaky-integrator approach with ZVU provides a practical drift-bounded solution without requiring external position aiding.
