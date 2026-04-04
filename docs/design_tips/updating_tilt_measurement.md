# Exploiting the Gyroscope to Update Tilt Measurement and eCompass
**Source:** updating_tilt_measurement.pdf | **Doc #:** DT0060 Rev 3 | **Pages:** 7

## Key Takeaways
- Gyroscope data enables tilt/heading updates when accelerometer or magnetometer data is unreliable (high-g motion, magnetic anomalies)
- Euler-angle integration suffers from gimbal lock at Pitch = ±90°; **quaternion implementation is strongly preferred**
- Three quaternion update methods provided: simple addition, exponential (geometrically exact), and Euler-to-quaternion multiplication
- Mixing gyro-derived orientation with Acc+Mag fusion (complementary filter) uses weighted averaging with discontinuity handling
- Gyroscope bias must be subtracted; sensitivity may have ±3% tolerance; Ts accuracy is critical

## Summary

This design tip shows how to propagate orientation (roll, pitch, yaw or quaternion) forward in time using gyroscope angular rate data. This is essential when accelerometer-based tilt or magnetometer-based heading is temporarily unreliable. Two implementations are presented: Euler angles and quaternions.

The Euler-angle approach computes angular derivatives from current angles and gyroscope data (Wx, Wy, Wz), then integrates with timestep Ts. However, this formulation has singularities at Pitch = ±90° (gimbal lock) where Tan(Theta) and 1/Cos(Theta) diverge.

The quaternion approach avoids all singularities. Three update methods are given with increasing accuracy: (1) linear addition + normalization, (2) quaternion exponential multiplication (geometrically exact), and (3) Euler-to-quaternion conversion then multiplication. Optional mixing with accelerometer/magnetometer-derived orientation via weighted averaging (complementary filter) is described, along with proper handling of quaternion sign ambiguity and Euler-angle wrapping discontinuities.

## Technical Details

### Euler Angle Implementation

**Step 1 — Angle Derivatives:**

$$\dot{\Phi} = W_x + W_y \sin\Phi \tan\Theta + W_z \cos\Phi \tan\Theta$$

$$\dot{\Theta} = W_y \cos\Phi - W_z \sin\Phi$$

$$\dot{\Psi} = \frac{W_y \sin\Phi}{\cos\Theta} + \frac{W_z \cos\Phi}{\cos\Theta}$$

**Step 2 — Integration:**

$$\Phi(t+T_s) = \Phi(t) + \dot{\Phi} \cdot T_s$$

$$\Theta(t+T_s) = \Theta(t) + \dot{\Theta} \cdot T_s$$

$$\Psi(t+T_s) = \Psi(t) + \dot{\Psi} \cdot T_s$$

**Angle Reduction** (to proper range):
1. Pitch = mod(Pitch, 360); if Pitch > 180: Pitch -= 360
2. If |Pitch| > 90: Pitch = sign(Pitch)·180 − Pitch, Roll += 180, Yaw += 180
3. Roll = mod(Roll, 360); if Roll > 180: Roll -= 360
4. Yaw = mod(Yaw, 360); if Yaw > 180: Yaw -= 360

### Quaternion Implementation

**Method 1 — Simple Addition:**

$$\dot{Q}_w = (-Q_x W_x - Q_y W_y - Q_z W_z) / 2$$

$$\dot{Q}_x = (+Q_w W_x - Q_z W_y + Q_y W_z) / 2$$

$$\dot{Q}_y = (+Q_z W_x + Q_w W_y - Q_x W_z) / 2$$

$$\dot{Q}_z = (-Q_y W_x + Q_x W_y + Q_w W_z) / 2$$

$$Q(t+T_s) = Q(t) + \dot{Q} \cdot T_s, \quad \text{then normalize}$$

**Method 2 — Quaternion Exponential (preferred):**

$$Q(t+T_s) = Q(t) \otimes \exp\!\left([0, W_x, W_y, W_z] \cdot \frac{T_s}{2}\right)$$

$$W = \sqrt{W_x^2 + W_y^2 + W_z^2}, \quad C = \cos\!\left(\frac{W \cdot T_s}{2}\right), \quad S = \sin\!\left(\frac{W \cdot T_s}{2}\right)$$

$$E_w = C, \quad E_x = S \cdot W_x/W, \quad E_y = S \cdot W_y/W, \quad E_z = S \cdot W_z/W$$

Quaternion multiplication:

$$Q_w' = Q_w E_w - Q_x E_x - Q_y E_y - Q_z E_z$$

$$Q_x' = Q_w E_x + Q_x E_w + Q_y E_z - Q_z E_y$$

$$Q_y' = Q_w E_y - Q_x E_z + Q_y E_w + Q_z E_x$$

$$Q_z' = Q_w E_z + Q_x E_y - Q_y E_x + Q_z E_w$$

**Method 3 — Euler-to-Quaternion then Multiply:**

$$P_w = \cos\tfrac{W_x T_s}{2}\cos\tfrac{W_y T_s}{2}\cos\tfrac{W_z T_s}{2} + \sin\tfrac{W_x T_s}{2}\sin\tfrac{W_y T_s}{2}\sin\tfrac{W_z T_s}{2}$$

$$P_x = \sin\tfrac{W_x T_s}{2}\cos\tfrac{W_y T_s}{2}\cos\tfrac{W_z T_s}{2} - \cos\tfrac{W_x T_s}{2}\sin\tfrac{W_y T_s}{2}\sin\tfrac{W_z T_s}{2}$$

$$P_y = \cos\tfrac{W_x T_s}{2}\sin\tfrac{W_y T_s}{2}\cos\tfrac{W_z T_s}{2} + \sin\tfrac{W_x T_s}{2}\cos\tfrac{W_y T_s}{2}\sin\tfrac{W_z T_s}{2}$$

$$P_z = \cos\tfrac{W_x T_s}{2}\cos\tfrac{W_y T_s}{2}\sin\tfrac{W_z T_s}{2} - \sin\tfrac{W_x T_s}{2}\sin\tfrac{W_y T_s}{2}\cos\tfrac{W_z T_s}{2}$$

Then standard quaternion multiplication $Q(t+T_s) = Q(t) \otimes P$.

### Mixing with Acc+Mag Fusion (Complementary Filter)

**Euler angle mixing** (handles wrapping discontinuity):

```
While |Angle1 - Angle2| > 180:
    Angle1 -= 360 * sign(Angle1 - Angle2)
MixedAngle = Angle1 * α + Angle2 * (1-α),  0 < α < 1
```

**Quaternion mixing** (NLERP):

```
If (Q1w·Q2w + Q1x·Q2x + Q1y·Q2y + Q1z·Q2z) < 0:
    Q2 = -Q2
Q = Q1·α + Q2·(1-α),  then normalize
```

SLERP may be preferred for uniform rotation increments.

### Quaternion to Euler Conversion

$$Q_{mod} = Q_w^2 + Q_x^2 + Q_y^2 + Q_z^2, \quad Q_t = Q_w Q_y - Q_x Q_z$$

**Singularity check:** If $Q_t > +Q_{mod}/2$: Phi = 0, Theta = +90°, Psi = −2·atan2(Qx, Qw). If $Q_t < −Q_{mod}/2$: Phi = 0, Theta = −90°, Psi = +2·atan2(Qx, Qw).

**General case:**

$$\Phi = \text{atan2}(2(Q_w Q_x + Q_y Q_z),\; Q_w^2 - Q_x^2 - Q_y^2 + Q_z^2)$$

$$\Theta = \arcsin\!\left(\frac{2 Q_t}{Q_{mod}}\right)$$

$$\Psi = \text{atan2}(2(Q_w Q_z + Q_x Q_y),\; Q_w^2 + Q_x^2 - Q_y^2 - Q_z^2)$$

### Practical Notes
- **Gyro bias** must be estimated (average output while stationary) and subtracted before use.
- **Sensitivity tolerance** up to ±3%; calibrate by comparing a full-rotation integral against a reference.
- **Ts accuracy** is critical; use the highest available ODR (LSM6DS3TR supports up to 1.6 kHz) for best results.

## Relevance to LSM6DS3TR Implementation

The LSM6DS3TR's built-in gyroscope enables continuous orientation updates even during dynamic motion when accelerometer-based tilt (DT0058) is unreliable. The quaternion exponential method (Method 2) is recommended for embedded use — it is singularity-free and geometrically exact. A complementary filter blending gyro-propagated quaternion with accelerometer-derived tilt (at low α, e.g., 0.02) provides long-term stability while rejecting short-term acceleration disturbances. Gyroscope bias should be estimated at startup and periodically re-estimated during detected still periods. The high 1.6 kHz ODR of LSM6DS3TR directly improves integration accuracy.
