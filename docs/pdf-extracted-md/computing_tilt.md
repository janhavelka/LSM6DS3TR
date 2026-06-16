# computing tilt

- Source PDF: `../design_tips/computing_tilt.pdf`
- Extraction tool: pdfplumber
- Page count: 6
- SHA256: `541e1501d15cc3a1be1e45f4e98a91d4005c9e94b471407ed3b9775fe4023c5d`

## Page 1

DT0058
Design tip
Computing tilt measurement and tilt-compensated eCompass
By Andrea Vitali
Main components
LSM303AGR Ultra-compact high-performance eCompass module:
ultra-low-power 3D accelerometer and 3D magnetometer
LSM303AH Ultra-compact high-performance eCompass module:
ultra-low-power 3D accelerometer and 3D magnetometer
Purpose and benefits
This design tip explains how to compute tilt (roll and pitch angles) from accelerometer data.
It also explains how to compute eCompass (yaw angle) from tilt-compensated
magnetometer data. The conversion from Euler angles to Quaternions is also shown.
Benefits:
• Added functionality with respect to data fusion provided by the MotionFX library
embedded in the X-Cube-MEMS1 software package which provides 9-axis
Acc+Mag+Gyro and 6-axis Acc+Gyro fusion but not 6-axis Acc+Mag
• Reduction of the firmware footprint with respect to using the full-blown data fusion
provided by the MotionFX library embedded in the X-Cube-MEMS1 software package -
see references in design Support Material paragraph
• Short essential implementation, which enables easy customization and enhancement
by the end-user (MotionFX is available only in binary format, not as source code)
• Easy to use solution, applicable to all microcontrollers, although the MotionFX library
can only be run on the STM32 MCU family
Description
Step 1: Computation of Phi (roll angle, also known as bank; see Figure 1 for reference)
based on accelerometer data (Gx, Gy and Gz; unit of measure does not matter)
Roll: Phi = Atan2( Gy, Gz )
Note: If Theta = ±90 deg, then Gy and Gz are near-zero and Phi is unstable; if
stability is desired, then Gz can be substituted by Gz+Gx*alpha, with
alpha = 0.01-0.05.
January 2021 DT0058 Rev 3 1/6
www.st.com

## Page 2

Step 2: Computation of Theta (pitch angle, also known as attitude; see Figure 1 for
reference) based on accelerometer data (Gx, Gy and Gz; unit of measure does not matter)
Gz2 = Gy * Sin( Phi ) + Gz * Cos( Phi )
Pitch: Theta = Atan( -Gx / Gz2)
Note: If Theta = ±90 deg, then Gy and Gz are near-zero and Gz2 is also near-zero;
division by zero should not be performed, Theta = -90 deg if Gx>0, and
Theta = +90 deg if Gx<0; Gx cannot be zero.
Step 3: Computation of Psi (yaw angle, also known as heading; see Figure 1 for reference)
based on magnetometer data (Bx, By and Bz; unit of measure does not matter)
By2 = Bz * Sin( Phi ) – By * Cos( Phi )
Bz2 = By * Sin( Phi ) + Bz * Cos( Phi )
Bx3 = Bx * Cos( Theta ) + Bz2 * Sin( Theta )
Yaw: Psi = Atan2( By2 , Bx3)
Note: If Theta = ±90 deg and if Phi is unstable, Psi will also be unstable; if Phi is
made stable as mentioned above, then Psi will also be stable. Regardless of
stability, the sum Phi+Psi will always be stable. See paragraph on singularities and
gimbal-lock.
Step 4: Conversion from Euler angles to Quaternions (optional step)
Qw = Cos(Phi/2)*Cos(Theta/2)*Cos(Psi/2) + Sin(Phi/2)*Sin(Theta/2)* Sin(Psi/2)
Qx = Sin(Phi/2)*Cos(Theta/2)*Cos(Psi/2) – Cos(Phi/2)*Sin(Theta/2)*Sin(Psi/2)
Qy = Cos(Phi/2)*Sin(Theta/2)*Cos(Psi/2) + Sin(Phi/2)*Cos(Theta/2)*Sin(Psi/2)
Qz = Cos(Phi/2)*Cos(Theta/2)*Sin(Psi/2) – Sin(Phi/2)*Sin(Theta/2)*Cos(Psi/2)
Singularities and gimbal-lock
If Theta = ±90 deg, Phi and Psi will describe a rotation around the same vertical
axis (one degree of freedom is lost, this is also known as gimbal lock); the sum
Psi+Phi will be correct when Theta=-90 deg, or Psi-Phi when Theta=+90 deg.
High-g motion
Accelerometer data cannot be used for tilt measurement (Phi and Theta) if high-g
is ongoing. Accelerometer data can only be used when the modulus is near g: Gx2
+ Gy2 + Gz2 = 1, when Gx / Gy / Gz are expressed in g.
Gyroscope data can be used to update the tilt output when the accelerometer data
cannot be used.
January 2021 DT0058 Rev 3 2/6
www.st.com

## Page 3

Hard/Soft iron effects and Magnetic anomalies
Magnetometer data cannot be used for the eCompass (Psi) if hard/soft iron effects
are not compensated and/or if magnetic anomalies are present. Hard-iron effects
are offsets that must be subtracted from Bx / By / Bz. Soft-iron effects are non-
unity axis sensitivities and non-zero cross-axis sensitivities that must be
compensated by matrix multiplication.
Magnetometer data can only be used when the modulus is of near nominal Earth
field value, which happens when there are no magnetic anomalies and hard/soft
iron effects are compensated: Bx2 + By2 + Bz2 = B.
Gyroscope data can be used to update the eCompass output when the
magnetometer data cannot be used.
Figure 1. Reference orientation for input data from accelerometer and magnetometer, and
reference orientation for output data: roll, pitch and yaw angles
Accelerometer output is
positive for axis aligned with
gravity and pointing down
Roll = +90
Roll = +90 Pitch = +90 Yaw = +90
Pitch = +90
Roll, Pitch, Yaw Acc (Gravity vector) Mag (Earth mag field)
degrees G = 1g = 1000mg B = 10..90uT = 100..900 mG
0, 0, 0 0, 0, +G +B Cos(i), 0, +B Sin(i)
+90, 0, 0 0, +G, 0 +B Cos(i), +B Sin(i), 0
0, +90, 0 -G, 0, 0 -B Sin(i), 0, +B Cos(i)
0, 0, +90 0, 0, +G 0, -B Cos(i), +B Sin(i)
+90, +90, 0 -G, 0, 0 -B Sin(i), +B Cos(i), 0
Generic tilt angle with respect to horizontal plane
The generic tilt angle, with respect to the horizontal plane, can be derived from the
formula of dot-product or vector-product of current acceleration vector and gravity
vector pointing downward. The gravity vector has components x=0, y=0, z=1.
January 2021 DT0058 Rev 3 3/6
www.st.com

## Page 4

The dot-product of two vectors A and B is A.B = |A| |B| cos(angle) = Ax Bx + Ay By + Az
Bz. When A is the current acceleration (Gx, Gy, Gz) and B is the gravity pointing downward
(x=0, y=0, z=1): |G| cos(angle) = Gz, therefore tilt angle = acos ( Gz / |G| ).
Tilt = acos( Gz / sqrt(Gx2+Gy2+Gz2) ) from the dot-product formula
The vector-product of two vectors A and B is C = AxB = |A| |B| N sin(angle), where N is
the unity vector orthogonal to both vectors A and B. Cx = Ay Bz – Az By, Cy = Az Bx – Ax
Bz, Cz = Ax By – Ay Bx. When A is the current acceleration (Gx, Gy, Gz) and B is the
gravity pointing downward (x=0, y=0, z=1): |C| = |G| sin(angle) = sqrt(Gx2 + Gy2), therefore
tilt angle = asin ( |C| / |G| ).
Tilt = asin( sqrt(Gx2 + Gy2) / sqrt(Gx2+Gy2+Gz2) ) from the vector-product
Taking into account measurement noise, the asin function is more accurate if the angle is
small, that is when -45<angle<+45 deg, while the acos function is accurate when the angle
is large, that is angle<-45 or angle>+45. To get highest accuracy for all angles, use the
atan or atan2 function:
Tilt = atan( sqrt(Gx2 + Gy2) / Gz ) if Gz=0, atan(±infinity)= ±90 deg
Tilt = atan2( sqrt(Gx2 + Gy2) , Gz ) no need to check if Gz=0
Figure 2. Roll and pitch angles compared to generic tilt angle with respect to horizontal plane
Optimized implementation for trigonometric functions (sin, cos, atan2)
If a floating-point unit (FPU) is available on the microcontroller (e.g. on Cortex-M4
in STM32F4 or STM32L4), then the vectorized computation may be available to
speed up computations. Data is collected in buffers, and then it is processed in
batches.
January 2021 DT0058 Rev 3 4/6
www.st.com

## Page 5

If single-cycle multiplication is available on the microcontroller (e.g. on Cortex-M3
in STM32F1, STM32L1, STM32F3), then polynomial approximation with Horner
evaluation order can be very effective.
Finally, the simplest microcontrollers may benefit from CORDIC implementation
which requires only table look-up and shift-and-add operations. Roughly speaking,
the CORDIC algorithm requires as many iterations as the number of precision bits;
also, two functions are computed simultaneously: sin & cos, atan2 and modulus.
Support material
Related design support material
FP-SNS-MOTENV1, STM32Cube function pack for IoT node with BLE connectivity and
environmental and motion sensors
User manual, UM2016, Getting started with the STM32 ODE function pack for IoT node with BLE
connectivity and environmental and motion sensors
Quick Start Guide, STM32Cube function pack for IoT node with BLE connectivity, environmental
and motion sensors (FP-SNS-MOTENV1)
X-CUBE-MEMS1, Sensor and motion algorithm software expansion for STM32Cube
User manual, UM2220, Getting started with MotionFX sensor fusion library in X-CUBE-MEMS1
expansion for STM32Cube
Documentation
Application note, AN4509, Tilt measurement using a low-g 3-axis accelerometer
Revision history
Date Version Changes
05-Nov-2015 1 Initial release
22-Nov-2018 2 Added paragraph on generic tilt; updated Figure 1
07-Jan-2021 3 Updated references to reflect current software packages
Corrected formula for tilt computation
January 2021 DT0058 Rev 3 5/6
www.st.com

## Page 6

IMPORTANT NOTICE – PLEASE READ CAREFULLY
STMicroelectronics NV and its subsidiaries (“ST”) reserve the right to make changes, corrections, enhancements,
modifications, and improvements to ST products and/or to this document at any time without notice. Purchasers should
obtain the latest relevant information on ST products before placing orders. ST products are sold pursuant to ST’s terms and
conditions of sale in place at the time of order acknowledgement.
Purchasers are solely responsible for the choice, selection, and use of ST products and ST assumes no liability for
application assistance or the design of Purchasers’ products.
No license, express or implied, to any intellectual property right is granted by ST herein.
Resale of ST products with provisions different from the information set forth herein shall void any warranty granted by ST for
such product.
ST and the ST logo are trademarks of ST. For additional information about ST trademarks, please refer to
www.st.com/trademarks. All other product or service names are the property of their respective owners.
Information in this document supersedes and replaces information previously supplied in any prior versions of this document.
© 2021 STMicroelectronics – All rights reserved
January 2021 DT0058 Rev 3 6/6
www.st.com
