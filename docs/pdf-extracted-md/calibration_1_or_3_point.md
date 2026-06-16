# calibration 1 or 3 point

- Source PDF: `../design_tips/calibration_1_or_3_point.pdf`
- Extraction tool: pdfplumber
- Page count: 5
- SHA256: `0e2089de13ab537736e174c76eaee17fc6ac164c69324862716859688db9e963`

## Page 1

DT0105
Design tip
1-point or 3-point tumble sensor calibration
By Andrea Vitali
Purpose and benefits
This design tip explains how to compute offset and gain for a 3-axis sensor (usually an
accelerometer) by performing a 1 or 3-point tumble calibration. A generalization of the
algorithm is also described.
Benefits:
• Added functionality with respect to calibration provided by the MotionFX library which
only provides magnetometer and gyroscope calibration and not accelerometer
calibration.
• Short and essential implementation which enables easy customization and
enhancement by the end-user (MotionFX is available only in binary format, not as
source code)
• Easy to use on every microcontroller (MotionFX can only be run on STM32)
Scope
This design tip applies to all accelerometers, eCompass modules, and iNemo inertial IMUs
from STMicroelectronics.
Specifications
Algorithm specifications:
• Input from 3-axis sensor: data triplets for each position
• Output of 1-point tumble c[axl,iyb,zra] tion: offset for each axis ( )
• Output of 3-point tumble calibration: offset for each axis ( ) and gain for
Xofs, Yofs, Zofs
each axis ( )
Xofs, Yofs, Zofs
Xgain, Ygain, Zgain
January 2022 DT0105 Rev 2 1/5
www.st.com

## Page 2

Algorithm description for 1-point tumble calibration
The algorithm is described for the particular case of an accelerometer. However, it can also
be used with other sensors, e.g. a magnetometer. See notes at the end of this document.
It is assumed that the sensor has nominal sensitivity equal to 1 for each axis and there is
no cross-axis sensitivity. True acceleration is related to measured acceleration as follows:
AAAAAAAA 11 00 00 ttttttttAAAAAAAA AAXXXXXX
�AAAAAAAA� = �00 11 00� �ttttttttAAAAAAAA� + �AAXXXXXX�
The sensor should be orienteAAd AAsAAoAA that on00ly o00ne11 axistt tt(tte.ttgAA. AAZAA AA= 1g) AAisXX sXXtXXimulated while the
others are orthogonal to the stimulus (e.g. X=Y=0g): trueAcc = [0, 0, +1]
Measured acceleration, derived from the equation shown at the beginning, is calculated by
plugging in the values listed above for true acceleration:
1.
2. AccX = 0 + Xofs, Xofs = AccX
3. AccY = 0 + Yofs, Yofs = AccY
Offsets aArcec Zre =ad 1ily + a Zvoafisla, b Zleo fasn =d AcaccnZ n-1ow be subtracted to go from measured acceleration
to true acceleration.
It must be noted that if the sensitivity is not 1 as assumed, or if cross-axis sensitivities are
not 0 as assumed, the computed offset will be wrong. Also if the true acceleration is not
[0, 0, 1] during calibration, the computed offset will be wrong. This can happen for non-ideal
sensors and imperfect alignment during calibration.
Algorithm description for 3-point tumble calibration
The algorithm is described for the particular case of an accelerometer. However, it can also
be used with other sensors, e.g. a magnetometer. See notes at the end of this document.
It is assumed that the sensor has no cross-axis sensitivity. True acceleration is related to
measured acceleration as follows:
AAAAAAAA AAXXXXXXXX 00 00 ttttttttAAAAAAAA AAXXXXXX
�AAAAAAAA� = � 00 AAXXXXXXXX 00 � �ttttttttAAAAAAAA� + �AAXXXXXX�
The sensor should be AAorAAiAAeAAnted so th00at only o00ne axAAisXX XXaXXtXX a timttttett ttisAA sAAtAAimAA ulateAAdXX (XXeXX.g. X, then Y,
then Z) while the others are orthogonal to the stimulus.
True acceleration for each position in a 3-point tumble calibration is as follows:
1[. x,yG,zr]avity vector along +X axis:
2. Gravity vector along +Y axis: trueAcc = [+1, 0, 0 ]
3. Gravity vector along +Z axis: trueAcc = [ 0, +1, 0 ]
January 2022 DT0105 Rev 2 2/5
trueAcc = [ 0, 0, +1]
www.st.com

## Page 3

Measured acceleration for each position in a 3-point tumble calibration, derived from the
equation shown at the beginning, is calculated by plugging in the values listed above for
true acceleration:
1.
2. AccX1 = Xgain+Xofs, AccY1 = 0 +Yofs, AccZ1 = 0 +Zofs
3. AccX2 = 0 +Xofs, AccY2 = Ygain+Yofs, AccZ2 = 0 +Zofs
Offsets cAacnc Xb3e =ta k e 0n d +ireXcotflys, f r oAmcc Yth3e = m e 0a s u r+emYoefns,t s AlicscteZ3d =ab oZgvaei no+r tZhoefys can be computed
by averaging two out of three measurements listed above. Averaging can be used to
improve the quality of the final estimate.
•
• AccX2+AccX3 = 2 Xofs, Xofs = (AccX2 + AccX3)/2
• AccY1+AccY3 = 2 Yofs, Yofs = (AccY1 + AccY3)/2
Once offAscectsZ 1a+reA cccoZm2p =u t2e dZ,o gfsa, i n Zso cfsa =n b(Aec ccoZm1 p+u AtecdcZ a2s) /fo2l lows:
•
• Xgain = AccX1 - Xofs
• Ygain = AccY2 - Yofs
Now offsZegtasi nc a=n Abcec Zs3u b- tZraocfst ed, and multiplication by inverse gains can be done to go from
measured acceleration to true acceleration.
Notes
Applications to other sensors:
• While it may be easy to exploit the known gravity vector to impose the desired true
reference on the accelerometer, it may not be possible to achieve perfect accuracy
when switching from one position to another during the 3-point tumble calibration
• An alternative way to perform the calibration is to use a gold reference sensor
which has the same orientation of the sensor to be calibrated, so that the desired
true reference can be measured and checked.
• Special equipment, arrangements or procedures may be needed to impose or
measure the desired true reference on other sensors such as magnetometers or
gyroscopes
• For the case of the magnetometer: Helmholtz coils can be used to impose the
desired true reference; alternatively, the Earth’s magnetic field vector can be used
together with a gold reference sensor.
January 2022 DT0105 Rev 2 3/5
www.st.com

## Page 4

• For the case of the gyroscope: a single axis turn table or a step motor spin table
can be used to impose the desired true reference; alternatively a gold reference
- -
sensor can be used as previously described.
Other algorithms: 6-point tumble calibration, discussed in Design Tip DT0053, can be used
to estimate offsets, gains and cross-axis gains. In the same Design Tip, a generalization is
also presented for N-point tumble calibration.
Support material
Related design support material
BlueMicrosystem1, Bluetooth low energy and sensors software expansion for STM32Cube
Open.MEMS, MotionFX, Real-time motion-sensor data fusion software expansion for STM32Cube
Documentation
Application note, AN4508, Parameters and calibration of a low-g 3-axis accelerometer
Application note, AN4615, Fusion and compass calibration APIs for the STM32 Nucleo with
the X-NUCLEO-IKS01A1 sensors expansion board
Design Tip, DT0053, 6-point tumble sensor calibration
Revision history
Date Version Changes
28-Aug-2018 1 Initial release
Updated “Purpose and benefits” and “Specifications” on
21-Jan-2022 2
page 1
January 2022 DT0105 Rev 2 4/5
www.st.com

## Page 5

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
© 2022 STMicroelectronics – All rights reserved
January 2022 DT0105 Rev 2 5/5
www.st.com
