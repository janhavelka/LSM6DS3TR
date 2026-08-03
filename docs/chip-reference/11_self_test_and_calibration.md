# Self-Test And Calibration

> Source confidence: self-test is a vendor-recommended procedure; calibration
> is explicit library/application policy. Library status: both are supported as
> bounded owner-scheduled operations with full-profile restoration/readback and
> explicit restoration evidence.

## Accelerometer Self-Test — AN5130 Figure 37

Keep the device stationary for the complete operation. Use the ST procedure,
not an arbitrary active profile:

1. Write `CTRL1_XL=0x38` (52 Hz, +/-4 g), `CTRL2_G=0x00`,
   `CTRL3_C=0x44` (BDU and IF_INC), and `CTRL4_C..CTRL10_C=0x00`.
2. Wait 100 ms.
3. Wait for `STATUS_REG.XLDA`, read and discard the first XYZ sample.
4. Average five XYZ samples with self-test disabled.
5. Write `CTRL5_C=0x01` for positive accelerometer self-test.
6. Wait 100 ms.
7. Wait for XLDA, read and discard the first stimulated sample.
8. Average five stimulated XYZ samples.
9. Convert using the +/-4 g sensitivity, compute the absolute stimulated minus
   baseline delta per axis, and require 90..1700 mg inclusive.
10. Write `CTRL1_XL=0x00` to power down the accelerometer, then
    `CTRL5_C=0x00` to disable self-test, in that order. Only then restore the
    complete pre-test managed profile.

Source: AN5130 Rev 1, Figure 37, pp. 99-100; datasheet mechanical
characteristics, pp. 21-22.

## Gyroscope Self-Test — AN5130 Figure 36

1. Write `CTRL1_XL=0x00`, `CTRL2_G=0x5C` (208 Hz, +/-2000 dps),
   `CTRL3_C=0x44`, and `CTRL4_C..CTRL10_C=0x00`.
2. Wait 150 ms.
3. Wait for `STATUS_REG.GDA`, read and discard the first XYZ sample.
4. Average five XYZ samples with self-test disabled.
5. Write `CTRL5_C=0x04` for positive gyroscope self-test.
6. Wait 50 ms.
7. Wait for GDA, read and discard the first stimulated sample.
8. Average five stimulated XYZ samples.
9. Convert using 70 mdps/LSB, compute the absolute delta per axis, and require
   150..700 dps inclusive.
10. Write `CTRL2_G=0x00` to power down the gyroscope, then
    `CTRL5_C=0x00` to disable self-test, in that order. Only then restore the
    complete pre-test managed profile.

Source: AN5130 Rev 1, Figure 36, pp. 97-98; datasheet mechanical
characteristics, pp. 21-22.

The public request permits 5..100 averaged samples per phase. Five is ST's
specified procedure; a larger caller-selected count is a bounded library policy
that only increases averaging after the same required first-sample discard.
Each needed sample allows at most three status checks before failing visibly.
Every sample, including the discarded sample, is separately gated by its test
ODR and checked ready before the XYZ burst: 20 ms at the 52 Hz accelerometer
setting and 5 ms at the 208 Hz gyroscope setting, each rounded up. Reading the
high output byte clears the corresponding latched ready flag, so a previously
observed flag cannot justify the next sample.

As a deliberate library extension, the managed X/Y/Z user offsets are cleared
during the test so their asymmetric add/add/subtract behavior cannot alter the
vendor thresholds. The exact prior offset registers and all other managed state
are restored and read back afterward.

The operation records primary test status separately from restoration status.
If a readiness or transport failure occurs while stimulus may be active, the
driver makes one bounded pass through the same sensor-power-down then
self-test-disable order before restoring the profile. A cleanup-write failure
does not loop. Any partial test write invalidates prior configuration provenance
until full profile reapply and readback succeeds. Cancellation remains
bus-silent and therefore reports unknown provenance instead of performing this
I2C cleanup.

The public hard callback ceiling is `16 * (samples + 1) + 87`: each of four
phases reserves three readiness checks plus one burst for its discard and
averaged samples, normal fixed test work uses 20 callbacks, one additional
callback is reserved for a failed terminal write followed by cleanup, and exact
profile restoration reserves 66.

## Calibration Is Software Bias Estimation, Not A Chip Procedure

Neither the datasheet nor AN5130 defines this library's bias-calibration
operation. `startCalibration()` is application-level estimation with explicit
fixture assumptions:

- Gyroscope bias uses stationary averaging in dps.
- Accelerometer bias uses a caller-supplied expected gravity vector and rejects
  an invalid or unstable fixture.
- Sample count is bounded, every sample has bounded ready checks, and cadence
  is gated by the configured ODR period.
- Results are reported; they are not silently written into hardware offsets or
  applied to future samples.

ST design tip DT0105 describes generic one- and three-point accelerometer
calibration math, but it is not an LSM6DS3TR-C register procedure and does not
override the production API contract.
