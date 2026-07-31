# Filters, Settling, And Data Validity

> Source confidence: vendor fact from AN5130 Rev 1. Library status: supported
> only for filter combinations admitted by `DeviceProfile`; excluded paths may
> still be real silicon capabilities.

## Why Settling Is Part Of The Data Contract

An ODR write can succeed before the output is valid. Power-mode, ODR, and
filter changes have transition-specific settling behavior, so a driver must
retain the configured mode and filter provenance. A successful write is not
enough to declare the next sample usable.

This library converts the tables below into one conservative `validAfter` gate.
The owner supplies monotonic time; the core performs no sleeps and does no I2C
while the gate is active. Ready bits are checked only after the gate.

## Accelerometer Filter Chain

- The analog anti-alias filter is active only in high-performance mode. Its
  bandwidth is normally 400 Hz below 1.666 kHz ODR and 1500 Hz at or above
  1.666 kHz. `CTRL1_XL.BW0_XL=1` forces 400 Hz at high ODR. Source: AN5130
  Table 8, p. 13.
- Digital LPF1 is selected by `CTRL1_XL.LPF1_BW_SEL` together with
  `CTRL8_XL.INPUT_COMPOSITE`. The composite path selects ODR/2 or ODR/4.
  Source: AN5130, pp. 13-14.
- LPF2/high-pass/slope behavior is selected in `CTRL8_XL`. Event engines may
  consume a different filtered path than output registers. Do not assume an
  event threshold sees the same signal as a host burst read. Source: datasheet
  filter block, pp. 32-35.

### Accelerometer Samples To Discard

This is AN5130 Table 13 for the combinations the library can reason about.
Transition tables may require one additional discarded sample when moving
between modes; high-performance to high-performance above 833 Hz requires
five samples for some changes. Source: AN5130, pp. 19-20.

| Target mode / ODR | Basic LPF1 path | LPF1_BW_SEL=1 or documented HP/slope path |
|---|---:|---:|
| 1.6/12.5/26/52 Hz low-power | 0 | 1 |
| 104/208 Hz normal | 0 | 1 |
| 12.5/26 Hz high-performance | 0 | 1 |
| 52 Hz high-performance | 1 | 1 |
| 104/208/416/833 Hz high-performance | 1 | 2 |
| 1.666 kHz high-performance | 2 | 2 |
| 3.333 kHz high-performance | 3 | 4 |
| 6.666 kHz high-performance | 13 | 13 |

Power-down entry is approximately 1 us. The table describes samples to reject,
not a permission to spin until they arrive.

## Gyroscope Filter Chain

The gyroscope path comprises the selectable high-pass filter, selectable LPF1,
and a non-configurable LPF2. LPF2 depends on ODR and is bypassed at 6.66 kHz.
Documenting only HPF plus LPF1 is incomplete. Source: AN5130 section 3.8,
pp. 15-18.

- `CTRL7_G.HP_EN_G` and `HPM_G[1:0]` select the high-pass path and its
  0.016/0.065/0.260/1.040 Hz encoding.
- `CTRL4_C.LPF1_SEL_G` enables LPF1; `CTRL6_C.FTYPE[1:0]` selects bandwidth.
- At 416 Hz ODR with LPF1 enabled, AN5130 Table 11 gives overall bandwidths
  138, 131, 121, and 138 Hz for FTYPE `00`, `01`, `10`, and `11` respectively.

The production profile rejects gyro HPF because AN5130's source-backed
turn-on/discard tables explicitly exclude that filter. A raw diagnostic write
does not create sampling-readiness provenance.

### Gyroscope Turn-On And Samples To Discard

Power-down to any active gyro mode has a 70 ms base gate. Sleep keeps the
mass-driving circuitry active and therefore wakes much faster; ST does not say
that sleep preserves a host calibration estimate. Source: AN5130 pp. 12,
20-22.

| Transition | Required action |
|---|---|
| Power-down -> sleep | wait 70 ms |
| Power-down -> low-power/normal | wait 70 ms, discard 1 sample |
| Power-down -> high-performance | wait 70 ms, then use the table below |
| Sleep -> low-power/normal | discard 1 sample |
| Low-power/normal -> high-performance | discard 2 samples |
| High-performance -> low-power/normal | discard 1 sample |
| Active -> power-down | about 1 us if accel also off; about 300 us otherwise |

| Gyro ODR | LPF1 off | LPF1 on FTYPE 00 / 01 / 10 / 11 |
|---:|---:|---:|
| 12.5 Hz | 2 | 2 / 2 / 2 / 2 |
| 26 Hz | 3 | 3 / 3 / 3 / 3 |
| 52 Hz | 3 | 3 / 3 / 3 / 3 |
| 104 Hz | 3 | 4 / 4 / 4 / 4 |
| 208 Hz | 3 | 4 / 4 / 5 / 4 |
| 416 Hz | 3 | 5 / 6 / 6 / 5 |
| 833 Hz | 3 | 7 / 8 / 9 / 6 |
| 1.66 kHz | 135 | 135 / 135 / 135 / 135 |
| 3.33 kHz | 270 | 270 / 270 / 270 / 270 |
| 6.66 kHz | 540 | 540 / 540 / 540 / 540 |

## BDU, Ready Flags, And Atomicity

`CTRL3_C.BDU=1` protects each individual 16-bit LSB/MSB output pair. It does
not make XYZ, accel+gyro, or motion+temperature one simultaneous sample. Burst
reads minimize skew, but same-cycle vector semantics require a separately
validated DRDY/FIFO policy. Source: AN5130 section 4.4, p. 25.

`CTRL4_C.DRDY_MASK` covers the documented accelerometer LPF1 and gyroscope LPF2
settling paths. Its FIFO invalid markers (`0x7FFF`, `0x7FFE`, `0x7FFD`) apply
when FIFO is active; they are not ordinary raw sensor values. Source: AN5130,
p. 24.
