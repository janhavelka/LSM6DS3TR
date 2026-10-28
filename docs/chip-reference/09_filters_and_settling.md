# Filters, Settling, And Data Validity

> Source confidence: vendor fact from AN5130 Rev 1. Library status: supported
> only for filter combinations admitted by `DeviceProfile`; excluded paths may
> still be real silicon capabilities.

## Why Settling Is Part Of The Data Contract

An ODR write can succeed before the output is valid. Power-mode, ODR, and
filter changes have transition-specific settling behavior, so a driver must
retain the configured mode and filter provenance. A successful write is not
enough to declare the next sample usable.

This library converts the tables below into one conservative `validAfter` gate
for paths with a documented qualification rule. Gyroscope HPF is an explicit
exception: a register image can be verified without proving its output settled.
The owner supplies monotonic time; the core performs no sleeps and does no I2C
while the time gate is active. Required discard reads are separate bounded
stages, and ready bits establish availability rather than analog stability.

As library scheduling policy, a newly established gate is armed from a fresh
caller-clock sample after the final managed-register readback. The driver
rounds the interval up to milliseconds and adds one clock-quantization tick
when the interval is nonzero. This keeps time spent in synchronous transport
callbacks from shortening the gate, including when the owner changes its
per-poll transaction budget. Configure, reset/boot/recovery replay, self-test
restoration, and reconciliation from untrusted state share this rule.
Read-only reconciliation of a previously `KNOWN` or `SETTLING` profile retains
its original validity timestamp without adding another tick or restarting it.

## Accelerometer Filter Chain

- The analog anti-alias filter is active only in high-performance mode. Its
  bandwidth is normally 400 Hz below 1.666 kHz ODR and 1500 Hz at or above
  1.666 kHz. `CTRL1_XL.BW0_XL=1` forces 400 Hz at high ODR. Source: AN5130
  Table 8, p. 13.
- Digital LPF1 has two outputs. `CTRL1_XL.LPF1_BW_SEL` selects the direct host
  branch; `CTRL8_XL.INPUT_COMPOSITE` independently selects the composite-filter
  input. The branch labeled ODR/2 is ODR/2 in high-performance mode but 740 Hz
  in low-power/normal mode. The ODR/4 branch is ODR/4 in every mode. Source:
  AN5130, Figure 2 and text, p. 13.
- LPF2/high-pass/slope behavior is selected in `CTRL8_XL`. Event engines may
  consume a different filtered path than output registers. Do not assume an
  event threshold sees the same signal as a host burst read. Source: datasheet
  filter block, pp. 32-35.

### Accelerometer Composite Filter Table

For the low-pass output path, AN5130 Table 9 gives the following LPF2 choices.
The counts are worst-case samples discarded for 99% settling, independent of
the separate mode-transition additions below.

| `LPF2_XL_EN` / `HPCF_XL` | Bandwidth | Samples discarded |
|---|---:|---:|
| LPF2 off, LPF1 output ODR/2 or ODR/4 | ODR/2 or ODR/4 | 14 |
| LPF2 on, `00` | ODR/50 | 40 |
| LPF2 on, `01` | ODR/100 | 80 |
| LPF2 on, `10` | ODR/9 | 15 |
| LPF2 on, `11` | ODR/400 | 320 |

The `INPUT_COMPOSITE` cell spans **all four LPF2 rows**: 0 selects the
low-latency input and 1 the low-noise input for every HPCF value. It does not
associate low noise only with `01` or low latency only with `10`. This was
checked visually against the actual AN5130 Table 9 and datasheet Table 73,
pp. 14 and 68 respectively; extraction loses the merged-cell boundaries.

With the high-pass path selected, `HPCF_XL=00` selects the slope path at
ODR/4 with 14 samples, while `01`/`10`/`11` select high-pass cutoffs ODR/100,
ODR/9, and ODR/400 with 80/15/320 samples. Table 9 specifies
`INPUT_COMPOSITE=0` throughout this path.

Reference mode requires `HP_REF_MODE=1`, `HP_SLOPE_XL_EN=1`, and
`HPCF_XL!=00`. It stores the current XYZ vector internally and subtracts it
from subsequent outputs; **both LPF2 and HP are unavailable** in this mode.
The first output after enabling reference mode must be discarded. It is not
an ordinary high-pass stream with an inferred cutoff. Source: AN5130 p. 14;
datasheet Table 72 footnote, p. 67.

### Admitted Register Paths And Dependencies

The bit definitions and counts below are vendor facts. Canonicalization,
qualification ownership and unsupported combinations are library policy.
`L`, `H`, `R` mean `LPF2_XL_EN`, `HP_SLOPE_XL_EN`, `HP_REF_MODE`.

| Host path | L/H/R | HPCF | Direct LPF1 selection | Composite input | Additional contract |
|---|---|---|---|---|---|
| LPF1 | 0/0/0 | `00` unless used by 6D LPF2 | ODR/2 or ODR/4 | canonical 0 unless used by 6D LPF2 | DC-preserving; 14-period gate |
| LPF2 | 1/0/0 | `00`/`01`/`10`/`11` | canonical 0 | 0 or 1 | DC-preserving; 40/80/15/320-period gate |
| Slope | 0/1/0 | `00` | canonical 0 | 0 | Difference of consecutive vectors divided by two; 14-period gate |
| High-pass | 0/1/0 | `01`/`10`/`11` | canonical 0 | 0 | Removes DC; 80/15/320-period gate |
| Reference | 0/1/1 | canonical `01` | canonical 0 | 0 | Relative to captured vector; LPF1 gate and explicit first-output discard; no LPF2 on 6D |

Contradictory host LPF2 and HP/slope selections are rejected, as are nondefault
values of fields that have no admitted consumer. The direct LPF1 selector is
irrelevant to a composite host path and must remain canonical there. The
analog 400 Hz override is admitted only for high-performance ODR at least
1.66 kHz, where it actually changes the default 1500 Hz bandwidth. Low-power
mode is limited to 208 Hz; 1.6 Hz is accelerometer low-power only. Composite
choices retain these mode/ODR restrictions rather than silently upgrading the
requested mode. Powered-down sensors do not produce qualified output.

`LOW_PASS_ON_6D=1` consumes the independent LPF2 branch and therefore requires
an admitted active, routed orientation profile. It may share HPCF with a host
HP/slope path, or use HPCF and composite input while the host uses direct LPF1.
The profile's accel time gate covers the slower of host output and orientation
LPF2. Reference mode cannot supply this branch. Neither the shared field nor
the common gate means the event engine observes the host output stream.

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

The conservative unknown-starting-state library rule retains the Table 9
maximum for the selected host/event path. Its smallest value, 14 periods,
covers every documented Table 12/13 LPF1 transition: at most 13 basic samples
plus one transition sample, or five for high-performance above-833 Hz changes.
This is a policy envelope, not a claim that every configuration needs fourteen
physical host reads. Filter gates use ceiling-rounded nominal periods and
saturating arithmetic. At 1.6 Hz, LPF2 ODR/400 alone needs 200 seconds; CLI
operation deadlines must be derived from the profile rather than a fixed
five-second assumption.

The elapsed-time gate is valid only with a separate buffered-data policy.
Interpretation-changing replay first requires FIFO Bypass, so old-generation
records are never relabeled. Collection activates only after verified image
and filter settling; the first complete enabled motion vector is consumed
and discarded by the bounded FIFO reader. Reference mode also performs its
required output-register discard using up to three availability checks and
one XYZ burst. After confirmed consumption, a new full output-period gate
starts from fresh owner time, including the quantization tick; only then may
FIFO collection and routes activate. The consumption and next-period gate
do not prove the internally captured reference was stationary or calibrated.
Reading an old queued record after a gate
expires does not qualify that record. Explicit stopping purge remains the
caller-visible destructive boundary for changing a collecting profile.

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

`DeviceProfile` owns all four FTYPE encodings, in `CTRL6_C[1:0]`. FTYPE must
remain canonical `00` while LPF1 is disabled; other values would otherwise
pretend to choose a bandwidth that is bypassed. LPF1 and HPF requests in
low-power/normal mode are rejected because hardware bypasses both there.

Typed gyro HPF configuration and exact readback are supported, but **qualified
gyro HPF sampling remains unresolved**. AN5130 Tables 14-16 explicitly exclude
the high-pass filter; the cutoff table supplies no qualified-settling bound.
Neither an arbitrary timeout, a guessed RC formula, DRDY, nor DRDY_MASK fills
that evidence gap. Only explicit opt-in direct reads may expose the gyro data
with unverified-filter quality. Ordinary ready-checked reads cannot claim a
fresh, settled HPF result, and a qualified accelerometer in a mixed request
cannot qualify the gyroscope. Physical/source qualification is **NOT RUN**.

The concrete state is `ConfigurationState::FILTER_UNQUALIFIED`, preserving
the stronger meaning of `KNOWN`. `SampleRequest::allowUnqualifiedFilter=true`
requires `checkDataReady=false`; output quality is `FILTER_UNVERIFIED`,
`freshMask=0`, and `qualifiedMask` excludes gyroscope. Qualified accel and
temperature requests remain available independently. Gyro FIFO collection is
rejected with HPF, while accel/fourth-only FIFO collection remains admissible.

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

Library policy uses the power-down transition envelope when the previous state
cannot be trusted: 70 ms plus this table in high-performance mode, or 70 ms
plus one period in low-power/normal mode.
That envelope also covers the documented two-sample active-to-high-performance
and one-sample active-to-low-power transitions. Sleep retains the 70 ms base
but is not an active sampling source. Applying this LPF envelope while HPF is
enabled may qualify other channels; it never proves HPF settling.

## BDU, Ready Flags, And Atomicity

`CTRL3_C.BDU=1` protects each individual 16-bit LSB/MSB output pair. It does
not make XYZ, accel+gyro, or motion+temperature one simultaneous sample. Burst
reads minimize skew, but same-cycle vector semantics require a separately
validated DRDY/FIFO policy. Source: AN5130 section 4.4, p. 25.

`CTRL4_C.DRDY_MASK` covers the documented accelerometer LPF1 and gyroscope LPF2
settling paths. Its FIFO invalid markers (`0x7FFF`, `0x7FFE`, `0x7FFD`) apply
when FIFO is active; they are not ordinary raw sensor values. Source: AN5130,
p. 24.

## Orientation Input In The Supported Interrupt Profile

Datasheet Figure 8, p. 34 sends the LPF1 ODR/2 branch directly to the 6D/4D
selector when `LOW_PASS_ON_6D=0`. This is before host composite-filter selection;
enabling the host LPF2 path does not implicitly filter orientation through LPF2.
The default `OrientationProfile` uses that branch. `lowPassOn6d=true` selects
the independent LPF2 output, whose HPCF and composite input are owned by the
same typed filter profile. Orientation result provenance preserves this input
selection rather than inferring it later from a newly staged profile.

AN5130 pp. 38-39 requires two consecutive samples to recognize an orientation
change. The basic engine requires active accel but does not inherit the 26 Hz
minimum of the separate embedded pedometer engine. Route activation follows
complete configuration readback and the existing output-validity gate. That
software gate is not evidence of physical orientation response or missed-pulse
behavior; those qualification cases remain **NOT RUN**.

## Maintenance And Immutable Interpretation

Raw/converted samples, FIFO records and event observations carry the filter
selection and qualification evidence captured by their operation. Conversion
uses that value provenance, never the driver's later desired profile. Host
observation time, stored counter ticks, availability and filter qualification
remain separate evidence.

Gravity and zero-rate calibration cannot consume HP/slope/reference streams
as if they preserved DC. Incompatible calibration requests are rejected before
I/O. Self-test uses the vendor's unfiltered recipe and restores the actual
verified production profile, including filters, FIFO and interrupt fields;
restoration does not substitute an unrelated desired draft. Software bias
estimates have no automatic lifetime across filter changes.
