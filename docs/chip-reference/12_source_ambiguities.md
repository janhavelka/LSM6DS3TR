# ST Source Ambiguities And Repository Resolutions

> Source confidence: vendor ambiguity plus an explicit conservative repository
> resolution. Library status is stated per item. These are not excuses to pick
> a convenient interpretation silently.

## `XL_HM_MODE` Register Ownership

The datasheet operating-mode narrative on p. 31 names
`FUNC_CFG_ACCESS.XL_HM_MODE`, but `FUNC_CFG_ACCESS` bit 4 is reserved. The
authoritative register table on p. 66 and AN5130 p. 10 place `XL_HM_MODE` in
`CTRL6_C[4]`.

**Resolution:** use `CTRL6_C[4]`; keep `FUNC_CFG_ACCESS[4]=0`. The library and
`CommandTable.h` already follow this resolution.

## `CTRL9_XL` Reset Value

Datasheet Table 19 (p. 49) reports `CTRL9_XL` reset `0x00`, while Table 75
(p. 68) describes `DEN_X`, `DEN_Y`, and `DEN_Z` defaults as 1, which implies
`0xE0`.

**Resolution:** never rely on this reset state. Production configure writes and
reads back an explicit `0x00`; reset/boot does not regain trust until the full
managed profile is replayed and verified.

## Absolute Wrist-Tilt Mask Default

The embedded-bank map/detail implies `A_WRIST_TILT_MASK=0xC0`, while AN5130's
narrative describes X-positive only, which would be `0x80`. Bits 1:0 are
reserved; `0x02` is never a valid axis mask.

**Resolution:** the feature is unsupported in production. Any future typed
profile must write an explicit mask, preserve reserved bits as zero, and verify
readback instead of inheriting a reset/default assumption.

## 245 dps Versus 250 dps

`CTRL2_G.FS_G=00` is labeled 245 dps in one register table, while the mechanical
and sensitivity tables specify the supported +/-250 dps range with
8.75 mdps/LSB.

**Resolution:** public/API naming is `DPS_250`, and conversion uses
8.75 mdps/LSB.

## 3332/6664 Hz Versus Rounded Names

The mechanical table uses 3332 and 6664 Hz; register and application material
commonly use 3.33 and 6.66 kHz. Public enums are `HZ_3330` and `HZ_6660` to
avoid implying more precision than the nominal ODR. Timing calculations use
the nominal rounded rates documented by the API.

## Temperature Cadence At 1.6 Hz Accelerometer ODR

Datasheet Table 5 footnote 2 (p. 25) says temperature ODR equals accelerometer
ODR in accelerometer low-power mode. AN5130 p. 96 says temperature is 12.5 Hz
for accel 12.5 Hz LP, 26 Hz for accel 26 Hz LP, and 52 Hz for every other accel
configuration. That leaves accel 1.6 Hz LP with gyro off contradictory.

**Resolution:** the library follows AN5130's 52 Hz case for scheduling but
requires `STATUS_REG.TDA` before every temperature-containing ready-checked
read. The ready bit, not the predicted cadence, is the final validity proof.

## Reset Timing Versus Library Guard

AN5130 specifies about 50 us for `SW_RESET` and 15 ms for boot/reboot. The
library uses a conservative 15 ms command-inaccessibility guard for both while
also polling the self-clearing command bit within a fixed ceiling.

**Resolution:** document 50 us as the chip fact and 15 ms as deliberate library
policy; never present the larger guard as an ST reset specification.

## Local Datasheet Page Count

ST's current PDF contains 115 pages. The repository's PDFium-normalized copy
contains numbered technical pages 1..114 and omits only page 115, the generic
ST legal notice.

**Resolution:** cite ST printed page numbers and record both page counts in the
source manifest. No technical register or procedure page is missing locally.

## FIFO Capacity: 4 KiB Versus 4096 Words

The overview and FIFO counters describe a 4 KiB FIFO, but datasheet p. 36 says
it holds 4096 samples of 16 bits, which would require 8 KiB. The threshold and
unread counters are 11-bit fields, and the status procedures describe the full
condition separately from the largest ordinary count.

**Resolution:** treat physical capacity as 4096 bytes = 2048 16-bit words.
`DIFF_FIFO`/threshold values represent at most 2047 ordinary words; interpret
the overrun/full state with `OVER_RUN` and `FIFO_EMPTY`, never from a zero count
alone.

## LPF1 `FTYPE` Register Location

The `CTRL4_C.LPF1_SEL_G` description on datasheet p. 64 points to
`FTYPE[1:0]` in `FUNC_CFG_ACCESS`, where those bits do not define gyro
bandwidth. The field table and bandwidth table place it in `CTRL6_C[1:0]`.

**Resolution:** use `CTRL6_C[1:0]`; keep reserved `FUNC_CFG_ACCESS` bits clear.

## User-Offset Register List Typo

The `CTRL6_C.USR_OFF_W` description lists `X_OFS_USR`, `FUNC_CFG_ACCESS`, and
`Z_OFS_USR`, omitting Y. The dedicated register pages define X/Y/Z at
`0x73`/`0x74`/`0x75`.

**Resolution:** the weight applies to `X_OFS_USR`, `Y_OFS_USR`, and
`Z_OFS_USR`; it does not apply to `FUNC_CFG_ACCESS`.

## I2C Sub-Address Count

Datasheet p. 40 says a repeated START follows "two sub-address bytes," while
the transaction figures, p. 39 description, and 8-bit register map show one
sub-address byte.

**Resolution:** transmit one 8-bit register sub-address. Multi-byte access then
uses `CTRL3_C.IF_INC`.

## `D4D_EN` Polarity

The `TAP_THS_6D.D4D_EN` field description on datasheet p. 89 reverses its
parenthetical values (`0: enabled; 1: disabled`) even though the field name,
4D description, and AN5130 p. 41 say setting it to 1 enables 4D and disables
Z-axis position detection.

**Resolution:** `D4D_EN=1` enables 4D; `0` retains 6D detection.

## `INT2_STEP_DELTA` Footnote Target

The datasheet p. 60 footnote says the delta time is defined in "Soldering
information." The actual field is embedded-bank-A `STEP_COUNT_DELTA` (`0x15`),
with timer prerequisites defined by AN5130.

**Resolution:** use `STEP_COUNT_DELTA`; with `CTRL10_C.TIMER_EN=1` and
`WAKE_UP_DUR.TIMER_HR=0`, one LSB is 1.6384 s.

## FIFO Trigger Wording

The `FIFO_MODE` table on datasheet p. 58 says the mode changes when a trigger
is deasserted. Other datasheet prose describes the current trigger level,
whereas the AN5130 mode procedures describe the first configured trigger event
and a latched mode that persists until Bypass is selected.

**Resolution:** implement the AN5130 event/edge procedures: the first trigger
event latches Continuous-to-FIFO into FIFO or Bypass-to-Continuous into
Continuous; software selects Bypass to reset/rearm. This feature remains
unsupported by the current typed production profile.

## Timestamp Sensor Dependency

AN5130 p. 50 groups timestamp with accelerometer-only embedded functions that
require accel ODR >=26 Hz, while its dedicated timestamp section on p. 57 says
the counter stops only when both accel and gyro are powered down.

**Resolution:** timestamp is unsupported in the production profile. Any future
implementation must validate its chosen sensor dependency on hardware rather
than silently relying on either broad statement.

## Package Height On The Live Product Page

The datasheet and feature summary specify 2.5 x 3.0 x 0.83 mm, while the live
product-page quality table currently labels the package 2.5 x 3 x 0.86 mm.

**Resolution:** use the original datasheet package drawing and tolerances for
mechanical/PCB work; do not derive a footprint from either rounded web label.
