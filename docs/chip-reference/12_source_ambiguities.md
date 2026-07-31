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
