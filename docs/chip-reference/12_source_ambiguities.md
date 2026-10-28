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
library uses a conservative command-inaccessibility guard for both while also
polling the self-clearing command bit within a fixed ceiling.

**Resolution:** document 50 us as the chip fact and 15 ms as deliberate library
policy. Arm that interval from a fresh caller-clock sample after the command
write and add one millisecond tick so integer clock truncation cannot shorten
the minimum. Never present the larger reset guard or the quantization margin as
an ST reset specification.

## `CTRL3_C.BOOT` Self-Clearing Is Undocumented

Datasheet p. 63 attaches "This bit is automatically cleared" to `SW_RESET`
only. AN5130 p. 49 offers a bit-poll alternative solely in the reset flow; its
reboot flow ends at "Wait 15 ms", and the boot status signal it names is the
`INT1_BOOT` pad, not a register bit. The same page states that `BOOT` does not
modify control-register content, so ST never says whether the `BOOT` bit clears
itself or persists from the host's own write.

**Resolution:** the library polls `CTRL3_C.BOOT` back to zero as its boot
completion criterion, after the post-write guard described above and within a
fixed poll ceiling.
That is a hardware-observed behavior, not a documented ST guarantee: retained
HIL campaigns pass the boot operation on real silicon. Record it as an
observation. Do not restate it as a datasheet fact, and re-validate it on
hardware before relying on it for a different part or revision.

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

## Accelerometer Composite-Input Table Layout

AN5130 Table 9, p. 14, and datasheet Table 73, p. 68, have a merged
`INPUT_COMPOSITE` cell across all four LPF2 HPCF rows. Plain-text extraction
places "1 (low noise)" beside HPCF `01` and "0 (low latency)" beside `10`,
which can incorrectly suggest that only those bandwidth/input pairs exist.

**Resolution:** visually inspected source tables show both inputs apply to
every LPF2 bandwidth. The HP/slope rows separately require input 0. The
maintained filter table, typed validation and wire-image tests follow the
actual cell boundaries. Canonical ignored fields are library policy, not
reserved silicon encodings.

## Accelerometer Reference Is Not An HPF Cutoff

AN5130 p. 14 requires `HP_REF_MODE=1`, `HP_SLOPE_XL_EN=1`, and nonzero HPCF
to enable reference mode, but says both LPF2 and HP filters are then unavailable.
The retained vector is subtracted from later outputs; the first output must
be discarded. Reusing the HPF cutoff name would imply a frequency response
the source does not establish.

**Resolution:** treat reference as a distinct interpretation with canonical
HPCF `01`, no LPF2 orientation input, a bounded actual output discard and the
conservative LPF1 validity gate. Report relative-to-captured-vector data, not
absolute gravity or a documented HPF frequency. The source does not establish
that the captured reference is a calibrated static gravity vector; physical
reference capture/transients remain **NOT RUN**.

## Gyroscope HPF Has No Qualified-Settling Bound

AN5130 p. 16 specifies the HPF cutoffs and its high-performance-only availability.
Its turn-on and discard Tables 14-16, pp. 20-22, explicitly apply with HPF
disabled. Neither those sources nor datasheet `CTRL7_G` defines a bounded
HPF output-qualification rule. A cutoff alone is insufficient evidence.

**Resolution:** support typed HPF register configuration/readback with explicit
unqualified direct-read opt-in and captured filter quality. Preserve the
meaning of settled `KNOWN` and `READY_CHECKED`; an HPF image cannot manufacture
those claims for gyro output. Qualified gyro HPF support remains an unresolved
source/physical gate. No timeout, DRDY bit or guessed RC formula closes it.
HPF and LPF1 requests in low-power/normal mode are rejected rather than
silently accepting bits that hardware bypasses. Hardware evidence is **NOT RUN**.

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

## FIFO Overrun Persistence During Consumption

AN5130 Rev 1 section 8.1.7, p. 79 describes OVER_RUN and says status updates
synchronously with FIFO reads and writes. It does not guarantee a sticky
history bit that survives a consuming read. A cleared post-read OVER_RUN and a
matching pattern therefore cannot by themselves prove that a Continuous burst
was not overwritten between observations; an overwrite by a whole pattern can
also leave the expected pattern index.

**Resolution (engineering inference):** the owner must be the sole FIFO
consumer and must not change interpretation outside the serialized driver.
For each enabled motion or fourth dataset, one trigger stores three words.
Reserve that complete group from the 2048-word capacity: the conservative
occupancy floor after a possible overflow is 2042 words for two motion datasets
or 2039 words when all three admitted datasets are enabled, before host removal.
After a burst consuming `N` words, overflow since the pre-status observation
cannot leave fewer than `2048 - groupWords - N` unread words: concurrent producer writes
cannot reduce occupancy and this driver removed only `N` words. Consequently,
Continuous acquisition rejects post-status occupancy `>= 2048 - groupWords - N` with
`OPERATION_INDETERMINATE` and `continuityLost`, even when OVER_RUN is clear and
the next pattern matches. This deliberately rejects some coherent nearly full
bursts rather than relying on undocumented flag persistence. It is a bounded
occupancy proof under the stated owner/schema contract, not a measured silicon
flag-timing guarantee.

FIFO stop-on-full does not overwrite the buffered words and can return a
coherent batch. Observing FULL_SMART still marks `continuityLost` conservatively
because collection may have stopped and future sensor samples may be absent.
The existing terminal consumed-word count and post-status snapshot preserve
evidence; no successful batch is published for the rejected Continuous case.
Physical overrun/flag-clear behavior and the occupancy boundary remain explicit
HIL qualification cases, **NOT RUN**.

## FIFO ODR Limit Versus Mixed Sensor Rates

Datasheet DocID030071 Rev 3 p. 35 says the internally triggered FIFO ODR must
not exceed either sensor ODR. AN5130 Rev 1 Figure 32 p. 87 instead derives the
trigger as `min(max(ODR_XL, ODR_G), ODR_FIFO)`. Its explicit 208 Hz gyro / 104 Hz
accel example in section 8.5.2 p. 89 uses FIFO 208 Hz with accel decimation 2.

**Resolution:** follow the later, concrete AN5130 trigger and decimator recipe.
Each enabled dataset must have effective trigger cadence divided by decimation
no faster than its active sensor's cadence. Compare register ODR cadence classes
rather than treating rounded labels such as 833/416 as an exact noninteger
ratio; the hardware classes form the source's nominal successive 2x cadence
series. Accelerometer 1.6 Hz is not admitted to this FIFO contract. Gyroscope
sleep is rejected whenever FIFO is enabled, even when its dataset is off,
because the source does not establish the shared trigger contribution of the
sleeping gyro's configured ODR. This is a
conservative production validation policy, not a claim of precise clock phase
or timestamp accuracy; physical cadence qualification remains NOT RUN.

## FIFO Zero Watermark And Reset Status

AN5130 section 8.6 p. 91 allows threshold 0..2047 and defines WaterM by count
reaching or exceeding the threshold. Reset register tables nevertheless show
an initial zero status image.

**Resolution:** watermark zero means the threshold is always reached
mathematically, never that watermark reporting is disabled. Return the observed
hardware flag; do not synthesize WaterM immediately after reset or claim its
first-update timing. Production interrupt routes and STOP_ON_FTH remain off.

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

**Resolution:** the joint timestamp/step fourth dataset requires accelerometer
ODR at least 26 Hz, satisfying the concrete pedometer requirement and the
conservative common dependency. It does not admit gyro-only timestamps or
claim that the broader contradiction is resolved. Physical sensor-dependency
qualification remains **NOT RUN**.

## Pedometer Minimum-Threshold Width

AN5130 p. 51 describes six minimum-threshold bits, while datasheet Table 251
p. 103 defines only `ths_min[4:0]`, with bits 6:5 reserved and bit 7 `PEDO_FS`.

**Resolution:** follow the datasheet field table. The fixed production policy
writes `CONFIG_PEDO_THS_MIN=0x10`, keeping reserved bits zero and selecting the
2 g internal pedometer range. Bank-A writes obey the power-down requirement
on datasheet p. 97 and use bounded readback/main-bank restoration.

## Package Height On The Live Product Page

The datasheet and feature summary specify 2.5 x 3.0 x 0.83 mm, while the live
product-page quality table currently labels the package 2.5 x 3 x 0.86 mm.

**Resolution:** use the original datasheet package drawing and tolerances for
mechanical/PCB work; do not derive a footprint from either rounded web label.

## Timestamp End-Counter Has No Readable Event Flag

Datasheet pp. 86 and 92-93 defines FUNC_SRC1/2 and routes: INT1_TIMER is a route
in MD1_CFG, not a status bit. AN5130 p. 57 describes its near-saturation pad
request and clears it only by writing `0xAA` to TIMESTAMP2, which also resets
the timer. Neither document assigns a readable timestamp-end source flag.

**Resolution:** `EventReadResult::timestampRouted` reports configuration only.
Do not infer a latched pad from raw counter proximity or repurpose an unrelated
FUNC_SRC bit. Acknowledgement uses explicit counter reset with epoch/effect
evidence. Physical pad/timing qualification remains **NOT RUN**.

## Conditional Source Latching And Orientation Direction Bits

AN5130 pp. 39 and 52 makes orientation/step latching conditional on both LIR
and the corresponding source route. Without routing, those source flags remain
pulsed even if LIR is set. The orientation text guarantees clearing the request
on a source read, but does not explicitly guarantee all directional bits clear.

**Resolution:** decode only supported source bits, retain raw bytes and separate
possible/confirmed acknowledgement effects from the decoded cause. Direction
bits do not manufacture a unique orientation when raw flags disagree. No event
read claims simultaneous sources, measured pin level, or recoverability of every
missed nonlatched pulse. The independent fake clears the request rather than
assuming undocumented zero directional bytes; physical behavior is **NOT RUN**.
