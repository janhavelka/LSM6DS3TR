# FIFO, Interrupts, Timestamp, And Embedded Engines

> Source confidence: vendor fact from the datasheet and AN5130. Library status:
> typed internal-motion FIFO acquisition and bounded destructive purge are
> supported in software; physical qualification is NOT RUN for the new path.
> Temperature and timestamp/step fourth datasets are supported in software.
> External datasets remain unsupported. Typed interrupt routes and 6D/4D source
> observation are supported in software; their physical electrical, pulse and
> orientation qualification is NOT RUN.

## FIFO Data Model

- Capacity is 4 KiB, addressed as up to 2048 16-bit words.
- FIFO data have no type tags. `FIFO_PATTERN[9:0]` in `FIFO_STATUS3/4` identifies
  the next word's position in the configured repeating pattern.
- Data set 1 is gyro XYZ; data set 2 is accel XYZ; data set 3 is
  `SENSORHUB1..6`; data set 4 is `SENSORHUB7..12` or a selected special source.
- `CTRL3_C.IF_INC=1` and `BDU=1` are required for the vendor FIFO procedures.
- Threshold `FTH[10:0]` is in 16-bit words, range 0..2047.
- The first sample acquired when switching into or out of FIFO operation must
  be discarded. Do not treat that transition sample as valid application data.

Source: datasheet pp. 34-38, 54-58, 79-81; AN5130 pp. 74-95.

All four data-set decimators use the same encoding: `000` disables that data
set, `001` stores without decimation, and `010`/`011`/`100`/`101`/`110`/`111`
select factors 2/3/4/8/16/32. The pattern index runs from zero through the last
word in the configured repeating sequence, then restarts. In the first pattern
cycle, enabled data sets are ordered 1, 2, 3, 4; each contributes its configured
words. Source: AN5130, pp. 76-79, 88-89.

## FIFO Modes

| Encoding | Mode | Exact behavior |
|---:|---|---|
| `000` | Bypass | FIFO disabled/cleared. |
| `001` | FIFO | Collect until full, then stop. |
| `011` | Continuous-to-FIFO | Overwrite oldest data before the trigger; on the trigger, transition to FIFO mode, continue filling if space remains, then stop when full. |
| `100` | Bypass-to-continuous | Remain bypassed until the first trigger edge, then continuously overwrite oldest data until software selects Bypass. |
| `110` | Continuous | Collect continuously and overwrite oldest data when full. |

Values `010`, `101`, and `111` are reserved. Source: datasheet p. 58;
AN5130 pp. 80-86.

## Trigger, Status, And Drain Rules

- With the normal internal trigger, effective FIFO cadence is
  `min(max(ODR_XL, ODR_G), ODR_FIFO)`. Sensor-hub end-of-operation or step
  detection can be selected instead. Source: AN5130 Figure 32, p. 87.
- `DIFF_FIFO[10:0]` reports unread 16-bit words. During overrun it can read zero;
  require decoded `DIFF_FIFO=0` exactly when `FIFO_EMPTY=1`. Either disagreement
  is indeterminate, and asserted `FIFO_EMPTY` must prevent a data read even if
  the count field is nonzero.
- `FIFO_FULL_SMART` predicts whether the next complete stored data set will
  make the FIFO full, not necessarily the next individual word.
- Read status and pattern before consuming `FIFO_DATA_OUT_L/H`. Never read data
  while `FIFO_EMPTY=1`, and never change ODR/decimation/FIFO configuration while
  unread data remain. Drain, select Bypass, reconfigure, then select the target
  mode. Source: AN5130 pp. 79, 83, 87-93.
- `ONLY_HIGH_DATA` is valid only with `DEC_FIFO_G != 000` and
  `DEC_FIFO_XL = 000`; it packs accel/gyro high bytes in a special layout.
  Setting the bit alone is not a complete recipe. Source: AN5130 p. 93.

## Supported Motion FIFO Contract

The motion implementation was audited against datasheet DocID030071 Rev 3,
printed pp. 34-38, 54-58 and 79-81, and AN5130 Rev 1 sections 8.1-8.6,
printed pp. 74-92. The source extracts and PDFs remain unchanged. Execution
started from fetched `main` commit
`362b140990bc4ba2bc7f32cef50f8d9f30b4d3cc` on 2026-10-26.

`FifoProfile` admits only Bypass, FIFO stop-on-full, and Continuous with
complete 16-bit internal gyro/accel datasets and an optional fourth dataset.
All eight decimator encodings above are represented. Reserved and triggered
modes, high-byte packing, stop-on-threshold and external datasets remain
unavailable through the production profile. Typed interrupt routes have their
own validated profile ownership.

For internal triggering the FIFO ODR is 12.5..6660 Hz. Enabled motion datasets
require their active sensor; a fourth-only schema still requires its documented
source dependency. Step-event collection does not claim a regular FIFO ODR.
Gyroscope sleep is rejected for every enabled FIFO profile, even with gyro
batching off, because its contribution to the shared trigger is unproven.
Accelerometer 1.6 Hz is also not admitted for FIFO. The
selected effective trigger cadence divided by each dataset's decimation must
not exceed that sensor's cadence. This conservative library constraint avoids
promising distinct vectors where the FIFO would repeatedly collect stale
sensor output. ODR register cadence classes are compared as the nominal 2x
series, not by dividing rounded labels such as 833/416. The datasheet/AN5130
mixed-rate conflict is resolved explicitly in the [ambiguity ledger](12_source_ambiguities.md#fifo-odr-limit-versus-mixed-sensor-rates).
This does not establish sample timestamps or zero phase skew.
Disabled FIFO profiles use the canonical all-disabled register image. An
enabled typed Bypass profile can retain a prepared motion schema without
collecting data.

Pattern generation is bounded by the least common multiple of enabled
factors: at most 96 trigger ticks for factors 3 and 32. At each tick, gyro XYZ
precedes accel XYZ, followed by the selected six-byte fourth dataset, when due.
The largest pattern has 393 words (131 complete records), for factors 1, 3
and 32. The motion-only largest pattern has 105 words; a single enabled
dataset has a three-word pattern.
AN5130 example 1 is `Gx Gy Gz Ax Ay Az`, example 2 is
`Gx Gy Gz Ax Ay Az Gx Gy Gz`. Example 3's motion subset is
`Gx Gy Gz Ax Ay Az Ax Ay Az`; its magnetometer dataset remains unsupported.
`FIFO_PATTERN` identifies a word position, never time or sensor freshness.

AN5130 section 8.6, p. 91 defines watermark as count reaching or exceeding
`FTH[10:0]`. **Watermark zero is always reached; it does not disable the
watermark.** The returned flag is the observed hardware bit; no immediate
post-reset WaterM value or first-update time is synthesized. `STOP_ON_FTH` remains zero, so the threshold does not cap capacity
or halt collection. A typed FIFO interrupt route may expose this observed status;
the watermark bit itself does not enable a route.

AN5130 section 4.7.1, p. 27 and section 8.4, p. 88 explicitly enable automatic
circular rounding when reading `FIFO_DATA_OUT_L/H`: a burst cycles over
`0x3E,0x3F`, not into timestamp registers. The core uses even-sized bursts of
at most 32 bytes and never consumes more words than the coherent nonempty
status observation permits. It never reads data when EMPTY is asserted.
The existing ambiguity policy maps overrun plus raw zero count to 2048 words;
a disagreement between decoded count and EMPTY fails visibly.

Before any interpretation-changing writes, configure/reset/boot/recovery,
self-test and power-down require hardware Bypass. A live non-Bypass mode is
rejected without changing interpretation, even if a status sample could have
observed an empty FIFO. This avoids the arrival race between an empty
observation and a later write. The explicit maintenance request
`FifoPurgeRequest::stopCollection=true` drains at most the requested bound,
then selects and verifies Bypass. It reports truncation and
`residualLossPossible` because arrivals or residual words can be discarded by
the final Bypass transition. It invalidates verified configuration and retains
the desired profile for explicit replay. No implicit reconfiguration flush is
claimed lossless.

The shared configuration engine programs and verifies the complete sensor and
FIFO interpretation while collection is disabled, observes its settling gate,
then writes and verifies the selected active mode. The acquisition owner
rejects the first complete vector from each enabled sensor after activation;
partial XYZ boundaries are also discarded, never published as vectors.
Read-only reconciliation cannot create new FIFO provenance for queued data
from an unknown generation.

`startFifoRead()` returns only complete `FifoMotionRecord` values into a
caller-owned bounded buffer. Each carries its operation token, configuration
generation, kind, immutable interpretation and host read uptime. Motion values
retain full scales; fourth values retain source/counter evidence. Records are usable
only after taking the matching successful result. Admission clears the buffer;
failure/cancellation publishes no successful batch and clears scratch records.
The terminal result retains pre/post status and pattern, confirmed/possible
consumption, discarded words, truncation, overrun and continuity evidence.
A consuming callback failure is never retried; it can remove words even when
no received bytes reach the host. Batches do not synthesize sample timestamps
or freshness by spacing records at the configured ODR.

Continuous acquisition does not assume OVER_RUN is sticky across data reads.
Under the sole-consumer contract, a possible overflow followed by an `N`-word
burst leaves at least `2048 - groupWords - N` unread words, where `groupWords`
reserves three words per enabled dataset. This is `2042 - N` for two motion
datasets and `2039 - N` for both motion datasets plus dataset four. A post-status count at or above
that floor fails `OPERATION_INDETERMINATE`, clears the batch, and marks
`continuityLost` even if OVER_RUN cleared and the pattern matches. This
engineering inference deliberately rejects nearly full Continuous batches that
might otherwise be coherent; it prevents a whole-pattern overwrite from hiding
behind matching pattern arithmetic. See the [overrun persistence resolution](12_source_ambiguities.md#fifo-overrun-persistence-during-consumption).
FIFO stop-on-full can return coherent buffered vectors, but FULL_SMART observed
in either mode conservatively marks global continuity lost. Batch coherence and
stream continuity remain distinct evidence.

`startFifoPurge()` remains destructive maintenance, not another acquisition
owner. Host fake tests establish software contracts only; physical FIFO
rounding, phase, overrun, transition and timing qualification remains **NOT
RUN**. See [physical qualification cases](https://github.com/janhavelka/LSM6DS3TR/blob/main/docs/HIL_VALIDATION.md#motion-fifo-qualification).

## Interrupt Behavior

INT1 and INT2 polarity, drive type, latching, and route selection are separate
concerns. Pulse duration is event-specific; there is no safe universal rule
that every nonlatched interrupt lasts 75 us or one ODR period.

- Pedometer/embedded pin pulses are approximately 75 us.
- Activity/inactivity uses `1/ODR_XL` behavior.
- Free-fall follows the active condition.
- Tap recognition follows its Shock/Quiet/Duration windows.

Interrupt generation uses accelerometer data, so the accelerometer must be
active for free-fall, wake-up, 6D/4D, tap, and activity/inactivity. Source:
AN5130 sections 5 and 6, pp. 32-57.

## Pedometer And Timestamp Dependencies

### Supported fourth-dataset contract

The source audit for prompt 02 uses AN5130 Rev 1 sections 6.1/6.5, 8.3/8.5
and 8.8/8.9, printed pp. 50-57, 87-90 and 94-95, plus datasheet Rev 3
FIFO_CTRL2/4, CTRL10_C, WAKE_UP_DUR and bank-A parameter definitions. The
actual PDF Tables 73 and 74 were visually inspected; their byte positions are
not reconstructed from damaged text extraction.

| Fourth source | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 |
|---|---|---|---|---|---|---|
| Timestamp plus steps, Table 73 p. 94 | timestamp[15:8] | timestamp[23:16] | unused | timestamp[7:0] | steps[7:0] | steps[15:8] |
| Temperature, Table 74 p. 95 | unused | unused | temperature[7:0] | temperature[15:8] | unused | unused |

Each fourth record occupies three FIFO words regardless of the number of
meaningful bytes. Temperature is signed 16-bit; unused bytes are ignored.
Timestamp is unsigned 24-bit and steps unsigned 16-bit. `FIFO_PATTERN` remains
a position in one replayed schema, never a record tag. Partial records are
discarded rather than published under the wrong kind.

The typed selector permits exactly one of off, temperature, or timestamp plus
step count. Temperature requires `FIFO_TEMP_EN=1`, `TIMER_PEDO_FIFO_EN=0`, a
nonzero fourth decimator and an active internal sensor. Timestamp/step uses
the fourth decimator with `TIMER_PEDO_FIFO_EN=1` and `FIFO_TEMP_EN=0`; its
joint pedometer dependency requires accelerometer ODR at least 26 Hz.
`MASTER_CONFIG.DATA_VALID_SEL_FIFO` remains zero; sensor-hub triggers are not
supported.

Temperature uses the internal trigger. Timestamp/step supports either that
trigger or step events. `TIMER_PEDO_FIFO_DRDY=1` changes the trigger for all
enabled datasets to `STEP_DETECTED`, including accompanying motion records;
decimators count those events, and no regular ODR or fresh motion conversion
per event is implied. AN5130 p. 94 does not require a FIFO ODR in step-event
mode. No motion dataset needs to be enabled merely to store the fourth source.

Configuration establishes the fixed bank-A pedometer policy with both sensors
proved powered down: `CONFIG_PEDO_THS_MIN=0x10` (2 g internal pedometer range,
minimum threshold 16), `PEDO_DEB_REG=0x6E` (six debounce steps and 1040 ms
debounce time), and `STEP_COUNT_DELTA=0`. The source requires power-down for
embedded-bank modifications (datasheet p. 97); bounded selection, readback and
main-bank restoration are part of the same configure engine. Partial bank
selection/restoration does not establish known main-bank state.

Timer resolution is established before enabling `TIMER_EN`; changing it
requires the effectful `0xAA` timestamp reset. Every configure/replay first
reads actual hardware resolution, including when the target disables the
counter profile: a 25 us to default/off transition still requires that reset.
Ordinary reads, reconcile and same-resolution configuration do not reset counters. Explicit
`startCounterReset` resets timestamp, steps or both in a known timestamp/step
Bypass profile; `PEDO_RST_STEP` is explicitly cleared by software. Reset bits
and advancing counters are not immutable equality-readback values.
Read-only reconcile can retain bank-A policy evidence from a prior verified
profile, but rejects unverified timestamp/step state: main-bank equality alone
cannot establish the fixed embedded-bank parameters without a bank write.

Records retain raw ticks, configured 25 us or 6.4 ms resolution, raw steps,
profile generation and counter epoch. Epoch and continuity evidence separate
source/reconfiguration/reset/loss boundaries; they do not expand wraps into a
monotonic clock. No total steps, missed-wrap count, sensor-to-host clock map,
or simultaneous accel/gyro sampling is inferred. Host admission, read and
terminal uptime remain separate observations. An ordinary raw wrap is not
proof of a reset or the number of elapsed wraps.

Temperature records describe buffered converted-register values, not proof of
a new temperature conversion per FIFO trigger. AN5130 p. 96 caps conversions
at 52 Hz (12.5/26 Hz for the stated low-power accelerometer cases with gyro
off). Neither matching the conversion cadence nor repeated/different values
establishes snapshot `TDA` freshness for an individual buffered record.

Source changes, bind/unbind, maintenance, raw writes, destructive raw-read
attempts, ambiguous consumption and cancellation invalidate affected continuity.
Unknown-bank diagnostics permit only access-register reads and a main-bank
zero write until main-bank selection is proved; a failed bank restore never
turns an aliased register read into normal evidence. Confirmed consumption and possible reset/bank
effects survive failure; unpublished caller records are cleared. Physical
cadence, pedometer policy, counter wrap and step-event behavior are **NOT RUN**.

- Embedded functions run from a 26 Hz accelerometer-derived engine and require
  an active accelerometer at least at 26 Hz.
- Step count persists across accelerometer Power-Down and pedometer disable.
  `PEDO_RST_STEP` must be cleared by software after use; it is not self-clearing.
  The default six-step debounce means the first counter/interrupt is reported
  on the seventh consecutive detected step, and debounce restarts after about
  one second of inactivity.
- If pedometer debounce is active and `SM_THS` is below `DEB_STEP`, the
  significant-motion threshold is effectively clamped to the debounce value.
- Step-delta timing requires `TIMER_EN=1` and `TIMER_HR=0`; only then is one
  `STEP_COUNT_DELTA` LSB 1.6384 s.
- Relative tilt requires a continuous >35 degree change for 2 s for its first
  event; later events trigger as soon as the device moves >35 degrees from the
  position captured by the preceding event.
- Bank-B absolute-wrist-tilt parameters reset whenever the accelerometer exits
  Power-Down. A custom configuration requires the documented 50 ms `FUNC_EN`
  initialization and bank-B rewrite on every such transition.
- Timestamp is a 24-bit counter. Writing `0xAA` to `TIMESTAMP2_REG` resets it
  and clears the end-counter interrupt. It wraps from `0xFFFFFF` to zero. Select
  6.4 ms or 25 us resolution before enabling it, and reset the counter when
  changing resolution. Source: AN5130 pp. 50-57.

## Sensor Hub Dependencies

- Internal triggering requires the accelerometer active and is limited to
  104 Hz.
- External INT2 triggering still requires at least one internal sensor active.
- The external trigger is high-level active and not polarity-programmable.
- Configure an external sensor with the internal trigger before switching to
  external trigger mode.
- Up to four external sensors may be scheduled. Their data occupy
  `SENSORHUB1..18`; NACK flags are exposed in `FUNC_SRC2`.
- Raw `SLVx_ADD` encodes a 7-bit address in bits 7:1 and the read/write control
  in bit 0. Thus 7-bit address `0x1E` is written as `0x3C` for slave-0 write and
  `0x3D` for read. Do not mix this register image with APIs that accept a
  7-bit address or an already shifted wire byte.
- Slave operations execute sequentially from slave 0 through slave 3 and pack
  at most 18 bytes into `SENSORHUB1_REG..SENSORHUB18_REG`. Only slave 0 supports
  writes and source-conditioned reads. `SLAVE1_CONFIG.write_once` affects the
  slave-0 write and is effective only when `Aux_sens_on != 00`.
- A pulsed sensor-hub end-of-operation/DRDY signal is about 150 us.
- Pass-through cannot be used with external INT2 triggering. With internal
  trigger plus pass-through, INT2 must be tied to ground; when the hub is
  already enabled, follow the documented START_CONFIG, >=5 ms quiesce,
  MASTER_ON, PULL_UP_EN, and PASS_THROUGH_MODE sequence to avoid arbitration.

Source: datasheet sensor-hub registers pp. 69, 76-85, 99-104; AN5130
pp. 58-73.

## Typed Route And Counter Acknowledgement Integration

The existing FIFO/counter profile supplies dependencies for typed watermark,
full, overrun, step-detected, step-overflow and timestamp-end routes. A route
never implicitly enables its source engine. Step-event FIFO collection remains
separate from MCU route selection and from unsupported trigger-switched modes.

`startEventRead` observes FIFO status without draining words and FUNC_SRC1
without resetting counters. Global LIR plus the corresponding route makes
step-source reads acknowledge the latched request. Without that latch/route
combination, source flags are short-lived observations rather than recoverable
history. The result retains each callback's own host time and effect evidence.

Timestamp end-counter has no readable source flag in FUNC_SRC1 or FUNC_SRC2.
AN5130 p. 57 describes a latched INT1 indication near `0xFF0000` at 25 us or
`0xFFFF00` at 6.4 ms. A raw timestamp near those values cannot prove the actual
pad state or recover missed wraps. The explicit timestamp reset writes `0xAA`
to `TIMESTAMP2`, acknowledges that indication and resets the raw timer; its
counter epoch and possible/confirmed effect remain visible. The existing reset
API requires the known timestamp/step Bypass profile. Event reads and reconcile
never perform that acknowledgement implicitly.
