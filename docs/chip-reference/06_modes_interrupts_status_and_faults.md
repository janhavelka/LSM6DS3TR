# Modes, Interrupts, Status, And Faults

> Source confidence: vendor fact. Library status: polling status and bounded
> FIFO acquisition/purge and typed supported interrupt/orientation profiles are
> implemented in software. New physical qualification remains NOT RUN.

## ODR And Power Modes

- Accelerometer and gyroscope ODR fields select both output data rate and operating mode. `0000` powers down the selected sensor. Source: datasheet, pp. 61-62.
- Accelerometer `XL_HM_MODE=1` disables high-performance mode; gyroscope `G_HM_MODE=1` disables high-performance mode. Source: datasheet, pp. 66-67.
- Gyroscope sleep mode is controlled by `CTRL4_C.SLEEP`. Source: datasheet, p. 64.
- After ODR or power-mode changes, discard or mask early samples according to the AN5130 settling guidance; the gyroscope power-down wake path can require about 70 ms before data are usable. Source: AN5130, section 3.2.

## Status Bits

| Register/bit | Meaning | Source |
|---|---|---|
| `STATUS_REG.TDA` | New temperature data available. | Datasheet, p. 72 |
| `STATUS_REG.GDA` | New gyroscope data available. | Datasheet, p. 72 |
| `STATUS_REG.XLDA` | New accelerometer data available. | Datasheet, p. 72 |
| `FIFO_STATUS2.WaterM` | FIFO level reached or exceeded watermark. | Datasheet, p. 80 |
| `FIFO_STATUS2.OVER_RUN` | FIFO filled and data loss/overwrite behavior occurred; interpret with mode and `FIFO_EMPTY`. | Datasheet, p. 80; AN5130, pp. 79, 83 |
| `FIFO_STATUS2.FIFO_EMPTY` | FIFO contains no data when set. | Datasheet, p. 80 |

## Source And Fault Registers

| Register | Role | Source |
|---|---|---|
| `WAKE_UP_SRC`, `TAP_SRC`, `D6D_SRC` | Wake/free-fall, tap, and 6D orientation event sources. | Datasheet, pp. 70-71 |
| `FUNC_SRC1`, `FUNC_SRC2`, `WRIST_TILT_IA` | Embedded-function, sensor-hub, and wrist-tilt event source flags. | Datasheet, pp. 86-87 |
| `SENS_SYNC_SPI_ERROR_CODE` | Sensor-sync SPI error code. | Datasheet, p. 93 |

## Interrupt Routing

### Audited route, dependency and acknowledgement matrix

Prompt 03 source audit (before implementation), 2026-10-27: datasheet
DocID030071 Rev 3 pp. 59-60, 72, 79-80, 86, 88-89, 92-93; AN5130 Rev 1
pp. 24, 33-35, 38-41, 50-52, 57, 96. These are silicon facts; the support
matrix records the implementation status separately.

| Source | Direct INT1 | Direct INT2 | Dependency | Observation and acknowledgement |
|---|---|---|---|---|
| Accel DRDY | INT1_CTRL bit 0 | INT2_CTRL bit 0 | Active accel | STATUS_REG.XLDA; output high-byte reads acknowledge, status reads do not. |
| Gyro DRDY | INT1_CTRL bit 1 | INT2_CTRL bit 1 | Active nonsleeping gyro | STATUS_REG.GDA; output high-byte reads acknowledge. |
| Temperature DRDY | none | INT2_CTRL bit 2 | At least one active sensor | STATUS_REG.TDA; output high-byte read acknowledges. |
| FIFO watermark/overrun/full | INT1_CTRL bits 3/4/5 | INT2_CTRL bits 3/4/5 | Collecting typed FIFO | FIFO_STATUS2 bits 7/6/5; status reads do not consume FIFO. |
| Step detected | INT1_CTRL bit 7 | none | Fixed pedometer profile, accel >=26 Hz | FUNC_SRC1 bit 4; read-clear only with LIR and the corresponding route. |
| Step overflow | none | INT2_CTRL bit 6 | Fixed pedometer profile | FUNC_SRC1 bit 3; same latch/read-clear condition. |
| Timestamp end | MD1_CFG bit 0 | none | Timestamp/step profile | No readable source flag; TIMESTAMP2=0xAA acknowledges and resets timestamp. |
| 6D/4D | MD1_CFG bit 2 | MD2_CFG bit 2 | Active accel and TAP_CFG bit 7 | D6D_SRC bit 6; request read-clear only with LIR and a 6D route. |

Global CTRL3_C polarity/drive apply to both pads. CTRL4_C.INT2_on_INT1
combines INT2 signals onto INT1 without creating missing direct routes.
DRDY_PULSE_CFG_G bit 7 selects 75 us motion DRDY pulses; status flags remain
latched. TAP_CFG.LIR is separate. Nonlatched 6D lasts one accel ODR period;
nonlatched step source flags last 1/26 Hz while step pad pulses are about 75 us.
FUNC_SRC2 has no supported cause and no timestamp-end status bit.

6D thresholds encode 80/70/60/50 degrees in TAP_THS_6D bits 6:5. D4D_EN=1
enables 4D, following the ambiguity ledger. Detection requires two consecutive
samples. With LOW_PASS_ON_6D=0, datasheet Figure 8 p. 34 selects LPF1 ODR/2
before the host composite filter; host LPF2 does not change that source path.
Reading latched D6D_SRC acknowledges its request; zero directional bits after
that read are not guaranteed. A raw source byte does not prove a measured pad
level or a single valid orientation when multiple directional bits are set.

| Register | Routes | Source |
|---|---|---|
| `INT1_CTRL` (`0x0D`) | Step detector, significant motion, FIFO full/overrun/watermark, boot, gyro data-ready, accel data-ready to INT1. | Datasheet, p. 59 |
| `INT2_CTRL` (`0x0E`) | Step delta/count overflow, FIFO full/overrun/watermark, temperature data-ready, gyro data-ready, accel data-ready to INT2. | Datasheet, p. 60 |
| `MD1_CFG` / `MD2_CFG` | Embedded function routes such as inactivity, tap, wakeup, free-fall, 6D, tilt to INT1/INT2. | Datasheet, pp. 91-92 |
| `DRDY_PULSE_CFG_G.DRDY_PULSED` | `0` latched data-ready; `1` pulsed data-ready with 75 us pulses. | Datasheet, p. 59 |

### Supported replay and event-read contract

`InterruptProfile` owns shared pad electrical controls, separate motion DRDY
mode and embedded LIR, typed direct routes and complete 6D/4D policy. Impossible
pin/source pairs and routes without their active sensor/FIFO/counter dependency
are rejected before I2C. The disabled profile remains the all-zero route image.
The production route masks are `INT1_CTRL=0xBB`, `INT2_CTRL=0x7F`,
`MD1_CFG=0x05`, `MD2_CFG=0x04`; unsupported source bits are never admitted.
Raw diagnostics reject nonzero writes to these four route registers and reject
the dedicated wrist-tilt, DEN-DRDY and sensor-hub Master-DRDY route bits.

Configure masks and reads back all four route registers before dependency
writes. It verifies the complete stable image with routes disabled, waits for
settling and activates any FIFO, then writes/reads back the selected routes.
Reconcile stays read-only and never reads event sources as a convenience.
Pre-existing source evidence is not silently discarded by a source read during
configure. Route activation may reassert a previously pending source, so an
asserted pad or post-configure observation does not prove a newly generated
event. Route masking itself can lose event history; terminal evidence records
that uncertainty rather than claiming preservation.

`startEventRead` requests separate STATUS, FIFO_STATUS2, FUNC_SRC1 and D6D_SRC
observations. Each retains its raw byte, host time and validity; the terminal
result retains generation, read mask and possible/confirmed acknowledgement
masks even when a later callback fails. Only a successful operation publishes
an event token. There is no simultaneous-source or measured-pad assertion.
`FUNC_SRC2` is not read. Timestamp routing is configuration evidence only;
acknowledgement is the explicit timestamp-reset operation, with counter epoch
and effect evidence, rather than a fabricated source flag.

Reset, boot, recovery and self-test mask routes before temporary settings and
restore them through the existing configure stages. Power-down verifies all
routes off. Calibration preserves routes but output reads can acknowledge DRDY;
normal samples have the same explicit output-read effect. Event history can be
lost across maintenance or an ambiguous source read and is reported as uncertain.
A FIFO event observation of OVER_RUN, or FULL_SMART in stop-on-full mode,
retains stream/counter continuity uncertainty in the existing owner even if a
later status no longer shows that flag. The observation itself does not drain
FIFO or reset counters. No API is ISR-safe; application ISRs notify the
external owner only.

## FIFO

- FIFO threshold is 11 bits across `FIFO_CTRL1.FTH[7:0]` and `FIFO_CTRL2.FTH[10:8]`; minimum resolution is 1 word = 2 bytes. Source: datasheet, pp. 54-55.
- FIFO can include gyro, accelerometer, third/fourth data sets, and optionally temperature. Source: datasheet, pp. 55-58.
- For proper FIFO status/data reads, the datasheet recommends setting `CTRL3_C.BDU=1`. Source: datasheet, pp. 79-81.
- FIFO data are untagged; use `FIFO_PATTERN` and the configured decimation/order to identify stream position. Do not change FIFO configuration while unread data remain, do not read when `FIFO_EMPTY` is set, and drain fast enough to free slots before overflow. Source: AN5130, FIFO reading procedure.
- The selected temperature or timestamp/step fourth source consumes three words
  per record. Step-event collection triggering applies to all enabled datasets;
  it does not enable the unsupported trigger-switched FIFO operating modes.
  Typed MCU interrupt routes are configured independently. Source: AN5130, pp. 87, 94-95.

## Self-Test

`CTRL5_C` controls gyro and accelerometer self-test. `ST_G=00` is normal mode, `01` positive sign, `11` negative sign, and `10` is not allowed. `ST_XL=00` is normal mode, `01` positive sign, `10` negative sign, and `11` is not allowed. Source: datasheet, pp. 64-65.

## FIFO Modes

| `FIFO_MODE[2:0]` | Behavior | Source |
|---:|---|---|
| `000` | Bypass; FIFO disabled. | Datasheet, p. 58 |
| `001` | FIFO mode; stops collecting data when full. | Datasheet, p. 58 |
| `011` | Continuous-to-FIFO; overwrite before the configured trigger event, then latch into FIFO mode, fill remaining space, and stop full. | Datasheet, p. 58; AN5130, pp. 80-83; [ambiguity ledger](12_source_ambiguities.md) |
| `100` | Bypass-to-continuous; latch into continuous overwrite mode on the first configured trigger event and remain there until Bypass is selected. | Datasheet, p. 58; AN5130, pp. 84-86; [ambiguity ledger](12_source_ambiguities.md) |
| `110` | Continuous; when full, new sample overwrites older sample. | Datasheet, p. 58 |

`FIFO_MODE` values `010`, `101`, and `111` are not assigned in the datasheet table. Source: datasheet, p. 58.
