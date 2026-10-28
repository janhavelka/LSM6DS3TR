# Chip Capability And Library Support Matrix

> This page prevents a real silicon feature from being mistaken for a supported
> production API. `CommandTable.h` may name registers that are intentionally not
> owned by a typed profile.

| Chip area | Library status | Production owner / contract |
|---|---|---|
| I2C addresses `0x6A` / `0x6B`, WHO_AM_I | supported | `DriverConfig`, `startProbe`, identity checks in destructive flows |
| Accelerometer/gyro ODR, full scale, admitted modes and filters | supported | replayable `DeviceProfile`, configure/readback/reconcile |
| BDU, IF_INC, little-endian motion/temperature burst reads | supported | managed profile and `startSample` |
| Ready-checked accel, gyro, temperature, and combined snapshots | supported | `SampleRequest`; temperature-containing requests require TDA |
| Conversion to g, dps, and degrees Celsius | supported | allocation-free conversion helpers with profile provenance |
| Software reset, boot, recovery, power-down | supported | bounded tokened operations, absolute deadline, profile restoration/readback |
| Built-in accelerometer and gyro self-test | supported | AN5130 procedure in `startSelfTest`, bounded status checks, explicit restoration evidence |
| Bias estimation/calibration | supported library policy | bounded `startCalibration`; results are explicit and not auto-applied |
| User offset registers | supported | typed profile, range/weight validation, write/readback |
| FIFO status evidence within a bounded destructive purge | supported maintenance operation | `startFifoPurge`; optional explicit stop reports possible residual loss |
| Raw main-bank register/block reads | diagnostic-only | one transaction, no production cache population |
| Safety-filtered raw register write | diagnostic-only | one transaction, invalidates configuration provenance |
| SPI host transport | unsupported | core binding is I2C-only; no SPI callback contract exists |
| Internal-motion FIFO configuration, acquisition, pattern decoding | supported in software; hardware NOT RUN | typed `FifoProfile`, `fifoPatternWord`, bounded `startFifoRead`, caller storage, immutable scale/generation provenance, conservative Continuous occupancy guard (no sticky-overrun assumption) |
| FIFO temperature and timestamp/step fourth datasets | supported in software; hardware NOT RUN | typed mutually exclusive source, three-word schema, raw 24-bit ticks/16-bit steps, resolution, generation and counter epoch; no expanded clock/totals or per-record temperature freshness |
| FIFO internal and step-event collection triggers | supported in software; hardware NOT RUN | step events require timestamp/step source and trigger all enabled datasets; decimation counts events, with no regular motion ODR claim |
| FIFO external sensor datasets | unsupported | no sensor-hub topology or external schema |
| FIFO triggered modes, high-byte packing, stop-on-threshold | unsupported | mode-switch triggers remain unsupported; supported collection triggers do not enable those modes |
| INT1/INT2 electrical and supported source routes | supported in software; hardware NOT RUN | typed shared polarity/drive, motion DRDY mode, global embedded LIR, supported direct pin/source pairs and bounded route masking/activation |
| Bounded source-event observation and acknowledgement evidence | supported in software; hardware NOT RUN | `startEventRead`, separate raw/time/valid observations and effect masks; no simultaneous snapshot or measured pad claim |
| 6D/4D orientation | supported in software; hardware NOT RUN | active accel, exact 80/70/60/50 degree threshold, routed engine and LPF1 ODR/2 input; 6D LPF2 remains unsupported until filter qualification |
| Pedometer fixed policy for timestamp/step FIFO | supported in software; hardware NOT RUN | accelerometer at least 26 Hz; fixed bank-A policy with powered-down write/readback and main-bank restoration; explicit bounded counter reset |
| Significant motion, relative/absolute tilt | unsupported | chip facts retained; no typed production profile |
| Tap, wake-up, free-fall, activity/inactivity | unsupported | chip facts retained; no typed production profile |
| Timestamp/pedometer counter lifetime | supported in software; hardware NOT RUN | raw modular counters with explicit epoch/continuity/reset evidence; ordinary reads/reconcile do not reset; no missed-wrap inference; typed INT1 end-counter route with explicit reset acknowledgement |
| Step-delta batching, boot notification, unsupported event routes | unsupported | typed route policy rejects these sources; raw nonzero route masks are not an escape |
| Sensor hub, pass-through, hard/soft-iron correction | unsupported | no external-sensor topology or bounded scheduler contract |
| DEN and sensor synchronization | unsupported | no product timing/electrical contract |

## Rules For Extending Support

A chip-only feature becomes supported only when one concrete caller needs it
and the change supplies all of the following:

1. A typed, fixed-memory, replayable profile with complete validation.
2. Exact managed writes and full readback, including reserved-bit policy.
3. Bounded staged execution, absolute deadline, and public transaction ceiling.
4. Configuration-provenance rules for partial writes, reset, cancellation, and
   restoration.
5. Native tests for every transfer failure stage and source-backed timing.
6. A repeatable HIL qualification plan; real matching evidence is required before
   claiming physical qualification. Software support alone is labelled explicitly.
7. Updates to this matrix, public Doxygen, README, and changelog.

Raw diagnostic access or an example recipe is not evidence that these
requirements have been met.
