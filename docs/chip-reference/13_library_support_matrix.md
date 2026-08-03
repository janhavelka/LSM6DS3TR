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
| FIFO status evidence within a bounded destructive purge | supported maintenance operation | `startFifoPurge`; no passive typed inspection or decoding/acquisition claim |
| Raw main-bank register/block reads | diagnostic-only | one transaction, no production cache population |
| Safety-filtered raw register write | diagnostic-only | one transaction, invalidates configuration provenance |
| SPI host transport | unsupported | core binding is I2C-only; no SPI callback contract exists |
| FIFO configuration, acquisition, pattern decoding | unsupported | needs a complete replayable profile and typed decoder |
| INT1/INT2 electrical/routing/threshold/event profiles | unsupported | needs electrical, latch, threshold, duration, and route ownership |
| Pedometer, significant motion, relative/absolute tilt | unsupported | chip facts retained; no typed production profile |
| Tap, wake-up, free-fall, activity/inactivity, 4D/6D | unsupported | chip facts retained; no typed production profile |
| Timestamp/pedometer data batching | unsupported | no rollover/routing/FIFO schema contract |
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
6. HIL coverage for the real electrical route and device behavior.
7. Updates to this matrix, public Doxygen, README, and changelog.

Raw diagnostic access or an example recipe is not evidence that these
requirements have been met.
