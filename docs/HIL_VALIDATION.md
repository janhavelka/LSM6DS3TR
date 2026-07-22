# Hardware-In-Loop Validation

This guide owns the repeatable physical validation procedure and retained
release evidence. Raw serial logs and JSON captures are generated outside the
repository by default; they are evidence, not source files.

## Targeted CLI Campaign

Build and upload the maintained owner-safe CLI, then let the runner perform a
stable watchdog reset before opening native USB:

```sh
pio run -e esp32s3dev -t upload --upload-port COM26
python tools/run_hil.py --port COM26 --watchdog-reset
```

The campaign covers bind/unbind lifecycle, probe/configure, every combination
of all/acceleration/angular-rate/temperature with ready-checked and direct
sampling, strict invalid input, busy admission, cancellation, token/result
identity, diagnostic reads/writes and provenance invalidation, self-test,
accelerometer and gyroscope calibration, FIFO purge, power-down, reset, boot,
recover, and reconcile. It checks transaction ceilings and finishes with zero
driver transport failures.

## One-Hour Owner Soak

The long campaign runs its invariant checks on the device and emits one compact
progress record every 30 seconds. This avoids using high-volume native-USB CLI
traffic as a proxy for sensor or transport reliability.

```sh
pio run -e esp32s3hil -t upload --upload-port COM26
python tools/run_owner_soak.py --port COM26 --expected-seconds 3600
```

The firmware uses a fixed-memory owner loop and grants one transport callback
per `poll()`. At 100 ms intervals it cycles all eight sample quantity/readiness
combinations and checks exact token/kind correlation, terminal success,
transaction bounds, validity/freshness masks, monotonic sequence, stable
configuration generation, conversion, and physical range. It also performs an
explicit probe plus configuration reconciliation every five minutes and
requires zero operation, contract, and transport failures.

The host monitor rejects missing/non-monotonic progress, an early terminal
record, insufficient samples, any reported failure, or a non-pass result. For
the one-hour run it also requires all 11 scheduled maintenance cycles, paired
probe/reconcile counts, at least one successful callback per sample, and
nonzero gravitational acceleration evidence. For
ESP32-S3 native USB, DTR and RTS are set before opening the port; do not replace
this with a monitor that momentarily asserts the boot straps.

## Version 2.0.0 Evidence

The 2026-07-22 campaign used an ESP32-S3 revision 0.1 at address `0x6A`, with
WHO_AM_I `0x6A`, SDA GPIO 8, SCL GPIO 9, 400 kHz I2C, and a 50 ms callback
timeout.

- Targeted CLI: passed 40 strict invalid-input cases and all lifecycle,
  sampling, maintenance, diagnostic, cancellation, and recovery stages. The
  driver transport counters moved from 70 to 1,178 successes with zero
  failures. Accelerometer and gyroscope self-test passed and both 16-sample
  calibrations completed successfully.
- One-hour owner soak: passed from 14:44:45 to 15:44:48 UTC. At the exact
  3,600,000 ms terminal record the device reported 35,988 samples with matching
  sequence, configuration generation 1, 54,459 successful transport callbacks,
  11 paired probe/reconcile maintenance cycles, and zero operation, contract,
  or transport failures. Observed temperature was 29.566-30.082 degrees C;
  peak absolute acceleration and angular rate were 971,791 micro-g and
  2,030,000 micro-dps, respectively.

## TunnelMonitor-node Compatibility Boundary

The local integration review compiles the public library header together with
TunnelMonitor-node's authoritative capacity contracts and verifies:

- 33-byte maximum library write and 32-byte maximum read callbacks fit the
  128-byte I2C payload capacity;
- fixed driver and result types fit the reviewed owner/module boundaries;
- all-quantity IMU output needs seven scalar readings, below the 48-reading
  device-result capacity;
- the driver can be advanced with one callback per owner turn and does not own
  tasks, locks, retries, health policy, or bus recovery.

For a future TunnelMonitor-node module, each callback is one physical backend
attempt with its generic same-operation retry and recovery disabled. The owner
may recover the bus only after it has taken the driver's terminal result, then
starts an explicit new operation selected from the reported effect and
configuration evidence. This prevents a bus retry from replaying a library
state-machine step after an ambiguous write.

No TunnelMonitor-node production source is changed by this validation. Its
current contracts do not yet define the IMU device kind/instance, mounting,
cadence, calibration persistence, health role, or sample schema. Those are
product decisions and must precede a concrete owner-private module.

The 2026-07-22 compatibility check used TunnelMonitor-node revision
`292ba6912ce96a93f7ec2d4d0578b8a2f5cc6db2`. The reviewed I2C and device
contract paths were clean; unrelated storage work was present elsewhere in its
working tree and was preserved. The cross-repository compile reported a
1,024-byte driver, 304-byte library operation result, and 264-byte application
device result. TunnelMonitor-node's complete native suite then passed
1,100/1,100 tests without any library-specific application change.

## Intentional Physical Limits

This board is strapped at `0x6A`; it cannot validate the alternate `0x6B`
address without a hardware change. The campaign also does not disconnect the
sensor, force SDA/SCL low, inject electrical NACK/timeout/brownout faults, or
claim a product mounting transform. Deterministic software fault behavior,
partial/ambiguous effects, deadlines, cancellation, clock boundaries, and
every transfer-stage failure are covered by the native fault-injection suite.
Electrical fault recovery, alternate-address hardware, mounting/axis signs,
and a real multi-device TunnelMonitor load remain separate fixture/product
validation gates.
