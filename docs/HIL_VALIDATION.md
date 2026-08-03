# Hardware-In-Loop Validation

This guide owns the repeatable physical validation procedure and retained
validation evidence. Raw serial logs and JSON captures are generated outside
the repository by default; they are evidence, not source files.

Install `pyserial` with `python -m pip install pyserial` before using either
runner. Replace `COMx` below with the port currently assigned to the fixture;
the runners require an explicit port because native USB can re-enumerate under
a different name after flashing or reset.

## Targeted CLI Campaign

Build and upload the maintained owner-safe CLI, then let the runner perform a
stable watchdog reset before opening native USB:

```sh
pio run -e esp32s3dev -t upload --upload-port COMx
python tools/run_hil.py --port COMx --watchdog-reset
```

On Windows, run the upload in a UTF-8 Python console (for example, set
`PYTHONUTF8=1` and `PYTHONIOENCODING=utf-8` in the invoking shell). This keeps
esptool 5 progress and reset output from being decoded through a legacy console
code page.

First-time Windows installs may also require Win32 long-path support. The
platform reports when it is disabled; enable it if extracting the 55.03.311
framework package fails under the legacy path limit.

The campaign covers bind/unbind lifecycle, probe/configure, every combination
of all/acceleration/angular-rate/temperature with ready-checked and direct
sampling, strict invalid input, busy admission, cancellation, token/result
identity, diagnostic reads/writes and provenance invalidation, self-test,
accelerometer and gyroscope calibration, FIFO purge, power-down, reset, boot,
recover, and reconcile. It checks transaction ceilings and finishes with zero
driver transport failures. Self-test and both calibrations must succeed; a
terminal operation with a failed primary result is not accepted as coverage.

## One-Hour Owner Soak

The long campaign runs its invariant checks on the device and emits one compact
progress record every 30 seconds. This avoids using high-volume native-USB CLI
traffic as a proxy for sensor or transport reliability.

```sh
pio run -e esp32s3hil -t upload --upload-port COMx
python tools/run_owner_soak.py --port COMx --expected-seconds 3600
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

## Retained ESP32-S3 Evidence

Both campaigns used the ESP32-S3 revision 0.1 fixture with the LSM6DS3TR-C at
address `0x6A`, WHO_AM_I `0x6A`, SDA GPIO 8, SCL GPIO 9, 400 kHz I2C, and a
50 ms callback timeout.

| Campaign | Tested source | pioarduino | Arduino-ESP32 | Bundled ESP-IDF | PlatformIO Core |
| --- | --- | --- | --- | --- | --- |
| 2026-07-31 platform upgrade | [`94126c8`](https://github.com/janhavelka/LSM6DS3TR/commit/94126c8f6247b68d96e85044b5b7e9fd5493938f) | 55.03.311 | 3.3.11 | 5.5.5 | 6.1.19 |
| 2026-07-22 version 2.0.0 baseline | [`v2.0.0`](https://github.com/janhavelka/LSM6DS3TR/tree/v2.0.0) | 54.03.20 | 3.2.0 | 5.4.1 | 6.1.18 |

### pioarduino 55.03.311 Upgrade

The fixture was supplied on `COM30`. After the new firmware selected its
hardware USB Serial/JTAG identity, Windows assigned `COM26`; esptool reported
the same `64:e8:33:73:a1:54` device. The fixture has 4 MB embedded flash and
2 MB QSPI PSRAM. Runtime metadata proved Arduino-ESP32 3.3.11, ESP-IDF 5.5.5,
and both configured memory sizes before functional testing began.

- Targeted CLI: passed all eight quantity/readiness combinations, 40 strict
  invalid-input cases, and every lifecycle, maintenance, diagnostic,
  cancellation, destructive, and recovery stage. Accelerometer and gyroscope
  self-test passed, both 16-sample calibrations succeeded, and transport
  counters moved from 70 to 1,178 successes with zero failures.
- Post-datasheet-re-audit targeted check: passed from 15:01:57 to 15:02:47 UTC
  on the same USB identity after the self-test cadence/shutdown,
  reset/boot/recovery prerequisite, and FIFO-empty-proof corrections. The
  campaign covered the complete lifecycle and maintenance surface, both
  self-tests and 16-sample calibrations succeeded, all 40 invalid-input checks
  passed, and transport counters moved from 70 to 825 successes with zero
  failures. The requested focused check intentionally did not repeat the
  one-hour soak.
- Final post-hardening targeted check: passed from 06:02:36 to 06:03:26 UTC on
  2026-08-01 after flashing the exact final firmware to the same fixture. This
  rechecked reset, boot, recovery, self-test, calibration, sampling,
  cancellation, diagnostics, and the strict invalid-input matrix after adding
  active-stimulus failure cleanup and bidirectional FIFO count/`FIFO_EMPTY`
  validation. Both self-tests and both 16-sample calibrations succeeded, all
  40 invalid-input checks passed, and transport counters again moved from 70
  to 825 successes with zero failures. The contradictory FIFO and injected
  self-test-failure branches remain native fault-injection checks because the
  fixture cannot safely force those internal status/transport faults. No soak
  was run.
- One-hour owner soak: passed from 09:52:52 to 10:52:54 UTC. At the exact
  3,600,000 ms terminal record the device reported 35,989 samples with matching
  sequence, configuration generation 1, 54,461 successful transport callbacks,
  11 paired probe/reconcile maintenance cycles, and zero operation, contract,
  or transport failures. Observed temperature was 30.136-31.000 degrees
  Celsius; peak absolute acceleration and angular rate were 1,078,785 micro-g
  and 126,078,750 micro-dps, respectively.

### Version 2.0.0 Baseline

- Targeted CLI: passed 40 strict invalid-input cases and all lifecycle,
  sampling, maintenance, diagnostic, cancellation, and recovery stages. The
  driver transport counters moved from 70 to 1,178 successes with zero
  failures. Accelerometer and gyroscope self-test passed and both 16-sample
  calibrations completed successfully.
- One-hour owner soak: passed from 14:44:45 to 15:44:48 UTC. At the exact
  3,600,000 ms terminal record the device reported 35,988 samples with matching
  sequence, configuration generation 1, 54,459 successful transport callbacks,
  11 paired probe/reconcile maintenance cycles, and zero operation, contract,
  or transport failures. Observed temperature was 29.566-30.082 degrees
  Celsius; peak absolute acceleration and angular rate were 971,791 micro-g
  and 2,030,000 micro-dps, respectively.

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
