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

On Windows, invoke the upload as `.\scripts\pio.cmd run ...`; the wrapper
honors `PLATFORMIO_CORE_DIR` before the default user package store. The bare
`pio` spelling above is for POSIX shells and CI.

If pioarduino 55.03.311 extraction crosses the legacy Win32 path limit, use an
existing PlatformIO Core from a short location for the entire build/HIL shell:

```powershell
$env:PLATFORMIO_CORE_DIR = 'C:\pio'
.\scripts\pio.cmd run -e esp32s3dev -t upload --upload-port COMx
python tools/run_hil.py --port COMx --watchdog-reset
```

The short directory must already contain
`C:\pio\penv\Scripts\pio.exe`; the wrapper intentionally reports a missing
Core instead of installing another one. The HIL runner uses the same variable
to find the matching esptool executable.

The retained ESP32-S3 fixture uses the runner defaults: DTR deasserted and
2 MiB expected PSRAM. For an ESP32-S2 native-TinyUSB fixture with no PSRAM,
assert DTR so CDC traffic is delivered and state the hardware expectation
explicitly:

```powershell
.\scripts\pio.cmd run -e esp32s2dev -t upload --upload-port COMx
python tools/run_hil.py --port COMx --chip esp32s2 --assert-dtr --expected-psram-bytes 0
```

The runner options above change only host endpoint handling and metadata
assertions; they do not weaken any driver, transaction, diagnostic, setter, or
maintenance check in the targeted campaign.

Accelerometer calibration additionally requires a stationary fixture with a
validated `+Z` gravity orientation. When that mechanical reference is not
available, omit only that optional operation and record the coverage limit:

```powershell
python tools/run_hil.py --port COMx --chip esp32s2 --assert-dtr --expected-psram-bytes 0 --skip-accel-calibration
```

The campaign still runs gyroscope calibration, both sensor self-tests, every
sampling mode, stress, diagnostics, recovery, and lifecycle procedure. A pass
with this option does not claim physical accelerometer-calibration coverage.

On Windows, run the upload in a UTF-8 Python console (for example, set
`PYTHONUTF8=1` and `PYTHONIOENCODING=utf-8` in the invoking shell). This keeps
esptool 5 progress and reset output from being decoded through a legacy console
code page.

Enabling Win32 long-path support is the system-wide alternative. The short
Core path above is useful when policy cannot be changed.

The targeted campaign now checks the complete operator surface in deliberate
phases:

1. It proves library, Arduino-ESP32, bundled ESP-IDF, flash, and PSRAM metadata;
   bus-ready/address/frequency diagnostics; complete last-error/mismatch
   records; and `help`, `?`, `version`, and `ver` output.
2. It scans both valid SA0 addresses cooperatively and requires `0x6A` to ACK,
   `0x6B` not to ACK, and the summary to distinguish expected address NACK from
   a bus failure. WHO_AM_I is then proved separately by the driver.
3. It exercises bind/unbind, same-binding no-op, job progress, busy admission,
   bus-silent cancellation, exactly correlated terminal tokens, cached
   last-result inspection, and configuration invalidation after a cancelled
   self-test effect.
4. It changes the owner bus to 100 kHz and back while sampling, rebinds to the
   fixture-absent `0x6B`, proves the explicit transport error and its diagnostic
   timestamp/detail, then restores `0x6A` and proves that rebind reset passive
   diagnostic provenance.
5. It exercises every typed profile field through the device CLI, including
   ODR, full-scale, power, filter, sleep, HPF-mode, offset, and fixed production
   invariants; applies and verifies the complete profile; samples with it;
   checks the coupled 1.6 Hz/low-power rule atomically; and restores/applies the
   default profile.
6. It samples all, acceleration, angular rate, and temperature in both
   ready-checked and direct modes, checking terminal timestamps/budgets,
   nonzero transactions, validity/freshness/quality, raw and full-scale
   provenance, monotonic sequence, exact raw-to-fixed-unit conversion, and the
   documented temperature operating range.
7. It runs exact cooperative `stress` counts in ready and direct modes, an
   eight-operation `stress_mix` rotation with two probes, two reconciliations,
   and four samples, requires successful physical-transaction deltas, then
   cancels a long stress session and proves cancellation is not counted as a
   failure.
8. It performs controlled register reads/block reads, injects a one-bit managed
   register mismatch through the restricted raw-write path, proves
   configuration invalidation and sample gating, checks exact mismatch
   register/expected/observed evidence, and restores the profile through the
   normal configure job.
9. It requires successful default self-test, both default-argument and custom
   16-sample calibration forms, FIFO purge, power-down, configure, reset, boot,
   recovery, and reconciliation. `--skip-accel-calibration` omits only the two
   orientation-dependent accelerometer calibration forms and records that
   limitation in the JSON summary. Failed primary maintenance results never
   count as coverage.
10. It finishes with a strict invalid-input matrix covering every new grammar
    family, requires an error status where one is emitted, verifies that
    rejected profile edits are atomic and that the matrix performs no I2C, and
    requires zero final driver transport failures.

Every driver terminal record must remain within its published transaction
ceiling. Raw serial output and a JSON summary are written outside the repository
unless explicit paths are supplied.

## One-Hour Owner Soak

The long campaign runs its invariant checks on the device and emits one compact
progress record every 30 seconds. This avoids using high-volume native-USB CLI
traffic as a proxy for sensor or transport reliability.

```sh
pio run -e esp32s3hil -t upload --upload-port COMx
python tools/run_owner_soak.py --port COMx --expected-seconds 3600
```

On Windows, invoke the upload through `.\scripts\pio.cmd` as described above.

The firmware uses a fixed-memory owner loop and grants one transport callback
per `poll()`. At 100 ms intervals it cycles all eight sample quantity/readiness
combinations and checks exact token/kind correlation, terminal success,
transaction bounds, validity/freshness masks, monotonic sequence, stable
configuration generation, exact raw-to-fixed-unit conversion, and temperature
range. It also retains observed acceleration/angular-rate maxima as telemetry,
performs an explicit probe plus configuration reconciliation every five
minutes, and requires zero operation, contract, and transport failures.

The host monitor rejects missing/non-monotonic progress, an early terminal
record, insufficient samples, any reported failure, or a non-pass result. For
the one-hour run it also requires all 11 scheduled maintenance cycles, paired
probe/reconcile counts, at least one successful callback per sample, and
nonzero gravitational acceleration evidence. For
ESP32-S3 native USB, DTR and RTS are set before opening the port; do not replace
this with a monitor that momentarily asserts the boot straps.

## Retained Physical Evidence

Only the most recent campaign is retained here, and the `Tested source` row
below is the exact revision it validates. Older
per-release run logs are not evidence for today's code; their results are
summarized per version in
[CHANGELOG.md](https://github.com/janhavelka/LSM6DS3TR/blob/main/CHANGELOG.md)
and their full text remains in Git history. Do not append a new block per run -
replace this one.

### Retained: ESP32-S2 expanded campaign at `1419ea2`

| Item | Value |
| --- | --- |
| Date | 2026-08-05, 14:15:27-14:17:16 UTC |
| Fixture | ESP32-S2 native-TinyUSB, 4 MB flash, no PSRAM |
| Sensor | address `0x6A`, SDA GPIO 8, SCL GPIO 9, 400 kHz, 50 ms callback timeout |
| Runtime | Arduino-ESP32 3.3.11, bundled ESP-IDF 5.5.5 |
| Tested source | [`1419ea2`](https://github.com/janhavelka/LSM6DS3TR/commit/1419ea2a50b56e04875cf4a7268661ffc8f01165) |

`1419ea2` precedes the v2.1.0 release tag (`08ae295`); it is not the release
commit. The driver core and example ESP32 HAL transport have changed since
this campaign (see `[Unreleased]` in
[CHANGELOG.md](https://github.com/janhavelka/LSM6DS3TR/blob/main/CHANGELOG.md)).
No physical campaign has been run against the current `main`.

The campaign completed every profile, sampling, stress, cancellation,
diagnostic, invalidation, reconciliation, FIFO-purge, power-down, reset, boot,
recovery, and strict invalid-input phase. Both sensor self-tests and both
gyroscope calibration forms passed. Transport counters moved from 70 to 1,634
successful callbacks with zero final failures; all 94 invalid-input checks
passed.

Coverage limits of this run, stated explicitly:

- Both accelerometer calibration forms were skipped because no validated `+Z`
  gravity fixture was available. This run makes no physical
  accelerometer-calibration or mounting-transform claim.
- No one-hour owner soak was run on the ESP32-S2 fixture. The most recent
  passing soak is the ESP32-S3 run recorded in the 2.1.0 changelog entry.
- Contradictory-FIFO and injected self-test-failure branches remain native
  fault-injection checks; a fixture cannot safely force those internal faults.

## Host Firmware Integration Boundary

The library is a device driver, not an application. A host firmware that embeds
it must supply the bus owner, and the compile-time contract below is what that
owner has to satisfy. These are the numbers to check against any candidate
host, not a claim about one particular project:

- transport callbacks receive at most 33 write bytes (including the register
  prefix) and at most 32 read bytes, so the owner's I2C payload capacity must
  meet those two bounds;
- an all-quantity IMU sample yields seven scalar readings, which must fit the
  host's per-device result capacity;
- the driver advances on one callback per owner turn and owns no task, lock,
  retry, health policy, or bus recovery, so it drops into a single-owner
  transport task without inverting control;
- driver and result objects are fixed-size, so a host that forbids steady-state
  allocation can size them statically.

For an integration where the host already has a generic retry/recovery layer,
disable same-operation retry for these callbacks. The owner may recover the bus
only after it has taken the driver's terminal result, then start an explicit
new operation chosen from the reported effect and configuration evidence. This
prevents a bus-level retry from replaying a library state-machine step after an
ambiguous write.

Product-side decisions - device kind/instance, mounting transform, cadence,
calibration persistence, health role, and sample schema - are deliberately out
of scope here. A host must decide them before writing its own driver-owning
module; this repository does not invent them.

## Intentional Physical Limits

The retained fixtures are strapped at `0x6A`; they cannot validate a physical
sensor at alternate address `0x6B` without a hardware change. The campaigns do
not disconnect the sensor, force SDA/SCL low, inject electrical
NACK/timeout/brownout faults, or claim a product mounting transform.
Deterministic software fault behavior,
partial/ambiguous effects, deadlines, cancellation, clock boundaries, and
every transfer-stage failure are covered by the native fault-injection suite.
Electrical fault recovery, alternate-address hardware, mounting/axis signs,
and a real multi-device host load remain separate fixture/product validation
gates.
