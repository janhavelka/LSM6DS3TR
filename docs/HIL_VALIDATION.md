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


## Motion FIFO Qualification

**Status: NOT RUN.** The motion FIFO software change starts from execution
baseline `362b140990bc4ba2bc7f32cef50f8d9f30b4d3cc`; native behavioral tests
and prior snapshot HIL logs do not validate the new silicon FIFO path. Use the
exact final firmware commit and separately authorized hardware fixture. Record
S2/S3, Arduino/native IDF backend, I2C rate/timeout, address strap, wiring,
part marking, orientation and complete CLI/raw/logic-analyzer evidence.

1. Establish default snapshot bring-up and readback first. Stage
   `profile set fifo continuous 104 1 1 24`, then `profile apply`. Verify with a
   bus trace that FIFO stays in Bypass through complete interpretation readback
   and settling, and that final FIFO mode write/readback occurs last.
2. Capture `fifo 96 16` with an empty queue and after accumulation. Verify
   automatic `0x3E/0x3F` rounding at 32-byte read boundaries, exact destructive
   count reduction, no reads while EMPTY, wrap and every initial pattern
   position. Use stationary gravity and controlled rotation to identify sensor
   axes. Confirm the first complete vector per enabled sensor is discarded.
3. Repeat gyro-only, accel-only, equal rates and AN5130 mixed 208/104 Hz with
   decimation 1/2, then every admitted decimation including 3 and 32 together.
   Verify pattern cycles and cadence with independent capture timing; do not
   infer time from `FIFO_PATTERN` or present host read time as sample time.
4. Exercise watermark 0, 1 and 2047 with routes off. Record actual WaterM update
   behavior, including its first update after reset. Exercise FIFO stop-on-full
   and Continuous overrun; capture FULL, OVER_RUN, EMPTY and 11-bit count at
   boundaries, including zero-count overrun. Check the library's conservative
   ambiguity handling against observed silicon.
   In Continuous mode, arrange overflow between status and data consumption,
   including enough overwrite to return the same pattern index. Record whether
   OVER_RUN clears during the burst; the driver must reject post-read occupancy
   `>= 2042 - N` for an `N`-word burst even with a cleared flag/matching pattern.
   Check counts immediately below and at this conservative boundary and retain
   the trace showing this owner is the sole consumer. Some coherent nearly full
   Continuous batches are deliberately rejected. In FIFO stop-on-full, verify
   a coherent batch remains distinguishable from `continuityLost` after
   FULL_SMART evidence; do not infer uninterrupted collection from valid vectors.
5. Request small word limits/capacities and partial XYZ boundaries while data
   continue arriving. Confirm complete-vector-only publication, bounded work,
   truncation and no hidden continuation/retry. Compare terminal consumed and
   discarded counts with the analyzer trace and observed next pattern.
6. While collection is live, request configure with a changed full scale,
   reset, boot, recover, self-test and power-down. Verify rejection before
   interpretation writes. Run `purge 2048 stop`; verify bounded drain followed
   by Bypass/readback and explicit possible residual loss for arrival races.
   Apply a new profile and prove old-scale data cannot appear under the new
   generation. For self-test, stop and apply a typed Bypass profile first.
7. Inject an authorized physical NACK/timeout or disconnect during FIFO
   consumption, and cancel before/after consumption. Verify no successful batch
   is published, partial data are not reused, effect/possible-consumption
   evidence survives, and the owner can explicitly stop/recover/reconfigure
   before new acquisition. Check another shared-bus device still meets its
   scheduling deadlines under changing poll budgets.
8. Run a sustained collection campaign at intended production cadence and bus
   loading. Account for every reported overrun, truncation, alignment/transition
   discard and uncertain consumption. Retain final commit/build identity and
   logs; qualify any claimed lossless interval only from matching evidence.

Do not extend the snapshot-focused `run_hil.py` pass claim to these FIFO cases.
The manual CLI/analyzer procedure above is a separate physical acceptance gate.

## Fourth Dataset And Counter Qualification

**Status: NOT RUN.** This software task starts from
`bce83376aa01d0e9836f9175a9993e93128956e5`, after motion-FIFO implementation
`0cc5ef2e5f395dbf87c6d18aaf81e49112c73f06` and its closeout. Use only a
separately authorized fixture. Retain exact final commit/build, S2/S3/backend,
address, wiring, full profile and timestamped raw/CLI/analyzer logs. Historical
snapshot or motion-only evidence does not qualify this extension.

1. Configure timestamp/step in Bypass with accelerometer at least 26 Hz. Trace
   both sensors reaching power-down before bank-A selection and parameter
   writes (`0x0F=0x10`, `0x14=0x6E`, `0x15=0`), their readback, and proved
   main-bank restoration. Verify resolution is established before TIMER_EN.
   Deliberately interrupt an authorized bank access; preserve bank uncertainty
   and verify fenced operations cannot treat a bank alias as WHO_AM_I.
2. Observe same-resolution configure and ordinary reconcile/read operations.
   Confirm neither silently resets timestamp nor steps. Explicitly run
   `counterreset timestamp`, `counterreset steps` and `counterreset both` in
   Bypass; verify timestamp `0xAA` command and software set/clear of step reset,
   along with epoch and possible/confirmed effects. Change 6.4 ms/25 us policy,
   then disable the 25 us counter profile; confirm actual-resolution prereads,
   required timestamp reset and discontinuity in both cases. No interrupt routes
   or generic clear-all commands are part of this task.
3. Capture temperature-only and timestamp/step-only FIFO batches, then mixtures
   with one or both motion datasets and every fourth decimator. Check dataset
   order, six-byte layouts from AN5130 Tables 73/74, pattern positions and
   boundaries across 32-byte bursts. Exercise factors 1/3/32 together; do not
   identify payloads by values or assume FIFO_PATTERN is a type tag.
4. Compare buffered temperature with independently observed conversion cadence:
   gyro active (52 Hz), gyro off with accelerometer low-power 12.5/26 Hz, and
   another admitted configuration. Batch faster than conversion and retain
   repeated values without labeling them new/fresh conversions. No temperature
   FIFO record inherits snapshot TDA freshness merely from its trigger.
5. Produce controlled step sessions under the fixed pedometer policy and log
   debounce behavior separately from software decoding. Compare internal versus
   step-event triggering with/without motion datasets. Prove step events trigger
   all enabled datasets and their decimators count events; do not infer regular
   motion sample spacing or simultaneous motion/counter captures.
6. Observe raw timestamp `0xFFFFFF -> 0` and step `0xFFFF -> 0` if practical,
   recording whether a test is actual wrap or explicit reset. At 25 us the full
   timestamp period is 419.4304 s; at 6.4 ms it is 107374.1824 s. Include gaps that
   could hide multiple wraps and verify no expanded clock/total or uptime map
   is invented. Power-down/disable pauses and counter retention are separate
   physical checks; record continuity/epoch changes honestly.
7. Exercise empty/full/overrun, partial fourth records, output exhaustion,
   purge/stop, source changes, destructive raw-read attempts, bind/unbind,
   failed consuming reads and cancellation with
   multiple poll budgets. Confirm terminal consumed-word evidence survives,
   no failed batch publishes scratch records, and counter uncertainty remains
   visible. With all three datasets, test Continuous post-read occupancy just
   below and at `2039 - N`; preserve sole-consumer evidence.
8. Retain a sustained real application load campaign, including repeated source
   changes and explicit maintenance/replay. Qualify only the cadence, counter,
   event and continuity claims demonstrated by matching logs; electrical
   interrupt routing remains a later prompt and is not qualified here.

## Interrupt And Orientation Qualification

**Status: NOT RUN.** Prompt 03 execution starts on 2026-10-27 from synchronized
`5b3d6bc1a6d8fe5831e537ba1bc92cef65dad5d5`, after the motion and fourth-dataset
software acceptance records. Use only a separately authorized fixture. Retain
exact final commit/build identity, S2/S3/backend, sensor address, complete
profile, supply/pull-resistor wiring and raw CLI/logic-analyzer logs. Successful
register readback and historical snapshot campaigns do not qualify these cases.

1. Exercise shared active-high/active-low and push-pull/open-drain policies with
   correctly chosen external pulls. Measure both pads and supply levels. Test
   direct INT1/INT2 routing and INT2-on-INT1 combining; show the real source bits
   remain in their respective route registers. Do not configure unsupported
   direct pin/source pairs or infer electrical levels from an event result.
2. Route accel and gyro DRDY independently and together. Compare latched output
   clearing with 75 us pulsed motion DRDY. Verify STATUS observation alone does
   not acknowledge the output. Route temperature on INT2 and log its actual
   cadence separately. Use the platform ISR hook only to set its reserved
   task-notification bit on the existing owner task. Verify bit ownership and
   zero-timeout consumption, including coalesced edges and a pending busy owner;
   all driver APIs and I2C run from the serialized owner task. No additional
   task, queue or GPIO attachment is implicit in this example.
3. Route FIFO watermark/full/overrun on each supported pad. Exercise threshold
   zero, ordinary thresholds, stop-on-full and Continuous loss. Correlate pad
   transitions with independently read FIFO status and existing acquisition loss
   evidence. Reading event status must not drain FIFO, acknowledge unrelated
   sources or silently reconfigure the collection profile.
4. Under the fixed pedometer profile, exercise step detected on INT1 and step
   overflow on INT2. Compare nonlatched source duration (1/26 Hz) with pad pulse
   duration (about 75 us), and routed/global-LIR read-to-clear behavior. Capture
   several events between reads and under owner loading; qualify only the loss
   behavior actually observed. Raw step wrap is not an expanded total.
5. Route timestamp end-counter on INT1 and observe its near-saturation request
   for both supported resolutions where practical. No event-source register
   contains a timer-end flag. After explicit bounded stop/reconfigure to the
   known timestamp/step Bypass profile, use `counterreset timestamp`; verify
   `TIMESTAMP2=0xAA`, the acknowledgement/reset effect and new counter epoch.
   Do not treat raw threshold proximity as proof of the pad's latched state.
6. Enable 6D/4D with each exact threshold and supported route/latch combination.
   Rotate through positive/negative X/Y/Z positions with independent references.
   Check two-sample recognition, 4D suppression of Z, one-ODR nonlatched requests
   and routed latched acknowledgement. The supported source path is LPF1 ODR/2;
   repeat using the optional 6D LPF2 path and its independent source gate. Preserve raw
   direction bytes after read-clear rather than requiring undocumented zeros.
7. Generate simultaneous causes and events between the four independent source
   callbacks. Check raw/time/valid fields, read mask and confirmed/possible
   acknowledgement effects. Inject an authorized NACK/timeout, cancel or expire
   the deadline around each source read and route phase. No internal retry or
   successful event token may conceal an incomplete or ambiguous observation.
8. Verify route masking before configure, reset/boot/recovery and self-test
   dependencies change, and final route restoration after full readback/settling.
   Check actual verified-profile restoration, power-down routes/ODRs off,
   calibration preserving routes and sample/calibration DRDY acknowledgements.
   Leave a source pending before final route activation and distinguish a
   reasserted old request from a newly generated event. Record event-history
   uncertainty and missed pending pulses honestly.
9. Run the intended application ISR/owner-load campaign on S2 and S3, retaining
   pulse counts, event counts, known losses, deadlines and scheduling data.
   Qualify interrupt loss, electrical behavior and orientation response only from
   matching logs; no current automated software pass claims this physical gate.


## Filter transient and qualification campaign

**Status: NOT RUN.** Software regression and CI do not measure filter transient,
DC response, reference capture or physical orientation response. Use separately
authorized hardware; this implementation does not authorize flashing or resets.

1. Capture board/chip identity, commit, full verified profile, owner poll budgets,
   transport callback durations and host timestamps. For every accel LPF1 path,
   LPF2 HPCF code/input, slope/HP code, reference mode and gyro FTYPE, apply a
   repeatable mechanical step and compare response with AN5130 Tables9/12-16.
   Include mode/ODR changes, 1.6 Hz, 12.5 Hz and maximum ODR. Distinguish vendor
   99% settling from application-specific precision and sensor tolerance.
2. Confirm register readback precedes gate arming even with slow callbacks and
   variable/zero poll budgets. At 1.6 Hz with LPF2 ODR/400 the source count320
   corresponds to 200 seconds before quantization and transport margin. Verify
   both CLIs derive sufficient deadlines without removing bounded cancellation.
3. Reference mode captures an initial vector, potentially during initialization;
   it does not establish a measured stationary/gravity reference. Observe the
   mandatory first XYZ consumption and next-period wait. Replay/reset/self-test
   recaptures hardware reference state; do not treat old and new relative axes as
   sharing a persistent physical baseline.
4. Exercise FIFO stop/purge before every interpretation change. Confirm collection
   stays Bypass through all gates/reference discard, and that the first complete
   vector of each newly enabled sensor is consumed as transition loss. Queue data
   before a change and verify it cannot be relabeled with the new filter generation.
5. Compare default 6D LPF1 ODR/2 against each admitted LPF2 bandwidth/input and
   6D/4D threshold. Host HP/slope and event LPF2 may differ; retain raw source/time
   evidence and measure actual pad response, latch clearing and missed events.
6. For all four gyro HPF cutoffs, use only `sample gyro unqualified` (or explicit
   SampleRequest opt-in). Record raw transients without settled/freshness claims.
   Ordinary ready/direct gyro requests, gyro FIFO and gyro bias calibration must
   reject. Unaffected accel/temperature retain independent qualification.
   A passing experiment alone does not create a universal bounded HPF guarantee:
   qualified gyro HPF remains an unresolved authoritative-source/physical gate.
7. Run unfiltered self-test from admitted filtered profiles and verify exact
   filter/route/FIFO restoration, including restored FILTER_UNQUALIFIED status.
   Verify slope/HP/reference bias calibration rejection; retain calibration
   generation/filter provenance and never automatically reuse an old estimate.

Retain raw traces and procedure outcomes per path. Document deviations rather
than changing source thresholds or claiming the whole matrix from one profile.
