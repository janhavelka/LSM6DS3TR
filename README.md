# LSM6DS3TR-C Driver Library

Framework-neutral, fixed-memory LSM6DS3TR-C accelerometer and gyroscope driver
for externally owned I2C buses. Arduino/PlatformIO and native ESP-IDF examples
are provided for ESP32-S2 and ESP32-S3.

Version 2 is an owner-scheduled API. The application owns the bus, pins,
locking, task, monotonic clock, deadlines, scheduling, retries, health policy,
and bus recovery. The library owns only LSM6DS3TR-C protocol and operation
state.

## Contract At A Glance

- `bind()` and `unbind()` perform zero I2C.
- Every production and maintenance hardware procedure is a staged, tokened
  operation. Advanced diagnostics are the explicit one-transaction exception.
- `poll(nowMs, maxTransactions)` never uses more than the caller's transport
  budget. A time-only wait stage uses zero callbacks.
- Every job also has a hard total callback ceiling; the accepted ceiling and
  actual count are returned in the terminal result.
- One hardware operation may be active and one terminal result may be pending.
- `cancelActiveJob()` is bus-silent and publishes a terminal cancelled result.
- `takeResult(token, out)` delivers the matching terminal result exactly once.
- Every operation uses one absolute deadline in the caller's 64-bit monotonic
  uptime domain. Progress never renews it.
- Transport callbacks receive at most 33 write bytes, including the register
  prefix, and at most 32 read bytes. Public constants let fixed-memory owners
  prove capacity at compile time.
- The driver never sleeps, creates a task, retries transport, recovers the bus,
  logs, or allocates dynamically.
- Configuration is trusted only after full readback. Partial or ambiguous
  writes remain observable as unknown or indeterminate state.
- Passive transport counters never block access or make recovery decisions.
- All non-const calls must be serialized. No API is ISR-safe.

The production profile supports polling snapshots. FIFO acquisition and motion
interrupt routing are intentionally not claimed in version 2. A profile with
FIFO or interrupts enabled returns `UNSUPPORTED_PROFILE`. The only FIFO
procedure is an explicitly destructive, bounded purge.

## Installation

### PlatformIO

Add this repository at an exact release tag or commit:

```ini
lib_deps =
  https://github.com/janhavelka/LSM6DS3TR.git#v2.0.1
```

The repository's `platformio.ini` pins pioarduino `platform-espressif32`
55.03.311 (Arduino-ESP32 3.3.11 and ESP-IDF 5.5.5), while CI pins the required
PlatformIO Core 6.1.19. Consuming applications retain control of their own
platform version. Native ESP-IDF component CI remains an independent ESP-IDF
5.4.4 compatibility target.

The ESP32-S3 example and HIL environments describe the validated fixture's
4 MB embedded flash and 2 MB QSPI PSRAM. Override those board settings when
building the repository examples for different ESP32-S3 hardware.

On Windows hosts that still enforce the legacy path limit, keep the existing
PlatformIO Core in a short directory and point the repository wrapper and HIL
tooling at it for the current shell:

```powershell
$env:PLATFORMIO_CORE_DIR = 'C:\pio'
.\scripts\pio.cmd run -e esp32s3dev
```

`C:\pio` must already contain the selected Core at
`C:\pio\penv\Scripts\pio.exe`; the wrapper deliberately never installs a
second Core. This avoids incomplete pioarduino framework/toolchain extraction
without changing the repository's pinned platform.

### ESP-IDF

Use the repository as a component through `EXTRA_COMPONENT_DIRS`, or copy it
into a project's `components` directory. The component manifest supports the
ESP-IDF 5.4 line; CI compiles the native example with exactly ESP-IDF 5.4.4 for
ESP32-S2 and ESP32-S3.

Public headers are under `include/LSM6DS3TR/`. Core code does not include
Arduino or ESP-IDF headers. See the [native ESP-IDF port guide](docs/IDF_PORT.md)
for the component and transport-owner contract.

## Owner-Safe Quick Start

The transport callbacks are synchronous physical attempts. Each must enforce
the supplied timeout and map platform errors to `Status`. They must not retry
or recover the bus.

```cpp
#include "LSM6DS3TR/LSM6DS3TR.h"

LSM6DS3TR::LSM6DS3TR imu;
LSM6DS3TR::OperationToken token{};

LSM6DS3TR::DriverConfig binding{};
binding.i2cWrite = appI2cWrite;
binding.i2cWriteRead = appI2cWriteRead;
binding.i2cUser = &applicationBus;
binding.address = LSM6DS3TR::SensorAddress::SA0_GND;  // 0x6A
binding.i2cTimeoutMs = 20;

LSM6DS3TR::Status status = imu.bind(binding);  // zero I2C
if (!status.ok()) {
  // Reject configuration; no hardware was touched.
}

const uint64_t now = applicationUptimeMs();
LSM6DS3TR::OperationTiming timing{now, now + 5000};
status = imu.startConfigure(LSM6DS3TR::DeviceProfile{}, timing, token);
if (!status.ok() && !status.inProgress()) {
  // Admission failed; token is not usable.
}
```

Advance the accepted operation only from the bus owner:

```cpp
if (imu.operationActive()) {
  const auto progress = imu.poll(applicationUptimeMs(), 1);
  // progress.transactionsUsed is 0 or 1 with this budget.
  // progress.transactions/progress.transactionLimit are cumulative.
  // After bounded self-test cleanup, ACTIVE plus a failed progress.status
  // exposes the primary error before a later poll performs restoration I2C.
  // progress.waiting means time or sensor data must advance; do not busy-spin.
}

if (imu.resultPending()) {
  LSM6DS3TR::OperationResult result{};
  status = imu.takeResult(token, result);
  if (status.ok()) {
    // Correlate by token and inspect result.state, result.status,
    // transactions/transactionLimit, hardwareStateMayHaveChanged,
    // and the job-specific result member.
  }
}
```

`startConfigure()` begins with its own WHO_AM_I check, so a profile is never
written before identity is established. Use `startProbe()` when presence and
identity must be checked independently. Both expect WHO_AM_I `0x6A` at the
bound address (`0x6A` or `0x6B`). A successful transfer that returns another
chip ID invalidates any prior verified provenance.

## Operation Lifecycle

An operation starter validates its request, timing, binding, exclusivity, and
pending-result state before hardware access. An accepted operation receives a
nonzero `OperationToken`. The application retains that token until it takes the
terminal result. Tokens are 64-bit values sized for practical non-reuse over a
driver instance's operating lifetime; they are correlation identities, not
durable identifiers across driver recreation or restart.

```text
start -> ACTIVE -> poll in caller-sized chunks -> terminal result -> take once
            |                     |
            +-- cancel ----------> CANCELLED
            +-- deadline --------> TIMED_OUT / INDETERMINATE
```

The terminal states are `SUCCEEDED`, `FAILED`, `CANCELLED`, `TIMED_OUT`, and
`INDETERMINATE`. `hardwareStateMayHaveChanged` distinguishes failures that
occurred after hardware could have been affected. A partially applied or
ambiguous configuration is never presented as verified.

Starter return values describe admission: an accepted operation returns
`IN_PROGRESS`; validation, binding, exclusivity, or pending-result failures
are returned immediately without I2C. `PollResult::status` describes current
progress and is terminal only when its state is terminal. The return value of
`takeResult()` describes result retrieval; after successful retrieval, inspect
the delivered `OperationResult::state` and `OperationResult::status` for the
operation outcome. `Status::msg` always points to static storage, while
`Status::detail` is error-specific diagnostic context.

Do not discard an accepted request merely because an application-side queue
deadline expired. Call `cancelActiveJob()`, then take its matching cancelled
result. `unbind()` is the hard local teardown: it performs no I2C and discards
active and untaken local state.

### Timing And Latency

`OperationTiming` contains the start time and a strictly later absolute
deadline. Every `poll()` call uses the same monotonic 64-bit uptime domain.
The driver does not read a framework clock.

The transaction budget counts callback invocations, not CPU-only state
transitions. A callback is synchronous, so the application must set
`i2cTimeoutMs` to a value acceptable for its owner task. A deadline cannot
preempt synchronous work already in flight. `poll()` receives one `nowMs`
snapshot and may execute its entire owner-granted `maxTransactions` chunk
before the owner supplies a newer time. Terminal deadline observation can
therefore be delayed by owner scheduling plus that complete callback chunk,
with every callback individually bounded by `i2cTimeoutMs`.

Use `maxTransactions = 1` for a shared owner loop unless a maintenance window
explicitly grants a larger chunk. Passing zero advances CPU/time-only state but
performs no transport callback.

`MAX_TRANSPORT_WRITE_BYTES` is 33 bytes: one register-address prefix and up to
32 payload bytes. `MAX_TRANSPORT_READ_BYTES` is 32 bytes. These are hard upper
bounds on buffers presented to the injected callbacks, not recommendations to
allocate dynamically.

Total callback ceilings are part of the public contract:

| Operation | Maximum transport callbacks |
| --- | ---: |
| Probe | 2 |
| Configure | 68 |
| Sample | 66 |
| Reset or boot | 89 |
| Recover | 88 |
| Reconcile | 35 |
| Power-down | 8 worst case; 6 when already in the main bank |
| Self-test | `maximumSelfTestTransactions(samples)` |
| Calibration | `maximumCalibrationTransactions(samples)` |
| FIFO purge | `maximumFifoPurgeTransactions(maxWords)` |

The three calculation helpers return zero for an invalid requested count.
Transaction-limit exhaustion is terminal and never causes a hidden retry.
Each `PollResult` reports callbacks used by that call and cumulatively, the hard
limit, and the current progress status. During self-test restoration after a
primary fault, operation state remains active while progress status preserves
that fault.
Ready-checked sampling permits at most 65 STATUS reads; if readiness is still
absent it fails `DATA_NOT_READY`, preserving the 66th callback for the burst
when readiness succeeds. Reset and boot permit at most 16 command-bit read
polls before `TRANSACTION_LIMIT_EXCEEDED`, preserving 66 callbacks for full
profile reapply/readback. Their ceiling also includes the verified main-bank
clear, identity validation, gyroscope Power-Down, an explicitly active
high-performance accelerometer, and command preparation; recovery instead
includes a main-bank precheck and identity validation before those preparation
writes.

## Operation Classes

| Class | APIs | Bounds and intended use |
| --- | --- | --- |
| Steady state | `startSample`, `startProbe` | Zero-I2C admission, caller deadline, and per-poll transaction cap. Ready-checked samples may wait and poll status; direct samples are labelled unverified. |
| Runtime multi-step | `startConfigure`, `startReset`, `startBoot`, `startRecover`, `startReconcile`, `startPowerDown` | Progress, deadline, cancellation, partial-effect reporting, and configuration readback share one operation model. Reset/boot contain a bus-silent 15 ms device-inaccessible gate rather than a delay. |
| Maintenance/diagnostic | `startSelfTest`, `startCalibration`, `startFifoPurge` | Explicit deadline and caller budget. Self-test averages 5..100 samples per phase; calibration accepts 1..1000 samples; purge discards at most 1..2048 FIFO words. These are not ordinary measurement-loop calls. |
| Advanced single transaction | `diagnosticReadRegister`, `diagnosticReadBlock`, `diagnosticWriteRegister` | Serialized application diagnostics only. Maximum block read is 32 bytes. No call is accepted during a job. Writes are restricted and invalidate configuration provenance. |

The chip has no nonvolatile programming procedure in this library. Calibration
results are returned to the caller; persistence, endurance policy, versioning,
and provenance belong to the application.

## Configuration And Recovery

`DeviceProfile` is the complete replayable production state supported by this
version: accelerometer and gyro ODR/full scale/power/filter settings, BDU,
gyro sleep, user offsets, FIFO-disabled policy, and interrupts-disabled policy.
Low-power/normal mode is limited to 208 Hz for both sensors; 1.6 Hz is valid
only for the accelerometer in low-power/normal mode. Invalid combinations are
rejected before I2C.
Gyro LPF1 requires high-performance mode. Gyro high-pass profiles are not
supported in production because the authoritative settling tables exclude
them; the named `GyroHpfMode` values remain register facts for diagnostics and
future typed support. Accelerometer slope/high-pass output and the
interrupt-specific 6D low-pass path are also not supported production snapshot
settings. User offsets are limited to -127..127 so the reserved -128 encoding
cannot enter the managed image.

Configuration state is explicit:

- `UNCONFIGURED`: no verified profile exists.
- `APPLYING`: a profile is being written or checked.
- `KNOWN`: full supported state was read back and matched.
- `UNKNOWN`: a partial/ambiguous write or mismatch invalidated provenance.
- `SETTLING`: configuration is verified but sensor output is not yet valid.

`configGeneration` increments after a fully verified profile application or a
verified power-down transition, never after a partial or ambiguous effect.
Samples capture the generation and exact accelerometer/gyro full scales from
their known profile. This prevents an old raw sample from being converted with
a new scale. The sample sequence is a 64-bit saturating counter sized to retain
practical ordering for the driver's operating lifetime; it is not persistent
across driver recreation or restart.

`startReconcile()` reads and compares the supported register image without
blindly replaying an ambiguous write. `startRecover()` is a caller-requested
device procedure; it does not recover the I2C bus or own backoff policy.
`DriverDiagnostics` contains passive evidence only.
WHO_AM_I is meaningful only in the main register bank. Probe, configure,
reconcile, recover, and FIFO purge therefore read `FUNC_CFG_ACCESS` first and
fail with unknown configuration if an alternate bank is selected. Configure,
reconcile, and FIFO purge validate identity next, before the first profile
write, the 33 managed-register reads, or FIFO access; recovery validates
identity before any reset/boot preparation write. Reset and boot explicitly
select the main bank, read it back, and validate identity before other register
access. Power-down conditionally performs the same clear/readback when its
precheck finds an alternate bank, then validates identity before writing either
ODR register.

`startPowerDown()` is admitted from any bound, idle configuration state. It
writes exact zero values to `CTRL1_XL` and `CTRL2_G`, reads both back, and on
success marks configuration `UNCONFIGURED`. This proves only that both ODRs are
off; it does not claim the rest of the register image is known. Any existing
desired runtime profile is retained only so an explicit later configure or
recovery operation can replay it.

## Sampling And Conversion

`SampleRequest::quantityMask` selects acceleration, angular rate, temperature,
or a combination. A managed ready-checked sample requires known, settled
configuration and BDU. Result fields carry:

- `validMask`: quantities whose fields are meaningful,
- `freshMask`: quantities proven fresh for this request,
- `SampleQuality`,
- sequence and configuration generation,
- driver read uptime,
- exact full-scale provenance.

Temperature is not silently claimed to be the same physical conversion instant
as motion data. BDU protects multi-byte register pairs; it does not prove that
all axes and sensors share one conversion instant. Products needing strict
same-cycle vectors require a separately validated DRDY/FIFO policy.
Whenever temperature is requested in ready-checked mode, its own `TDA` status
bit is required, including mixed motion-plus-temperature requests.
Between unsuccessful STATUS checks, polling uses a bus-silent gate rounded up
from the slowest requested quantity's conversion cadence. Motion uses its
configured ODR. Per AN5130, temperature uses 12.5 Hz only when the gyro is
powered down and the accelerometer is in low-power/normal mode at 12.5 Hz,
26 Hz for the equivalent 26 Hz case, and 52 Hz otherwise. A sleeping gyro with
a non-power-down ODR can still support temperature; sleep excludes angular-rate
sampling, not temperature sampling. This avoids a fixed busy-poll cadence while
preserving the 65-check ceiling.

Conversion is pure and does not read mutable driver state:

```cpp
LSM6DS3TR::ConvertedSample converted{};
status = LSM6DS3TR::convertSample(result.sample, converted);
```

Converted integer units are micro-g, micro-degrees-per-second, and
milli-degrees Celsius. Additional allocation-free helpers expose ODR period,
settling time, exact sensitivities, raw decoding, calibration validation, and
bias application.

## Self-Test, Calibration, And FIFO Purge

Self-test and calibration are staged so owner scheduling remains in control.
Self-test returns baseline, stimulus, delta, pass flags, primary status, and
restoration status. If restoration cannot be proved, configuration becomes
unknown.

For these maintenance jobs, the caller's absolute deadline is never renewed.
It bounds the scheduling window observed at poll boundaries; it cannot preempt
an owner-granted synchronous callback chunk. Size it from the documented waits,
total callback ceiling, per-callback transport timeout, maximum per-poll budget,
and owner scheduling delay.

The self-test sample count applies independently to the baseline and stimulated
averages for both sensors. Each of those four phases first discards one sample,
as required by AN5130. The operation uses the vendor's 52 Hz / +/-4 g
accelerometer setup and 208 Hz / +/-2000 dps gyroscope setup. Four fixed,
bus-silent settle gates total 400 ms. Every discarded or
collected sample receives at most three STATUS checks and one data read, with a
20 ms accelerometer or 5 ms gyroscope zero-I2C gate after a read and between
readiness checks. Three failed readiness checks produce `DATA_NOT_READY`. If
stimulus may be active, any primary failure first attempts the same bounded
sensor-power-down then self-test-disable sequence used by successful self-test
completion.
The owner-visible primary-failure boundary occurs after that cleanup and before
full-profile restoration. The callback ceiling is
`16 * (samples + 1) + 87`, including one extra failure-cleanup callback.
Self-test temporarily removes the managed user-offset contribution, establishes
the required sensor modes,
and powers down the opposite sensor where required. A verified gyro-sleep
profile is explicitly woken for the gyro phase, and the complete original
profile is restored afterward.

Calibration likewise allows at most three STATUS checks per collected sample.
It uses a zero-I2C gate equal to the configured sensor ODR period, rounded up
to milliseconds, after each non-final data read and between readiness checks.
It finishes after exactly the requested valid sample count and has a ceiling
of `4 * samples`. Neither maintenance procedure retries a failed transfer
or writes nonvolatile memory. Gyroscope calibration rejects a sleeping gyro
before I2C; waking and later restoring it is an explicit self-test procedure,
not hidden calibration policy.

Accelerometer calibration requires an explicit expected gravity vector in
sensor-native axes; the driver never assumes Z-up or a product mounting
orientation. Bias values are finite-checked. The caller decides mounting
transforms and durable calibration policy.

`startFifoPurge()` first verifies the main register bank, live device identity,
and the managed IF_INC/BDU prerequisites, then destroys unread FIFO data. Its
result reports initial unread count/pattern, discarded words, final unread
count, overrun, and truncation. It performs at most `maxWords + 5` callbacks,
retries none, and completes only after the final FIFO status read or a terminal
failure. The unread count must be zero exactly when `FIFO_EMPTY` is asserted;
either contradictory status fails visibly, and asserted `FIFO_EMPTY` always
prevents a destructive read. It is not a FIFO acquisition or waveform API.

## Advanced Diagnostics

Diagnostic register reads do not populate production caches. An accepted raw
write invalidates verified configuration and sample provenance even when the
transport reports success, because arbitrary register changes cannot be
represented as the verified `DeviceProfile`.
Diagnostic writes use a per-register writable mask and reject read-only
registers, reserved bits, bank switching, destructive reset/boot and I2C
disable controls, and invalid ODR/full-scale/FIFO/self-test/offset encodings.
These checks return `INVALID_PARAM` without I2C. Passing a safety check does not
make the write part of the production profile contract.

Keep diagnostic access out of normal sensor adapters. It exists for controlled
bring-up and service work, not application configuration recipes.
Sensor-sync and DRDY-pulse configuration are managed and read back as zero by
the production profile. Accepted diagnostic writes to those registers
invalidate configuration provenance, and reconciliation detects the mismatch.
`CommandTable.h` exposes chip protocol constants for such controlled work; a
constant's presence is not a claim that version 2 provides a typed production
profile for that feature.

## Concurrency, ISR, And Ownership

The driver contains mutable operation state and transport callbacks. It is not
internally locked and is neither copyable nor movable.

- Use it from one owner task, or serialize every non-const call externally.
- Never call it or its transport from an ISR. An ISR may publish a flag or
  timestamp for the owner.
- Do not call raw I2C APIs from other tasks while the owner is advancing a job.
- Keep retry, absence/offline policy, queue expiry, health projection, and bus
  recovery above the driver.

### External Owner Task Integration

Keep the driver object and transport adapter private to the task that owns the
physical I2C backend. A library callback executes synchronously inside that
owner and calls the backend directly; it must not submit another request to the
same owner queue. The application request identity and the library's
`OperationToken` are separate values and must both be retained until the
terminal result is published. One `poll(nowMs, 1)` per owner turn gives the
normal one-backend-transfer scheduling bound.

The reviewed TunnelMonitor-node fit, callback capacities, and remaining
product decisions are retained in the
[HIL validation guide](https://github.com/janhavelka/LSM6DS3TR/blob/main/docs/HIL_VALIDATION.md#tunnelmonitor-node-compatibility-boundary).
This repository does not invent those decisions or add a second bus owner.

## Examples

- [Arduino owner-safe CLI](examples/01_basic_bringup_cli/main.cpp) uses a fixed
  input buffer, application-owned `Wire`, a wrap-extended 64-bit uptime, one
  callback per owner poll, token correlation, cancellation, and exact-once
  result taking.
- [Native ESP-IDF example](examples/idf/basic) uses `app_main`, fixed C
  buffers, `driver/i2c_master.h`, `esp_timer_get_time`, and FreeRTOS yielding.
  It does not use Arduino compatibility code.
- The repository-only [ESP32-S3 owner-soak harness](https://github.com/janhavelka/LSM6DS3TR/blob/main/examples/02_owner_soak/main.cpp)
  runs a low-output, fixed-memory physical campaign with one callback per poll
  and checks all ready/direct quantity combinations on-device.

### Bring-Up CLI

The Arduino and native ESP-IDF examples expose the same fixed-memory operator
surface. The console processes bounded input while the application owner keeps
advancing at most one driver transport callback per `poll()`. Commands that
change the binding, bus, staged profile, or hardware require the application
and driver to be idle. `help`, `version`, `status`, `diag`, and job inspection
remain available as evidence while work is active.

| Command | Contract |
| --- | --- |
| `help` / `?`, `version` / `ver` | Print the exact grammar and library/platform metadata. |
| `status` / `diag` | Print bus readiness, selected and bound address, bus frequency and timeout, binding/job/result state, configuration generation and settling time, complete last transport error with timestamp/age, register mismatch evidence, current poll progress, and staged/desired/verified profiles. No I2C. |
| `job [current\|last]`, `result` | Show the cooperative session counters, active driver token/kind, cumulative and per-poll transaction evidence, wait/status state, and optionally the cached last terminal result. `result` is the `job last` convenience form; it does not call `takeResult()` again. |
| `bind`, `unbind`, `cancel` | Bind/unbind are zero-sensor-I2C lifecycle operations. `cancel` is the normal bus-silent terminal path. `unbind` is an explicit hard teardown that may discard an active or untaken result and clears cached evidence. |
| `scan` | Cooperatively issue one address-only ACK check per owner turn for the two valid SA0 addresses, `0x6A` and `0x6B`. ACK is presence only; `probe` still performs the WHO_AM_I identity proof. |
| `addr [0x6a\|0x6b]` | Show or select the driver address. A changed address replaces the binding without a sensor transfer, resets configuration/diagnostic provenance, and requires a new `probe`/`configure`. |
| `freq [100000\|400000]` | Show or select an owner-controlled I2C rate allowed by the chip. A successful owner-side clock/handle change preserves verified sensor configuration. |
| `probe`, `configure`, `sample [all\|accel\|gyro\|temp] [ready\|direct]` | Start the matching tokened driver job. `configure` applies the staged profile; ready-checked and direct samples retain distinct freshness evidence. |
| `reset`, `boot`, `recover`, `reconcile`, `powerdown` | Start the bounded lifecycle procedure and publish its exact terminal configuration/effect evidence. |
| `selftest [5..100]` | Run both vendor self-test paths with the requested samples per baseline/stimulated phase; the default is 5. The fixture must be stationary. |
| `calxl [samples [x y z [max_p2p_g]]]` | Run 1..1000 accelerometer samples (default 32) against an expected gravity vector (default `0 0 1`, each component -16..16 g, vector magnitude 0.8..1.2 g) and a positive peak-to-peak limit up to 4 g (default 0.08 g). The CLI derives its absolute deadline from sample count and the verified accelerometer ODR. |
| `calg [samples [max_p2p_dps]]` | Run 1..1000 gyroscope samples (default 32) with a positive peak-to-peak limit up to 2000 dps (default 3 dps). The CLI derives its absolute deadline from sample count and the verified gyroscope ODR. |
| `purge <1..2048>`, `rreg <reg>`, `wreg <reg> <value>`, `dump <reg> <1..32>` | Explicit maintenance/diagnostic calls. Purge is destructive. Accepted raw writes invalidate configuration provenance; diagnostics are never normal acquisition policy. |
| `stress [count] [quantity] [mode]` | Cooperatively run an exact 1..10000 operation count (default 100), validating terminal kind/state/status, nonzero bounded transactions, no hardware-change claim, sample masks/freshness, stable configuration generation, monotonic sequence, and conversion. Quantity defaults to `all`; mode defaults to `ready`, and quantity/mode may be supplied while count is omitted. |
| `stress_mix [count]` | Run a deterministic non-destructive rotation of probe, reconcile, ready-checked sample, and direct sample. It reports exact success/failure and transport deltas and accepts `cancel`. |

`profile`, `profile show`, `cfg`, and `settings` print the staged draft plus
the driver's desired and verified copies and whether each matches. `profile
validate` rechecks the whole draft, `profile defaults` stages a fresh default,
and `profile apply` starts the normal identity-checking configure job. Every
`profile set` builds a candidate copy and commits it only when the entire
resulting `DeviceProfile` is valid; a parse, arity, cross-field, or unsupported
value error leaves the draft unchanged.

| `profile set` field | Accepted value |
| --- | --- |
| `xl_odr` | `pd`/`powerdown`, `1.6`, `12.5`, `26`, `52`, `104`, `208`, `416`, `833`, `1660`, `3330`, or `6660` |
| `xl_fs` | `2`, `4`, `8`, or `16` g |
| `xl_power` | `hp`/`high` or `lp`/`low`; 1.6 Hz requires LP and LP is limited to 208 Hz |
| `xl_lpf2` | Boolean `0/1`, `off/on`, or `false/true` |
| `g_odr` | `pd`/`powerdown`, `12.5`, `26`, `52`, `104`, `208`, `416`, `833`, `1660`, `3330`, or `6660` |
| `g_fs` | `125`, `250`, `500`, `1000`, or `2000` dps |
| `g_power` | `hp`/`high` or `lp`/`low`; LP is limited to 208 Hz |
| `g_lpf1`, `g_sleep` | Boolean; LPF1 requires gyro high-performance mode |
| `g_hpf_mode` | `0.016`, `0.065`, `0.260`, or `1.040`; the register mode is staged while production HPF remains disabled |
| `offset_weight` | `1` or `16`, selecting the chip's nominal 1 mg/LSB or 16 mg/LSB modes (0.9765625 or 15.625 mg/LSB exactly) |
| `offset` | Three signed register values, each from -127 through 127 |
| `xl_slope_hp`, `xl_6d_lpf`, `g_hpf`, `fifo`, `interrupts` | Boolean invariant probes; only `false`/`off`/`0` is a valid version 2 production profile |
| `bdu` | Boolean invariant probe; only `true`/`on`/`1` is valid for managed burst reads |

Because every field edit must leave a valid complete draft, coupled changes
may require a safe order. For example, stage 208 Hz before selecting LP, then
stage 1.6 Hz. `profile defaults` is the deterministic escape from an unwanted
draft; it does not touch hardware until `profile apply`.

## Building And Validation

```sh
python scripts/generate_version.py check
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python tools/check_chip_docs_coverage.py
python -m py_compile tools/run_hil.py tools/run_owner_soak.py tools/test_run_hil.py
python tools/test_run_hil.py
python tools/build_docs.py
pio test -e native
pio run -e esp32s3dev
pio run -e esp32s3hil
pio run -e esp32s2dev
pio pkg pack
python tools/check_package_contract.py
```

On Windows, invoke each `pio` command through `.\scripts\pio.cmd`; the wrapper
honors `PLATFORMIO_CORE_DIR` and otherwise uses the current user's default
PlatformIO installation. This keeps package and compiler selection consistent
with the HIL tooling. The short `C:\pio` session example under Installation is
the recommended workaround when Windows path limits prevent pioarduino package
extraction.

CI also compiles the native IDF example for `esp32s2` and `esp32s3` with
ESP-IDF 5.4.4. Hardware-in-loop validation is separate from host and compile
evidence; maintained commands, coverage, and retained physical results are in
the [HIL validation guide](https://github.com/janhavelka/LSM6DS3TR/blob/main/docs/HIL_VALIDATION.md).

### API Documentation

From a repository checkout, run `python tools/build_docs.py` to remove any
stale generated pages and build the local reference under
`docs/doxygen/html/`. The generated tree is intentionally ignored; source
comments, this README, and the checked-in guides remain authoritative. The
<a href="docs/DOCUMENTATION.md">Documentation map</a> distinguishes maintained contracts
from source reference material. The packaged
<a href="docs/chip-reference/00_reference_index.md">LSM6DS3TR-C chip reference</a> is the maintained,
source-audited translation of the datasheet and AN5130 for engineers and AI
coders; it also separates silicon capability from this library's support
contract.

Doxygen is strict for the supported owner-safe API: undocumented public
symbols, missing parameter documentation, broken documentation commands, and
warnings fail generation and CI. `CommandTable.h` is intentionally omitted
from the generated reference because it is a low-level register-fact catalog,
not a typed production feature contract. Use it only with the controlled
diagnostic APIs and the cited vendor documentation.

## Migrating From 1.x

Version 2 is intentionally breaking. Every removed version 1 feature has an
explicit replacement or restriction:

| Version 1 surface | Version 2 replacement or restriction |
| --- | --- |
| `Config`, `begin()`, and `end()` | Use zero-I2C `DriverConfig` plus `bind()`/`unbind()`, then explicitly start probe, configure, or power-down operations. |
| `tick()`, blocking reset/recover/self-test/calibration, and synchronous ODR/full-scale/filter setters | Use the matching `start*`, caller-budgeted `poll()`, and tokened `takeResult()`. Apply runtime settings as one replayable `DeviceProfile`. |
| Synchronous raw motion/temperature reads and cached measurement getters | Use `startSample()` in ready-checked or explicitly direct-unverified mode. The terminal `RawSampleResult` carries validity, freshness, scale, sequence, and configuration provenance. |
| Mutable-state conversion and software bias getters/setters | Use pure `convertSample()` and `applyBias()`. `startCalibration()` returns a candidate bias; the application owns its validation context, persistence, version, and later application. |
| Driver `READY/DEGRADED/OFFLINE` admission and automatic recovery policy | Use passive `DriverDiagnostics` as evidence. The external owner controls absence, health, retry, backoff, and bus recovery. |
| FIFO configuration and drain APIs | Production profiles require FIFO bypass. Only the bounded, destructive purge remains; typed FIFO acquisition needs a future replayable profile and decoder. |
| Interrupt routes, tap/wake/free-fall/6D/tilt/wrist/significant-motion, pedometer and step-counter control/source APIs | Unsupported as version 2 production configuration. Controlled diagnostic reads may aid service work, but production use requires a future typed, fully replayable profile and board electrical contract. |
| Timestamp control/readout and sensor-hub configuration/output APIs | Unsupported as version 2 production configuration. Do not rebuild these features from raw recipes in an application adapter; add a typed, bounded library profile when a current product needs them. |

## Version Metadata

`library.json` is the version source of truth. Run
`python scripts/generate_version.py sync` after changing it. The script owns
`include/LSM6DS3TR/Version.h`, the IDF component version, and Doxygen project
version; CI rejects drift.

License: MIT. See [LICENSE](LICENSE).
