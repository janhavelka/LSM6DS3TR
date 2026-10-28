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

The production profile supports polling snapshots and bounded typed internal
accelerometer/gyroscope FIFO acquisition with optional temperature or joint
timestamp/step records, typed INT1/INT2 routing and bounded 6D/4D event
observation. FIFO and routes default to disabled. External sensors,
trigger-switched FIFO modes and unrelated event engines remain unsupported.
Software verification is separate from physical qualification; new FIFO,
counter, electrical, pulse, interrupt-loss and orientation cases are **NOT RUN**.

## Installation

### PlatformIO

Add this repository at an exact release tag or commit:

```ini
lib_deps =
  https://github.com/janhavelka/LSM6DS3TR.git#v2.1.0
```

The new FIFO, counter and interrupt APIs documented here are part of unreleased
2.3.0. Pin an exact reviewed implementation commit to use them; no 2.3.0 release
tag has been published.

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
| Configure | 110 worst case, including route masking/activation, reference discard, counter policy and active FIFO |
| Sample | 66 |
| Reset or boot | 131 |
| Recover | 130 |
| Reconcile | 35 |
| Power-down | 17 worst case; 15 when already in the main bank |
| Explicit counter reset | 7 |
| Event source read | 6: bank, identity and at most four separate source callbacks |
| Self-test | `maximumSelfTestTransactions(samples)` |
| Calibration | `maximumCalibrationTransactions(samples)` |
| FIFO acquisition | `maximumFifoReadTransactions(maxWords)` = `2 * maxWords + 3` |
| FIFO purge | `maximumFifoPurgeTransactions(maxWords, stopCollection)` = `maxWords + 5` (+2 for stop) |

The calculation helpers return zero for an invalid requested count.
Transaction-limit exhaustion is terminal and never causes a hidden retry.
Each `PollResult` reports callbacks used by that call and cumulatively, the hard
limit, and the current progress status. During self-test restoration after a
primary fault, operation state remains active while progress status preserves
that fault.
Ready-checked sampling permits at most 65 STATUS reads; if readiness is still
absent it fails `DATA_NOT_READY`, preserving the 66th callback for the burst
when readiness succeeds. Reset and boot permit at most 16 command-bit read
polls before `TRANSACTION_LIMIT_EXCEEDED`, preserving 99 callbacks for full
counter policy/profile reapply/readback and optional FIFO/route activation. Their ceiling also includes the verified main-bank
clear, identity validation, gyroscope Power-Down, an explicitly active
high-performance accelerometer, and command preparation; recovery instead
includes a main-bank precheck and identity validation before those preparation
writes.

## Operation Classes

| Class | APIs | Bounds and intended use |
| --- | --- | --- |
| Steady state | `startSample`, `startProbe`, `startFifoRead`, `startEventRead` | Zero-I2C admission, caller deadline, and per-poll transaction cap. Ready-checked samples may wait and poll status; direct samples are labelled unverified. |
| Runtime multi-step | `startConfigure`, `startReset`, `startBoot`, `startRecover`, `startReconcile`, `startPowerDown` | Progress, deadline, cancellation, partial-effect reporting, and configuration readback share one operation model. Reset/boot contain a bus-silent device-inaccessible gate armed from a fresh post-command clock sample for at least 15 ms, rather than a delay. |
| Maintenance/diagnostic | `startSelfTest`, `startCalibration`, `startFifoPurge`, `startCounterReset` | Explicit deadline and caller budget. Self-test averages 5..100 samples per phase; calibration accepts 1..1000 samples; purge discards at most 1..2048 FIFO words; counter reset explicitly changes the selected raw-counter epoch. |
| Advanced single transaction | `diagnosticReadRegister`, `diagnosticReadBlock`, `diagnosticWriteRegister` | Serialized application diagnostics only. Maximum block read is 32 bytes. No call is accepted during a job. Writes are restricted and invalidate configuration provenance. |

The chip has no nonvolatile programming procedure in this library. Calibration
results are returned to the caller; persistence, endurance policy, versioning,
and provenance belong to the application.

## Configuration And Recovery

`DeviceProfile` is the complete replayable production state supported by this
version: accelerometer and gyro ODR/full scale/power/filter settings, BDU,
gyro sleep, user offsets, typed FIFO/fourth-source/counter policy, and typed interrupt/orientation policy.
Low-power/normal mode is limited to 208 Hz for both sensors; 1.6 Hz is valid
only for the accelerometer in low-power/normal mode. Invalid combinations are
rejected before I2C.
Accelerometer filters explicitly select LPF1 ODR/2 or ODR/4, LPF2
ODR/50, ODR/100, ODR/9 or ODR/400, slope ODR/4, high-pass ODR/100,
ODR/9 or ODR/400, or reference subtraction. LPF2 input selects low latency
or low noise. Forced 400 Hz analog bandwidth is admitted only at active
high-performance ODRs of at least 1.66 kHz; automatic bandwidth remains the
default. The retained LPF2 and HP/slope booleans are mutually exclusive;
validation rejects contradictory selectors and nondefault inactive settings.
Reference mode uses the canonical `highPassSlopeEnabled=true`,
`referenceEnabled=true`, `hpcf=AccelHpcf::DIV_100` image and consumes its first
output vector. The reference result subtracts a captured initial position;
it is not ordinary high-pass output. The orientation-specific LPF2 input
requires a configured 6D/4D owner and cannot accompany reference mode.

Gyro LPF1 admits all four `GyroLpf1Bandwidth` FTYPE choices in high-performance
mode. Gyro HPF admits the four typed `GyroHpfMode` cutoffs only with an active,
awake high-performance gyro, and excludes that gyro's FIFO dataset. Its
register image is replayed and read back, but AN5130 excludes HPF from the
settling tables. Only an explicitly opted-in direct sample may return that
channel, marked `FILTER_UNVERIFIED`; no timeout or DRDY flag qualifies it.
Qualified gyro HPF settling remains an unresolved source/physical gate.
User offsets are limited to -127..127 so the reserved -128 encoding cannot
enter the managed image.

Configuration state is explicit:

- `UNCONFIGURED`: no verified profile exists.
- `APPLYING`: a profile is being written or checked.
- `KNOWN`: the complete image is verified and every active output path has passed its qualification gate.
- `UNKNOWN`: a partial/ambiguous write or mismatch invalidated provenance.
- `SETTLING`: configuration is verified but documented output gates/discards remain pending.
- `FILTER_UNQUALIFIED`: the complete image is verified and documented gates are complete, but gyro HPF output has no source-backed settling guarantee. `getVerifiedProfile()` exposes this verified image; it does not qualify that channel.

New settling gates start from a fresh caller-clock sample after the final
managed-register readback, including reset/boot/recovery and self-test profile
restoration. Each nonzero interval is rounded up to milliseconds and includes
one clock-quantization tick. The owner can supply this fresh time with a
zero-transaction-budget poll. Read-only reconciliation preserves an existing
trusted validity timestamp and generation; untrusted state requires a new conservative gate.
The source tables use the maximum documented transition requirement when the
prior state is unknown. Reference initialization consumes one ready-checked
XYZ vector in bounded stages before publishing the configured state.
Possible and confirmed consumption are reported separately. FIFO collection
starts only after these gates; its initial per-sensor vectors are discarded
by the FIFO owner, so late host reads never qualify pre-transition records.

`configGeneration` increments after a fully verified profile application or a
verified power-down transition, never after a partial or ambiguous effect.
Samples capture the generation, exact accelerometer/gyro full scales, and
`FilterProvenance` containing both filter chains, ODRs and power modes. FIFO
records, event observations and calibration candidates retain the same value
provenance. Conversion copies it without consulting current driver state,
so changing the profile cannot reinterpret an older result. The sample sequence is a 64-bit saturating counter sized to retain
practical ordering for the driver's operating lifetime; it is not persistent
across driver recreation or restart.

`startReconcile()` reads and compares the supported register image without
blindly replaying an ambiguous write. `startRecover()` is a caller-requested
device procedure; it does not recover the I2C bus or own backoff policy.
`DriverDiagnostics` contains passive evidence only.
For timestamp/step profiles, reconcile retains fixed bank-A policy proof from
the prior verified configuration. It rejects an unverified profile rather than
switching banks or claiming to reconstruct that proof from main-bank reads.
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
success marks configuration `UNCONFIGURED`. It first clears and verifies all
four route registers. This proves that the supported routes and both ODRs are
off; it does not claim the rest of the register image is known. Any existing
desired runtime profile is retained only so an explicit later configure or
recovery operation can replay it.

## Interrupt Profiles And Event Evidence

`DeviceProfile::interrupts` owns one replayable policy. Shared `polarity` and
`drive` apply to both pads; `dataReadyMode` chooses latched or 75 us pulsed
motion DRDY. `latchEvents` controls the separate global embedded-event latch.
`int2OnInt1` ORs enabled INT2 signals onto INT1; it does not create unsupported
direct routes. The application owns GPIO configuration, pull resistors,
interrupt attachment, debounce and scheduling.

Each `int1`/`int2` route is typed and validated before I2C:

| Source | Direct INT1 | Direct INT2 | Required profile state |
| --- | --- | --- | --- |
| Accel/gyro DRDY | yes | yes | Corresponding active sensor; gyro must not sleep |
| Temperature DRDY | no | yes | At least one sensor active |
| FIFO watermark/full/overrun | yes | yes | Collecting typed FIFO |
| Step detected | yes | no | Existing fixed timestamp/step policy, accel at least 26 Hz |
| Step overflow | no | yes | Existing fixed timestamp/step policy |
| Timestamp end counter | yes | no | Existing timestamp/step policy |
| 6D/4D | yes | yes | Active accel and complete routed orientation profile |

Orientation selects `SIX_D` or `FOUR_D` and an exact threshold of 80, 70, 60 or
50 degrees. Threshold codes apply at every accelerometer full scale. The
basic engine requires two consecutive samples and active accel; it does not
inherit the pedometer's 26 Hz minimum. With `lowPassOn6d=false`, its input is
the LPF1 ODR/2 branch before the host composite filter. With
`lowPassOn6d=true`, the engine uses LPF2 with the shared `hpcf` and
`compositeInput` choices; the output gate includes this branch even when host
output uses LPF1 or HP/slope. Its selection is captured in event provenance
and does not silently change the host output path.

Configuration clears and verifies all four route registers before changing
dependencies, writes/readbacks the stable image with routes masked, waits for
settling and activates FIFO as required, then enables/verifies the selected
routes. It does not read source registers to discard pre-existing events.
Activating a route may reassert a previously pending source; a new pad edge or
post-configure source observation does not prove a new event was generated.
Read-only reconcile checks stable route/electrical fields without consuming
sources. No configuration readback proves actual board electrical behavior.

`startEventRead(EventReadRequest, timing, token)` selects data-ready status,
FIFO status, embedded step causes and/or orientation causes. The result has
four independent raw-byte/time/valid observations, a configuration generation,
successful-read mask, and possible/confirmed acknowledgement masks. A failure
retains meaningful earlier observations and effect evidence, but publishes no
successful event token. Reads are never retried internally. Separate callbacks
are not a simultaneous snapshot, and source flags are not measured pad levels.

STATUS and FIFO-status observation does not consume output or FIFO words.
Observed FIFO overrun or stop-on-full retains stream/counter continuity
uncertainty even if a later status clears the flag.
Reading a routed, latched step or orientation source acknowledges that source;
nonlatched pulses can disappear before the owner polls. Latching requires both
global LIR and the corresponding route. The orientation result preserves axis
bits without manufacturing one orientation from inconsistent raw flags.
`FUNC_SRC2` has no supported cause and is not read by this operation.

There is no readable timestamp-end flag. `timestampRouted` is a configuration
fact only; event reads do not infer a pending pad or reset the counter. The
explicit `counterreset timestamp` command uses the existing bounded reset API,
requires the known timestamp/step Bypass profile, acknowledges the end-counter
request and starts a new counter epoch. It is not a generic clear-all command.

Sample and calibration output reads can acknowledge DRDY; their effect masks
make that visible while calibration preserves the configured routes. Self-test
masks routes before its temporary profile and restores the actual verified
profile. Reset/boot/recovery mask routes before mode changes and replay them
through the same configure owner. Power-down verifies routes off. Those
lifecycle changes can lose event history, so terminal evidence never promises
that pending pulses or latched requests survived maintenance.

The ESP32 examples provide an unconnected platform ISR hook that only notifies
the existing owner task. They reserve one FreeRTOS task-notification bit and
consume it with a zero-timeout check, retaining a bounded owner-local pending
flag until work can start. Applications must reserve that notification bit for
the IMU hook; no new task or queue is created. Multiple edges can coalesce into
one notification. The owner starts/polls the typed event job outside ISR
context. No GPIO or ISR is attached by default. A notification requests work;
it does not count events, identify a sensor cause or measure a pin level.

## Sampling And Conversion

`SampleRequest::quantityMask` selects acceleration, angular rate, temperature,
or a combination. A managed ready-checked sample requires verified configuration,
BDU and qualified requested channels. An accel/temperature-only request can
remain qualified when an unrequested gyro HPF channel is unqualified. Result
fields carry:

- `validMask`: quantities whose fields are meaningful,
- `freshMask`: quantities proven fresh for this request,
- `qualifiedMask`: filter qualification for each quantity, independent of availability/freshness,
- `SampleQuality`,
- sequence and configuration generation,
- driver read uptime,
- exact full-scale and immutable filter/ODR/power provenance.

`checkDataReady=false` normally selects `DIRECT_UNVERIFIED` freshness while
retaining filter qualification. A request that includes gyro HPF must also
set `allowUnqualifiedFilter=true` and must leave `checkDataReady=false`.
It returns raw data with `FILTER_UNVERIFIED`, `freshMask=0`, and no gyro bit
in `qualifiedMask`, including mixed accel/gyro/temperature requests. Ordinary
ready/direct requests reject that unqualified channel before I2C. No elapsed
time changes this distinction. The opt-in does not weaken `READY_CHECKED`
or the meaning of `KNOWN`.

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
26 Hz for the equivalent 26 Hz case, and 52 Hz otherwise. Each retry gate is
armed from a fresh caller-clock sample after the preceding STATUS callback and
adds one millisecond tick so callback duration and whole-millisecond clock
truncation cannot shorten it. A sleeping gyro with a non-power-down ODR can
still support temperature; sleep excludes angular-rate sampling, not
temperature sampling. This avoids a fixed busy-poll cadence while preserving
the 65-check ceiling.

Conversion is pure and does not read mutable driver state:

```cpp
LSM6DS3TR::ConvertedSample converted{};
status = LSM6DS3TR::convertSample(result.sample, converted);
```

Converted integer units are micro-g, micro-degrees-per-second, and
milli-degrees Celsius. Additional allocation-free helpers expose ODR period,
settling time, exact sensitivities, raw decoding, calibration validation, and
bias application.

## Typed FIFO Acquisition

`DeviceProfile::fifo` selects Bypass, FIFO stop-on-full, or Continuous,
FIFO ODR, separate gyro/accel decimation and a 0..2047-word watermark. The
default motion schema uses internal triggering and full 16-bit XYZ. The selected sensor
must be active. Gyroscope sleep is rejected for every enabled FIFO profile,
even when gyro batching is off, because the shared trigger under sleep is not
established by the source. FIFO trigger cadence divided by each decimation
must not exceed that sensor's cadence. Accelerometer 1.6 Hz is excluded from
FIFO profiles. Decimation factors are off, 1, 2, 3, 4, 8, 16 and 32; the pure
`fifoPatternWord()` helper bounds the untagged pattern at 96 trigger ticks and
393 words with a fourth dataset (105 words for motion only), including factor 3. Watermark zero is always reached and never acts
as a disable switch. See the [source audit](docs/chip-reference/10_fifo_and_embedded_functions.md#supported-motion-fifo-contract).

```cpp
LSM6DS3TR::DeviceProfile profile{};
profile.fifo.enabled = true;
profile.fifo.mode = LSM6DS3TR::FifoMode::CONTINUOUS;
profile.fifo.odr = LSM6DS3TR::Odr::HZ_104;
profile.fifo.gyroDecimation = LSM6DS3TR::FifoDecimation::EVERY_1;
profile.fifo.accelDecimation = LSM6DS3TR::FifoDecimation::EVERY_1;
profile.fifo.watermarkWords = 24;
// Configure, poll to completion, and take the successful configure result.

static LSM6DS3TR::FifoMotionRecord records[16];
LSM6DS3TR::FifoReadRequest request{};
request.records = records;
request.capacity = 16;
request.maxWords = 96;
status = imu.startFifoRead(request, timing, token);
// Keep records alive and untouched through the matching takeResult().
// Use records[0..result.fifoRead.records) only for a matching successful result.
```

The caller supplies capacity 1..682 records and a maximum of 1..2048 consumed
words; null pointers, zero/oversized bounds and unverified profiles are rejected
without I2C. Recompute `timing` from the current owner clock for every operation.
On acceptance the buffer is cleared and becomes scratch storage until terminal
consumption. Failure, timeout and cancellation clear unpublished records and
publish a zero record count. Do not inspect partial scratch data or reuse/free
storage before taking the result (or explicitly unbinding). Retain copied
records if they must outlive buffer reuse; their token and immutable generation
and scales allow interpretation without mutable driver state.

Acquisition snapshots status and the next-word pattern before consuming,
uses automatic FIFO output rounding in bursts of at most 32 bytes, and checks
post-consumption evidence. It never reads while EMPTY is asserted, and count/
EMPTY contradictions fail visibly. It returns complete typed three-word records;
initial partial vectors and the first complete vector of each enabled sensor
after FIFO activation are discarded. It stops at the word/output bound, reports
truncation and residual data, and never extends a drain because new data arrive.

`fifoRead` retains before/after status validity, count, pattern, empty/full/
watermark/overrun flags and host read uptime, confirmed words consumed,
possible consumption on a failed callback, discarded words and continuity
loss. A failed read may have removed data even if the callback delivered no
bytes; the core never retries it. Overrun or pattern discontinuity prevents a
lossless batch claim. A record's `readUptimeMs` is the host capture observation,
not sample time. The library neither synthesizes sample timestamps nor claims
all axes/sensors were sampled simultaneously because BDU is enabled.

Continuous acquisition also rejects an uncertain nearly full burst even if
OVER_RUN clears during the read and the ending pattern matches. With this driver
as the sole FIFO consumer, a possible overflow followed by `N` consumed words
leaves at least `2048 - groupWords - N` unread words (capacity minus three
words per enabled dataset, minus the burst). This gives `2042 - N` for both
motion datasets and `2039 - N` when the fourth source is also enabled. A post-status count reaching that floor
returns `OPERATION_INDETERMINATE`, zero records and `continuityLost`. This
conservative policy can reject a coherent nearly full batch; it never relies on
a sticky overrun flag. FIFO stop-on-full can return coherent queued vectors,
while an observed FULL_SMART marks global continuity lost because collection
may have stopped. All FIFO reads and configuration changes must remain under
the same serialized owner for this evidence to be meaningful.

Configure keeps FIFO in Bypass while applying/verifying the complete image and
waiting for settling; active collection is enabled and verified last. Changing
scale, ODR or any interpretation while hardware FIFO is live is rejected before
interpretation writes. Reset, boot, recovery, self-test and power-down apply the
same rule. `startReconcile()` stays read-only and cannot assign a fresh verified
generation to unknown queued data. Use explicit bounded maintenance first:

```text
purge 2048 stop
profile set fifo continuous 104 1 1 24
profile apply
fifo 96 16
```

Stopping collection drains no more than the requested initial bound, then
selects/verifies Bypass. `residualLossPossible` remains explicit because data
can arrive between the last status observation and Bypass. This maintenance
invalidates verified state, retaining the desired profile for an explicit
configure or recovery. Self-test can run after stopping and applying a typed
Bypass profile; it restores that profile. Continuous collection is never
silently interrupted or flushed under a success claim of stream continuity.

Physical qualification is **NOT RUN** for this feature. The
[motion FIFO qualification procedure](https://github.com/janhavelka/LSM6DS3TR/blob/main/docs/HIL_VALIDATION.md#motion-fifo-qualification)
covers the silicon behavior that host fault tests cannot establish.

### Fourth Dataset And Counter Lifetime

`FifoProfile::fourthDataset` selects `OFF`, `TEMPERATURE` or `TIMESTAMP_STEP`;
`fourthDecimation` selects 1/2/3/4/8/16/32 when enabled. Temperature and
timestamp/step are mutually exclusive six-byte formats. The shared
`fifoPatternWord`/`startFifoRead` path returns records distinguished by `kind`;
`FifoRecord` aliases the retained `FifoMotionRecord` type. Existing motion
buffer ownership and terminal-publication rules apply unchanged.

Temperature uses bytes 3/4 of its six-byte record as signed little-endian raw
temperature; the other bytes are ignored. It uses internal triggering and
requires an active accelerometer or gyroscope even when no motion dataset is
batched. Repeated FIFO values do not establish a new conversion: the source
converts at most 52 Hz, or 12.5/26 Hz in the documented low-power cases. Buffered
temperature has no per-record `TDA` freshness claim.

Timestamp/step requires accelerometer ODR at least 26 Hz. Its byte order is
`timestamp[15:8], timestamp[23:16], unused, timestamp[7:0], steps[7:0],
steps[15:8]`, as visually checked in AN5130 Table 73. Records preserve raw
24-bit timestamp ticks, `TimestampResolution::MS_6_4` or `US_25`, raw 16-bit
steps, configuration generation and counter epoch. Neither raw counter is
expanded into an inferred monotonic total. Host admission/read/terminal time,
sensor counter time and physical motion sample time remain distinct.

`FifoTrigger::STEP_EVENT` is admitted only with timestamp/step. It changes the
trigger for every enabled FIFO dataset to step detection; decimation counts
events, and accompanying motion records have no implied regular ODR or new
conversion per event. `INTERNAL` retains the existing shared motion-trigger
cadence. Sensor-hub triggers and trigger-switched operating modes remain
unsupported.

Configure establishes fixed bank-A pedometer parameters with both sensors
proved powered down, verifies them, and restores/proves the main bank before
normal configuration. The policy is 2 g pedometer processing, threshold 16,
six debounce steps, 1040 ms debounce, and no step-delta batching. Selection or
restoration failure/cancellation exposes bank uncertainty; ordinary main-bank
operations retain their fences. Timestamp resolution is programmed before
counter enable. Every configure/replay reads the actual current resolution,
including when disabling timestamp/step support, and a resolution change issues
the required staged timestamp reset. Switching from 25 us to an off/default
profile therefore resets the timer as well. Same-resolution configure and
ordinary reads/reconcile do not silently reset timestamp or steps.

Use `startCounterReset(CounterResetRequest, timing, token)` only for an
explicit timestamp, step or joint reset with a known timestamp/step Bypass
profile. Timestamp reset writes `0xAA` to `TIMESTAMP2_REG`; step reset explicitly
sets and clears `PEDO_RST_STEP`. Terminal `counters` evidence distinguishes
confirmed/possible reset effects, epoch and continuity loss;
`registerBankMayBeUnknown` separately exposes bank uncertainty.
Source/profile changes, lifecycle interruptions, bind/unbind, purge, raw writes,
destructive raw-read attempts, cancellation and uncertain consumption invalidate affected continuity. A new
epoch does not mean an unobserved counter was read as zero; only successful
explicit joint reset can clear the sticky counter-continuity uncertainty.

Both CLIs expose atomic source edits and a separate effectful reset command:

```text
profile set fifo bypass 104 1 1 24
profile set fifo_fourth timestamp_step 1 internal 25us
profile apply
counterreset both
```

After taking the reset result, apply the desired active FIFO mode explicitly.
For temperature, use `profile set fifo_fourth temperature 1 internal 6.4ms`.
Physical temperature cadence, pedometer behavior, event triggering and counter
wrap remain [separate qualification cases](https://github.com/janhavelka/LSM6DS3TR/blob/main/docs/HIL_VALIDATION.md#fourth-dataset-and-counter-qualification), **NOT RUN**.

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
bus-silent vendor settle minima total 400 ms; each gate adds one millisecond
clock-quantization tick and is armed after its controlling write. Every
discarded or collected sample receives at most three STATUS checks and one data
read, with a 20 ms accelerometer or 5 ms gyroscope zero-I2C gate after a read
and between readiness checks. Each gate is armed from fresh post-callback time
with a one-millisecond clock-quantization tick. Three failed readiness checks
produce `DATA_NOT_READY`. If
stimulus may be active, any primary failure first attempts the same bounded
sensor-power-down then self-test-disable sequence used by successful self-test
completion.
The owner-visible primary-failure boundary occurs after that cleanup and before
full-profile restoration. The callback ceiling is
`16 * (samples + 1) + 126`, including one extra failure-cleanup callback
and the maximum counter-policy and reference-discard restoration work.
Self-test temporarily removes the managed user-offset contribution, establishes
the required sensor modes,
and powers down the opposite sensor where required. A verified gyro-sleep
profile is explicitly woken for the gyro phase, and the complete original
verified profile is restored afterward, including filter and interrupt choices;
an unapplied desired draft cannot replace the restoration target.

Calibration likewise allows at most three STATUS checks per collected sample.
It uses a zero-I2C gate equal to the configured sensor ODR period, rounded up
to milliseconds, after each non-final data read and between readiness checks.
Each gate is armed from fresh post-callback time with a one-millisecond
clock-quantization tick. It finishes after exactly the requested valid sample
count and has a ceiling of `4 * samples`. Neither maintenance procedure retries
a failed transfer or writes nonvolatile memory. Gyroscope calibration rejects
a sleeping gyro before I2C; waking and later restoring it is an explicit
self-test procedure, not hidden calibration policy.

Accelerometer calibration requires an explicit expected gravity vector in
sensor-native axes; the driver never assumes Z-up or a product mounting
orientation. Bias values are finite-checked. The caller decides mounting
transforms and durable calibration policy. Calibration rejects accel slope,
high-pass and reference output or gyro HPF for its requested sensor before
I2C, because those paths cannot provide gravity/zero-rate DC evidence.
Candidates carry immutable filter context; the driver never automatically
reuses a bias across a filter change. Self-test always uses the vendor
unfiltered recipe and reports primary and restoration failures separately.

`startFifoPurge()` first verifies the main register bank, live device identity,
and the managed IF_INC/BDU prerequisites, then destroys unread FIFO data. Its
result reports initial unread count/pattern, discarded words, final unread
count, overrun, and truncation. It performs at most `maxWords + 5` callbacks
(or two more when `stopCollection` is requested),
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

Raw reads that touch FIFO data bytes break decoder/counter continuity before
the transport attempt, including a failed attempt. After failed bank
restoration, diagnostics reject ordinary main-bank
access until bank state is proved: only `FUNC_CFG_ACCESS` reads and a zero
write that restores the main bank remain available. Reading back zero proves
main-bank selection; reset/boot or power-down can also perform their bounded
restoration sequence. No diagnostic reset or sample read restores counter
continuity or infers a missed-wrap total.
The explicit raw timestamp-reset write (`TIMESTAMP2_REG=0xAA`) is permitted
as a diagnostic exception and invalidates provenance; normal application code
uses the tokened counter-reset operation and its effect result.

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

The callback capacities, ownership rules, and product decisions a host firmware
must settle before embedding this driver are listed in the
[HIL validation guide](https://github.com/janhavelka/LSM6DS3TR/blob/main/docs/HIL_VALIDATION.md#host-firmware-integration-boundary).
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
| `status` / `diag` | Print bus readiness plus the retained initialization code/detail/message, selected and bound address, bus frequency and timeout, binding/job/result state, configuration generation and settling time, complete last transport error with timestamp/age, register mismatch evidence, current poll progress, and staged/desired/verified profiles. No I2C. |
| `job [current\|last]`, `result` | Show the cooperative session counters, active driver token/kind, cumulative and per-poll transaction evidence, wait/status state, and optionally the cached last terminal result. `result` is the `job last` convenience form; it does not call `takeResult()` again. |
| `bind`, `unbind`, `cancel` | Bind/unbind are zero-sensor-I2C lifecycle operations. `cancel` is the normal bus-silent terminal path. `unbind` is an explicit hard teardown that may discard an active or untaken result and clears cached evidence. |
| `scan` | Cooperatively issue one address-only ACK check per owner turn for the two valid SA0 addresses, `0x6A` and `0x6B`. ACK is presence only; `probe` still performs the WHO_AM_I identity proof. |
| `addr [0x6a\|0x6b]` | Show or select the driver address. A changed address replaces the binding without a sensor transfer, resets configuration/diagnostic provenance, and requires a new `probe`/`configure`. |
| `freq [100000\|400000]` | Show or select an owner-controlled I2C rate allowed by the chip. A successful owner-side clock/handle change preserves verified sensor configuration. |
| `probe`, `configure`, `sample [all\|accel\|gyro\|temp] [ready\|direct\|unqualified]` | Start the matching tokened driver job. `configure` applies the staged profile; ready/direct/unqualified modes retain distinct freshness and filter-qualification evidence. |
| `reset`, `boot`, `recover`, `reconcile`, `powerdown` | Start the bounded lifecycle procedure and publish its exact terminal configuration/effect evidence. |
| `selftest [5..100]` | Run both vendor self-test paths with the requested samples per baseline/stimulated phase; the default is 5. The fixture must be stationary. |
| `calxl [samples [x y z [max_p2p_g]]]` | Run 1..1000 accelerometer samples (default 32) against an expected gravity vector (default `0 0 1`, each component -16..16 g, vector magnitude 0.8..1.2 g) and a positive peak-to-peak limit up to 4 g (default 0.08 g). The CLI derives its absolute deadline from sample count and the verified accelerometer ODR. |
| `calg [samples [max_p2p_dps]]` | Run 1..1000 gyroscope samples (default 32) with a positive peak-to-peak limit up to 2000 dps (default 3 dps). The CLI derives its absolute deadline from sample count and the verified gyroscope ODR. |
| `fifo <1..2048 words> [1..32 records]` | Bounded typed acquisition into persistent CLI storage, raw XYZ with immutable scales/generation/read uptime and complete loss evidence; cached results print records only while their buffer token still matches. |
| `counterreset <timestamp\|steps\|both>` | Explicit bounded counter reset in a known timestamp/step Bypass profile; timestamp reset also acknowledges its end-counter request and reports epoch/effects. |
| `events [ready\|fifo\|embedded\|orientation\|all]` | Bounded source observation; `all` selects only configured groups. Routed latched step/orientation reads can acknowledge sources, and partial-read/effect evidence remains visible. |
| `purge <1..2048> [stop]`, `rreg <reg>`, `wreg <reg> <value>`, `dump <reg> <1..32>` | Explicit maintenance/diagnostic calls. Purge is destructive; `stop` explicitly drains bounded content then selects Bypass, reports possible residual loss, and requires explicit replay. Accepted raw writes invalidate configuration provenance; diagnostics are never normal acquisition policy. |
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
| `xl_lpf2` | Boolean `0/1`, `off/on`, or `false/true`; must preserve a coherent output path |
| `xl_filter` | Atomic host path: `lpf1 2`/`lpf1 4`, `lpf2 50`/`100`/`9`/`400`, `slope 4`, `highpass 100`/`9`/`400`, or `reference 100`; resets dependent host selectors |
| `xl_hpcf` | `50`, `100`, `9`, `400`; shared composite-filter code, with `50` encoding slope ODR/4 on the HP branch |
| `xl_input` | `latency` or `noise`; noise is admitted only where LPF2 has an active consumer |
| `xl_analog` | `auto` or `400`; forced 400 requires active high-performance ODR at least 1660 |
| `g_odr` | `pd`/`powerdown`, `12.5`, `26`, `52`, `104`, `208`, `416`, `833`, `1660`, `3330`, or `6660` |
| `g_fs` | `125`, `250`, `500`, `1000`, or `2000` dps |
| `g_power` | `hp`/`high` or `lp`/`low`; LP is limited to 208 Hz |
| `g_lpf1`, `g_sleep` | Boolean; LPF1 requires gyro high-performance mode |
| `g_ftype` | `0`, `1`, `2`, `3`; nondefault choices require `g_lpf1 on` |
| `g_hpf_mode` | `0.016`, `0.065`, `0.260`, or `1.040`; nondefault choices require `g_hpf on` |
| `offset_weight` | `1` or `16`, selecting the chip's nominal 1 mg/LSB or 16 mg/LSB modes (0.9765625 or 15.625 mg/LSB exactly) |
| `offset` | Three signed register values, each from -127 through 127 |
| `xl_slope_hp`, `xl_6d_lpf`, `g_hpf` | Boolean path selectors; 6D LPF requires orientation, gyro HPF requires active awake HP mode and excludes gyro FIFO; use `xl_filter` for coupled accel changes |
| `interrupts` | Boolean policy admission; `off` clears the complete interrupt policy to canonical defaults |
| `irq` | `<high\|low> <push_pull\|open_drain> <latched\|pulsed> <latch_events bool> <combine bool>`; atomic shared electrical/DRDY/latch policy |
| `int1`, `int2` | `<source> <bool>`; source is `accel`, `gyro`, `temp`, `watermark`, `full`, `overrun`, `step`, `step_overflow`, `timestamp` or `orientation`; unsupported direct pin/source pairs are rejected |
| `orientation` | `off`, or `<6d\|4d> <80\|70\|60\|50> <int1\|int2\|both>`; mode, threshold and routes change atomically |
| `irq_polarity`, `irq_drive`, `irq_drdy`, `irq_latch`, `irq_combine` | Individual edits to shared interrupt controls; each must preserve a valid complete profile |
| `fifo` | `off` resets FIFO fields; `<bypass\|fifo\|continuous> <odr> <g_dec> <xl_dec> <watermark>` atomically enables a complete motion profile |
| `fifo_mode`, `fifo_odr` | `bypass`/`fifo`/`continuous`; FIFO ODR 12.5..6660 Hz; each edit must preserve valid sensor cadence |
| `fifo_g_dec`, `fifo_xl_dec` | `off`, `1`, `2`, `3`, `4`, `8`, `16`, `32`; at least one enabled dataset |
| `fifo_fourth` | `off`, or `<temperature\|timestamp_step> <dec> <internal\|step> <6.4ms\|25us>`; changes the coupled source/decimator/trigger/resolution atomically |
| `fifo_fourth_dec`, `fifo_trigger`, `timestamp_resolution` | Fourth decimation, shared trigger and counter resolution; each edit must leave the complete profile valid |
| `fifo_watermark` | 0..2047 words; zero is always reached, not disabled |
| `bdu` | Boolean invariant probe; only `true`/`on`/`1` is valid for managed burst reads |

Because every field edit must leave a valid complete draft, coupled changes
may require a safe order. For example, stage 208 Hz before selecting LP, then
stage 1.6 Hz. `profile defaults` is the deterministic escape from an unwanted
draft; it does not touch hardware until `profile apply`.

Both CLIs include the profile's settling interval in configure, lifecycle
replay, reconcile, and self-test restoration deadlines. For example, 1.6 Hz
LP with LPF2 ODR/400 needs 200 seconds of settling alone. The derived budget
adds clock quantization, bounded reference-discard waits, and bus/scheduling
allowance, and saturates safely. Profile output prints the interval and budget.
The allowance also reserves every permitted callback at the configured
transport timeout. Sample budgets cover all 65 readiness checks and cadence
waits; calibration budgets include every sample wait and callback. Replay uses the driver's
desired profile, and self-test restores its verified profile; an unapplied
staged draft does not change those budgets.

A deliberate unqualified gyro diagnostic sequence is:

```text
profile set g_hpf on
profile set g_hpf_mode 0.065
profile apply
sample gyro unqualified
```

Wait for the configure result before starting the sample. `filter_unqualified`
is an expected verified configuration state in this sequence; the sample
reports `filter_unverified` with zero freshness. To return to qualified gyro
sampling, stage `g_hpf off` and apply the profile again. Hardware filter
transient and 6D response qualification remains **NOT RUN** for these additions.

## Building And Validation

```sh
python scripts/generate_version.py check
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python tools/test_check_idf_example_contract.py
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
evidence. The expanded version 2.1 campaign passed on the ESP32-S2 fixture with
accelerometer calibration explicitly omitted because no validated gravity
orientation was available. Maintained commands, exact coverage limits, and
retained physical results are in
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

The [retained branch review](https://github.com/janhavelka/LSM6DS3TR/blob/main/docs/RETAINED_BRANCH_AUDIT.md)
records historical audit/port dispositions, remaining qualification work,
and the retain/retire workflow.
The [optional feature implementation prompts](https://github.com/janhavelka/LSM6DS3TR/blob/main/docs/prompts/README.md)
define the ordered FIFO, fourth-dataset, interrupt and filter work from audit R2;
the prompts themselves do not implement those features.

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
| FIFO configuration and drain APIs | Use typed `FifoProfile` and caller-buffer `startFifoRead` for motion and the selected temperature or timestamp/step fourth dataset; `startFifoPurge` remains explicitly destructive. External sensors remain unsupported. |
| Interrupt routes and event sources | Use typed electrical/per-pin routes, 6D/4D orientation and bounded `startEventRead`; timestamp acknowledgement uses explicit counter reset. Tap/wake/free-fall/tilt/wrist/significant-motion, step-delta, boot, sensor-hub and DEN routes remain unsupported. |
| Timestamp control/readout and sensor-hub configuration/output APIs | Timestamp/step FIFO records and explicit `startCounterReset` replace the supported subset. Standalone counter sampling, expanded clocks, sensor-hub configuration and external outputs remain unsupported. |

## Version Metadata

`library.json` is the version source of truth. Run
`python scripts/generate_version.py sync` after changing it. The script owns
`include/LSM6DS3TR/Version.h`, the IDF component version, and Doxygen project
version; CI rejects drift.

License: MIT. See [LICENSE](LICENSE).
