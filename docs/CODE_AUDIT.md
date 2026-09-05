# Code Audit Verification Report — 2026-09-05

All 11 findings have been checked against the synchronized codebase. Ten were
valid historical defects and one (3.1) was partly correct. **All are already
resolved in the reviewed baseline; no additional production-code change is
justified.** Several original remedies were incomplete or unnecessarily
complicated. The existing implementation incorporates the simpler proper
solutions described below.

## Scope and source history

The review fetched all remotes with pruning, verified a clean working tree,
compared local and remote branch tips, and ran the upstream fast-forward check.
`main` was the newest branch, and both `HEAD` and `origin/main` were
`f82071420e87301900711c7d103cd714106d9e74`.

At that commit this file was already a resolution report, not an outstanding
worklist. To avoid treating previous conclusions as proof, this review read
both the [original 2026-08-27 audit at 0183f63](https://github.com/janhavelka/LSM6DS3TR/blob/0183f63/docs/CODE_AUDIT.md)
and the [2026-08-31 resolution at f820714](https://github.com/janhavelka/LSM6DS3TR/blob/f820714/docs/CODE_AUDIT.md),
then inspected the actual code, regression tests, and relevant history.

Three parallel reviews covered core math/provenance, operation timing, and
examples/transports/HIL. Their conclusions were checked against source and
validation evidence. The maintained chip-reference index and applicable
conversion, protocol, initialization, settling, self-test, and ambiguity topics
were consulted. Arduino transport semantics were checked against installed
Arduino-ESP32 3.3.11 `Wire.h`, `Wire.cpp`, and `esp32-hal-i2c-ng.c`, plus the
bundled ESP-IDF I2C headers in the existing `%USERPROFILE%\.platformio\packages`
cache. PlatformIO commands used the separately selected `C:\pio` installation.

## Finding-by-finding verdicts

| Finding | Original claim | Current disposition |
|---|---|---|
| 1.1 Calibration averaging precision | Valid | Shared floating-point mean scaling is correct; retain it. |
| 1.2 Owner-soak rejected-start loop | Valid | Immediate terminal failure is simpler than a rejection counter. |
| 1.3 Arduino read-error collapse | Valid; proposed classification was unsafe | Native-result-preserving HAL calls are the proper existing fix. |
| 1.4 ESP-IDF CLI hidden on initialization failure | Valid | CLI remains available and initialization status is retained. |
| 2.1 Lifetime mismatch leaking into later results | Valid | Operation-local evidence and lifetime diagnostics are separated. |
| 2.2 Reconcile extending a trusted settle gate | Valid | Existing trusted state, timestamp, and generation are preserved. |
| 2.3 Self-test wait evidence with zero budget | Valid | Cadence arming and wait evidence work with zero and positive budgets. |
| 2.4 Guard anchored before command completion | Valid; proposed interval was incomplete | Fresh post-callback time plus one clock tick enforces the minimum. |
| 3.1 Unused status values | Partly valid | Accurate comments retain the append-only public values. |
| 3.2 Redundant BDU admission guard | Valid | Validation remains the single source of this invariant. |
| 3.3 Incorrect HIL motion-range assertions | Valid | Both HIL paths check conversion; motion maxima are telemetry. |

### 1.1 Calibration averaging precision

In `src/LSM6DS3TR.cpp`, `meanRawAxesToFloat()` computes `scale / count` in
floating point and applies it to each 64-bit sum. Both accelerometer and
gyroscope calibration use this helper. Neither path narrows the mean to
`int16_t` or performs an integer division before conversion.

This is simpler than the original proposed helper that multiplies integer
sums by fixed-unit sensitivities. It reuses the existing floating-point scale,
needs no additional state, and preserves positive and negative fractional-LSB
means. Validated sample counts prevent division by zero. Peak-to-peak spans
remain wide enough for the full signed raw domain. Self-test averaging remains
unchanged, as the original audit explicitly required.

The retained fractional-mean regression passes. An independent temporary C++
harness also passed 112 calibration cases: both sensors, every full scale,
counts 1/2/3/1000, positive and negative fractional means, and signed gyro
endpoints. Expected means were calculated independently in double precision.

### 1.2 Owner-soak start rejection

`examples/02_owner_soak/main.cpp::acceptStart()` commits the token and phase
only after an accepted start. Rejection logs one `HIL_START_FAILURE`, increments
the failure count, and enters `COMPLETE`. `scheduleWork()` exits in that phase;
the host soak runner also recognizes the marker as immediate failure.

The original proposal described stopping but then added a consecutive-failure
counter and repeated attempts. That state is unnecessary: this harness starts
work only when its owner expects admission, so rejection already fails the
campaign. The current void helpers express that terminal policy directly.

### 1.3 Arduino transport error fidelity

The original defect was real: `endTransmission(false)` stages the transfer,
while `requestFrom()` performs it and loses the native error. Its proposed
zero-byte-to-address-NACK mapping is incorrect because zero bytes do not prove
which failure occurred.

`examples/common/I2cTransport.h` already uses `i2cWrite()` and
`i2cWriteReadNonStop()` with the public `TwoWire::getBusNum()`. That getter
avoids hardcoding a peripheral number. The callbacks do not first enter a Wire
transaction, so they do not strand its lock. Both writes and reads retain
native status detail; timeout is typed, while ambiguous state/resource/NACK
results remain `I2C_ERROR`. The separate address-only probe may classify an
address NACK using its narrower result contract.

These are appropriate example-only ESP32 integration calls. No platform header
has entered the framework-neutral core. One qualification to the old report:
the HAL uses `portMAX_DELAY` for its internal semaphore. The supplied timeout
bounds the physical transfer; bounded callback execution also relies on the
required single bus owner preventing lock contention. The example is not a
promise of bounded execution under arbitrary concurrent SDK bus use.

### 1.4 Native ESP-IDF diagnostic availability

`examples/idf/basic/main/main.cpp::configureI2c()` returns a typed `Status`.
`app_main()` retains it, conditions binding and startup probing on success,
and enters `cliLoop()` after either initialization outcome. `diag` reports the
retained `bus_init` code, detail, and message.

The current separation of `mapEspError()` and `mapEspProbeError()` is needed:
`ESP_ERR_NOT_FOUND` means no free bus during bus creation, but address NACK for
`i2c_master_probe()`. A single generic NACK mapper would falsify diagnostics.
The existing static checker covers CLI reachability and this context boundary.

### 2.1 Mismatch evidence ownership

`_start()` clears the working operation result. `_recordMismatch()` populates
its mismatch triple and the separate lifetime triple. `_finish()` no longer
copies lifetime evidence into an unrelated result. Full profile verification
clears the lifetime diagnostics without erasing this operation's evidence.

The existing failed-configure/successful-power-down regression proves a clean
successful result while diagnostics retain the earlier mismatch, and then
checks that successful configuration clears those diagnostics. No special
self-test status policy is necessary: primary self-test measurement failure
must not be inferred from an unrelated lifetime register mismatch.

### 2.2 Reconcile and existing settling provenance

Admission snapshots the effective `configurationState(nowMs)`, so an expired
raw `SETTLING` member is already captured as `KNOWN`. `_stepConfigure()` trusts
exactly prior `KNOWN` or `SETTLING` for read-only reconciliation and restores
the existing validity timestamp. It does not increment configuration
generation. Unknown/unconfigured provenance still receives a conservative new
gate because the time of external hardware changes is unproved.

Existing tests cover both trusted states, including all 35 read-only callbacks
and the exact unexpired timestamp. Restoring these fields is the minimal fix;
removing settling altogether would weaken unknown-state recovery.

### 2.3 Wait evidence and cadence

Self-test, calibration, and unsuccessful ready-sample checks use `_step == 2`
as a compute-only state after callbacks that require a cadence interval.
`_waiting` ends that poll, and a later poll arms the interval from fresh owner
time. Explicit safe-state predicates permit this arming with zero callback
budget. Live gates report waiting consistently; completed self-test averages
and failure/restore transitions clear obsolete gates.

The predicates matter because the numeric step is reused elsewhere for actual
I2C work. Executing every step 2 with zero budget would violate the transport
contract. Current code keeps that distinction without another state member.
The retained tests cover all 24 minimum self-test bursts, delayed arming,
calibration waits, unsuccessful readiness checks, cancellation, and cleanup.

### 2.4 Reset/boot/recovery and self-test minimum guards

After a controlling command write, the wait timestamp is deliberately left
unarmed and the poll returns. A subsequent compute-only step sets the guard
from fresh time, adding one millisecond to account for whole-millisecond clock
truncation. Reset, boot, recovery, and all four self-test settle stages use
this rule. Absolute operation deadlines and saturation still take precedence.

The original nominal 15 ms re-arm could remain almost one tick short. The
existing margin fixes that without changing vendor constants or adding a
clock callback/member. The 15 ms reset guard is documented library policy;
AN5130's approximately 50 us reset duration remains a separate silicon fact.
No transport retry or extra physical transaction was introduced.

An independent temporary C++ timing harness passed 48 cases covering 280
gates: six operation types, callback budgets 1/2/8/255, and zero/positive-budget
arming. Callbacks advanced simulated time by 7 ms and arming was delayed another
23 ms. Every checked gate remained bus-silent through `armTime + minimum`, and
all operations completed within their callback ceilings.

### 3.1 Retained status values

`DEVICE_NOT_FOUND` is numeric value 18 and `FIFO_EMPTY` is 23; the original
report's values 19/25 were wrong. Their current Doxygen accurately describes
an optional transport classification and a reserved core value respectively.
The temporary core harness confirmed `DEVICE_NOT_FOUND` and its detail survive
into the terminal result and diagnostics, and an empty FIFO purge succeeds
with zero discarded words.

Keeping the values preserves the append-only API. Deleting them or making an
empty purge fail would change a valid public contract for no current need.

### 3.2 BDU validation

`startSample()` already requires a verified profile. Desired profiles pass
`validateProfile()`, which rejects disabled BDU, and verified profiles come
only from those profiles or a previously verified self-test restore profile.
The removed branch was therefore redundant. Existing validation/admission tests
cover this invariant. An assertion, extra status, or extra hardware read would
add behavior without fixing this finding.

### 3.3 HIL conversion assertions

The owner soak compares each converted motion axis with its signed raw count
multiplied by the sensitivity selected by immutable sample provenance.
`tools/run_hil.py` checks its default provenance before making the corresponding
comparison. Both retain motion maxima as telemetry instead of interpreting
nominal full-scale labels as exact signed-code bounds. The temperature
operating-range check remains separate from conversion verification.

The negative gyro endpoint at the default scale is legitimately
-286,720,000 micro-dps; the old 251 dps limit was invalid. Existing host tests
accept signed endpoints and reject wrong conversions. A further temporary
matrix passed 114 checks across all seven nonempty quantity masks, ready/direct
modes, signed motion endpoints, fractional temperature values of both signs,
and deliberately incorrect conversions.

## Deliberate nonchanges rechecked

- `MAX_TRANSPORT_WRITE_BYTES = 33` is a public buffer ceiling, not an assertion
  that every managed write uses that size. Existing compile contracts pin it.
- Ordinary output sampling retains the documented nondestructive classification.
  Ready-flag read side effects do not make it the explicitly destructive FIFO
  purge or justify changing `hardwareStateMayHaveChanged` semantics here.
- `CTRL7_G` diagnostic mask `0xF8` correctly includes `ROUNDING_STATUS` at bit 3
  and excludes reserved bits 2:0; source and diagnostic tests agree.
- The conservative reset guard and the hardware-observed, undocumented BOOT
  self-clear assumption remain accurately recorded in the ambiguity ledger.
- Portable HIL host-integration facts remain useful: fixed objects, external
  ownership, 33-byte write/32-byte read ceilings, and seven scalar sample fields.

## Work performed and verification

This change refreshes this report and records the re-verification under
Unreleased in `CHANGELOG.md`. Production sources, tests, public API, version
metadata, and supported behavior needed no additional change. Prior fixes
remain attributed to their existing commits; this review does not claim to
have implemented them again.

Fresh local checks on the reviewed source:

- `.\scripts\pio.cmd test -e native`: **103/103 passed** using the selected
  PlatformIO Core 6.1.19 (`PLATFORMIO_CORE_DIR=C:\pio`).
- CLI, native-IDF, core-timing, and chip-documentation contract checkers: passed;
  chip coverage includes 14 maintained topics and 50 exact register facts.
- Python compilation of the HIL runners and `python tools/test_run_hil.py`:
  passed, including **18/18 host tests**.
- Temporary independent calibration, timing, and HIL parsing matrices: passed as
  detailed above. These were review experiments, not added production paths.
- `python tools/build_docs.py`: strict, warning-free Doxygen build passed.
- `.\scripts\pio.cmd pkg pack --output <temporary archive>` followed by
  `python tools/check_package_contract.py <temporary archive>`: passed,
  **37 files and 20 linked Markdown documents**. Existing local archives were
  preserved.
- `python scripts/generate_version.py check`: all generated artifacts current.

The local Arduino build command
`.\scripts\pio.cmd run -e esp32s3dev -e esp32s3hil -e esp32s2dev` could not
provide firmware evidence. The selected existing `C:\pio` installation failed
to resolve toolchain files and esptool package metadata; compilation then failed
because `xtensa-esp32s3-elf-g++` was unavailable. Remaining environment attempts
were stopped after that failure. No replacement PlatformIO Core
was installed or toolchain configuration changed manually. Native `idf.py`
was also unavailable in this shell. These are local environment limitations,
not passing compile results or demonstrated source defects.

For traceability, the reviewed baseline also has a
[successful seven-job CI run](https://github.com/janhavelka/LSM6DS3TR/actions/runs/33442779481),
including all three Arduino environments and native ESP-IDF 5.4.4 builds for
ESP32-S2 and ESP32-S3. That is existing evidence from 2026-08-31, not a newly
performed local build. No physical HIL campaign or one-hour soak was run for
this review.
