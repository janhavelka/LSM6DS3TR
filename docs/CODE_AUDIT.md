# Code Audit Verification And Remediation Report — 2026-09-08

All 11 findings have been independently checked against the synchronized
codebase. Ten were valid historical defects and one (3.1) was partly correct.
Their original defects were already addressed, but the deeper review of **2.4
found a remaining timing defect in shared configuration settling**, fixed in
this change. The other existing remedies remain the simplest appropriate
solutions. This report distinguishes those retained fixes from the new work.

## Scope and source history

The review fetched all remotes with pruning, verified a clean working tree,
compared local and remote branch tips, and ran the upstream fast-forward check.
`main` was the newest branch, and both baseline `HEAD` and `origin/main` were
`9874e81c134c231d501bc46b54f69cc9c642a96e`. The fast-forward check required no
update.

At that commit this file was already a resolution report, not an outstanding
worklist. To avoid treating previous conclusions as proof, this review read
both the [original 2026-08-27 audit at 0183f63](https://github.com/janhavelka/LSM6DS3TR/blob/0183f63/docs/CODE_AUDIT.md)
and the [2026-08-31 resolution at f820714](https://github.com/janhavelka/LSM6DS3TR/blob/f820714/docs/CODE_AUDIT.md),
then inspected actual code, regression tests, and fixes in `7d44e39` and
`f820714`. The previous report's conclusions were not treated as proof.

Three independent parallel reviews covered core math/provenance, operation
timing, and examples/transports/HIL. The primary review checked their source
evidence and the resulting patch. The maintained chip-reference index and
applicable conversion, protocol, initialization, settling, self-test, and
ambiguity topics were consulted. Arduino transport semantics were checked
against installed Arduino-ESP32 3.3.11 `Wire.h`, `Wire.cpp`, and
`esp32-hal-i2c-ng.c`, plus the bundled ESP-IDF I2C headers in `C:\pio\packages`.
PlatformIO commands used that existing selected `C:\pio` installation.

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
| 2.4 Guard anchored before command completion | Valid; prior fix missed shared profile settling | Retain corrected command/self-test guards; fix new configuration gates to use fresh post-readback time plus one clock tick. |
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
harness also passed 100 calibration cases: both sensors, every full scale,
counts 1/2/3/999/1000, positive and negative means, and signed gyro endpoints at
count 1000. Expected means were calculated independently in double precision.

### 1.2 Owner-soak start rejection

`examples/02_owner_soak/main.cpp::acceptStart()` commits the token and phase
only after an accepted start. Rejection logs one `HIL_START_FAILURE`, increments
the failure count, and enters `COMPLETE`. `scheduleWork()` exits in that phase;
the host soak runner also recognizes the marker as immediate failure.

The original proposal described stopping but then added a consecutive-failure
counter and repeated attempts. That state is unnecessary: this harness starts
work only when its owner expects admission, so rejection already fails the
campaign. The current void helpers express that terminal policy directly.

An independent temporary harness called each of the four actual start helpers
against an unbound driver, then called `scheduleWork()` 10,000 times per helper.
Each case produced exactly one rejection log and no retry.

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
This does not promise CLI availability if console initialization itself fails.

The current separation of `mapEspError()` and `mapEspProbeError()` is needed:
`ESP_ERR_NOT_FOUND` means no free bus during bus creation, but address NACK for
`i2c_master_probe()`. A single generic NACK mapper would falsify diagnostics.
The static checker covers CLI reachability and this context boundary by
inspecting each mapper's definition body independently of source order and
forward declarations.

### 2.1 Mismatch evidence ownership

`_start()` clears the working operation result. `_recordMismatch()` populates
its mismatch triple and the separate lifetime triple. `_finish()` no longer
copies lifetime evidence into an unrelated result. Full profile verification
clears the lifetime diagnostics without erasing this operation's evidence.

Two [native regressions](../test/test_basic.cpp) split this evidence.
`test_lifetime_mismatch_diagnostics_do_not_leak_into_later_results` runs a
failed configure followed by a successful power-down and proves that the new
result has a zero mismatch triple while lifetime diagnostics retain the
earlier mismatch. `test_configuration_readback_mismatch_reports_exact_register_values`
then separately proves that successful full-profile **reconciliation** produces
a clean result and clears the lifetime diagnostic triple. It does not perform
another configure. Independent temporary checks extended the sequence through
successful probe and empty FIFO purge, with the same correct separation. No
special self-test status policy is necessary: primary self-test measurement
failure must not be inferred from an unrelated lifetime register mismatch.

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

### 2.4 Post-callback timing and the remaining configuration defect

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

However, shared `_stepConfigure()` finalization still computed a new
`_validAfterUptimeMs` using the timestamp supplied before a poll's callbacks.
A poll could execute profile writes and final readback before this compute
stage. Completing readback did not prove that the supplied timestamp was after
those writes. The previous fixed-budget timing experiments missed this case.

An independent reproduction used a 12.5 Hz accelerometer with gyro off,
successful 7 ms writes within a 10 ms callback timeout, and transaction budgets
of 12 followed by 255. The second poll began at 1070 ms; accelerometer activation
completed at 1084 ms and the filter write at 1133 ms. These are simulated-clock
values, not hardware measurements.

| Evidence | Baseline | Fixed |
|---|---:|---:|
| Final profile write completed | 1231 ms | 1231 ms |
| Configuration declared valid | 2190 ms | 2352 ms |
| Elapsed since accelerometer activation | 1106 ms | 1268 ms |
| Required nominal settling interval | 1120 ms | 1120 ms |
| Physical callbacks | 68 | 68 |

The baseline declared `KNOWN` 14 ms before even the activation-based minimum.
The fix adds nine production lines in [`_stepConfigure()`](../src/LSM6DS3TR.cpp):
both final managed-readback paths set the existing `_pollBoundary`, so the
existing compute stage receives fresh time from a later poll. Newly computed
positive intervals receive a saturating one-millisecond quantization margin.
At a fresh arming time of 1231 ms, the reproduction now retains all 1121 ms.

The shared fix covers configure, reset/boot/recovery replay, self-test
restoration, and reconciliation from untrusted state. No new state, clock
callback, retry, or physical transaction is added. Absolute deadlines still
take precedence. Trusted reconciliation preserves its exact timestamp and
generation; zero-settle profiles add no timed delay and can finish in a
zero-budget compute poll without advancing time. Separate per-operation fixes
or estimates derived from callback timeout ceilings would be more complex.

Two retained [native regressions](../test/test_basic.cpp) cover the new behavior:

- `test_configuration_settle_arms_after_readback_with_changing_budgets` covers
  configure and untrusted reconcile, 7 ms callbacks, budgets 12 then 255,
  another 23 ms before zero/positive-budget arming, exact gate boundaries,
  generation, and callback counts.
- `test_configuration_settle_saturates_and_zero_settle_needs_no_time_advance`
  covers `UINT64_MAX` saturation, deadline precedence, and powered-down profiles
  with no settle interval.

An isolated build of these retained tests against baseline `9874e81` failed
the changing-budget assertion (expected `validAfter=2620`, observed `2204`);
both tests pass against the fixed core. That test advances both read and write
callbacks and delays arming, so its timestamps intentionally differ from the
write-only-latency reproduction table. The saturation/zero-settle test passes
on both versions, confirming preservation of those boundaries.

The original trusted-settle reconcile test now explicitly performs the fresh
compute poll after readback. An independent reviewer additionally exercised
12 reconcile boundary cases: prior `KNOWN`/`SETTLING`/`UNKNOWN`, before/after
publication, and cancellation/deadline. Trusted timestamps and generation were
preserved; interruption added no I2C and retained conservative state.

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
accept signed endpoints and reject wrong conversions. Independent tests also
exercised 40 signed-endpoint/full-scale combinations in the actual owner-soak
code, retaining valid telemetry and detecting deliberately wrong conversions.

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

This change fixes shared configuration-gate anchoring in
[`src/LSM6DS3TR.cpp`](../src/LSM6DS3TR.cpp), adds two focused native regressions,
and adjusts the existing trusted-reconcile test to perform explicit post-readback
finalization. It updates the public timing Doxygen, README,
[maintained settling reference](chip-reference/09_filters_and_settling.md),
changelog, and this report. The other fixes remain attributed to their existing
commits; this review does not claim to have implemented them again.

There is no new state, API, transaction, allocation, transport retry, or renewed
deadline. Version metadata remains unchanged and the correction is recorded
under Unreleased; this is not a tagged release.

Fresh local checks on the reviewed source:

- `.\scripts\pio.cmd test -e native`: **105/105 passed** using the selected
  PlatformIO Core 6.1.19 (`PLATFORMIO_CORE_DIR=C:\pio`).
- CLI, native-IDF, core-timing, and chip-documentation contract checkers: passed;
  chip coverage includes 14 maintained topics and 50 exact register facts.
- Python compilation of the HIL runners and `python tools/test_run_hil.py`:
  passed, including **18/18 host tests**.
- Independent compiled core checks: **100 calibration cases**, transport-code
  preservation, mismatch ownership, empty purge, and **12 reconcile interruption
  cases** passed, including rechecking against the settling fix.
- A temporary C++ harness compiled the actual owner-soak code, transport header,
  and baseline core against isolated SDK stubs with
  `g++ -std=c++17 -Wall -Wextra -Werror`: **63 cases passed**. These cover 16
  native write/read result mappings with bus 1 and exact timeout forwarding,
  three invalid/short-read boundaries, four rejected-start helpers followed by
  10,000 scheduler calls each, and 40 signed-endpoint/full-scale checks. These
  unchanged integration paths also compiled in the baseline Arduino builds.
- The independent changing-budget reproduction failed on the baseline and
  passed after the fix with the timing/callback evidence in section 2.4.
  Temporary experiments were not added to production paths.
- `python tools/build_docs.py`: strict, warning-free Doxygen build passed.
- `.\scripts\pio.cmd pkg pack --output <temporary archive>` followed by
  `python tools/check_package_contract.py <temporary archive>`: passed,
  **37 files and 20 linked Markdown documents**. Existing local archives were
  preserved.
- `python scripts/generate_version.py check`: all generated artifacts current.

The baseline's three Arduino environments built successfully using the selected
existing installation. The post-change command
`.\scripts\pio.cmd run -e esp32s3dev -e esp32s3hil -e esp32s2dev` then failed
during package/toolchain setup: `idf_tools.py` reported an unexpected archive
layout, package copying reported missing files, and the compiler executables
`xtensa-esp32s3-elf-g++`/`xtensa-esp32s2-elf-g++` were unavailable. All three
post-change environments therefore failed locally. This is not a passing
firmware build or a demonstrated source failure. No replacement Core was
installed and no toolchain paths were manually repaired.

For traceability, the reviewed baseline also has a
[successful seven-job CI run](https://github.com/janhavelka/LSM6DS3TR/actions/runs/33987135742),
including all three Arduino environments and native ESP-IDF 5.4.4 builds for
ESP32-S2 and ESP32-S3. That validates the baseline, not the new core change.
Native `idf.py` is unavailable in this shell. The remediation commit `7710af3`
subsequently passed [all seven CI jobs](https://github.com/janhavelka/LSM6DS3TR/actions/runs/34210810511),
including all three Arduino environments and both native-IDF targets. No
physical HIL campaign or one-hour soak was run, and stub-backed integration
checks do not establish hardware timing.

## Independent follow-up: three residual issues

An independent audit of `7710af3` identified three valid low-severity issues.
These corrections leave the driver runtime behavior and the original 11 fixes
unchanged:

1. The IDF checker sliced mapper source between `find()` offsets, so a reordered
   generic mapper could yield an empty body and pass silently. A forward
   declaration also defeated a simple source-order check. The checker now
   matches definitions followed by an opening brace and extracts balanced
   bodies independently of declaration/definition order. A missing definition
   fails explicitly. The unused callback-boundary search was removed.
2. The shared reset/boot guard comment incorrectly called the interval a vendor
   minimum for both operations. The comment now distinguishes AN5130's 15 ms
   BOOT interval from the conservative SW_RESET policy (vendor figure about
   50 us). Constants, timing, and the already-correct section 2.4 explanation
   are unchanged.
3. Section 2.1 attributed clearing diagnostics to the failed-configure/then-
   power-down regression. That test only proves result isolation and retained
   lifetime evidence. The separate readback-mismatch regression proves clearing
   through successful **reconciliation**. The report now names both tests and
   the correct operation without relying on stale line numbers.

The new `tools/test_check_idf_example_contract.py` runs three mutation tests
with 11 scenarios: valid mappers in both orders and with a forward declaration;
invalid generic NACK and busy classifications in all three layouts; and
declarations without definitions for each mapper. The reordered bad mappings
reproduced the original silent pass before the fix and are now rejected. The
tests exercise the complete checker against in-memory mutations of the actual
IDF source and run in the existing CI validation job.

Follow-up local verification passed:

- `.\scripts\pio.cmd test -e native`: **105/105**.
- `python tools/test_check_idf_example_contract.py`: **3/3 tests**, covering
  the 11 mutation scenarios above.
- CLI, native-IDF, core-timing, and chip-documentation contract checkers.
- HIL host tests: **18/18**; Python compilation of the changed checker/tests.
- Generated-version check and strict Doxygen build.
- Fresh package check: **37 files and 20 linked Markdown documents**.

CI was green at the follow-up baseline `7710af3`; no existing CI failure needed
repair. The validation job now includes the mapper mutation regressions, and
the follow-up commit's CI result is reported separately after pushing.
