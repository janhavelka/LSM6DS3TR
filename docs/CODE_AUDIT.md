# Code Audit Resolution Report — 2026-08-31

This report records the resolution of every finding in the 2026-08-27 code
audit. The review started from clean, synchronized `main` at `0183f63`, with
local `HEAD` and `origin/main` identical. Each finding was checked against the
current implementation rather than accepted from the report at face value.

The review also checked the maintained chip-reference index and the applicable
protocol, timing, initialization, filter, self-test, and ambiguity topics before
changing silicon-facing behavior. For the Arduino transport finding, the
pinned Arduino-ESP32 3.3.11 `Wire` and ESP32 HAL sources were inspected to
verify the actual error and locking contracts.

## Outcome

| Finding | Verdict | Resolution |
|---|---|---|
| 1.1 Calibration averaging precision | Valid | Fixed with one shared floating-point mean-scale helper. |
| 1.2 Owner-soak start rejection loop | Valid | Fixed with immediate terminal failure; no retry state was needed. |
| 1.3 Arduino read error collapse | Valid; proposed remedy was incomplete | Replaced the lossy `Wire` staging path with the pinned ESP32 HAL combined transaction. |
| 1.4 ESP-IDF CLI hidden on bus failure | Valid | Retained typed initialization status and always entered the CLI. |
| 2.1 Stale mismatch in later results | Valid | Separated operation-local evidence from retained diagnostics. |
| 2.2 Reconcile restarts a settle gate | Valid | Preserved trusted `KNOWN` and `SETTLING` gates exactly. |
| 2.3 Self-test zero-budget wait evidence | Valid; adjacent positive-budget defect also existed | Made post-burst wait reporting consistent and cleared completed gates. |
| 2.4 Reset/boot guard uses pre-write time | Valid; proposed 15 ms re-arm was still short at a clock boundary | Re-armed from fresh time with a one-tick quantization margin; applied the same rule to self-test settles. |
| 3.1 Unused status values | Partly valid; the report's enum numbers and transport conclusion were wrong | Retained append-only values and corrected their API documentation. |
| 3.2 Unreachable sample BDU guard | Valid | Removed the redundant branch. |
| 3.3 Invalid owner-soak range guards | Valid | Replaced them with exact provenance-based conversion checks. |

No breaking public API or enum reorder was introduced. `library.json` remains
the version source of truth, and the changes are recorded under Unreleased.

## Finding-by-finding decisions

### 1.1 Calibration averaging precision

The finding was correct. Both calibration paths divided the 64-bit raw sums
using integer arithmetic, narrowed the result to `int16_t`, and only then
converted to physical units. This discarded any fractional-LSB mean despite
the fixed-count accumulator.

The implementation now computes one floating-point scale of
`sensitivity / sampleCount` and applies it directly to each 64-bit sum. This is
simpler than adding separate mean members or multiplying large integer sums by
fixed-unit sensitivities, and it avoids unnecessary integer-product bounds.
Peak-to-peak calculation remains unchanged and exact over the signed 16-bit raw
domain. A two-sample regression with raw X values 1 and 0 now proves the
expected 0.004375 dps bias instead of zero.

### 1.2 Owner-soak start rejection loop

The finding was correct. The soak changed phase before knowing whether the
driver had accepted the job. A rejected start therefore left no valid token but
could repeatedly revisit the same phase.

The simplest proper policy is not a retry counter: the soak scheduler only
attempts starts while it owns an idle driver, so rejection is already an
invariant failure. `acceptStart()` now commits the requested phase only after a
valid accepted token and moves directly to `COMPLETE` after logging one
`HIL_START_FAILURE`. The host runner already treats that marker as an immediate
failure.

### 1.3 Arduino write-read transport errors

The defect was valid, but wrapping the existing `Wire` calls could not fix it.
In the pinned Arduino-ESP32 3.3.11 implementation,
`endTransmission(false)` only stages a non-stop transfer and returns success;
the following `requestFrom()` performs the combined transaction and discards
its native `esp_err_t`. Consequently the old adapter could not distinguish a
timeout, NACK, busy bus, or generic bus failure.

The adapter now calls `i2cWriteReadNonStop()` using the public
`TwoWire::getBusNum()`. That HAL call performs the same combined repeated-start
transaction, owns the bus lock, accepts the bounded timeout, and returns the
native error. The mapping preserves timeout, busy, address-not-found, and raw
error detail. Failures whose ACK phase is not identified remain generic
`I2C_ERROR`; the adapter does not invent an address-versus-data NACK claim.
Static contract checks now require this path and reject a return to
`endTransmission(false)` staging.

This remains intentionally specific to the repository's pinned ESP32 Arduino
integration in `examples/common/`; no platform header entered the framework-
neutral library core.

### 1.4 ESP-IDF CLI availability after initialization failure

The finding was correct. `app_main()` returned after bus creation or device
registration failure, removing the diagnostic interface at the point it was
most useful.

`configureI2c()` now returns the example's normal typed `Status`; a file-scope
status retains the exact initialization result. Binding and the startup probe
only run after successful initialization, but `cliLoop()` is entered in either
case. `diag` prints a stable `bus_init code=... detail=... message=...` record.
The native-example checker pins both the diagnostic token and the non-fatal
control flow.

### 2.1 Operation-local versus lifetime mismatch evidence

The finding was correct. `_recordMismatch()` correctly populated both the
active result and lifetime diagnostics, but `_finish()` then copied the
lifetime mismatch into every later result.

The final copy was removed. A newly accepted operation already zero-initializes
its working result, while `_recordMismatch()` remains the sole writer of that
operation's mismatch evidence. The lifetime `_mismatch*` fields remain visible
through `diagnostics()` until a complete verification clears them. A regression
now fails configure readback, succeeds at power-down, proves that power-down's
result has no mismatch, and independently proves that diagnostics retain the
older failure.

No special self-test status policy was added: self-test does not legitimately
derive its primary result from an unrelated lifetime mismatch.

### 2.2 Reconciliation of an unexpired settling profile

The finding was correct. Reconcile preserved the prior gate only when the
captured state was `KNOWN`; an unexpired, already-verified `SETTLING` profile
therefore received a newly computed later `validAfterUptimeMs`.

A read-only reconciliation now trusts both captured `KNOWN` and `SETTLING`
states and restores the exact prior state and timestamp. Unknown or
unconfigured provenance still receives a conservative newly computed gate.
The earlier effective-state snapshot remains important: an already-expired raw
`SETTLING` member is captured as `KNOWN` at admission. A regression constructs
an unexpired gate, reconciles before it expires, checks all 35 callbacks are
reads, and proves neither the timestamp nor generation changes.

### 2.3 Self-test and calibration wait evidence

The reported zero-budget inconsistency was real. The same omission also made a
positive-budget poll that ended on a non-final data burst under-report the
newly armed cadence wait. Calibration had the same adjacent positive-budget
problem.

Every non-final self-test and calibration data burst now arms both the deadline
and `waiting`. A final self-test average explicitly clears both so it does not
publish a stale wait while moving to the next write stage. Zero-budget polling
recognizes self-test gates as it already did for sample/calibration gates.
Regression coverage checks all 24 data bursts of the minimum self-test: the
discard plus five averaged reads in each of four phases, including that each
sixth/final burst reports no obsolete cadence wait.

### 2.4 Post-command and self-test settle timing

The finding was correct: reset/boot/recovery previously based their 15 ms guard
on the `nowMs` supplied before the command transaction. Re-arming a nominal
15 ms interval on a later poll was still not sufficient, however, because the
public examples truncate monotonic time to whole milliseconds; the elapsed
physical interval can otherwise be almost one tick short.

After the command write, the state machine now leaves the deadline unarmed. A
later safe compute-only step, including `poll(nowMs, 0)`, samples fresh time and
sets the deadline to the policy interval plus one millisecond tick. The same
pre-write anchoring pattern existed in all four vendor self-test settles
(100 ms acceleration baseline, 100 ms stimulus, 150 ms gyroscope baseline,
50 ms stimulus), so those were corrected consistently. Constants describing
vendor or library minimums were not falsified, transaction ceilings did not
change, and absolute operation deadlines still take precedence.

The existing ready check following each self-test sample-cadence gate remains
the final proof of new data; adding extra substeps there would not improve the
hardware evidence.

### 3.1 `DEVICE_NOT_FOUND` and `FIFO_EMPTY`

The report correctly observed that the core does not synthesize these codes,
but two details were wrong:

- `DEVICE_NOT_FOUND` is value 18 and `FIFO_EMPTY` is value 23, not 19 and 25.
- A user transport is allowed to return `DEVICE_NOT_FOUND`; the core preserves
  typed callback status rather than normalizing it away.

`FIFO_EMPTY` remains unnecessary for the current purge contract because an
already-empty FIFO is a successful zero-discard result. Both values are
append-only public API and removing them would renumber every following code.
They were therefore retained with accurate Doxygen: `DEVICE_NOT_FOUND` is an
optional transport classification, while `FIFO_EMPTY` is reserved by the
current core contract.

### 3.2 Redundant BDU guard

The finding was correct. A sample can only use a verified profile, and every
verified production profile has already passed `validateProfile()`, which
requires BDU. The later runtime branch was unreachable and suggested a second
source of truth. It was deleted and replaced with a local explanation of the
existing invariant. Admission behavior and I2C traffic are unchanged.

### 3.3 Owner-soak range assertions

The finding was correct. The fixed acceleration threshold could never be
crossed at the default ±2 g sensitivity, while the gyroscope threshold was
below valid full-scale output at ±250 dps and could reject legitimate raw
values.

Acceleration and angular-rate maxima are still retained as useful telemetry,
but the invalid physical-limit assertions were removed. The soak now verifies
the meaningful invariant exactly: each converted axis must equal its raw
signed count multiplied by the sensitivity selected by that sample's immutable
full-scale provenance. Temperature retains the documented -40..85 °C
operating-range check. The HIL guide now describes these checks accurately.

## Additional contract guards

- The Arduino CLI checker requires the error-preserving HAL combined read and
  rejects the old lossy repeated-start sequence.
- The same checker pins owner-soak phase commit after accepted starts, terminal
  rejection handling, exact conversion checks, and removal of the stale range
  literals.
- The ESP-IDF checker requires retained `bus_init` diagnostics and proves the
  owner CLI remains after conditional initialization/binding.
- Chip timing and maintained-document coverage checks continue to pass.
- Timing documentation now distinguishes vendor minima from the library's
  one-tick integer-clock margin.
- Strict documentation/package validation exposed two pre-existing stale
  Markdown links; the documentation map and HIL changelog reference were
  corrected instead of suppressing those validators.

## Verification evidence

The following completed locally on the synchronized source:

- `.\scripts\pio.cmd test -e native`: **103/103 tests passed**.
- `.\scripts\pio.cmd run -e esp32s3dev -e esp32s3hil -e esp32s2dev`:
  **all three Arduino firmware environments built successfully** against
  Arduino-ESP32 3.3.11 and bundled ESP-IDF 5.5.5.
- `python tools/check_cli_contract.py`: passed.
- `python tools/check_idf_example_contract.py`: passed.
- `python tools/check_core_timing_guard.py`: passed.
- `python tools/check_chip_docs_coverage.py`: passed, covering 14 maintained
  topics and 50 exact register facts.
- `python -m py_compile ...` and `python tools/test_run_hil.py`: passed,
  including all 16 HIL-runner host tests.
- `python tools/build_docs.py`: warning-free Doxygen build passed.
- `pio pkg pack` plus `python tools/check_package_contract.py`: passed with 37
  files and 20 linked Markdown documents, using an isolated temporary archive
  so an older ignored local archive was not overwritten.
- `python scripts/generate_version.py check`: all generated version artifacts
  were current.

The local shell did not contain `idf.py`, and repository policy forbids
silently installing another toolchain. Therefore the native ESP-IDF example's
static contract was checked here, while its two-target compilation remains a
CI build gate. No physical sensor was connected for this review, so the
targeted HIL campaign and one-hour soak were not claimed as new evidence.
