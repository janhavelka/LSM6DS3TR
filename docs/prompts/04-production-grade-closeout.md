# AI Coder Prompt: Production-Grade Closeout For LSM6DS3TR Library

You are working in the LSM6DS3TR repository. Do a focused production-grade
closeout pass. The library is already in good shape and has recent targeted HIL
coverage; do not rewrite architecture or rerun broad HIL loops just to generate
activity.

Goal:
Close the remaining production-readiness gaps that are either missing or not
clearly documented. Keep the design simple, functional, deterministic, and
robust. Reuse existing code, helpers, and tests wherever feasible.

## Required Context

1. Read `AGENTS.md` first and follow it.
2. Read:
   - `README.md`
   - `CHANGELOG.md`
   - `include/LSM6DS3TR/Status.h`
   - `include/LSM6DS3TR/Config.h`
   - `include/LSM6DS3TR/LSM6DS3TR.h`
   - `src/LSM6DS3TR.cpp`
   - `test/test_basic.cpp`
   - `docs/reports/hil-validation-COM26-20260622.md`
   - `docs/reports/hil-targeted-poll-COM26-20260623.md`
3. Check `git status` and preserve dirty user changes.
4. Do not edit `include/LSM6DS3TR/Version.h` directly. If a version bump is
   needed, update `library.json` and run the version generator.
5. You may spawn subagents for read-only audits, but integrate final decisions
   yourself.

## Non-Goals

- Do not add a bus manager, scheduler, queue framework, registry, plugin system,
  or fake-device production path.
- Do not make examples own library policy.
- Do not add silent retries or fake success.
- Do not replace the managed synchronous architecture.
- Do not claim HIL passed unless you actually ran it on hardware.

## Priority 1: Fix The Time-Base Contract

Problem:
`Config::nowMs` is documented as optional and `poll(nowMs, ...)` accepts an
explicit clock, but `requestMeasurement(true)` currently arms its deadline from
`_nowMs()`, which falls back to `0` when `Config::nowMs` is not set. This can
make manually split ready-checked jobs timeout for reasons unrelated to I2C or
sensor readiness. HIL observed this in the targeted poll pass.

Required design:

- Keep `Config::nowMs` optional for compatibility.
- Use the explicit `poll(nowMs, maxInstructions)` timestamp for staged poll job
  deadlines and staged sample timestamps.
- `requestMeasurement(true)` must schedule the job without starting the ready
  deadline. Arm the deadline on the first positive-budget `poll()` that actually
  executes the status-read step.
- `poll(..., 0)` must remain a no-progress call and must not arm deadlines.
- For raw samples completed inside a poll job, store the sample timestamp from
  the poll timestamp that performed the raw burst.
- For direct blocking `readAllRaw()` and other synchronous APIs, keep using
  `Config::nowMs` when present and `0` when absent, and document that behavior.

Suggested implementation names:

- Add private `bool _pollDeadlineArmed = false;`
- Replace deadline-sentinel checks with `_pollDeadlineArmed`.
- Add a private helper:
  - `Status _readRawAllWithTimestamp(uint32_t sampleTimestampMs);`
  - Keep `Status _readRawAll();` as a wrapper using `_nowMs()`.
- In the sample poll raw-burst step, call `_readRawAllWithTimestamp(_pollNowMs)`.

Acceptance tests:

- Without `Config::nowMs`, `requestMeasurement(true)` followed by a delayed first
  positive-budget `poll(1000, 1)` must not immediately timeout because the
  request was scheduled at fallback time `0`.
- `poll(1000, 0)` after scheduling must return `IN_PROGRESS`, leave the job
  busy, and not arm the ready deadline.
- A ready-checked job that remains not-ready after the deadline starts must still
  return `TIMEOUT`.
- A poll-completed sample must report `sampleTimestampMs() == nowMs` from the
  poll that performed the raw burst.
- Update README and Doxygen to state the exact clock contract.

## Priority 2: Reserve `IN_PROGRESS` For Driver-Owned Jobs

Problem:
Transport callbacks can currently return `Err::IN_PROGRESS`, and direct
synchronous calls can pass that status through without any driver-owned job that
can complete it. That makes the status contract ambiguous.

Required design:

- Transport callbacks must be synchronous and bounded. They must not return
  `Err::IN_PROGRESS`.
- Append a new error code at the end of `Err`:
  - `I2C_BUSY` - transport or shared bus temporarily busy; no driver poll job is
    scheduled.
- If a transport callback returns `Err::IN_PROGRESS`, normalize it to
  `Err::I2C_BUSY` with static message `"Transport busy"`.
- `I2C_BUSY` should be treated as a tracked transport failure for normal tracked
  I2C operations.
- Keep `Err::IN_PROGRESS` only for:
  - `requestMeasurement()`
  - `poll()`
  - `startSoftReset()`
  - `startBoot()`
  - `startRefreshCachedConfig()`
  - `startFifoDrain()`
  - `startAccelBiasCapture()`
  - `startGyroBiasCapture()`
  - future driver-owned staged jobs

Acceptance tests:

- A fake transport returning `IN_PROGRESS` from a direct read must produce
  `I2C_BUSY`, not `IN_PROGRESS`.
- The failure must update health like other tracked transport failures.
- Staged driver jobs must still return `IN_PROGRESS` while active.
- Update `errToStr()` in Arduino and IDF CLIs.
- Update README and Doxygen transport callback docs.

## Priority 3: Harden FIFO Convenience Reads

Problem:
`readFifoWord()` checks `status.empty`, but it should also treat
`unreadWords == 0` as empty and should not silently return `OK` on overrun.

Required design:

- Append a new error code at the end of `Err`:
  - `FIFO_OVERRUN` - FIFO overrun was observed; data freshness is not guaranteed.
- `readFifoWord()` must:
  - return `FIFO_EMPTY` if `status.empty || status.unreadWords == 0`
  - return `FIFO_OVERRUN` if `status.overrun` is true
  - otherwise read exactly one FIFO word as before
- `startFifoDrain()` / `_pollFifoDrainStep()` must not report clean success when
  the initial status indicates overrun. Return `FIFO_OVERRUN` unless there is an
  explicit documented recovery path.

Acceptance tests:

- FIFO empty flag true returns `FIFO_EMPTY`.
- FIFO empty flag false but `unreadWords == 0` returns `FIFO_EMPTY`.
- FIFO overrun returns `FIFO_OVERRUN` and does not read a data word.
- Staged FIFO drain returns `FIFO_OVERRUN` on overrun status.
- Update CLI string mappings, README, and changelog.

## Priority 4: Prevent Bad Calibration From Poisoning Samples

Problem:
`captureAccelBias()`, `captureGyroBias()`, and staged calibration jobs
auto-apply computed bias without checking stillness or orientation quality.
Bad field calibration can silently affect all converted samples.

Required design:

- Append new error codes at the end of `Err`:
  - `CALIBRATION_UNSTABLE`
  - `CALIBRATION_ORIENTATION`
- Add configurable but bounded calibration quality limits:

```cpp
struct CalibrationLimits {
  float accelMaxPeakToPeakG = 0.08f;
  float accelMaxHorizontalMeanG = 0.20f;
  float accelMinZMeanG = 0.80f;
  float accelMaxZMeanG = 1.20f;
  float gyroMaxPeakToPeakDps = 3.0f;
};
```

- Add `CalibrationLimits calibrationLimits;` to `Config`.
- Use the same validation rules for blocking and staged calibration:
  - accel peak-to-peak on each axis must be `<= accelMaxPeakToPeakG`
  - `abs(meanX)` and `abs(meanY)` must be `<= accelMaxHorizontalMeanG`
  - `meanZ` must be within `[accelMinZMeanG, accelMaxZMeanG]`
  - gyro peak-to-peak on each axis must be `<= gyroMaxPeakToPeakDps`
- Do not update existing accel/gyro bias on calibration failure.
- Keep sample count validation unchanged: `1..10000`.

Suggested private fields for staged calibration:

- `_calibrationMinX`
- `_calibrationMaxX`
- `_calibrationMinY`
- `_calibrationMaxY`
- `_calibrationMinZ`
- `_calibrationMaxZ`

Acceptance tests:

- Stable accel samples with mean near `{0, 0, 1}` update accel bias.
- Accel samples with large peak-to-peak return `CALIBRATION_UNSTABLE` and leave
  previous bias unchanged.
- Accel samples with wrong orientation return `CALIBRATION_ORIENTATION` and
  leave previous bias unchanged.
- Stable gyro samples update gyro bias.
- Gyro samples with large peak-to-peak return `CALIBRATION_UNSTABLE` and leave
  previous bias unchanged.
- Staged and blocking calibration must behave consistently.

## Priority 5: Move Self-Test Into Core API

Problem:
`Err::SELF_TEST_FAIL` and self-test register bits exist in the public headers,
but the core library has no self-test API. The Arduino CLI currently duplicates
chip protocol logic in example code.

Required design:

- Add a core self-test result type:

```cpp
struct SelfTestResult {
  Axes accelBaseline;
  Axes accelStimulus;
  Axes accelDelta;
  Axes gyroBaseline;
  Axes gyroStimulus;
  Axes gyroDelta;
  bool accelPass = false;
  bool gyroPass = false;
};
```

- Add a blocking API first:

```cpp
Status runSelfTest(SelfTestResult& out, uint16_t samples = 5);
```

- Keep it bounded. Use finite polling caps and deadlines as existing blocking
  calibration/reset code does.
- Preserve and restore all affected registers even on failure where possible.
- Use datasheet thresholds with explicit constants:
  - `SELF_TEST_ACCEL_MIN_MG = 90.0f`
  - `SELF_TEST_ACCEL_MAX_MG = 1700.0f`
  - `SELF_TEST_GYRO_MIN_DPS = 150.0f`
  - `SELF_TEST_GYRO_MAX_DPS = 700.0f`
- Configure gyro self-test in the full-scale mode required by the datasheet
  threshold. Do not trust the current CLI's `20..80 dps` range until verified
  against the datasheet.
- Update Arduino CLI `selftest` to call `runSelfTest()` instead of duplicating
  register protocol.
- Add staged `startSelfTest(samples)` only if it can reuse the existing poll
  machinery cleanly without introducing a broad framework. Otherwise document
  blocking self-test as the current production API.

Acceptance tests:

- Passing self-test returns `OK` and fills deltas/pass flags.
- Out-of-range accel or gyro delta returns `SELF_TEST_FAIL`.
- Register restore is attempted after both success and failure.
- Transport errors propagate and are observable.
- Example CLI output remains machine-parseable by HIL runner.

## Priority 6: Clarify Poll Job Cancellation Semantics

Problem:
HIL confirmed `job cancel` clears CLI manual mode but does not cancel the active
driver poll job; normal `tick()` then completes the active job. This is not a
driver crash, but the name is misleading.

Required design:

- Do not add a core cancellation API unless you can define safe semantics for
  partially applied reset/boot/config/FIFO/calibration jobs.
- Prefer documentation and CLI wording:
  - Keep `job cancel` for compatibility.
  - Change help/output to: `"Clear CLI manual polling; active driver job may continue on tick"`.
  - Add `job status` output that makes `busy=yes` visible after cancel if a job
    is still active.
- README must document that active staged jobs are expected to complete or fail;
  `end()` is the hard lifecycle reset.

Acceptance tests:

- `job cancel` output must not imply a core job was cancelled.
- Existing HIL cancel probe should still pass.

## Priority 7: Align IDF And Arduino CLI Dialects

Problem:
The targeted HIL plan is Arduino-dialect. The IDF CLI currently accepts fewer
human-friendly tokens for ODR/FIFO settings.

Required design:

- Add native C helpers to the IDF CLI, not Arduino `String` or facades:
  - `parseOdrToken()`
  - `parseFifoModeToken()`
  - `parseFifoDecimationToken()`
- Accepted ODR tokens:
  - `0`, `1.6`, `12.5`, `12`, `26`, `52`, `104`, `208`, `416`, `833`,
    `1660`, `3330`, `6660`
- Accepted FIFO mode tokens:
  - `bypass`, `fifo`, `c2f`, `continuous-to-fifo`, `b2c`,
    `bypass-to-continuous`, `cont`, `continuous`, `0`, `1`, `3`, `4`, `6`
- Accepted FIFO decimation tokens:
  - `off`, `0`, `1`, `2`, `3`, `4`, `8`, `16`, `32`
- If output differences still require it, add `--cli-dialect arduino|idf` to
  `tools/hil_smoke.py`; otherwise keep one shared plan.
- Correct README IDF command examples so they parse on IDF.

Acceptance tests:

- `python tools/check_idf_example_contract.py` passes.
- IDF example source still contains no Arduino forbidden tokens.
- `tools/hil_smoke.py --dry-run --parser-self-test --suite targeted` passes.

## Priority 8: Promote HIL Evidence And Release Gates Into Docs

Required documentation updates:

- Add a README section named `HIL Evidence And Release Gates`.
- Summarize existing evidence:
  - native tests
  - ESP32-S3/S2 Arduino builds
  - broad validation HIL
  - targeted poll/feature HIL
  - reset/boot retry
  - cancel probe
- State remaining HIL gates before claiming final field readiness:
  - physical bus fault injection: hold SDA/SCL low or force NACK/timeout
  - IMU power-cycle or disconnect/reconnect
  - absent-device behavior
  - alternate address `0x6B`
  - live native ESP-IDF HIL
  - clean soak where prompt-capture UNKNOWNs are resolved or explained
- Update README quick-start transport callback sample to apply `timeoutMs` or
  point directly to `examples/common/I2cTransport.h`.
- Document `job status|auto|start|poll|run|get|getraw|cancel` grammar and
  caveats, including:
  - `poll(..., 0)` makes no progress
  - ready-checked sample jobs can timeout if split across slow manual commands
  - `job cancel` is CLI manual-mode cleanup only

## Priority 9: Changelog And Versioning

If you add public API, public structs, or public error codes:

- Update `library.json` from `1.1.0` to `1.2.0`.
- Run the version generator; do not hand-edit `Version.h`.
- Update `CHANGELOG.md` under `Unreleased` with:
  - Added: new error codes, calibration limits, core self-test API
  - Changed: time-base contract for staged poll jobs, FIFO overrun behavior
  - Fixed: transport `IN_PROGRESS` leakage, README timeout example
  - Documentation: HIL evidence and CLI job grammar

If you decide not to add a public API item above, document the reason in the
final response and in the relevant docs.

## Verification

Run the smallest relevant checks after each fix. Before final response, run:

```bash
python tools/check_core_timing_guard.py
python tools/check_chip_docs_coverage.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python tools/hil_smoke.py --dry-run --parser-self-test --suite targeted
pio test -e native
pio run -e esp32s3dev
pio run -e esp32s2dev
git diff --check
```

If ESP-IDF is available:

```bash
cd examples/idf/basic
idf.py set-target esp32s3
idf.py build
idf.py set-target esp32s2
idf.py build
```

HIL:

- Do not rerun broad validation repeatedly.
- If COM hardware is available and code behavior changed, run exactly one
  targeted pass:

```bash
python tools/hil_smoke.py --suite targeted --port COM26 --reset-before --stress-count 50
```

- If a physical fault rig is available, run a separate bounded fault-injection
  pass for bus hold/NACK/disconnect/recover and record it under `docs/reports`.

## Final Response Requirements

Report:

- Files changed.
- Production gaps fixed, ordered by severity.
- Public API or error-code changes.
- Tests/builds/HIL commands run with results.
- Anything intentionally left as documentation-only and why.
- Any remaining release gates.
