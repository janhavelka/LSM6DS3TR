# LSM6DS3TR Targeted HIL Poll And Feature Audit - COM26

Date: 2026-06-23
Board: ESP32-S3 on COM26
Firmware: Arduino bring-up CLI, version 1.1.0 dirty build `f10e8e4`
Device address: 0x6A

## Scope

This was a smaller targeted HIL pass, not a repeat of the broad validation/soak run. It focused on:

- staged `poll(nowMs, maxInstructions)` behavior with budgets `0`, `1`, and `255`
- busy, invalid-param, no-progress, reset, boot, refresh, FIFO drain, and calibration jobs
- all exposed chip/library feature groups: data reads, full scale, ODR, power modes, filters, embedded functions, offsets/biases, FIFO, source registers, direct register access, selftest, and short stress

## Artifacts

- Full targeted feature run: `docs/reports/artifacts/hil-COM26-20260623-targeted-results.md`
- Full targeted transcript: `docs/reports/artifacts/hil-COM26-20260623-targeted-transcript.txt`
- Poll-edge retest: `docs/reports/artifacts/hil-COM26-20260623-poll-edge-retest.md`
- Reset/boot retry: `docs/reports/artifacts/hil-COM26-20260623-reset-boot-retry.md`
- Cancel probe: `docs/reports/artifacts/hil-COM26-20260623-job-cancel-probe.md`

## Results

- Full targeted run: 136 PASS, 8 FAIL, 0 UNKNOWN across 144 commands.
- The 8 failures were all early `TIMEOUT` results from a ready-checked sample job split across slow host serial commands: `job start sample`, `job poll 0`, then later positive-budget polls. The device did not crash, did not lose the prompt, and ended `READY`.
- All non-poll feature coverage after that point passed: full-scale ranges, ODRs, power modes, filters, timestamp/pedometer/motion/tilt features, offsets, staged calibration, FIFO config/drain/read, source registers, register access, invalid-input checks, short stress, selftest, refresh, settings, and final health.
- Poll-edge retest verified the corrected diagnostic pattern: direct staged jobs handle `budget=0` no-progress, `budget=1` completion, `BUSY`, invalid parameters, refresh budget splitting, and high-budget completion without crashes.
- Reset/boot retry passed 4/4 with `job run reset 1 60 2`, `health`, `job run boot 1 60 2`, `health`.
- Cancel probe passed 5/5 and confirmed the CLI behavior described below.

## Findings

1. Ready-checked sample jobs are deadline-sensitive when manually split across serial commands.
   The full run proved that host/serial latency can exceed the measurement-ready deadline before the first positive-budget `poll()`. This is not a crash; it is an expected timeout path. For HIL diagnostics, use `job run sample ...` or `job start direct` when testing pure instruction budgeting.

2. `poll(..., 0)` is a no-progress state by design.
   HIL confirmed `job run direct 0 3 0` remains `IN_PROGRESS` until a later positive-budget poll completes it. This is a useful stress case and a caller footgun if an application accidentally keeps passing zero.

3. `job cancel` is a CLI manual-mode cancel, not a driver job cancel.
   Hardware output showed `job cancel` prints `busy=yes` immediately after clearing manual mode; the normal loop then completes the active direct job on the next `tick()`. This is acceptable as a diagnostic command if documented, but the name is easy to misread.

4. No hardware crash, offline transition, watchdog symptom, or prompt loss survived retry.
   One poll-edge retest reset command had a serial prompt-capture timeout, but the next command showed `READY`; the isolated reset/boot retry passed cleanly.

5. IDF direct-register diagnostics had weaker reserved-register guarding than Arduino.
   The IDF CLI allowed direct `rreg`/`wreg`/`dump` access through `0x43..0x48`; this pass aligned it with the Arduino CLI by blocking that reserved range.

## Follow-Up

- Keep the updated targeted suite using direct jobs for step-by-step budget tests.
- Consider renaming or documenting `job cancel` as `job auto`/manual-mode cleanup, or add a real core cancel API only if production code needs it.
- If the HIL rig can power-gate the IMU or hold SDA/SCL, run a separate fault-injection pass for OFFLINE/recover behavior. This pass did not physically fault the bus.
