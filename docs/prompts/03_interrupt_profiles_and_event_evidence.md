# 03 - Interrupt profiles and event evidence

**Status: implementation/verification PENDING; hardware NOT RUN.** Original execution instructions follow for
`C:\Users\Honza\Documents\Projects\LSM6DS3TR`.
Preparation baseline: `e262bf22f3d0cf035fbbca10f45b3e0752bf2bac`.
Historical audit tip: `035eff530e72ea43e4b42b02df0ea2ff05e33383`.
Execution started on 2026-10-27 from fetched, synchronized baseline
`5b3d6bc1a6d8fe5831e537ba1bc92cef65dad5d5`, after completed motion-FIFO and
fourth-dataset/counter implementations. Their resolution SHAs and this task's
source audit, software acceptance evidence and final resolution are recorded
in the [execution record](README.md#prompt-03-execution-record). Current
implementation/final verification is **PENDING**; physical qualification is
**NOT RUN**.

Read [the complete execution index](README.md), whose source, architecture,
Git/CI commands and closeout rules apply. **Requires completed 01 and 02**;
record their resolution SHAs and inspect the actual current source.

## Execute this task

Implement typed INT1/INT2 electrical and routing configuration for the already
supported motion/temperature DRDY, FIFO status and step/timestamp sources, plus
a complete bounded 6D/4D orientation event path. Provide explicit event-source
observation and acknowledgement semantics through the existing operation
engine. This addresses the interrupt-routing part of audit R2 and provides
the typed consumer for prompt 04's 6D low-pass filter option.

Scope includes DRDY, FIFO watermark/full/overrun, step detect/count overflow,
timestamp end-counter indication and 6D/4D. Admit each only on the pin and with
dependencies actually supported by the silicon and the implemented profile.
Step-delta batching, boot notification, tap, wake-up/free-fall, activity,
significant motion, tilt/wrist tilt, sensor hub and DEN are not implicitly
requested. Keep them explicitly unsupported, including raw route-mask attempts.

## Recheck the finding and source authority

At preparation, `InterruptProfile` has only `enabled`; `validateProfile()`
rejects enabled routing. `_prepareManagedImage()` zeros DRDY_PULSE_CFG_G,
INT1_CTRL, INT2_CTRL, TAP_CFG, TAP_THS_6D, MD1_CFG and MD2_CFG. `buildCtrl3()`
only sets IF_INC/BDU, so it does not represent pad polarity/drive. There is no
typed event-source operation or captured-event result. A writable register
constant is not existing feature support.

Inspect current Config/Status/public result types, builders/managed image,
`_stepConfigure`, `_stepSample`, reset/maintenance restoration,
`_safeDiagnosticWrite`, `_read`, `poll`, cancellation and `takeResult`.
Read current 01/02 profile/FIFO/counter ownership, FakeBus and native/CLI tests,
profile equality and mirrored CLI commands. Identify shared bits before editing
so interrupt configuration does not overwrite FIFO timer resolution or filter
settings.

Use maintained topics 02/05/06/09/10/12/13 and the ST datasheet register
definitions for CTRL3_C, CTRL4_C, DRDY_PULSE_CFG_G, INT1_CTRL, INT2_CTRL,
TAP_CFG, TAP_THS_6D, D6D_SRC, FUNC_SRC1/2 and MD1_CFG/MD2_CFG; AN5130
sections 4, 5 (6D/4D), 6.1/6.5 and 8. Read printed pages for each field and
recipe rather than importing another device's masks.

Record a route/dependency/clear-semantics matrix before coding. Global pad
polarity and drive controls, DRDY pulse/latched mode, embedded-event latch,
source-specific durations and INT2-to-INT1 combining are different concepts.
Do not invent independent per-pin controls when hardware shares them or a
universal pulse width. Follow topic 12's D4D_EN polarity resolution. Timestamp
end-counter acknowledgement resets the counter; it is not an innocuous status
read. Hardware state can change while sources are observed.

## Implementation direction

1. Extend the existing replayable profile with typed electrical mode and
   supported per-pin routes, plus a complete 6D/4D profile: source-backed
   orientation threshold, dimension policy, latch and route dependencies.
   Represent only real hardware choices, validate combinations atomically and
   preserve compatibility of the disabled/default profile. A generic byte mask
   with no source ownership is insufficient.
2. Keep one owner for shared register fields. Compose the complete expected
   image from FIFO, timer/pedometer, filters and interrupts using small named
   builders. Reject incompatible source requests before I/O; a route cannot
   enable an otherwise unsupported or unconfigured engine behind the caller's
   back. Threshold units/rounding and source ODR prerequisites are documented
   and checked. Keep `lowPassOn6d` unsupported until prompt 04 completes its
   actual filter contract; default 6D source behavior here must be explicit.
3. Extend configure stages to avoid exposing half-applied routes: establish
   safe route masking, configure/verify dependencies, then enable/verify the
   selected routes with explicit handling of pre-existing source evidence.
   Do not silently read-to-clear all sources as a convenience. Readback must
   compare owned stable bits, not dynamic event flags. Reconcile is read-only
   and must not acknowledge events or reset counters.
4. Add a bounded tokened event-read operation or extend the appropriate current
   operation/result owner. Return raw source bytes, decoded supported causes,
   profile generation and host observation time. Make read-to-clear effects and
   possible lost evidence visible on success/failure/cancellation. No implied
   simultaneous snapshot across separate source-register callbacks. Source
   flags and physical pad level are different evidence.
5. Provide explicit acknowledgement only where required by the source. A
   timestamp-reset acknowledgement must use 02's counter-epoch/effect handling
   and invalidate old continuity. Avoid a generic clear-all method with hidden
   counter/FIFO side effects. Distinguish merely observing a nonlatching source
   from clearing a latched one; do not promise that every missed pulse is
   recoverable by polling.
6. Keep MCU pin configuration, pull resistors, interrupts, debounce and task
   scheduling in the application. Core APIs remain not ISR-safe: an application
   ISR may notify its owner, which starts/polls jobs outside the ISR. Add a
   small diagnostic example of that boundary without creating a driver task,
   event queue or GPIO callback dependency. Do not make successful software
   configuration claim the board's electrical route has been measured.
7. Define reset/recovery/self-test/calibration/power-down effects on routes and
   latched events, restore the verified profile, and preserve any lost/pending
   event evidence that can actually be known. A temporary maintenance profile
   must not leak back into the desired/verified production profile.
8. Extend CLI profile grammar/equality/output and both command implementations
   with bounded route/event operations. Keep source-clearing commands visibly
   distinct from cache-only diagnostics; update contract checkers with the real
   supported surface rather than allowing arbitrary register aliases.

## Required regression evidence

- Source-derived route/pin table, including every admitted source on every
  supported pin, invalid pin/source pairs, shared polarity/drive constraints,
  DRDY pulse/latched behavior, 6D/4D threshold encodings and dependencies.
  Default disabled routing must still clear all owned route bits exactly.
- No unrelated FIFO/filter/timer bits are overwritten. Configure and full
  readback/reconcile verify electrical controls and all routes; mismatched
  register evidence names the exact register and values.
- Independent fake source latches/read-clear rules, events arriving between
  reads, simultaneous causes, overrun/watermark combinations, 6D signs,
  step overflow and timestamp end-counter acknowledgement/epoch reset.
  Decode arbitrary raw bytes only into supported cause fields.
- Fault, ambiguous effect, deadline and cancellation at every added stage,
  including source read and reset-acknowledgement failure. Prove no duplicated
  consumption, no stale event output, no silent read-clear retries, exact
  transaction budgets, zero-budget behavior and exactly-once terminal results.
- Maintenance route masking/restoration and no permanent state corruption;
  active/pending-result exclusion; external-owner notification example consumes
  no core API in ISR context. Preserve all 01/02 and baseline tests.
- CLI invalid edits preserve the previous draft and perform zero I2C; both
  frameworks expose the same typed semantics and source/effect evidence.

## Completion

Execute index validation and exact-final-commit CI. Update the supported route
matrix, pin integration guidance, public Doxygen, CLI/HIL procedures, AGENTS,
changelog, audit R2 and prompt status with actual resolution SHA. Explicitly
retain unsupported route/engine combinations. Physical pulse timing, electrical
levels, interrupt loss under load and actual orientation response remain
separate qualification cases; mark NOT RUN unless matching authorized logs
exist. Commit/push owned changes, confirm upstream sync, then continue to 04.
