# 01 - Typed motion FIFO and bounded acquisition

**Status: SOFTWARE IMPLEMENTED; FINAL VERIFICATION PENDING.** Original execution path:
`C:\Users\Honza\Documents\Projects\LSM6DS3TR`.
Preparation baseline: `e262bf22f3d0cf035fbbca10f45b3e0752bf2bac`.
Historical audit tip: `035eff530e72ea43e4b42b02df0ea2ff05e33383`.
Read [the complete execution index](README.md) first; its source authority,
Git/CI commands, invariants and closeout requirements are part of this prompt.
No predecessor is required. Record the actual fetched execution baseline.

## Execute this task

Implement a complete typed FIFO acquisition path for internal accelerometer
and gyroscope data, using the existing owner-driven operation engine. Deliver
a useful implementation and tests, not just schema declarations or a proposal.
This resolves the motion-acquisition portion of audit R2 / historical H4,H7.
Keep special fourth datasets unsupported until prompt 02 and interrupts until
prompt 03. Default polling-snapshot behavior must remain unchanged.

## Recheck the current mechanism

At the baseline, `Config.h::FifoProfile` contains only `enabled`;
`validateProfile()` rejects any enabled FIFO; `_prepareManagedImage()` writes
zero to FIFO_CTRL1..5; `_stepFifoPurge()` consumes and discards words, with
status/pattern/loss evidence. It is not a typed acquisition implementation.
`MANAGED_REGISTER_COUNT` is 33 and `_stepConfigure()` currently does a linear
write-all/read-all pass. Enabling FIFO by filling those array slots alone would
activate collection before the complete sensor profile is established and
would misinterpret buffered words after configuration changes.

Read `Config.h`, `LSM6DS3TR.h`, `Status.h`, `CommandTable.h`; current builders,
`validateProfile`, `_prepareManagedImage`, `_stepConfigure`, `_stepFifoPurge`,
`_read`, `_writeByte`, `poll`, `_finish`, `_fail`, cancellation and result
handling in `src/LSM6DS3TR.cpp`. Trace reset/boot/recovery, self-test restoration,
reconciliation, power-down and raw diagnostics into the same profile owners.

Read `test/test_basic.cpp` cases for complete-profile clearing, failure at every
configure stage, ceilings, identity mismatch, raw invalidation and all FIFO
purge cases; inspect their `RUN_TEST` registrations. Read
`test/support/FakeBus.h`, `test/test_compile_contracts.cpp`,
`test/test_profile_cli.cpp`, `examples/common/ProfileCli.h`, both CLIs, and
the three contract/coverage checkers before changing their expectations.

## Source audit before implementation

Use maintained topics 04/05/06/09/10/12/13, datasheet pp. 34-38, 54-58 and
79-81, and AN5130 sections 8.1-8.6, printed pp. 74-92. Record the exact
supported modes, ODR/decimation relationships, pattern generation, empty/full
semantics, FIFO register-read rounding and transition discard requirements.
The FIFO is untagged, 4 KiB / 2048 words. `FIFO_PATTERN` identifies the next
word, not a sample timestamp. Four dataset decimators encode disabled, 1, 2,
3, 4, 8, 16 and 32; only motion datasets are admitted here. Do not use a
different IMU's tagged decoder or infer simultaneous axes from BDU.

AN5130 8.3.1 requires draining before Bypass/reconfiguration; 8.4 specifies
status and pattern before consumption and forbids data reads while empty.
Resolve the overrun/count-zero case through existing ambiguity policy. Audit
source-backed burst auto-rounding before using larger FIFO reads. Preserve
known count/EMPTY contradiction detection; a raw zero count is not enough.

## Implementation direction

1. Extend the existing profile with typed, validated motion FIFO mode, ODR,
   gyro/accel decimation and watermark. Admit Bypass, FIFO stop-on-full and
   Continuous modes with normal internal trigger and full 16-bit motion data.
   Keep reserved/triggered modes, external datasets, special fourth datasets,
   high-byte packing and stop-on-threshold variants rejected here. Specify
   zero-watermark behavior from source, and reject cross-field combinations
   whose data cadence cannot support the interpretation being promised.
2. Extend the existing register builders/image and configuration stages. Use
   small named phases or helpers when the linear image assumption breaks;
   avoid separate configure engines for each feature. Establish safe disabled
   collection, program sensor/FIFO interpretation, verify it, observe settling
   and transition discard policy, then activate the selected FIFO mode at a
   documented point. Keep desired versus verified generation separate.
3. Define bounded reconfiguration while FIFO data remain. Follow the vendor
   drain/Bypass recipe with a fixed work limit and explicit truncation/loss
   result, or reject the transition before changing interpretation and require
   explicit bounded maintenance first. Account for arrivals between observations;
   no infinite drain-until-empty, silent FIFO flush or claim that old queued
   samples use the new scale. Apply the same rule to reset/self-test/power-down
   paths that necessarily interrupt collection. Reconciliation stays read-only.
4. Add one tokened `start*` FIFO-acquisition job within `JobKind`/`poll`. Use a
   caller-owned bounded output buffer and/or a small fixed terminal payload,
   explicit maximum words/records, and a public hard callback-ceiling helper.
   Document buffer lifetime through terminal consumption, null/capacity/size
   validation and whether storage is scratch before success. No heap buffer or
   hidden background queue. Keep every receive within 32 bytes.
5. Build one pure bounded pattern/schema helper used by acquisition. Decode
   from the verified profile and observed starting pattern. Return complete
   typed motion records with immutable scale/profile provenance. Specify partial
   XYZ boundaries, alignment, retained versus discarded words and end-of-buffer
   behavior; do not label incomplete data as a valid vector. Pattern length and
   index arithmetic must have an explicit finite bound, including factor 3.
6. Capture pre/post status, starting/ending pattern, confirmed words consumed,
   possible consumption, truncation and overrun. A callback failure can consume
   FIFO data even when no bytes reach the host; never retry that read internally
   or claim continuity afterwards. A failed/cancelled job publishes no successful
   batch, but preserves effect/loss evidence. Prevent stale caller output from
   being mistaken for a new result. Hardware consumption is not rollbackable.
7. Distinguish capture/read time from sample time and freshness. Do not synthesize
   timestamps by spacing all records at the configured ODR. Overrun or a pattern
   discontinuity prevents a lossless claim; a proven coherent record can remain
   distinguishable from global stream continuity if the contract explicitly
   supports that distinction.
8. Preserve destructive purge as maintenance. Reuse safe shared status decoding
   and effect accounting where appropriate, but do not turn purge into a second
   acquisition owner. Extend profile CLI equality/validation and both native
   and Arduino diagnostic commands with identical bounded behavior. Keep new
   chip protocol in the core, never as raw-register recipes in the examples.

## Required regression evidence

- Independently specified wire images and pattern vectors: gyro-only,
  accel-only, equal rates, AN5130 mixed-rate examples, each admitted decimation,
  starting at every pattern position, invalid enum/ODR/source combinations,
  threshold bounds and maximum supported pattern. Pure validation performs
  zero I2C and preserves the caller's profile on failure.
- Behavioral FIFO fake with actual queued words and independent destructive
  output semantics; empty, nonempty, full, overrun/count-zero, contradictory
  EMPTY/count, short output capacity, partial vector boundaries, concurrent
  arrivals, pattern mismatch and wrap. Test callback chunks at the transfer
  limit and prove no data read is issued while EMPTY is asserted.
- Every configure/reconcile/read stage: address and generic NACK/timeout,
  ambiguous write/consuming read, identity mismatch, cancellation before/after
  effects, absolute deadline, zero/one/large changing poll budgets and final
  callback ceiling. Assert exact attempted callbacks and terminal evidence.
- Buffer lifetime, invalid requests, no output on failure, no stale result,
  exactly-once tokens, pending-result backpressure, rebind/end invalidation,
  reset/recovery/self-test restoration and acquisition after explicit recovery.
  Reconfiguration must never interpret buffered old-scale data as new-scale.
- Keep existing snapshot/scaling, self-test, calibration, purge and CLI tests
  passing. Update compile-time fixed sizes with a measured, explained bound.

## Completion

Run the index's focused/full validation and exact-final-commit CI. Update the
support matrix specifically for motion FIFO while leaving fourth datasets,
routes and other omitted features unsupported. Replace the coverage checker's
old blanket unsupported assertion with equivalent precise supported/unsupported
checks. Update timing ceilings, integration/CLI documentation, AGENTS as
appropriate, changelog and audit R2 with this partial resolution. Record
hardware NOT RUN separately; add physical qualification cases without claiming
that fake FIFO behavior validates the silicon. Commit/push owned changes and
confirm upstream sync. Mark this prompt implemented only when its software
acceptance is met, then proceed to 02.
