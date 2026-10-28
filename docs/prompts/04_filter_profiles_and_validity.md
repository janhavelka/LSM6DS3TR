# 04 - Filter profiles, settling and data validity

**Status: IMPLEMENTED (software); hardware NOT RUN; qualified gyro HPF unresolved.**
Original execution instructions follow for
`C:\Users\Honza\Documents\Projects\LSM6DS3TR`.
Preparation baseline: `e262bf22f3d0cf035fbbca10f45b3e0752bf2bac`.
Historical audit tip: `035eff530e72ea43e4b42b02df0ea2ff05e33383`.
Execution on 2026-10-28 started from fetched synchronized `10cbe28` after
reviewing completed 01-03. Scope, source audit, verification and implementation
resolution are recorded in the [execution record](README.md#prompt-04-execution-record).
Gyro HPF supports explicit unqualified direct reads only; its bounded qualified
settling rule remains an unresolved source/physical gate. Hardware is **NOT RUN**.
Read [the complete execution index](README.md), including its exact verification
commands and Git/CI requirements. **Requires completed 01-03**, whose current
implementations and resolution SHAs must be reviewed before editing shared
profile, FIFO interpretation or event-routing code.

## Execute this task

Complete typed accelerometer output/filter choices, gyroscope LPF1 bandwidth
choices, gyroscope HPF configuration with honest validity semantics, and the
6D low-pass selection of prompt 03. Replace unsupported booleans and implicit
fixed defaults with a small coherent profile and settling contract. This
addresses the filter portion of audit R2 / historical M5. Do not merely remove
the `UNSUPPORTED_PROFILE` checks or assume a data-ready flag proves filter
settling.

## Verified baseline mechanism

`validateProfile()` currently rejects gyro HPF, accel HP/slope and 6D LPF.
`buildCtrl6()` never sets FTYPE; `gyroSettleSamples()` therefore uses only
FTYPE=00. `AccelFilterConfig` contains LPF2, HP/slope and 6D LPF booleans but
no full HPCF/output-path/reference-mode selection. `buildCtrl8()` does not
represent that complete chain. `requiredSettleUs()` uses 14 or 40 accel samples
and the current limited gyro table. `_stepConfigure()` arms its validity gate
from fresh owner time after complete readback, plus quantization margin;
trusted read-only reconcile preserves the prior gate. Preserve that correct
timing boundary while extending the actual filter model.

Current `ConfigurationState::KNOWN` includes valid-output confidence, not just
verified register bytes. `RawSampleResult` records scale and sample quality but
not the complete filter interpretation. Extending filters must address these
semantics explicitly. A register-writing feature alone is not full qualified
sample support.

Read the current public profiles/results/statuses and `validateProfile`,
`buildCtrl1/4/6/7/8`, `gyroSettleSamples`, `requiredSettleUs`, `_stepConfigure`,
`_checkReadyForKnownConfiguration`, `_stepSample`, conversion helpers,
self-test/calibration and restoration. Inspect 01/02 FIFO provenance and 03
6D source routing. Read `test/test_basic.cpp` timing/configuration/calibration/
self-test tests, compile contracts, FakeBus, ProfileCli and both CLIs.

## Source audit before enabling any path

Read maintained topics 05/09/11/12/13, datasheet filter/register definitions,
and AN5130 sections 3.7/3.8 and 3.9 (printed pp. 13-22), Table 9 and the
gyro/transition tables. Inspect the actual PDF when extraction loses columns.
Build a compact source-derived table for output path, bandwidth code,
power-mode/ODR legality, mandatory discard, time gate and event-source path.
Label vendor facts versus conservative library policy.

Important existing limits to preserve or resolve:

- Accel LPF2 bandwidth choices have different settling counts; HP/slope is
  selected by HPCF/path bits, not an independent filter boolean. Reference mode
  has extra preconditions and an initial discard; do not confuse it with an
  ordinary HP output.
- FTYPE is in CTRL6_C, not FUNC_CFG_ACCESS. Power transitions add requirements
  beyond a nominal LPF cutoff; low-power gyro bypasses HPF regardless of its bit.
- `LOW_PASS_ON_6D` affects the orientation engine's path; it is not a new host
  sample filter. Keep its 03 route/dependency contract explicit.
- AN5130's current turn-on/discard tables exclude gyro HPF. A cutoff frequency
  alone does not establish a source-guaranteed settling time. DRDY_MASK covers
  only documented LPF paths; it cannot be used as proof for every new filter.
- BDU protects component byte pairs, not simultaneous XYZ or cross-sensor data.
  Data-ready proves availability, not analog stability or DC response.

## Implementation direction

1. Extend/refactor the existing typed filter profiles so supported output-path
   combinations are explicit and contradictory settings cannot accidentally
   map to the same hardware image. Include documented accel low-pass bandwidth,
   slope/high-pass and reference choices; gyro LPF1 FTYPE and HPF frequency;
   and 6D low-pass selection tied to an admitted 6D profile. Preserve default
   encodings and source compatibility where practical. Avoid a generic DSP
   framework or software filter unrelated to the chip.
2. Validate every combination of mode, ODR, source/path, filter bandwidth,
   reference selection and dependent event/FIFO profile. Compose owned register
   bits through the existing builders, preserving timer/FIFO/interrupt fields.
   Full readback/reconcile, reset/recovery and maintenance restoration must
   verify the same complete image.
3. Replace the limited settling shortcut with a pure source-backed bounded
   calculation for all newly qualified paths. Include transition/discard rules,
   safe integer rounding, overflow/saturation and the post-readback fresh-time
   boundary. Select a conservative documented model where previous state is
   unknown. Preserve trusted reconcile's generation/gate and test low ODRs
   whose settle times exceed ordinary CLI deadlines.
4. Handle actual discard requirements as bounded staged consumption when the
   source requires it; waiting elapsed time must not silently stand in for
   removal of an invalid FIFO record. Preserve FIFO interpretation and explain
   pre-/post-transition data handling. Never mark pre-settling buffered records
   valid merely because their host read occurs after the gate expires.
5. For gyro HPF, first establish whether authoritative source evidence supports
   a bounded qualified-sampling rule. If it does, implement and cite it. If it
   does not, still complete typed configuration/readback and a deliberately
   explicit unqualified read contract: an opt-in direct filtered read may
   return data with unverified-filter quality and no false freshness/settling
   claim; ordinary qualified/ready-checked requests must reject that path.
   Separate known configuration from qualified output cleanly within the
   existing state/quality model; do not redefine current KNOWN or READY_CHECKED
   to mean less. Do not use an arbitrary caller timeout or guessed RC formula
   to manufacture sensor validity. This limited HPF support must be named in
   the support matrix; qualified HPF settling remains an explicit unresolved
   physical/source gate, not a hidden implemented claim.
6. Attach enough immutable filter/path and qualification provenance to new
   sample/FIFO/event results for later interpretation. Conversion must not read
   the driver's newly changed profile to reinterpret an older result. Keep host
   observation time, sensor counter time, record availability and filter
   qualification distinct. Prefer extending existing value types to a second
   cache or data publication system.
7. Audit calibration and self-test against filtered DC behavior. A high-pass
   stream cannot silently serve gravity or zero-rate bias estimation. Either
   reject incompatible calibration requests without I/O, or use a documented
   bounded temporary profile with exact restoration and separate primary/
   restoration failures. Self-test must use the vendor's unfiltered recipe and
   restore the actual verified FIFO/filter/interrupt profile, not a newly
   staged desired profile. No automatic reuse of a bias estimate across a
   filter change without a documented contract.
8. Update profile CLI grammar/equality, display of active filter/quality and
   derived deadlines, both frameworks' command behavior and checker expectations.
   The diagnostic UX must make unqualified HPF reads explicit. Reuse small
   builders/table helpers; delete obsolete fixed-column assumptions rather
   than adding scattered exceptions for each new filter.

## Required regression evidence

- Independent tables for every admitted accel path/HPCF, gyro FTYPE/HPF mode,
  ODR/power combination and 6D dependency. Exact wire-image assertions,
  invalid enums/contradictions/reserved bits, and zero-I/O rejection tests.
- Verify source table settling counts, plus transition/discard and quantization
  boundaries, including 1.6 Hz, high ODRs, near-uint64 limits, changed poll
  budgets, zero budget and callbacks that consume time. Configure/restoration
  gates begin after readback; trusted reconcile does not extend them.
- Unqualified gyro HPF can never publish ordinary READY_CHECKED/settled
  evidence; explicit opt-in preserves raw data with truthful quality. Default
  samples retain their existing contract. Test mixed quantities and FIFO
  records so one qualified channel does not qualify another.
- Before/after filter changes, preserve original-result interpretation; FIFO
  transition samples and reference-mode first sample are consumed/discarded
  with explicit effects, not converted into valid data accidentally.
- Calibration rejection or temporary-profile normalization/restoration;
  self-test primary/restoration errors; configuration mismatch; failure,
  ambiguous write/read, cancel and deadline at every new phase. Assert exact
  callback ceilings and no retries, no stale publication and no leaked profile.
- Both CLI surfaces reject atomic invalid edits and calculate sufficient
  bounded deadlines from the profile. Preserve all 01-03 and baseline tests,
  compile-time ownership/size bounds and native test registration.

## Completion

Run index validation and exact-final-commit CI. Update source tables/ambiguities,
support matrix, quality/state Doxygen, README/CLI guidance, AGENTS, changelog
and audit R2 with exact admitted scope and resolution SHA. Distinguish normal
qualified paths from explicit unqualified HPF support; never mark remaining
qualification work closed just because registers can be read back. Update
HIL procedures for actual filter transients/6D response without claiming a run.
Commit/push owned changes and verify sync. Reconcile all four prompt statuses
and audit R2 coverage; no branch deletion, release or hardware action is part
of completing this software sequence.
