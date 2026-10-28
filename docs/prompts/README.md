# LSM6DS3TR-C optional feature implementation sequence

Prepared: 2026-10-25. **Status: prompts 01-04 IMPLEMENTED (software); hardware qualification NOT RUN.** These files are executable
engineering instructions; creating them does not add features or qualify
hardware. They implement the optional work described by R2 in the
[retained branch audit](../RETAINED_BRANCH_AUDIT.md).

## Repository and pinned evidence

- Repository: `C:\Users\Honza\Documents\Projects\LSM6DS3TR`, remote
  `https://github.com/janhavelka/LSM6DS3TR`.
- Prompt preparation baseline: `main` at
  `e262bf22f3d0cf035fbbca10f45b3e0752bf2bac`, fetched and synchronized with
  `origin/main`; initially clean.
- Audit's implementation baseline:
  `513c5641d9b7dda63fddc27267ea62aa8d03ff93`. The intervening audit commit
  changes documentation, not driver behavior.
- Historical exploration tip:
  `035eff530e72ea43e4b42b02df0ea2ff05e33383`, branch
  `origin/exploration/lsm6ds3trc-industry-readiness`; only its report/prompts
  are unique. Do not merge it or use its obsolete API as implementation.
- Historical IDF tip:
  `61a47ad4961d56ac8b4df5efbb6d3d77868d32ae`, branch
  `origin/feature/lsm6ds3tr-idf-port`, already an ancestor of main.

At execution, read the entire current AGENTS, CI workflow, this index, the
selected prompt, and changed owning sources. Fetch/inspect main and upstream;
continue from the actual latest commit and record it. Baseline SHAs are evidence,
not instructions to reset to an old checkout. Later prompts also inspect the
committed results of earlier prompts. Preserve unrelated changes, work on the
existing branch, and let the lead own Git mutations when using subagents.

## Prompt 01 execution record

- Execution date: 2026-10-26; actual workspace is
  `C:\Users\HonzovoSpectre\Documents\Projects\LSM6DS3TR`.
- Fetched execution baseline: `main` and `origin/main` at
  `362b140990bc4ba2bc7f32cef50f8d9f30b4d3cc`, clean and synchronized.
- Scope: typed internal-motion FIFO only. Fourth datasets/counters, routes and
  filter extensions remain separate prompts 02-04, not silently enabled here.
- Implementation resolution: `0cc5ef2e5f395dbf87c6d18aaf81e49112c73f06`.
  [Implementation CI](https://github.com/janhavelka/LSM6DS3TR/actions/runs/40387449884)
  passed all seven jobs, including native ESP-IDF 5.4.4 for S2 and S3.
- Closeout adds named configuration phases and independent acceptance coverage
  for every motion decimator wire image, all 105 maximum-pattern positions,
  the Continuous occupancy boundary, and exact reset/boot/recovery ceilings.
  Native verification: **142/142 tests passed**. The exact final source revision
  is the commit containing this closeout; its own GitHub CI check is the final
  verification record. The implementation run above verifies only its stated SHA.
- Local validation passed: all three Arduino targets (`esp32s3dev`,
  `esp32s3hil`, `esp32s2dev`), core timing/version/CLI/IDF/chip coverage checks,
  three IDF checker tests, 18 HIL-runner tests, runner Python compilation,
  strict Doxygen generation, package validation (37 files), and whitespace.
  The clean documentation wrapper passed initially; a repeat encountered a
  Windows read-only generated directory. Direct `doxygen Doxyfile` passed;
  clean-wrapper execution also passed in CI. Native ESP-IDF was verified in CI,
  not locally, because this shell has no exported IDF SDK.
- Hardware: **NOT RUN**; repeatable qualification cases are in
  [HIL validation](../HIL_VALIDATION.md#motion-fifo-qualification).

## Prompt 02 execution record

- Execution date: 2026-10-26; actual workspace is
  `C:\Users\HonzovoSpectre\Documents\Projects\LSM6DS3TR`.
- Fetched execution baseline: `main` and `origin/main` at
  `bce83376aa01d0e9836f9175a9993e93128956e5`, clean and synchronized.
- Prerequisite 01: implementation
  `0cc5ef2e5f395dbf87c6d18aaf81e49112c73f06` and acceptance closeout
  `bce83376aa01d0e9836f9175a9993e93128956e5`.
- Scope: typed temperature or timestamp/step fourth dataset; shared internal
  or step-event triggers; raw modular counters with resolution/epoch/effect
  evidence; fixed bank-A pedometer policy and explicit bounded reset.
  Interrupt routes, external datasets and filter extensions remain prompts
  03/04 or unsupported capabilities.
- Source audit: AN5130 Tables 73/74 were visually inspected in the actual PDF;
  timestamp byte order and unused temperature bytes are documented in topic 10.
  The fixed pedometer policy follows datasheet bank-A field definitions and
  the power-down write requirement. Timestamp dependency ambiguity remains
  explicit; the joint dataset requires accelerometer ODR at least 26 Hz.
- Local software verification: **170/170 native tests passed**, all contract
  guards passed, IDF checker self-tests **3/3**, and HIL-runner tests **18/18**.
  Strict Doxygen generation passed after clearing Windows read-only attributes
  only inside the verified generated-documentation directory. This is host
  software evidence, not a physical HIL run.
- Implementation resolution: `f7c2d5d87a505d428a379b6628c447b62ec5a1b8`. The following closeout
  commit records this SHA without changing the implemented behavior.
- Final local verification passed: **170/170 native tests**, all three Arduino
  targets (`esp32s3dev`, `esp32s3hil`, `esp32s2dev`), core timing/version/CLI/IDF/
  chip-reference guards, three IDF checker tests, 18 HIL-runner tests, runner
  Python compilation, strict Doxygen, package validation (37 files and 20 linked
  Markdown documents), and whitespace. No standalone ESP-IDF SDK is exported
  in this shell; native ESP-IDF 5.4.4 S2/S3 builds are verified by CI.
- Exact-final-commit CI is the completion gate: inspect all seven jobs attached
  to the commit containing this closeout record. The local evidence above does
  not substitute for that commit's own CI result. No release or tag was created.
- Hardware: **NOT RUN**; repeatable new cases are in
  [fourth-dataset qualification](../HIL_VALIDATION.md#fourth-dataset-and-counter-qualification).

## Prompt 03 execution record

- Execution started: 2026-10-27; actual workspace is
  `C:\Users\HonzovoSpectre\Documents\Projects\LSM6DS3TR`.
- Fetched execution baseline: clean `main` and `origin/main` at
  `5b3d6bc1a6d8fe5831e537ba1bc92cef65dad5d5`.
- Prerequisites: motion FIFO implementation
  `0cc5ef2e5f395dbf87c6d18aaf81e49112c73f06` and closeout
  `bce83376aa01d0e9836f9175a9993e93128956e5`; fourth-dataset/counter implementation
  `f7c2d5d87a505d428a379b6628c447b62ec5a1b8` and closeout
  `5b3d6bc1a6d8fe5831e537ba1bc92cef65dad5d5`. The latter exact revision passed
  all seven CI jobs, including native ESP-IDF S2/S3 builds.
- Scope: typed shared pad controls, supported direct source routes, complete
  6D/4D orientation and bounded event observation/acknowledgement evidence.
  Timestamp acknowledgement retains the existing explicit reset/epoch owner.
  External datasets, unrelated event routes and 6D LPF2/filter extensions remain
  unsupported or assigned to prompt 04.
- Source audit before coding: route/dependency/clear matrix in maintained topic
  06, based on visually inspected datasheet register tables and AN5130
  orientation/pedometer/timestamp sections. No readable timestamp-end flag was
  invented. The default 6D path is LPF1 ODR/2; global DRDY mode and embedded LIR
  remain separate concepts.
- Implementation resolution: `d35165d4d12e8d2a1be6e323326b4df966fac00b`.
  Local software closeout completed on 2026-10-28. The following documentation
  commit records that resolution without changing implemented behavior.
- Final local verification passed: **202/202 native tests**, all three Arduino
  targets (`esp32s3dev`, `esp32s3hil`, `esp32s2dev`), core timing/version/CLI/IDF/
  chip-reference guards, three IDF checker tests, 18 HIL-runner tests, runner
  Python compilation, strict Doxygen, package validation (37 files and 20 linked
  Markdown documents), and whitespace. Source review added raw-route escape
  fences and preserved FIFO loss evidence; target builds verified Arduino macro
  compatibility and the existing-task FreeRTOS ISR notification example.
- Exact-final-commit CI is the completion gate: inspect all seven jobs attached
  to the commit containing this closeout record. Native ESP-IDF 5.4.4 S2/S3
  builds are verified there; no standalone SDK was exported in the local shell.
  Local tests do not substitute for that exact commit's CI. No release/tag was
  created; the backward-compatible additions use unreleased 2.3.0 metadata.
- Hardware: **NOT RUN**; see
  [interrupt/orientation qualification](../HIL_VALIDATION.md#interrupt-and-orientation-qualification).
  Electrical levels, pulse durations, interrupt loss under load and orientation
  response remain distinct physical gates.

## Prompt 04 execution record

- Execution date: 2026-10-28; actual workspace is
  `C:\Users\HonzovoSpectre\Documents\Projects\LSM6DS3TR`.
- Fetched execution baseline: clean synchronized `main` / `origin/main` at
  `10cbe2822a9f86f45a7d0327ddf42b8e3c91bb81` (power-down routing follow-up).
- Reviewed prerequisite implementation resolutions: motion FIFO
  `0cc5ef2e5f395dbf87c6d18aaf81e49112c73f06`, fourth datasets/counters
  `f7c2d5d87a505d428a379b6628c447b62ec5a1b8`, and interrupt/orientation
  `d35165d4d12e8d2a1be6e323326b4df966fac00b`; their acceptance records above
  remain separate from this filter change.
- Source audit: visually inspected AN5130 Tables9/12-16 and datasheet Figure8.
  Corrected the extracted merged LPF2 input column; all four HPCF bandwidths
  admit both low-latency and low-noise input. FTYPE belongs to CTRL6_C.
  Reference mode disables LPF2/HP and is relative to a captured initial vector.
- Implemented scope: all admitted accel LPF1/LPF2/HPCF/input/analog, slope/HP/
  reference choices, all gyro LPF1 FTYPE codes, independent orientation LPF2,
  complete image/readback/restoration, conservative source-backed time gates,
  staged first reference discard and immutable per-quantity qualification.
- Gyro HPF has typed configuration/readback and explicit direct-only opt-in.
  `FILTER_UNQUALIFIED` does not weaken `KNOWN`; `FILTER_UNVERIFIED` has no
  freshness evidence. Gyro HPF FIFO and bias calibration remain rejected.
  Qualified gyro HPF settling is an **unresolved source/physical gate** because
  AN5130 excludes it; register support does not close that claim.
- Default encodings/source fields are preserved. Canonical inactive filter
  fields are now enforced; no alias profile silently represents a bypassed
  bandwidth. The existing unreleased 2.3.0 feature metadata is retained; no
  release, tag, branch deletion or hardware action is part of this work.
- Local software verification: **236/236 native tests**, including 34 added
  filter/operation/CLI regressions with source-independent tables, callback-time
  and zero-budget gates, reference/FIFO transition loss, fault/cancel/deadline
  matrices, exact ceilings and vendor-recipe restoration. All three Arduino
  targets, core timing/version/CLI/IDF/chip-reference guards, three IDF checker
  self-tests, 18 HIL-runner tests, Python compilation, strict Doxygen, package
  validation (37 files / 20 linked documents) and whitespace passed. The generated
  docs directory needed its Windows read-only attributes cleared before the
  clean Doxygen wrapper; no source or check was bypassed.
- Implementation resolution: `10f6898386dfe510254232d0be677643309e40ed`. This closeout
  records that immutable SHA without changing implementation behavior.
  Exact-final-commit CI is the final gate: inspect all seven jobs attached to
  the commit containing this closeout record, including native ESP-IDF 5.4.4
  S2/S3. The local evidence above does not substitute for that exact revision's
  CI; no exported local IDF SDK is available.
- Hardware: **NOT RUN**; repeatable cases are in
  [filter qualification](../HIL_VALIDATION.md#filter-transient-and-qualification-campaign).

## Independent audit of prompts 01-04

- Audit date: 2026-10-28. Fetched, clean and synchronized baseline:
  `fc8bdef87fb2e1ccecf79c5df3f2a4b219ee216e` on the existing `main` branch.
  All four original prompt bodies were reread in full. Three independent
  reviewers inspected FIFO/counters, interrupts/lifecycle and filters/CLIs
  against current code, tests, vendor references and implementation diffs.
- Confirmed corrections: self-test now checks current bank/identity/FIFO Bypass
  before writes; gates reserve a replacement output after vendor discard counts;
  failed output reads retain possible BDU-held-pair evidence until bounded
  release; sample/calibration acknowledgements set the common effect flag;
  rounded FIFO bursts cannot claim reads of later embedded-event sources.
  Diagnostic output reads must cover complete low/high pairs. Reconcile remains
  read-only and cannot erase uncertain held-output evidence.
- Added independent complete-set FIFO producer coverage for full-stop behavior,
  retained coherent records and explicit Bypass rearming. Corrected BDU wording
  and the stale statement that all production interrupt routes remain disabled.
- Final local verification passed: **249/249 native tests**, all three Arduino
  targets, core timing/version/CLI/IDF/chip-reference guards, three IDF checker
  tests, 18 HIL-runner tests, Python compilation, strict Doxygen, package
  validation (37 files / 20 linked documents) and whitespace. The resolution is
  the commit containing this audit record. Its own seven-job CI run, including
  native ESP-IDF 5.4.4 S2/S3, is the exact-revision completion evidence; earlier
  implementation runs do not verify these corrections.
- Changes extend existing profile, operation and fake-bus owners; no new service,
  allocation, transport dependency, release or hardware action was introduced.
  Physical qualification remains **NOT RUN**, and qualified gyro HPF remains
  unsupported without the separate source/physical evidence described above.

## Coverage and execution order

| Order | Finding / present mechanism | Requested implementation | Dependencies | Status |
| --- | --- | --- | --- | --- |
| [01](01_fifo_profile_and_acquisition.md) | R2 / old H4,H7: original enable-only profile and purge-only mechanism replaced. | Typed motion FIFO profile, safe replay, pattern interpretation and bounded caller-buffer acquisition. | None | IMPLEMENTED (software); hardware NOT RUN |
| [02](02_fourth_dataset_and_counter_provenance.md) | R2 / old H4: typed fourth-source schema and fixed timestamp/pedometer ownership added. | Temperature versus timestamp-plus-step records, explicit counter lifetime and mutually exclusive schema. | 01 | IMPLEMENTED (software); hardware NOT RUN |
| [03](03_interrupt_profiles_and_event_evidence.md) | R2: typed global controls, direct source routes and bounded independent event observations. | Complete electrical/routing profile for supported DRDY/FIFO/step sources and a bounded 6D/4D event path. | 01, 02 | IMPLEMENTED (software); hardware NOT RUN |
| [04](04_filter_profiles_and_validity.md) | R2 / old M5: complete typed accel/HPCF and gyro FTYPE; explicit unqualified-only gyro HPF. | Source-backed settling, staged reference discard, immutable qualification and independent 6D LPF2 input. | 01, 02, 03 | IMPLEMENTED (software); qualified gyro HPF unresolved; hardware NOT RUN |

Execute in order because the prompts share the profile image, operation engine,
CLI surface, fixed-memory ceilings and tests. Each prompt delivers useful,
tested behavior; prompt 01 is not a speculative foundation-only change.
Selecting a prompt requests its concrete scope. Do not stop at another plan or
request routine permission to do already-requested software work. Resolve
ordinary API design details from source and current conventions. Where a vendor
claim cannot be proved, retain a narrow unsupported/explicitly unqualified
boundary, implement the rest, and record the precise limitation.

The sequence deliberately excludes sensor-hub transports, wrist tilt, DEN,
high-byte FIFO packing, trigger-switched FIFO modes, general tap/wake/free-fall
engines, and arbitrary register scripting. Those are separate capabilities,
not prerequisites for the selected FIFO/route/filter contracts. Step/timestamp
ownership is included only because prompt 02's fourth dataset needs it; 6D/4D
is included because the existing `lowPassOn6d` option otherwise has no complete
typed consumer. Do not silently expand the feature set.

## Required source authority

Read [chip-reference index](../chip-reference/00_reference_index.md), then
topics 04, 05, 06, 07, 09, 10, 11, 12 and 13 as applicable. The source baseline
is ST datasheet DocID030071 Rev 3 (May 2017) and AN5130 Rev 1 (March 2018),
retained locally as the two PDFs and their `docs/pdf-extracted-md/` extracts.
Use printed vendor page numbers, section/table names and exact source revisions.
When extraction damages tables, inspect the PDF, not an invented reconstruction.
Resolve conflicts in topic 12 and update affected code/tests/reference together.
Check official ST sources if a local conflict cannot be resolved; no secondary
driver overrides the silicon documents. Never borrow a similarly named IMU's
FIFO tag format or filter constants.

Verified current architecture anchors:

- `include/LSM6DS3TR/Config.h`: `DeviceProfile`, `FifoProfile`,
  `InterruptProfile`, `AccelFilterConfig`, `GyroFilterConfig`.
- `src/LSM6DS3TR.cpp`: `validateProfile`, register builders,
  `_prepareManagedImage`, `_stepConfigure`, `_stepResetBoot`,
  `_stepSelfTest`, `_stepFifoPurge`, `_safeDiagnosticWrite`, `_read`, `_writeByte`,
  `poll`, `cancelActiveJob` and `takeResult`.
- `include/LSM6DS3TR/LSM6DS3TR.h`: single job/result ownership, current 33-register
  image, callback ceilings and 32-byte receive/33-byte write limits.
- `test/test_basic.cpp`: real native test runner and explicit `RUN_TEST`
  registration; `test/test_compile_contracts.cpp`: ownership and size ceilings;
  `test/test_profile_cli.cpp`; `test/support/FakeBus.h`: transport/fault model.
- `examples/common/ProfileCli.h`, Arduino CLI `main.cpp`, native
  `examples/idf/basic/main/main.cpp`: mirrored diagnostic consumers.

## Implementation rules for every prompt

Audit each finding again against the actual execution baseline. Explain the
current failing/rejected behavior and the desired observable behavior before
editing. Prefer a small cohesive refactor of the current register builders,
shared decode helpers or state stages over duplicated branches and patches
around their assumptions. Delete obsolete code when replacing its ownership.
No generic feature registry, parallel scheduler, manager layer or future stubs.

Keep the neutral core, zero-I/O admission, one active job/one retained terminal
result, immutable deadline, per-poll and whole-operation callback budgets,
bus-silent cancellation, exactly-once result consumption and external bus/ISR
ownership. Every callback is one physical attempt. No hidden retries, delays,
allocations, ISR callbacks or GPIO/bus configuration in core.

Do not relax an UNSUPPORTED fence until the specific admitted profile has
validation, deterministic encoding, safe sequencing, readback, failure/cancel
semantics, tests and documentation. Defaults and existing snapshot behavior
must remain compatible. Mutable counters and command bits must not enter an
ordinary immutable register equality loop. Distinguish verified configuration,
consumed data, possible effects, data validity and physical qualification.

Derive changed callback ceilings from all stages including failure restoration;
update self-test/reset/recovery/reconcile callers and exact-ceiling regressions.
Keep object/record/buffer bounds explicit; justify any size increase and assert
the new bound rather than deleting size checks. Retain fixed transfer limits
unless a concrete reviewed API change proves an increase necessary.

Extend the existing independent fake model and register-level fixtures. Tests
must assert public behavior, source-derived wire images and outcomes under
failures, not mirror implementation calculations. Register every new suite in
the native runner/build. No production test doubles.

## Executable verification and CI closeout

Run relevant focused native tests while implementing, then the existing CI
commands from the repository root before completion. On Windows:

```powershell
python tools/check_core_timing_guard.py
python scripts/generate_version.py check
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python tools/test_check_idf_example_contract.py
python tools/check_chip_docs_coverage.py
python -m py_compile tools/run_hil.py tools/run_owner_soak.py tools/test_run_hil.py
python tools/test_run_hil.py
.\scripts\pio.cmd test -e native
.\scripts\pio.cmd run -e esp32s3dev -e esp32s3hil -e esp32s2dev
python tools/build_docs.py
.\scripts\pio.cmd pkg pack
python tools/check_package_contract.py
git diff --check
```

Use the existing PlatformIO wrapper/installation; CI pins Core 6.1.19 and the
repository pins pioarduino 55.03.311. Do not install another Core or loosen pins.
CI additionally compiles native ESP-IDF 5.4.4 for S2 and S3. For local SDK checks,
from an exported 5.4.4 environment, use separate build directories:

```powershell
idf.py -C examples/idf/basic -B build/idf-s2 set-target esp32s2
idf.py -C examples/idf/basic -B build/idf-s2 build
idf.py -C examples/idf/basic -B build/idf-s3 set-target esp32s3
idf.py -C examples/idf/basic -B build/idf-s3 build
```

All four implementation prompts change public behavior and need exact-final-
commit CI. The current workflow has push/PR triggers and no manual dispatch:
commit scoped work, push normally, and inspect the matching run through terminal
results, for example `gh run list --workflow ci.yml --commit <full-sha>` followed
by `gh run watch <run-id> --exit-status` and `gh run view <run-id>`.
Confirm all matrix jobs and commit SHA, not an older green run. Fix confirmed
failures without weakening checks; report genuine external blockers. Do not
modify workflow triggers to dodge this requirement or repeatedly dispatch
unchanged suites. GitHub CLI commands are examples; use the available supported
terminal authentication/tooling. Never publish a release/tag merely to run CI.

Update public Doxygen, README, maintained topics/support matrix, CHANGELOG
`[Unreleased]`, audit R2 and this status table with actual scope, resolution SHA,
tests and outstanding physical evidence. Current
`tools/check_chip_docs_coverage.py` deliberately expects the FIFO support row
to say unsupported; replace that assertion with accurate granular expectations
when behavior changes and extend coverage rather than disabling the check.
Update AGENTS' explicit unsupported statements only when the corresponding
complete feature exists. Generated version headers stay generated; decide
SemVer from the actual compatible/breaking API change, without inventing a
release. Commit/push owned files and verify upstream synchronization.

HIL remains a distinct evidence gate: update repeatable procedures for new
software behavior, and report hardware NOT RUN unless real matching logs exist.
Do not flash, reset or use a fixture without its separate authorization. Missing
hardware does not justify leaving implementable software as placeholder work;
it also does not justify claiming physically proven filter/interrupt behavior.

## Documentation preparation checks

The current `library.json` package allowlist and Doxyfile exclude `docs/prompts`.
Keep these instructions repository-only; packaged README/documentation-index
links must use repository URLs. No workflow, package-manifest or production
code changes are necessary just to store this promptset. The preparation change
uses local Markdown/link/whitespace and existing documentation guards; it is
not evidence that any future implementation test above has run.
