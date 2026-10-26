# LSM6DS3TR-C optional feature implementation sequence

Prepared: 2026-10-25. **Status: prompt 01 IMPLEMENTED (software); prompts 02-04 NOT IMPLEMENTED.** These files are executable
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

## Coverage and execution order

| Order | Finding / present mechanism | Requested implementation | Dependencies | Status |
| --- | --- | --- | --- | --- |
| [01](01_fifo_profile_and_acquisition.md) | R2 / old H4,H7: original enable-only profile and purge-only mechanism replaced. | Typed motion FIFO profile, safe replay, pattern interpretation and bounded caller-buffer acquisition. | None | IMPLEMENTED (software); hardware NOT RUN |
| [02](02_fourth_dataset_and_counter_provenance.md) | R2 / old H4: no fourth-dataset selector/decoder or managed timestamp/pedometer dependencies. | Temperature versus timestamp-plus-step records, explicit counter lifetime and mutually exclusive schema. | 01 | NOT IMPLEMENTED |
| [03](03_interrupt_profiles_and_event_evidence.md) | R2: `InterruptProfile` is only an enable flag; managed routes are zero; no typed event-source result. | Complete electrical/routing profile for supported DRDY/FIFO/step sources and a bounded 6D/4D event path. | 01, 02 | NOT IMPLEMENTED |
| [04](04_filter_profiles_and_validity.md) | R2 / old M5: gyro HPF, accel HP/slope and 6D LPF rejected; FTYPE/HPCF choices are incomplete; settling helper covers the restricted defaults. | Typed filter paths, correct settling/readiness provenance, and 6D filter integration. | 01, 02, 03 | NOT IMPLEMENTED |

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
