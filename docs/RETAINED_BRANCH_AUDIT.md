# Retained audit and port branch review

Review date: 2026-10-23. Reviewed current `main`:
`513c5641d9b7dda63fddc27267ea62aa8d03ff93`.

## Decision

There is no implementation patch stranded on the unmerged exploration branch.
Its sole unique commit adds an audit and hardening prompts. The old native
ESP-IDF port and all retained hardening/release branches are already ancestors
of current main. Most historical defects were addressed by the newer bounded
operation engine, explicit configuration provenance, and current integration
tests.

The reviewed snapshot intentionally rejected FIFO acquisition, fourth-dataset
decoding, interrupt routing and some filters. Subsequent prompt 01 implements
motion acquisition; prompt 02 adds typed fourth-source/counter behavior under
the current architecture. R2 and the execution index record each exact software
gate. Prompts 03/04 add typed routing and filters with explicit unqualified-only
gyro HPF support; its qualified settling remains unresolved. The
deployment gate is physical validation of the actual firmware; retained
campaigns predate these changes.

## Snapshot and graph evidence

The lead fetched `origin`; main matched its upstream and the tree was clean.
Main was the newest remote tip by commit date. Repository AGENTS and CI were
read before edits. No historic checkout, merge, deletion, or code fix occurred.
Counts are main-only / retained-ref-only commits from
`git rev-list --left-right --count main...<ref>`.

| Remote ref after `origin/` | Exact tip | Counts | Disposition |
| --- | --- | --- | --- |
| `exploration/lsm6ds3trc-industry-readiness` | `035eff530e72ea43e4b42b02df0ea2ff05e33383` | 54 / 1 | Unique audit/prompts only; archive reference, then eligible for retirement. |
| `feature/lsm6ds3tr-idf-port` | `61a47ad4961d56ac8b4df5efbb6d3d77868d32ae` | 51 / 0 | Already integrated; eligible for retirement. |
| `hardening/tunnelmonitor-suitability-reaudit` | `c44a91a28e6ee33fcfb4f3178e7551ab31369421` | 30 / 0 | Already integrated; eligible for retirement. |
| `hardening/com26-tunnelmonitor-hil` | `ddab91f15b4d14c6b83126ff69186b37dfc2954e` | 20 / 0 | Already integrated; eligible for retirement. |
| `agent/ci-node24-actions` | `c9116a2e4b05aee8e34741fc0d5a702588b135d2` | 25 / 0 | Already integrated; no lost workflow patch. |
| `agent/consolidate-v2.0.0` | `3a53f44e6cd98255049ffdb03c5ee927d8de77fb` | 23 / 0 | Already integrated; historical release work. |
| `agent/release-v2.0.1` | `2e2d0f8a4de9d833656063f1801074e5553ec824` | 27 / 0 | Already integrated; historical release work. |

Exploration merge base:
`4e0ce5fe177a54c92e203005b08c7e21999d32a3`.
Its unique commit `035eff530e72ea43e4b42b02df0ea2ff05e33383`,
"docs: add LSM6DS3TR-C hardening prompts for industry readiness and validation
sequence", adds `docs/LSM6DS3TRC_INDUSTRY_READINESS_AUDIT.md` and ten prompt
files (1,482 added lines). It changes no source, tests, component files, or CI.
`git cherry main <exploration>` reports `+`; this means the documentation patch
is not patch-equivalent, not that the old findings are still implementation
defects.

The [original report at its exact tip](https://github.com/janhavelka/LSM6DS3TR/blob/035eff530e72ea43e4b42b02df0ea2ff05e33383/docs/LSM6DS3TRC_INDUSTRY_READINESS_AUDIT.md)
supplies the finding IDs below. The review compared its remediation requests
with actual current sources, regression tests, support boundaries, and CI.
Line numbers refer to the reviewed main SHA. They do not describe the older
implementation quoted by the report.

## Finding-by-finding disposition

| Old finding | Current disposition and evidence |
| --- | --- |
| H1: Arduino-bound core | Resolved. `include/LSM6DS3TR/Config.h:12` injects transport; `src/LSM6DS3TR.cpp:594` binds without I/O; operation time is supplied by the owner. Core timing guard and native IDF contract check pass. `examples/idf/basic` and real IDF CI builds now exist. |
| H2: raw writes break hidden state | Resolved within the current diagnostic contract. `src/LSM6DS3TR.cpp:2500` applies `_safeDiagnosticWrite`, blocks active/pending jobs, invalidates configuration before an accepted write, and performs one bounded attempt. `test/test_basic.cpp:2444`, `:2559`, and `:2611` cover unsafe writes, exact register addresses, and ambiguous effects. |
| H3: partial configuration lacks dirty state | Resolved. `_stepConfigure` at `src/LSM6DS3TR.cpp:1035`, `_invalidateConfiguration` at `:1003`, and terminal effect evidence distinguish unknown, applying, settling, and verified state. `test/test_basic.cpp:703`, `:760`, `:870`, and `:906` inject failures/cancellation at every apply stage and reject untrusted sampling. |
| H4: incomplete fourth FIFO dataset | The earlier removal fence was replaced by prompt 01 motion acquisition and prompt 02 typed temperature or timestamp/step decoding. Raw counter widths, resolution, epoch, consumption/loss and explicit reset effects are owned by the same bounded engine. External datasets remain unsupported; R2 and its execution records separate software completion from physical qualification. |
| H5: reset/boot sequencing | Resolved in software. `_stepResetBoot` at `src/LSM6DS3TR.cpp:1244` proves bank/identity, applies required sensor modes, issues the command, arms a fresh post-write inaccessible interval, bounds command polls, and reapplies/verifies the profile. `test/test_basic.cpp:1405`, `:1458`, `:1556`, and `:1612` cover order, waits, failures, and ceilings. Physical current-firmware evidence remains R1. |
| H6: narrow statuses | Resolved by the version-2 status contract in `include/LSM6DS3TR/Status.h:11`: deadlines, stale/pending results, unknown/mismatched configuration, settling, unsupported profiles, indeterminate effects, and typed transport failures are explicit. The old managed-health policy is deliberately outside the driver. |
| H7: blocking FIFO drain | Resolved in software by prompt 01's bounded owner-polled acquisition and separate destructive purge. `startFifoRead` retains caller-buffer, pattern, effect and deadline evidence; `startFifoPurge` remains maintenance. Their fault/limit/cancel regressions and exact revision evidence are recorded under R2; physical qualification is separate. |
| H8: no hardware evidence | Historical statement no longer literally true: `docs/HIL_VALIDATION.md:165` records an S2 campaign at `1419ea2`, and earlier S3 evidence is indexed by the guide/changelog. Neither qualifies current main. Fault injection, alternate strap, mounting, and load limitations remain explicit; see R1. |
| M1: unused tick timestamp | Superseded by `poll(nowMs, maxTransactions)` and caller-specified absolute operation timing; the old ignored timestamp API is absent. `test/test_basic.cpp:499`, `:587`, and `:1163` verify deadline/settling/readiness behavior. |
| M2: copyable driver | Resolved by deleted copy/move and compile-time assertions in `test/test_compile_contracts.cpp:46` onward. |
| M3: Arduino timeout detail loss | Resolved for the maintained single-owner adapter: `examples/common/I2cTransport.h` uses native HAL return status for combined reads/writes, preserving timeout/raw detail. `docs/CODE_AUDIT.md` section 1.3 explains the remaining SDK semaphore ownership assumption; arbitrary concurrent SDK use is not promised bounded. |
| M4: validation/time-source gaps | Resolved for supported API: `DriverConfig` and `validateProfile` reject missing callbacks, zero timeout, invalid modes/ranges and unsupported profiles; owner operation timing replaces the optional Arduino-clock fallback. `test/test_basic.cpp:160`, `:182`, `:372`, and `:2869` exercise the fences. |
| M5: gyro HPF labels | Corrected encodings in `include/LSM6DS3TR/Config.h:100` and exact static assertions in `test/test_compile_contracts.cpp:26`. Prompt 04 adds typed HPF configuration/readback and explicit unqualified direct samples. Qualified HPF settling remains an unresolved source/physical gate; ordinary gyro samples, gyro FIFO and gyro bias calibration reject it. See R2 and the execution record. |
| M6: freshness unclear | Resolved: terminal samples carry validity/freshness masks, sequence, scale and profile provenance. Ready-checked sampling and direct-unverified sampling differ explicitly. `test/test_basic.cpp:1066`, `:1095`, and `:1163` cover atomicity and no false freshness. |
| M7: SPI/broader capability claims | Resolved by current I2C binding and the maintained support matrix, which expressly lists SPI and other silicon-only features as unsupported. No SPI implementation is stranded on the retained branch. |
| M8: absent IDF/CI checks | Resolved: `.github/workflows/ci.yml` runs the IDF checker and its tests, builds native S2/S3 examples with ESP-IDF 5.4.4, checks chip documentation and package contracts, and runs native tests/Arduino builds. |
| L1: SA0 voltage wording | The public address comment now says "SA0 tied high" (`include/LSM6DS3TR/Config.h:40`), not an instruction to connect the core supply. The public enum name `SA0_VDD` remains for compatibility. I/O-domain electrical levels and mode-select behavior are in chip-reference topics 02/03; a future quick-start wiring reminder is documentation polish, not a missing runtime fix. |
| L2: header undefines DISABLED | Resolved: no public-header mutation; `test/test_compile_contracts.cpp:6` defines an application macro, includes Config, and asserts the definition/value survives. |
| L3: hardware/example caveats | Covered by the pin/electrical reference, support matrix, diagnostic example descriptions, and explicit HIL limits. README does not repeat a full wiring checklist; use the maintained pin/electrical topics before bring-up. Current physical acceptance remains R1. |
| L4: fragmented scaling | Exact current sensitivities/units live in the maintained chip reference and conversion helpers; `test/test_basic.cpp:233`, `:253`, and `:270` cover every motion full-scale and temperature boundaries. No alternative scaling implementation needs recovery. |
| L5: package artifact hygiene | Resolved: `.gitignore:4` excludes generated LSM6DS3TR archives; CI packs and validates the allowlisted package through `tools/check_package_contract.py`. |

## Remaining work and acceptance criteria

### R1 - Required for deployment qualification: current-firmware HIL

Owner: fixture/application integration. Run the supported targeted CLI campaign
and owner soak from [HIL validation](HIL_VALIDATION.md) on the exact proposed
firmware. Record commit, build identity, transport/platform, strap, wiring,
sensor and profile. The retained S2 `1419ea2` campaign predates changes to the
core and HAL adapter and explicitly skipped accelerometer calibration without
a validated +Z fixture; it did not include an S2 hour soak.

Acceptance for a deployment claim includes appropriate S2/S3/backend coverage,
profile/readback, sampling/scaling/freshness, reset/boot/settling, calibrated
orientation for accelerometer calibration when claimed, bounded owner soak,
and physical fault/recovery with other devices sharing the bus. Alternate
`0x6B`, electrical NACK/timeout/brownout, axis/mounting, and load tests remain
separate fixture gates. Retain the runner's JSON/raw output with immutable
firmware provenance. A host parser pass or historical serial summary does not
close this item. No hardware was accessed in this review.

### R2 - Conditional capability work: FIFO, interrupts, and filtered modes

The [implementation prompt sequence](prompts/README.md), prepared on 2026-10-25,
turns this item into four ordered software tasks covering motion FIFO,
fourth-dataset/counter interpretation, supported interrupt routes and filters.
It records source mechanisms, selected scope, regression acceptance and CI
requirements. Prompt 01 supplies the motion-acquisition implementation in
`0cc5ef2e5f395dbf87c6d18aaf81e49112c73f06`, with all seven CI jobs passed and
142 native tests passed after closeout coverage additions. Prompt 02 starts
from its closeout `bce83376aa01d0e9836f9175a9993e93128956e5` and extends the
same profile/schema/operation owners with temperature or timestamp/step
records, shared step-event triggering, fixed pedometer policy and explicit
counter-reset/epoch/loss evidence. Implementation resolution is
`f7c2d5d87a505d428a379b6628c447b62ec5a1b8`. All 170 native tests, three Arduino
targets and local contract/documentation/package checks passed. The exact final
closeout commit must also pass all CI jobs; its evidence is described in the
[execution record](prompts/README.md#prompt-02-execution-record).
Prompt 03 starts from the accepted fourth-dataset closeout and supplies typed
shared electrical controls, supported direct routes, 6D/4D orientation and
bounded independent source observations with acknowledgement effects. Its
implementation resolution is `d35165d4d12e8d2a1be6e323326b4df966fac00b`.
All 202 native tests, three Arduino targets and local documentation/contract/
package checks passed. The exact final closeout commit must also pass all seven
CI jobs; see the [execution record](prompts/README.md#prompt-03-execution-record).
Prompt 04 completes typed accelerometer LPF1/LPF2/HPCF/input, slope/high-pass/
reference and analog choices; all gyro FTYPE codes; orientation LPF2; complete
readback/restoration; source-backed timing and bounded reference consumption.
Samples/FIFO/events carry immutable filter and per-quantity qualification.
Implementation resolution: `10f6898386dfe510254232d0be677643309e40ed`. All 236
native tests, three Arduino targets and local documentation/contract/package
checks passed. Exact-final-commit CI remains the distinct completion gate in the
[execution record](prompts/README.md#prompt-04-execution-record).
Gyro HPF register configuration and explicit direct unqualified reads are
implemented; **qualified gyro HPF settling remains unresolved** because vendor
transition tables exclude it. That remaining claim is not closed by readback,
DRDY or software tests. Physical fourth-dataset, electrical, pulse, interrupt-loss,
filter transient and orientation evidence is **NOT RUN**.

The motion change starts from fetched baseline
`362b140990bc4ba2bc7f32cef50f8d9f30b4d3cc` on 2026-10-26. It adds typed motion
profiles and bounded caller-buffer acquisition with generation/scale provenance,
source-backed pattern interpretation, explicit transition discards, pre/post
status and consumption/loss evidence. Interpretation-changing procedures reject
live FIFO and require explicit bounded purge/stop first; read-only reconcile
cannot label unknown queued data with new provenance. Both CLIs expose the same
bounded core jobs. Native FIFO fault tests and the existing contract/CI suites
are the software evidence gate; the exact resolution SHA and final outcomes are
recorded in the [execution index](prompts/README.md). Hardware is **NOT RUN** for
this path; [qualification cases](HIL_VALIDATION.md#motion-fifo-qualification)
remain open independently of the software gate.

Owner: the consuming application and driver maintainer. All four requested
software scopes now share the current operation/profile owners. Remaining
physical gates and qualified gyro HPF require distinct evidence under
[the support matrix](chip-reference/13_library_support_matrix.md); unsupported
external datasets and unrelated event engines are outside this sequence.

Acceptance requires a typed replayable configuration with full validation and
readback, bounded caller-buffer acquisition, exact fourth-dataset/pattern
schema including temperature/step/timestamp interactions and rollover,
overrun/loss evidence, absolute deadlines and transaction ceilings, and
cancellation/partial-effect provenance. Interrupt support additionally needs
complete electrical, routing, latch, threshold and duration ownership. A
supported filter needs source-backed settling and sample-validity rules.
Inject every transfer-stage failure in native tests; test empty/full/overrun,
mixed datasets, malformed profiles, and cancellation; then qualify the actual
electrical routes and data behavior on hardware. Do not restore old raw recipes
or rename destructive purge as acquisition. These are optional feature projects,
not release-blocking unmerged fixes under the current supported contract.

## Reproduction and retain/retire workflow

```powershell
git status --short --branch
git fetch origin
git rev-parse HEAD origin/main
git for-each-ref --format='%(refname:short) %(objectname) %(committerdate:iso-strict)' refs/remotes/origin
$auditRef = 'origin/exploration/lsm6ds3trc-industry-readiness'
git merge-base main $auditRef
git rev-list --left-right --count "main...$auditRef"
git log --format='%H %s' "main..$auditRef"
git cherry main $auditRef
git diff --stat "main...$auditRef"
git show "${auditRef}:docs/LSM6DS3TRC_INDUSTRY_READINESS_AUDIT.md"
git merge-base --is-ancestor origin/feature/lsm6ds3tr-idf-port main
```

1. Repeat these checks if any tip moves. Keep the implementation and this
   current disposition together on the existing main branch.
2. Assign R1 to the fixture owner. Decide R2 from an actual consumer requirement;
   absence of an optional feature does not justify an old branch merge.
3. The ancestor refs are retirement candidates after checking active consumers
   and PRs. Preserve the unique exploration tip with a durable archive ref or
   a verified Git bundle before removing its branch; merely recording its SHA does
   not keep an unreachable Git object alive. This review deletes no refs.
4. For new behavior, follow the present support/CI contract and write focused
   regressions. Never replace current AGENTS or architecture with old prompts.
   Stage owned changes only, commit/push without force, and verify upstream
   synchronization. The lead owns shared Git mutations.

## Local checks and future CI

This documentation review passed the current core timing, CLI, native IDF,
chip-documentation coverage, and version-metadata guards. The IDF checker
self-test passed 3/3 tests. Local Markdown targets in the changed files and
`git diff --check` were checked. No full C++ suite, target build, package build,
Doxygen generation, or hardware campaign was run for this documentation change.

For a future implementation change reuse `.github/workflows/ci.yml`:
`scripts/pio.cmd test -e native`; build `esp32s3dev`, `esp32s3hil`, and
`esp32s2dev`; run `tools/check_core_timing_guard.py`, `check_cli_contract.py`,
`check_idf_example_contract.py`, `test_check_idf_example_contract.py`,
`check_chip_docs_coverage.py`, `test_run_hil.py`, `build_docs.py`, version
generation checks, and package packing plus `check_package_contract.py`.
CI also compiles the native IDF S2/S3 examples using ESP-IDF 5.4.4 and pins
PlatformIO 6.1.19. On Windows use the existing `scripts/pio.cmd` installation.
Major behavior changes require green CI on the exact final commit; this
documentation-only chunk requires no extra dispatch. A normal main push can
still trigger its configured CI.
