# LSM6DS3TR-C Suitability Re-audit For TunnelMonitor-node

**Re-audit date:** 2026-07-19
**Library release:** 2.0.0 breaking refactor
**Library branch:** `hardening/tunnelmonitor-suitability-reaudit`
**Decision:** library mechanics suitable for an external I2C owner; product
integration not approved because TunnelMonitor sensing and capacity contracts
remain undefined

This report supersedes the 2026-07-18 audit of version 1.2.0. Old line numbers
and proposed API sketches are no longer authoritative.

## Exact Audit Basis

Library evidence:

- repository: `C:\Users\Honza\Documents\Projects\LSM6DS3TR`;
- re-audit baseline: `d465e63fc76281a12707ad379e504380c51235a1`;
- version 1.2.0 comparison point: tag commit
  `ae0491a35b627973adfab6af3fed2b015809faab`;
- implementation branch created before edits:
  `hardening/tunnelmonitor-suitability-reaudit`;
- reproducibility/version slice:
  `c65f919` (`build: pin toolchains and unify 2.0.0 metadata`);
- owner-scheduled core and fault suite:
  `d752e2f` (`feat: add owner-scheduled operation engine`);
- final-review provenance fix:
  `0da67fc` (`fix: preserve self-test failure provenance`);
- stale version 1 component-policy cleanup:
  `f3d8ce7` (`refactor: remove stale v1 component policy`);
- sleeping-gyro maintenance fix:
  `67868ee` (`fix: handle sleeping gyro in maintenance jobs`);
- complete disabled-state reconciliation fix:
  `1e23191` (`fix: reconcile complete disabled register state`);
- documentation/integration closeout: the commit containing this report.

TunnelMonitor evidence was inspected read-only:

- repository: `C:\Users\Honza\Documents\Projects\TunnelMonitor-node`;
- branch: `develop`;
- revision: `0897f12c1a1369367747d1063936906005391580`;
- working tree: clean and aligned with `origin/develop` when inspected;
- no TunnelMonitor files were changed.

Before closeout, that separate workspace was externally switched to clean
branch `docs/mb85rc-suitability-contract-facts` at `b708f511964db6c51e949e99c67820476f00f9c7`.
Its tree is identical to the audited `0897f12...` revision (`git diff
--name-status` is empty); the two intervening documentation commits add and
then revert the same FRAM-only change. Final native validation ran on `b708f51`.

Applicable instructions read in full:

- this repository's `AGENTS.md`;
- `TunnelMonitor-node/AGENTS.md`;
- the complete original suitability audit and task instructions.

Authoritative TunnelMonitor areas inspected:

- `docs/guidelines/i2c_peripherals.md`;
- `docs/guidelines/ownership.md`;
- `docs/guidelines/measurement_data.md`;
- `docs/guidelines/time_health_watchdog.md`;
- `docs/guidelines/target_architecture.md`;
- `docs/guidelines/implementation_plan.md`;
- `docs/guidelines/open_questions.md`;
- `docs/guidelines/decisions.md`;
- `include/TunnelMonitor/BoardPins.h`;
- I2C, health, sample, capacity, and measurement contracts under
  `include/TunnelMonitor/`;
- `src/i2c/I2cTask.cpp`, relevant measurement/system code, and native I2C and
  measurement tests;
- TunnelMonitor `platformio.ini` and exact dependency pins.

A full source/contract search outside historical reports found no approved
LSM6DS3TR, IMU, motion, accelerometer, or gyroscope product contract.

## Tool And Baseline Evidence

Tools available locally at re-audit start:

| Tool | Observed version/state |
| --- | --- |
| Python | 3.12.10 |
| PlatformIO Core | 6.1.18 |
| CMake | 4.0.1 |
| Ninja | 1.12.1 |
| g++ | 15.1.0 |
| Doxygen | 1.13.2 |
| cppcheck | 2.17.0 |
| Git | 2.52.0 |
| `clang++` | unavailable |
| `idf.py` | unavailable |

Pre-edit baseline results on `d465e63`:

| Check | Result |
| --- | --- |
| Generated version check | PASS |
| Core timing guard | PASS |
| Arduino CLI contract | PASS |
| ESP-IDF example text contract | PASS |
| Chip-document coverage | PASS |
| Version 1 HIL parser self-test/dry-run | PASS, 10 planned commands |
| Native PlatformIO tests | PASS, 113/113 |
| ESP32-S3 Arduino build | PASS, RAM 22,640 B, flash 426,066 B |
| ESP32-S2 Arduino build | PASS, RAM 37,040 B, flash 416,733 B |
| Actual local ESP-IDF build | NOT RUN, `idf.py` unavailable |
| Physical HIL | NOT RUN |

The baseline proves the original suite was healthy before the breaking change;
it does not prove version 2 behavior. Final version 2 validation is recorded in
the closeout section below.

## Implemented Architecture

Version 2 uses one fixed-memory owner-scheduled operation model:

```text
zero-I2C bind
    -> start(kind, absolute deadline) -> nonzero library token
    -> poll(caller uptime, caller transaction budget)
    -> terminal atomic result
    -> take matching token exactly once
    -> zero-I2C unbind when required
```

The application owns the bus, transport timeout, locking, task, scheduling,
clock, request queue, retries, health, backoff, and bus recovery. The library
never sleeps, retries transport, creates a task, logs, allocates dynamically,
or denies access because of its own offline policy.

One operation is active at a time. One terminal result can be pending. Starts,
cancellation, and unbinding perform no I2C. Each `poll()` is limited by the
caller's transport-callback budget; wait stages consume zero callbacks.
Operation results include identity, state, status, start/completion uptime,
actual and maximum transaction counts, and whether hardware may have changed.
Every job has both an absolute deadline and a hard total callback ceiling;
transport failures are never retried internally.

Configuration is one replayable `DeviceProfile`. Version 2 supports polling
snapshots with FIFO and interrupts explicitly disabled. Unsupported enabled
profiles are rejected. FIFO data acquisition and motion-event routing are not
falsely advertised.

## Finding Summary

| Finding | Revalidated | Disposition | Final status |
| --- | --- | --- | --- |
| LSM6-01 sensing role undefined | Yes | Product decision remains outside library | OPEN external integration gate |
| LSM6-02 blocking lifecycle | Yes | Replaced with zero-I2C binding and staged jobs | RESOLVED |
| LSM6-03 exclusivity/cancellation | Yes | Global operation exclusion and bus-silent cancellation | RESOLVED |
| LSM6-04 freshness/validity | Yes | Atomic tokened result with valid/fresh masks | RESOLVED |
| LSM6-05 cached-scale conversion | Yes | Full-scale provenance carried in raw result; pure conversion | RESOLVED |
| LSM6-06 BDU/coherence claims | Yes | BDU required for managed sample; quality claims narrowed | RESOLVED |
| LSM6-07 unknown configuration | Yes | Explicit configuration state gates managed data | RESOLVED |
| LSM6-08 incomplete desired state | Yes | Complete supported profile image plus readback | RESOLVED for declared scope |
| LSM6-09 reset/boot order | Yes | Staged preparation, 15 ms silent gate, replay/readback | RESOLVED |
| LSM6-10 settling | Yes | Verified `validAfterUptimeMs` and `SETTLING` gate | RESOLVED |
| LSM6-11 conflicting health/recovery | Yes | Offline admission removed; diagnostics passive | RESOLVED |
| LSM6-12 unusable FIFO | Yes | Acquisition removed; bounded destructive purge only | RESOLVED by restriction |
| LSM6-13 unsafe interrupts/events | Yes | Enabled interrupt profiles unsupported | RESOLVED by restriction |
| LSM6-14 blocking diagnostics/clock | Yes | Staged jobs and one caller 64-bit clock domain | RESOLVED |
| LSM6-15 mounting/calibration | Yes | Explicit expected gravity; product transform/persistence external | LIBRARY RESOLVED, product gate open |
| LSM6-16 units/header hygiene | Yes | Exact-unit helpers; macro collision removed | RESOLVED |
| LSM6-17 release reproducibility | Yes | One version source and pinned compile inputs | RESOLVED |
| LSM6-18 TunnelMonitor contracts/capacity | Yes | No contract invented; deliberate firmware work still required | OPEN external integration gate |

## Detailed Finding Dispositions

### LSM6-01 — TunnelMonitor sensing role

**Current evidence:** TunnelMonitor hardware revision `2.0.0`, board profile
`tunnelmonitor_s3_hw200`, and its authoritative I2C inventory still define RTC,
FRAM, ENV, INA228, and OLED only. `BoardPins` assigns I2C SDA/SCL GPIO 8/9 but
no LSM6 address strap or INT1/INT2 pin. No source schema, mounting transform,
calibration policy, ODR/range, required/optional role, or snapshot/event/
waveform meaning exists.

**Resolution:** keep the library general-purpose. A polling snapshot is a
mechanically supported option, not a TunnelMonitor product decision. No
firmware contract or board fact was invented.

**Status:** open external integration gate. It does not block completion of the
general library.

### LSM6-02 — Blocking lifecycle

**Current evidence:** version 1 `begin()`, `recover()`, and `end()` combined
binding and multiple hidden transfers.

**Resolution:** `bind()` validates and stores a non-owning transport without
I2C. `unbind()` is also bus-silent. Probe, configuration, recovery,
power-down, reset, and boot are explicit staged operations with absolute
deadlines and caller transaction budgets. Configuration validates WHO_AM_I
before its first write, and reconcile does so before its 33 managed reads. A
positive chip-ID mismatch from probe, configure, reconcile, or recover
invalidates prior verified provenance.

**Status:** resolved.

### LSM6-03 — Exclusivity and cancellation

**Resolution:** every hardware-touching job shares one active-operation gate.
Diagnostic access is rejected while a job is active. `cancelActiveJob(nowMs)`
performs no I2C and publishes a tokened `CANCELLED` result. Cancellation after
possible writes preserves unknown configuration/hardware-change evidence.

**Status:** resolved.

### LSM6-04 — Freshness and validity

**Resolution:** a sample is contained in its terminal `OperationResult` and
cannot be taken with another token. `RawSampleResult` includes independent
valid/fresh masks, quality, sequence, configuration generation, read uptime,
and raw fields. Every ready-checked request containing temperature requires
TDA, including mixed requests. No historical cache is exposed as the answer to
a later job.

**Status:** resolved.

### LSM6-05 — Full-scale provenance

**Resolution:** each raw sample contains its accelerometer and gyro full-scale
values. `convertSample()` and fixed-unit decoders are pure functions and do not
read current driver configuration.

**Status:** resolved.

### LSM6-06 — BDU and coherence

**Resolution:** managed multi-byte samples require verified BDU. Readiness,
validity, and quality are explicit. Documentation states that BDU protects
register pairs but does not prove a single conversion instant across every
axis/sensor. Direct sampling is labelled `DIRECT_UNVERIFIED`.

**Status:** resolved for polling snapshots. Strict same-cycle acquisition
remains a future product/FIFO requirement.

### LSM6-07 — Unknown configuration

**Resolution:** `ConfigurationState` is `UNCONFIGURED`, `APPLYING`, `KNOWN`,
`UNKNOWN`, or `SETTLING`. Managed sampling requires known and settled state.
Generation increments only after verified profile apply or verified
power-down, never after a partial/ambiguous effect. Raw diagnostic writes
invalidate provenance.

**Status:** resolved.

### LSM6-08 — Desired-state replay

**Resolution:** supported production settings are represented in one
`DeviceProfile`, built into a complete managed register image, replayed, and
read back. FIFO and interrupts have explicit disabled-only profile policy;
enabled requests fail rather than relying on retained raw register state.
Configure reserves one callback for WHO_AM_I followed by the 66-transfer full
write/readback image; reconcile reserves one plus its 33 managed reads.
Sensor-sync and DRDY-pulse controls are explicitly cleared and verified so a
device state retained across an MCU-only restart cannot escape the profile.
Low-power/normal ODRs stop at 208 Hz. Power-down is admissible from any bound
idle configuration state, writes and reads back exact zero ODR registers, and
then leaves configuration unconfigured because no other register state was
proved. An existing desired runtime profile remains available only for a later
explicit configure or recovery.

**Status:** resolved for the declared version 2 scope.

### LSM6-09 — Reset and boot sequencing

**Resolution:** reset/boot/recovery are staged jobs. They prepare required
sensor modes, issue the command, enforce the documented 15 ms period with no
I2C, then poll, replay, and verify the desired profile. No blocking delay is
inside the core.

**Status:** resolved.

### LSM6-10 — Settling

**Resolution:** verified configuration records `validAfterUptimeMs` from pure
settling calculations. Until then the public state is `SETTLING` and managed
sampling does not publish interpreted data.

**Status:** resolved.

### LSM6-11 — Health and recovery ownership

**Resolution:** READY/DEGRADED/OFFLINE admission and automatic recovery policy
were removed. `DriverDiagnostics` reports passive transport counts, last
transport error/time, and configuration evidence only. The owner decides
retry, absence, backoff, health, and bus recovery.

**Status:** resolved.

### LSM6-12 — FIFO acquisition

**Resolution:** incomplete FIFO configuration and word-drain acquisition APIs
were removed. Production profiles require FIFO disabled. `startFifoPurge()` is
explicitly destructive, bounded to 1..2048 requested word reads, and reports
initial/final counts, starting pattern, overrun, discarded count, and
truncation.

**Status:** resolved by restricting claimed functionality. A typed FIFO
decoder/acquisition API is required before any vibration or waveform use.

### LSM6-13 — Interrupts and motion events

**Resolution:** production profiles require interrupts disabled. No raw event
recipe is described as replayable production configuration. Diagnostic access
does not become an application interrupt contract.

**Status:** resolved by restriction. A future event feature requires product
pin/electrical decisions and a typed replayable profile.

### LSM6-14 — Diagnostics and clock

**Resolution:** self-test and calibration are staged tokened jobs. Every job
and diagnostic timestamp uses the caller's 64-bit monotonic uptime. Hard total
callback ceilings are public constants or pure request-sized calculations;
terminal results report the applied ceiling. Sampling caps readiness at 65
STATUS reads while reserving its final callback for data. Reset/boot/recovery
cap command-bit polling at 16 while reserving full profile replay/readback.
Self-test and calibration cap readiness at three STATUS reads for every
discard/collected sample; self-test uses 3 ms post-sample gates and calibration
uses the rounded configured ODR period. Results preserve primary and
restoration status; failed restoration invalidates configuration. There is no
blocking convenience facade in the core.

**Status:** resolved.

### LSM6-15 — Mounting and calibration

**Resolution:** accelerometer calibration takes an explicit expected gravity
vector in sensor-native axes. Validation rejects non-finite/invalid requests;
the driver does not assume Z-up. Bias application is pure. Mounting transform,
rotation convention, persistence version, provenance, and validity policy stay
above the chip driver.

**Status:** library portion resolved; TunnelMonitor product decisions and HIL
remain external gates.

### LSM6-16 — Units and header hygiene

**Resolution:** exact integer helpers are named
`accelSensitivityMicroGPerLsb()` and
`gyroSensitivityMicroDpsPerLsb()`. Converted output units are micro-g,
micro-dps, and milli-Celsius. The public FIFO decimation enum and its
consumer-macro `#undef` workaround were removed with the unsupported FIFO
configuration surface.

**Status:** resolved.

### LSM6-17 — Release metadata and reproducibility

**Current evidence:** the original mismatch was `library.json` 1.2.0 versus
`idf_component.yml` 1.1.0, an unpinned PlatformIO platform/Core, broad IDF
metadata, and no real IDF CI build.

**Resolution:** `library.json` is the 2.0.0 source of truth.
`generate_version.py` generates/checks `Version.h`, the IDF component version,
and Doxygen project version. Target builds use the exact pioarduino 54.03.20
release ZIP and CI installs PlatformIO Core 6.1.18. The manifest supports only
the IDF 5.4 line; CI compiles the native example with IDF 5.4.0 for ESP32-S2
and ESP32-S3.

**Status:** resolved. Local `idf.py` was unavailable, so no local IDF compile is
claimed; CI is the actual compile gate.

### LSM6-18 — TunnelMonitor contracts and capacities

**Current evidence at revision `0897f12...`:**

- `DeviceId` has no motion sensor and `I2cOperation` has no motion read;
- known I2C device count is fixed at 5;
- measurement source request capacity is 4 and all four are used;
- production device-health count 16 equals capacity 16;
- sample field count is 37 of capacity 48;
- I2C result payload is 128 bytes and result queue depth is 8;
- result settlement requires exact request ID, submission token, device, and
  operation identity;
- normal owner work advances one backend/library callback per poll.

**Resolution:** no TunnelMonitor mutation was made. After LSM6-01 is decided,
firmware must append stable IDs, add a private adapter, calculate profile
capacities, define copied status and sample schema, and update CSV/replay/cloud
compatibility together. The adapter must correlate firmware identity with the
library token and must cancel/take expired work.

**Status:** open external integration gate.

## TunnelMonitor Integration Conditions

Before an adapter is authorized, the product must freeze:

1. exact part and selected board/profile;
2. required/optional role;
3. SA0 strap and address;
4. polling or a reviewed interrupt pin/electrical contract;
5. mounting transform and positive rotation convention;
6. output meaning: snapshot, tilt, event, statistics, or waveform;
7. ODR, range, filters, power, BDU, and accepted skew;
8. calibration persistence and validity policy;
9. sample fields, integer units, schema/profile version;
10. compile-time inventory, source, health, payload, and sample capacities.

A low-rate polling snapshot is the smallest reasonable first scope. It is not
an approved product decision merely because the library can now support it.

## Final Version 2 Validation

Local closeout results for the final version 2 working tree:

| Check | Result |
| --- | --- |
| Generated metadata synchronization | PASS |
| Core timing guard | PASS |
| Arduino CLI contract | PASS |
| ESP-IDF example contract | PASS |
| Chip-document coverage | PASS |
| Arduino example host syntax check | PASS, g++ 15.1 C++17 |
| Native fault-injection/contract suite | PASS, 71/71 |
| ESP32-S3 Arduino build | PASS, RAM 23,216 B, flash 370,322 B |
| ESP32-S2 Arduino build | PASS, RAM 37,048 B, flash 343,421 B |
| PlatformIO package creation | PASS, `LSM6DS3TR-2.0.0.tar.gz` |
| Doxygen | PASS, zero warnings |
| `git diff --check` | PASS, no whitespace errors |
| TunnelMonitor native contract suite | PASS, 1050/1050 at tree-equivalent `b708f51` |
| Local ESP-IDF compile | NOT RUN, `idf.py` unavailable |
| Remote ESP-IDF 5.4.0 matrix | Workflow configured; final revision result not yet observed |
| Physical version 2 HIL | NOT RUN |

Local physical HIL and local ESP-IDF compilation were not performed. Retained
version 1 fixture evidence is historical only; its incompatible runner was
removed instead of presenting a parser self-test as version 2 validation.
Version 2 still requires exact
hardware validation for disconnect/reconnect, NACK/timeout/held bus, reset and
brownout, `0x6B`, mounting/axis sign, full shared-bus load, and unattended soak.

## Final Recommendation

Version 2 closes every applicable library-side blocker identified by the
original audit without adding a generic IMU framework or application policy.
It is suitable as a chip driver behind an external single I2C owner using
bounded polling chunks.

TunnelMonitor integration remains intentionally blocked by LSM6-01 and
LSM6-18 until product and capacity contracts are approved. The correct next
step is a private `I2cTask` adapter for the selected profile, not another task,
raw register recipes, or application state machines that compensate for
library defects.
