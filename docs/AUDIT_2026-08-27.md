# LSM6DS3TR-C Library Audit — 2026-08-27

Source-backed audit of the driver, examples, and tooling against the
LSM6DS3TR-C datasheet (Rev 3) and AN5130 (Rev 1). Every claim below was
re-derived from the vendor text or from a compiled repro, not from a similar
part.

Baseline at audit time: 100/100 native tests pass; every repository contract
checker passes.

**Already fixed in this change** (mechanical, verified, tests green) — see
[CHANGELOG.md](../CHANGELOG.md). This report covers only what still needs a
decision.

Each item states the defect, the failure it produces, and one concrete minimal
fix. Fixes prefer removing or relocating logic over adding guards.

---

## Priority 1 — Wrong results or broken tooling

### 1.1 Calibration averaging throws away the precision it exists to produce

**Where:** [src/LSM6DS3TR.cpp:1807](../src/LSM6DS3TR.cpp#L1807)

`_stepCalibration` accumulates every sample into `int64_t` sums, then collapses
the mean back to `int16_t` raw counts before scaling:

```cpp
const RawAxes mean{static_cast<int16_t>(_sumX / _samplesDone), ...};
const Axes measured = rawAxesToFloat(mean, scale);
```

The integer divide discards the fractional part — exactly the quantity that
averaging up to 1000 samples is meant to recover. Worse, C++ integer division
truncates toward zero, so the error is asymmetric: positive means round down,
negative means round up. Because noise dithers the samples, the expected
reported bias is biased toward zero by roughly half an LSB, and that offset
does **not** shrink as `samples` grows.

**Failure:** a gyroscope bias of +0.004 dps at ±250 dps (8.75 mdps/LSB) is
reported as 0.000 dps regardless of whether the caller averaged 32 or 1000
samples. The documented purpose of `CalibrationRequest::samples` is defeated.

**Fix — one helper, two call sites, no new state.** Beside `rawAxesToFloat`
([src/LSM6DS3TR.cpp:302](../src/LSM6DS3TR.cpp#L302)):

```cpp
Axes meanMicroAxesToFloat(int64_t sumX, int64_t sumY, int64_t sumZ,
                          uint16_t count, int32_t microPerLsb) {
  const float divisor = 1000000.0f * static_cast<float>(count);
  return Axes{static_cast<float>(sumX * microPerLsb) / divisor,
              static_cast<float>(sumY * microPerLsb) / divisor,
              static_cast<float>(sumZ * microPerLsb) / divisor};
}
```

Delete the `mean` declaration (it has no other use) and replace the two
`rawAxesToFloat(mean, scale)` calls with
`meanMicroAxesToFloat(_sumX, _sumY, _sumZ, _samplesDone, sensitivity)`. Keep
the `scale` locals: `peakToPeak` is a genuine integer count span and is already
correct.

`count` cannot be zero (control reaches this point only once
`_samplesDone >= request.samples`, and `samples` is validated 1..1000).
Overflow is impossible: `|sum| <= 1000 * 32768 = 3.28e7`, times 70000 =
2.3e12, far inside `int64_t`.

**Do not** apply the same change to the self-test averages at
[src/LSM6DS3TR.cpp:1626](../src/LSM6DS3TR.cpp#L1626). `_phaseBaseline` and
`_phaseStimulus` are `RawAxes` members carried across substeps; converting them
would require changing the member types and would shift the reported deltas
against the vendor's 90..1700 mg / 150..700 dps thresholds. That is a separate,
HIL-revalidated change.

---

### 1.2 Owner-soak firmware spins at full rate when a job start is rejected

**Where:** [examples/02_owner_soak/main.cpp:338](../examples/02_owner_soak/main.cpp#L338)

`startProbe`/`startReconcile`/`startSample` set `phase` before `acceptStart()`,
and no schedule deadline (`nextSampleMs`, `nextMaintenanceMs`) is advanced when
a start is rejected — they advance only inside `handleTerminal()`, which never
runs because no token was issued. `scheduleWork()` therefore re-evaluates the
same overdue deadline on the next `loop()` iteration, forever.

**Failure:** each attempt runs `Serial.printf("HIL_START_FAILURE ...")` plus
`Serial.flush()`. Once the driver enters a state that rejects starts (for
example a configuration invalidated by a mismatch, which `reconcile` does not
repair), the device enters an unbounded full-rate print loop for the remaining
~1 hour of the soak, swamping the host monitor.

**Fix — stop, do not back off.** A persistent start rejection already means the
soak has failed (`operationFailures > 0` guarantees `HIL_SOAK_FAIL`), so match
the existing abort style at lines 248-266 rather than adding a retry policy:

1. Propagate the `bool` that `acceptStart`/`start*` already return but every
   call site discards.
2. Add `uint32_t consecutiveStartFailures = 0;` beside the other file-scope
   counters.
3. In `scheduleWork()`, route both branches through one handler that increments
   the counter on rejection, resets it on success, and aborts the soak with the
   existing terminal-record path once it exceeds a small fixed bound.

This removes the loop without inventing backoff, and keeps the "deterministic,
bounded, no unbounded retries" contract the driver itself follows.

---

### 1.3 Wire read transport cannot report NACK, timeout, or bus faults

**Where:** [examples/common/I2cTransport.h:133](../examples/common/I2cTransport.h#L133)

`wireWriteRead()` runs the write phase with `wire->endTransmission(false)` and
checks the result. On Arduino-ESP32 — the only platform `library.json` declares
— `endTransmission(false)` performs **no bus activity**: it sets `nonStop` and
returns 0 unconditionally. So the `if (result != 0)` branch is unreachable, and
the real combined transaction happens inside `requestFrom()`, which discards
the underlying `esp_err_t` and returns only a byte count.

**Failure:** every read failure — address NACK, data NACK, timeout, arbitration
loss — surfaces as `Err::I2C_ERROR` / "I2C read length mismatch" / detail 0.
`I2C_NACK_ADDR`, `I2C_NACK_DATA`, `I2C_TIMEOUT` and `I2C_BUS` can never be
produced by a read. `DriverDiagnostics::lastTransportError` is therefore
useless for read faults, and an owner cannot distinguish "sensor absent" from
"bus stuck" — the exact distinction its recovery policy needs.

**Fix — narrow, and mind two traps.**

The tempting fix (call `i2cWriteReadNonStop()` directly to recover the
`esp_err_t`) is **wrong as usually written**, for two reasons worth recording:

- `TwoWire::num` is `protected`, so the adapter cannot recover the bus index
  from the `TwoWire*` it receives in `user`. Hardcoding bus `0` would silently
  drive the wrong peripheral when the caller passes `Wire1`.
- `beginTransmission()` takes the TwoWire FreeRTOS semaphore, released only by
  `endTransmission(true)` or `requestFrom()`. Jumping to the raw HAL without
  one of those holds the lock forever and deadlocks the next Wire call.

The safe minimal improvement is to keep the Wire API and classify what Wire
does expose: distinguish `read == 0` (nothing arrived — treat as
`I2C_NACK_ADDR`) from `0 < read < rxLen` (truncated transfer — `I2C_BUS`), and
delete the unreachable `endTransmission(false)` result check with a comment
explaining why it cannot fail. Full `esp_err_t` fidelity requires the native
ESP-IDF transport, which already has it — that is the honest place to point
owners who need it.

---

### 1.4 ESP-IDF example hides its CLI exactly when it is needed

**Where:** [examples/idf/basic/main/main.cpp:1520](../examples/idf/basic/main/main.cpp#L1520)

`app_main()` prints the I2C failure and `return`s before `cliLoop()`. ESP-IDF
deletes the main task when `app_main` returns, so `help`, `status`, `diag`,
`scan`, `bind` are all unreachable in the one situation an operator needs them.
The Arduino twin does the opposite: `setup()` returns early but `loop()` keeps
servicing input, and `printDiagnostics()` reports the retained failure as
`bus_init code=... detail=... message=...`.

The IDF example also emits no `bus_init` line at all;
`tools/check_cli_contract.py` requires that token in the Arduino main, while
`check_idf_example_contract.py` silently omits it — so the guards do not catch
the divergence.

**Fix — mirror the Arduino lifecycle, do not band-aid the printf:**

1. Add file-scope `Status busInitializationStatus = Status::Error(Err::INVALID_CONFIG, "I2C bus initialization not attempted");`
2. Change `configureI2c()` to return `Status` via the existing `mapEspError()`
   helper instead of raw `esp_err_t`, removing the impedance mismatch at the
   call site rather than duplicating a mapping there.
3. Rewrite the `app_main` tail to print status, bind only when the bus is
   ready, and **always** enter `cliLoop()`.
4. Add `bus_init code=` to `REQUIRED_IDF_TOKENS` in
   `tools/check_idf_example_contract.py` so the two examples cannot diverge
   again.

---

## Priority 2 — Contract and provenance defects

### 2.1 Successful results report a stale mismatch register

**Where:** [src/LSM6DS3TR.cpp:1996](../src/LSM6DS3TR.cpp#L1996)

`_finish()` unconditionally overwrites the per-operation
`_workingResult.configuration.{mismatchRegister,expectedValue,observedValue}`
with the *driver-lifetime* `_mismatch*` members. Those are cleared only by a
fully successful CONFIGURE/RECONCILE/RESET/BOOT/RECOVER. Every other job kind —
PROBE, SAMPLE, POWER_DOWN, CALIBRATION, FIFO_PURGE — therefore publishes a
**SUCCEEDED** result carrying a mismatch triple from an earlier, unrelated
operation.

The header documents these as per-operation evidence ("First register with
failed readback, or zero"), while the lifetime view already exists separately
on `DriverDiagnostics`. The two structs are supposed to differ; today they do
not.

**Failure:** a CONFIGURE fails on `CTRL1_XL`; the owner then runs POWER_DOWN,
which succeeds. `result.configuration.mismatchRegister` reads `0x10` on a
successful power-down, so an owner that logs mismatch evidence on success
reports a register fault that did not occur.

**Fix — make `_finish` stop writing the field, and let `_recordMismatch` own it:**

1. Delete the three `_workingResult.configuration.mismatch*` assignments in
   `_finish()` (**lines 1996-1998**). Keep the `state`/`generation`/
   `validAfterUptimeMs` assignments — those are genuinely current-state.
2. Delete the three `_workingResult.configuration.* = 0;` lines in
   `_stepConfigure()` (**lines 1102-1104**). **Keep lines 1099-1101**, which
   clear the lifetime `_mismatch*`; removing those would stop `diagnostics()`
   clearing after a successful repair.
3. Keep `_recordMismatch()`'s `_workingResult` writes — after step 1 they
   become the only populator of the per-operation triple.

**Expected behavior change to note in the changelog:** if a SELF_TEST's primary
phase recorded a mismatch and its restore then succeeded, the mismatch now
survives into the result instead of being wiped by the restore's clearing
block. That is the correct per-operation semantics, but it will look like a
change.

**Add a regression test** for the invariant nothing currently covers: a
SUCCEEDED POWER_DOWN after a failed CONFIGURE must report
`configuration.mismatchRegister == 0` while `diagnostics().mismatchRegister`
still reports the failing register.

---

### 2.2 A read-only reconcile can push data validity into the future

**Where:** [src/LSM6DS3TR.cpp:1112](../src/LSM6DS3TR.cpp#L1112)

The settle gate is preserved only when the state *before* the operation was
`KNOWN`. If it was `SETTLING`, control falls into the `else` branch, which
recomputes `requiredSettleUs()` and sets `_validAfterUptimeMs = nowMs + settleMs`.

Reconcile issues no writes at all (`MAX_RECONCILE_TRANSACTIONS = 35` = 33
readbacks + 2 identity reads), so it causes no filter or turn-on transient.
Restarting the AN5130-derived gate delays validity for no silicon reason, and
the reconcile job then blocks on its own fabricated deadline.

`SETTLING`-before-operation is reachable and designed: a configure that times
out inside its settle window leaves `SETTLING` with a valid
`_validAfterUptimeMs`, because the rollback paths only restore state when it is
`APPLYING`.

**Fix — state the predicate the code already means:**

```cpp
const bool priorGateTrusted =
    _configurationStateBeforeOperation == ConfigurationState::KNOWN ||
    _configurationStateBeforeOperation == ConfigurationState::SETTLING;
uint64_t settleMs = 0U;
if (reconcileOnly && priorGateTrusted) {
  _validAfterUptimeMs = _validAfterBeforeOperationMs;
  _configurationState = _configurationStateBeforeOperation;
} else { /* unchanged */ }
```

Restrict the predicate to exactly those two states: `UNKNOWN`/`UNCONFIGURED`
must keep recomputing a conservative gate, because the write time is genuinely
unknown there. `_validAfterBeforeOperationMs` is never garbage under either
trusted state — `_invalidateConfiguration()` zeroes it whenever the state
leaves KNOWN/SETTLING, and SETTLING is only ever set together with a freshly
computed deadline.

*(The related snapshot bug — the raw field never resolving SETTLING→KNOWN — is
already fixed in this change.)*

---

### 2.3 `poll(nowMs, 0)` under-reports `waiting` during self-test time gates

**Where:** [src/LSM6DS3TR.cpp:2176](../src/LSM6DS3TR.cpp#L2176)

`_stepSelfTest`'s `averageStep` enters a bus-silent ODR-cadence gate after every
sample (`+20 ms` accelerometer, `+5 ms` gyroscope). In the `maxTransactions == 0`
branch, `_waiting` is cleared and `safeComputeStep` covers only substeps
3/7/13/17 and the restore — the sampling substeps 4/5/8/9/14/15/18/19 fall
through to a residual chain that handles RESET/BOOT/RECOVER and
SAMPLE/CALIBRATION but omits SELF_TEST.

The identical gate in `_stepCalibration` *is* handled, so this is an oversight,
not a design choice.

**Failure:** the same driver state answers `waiting` differently depending on
the budget passed. An owner following the documented contract ("`waiting` means
time or sensor data must advance") busy-spins through a self-test's inter-sample
gates instead of yielding.

**Fix — four edits; edit 4 alone is not safe.** Establish under SELF_TEST the
invariant SAMPLE and CALIBRATION already hold ("`_waitUntilMs != 0` means a live
gate"):

1. `beginRestore()` lambda — add `_waitUntilMs = 0;` after `_step = 0;`.
   Safe: `_stepConfigure` never reads `_waitUntilMs`.
2. `routeFailureToRestore` — add `_waitUntilMs = 0;` before the
   `_substep = 90/91/92/93` chain.
3. End of the averaged-phase completion block (~line 1665) — add
   `_waitUntilMs = 0;` next to `_step = 0;`. **Do not** add it to the
   substep 4/8/14/18 completion: substeps 5/9/15/19 legitimately consume that
   gate.
4. Widen the residual arm to include `_job == JobKind::SELF_TEST`.

Applying 4 without 1-3 trades false negatives for false positives (a stale
`_waitUntilMs` would claim a gate where none exists).

---

### 2.4 Reset/boot 15 ms guard is anchored to a pre-write timestamp

**Where:** [src/LSM6DS3TR.cpp:1309](../src/LSM6DS3TR.cpp#L1309)

Substep 6 arms the device-inaccessible guard with
`_waitUntilMs = saturatingAdd(nowMs, cmd::BOOT_TIME_MS)`, where `nowMs` is the
time the owner passed into `poll()` **before** the CTRL3_C command was written.
Every transaction executed earlier in that same `poll()` call, plus the command
write itself, is charged against the 15 ms budget; millisecond truncation costs
up to another millisecond. The guard can therefore release ~14 ms of real time
after the BOOT command, inside the window AN5130 says registers are
inaccessible.

**Fix — re-anchor only:**

- Substep 6: clear the guard (`_waitUntilMs = 0;`) while keeping
  `_substep = 7; _waiting = true;`. The poll loop then breaks and the owner
  supplies a fresh clock.
- Head of substep 7, before any read:
  `if (_waitUntilMs == 0U) { _waitUntilMs = saturatingAdd(nowMs, cmd::BOOT_TIME_MS); _waiting = true; return inProgressStatus(); }`
- **Also update** the zero-budget branch (~line 2172), which computes
  `_waiting = nowMs < _waitUntilMs` for `_substep == 7U`; with the `0` sentinel
  it would report `waiting=false` during the arming gap. Either use
  `_waiting = (_waitUntilMs == 0U) || nowMs < _waitUntilMs;` or, cleaner,
  introduce a dedicated `_guardArmed` bool instead of overloading `0`.

**Do not** route the transport failure at line 1320 through `_readyPolls`. The
README, the class contract, and `Config.h` all state the driver never retries
transport; the owner is the retry authority.

---

## Priority 3 — Dead surface and honest contracts

### 3.1 `Err::DEVICE_NOT_FOUND` and `Err::FIFO_EMPTY` are never produced

**Where:** [include/LSM6DS3TR/Status.h:30](../include/LSM6DS3TR/Status.h#L30)

No line in `src/`, `include/`, `test/`, `examples/` or `tools/` constructs
either code. An absent device surfaces as the raw transport error or as
`CHIP_ID_MISMATCH`; an empty FIFO is treated as *success* by `_stepFifoPurge`.

**Do not delete them.** `Err` is declared append-only within API major 2;
removing values 19 and 25 renumbers `CHIP_ID_MISMATCH` through `I2C_BUSY` and
silently breaks any consumer that logged or persisted the numeric code. Do not
synthesize an emission site either — returning `FIFO_EMPTY` from the
`unread == 0` short-circuit would turn a legitimately empty purge into an error
the caller must special-case, contradicting the README.

**Fix — the defect is in two doc comments, and the two values differ:**

- `DEVICE_NOT_FOUND` belongs to the *transport* vocabulary an integrator's
  `I2cWriteFn`/`I2cWriteReadFn` may return. Retag it: "Application transport
  reported no device at the address; never synthesized by the driver."
- `FIFO_EMPTY` describes an outcome of a driver generation that no longer
  exists. Mark it reserved: "Reserved; the bounded FIFO purge reports an empty
  FIFO as success."

Revisit both at the next major version.

### 3.2 `startSample`'s BDU guard is unreachable

**Where:** [src/LSM6DS3TR.cpp:734](../src/LSM6DS3TR.cpp#L734)

`_verifiedProfile` is only ever assigned from `_desiredProfile` (or from
`_selfTestRestoreProfile`, itself a copy of `_verifiedProfile`), and
`_desiredProfile` is only assigned in `startConfigure` **after**
`validateProfile` has rejected `blockDataUpdate == false`. The preceding
`_checkReadyForKnownConfiguration` additionally guarantees
`_hasVerifiedProfile`. The branch cannot fire — and its error code would be
wrong if it could: a profile legitimately lacking BDU is
`UNSUPPORTED_PROFILE`, not `CONFIGURATION_UNKNOWN`.

**Recommended fix:** delete the three lines and leave the BDU requirement at its
single chokepoint, `validateProfile`, with a one-line comment noting that
`_checkReadyForKnownConfiguration` guarantees a validated profile.

**Do not** replace it with `assert()`: the library has zero assert usage today,
`<cassert>` vanishes under `NDEBUG`, and aborting contradicts the Status-based
error model. If real defense-in-depth against a device that silently diverged
is wanted, the correct shape is the one `_stepFifoPurge` already uses — read
CTRL3_C inside the sample state machine and fail with `CONFIGURATION_UNKNOWN`
on a missing BDU bit. That costs one extra transaction per sample and requires
bumping `MAX_SAMPLE_TRANSACTIONS`; it is a design change, not a cleanup, and
must not be bundled with the deletion.

### 3.3 Owner-soak physical-range guards do not match the configured full scale

**Where:** [examples/02_owner_soak/main.cpp:130](../examples/02_owner_soak/main.cpp#L130)

`updateRanges()` bounds converted samples against nominal-range literals:

```cpp
if (maximum > 2100000LL)   noteContractFailure("accel_range");   // 2.1 g
if (maximum > 251000000LL) noteContractFailure("gyro_range");    // 251 dps
```

The soak always runs the default profile (±2 g, ±250 dps). ST's sensitivities
are not `full_scale / 32768`:

- Accelerometer: `61 µg/LSB × 32768 = 1,998,848 µg` — **below** the 2,100,000
  threshold. The `accel_range` check is dead code that can never fire.
- Gyroscope: `8750 µdps/LSB × 32768 = 286,720,000 µdps` = 286.7 dps — **above**
  the 251 dps threshold. A legitimate saturated reading trips a false
  `gyro_range` contract failure and fails an otherwise-passing soak.

**Fix — split the two things this check conflates.** A range check computed
from the nominal label cannot validate conversion; do the real check in
`validateSample()`, where the raw sample is still in scope, by recomputing the
expectation from the provenance the driver stamped on the sample and requiring
exact equality:

```cpp
if (converted.accelMicroG.x != static_cast<int64_t>(result.sample.accel.x) * accelSens) ...
```

using `accelSensitivityMicroGPerLsb(result.sample.accelFullScale, ...)` and the
gyroscope equivalent. That tests what the guide claims ("conversion") and is
genuinely non-dead. If a plausibility bound is still wanted, derive it from
`32768 × sensitivity`, not from the nominal range label.

---

## Deliberately not changed

Recorded so they are not re-litigated:

- **`MAX_TRANSPORT_WRITE_BYTES = 33` while the driver writes at most 2 bytes.**
  Not a defect. It is a hard upper bound on buffers presented to injected
  callbacks, frozen by `test_compile_contracts.cpp` because external owners
  size static buffers from it. A 2-byte write satisfies an at-most-33 bound; no
  block-write API ever existed.
- **The 14-byte output burst in `_stepSample` does not set
  `hardwareStateMayHaveChanged`.** Reading the high output byte does clear the
  latched ready flag, but "consuming read" is this repository's term for the
  FIFO purge — the one read that destroys queued device data. Sampling is
  documented as non-destructive throughout.
- **`CTRL7_G` diagnostic mask `0xF8`.** Correct. In the column-mangled datasheet
  extract, `ROUNDING_STATUS` is a wrapped cell occupying **bit 3** — the same
  wrap pattern as `USR_OFF_W` in `CTRL6_C`, whose bit-3 position is
  independently known. The mask covers bits 7:3 and excludes reserved bits 2:0.
- **`SW_RESET` waits 15 ms though AN5130 specifies ~50 µs.** Deliberate library
  policy, already recorded in the ambiguity ledger.
- **Retaining host-integration contract facts in the HIL guide.** These were
  generalized in this change rather than deleted; the capacity numbers are the
  portable part and apply to any host firmware.
