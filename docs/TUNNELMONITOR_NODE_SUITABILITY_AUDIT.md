# LSM6DS3TR-C library suitability for TunnelMonitor-node

**Audit date:** 2026-07-18

**Decision:** not suitable unchanged; suitable as a base for a focused breaking refactor

**Recommended first scope:** low-rate, polling-based accelerometer/gyroscope snapshot

**Library candidate audited:** `v1.2.0`, commit `ae0491a35b627973adfab6af3fed2b015809faab`

## Executive summary

The library has a solid chip-level base. It has an injected I2C transport, fixed memory, useful register validation, correct signed raw decoding, a bounded poll engine, native tests, Arduino builds, and retained hardware evidence. These parts should be kept.

The library is not ready to be used unchanged inside TunnelMonitor's `I2cTask`. The main problem is not basic register access. The problem is ownership and result integrity around that access:

- `begin()` and `recover()` perform long synchronous I2C sequences.
- Active staged jobs can be disturbed by direct reads, setters, and raw access.
- There is no real bus-silent job cancellation.
- A failed new read can leave an old sample available as if it were usable.
- A cached raw sample can be converted with a newer full-scale setting and be wrong by up to 8 times.
- Results do not say which accelerometer, gyroscope, or temperature fields are valid and fresh.
- Unknown or partially applied configuration does not stop interpreted measurements.
- Reset and boot handling does not follow the device application note sequence and can access registers during the documented boot-inaccessible period.
- FIFO acquisition is not implemented: the current drain discards all words, and fourth-dataset configuration is incomplete.
- Driver-owned offline and recovery policy conflicts with TunnelMonitor's single I2C owner.

TunnelMonitor also has no current LSM6 hardware or measurement contract. Before integration, the product must decide whether this device is for a low-rate orientation snapshot, motion/shock events, or vibration waveform capture. These are different systems. A low-rate snapshot can fit the current bounded `I2cTask` design after the core refactor. Continuous waveform capture cannot be added as another periodic scalar sensor read.

The preferred solution is a focused breaking refactor of the library. Do not add an adapter state machine that tries to hide the current blocking lifecycle or stale-result behavior.

## Decision in one table

| Use case | Current fit | Fit after focused refactor | Comment |
|---|---:|---:|---|
| Chip probe and service diagnostics | Partial | Good | Current code is useful, but lifecycle and health ownership still need care. |
| Low-rate acceleration/gyro snapshot | No | Good | This is the recommended first platform scope. |
| Derived static tilt/condition value | No | Good | Requires a frozen mounting transform and application policy. |
| Shock, wake, tap, or motion interrupt | No | Conditional | Requires a board interrupt pin and typed, replayable interrupt configuration. |
| FIFO-based vibration statistics | No | Conditional | Requires a real FIFO decoder, loss reporting, timing budget, and product contract. |
| Continuous vibration waveform | No | Major product work | Needs a bounded acquisition, storage, and upload design outside the ordinary scalar sample path. |

## Audit basis

### Exact source revision

The working checkout at the time of the audit was on the older branch `exploration/lsm6ds3trc-industry-readiness` at commit `035eff530e72ea43e4b42b02df0ea2ff05e33383` and reported a 1.1-era version.

This audit uses the maintained release candidate instead:

- annotated tag: `v1.2.0`
- annotated tag object: `e1ea4b2c379820165b36ebf9858ea6af4d6a50c4`
- peeled commit: `ae0491a35b627973adfab6af3fed2b015809faab`
- commit date: 2026-06-25
- local tracking state: also `origin/main` and `origin/HEAD`

All library source line references in this report refer to that exact `v1.2.0` commit unless stated otherwise. TunnelMonitor references refer to its working tree on 2026-07-18.

### Latest-branch revalidation

On 2026-07-18, `origin` was fetched again with remote-branch pruning and tags.
`origin/HEAD` selects `origin/main`, and `origin/main` is the newest maintained
remote branch. The exploration branch has one branch-only commit and is thirteen
main-line commits behind the current release line. The working checkout was
safely switched to local `main` and fast-forwarded to
`ae0491a35b627973adfab6af3fed2b015809faab`. It is now zero commits ahead and
zero commits behind `origin/main`, and the audit report remains the only
untracked file.

The full source delta from the historical audit-start branch to v1.2.0 was
reviewed, then every hard finding, source reference, severity, and recommended
action was checked again against the final `main` source. The report had
already used this exact v1.2.0 commit, so the findings and severities remain
current. One reset/boot citation was corrected because its synchronous and
staged source ranges were labelled in reverse.

### Sources reviewed

- Public headers under `include/LSM6DS3TR/`
- Core implementation `src/LSM6DS3TR.cpp`
- Native tests, examples, CLI contracts, CI files, and package metadata
- `README.md`
- Existing `docs/tunnelmonitor_fit_report.md`
- Existing `docs/reports/hil-evidence-summary.md`
- Bundled ST `LSM6DS3TR-C_datasheet.pdf`, DocID030071 Rev 3
- Bundled ST application note `AN5130`, Rev 1
- TunnelMonitor architecture authority under `docs/guidelines/`
- TunnelMonitor I2C, measurement, status, board, and capacity contracts

The bundled PDFs were visually checked on the relevant device-mode, timing, output, FIFO, interrupt, reset, and I2C pages. The existing short TunnelMonitor fit report was treated as historical API evidence, not as an integration decision.

## TunnelMonitor constraints that control this decision

TunnelMonitor is firmware for unattended field operation. Its priority is durable data, bounded liveness, simple ownership, and useful diagnostics. The LSM6 integration must follow the existing architecture rather than create a special path.

### One owner of I2C

`I2cTask` is the only I2C owner. A private LSM6 adapter may call the chip library, but no other task, service, ISR, CLI command, web handler, or display code may touch the library or bus directly.

Relevant project authority:

- `docs/guidelines/i2c_peripherals.md:28-34`
- `docs/guidelines/i2c_peripherals.md:100-118`
- `docs/guidelines/ownership.md:28-33`
- `docs/guidelines/ownership.md:46-49`
- `src/i2c/I2cTask.cpp:919-995`

The normal owner path advances at most one backend callback per poll. A multi-register operation must therefore be a staged job, not a blocking library call.

### The device is not in the current product contract

No current authoritative project contract defines an LSM6, IMU, accelerometer, gyroscope, tilt, shock, or motion source. The current I2C inventory is RTC, FRAM, ENV, INA228, and OLED.

Evidence:

- `docs/guidelines/i2c_peripherals.md:18-26`
- `docs/guidelines/i2c_peripherals.md:145-154`
- `include/TunnelMonitor/BoardPins.h:80-87`
- `platformio.ini:54-57`

The product requirement must come first. The library must not decide whether the product stores raw acceleration, tilt, vibration statistics, or event counts.

### Address is available, but must be frozen

The chip supports `0x6A` and `0x6B`. Neither conflicts with the current TunnelMonitor addresses. The board profile still needs one exact SA0 strap and one exact address. Runtime probing of both addresses must not be used as product selection.

### No interrupt pin exists in the board profile

`BoardPins` has no LSM6 INT1 or INT2 assignment. An unlisted GPIO must not be assumed available. A polling-only first version is reasonable for low-rate snapshots. FIFO watermark, wake, shock, or motion events require a deliberate board revision with pin, polarity, drive, latch, and HIL decisions.

An interrupt handler may record a bounded flag and timestamp. It must not perform I2C.

### Existing capacities are tight

Several current capacities are already at their configured product limit:

- The known-I2C-device table is fixed at five rows.
- Measurement runtime tracks four source requests: SHZK, VibWire, ENV, and power.
- Public device-health capacity and current production count are both 16.
- `Sample` uses 37 of 48 field slots.
- `I2cResult` payload capacity is 128 bytes and result queue depth is eight.

Evidence:

- `include/TunnelMonitor/i2c/I2cConfig.h:9-11`
- `include/TunnelMonitor/i2c/I2cConfig.h:81-82`
- `include/TunnelMonitor/measurement/MeasurementScheduler.h:11-16`
- `include/TunnelMonitor/contracts/Capacities.h:81-100`
- `include/TunnelMonitor/contracts/Sample.h:42-80`
- `src/system/SystemRuntime.cpp:1597-1643`

A simple seven-field snapshot, six axes plus temperature, would bring the scalar sample to 44 fields and still fit. It would leave only four free fields. Do not add raw axes, converted axes, magnitudes, Euler angles, statistics, and event counters together without an explicit schema and capacity change.

For platformization, selected product descriptors should calculate device inventory, source count, sample fields, and health rows at compile time. Existing enum values must remain stable and new `DeviceId` and `I2cOperation` values must be appended, not inserted or renumbered.

## Strengths to keep

The library should be refactored, not replaced without cause. The following parts are useful:

- Framework-neutral core and public headers.
- Injected read/write callbacks; the library does not own `Wire`, pins, a task, or a mutex.
- Timeout value passed to the transport callback.
- No dynamic allocation or dynamic STL containers in production code.
- Fixed-size structures and bounded stack buffers.
- Correct validation of addresses `0x6A` and `0x6B` and WHO_AM_I `0x6A`.
- Correct little-endian signed 16-bit raw decoding, including edge values.
- One contiguous 14-byte temperature, gyro, and accelerometer burst.
- BDU and IF_INC enabled in the default configuration.
- Useful enum validation for ODR, full scale, power mode, filters, and FIFO fields.
- A staged poll engine with an explicit I2C instruction budget.
- A raw sample cache committed only after the burst transport succeeds.
- Useful transport and device status codes.
- Detection of ambiguous configuration state through `cachedConfigDirty`.
- Explicit FIFO overrun status reporting.
- A broad native test suite and retained real-device evidence.

These are the correct foundations for a platform driver. The refactor should concentrate on lifecycle, operation ownership, result semantics, and complete desired-state replay.

## Hard findings and required refactors

### LSM6-01 — The sensing role is not defined in TunnelMonitor

**Severity:** integration blocker, project decision
**Applies to:** all uses

The current TunnelMonitor product and board contracts do not include this sensor. A low-rate orientation sample, a shock detector, and a vibration waveform source require different ODR, FIFO, interrupt, schema, storage, and health policies.

**Required before library integration**

Freeze:

1. Exact supported part: LSM6DS3TR-C only, or a named compatible family.
2. Selected product profile and board revision.
3. Required or optional device role.
4. Exact address and SA0 strap.
5. Polling or interrupt operation.
6. Sensor mounting direction relative to tunnel axes.
7. Output: snapshot, tilt, event, statistics, or waveform.
8. ODR, full scale, filters, and power modes.
9. Calibration, persistence, and validity policy.
10. Sample schema and configuration version.

**Reasonable first decision**

Start with a polling-based, low-rate accelerometer/gyro snapshot. Keep FIFO, interrupts, sensor-hub functions, and waveform storage out until a product requirement needs them.

### LSM6-02 — `begin()`, `recover()`, and `end()` do not fit the owner loop

**Severity:** high, library refactor required
**Applies to:** all production uses

`begin()` clears the current object state before validating the candidate configuration, reads WHO_AM_I, and then applies about 16 I2C transactions synchronously. With the default 50 ms callback timeout, one call exposes about 850 ms of cumulative callback budget. `recover()` repeats WHO_AM_I and the configuration sequence.

Evidence:

- `src/LSM6DS3TR.cpp:512-642`
- `src/LSM6DS3TR.cpp:688-714`
- `src/LSM6DS3TR.cpp:2982-3032`

`end()` performs two hidden power-down writes, ignores both results, and clears the object:

- `src/LSM6DS3TR.cpp:649-670`

This cannot be made safe by calling `begin()` or `recover()` inside one `I2cTask` callback. It would hold the owner and make timeout, fairness, and cancellation unclear.

**Required refactor**

- Make configuration binding zero-I/O.
- Validate a new configuration before replacing a working binding.
- Add staged probe, configure, reset, and recovery jobs.
- Perform no more than the owner-approved callback budget per `poll()`.
- Make `unbind()` or `end()` bus-silent.
- Provide a separate explicit and fallible power-down job.
- Publish the terminal status only after configuration readback succeeds.

### LSM6-03 — Jobs are not globally exclusive and cannot be cancelled

**Severity:** high, library refactor required
**Applies to:** all production uses

Job starters check `pollBusy()`, but direct reads, configuration setters, raw-register access, blocking calibration, and self-test are still callable while a job is active.

Examples:

- job start checks: `src/LSM6DS3TR.cpp:745-795`
- direct sample read: `src/LSM6DS3TR.cpp:1693-1764`
- full-scale and ODR setters: `src/LSM6DS3TR.cpp:1826-1902`
- blocking self-test: `src/LSM6DS3TR.cpp:824-1009`

A setter can change ODR or scale after the staged status check and before the data burst. A raw write can change a register while a staged configuration refresh is building its snapshot.

There is no public cancellation. `README.md:319-321` says CLI cancel only clears CLI state; the driver job continues until completion or failure, while `end()` is the hard reset.

**Required refactor**

- Enforce one active hardware operation across every public hardware-touching method.
- Keep only cache-only getters available during an active operation.
- Add `cancelActiveJob()` that performs no bus I/O and discards pending output.
- If cancellation follows a partial configuration write, finish with configuration state `Unknown`.
- Make deadline expiry and cancellation distinct terminal results.

This cancellation is important for TunnelMonitor. The owner must be able to expire a queue command without leaving the driver permanently busy.

### LSM6-04 — Freshness and field validity are unsafe

**Severity:** high, data correctness
**Applies to:** all sample uses

The request is accepted when either the accelerometer or gyro is active, but every returned structure always contains acceleration, gyro, and temperature fields. There is no validity mask.

Evidence:

- activation check: `src/LSM6DS3TR.cpp:725-729`
- readiness checks only accel and gyro: `src/LSM6DS3TR.cpp:1222-1228`
- all fields always decoded: `src/LSM6DS3TR.cpp:1233-1241`
- result types have no validity: `include/LSM6DS3TR/LSM6DS3TR.h:41-53`

This can expose powered-down gyro output as a valid value. Temperature is returned even though its ready flag is not checked and its refresh timing differs from the motion ODR.

A failed new request also leaves the previous `_hasSample` value intact. `getMeasurement()` and `getRawMeasurement()` accept any historical cache, not specifically the latest successful request:

- request handling: `src/LSM6DS3TR.cpp:745-767`
- failure paths: `src/LSM6DS3TR.cpp:1213-1219`
- getters: `src/LSM6DS3TR.cpp:1664-1690`

The adapter could accidentally publish yesterday's good cache as the response to today's failed operation.

**Required refactor**

- Return operation status and sample in one atomic terminal result.
- Add `validMask`, `freshMask`, `sequence`, and `configGeneration`.
- Distinguish `readUptimeMs` from physical sample time.
- Expose `takeSample(expectedToken)` or immutable sequence-numbered results.
- Clear or supersede old readiness on every new request.
- Do not treat temperature as same-cycle motion data.

### LSM6-05 — A cached sample can be converted with the wrong full scale

**Severity:** high, concrete correctness defect
**Applies to:** converted samples

The driver caches raw codes, but converts them later using the current full-scale configuration. Full-scale setters do not invalidate or version the cache.

Evidence:

- raw cache commit: `src/LSM6DS3TR.cpp:1742-1764`
- converted getter: `src/LSM6DS3TR.cpp:1664-1678`
- conversion uses current settings: `src/LSM6DS3TR.cpp:1804-1819`
- scale setters: `src/LSM6DS3TR.cpp:1871-1902`

Example:

1. Read a sample at plus/minus 2 g.
2. Change the accelerometer to plus/minus 16 g.
3. Read the cached converted sample.
4. Old raw codes are interpreted with the new sensitivity and can be wrong by 8 times.

**Required refactor**

- Store full scale or configuration generation with every raw sample.
- Invalidate cached samples after any interpretation-changing write.
- Prefer conversion from a self-contained raw result rather than mutable driver state.

### LSM6-06 — BDU and coherence guarantees are overstated

**Severity:** high for strict vector coherence; medium for slow condition snapshots
**Applies to:** managed multi-byte samples

`requestMeasurement()` requires BDU only when both accel and gyro are active. A single XYZ sensor still needs multi-byte reads and can tear when BDU is disabled. `requestMeasurement(false)` bypasses readiness and BDU checks.

Evidence:

- BDU condition: `src/LSM6DS3TR.cpp:730-737`
- 14-byte burst: `src/LSM6DS3TR.cpp:1233-1235`

The datasheet states that BDU protects each output LSB/MSB pair. It does not by itself prove that X, Y, Z, accelerometer, gyro, and temperature all belong to one conversion instant. This matters most at high ODR.

**Required refactor**

- Require BDU for every managed multi-byte sample.
- Keep a no-ready/no-BDU read only as a clearly unsafe diagnostic, if retained.
- Report sample quality honestly: ready-checked, direct-unverified, or FIFO-framed.
- Define an accepted vector-skew bound for the selected low-rate profile.
- If same-cycle vectors are required, use a proven DRDY/FIFO acquisition policy instead of claiming the burst alone guarantees it.

### LSM6-07 — Unknown configuration does not block interpreted data

**Severity:** high, data correctness
**Applies to:** all configured reads

The library marks `_cachedConfigDirty` after ambiguous and partial writes. This is a good foundation. The flag is only diagnostic, however. Sampling and conversion continue while ODR, full scale, filters, BDU, or FIFO state may differ between the cache and device.

Evidence:

- public dirty state: `include/LSM6DS3TR/LSM6DS3TR.h:264-265`
- example dirty transition: `src/LSM6DS3TR.cpp:1842-1848`
- sample/conversion paths: `src/LSM6DS3TR.cpp:1664-1823`

**Required refactor**

- Replace the Boolean with an explicit configuration state: `Unconfigured`, `Applying`, `Known`, or `Unknown`.
- Block interpreted measurements unless measurement-affecting configuration is known.
- Allow diagnostic raw reads only with a `ConfigUnknown` quality flag.
- Clear `Unknown` only after a complete readback/reconcile job.
- Increment a configuration generation only after a verified apply.

### LSM6-08 — Desired-state application and recovery are incomplete

**Severity:** high if events/interrupts are used; medium for the recommended polling scope
**Applies to:** boot, reset, brownout, and recovery

Initialization writes a subset of device state without first resetting or adopting every retained register. Whole-register builders also force fields that are not represented in the public desired configuration.

Examples:

- CTRL3_C is rebuilt with BDU and IF_INC and therefore forces default interrupt electrical fields: `src/LSM6DS3TR.cpp:2990-2993`
- CTRL4_C, CTRL6_C, CTRL7_G, CTRL8_XL, and CTRL10_C are built from partial cached models: `src/LSM6DS3TR.cpp:2995-3010`
- interrupt and event registers are exposed mainly through raw access: `README.md:50-53`, `README.md:259-272`

After an MCU-only restart, old event registers may remain active. After sensor reset or brownout, raw-configured routes and thresholds are lost and `recover()` cannot replay them.

**Required refactor**

- Choose one policy: reset then apply a complete desired register image, or read and adopt an explicitly defined register set.
- Do not combine managed whole-register writes with undocumented retained raw state.
- Read back and compare every register that controls a supported feature.
- For a polling-only first profile, explicitly disable interrupts/events and omit their raw configuration from production.
- If events are required later, add typed electrical, route, latch, threshold, duration, and source-snapshot fields to the replayable profile.

### LSM6-09 — Reset and boot sequencing does not follow the device procedure

**Severity:** high, device-state correctness
**Applies to:** reset, boot, and recovery

AN5130 describes a sequence that powers down the gyro, places the accelerometer in high-performance mode, then issues BOOT or SW_RESET. The current synchronous and staged paths write CTRL3_C directly without staging those modes:

- staged reset/boot: `src/LSM6DS3TR.cpp:1328-1375`
- synchronous reset/boot: `src/LSM6DS3TR.cpp:2536-2613`

The application note also states that registers are inaccessible for about 15 ms during boot. The current staged and synchronous paths attempt to read CTRL3_C on the next loop/call and treat the first transport failure as terminal.

Current fake-bus tests auto-clear the command bit immediately and do not model the inaccessible interval or required write ordering.

**Required refactor**

- Save the desired runtime profile.
- Stage gyro power-down.
- Stage accelerometer high-performance mode.
- Issue BOOT or SW_RESET.
- Enforce a 15 ms no-I2C gate before the first status read.
- Poll with a whole-operation deadline.
- Reapply and read back the full supported profile.
- Test exact write order and failure at every stage.

Do not fix this with a blocking sleep. The owner supplies monotonic time and advances the job.

### LSM6-10 — Runtime ODR and power changes have no settling state

**Severity:** medium-high
**Applies to:** dynamic configuration and startup

The device guidance requires discarding or masking early data after ODR, filter, or power-mode changes. Gyro wake from power-down includes a material startup interval. Current setters commit and return immediately, preserve the old sample, and permit another read:

- ODR setters: `src/LSM6DS3TR.cpp:1826-1868`
- power setters: `src/LSM6DS3TR.cpp:2003-2051`

**Required refactor**

- Track `validAfter` or a required discard count after begin and interpretation-changing writes.
- Invalidate the previous sample.
- Reject or defer production sample requests while settling.
- Provide a pure `requiredSettleUs(profileChange)` helper.

The simplest TunnelMonitor policy is a static runtime profile. Do not add automatic range switching unless a measured product requirement needs it.

### LSM6-11 — Driver-owned health and recovery conflict with `I2cTask`

**Severity:** high for TunnelMonitor architecture
**Applies to:** all production uses

The driver owns READY, DEGRADED, and OFFLINE states, latches offline, blocks normal bus access, and requires `recover()`:

- health API: `include/LSM6DS3TR/LSM6DS3TR.h:15-21`, `230-265`
- offline admission gate: `src/LSM6DS3TR.cpp:2975-2979`
- recovery: `src/LSM6DS3TR.cpp:688-714`

TunnelMonitor already owns device health, backoff, queue expiry, bus recovery, and diagnostics in `I2cTask`. Two owners can disagree and can block each other's recovery path.

The library also counts health per physical transaction. Every successful register access resets consecutive failures. A multi-register operation that repeatedly fails late can keep resetting the failure streak before the logical operation fails.

**Required refactor**

- Remove offline admission and automatic recovery policy from the chip core.
- Return precise per-operation status and configuration state.
- Let `I2cTask` own offline state, backoff, and recovery decisions.
- Passive counters are acceptable only if they never block access or start I2C.

### LSM6-12 — FIFO is not a usable acquisition API

**Severity:** high if FIFO is selected; not required for the first snapshot scope
**Applies to:** vibration, watermark, or batch capture

The device FIFO stores untagged 16-bit words. The starting `FIFO_PATTERN` is required to identify dataset and axis. The current API does not provide that association.

`startFifoDrain()` reads every word into a local buffer and discards it. Only the number of words read is exposed:

- public API: `include/LSM6DS3TR/LSM6DS3TR.h:728-733`
- discard loop: `src/LSM6DS3TR.cpp:1548-1586`

If the purpose is to empty the FIFO, this should be named `purgeFifo()`. It is not a data drain for a consumer.

Fourth-dataset support is also incomplete:

- `FifoConfig` advertises temperature and timestamp/step storage: `include/LSM6DS3TR/LSM6DS3TR.h:76-87`
- enable bits are built in `_buildFifoCtrl2()`: `src/LSM6DS3TR.cpp:3192-3197`
- `_buildFifoCtrl4()` does not configure the required fourth-dataset decimation: `src/LSM6DS3TR.cpp:3204-3208`

The API allows temperature and timestamp/step booleans together even though the documented fourth-dataset selection has constraints. Managed reconfiguration also does not first enter bypass or clearly report unread data loss. The datasheet states that the first sample after a FIFO mode switch must be discarded.

**Required only if FIFO is approved**

- Return data through a fixed caller buffer or bounded callback.
- Capture the starting pattern before reading words.
- Decode dataset, axis, and pattern position for every word.
- Provide explicit fourth-dataset source and decimation enums.
- Validate mutually exclusive combinations.
- Enter bypass before reconfiguration and report discarded words.
- Handle the required first-sample discard.
- Report initial and final unread count, watermark, full, and overrun.
- Treat overrun as data loss, never as an ordinary valid batch.
- Add a throughput and maximum-drain-latency calculation for the selected profile.

At 400 kHz on a shared bus, the highest public ODR combinations cannot be assumed sustainable through one-word transactions. If the product only needs slow orientation, keep FIFO in bypass and do not build this feature yet.

### LSM6-13 — Interrupt and motion-event configuration is not platform-safe

**Severity:** conditional
**Applies to:** INT1/INT2, wake, tap, free-fall, 6D, tilt, FIFO event

The library knows the register addresses but does not provide a complete typed and replayable event profile. Raw writes are not enough for unattended recovery. They can be lost after reset and may be silently overwritten by managed whole-register builders.

TunnelMonitor also has no interrupt GPIO assigned.

**Required only if an interrupt use case is approved**

- Add `InterruptPin`, `InterruptPolarity`, `InterruptDrive`, and `InterruptLatchMode`.
- Add a typed route mask for supported sources.
- Add typed threshold/duration fields for the selected events.
- Add one decoded source snapshot read that documents clear-on-read behavior.
- Include every supported interrupt register in desired-state apply and readback.
- Add a board-profile pin and electrical decision.
- Keep I2C outside the ISR.

Do not add all embedded functions just because the chip has them. Add only the event modes used by a selected product profile.

### LSM6-14 — Blocking diagnostics and clock semantics are unsafe

**Severity:** medium-high
**Applies to:** self-test, calibration, synchronous reset/boot

`runSelfTest()` performs a large number of transactions in one public call. Its timing depends on optional `Config::nowMs`. When the clock callback is absent, `_settleSelfTest()` performs one status read and returns instead of waiting the required 100 ms or 800 ms:

- optional time hook: `include/LSM6DS3TR/Config.h:144-146`
- self-test: `src/LSM6DS3TR.cpp:824-1009`
- settle helper: `src/LSM6DS3TR.cpp:3052-3071`

Staged jobs use caller-provided `poll(nowMs)`, while health timestamps use the optional callback. These epochs are not required to match. Public sample time is 32-bit and is I2C read time, not physical conversion time.

**Required refactor**

- Make self-test and calibration staged diagnostic jobs.
- Use one owner-provided monotonic time domain.
- Use wrap-safe deadline helpers internally and a 64-bit outward uptime.
- Name the normal timestamp `readUptimeMs` unless a sensor timestamp is actually captured.
- Preserve both primary failure and restoration failure.
- Mark configuration unknown if diagnostic restoration fails.

Blocking diagnostic convenience functions may exist only in a separate non-production facade whose behavior and maximum time are explicit. They must not be used by `I2cTask`.

### LSM6-15 — Mounting and calibration are not product-safe

**Severity:** medium-high for physical meaning
**Applies to:** all interpreted motion values

The driver reports sensor-native axes. TunnelMonitor has no frozen mapping from sensor axes to tunnel/product axes. Bias capture assumes a stationary Z-up orientation. That assumption is not valid for an arbitrary PCB installation.

Manual bias setters also accept NaN and infinity and apply them to every converted result:

- setters: `src/LSM6DS3TR.cpp:3220-3230`
- application: `src/LSM6DS3TR.cpp:3236-3245`

**Required refactor and project work**

- Define a signed permutation transform in the product board descriptor.
- Validate that every output axis maps to exactly one sensor axis.
- State the positive rotation convention.
- Do not assume sensor Z is product up.
- Make bias setters fallible and reject non-finite or unreasonable values.
- Store calibration version, units, profile, validity, and provenance in TunnelMonitor persistence.
- Keep persistence policy out of the chip driver.

Six physical orientations and known positive/negative rotation are required HIL gates.

### LSM6-16 — Public helper units and header hygiene contain concrete defects

**Severity:** medium
**Applies to:** public API and packaging

The Doxygen for `accelSensitivity()` says g/LSB and `gyroSensitivity()` says dps/LSB. The implementation and tests return mg/LSB and mdps/LSB:

- declarations: `include/LSM6DS3TR/LSM6DS3TR.h:521-528`
- implementation: `src/LSM6DS3TR.cpp:1995-2000`
- tables: `src/LSM6DS3TR.cpp:322-351`
- tests: `test/test_basic.cpp:1627-1636`

The public `Config.h` also undefines a consumer macro named `DISABLED` before declaring `FifoDecimation`. A library header must not silently change application preprocessor state.

**Required refactor**

- Rename helpers to `accelSensitivityMgPerLsb()` and `gyroSensitivityMdpsPerLsb()`, or return the documented base units.
- Prefer exact integer sensitivity helpers for firmware conversion.
- Rename `FifoDecimation::DISABLED` to `Off` or `NotStored`.
- Remove the public `#undef`.

### LSM6-17 — Release metadata and reproducibility are not clean

**Severity:** medium, release gate
**Applies to:** exact dependency pinning

`library.json` and generated `Version.h` report 1.2.0, but `idf_component.yml:1` reports 1.1.0. An ESP-IDF package made from the v1.2.0 source therefore identifies itself incorrectly.

Build inputs are also broad:

- PlatformIO uses unpinned `platform = espressif32`.
- CI installs an unpinned PlatformIO package.
- IDF metadata accepts `idf >=6.0.1`.

**Required refactor/release work**

- Generate all package versions from one source.
- Pin the tested PlatformIO platform/toolchain in release CI.
- Actually compile the ESP-IDF example if ESP-IDF is advertised.
- In TunnelMonitor, exact-pin the accepted library tag or commit.

### LSM6-18 — TunnelMonitor contracts and capacities need deliberate extension

**Severity:** integration blocker after library refactor
**Applies to:** TunnelMonitor implementation

There is no `DeviceId`, `I2cOperation`, command, result, health row, public status, measurement source, or sample schema for motion data. Adding the sensor alongside all current devices exceeds current health capacity and current measurement-source capacity.

**Required project refactor**

- Append `DeviceId::MotionSensor` and `I2cOperation::ReadMotion` without renumbering existing values.
- Add the known-device row only to selected profiles that contain the sensor.
- Refactor hard-coded measurement sources into bounded compile-time descriptors.
- Calculate selected-profile health, source, and sample capacities with static checks.
- Add a compact copied `MotionStatus`; CLI, web, display, and cloud consume status, not the driver.
- Update sample schema, CSV header/row, replay, cloud payload, and profile identifiers together.

Do not send high-rate FIFO batches through the ordinary eight-entry `I2cResult` queue as scalar samples.

## Refactor shape recommended for the library

The following is intentionally small. It is a chip driver, not an IMU framework.

### Lifecycle

Use one explicit lifecycle:

```text
Unbound -> Bound -> Probing -> Applying -> Settling -> Ready
                                      \-> ConfigUnknown
Any active job --cancel--> Bound or ConfigUnknown
Ready --reset/recover--> Applying -> Settling -> Ready
```

Recommended operations:

```cpp
Status bind(const DriverConfig& config);       // zero I2C
Status startProbe();
Status startConfigure(const DeviceProfile& profile);
Status startSample(const SampleRequest& request);
Status startReset();
Status startSelfTest(const SelfTestRequest& request);
PollResult poll(uint64_t nowMs, uint8_t maxTransactions = 1);
Status cancelActiveJob();                      // zero I2C
Status takeResult(OperationToken token, OperationResult& out);
void unbind();                                 // zero I2C
```

Rules:

- One active hardware operation.
- One transport callback per normal TunnelMonitor poll.
- No sleeps, internal retry loops, task creation, bus recovery, logging, or offline gate.
- Whole-operation deadline is supplied by the owner or request.
- Every terminal result is published atomically.
- Partial writes finish as `ConfigUnknown` until readback/reconcile.

### Configuration model

Use one complete profile for the features the library claims to manage:

```cpp
struct DeviceProfile {
  AccelOdr accelOdr;
  AccelFullScale accelFullScale;
  AccelPowerMode accelPowerMode;
  GyroOdr gyroOdr;
  GyroFullScale gyroFullScale;
  GyroPowerMode gyroPowerMode;
  AccelFilter accelFilter;
  GyroFilter gyroFilter;
  bool blockDataUpdate;
  FifoProfile fifo;             // bypass-only in first Tunnel scope
  InterruptProfile interrupts; // disabled-only in first Tunnel scope
};
```

The applied state needs:

- desired profile
- verified profile
- `ConfigurationState`
- monotonic `configGeneration`
- `validAfterUptimeMs`
- last mismatch register and values for diagnostics

Do not expose per-read register knobs through TunnelMonitor's generic I2C command payload. Product configuration is compile-time/private adapter policy.

### Atomic sample result

Keep raw data as the authoritative chip result. Conversion should be pure and must not depend on mutable driver configuration.

```cpp
enum class SampleQuantity : uint8_t {
  Acceleration = 1U << 0,
  AngularRate = 1U << 1,
  Temperature = 1U << 2,
};

enum class SampleQuality : uint8_t {
  ReadyChecked,
  DirectUnverified,
  FifoFramed,
  ConfigUnknown,
  Settling,
};

struct RawSampleResult {
  RawAxes accel{};
  RawAxes gyro{};
  int16_t temperatureRaw{0};
  uint8_t validMask{0};
  uint8_t freshMask{0};
  SampleQuality quality{SampleQuality::DirectUnverified};
  uint32_t sequence{0};
  uint32_t configGeneration{0};
  uint64_t readUptimeMs{0};
};
```

Either include the exact full-scale values in this result or make the configuration generation resolve to an immutable profile snapshot. Invalidate prior samples on relevant configuration changes.

### Error model

Keep transport and device errors precise. Add explicit logical results where they are currently inferred:

- `DeadlineExpired`
- `Cancelled`
- `ConfigurationUnknown`
- `ConfigurationMismatch`
- `Settling`
- `DataNotReady`
- `StaleResult`
- `FifoOverrun`
- `FifoDataDiscarded`
- `UnsupportedProfile`

TunnelMonitor's adapter maps these into the stable project `I2cResult` and device health policy. The core must not return OFFLINE as an admission decision.

## Recommended TunnelMonitor contract shape

Do not expose third-party driver objects or chip register types in public project contracts.

For the recommended snapshot scope, a compact result can fit the existing 128-byte I2C payload:

```cpp
enum class MotionSensorKind : uint8_t {
  Unknown = 0,
  Lsm6ds3trC = 1,
};

enum class MotionField : uint8_t {
  AccelX = 0,
  AccelY = 1,
  AccelZ = 2,
  GyroX = 3,
  GyroY = 4,
  GyroZ = 5,
  Temperature = 6,
};

struct MotionReadCommand {
  uint8_t flags{0};
  uint8_t reserved[3]{};
};

struct MotionReadResult {
  MotionSensorKind sensorKind{MotionSensorKind::Unknown};
  uint8_t address{0};
  uint16_t validMask{0};
  uint16_t statusFlags{0};
  int32_t accelMilliG[3]{};
  int32_t gyroMilliDegreesPerSecond[3]{};
  int32_t temperatureMilliCelsius{0};
  uint32_t configurationGeneration{0};
  uint32_t sequence{0};
  uint64_t captureUptimeMs{0};
};
```

Useful status flags are:

- configuration verified
- accelerometer ready/fresh
- gyro ready/fresh
- temperature ready/fresh
- settling
- sensor reset observed
- configuration mismatch observed
- FIFO overrun/data lost, only if FIFO is later enabled

Use fixed integer units at the durable TunnelMonitor contract. Floating-point convenience can remain inside diagnostics or pure conversion helpers. Do not persist a float waveform.

## Useful enums, types, and helpers

### Core types worth adding

- `SensorAddress` with `Low = 0x6A`, `High = 0x6B`
- `OperationToken`
- `JobKind`
- `JobState`
- `PollResult`
- `ConfigurationState`
- `SampleQuantityMask`
- `SampleQuality`
- `RawSampleResult`
- `AppliedProfileSnapshot`
- `Calibration` with version and finite bounds

Conditional FIFO types:

- `FifoDataset`
- `FifoAxis`
- `FifoFourthDataset`
- `FifoPatternCursor`
- `FifoElement { raw, dataset, axis, patternIndex }`
- `FifoDrainResult { wordsRead, initialPattern, finalUnreadWords, overrun, discarded }`

Conditional interrupt types:

- `InterruptPin`
- `InterruptPolarity`
- `InterruptDrive`
- `InterruptLatchMode`
- `InterruptRouteMask`
- `InterruptSourceSnapshot`

Project installation types:

- `MotionAxis`
- `AxisSign`
- `AxisTransform`
- `MotionCalibration`
- `MotionStatus`

### Pure helpers worth adding

- `isValidAddress()`
- `odrMilliHz()` and `odrPeriodUs()`
- `requiredSettleUs()` or `requiredDiscardSamples()`
- `validateProfile()` returning a typed reason
- `accelSensitivityMicroGPerLsb()`
- `gyroSensitivityMicroDpsPerLsb()`
- `decodeAcceleration()`
- `decodeAngularRate()`
- `decodeTemperatureMilliC()`
- `fifoPatternLength()`
- `decodeFifoElement()`
- `fifoFillTimeUs()`
- `estimateI2cBytesPerSecond()`
- `validateAxisTransform()`
- `applyAxisTransform()`
- `isValidCalibration()`
- `toString()` for stable diagnostic enums

Helpers must be deterministic, allocation-free, and easy to table-test.

## API cleanup

The current public surface is broad. For platform use, separate it into three clear groups:

1. **Production owner-safe API** — bind, staged jobs, poll, cancel, take result, cache-only status.
2. **Pure codec API** — validation and conversions with no I2C.
3. **Diagnostic/raw API** — explicitly unsafe for production policy and unavailable during any job.

Recommended cleanup:

- Delete copy and move operations for a live driver that contains callbacks and mutable job state.
- Rename ambiguous unit helpers.
- Rename the discard-only FIFO operation to purge unless it starts returning data.
- Remove false-success FIFO fields until fully implemented.
- Avoid raw writes to read-only output and identity registers.
- Do not make a successful raw write trigger a hidden long synchronous refresh.
- Keep raw access out of the TunnelMonitor production adapter.
- Remove claims that unsupported embedded-bank features can be configured through the raw main-bank API.
- Keep diagnostic strings optional; use typed codes at the owner boundary.

## What not to do

The following band-aids would preserve the failure modes and should be rejected:

- Calling the current `begin()` or `recover()` from one I2C owner callback.
- Adding another firmware task just for the IMU.
- Letting an adapter stop polling a timed-out job without cancelling its internal state.
- Reading `getMeasurement()` after a failure and assuming the cached value is new.
- Checking `cachedConfigDirty()` only for diagnostics while continuing scaled samples.
- Reapplying interrupt settings through scattered raw writes after every reset.
- Treating `startFifoDrain()` as waveform acquisition.
- Sending high-rate FIFO words through the scalar measurement queue.
- Adding sleeps for reset, boot, settling, or self-test inside `I2cTask`.
- Duplicating full-scale conversion and register behavior in TunnelMonitor to compensate for the library.
- Building a generic IMU service registry or plugin system for one concrete device.

## Nice-to-have improvements

These are useful after the hard requirements. They must not delay the first safe snapshot integration.

- A compact configuration-difference report showing the first mismatched register.
- Compile-time profile validation for constant product configurations.
- Fixed-capacity trace entries for the last few driver job transitions, supplied by the owner.
- A formatter that prints raw and converted samples with units for CLI diagnostics.
- A bandwidth helper that reports expected I2C occupancy for snapshot or FIFO profiles.
- A six-orientation HIL command that reports expected versus measured axis/sign.
- A self-test result that includes baseline, stimulated delta, limits, and restore status without floats where practical.
- A device revision/identity snapshot containing WHO_AM_I and accepted profile version.
- Separate `purgeFifo()` and `drainFifoData()` names so data loss is never ambiguous.
- A host-side FIFO pattern decoder test tool using captured register bytes.

Do not add sensor fusion, Euler angles, quaternions, step algorithms, machine learning, or generic motion frameworks without a product requirement. Derived tilt and vibration statistics are application policy and normally belong above the chip driver.

## Validation performed during this audit

The following checks were run against exact `v1.2.0` with PlatformIO Core 6.1.19:

| Check | Result |
|---|---|
| Core timing guard | PASS |
| Generated version check | PASS |
| CLI contract check | PASS |
| ESP-IDF example contract text check | PASS |
| HIL parser self-test and dry-run plan | PASS, 10 planned smoke commands |
| Chip-document coverage check | PASS |
| Native Unity suite | PASS, 113/113 in 4.318 s |
| ESP32-S3 Arduino build | PASS, RAM 22,640 B, flash 426,082 B |
| ESP32-S2 Arduino build | PASS, RAM 37,040 B, flash 416,733 B |
| PlatformIO package creation | PASS, temporary package removed |
| `git diff --check` | PASS |
| Doxygen generation | Completed with three unresolved-reference warnings |
| Actual ESP-IDF compile | NOT RUN; `idf.py` unavailable and CI does not compile it |
| New physical HIL | NOT RUN |
| TunnelMonitor target integration build | NOT RUN; no integration exists |

The S2/S3 builds compile the library bring-up CLI, not TunnelMonitor's target board profile or shared-bus runtime. The ESP-IDF contract check is a text/contract check, not a compiler result.

### Native test gaps to close

Keep the 113-case baseline and add:

- sample cache invalidation and scale-generation tests
- latest-operation token/freshness tests
- validity masks for every powered-down sensor combination
- BDU enforcement for accel-only, gyro-only, and direct diagnostic paths
- active-job exclusion for every hardware-touching method
- bus-silent cancellation at every job stage
- dirty/unknown configuration measurement gates
- reset and boot exact write-order tests
- boot fake-bus mode that rejects every access during the first 15 ms
- failure injection at every configure/reset/reconcile stage
- settling and discard behavior after each supported profile change
- deadline tests around `UINT32_MAX` if any 32-bit internal deadline remains
- all public accelerometer and gyro full-scale codec values
- all supported ODR encodings and power-mode restrictions
- self-test timing with and without an owner clock
- restore failure preserving configuration unknown state
- copy/move compile checks after deletion
- public header macro-hygiene check

If FIFO is approved, also add:

- pattern start at every possible word
- all dataset and axis mappings
- wrap, partial frame, watermark, full, and overrun
- fourth-dataset combinations and decimation
- first-sample discard after mode change
- reconfiguration with unread words
- cancellation with partial consumer buffer
- throughput limits for the accepted product profile

### HIL evidence already present

The retained summary in `docs/reports/hil-evidence-summary.md` is useful positive evidence for ordinary Arduino behavior on one device:

- ESP32-S3 fixture, GPIO 8/9, 400 kHz, 50 ms timeout
- LSM6DS3TR-C at `0x6A`, WHO_AM_I `0x6A`
- smoke: 10/10 pass
- broad sweep: 120/120 pass
- 8-hour run: 41,261 PASS, 0 FAIL, 293 UNKNOWN
- requested 20-hour run stopped around 7 h 33 min: 39,825 PASS, 1 FAIL, 29 UNKNOWN; summary attributes these to serial framing
- later 30-minute run: 2,095/2,095 pass
- final 10-minute run: 1,102/1,102 pass

This is strong evidence for the tested fixture, including self-test, FIFO exercises, reset/recover, and ordinary stress. It is not TunnelMonitor acceptance evidence.

Limits:

- raw transcripts and structured artifacts were deleted, leaving a narrative summary that cannot be independently replayed
- no held SDA/SCL, injected NACK/timeout, or brownout test
- no sensor power-cycle/disconnect/reconnect while MCU stays alive
- no absent-device target test
- no `0x6B` hardware test
- no live ESP-IDF HIL
- no exact TunnelMonitor hardware revision 2.0.0 coexistence test
- no all-device shared-bus fairness/deadline test
- no mounting, orientation, bias, vibration, or environmental acceptance test
- no new physical HIL was performed in this audit

## Release gates for a low-rate TunnelMonitor snapshot

### Product and board gates

- [ ] Freeze LSM6DS3TR-C part and selected product profile.
- [ ] Freeze required/optional health role.
- [ ] Freeze SA0 strap and address.
- [ ] Freeze poll-only operation or add a reviewed interrupt pin.
- [ ] Freeze mounting transform and rotation convention.
- [ ] Freeze ODR, full scale, filters, power modes, and accepted skew.
- [ ] Freeze output fields, units, calibration, and schema/profile version.

### Library gates

- [ ] Zero-I/O bind and unbind.
- [ ] Staged probe, configure, sample, reset, and recovery.
- [ ] One hardware operation at a time.
- [ ] Bus-silent cancellation.
- [ ] Atomic tokened terminal result.
- [ ] Valid/fresh masks and configuration generation.
- [ ] Cached-scale bug fixed.
- [ ] BDU enforced for managed samples.
- [ ] Explicit configuration state and readback.
- [ ] Correct reset/boot order and 15 ms inaccessible gate.
- [ ] Settling/discard state.
- [ ] Driver-owned offline/recovery admission removed.
- [ ] Self-test made staged and clock-correct.
- [ ] Sensitivity units, bias validation, macro hygiene, and package version fixed.
- [ ] FIFO and interrupt features either omitted/disabled or fully typed and tested.

### TunnelMonitor native gates

- [ ] Stable append-only device and operation values.
- [ ] Selected-profile address and inventory compile checks.
- [ ] Reserved command bytes rejected.
- [ ] Result size stays at or below 128 bytes.
- [ ] Exact library error mapping.
- [ ] One transport callback per normal owner poll.
- [ ] Owner deadline, cancellation, queue expiry, absence, and reconnect.
- [ ] No hidden library retry, recovery, or health gate.
- [ ] Axis transform pure-function vectors.
- [ ] Measurement source and health capacities statically fit.
- [ ] Valid, stale, and error masks assemble correctly.
- [ ] CSV, replay, cloud payload, and schema/profile compatibility.
- [ ] CLI, web JSON, display, and inactive-profile visibility use copied status.
- [ ] No steady-state allocation and bounded stack/result storage.

### TunnelMonitor HIL gates

- [ ] Exact ESP32-S3-N16R8 board and hardware revision 2.0.0 or the approved new board revision.
- [ ] Exact sensor, supply, pull-ups, address strap, and production 400 kHz bus.
- [ ] WHO_AM_I and configuration readback after cold boot, MCU reset, sensor reset, sensor brownout, and I2C recovery.
- [ ] Six static orientations with expected axis sign and approximately plus/minus 1 g.
- [ ] Known clockwise/counter-clockwise rotation for gyro sign and scale.
- [ ] Stationary bias, noise, repeatability, and temperature behavior against frozen limits.
- [ ] Worst-case bus load with OLED, RTC, FRAM, ENV, power, scan, and recovery active.
- [ ] Absent device, reconnect, NACK, timeout, held-bus, and queue-expiry behavior.
- [ ] Long unattended soak with owner timing, queue depth, stack high-water, heap stability, and sample continuity.
- [ ] Retain condensed raw evidence sufficient to review failures.

For a vibration product, add a reference shaker or known excitation across the required frequency band. Hand motion is not a sufficient scale or bandwidth test.

## Suggested implementation order

1. Freeze the product sensing role and board facts.
2. Refactor library lifecycle, exclusivity, cancellation, and result semantics.
3. Correct reset/boot, settling, configuration readback, and error ownership.
4. Fix unit, calibration, header, and package defects.
5. Add library native tests and rerun Arduino/actual IDF builds.
6. Add a private `I2cTask` adapter for a low-rate snapshot.
7. Extend bounded product descriptors, health, status, and sample schema.
8. Run TunnelMonitor native integration tests.
9. Run exact-board shared-bus HIL and retain reviewable evidence.
10. Add FIFO or interrupt features only if the selected product requirement needs them.

## Final recommendation

Use LSM6DS3TR `v1.2.0` as the protocol and test baseline, not as the production integration version.

Make a clean breaking refactor in the library before adding it to TunnelMonitor. The minimum platform-ready result is a zero-I/O binding, staged and cancellable operations, one-operation exclusivity, verified configuration state, correct reset/boot timing, settling control, and one atomic validity-aware raw sample result. Keep health and recovery policy in `I2cTask`.

For the first TunnelMonitor profile, use polling and a low-rate snapshot with FIFO and interrupts explicitly disabled. This is the simplest safe design. If the product later requires events or waveform capture, extend the same desired-profile and staged-job model with typed interrupt or FIFO support; do not use scattered raw-register recipes or a second task as a shortcut.
