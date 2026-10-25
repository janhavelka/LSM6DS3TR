# 02 - Fourth FIFO dataset and counter provenance

**Status: NOT IMPLEMENTED.** Execute in
`C:\Users\Honza\Documents\Projects\LSM6DS3TR`.
Preparation baseline: `e262bf22f3d0cf035fbbca10f45b3e0752bf2bac`.
Historical audit tip: `035eff530e72ea43e4b42b02df0ea2ff05e33383`.
Read [the complete execution index](README.md) first. Its invariants, authority,
verification commands and Git/CI closeout apply. **Requires completed prompt
01**, with its actual resolution SHA recorded. Rebase the analysis on current
main without resetting to the preparation baseline.

## Execute this task

Extend the motion FIFO implementation with exactly one selected fourth dataset:
temperature, or timestamp plus step count. Implement the minimal typed timer
and pedometer ownership needed for that schema, including lifetime/discontinuity
evidence. This closes the fourth-dataset portion of audit R2 / historical H4.
The result must be decoded data with clear provenance, not merely extra writable
bits. Keep sensor-hub datasets and unrelated embedded engines unsupported.

## Verify the mechanism and authorities

At preparation, the driver has no fourth-dataset selection or counter result;
the managed image clears `CTRL10_C`, `MASTER_CONFIG`, FIFO_CTRL2/4 and
WAKE_UP_DUR. A boolean `fifo.enabled` could not distinguish temperature from
step/timestamp formats. Prompt 01 supplies the current bounded FIFO job/schema;
inspect that implementation, its configure sequencing and fake model before
extending it. Do not make another drain engine or decoder framework.

Read `Config.h`, `LSM6DS3TR.h`, `CommandTable.h`, builders/validation/managed
image and configuration/reset/reconcile/self-test stages in
`src/LSM6DS3TR.cpp`; `test/test_basic.cpp`, compile contracts, profile CLI
tests and `FakeBus.h`. Trace all current acquisition/provenance fields added
by 01. Audit ADC configuration versus independently changing counters.

Read maintained topics 05/06/07/09/10/12/13 and the actual AN5130 sections
6.1, 6.5, 8.3, 8.5, 8.8 and 8.9 (printed pp. 50-57, 87-90, 94-95), plus
datasheet FIFO_CTRL2/4, CTRL10_C, WAKE_UP_DUR, timestamp and step registers.
Specifically inspect PDF Tables 73 and 74: fourth datasets occupy six bytes,
but timestamp byte order is not a plain contiguous little-endian 24-bit value;
temperature uses only its defined pair, with unused bytes. Do not infer layout
from corrupted extracted-table line breaks. Record independently derived byte
maps and test vectors before writing decode code.

AN5130 temperature mode requires TIMER_PEDO_FIFO_EN clear and FIFO_TEMP_EN
set. Timestamp/step uses TIMER_PEDO_FIFO_EN and the fourth decimator. The
step-trigger selector changes the trigger for **all enabled FIFO datasets**,
not only the fourth record. Timestamp and temperature cannot both be inferred
from the same payload. `FIFO_PATTERN` is position, never a type tag.

## Implementation direction

1. Replace incompatible booleans with one typed fourth-dataset selector and
   typed trigger/resolution/decimation fields in the existing replayable profile.
   Admit disabled, temperature, and timestamp-plus-step. Reject mutually
   incompatible routes, absent sources and malformed decimators before I/O.
   Temperature uses the internal motion trigger. Support the documented
   internal and step-event triggers for timestamp/step with explicit semantics
   for any accompanying motion records; no implied regular ODR in event mode.
2. Own the necessary timestamp/pedometer controls through the same configure
   engine and builders. Use an explicit source-backed fixed pedometer operating
   policy unless configurable banked parameters have a present caller. Do not
   add arbitrary embedded-bank access APIs. If banked settings are needed for
   the selected policy, implement bounded bank selection/readback/restoration
   and mark bank/configuration unknown on partial failure. Preserve main-bank
   fences for all unrelated public operations.
3. Establish the timestamp resolution before enabling the counter and follow
   documented reset requirements on resolution changes. A reset command is an
   effectful staged operation, not a persistent equality-tested register value.
   Pedometer reset is likewise explicit; PEDO_RST_STEP must be cleared by
   software. Never reset counters implicitly on a read, ordinary reconcile or
   every configure just to simplify tests. State which unavoidable hardware
   lifecycle changes start a new epoch.
4. Preserve the existing timestamp sensor-dependency ambiguity. For this joint
   dataset use the source-backed pedometer requirement of an active accelerometer
   at least 26 Hz, satisfying the conservative common dependency. Do not claim
   gyro-only timestamp support. Keep the dependency and future physical
   qualification caveat in the ambiguity ledger.
5. Extend the single bounded schema/decoder from 01 with complete six-byte
   records and explicit kinds. Preserve raw timestamp ticks, 24-bit width,
   configured resolution, 16-bit step count, profile generation and counter
   epoch. Keep host admission/read/terminal time distinct. If providing expanded
   timestamps or step totals, state the observation bound needed to disambiguate
   wraps and return explicit discontinuity/unknown evidence after loss/reset;
   never guess the number of missed wraps. A simple raw counter plus epoch is
   preferable to a hidden clock-synchronization service.
6. Temperature batching does not prove a new temperature conversion on every
   FIFO trigger. Apply the source cadence and readiness limits without copying
   snapshot TDA freshness semantics onto every stored record. Distinguish
   buffered, decoded and temporally qualified data. Timestamp/step values
   describe their captured counter state; they do not establish simultaneous
   accel/gyro sampling or map automatically onto the owner's uptime clock.
7. Make source changes, reset, reconfigure, cancellation, raw writes, FIFO purge
   and failed/ambiguous consumption invalidate the affected decoder/counter
   continuity. Preserve meaningful confirmed-consumption evidence on failed
   jobs. Reuse existing terminal-result ownership and caller-buffer rules.
   Extend both CLIs and profile equality atomically, with source choices visible
   to users and no diagnostics that silently reset counters.

## Required regression evidence

- PDF-derived byte fixtures for Tables 73/74 with deliberately different bytes
  in each timestamp position, signed temperature endpoints and nonzero unused
  bytes. Prove exactly the defined bytes are interpreted. Cover each starting
  pattern position and fourth-dataset decimation mixed with motion.
- Reject temperature+step/timestamp conflicts, disabled fourth decimator,
  absent/too-slow accelerometer for pedometer, invalid timer resolution, external
  trigger and sensor-hub settings. Rejections are atomic and bus-silent.
- Validate the complete register image and sequence: timer resolution before
  enable/reset, source/trigger selection, pedometer enable and reset/clear,
  stable readback versus volatile command bits. Reconcile must not reset a
  running counter or consume FIFO/source evidence.
- Timestamp `0xFFFFFF -> 0`, step `0xFFFF -> 0`, explicitly reset epochs,
  resolution changes, unknown/multiple-wrap intervals, FIFO overrun and
  interrupted consumption. Never convert an ambiguous interval into a fresh
  monotonic timestamp. Test repeated temperature values without false freshness.
- Internal and step-event triggers, with/without motion datasets, zero/one/
  large poll budgets, output exhaustion, capacity boundaries and partial records.
  Extend the independent fake with explicit counter and event inputs, not a
  clone of the production byte decoder.
- Fail/cancel/deadline at every new transfer/phase, including possible reset
  effect and bank-restoration failures if applicable. Assert terminal result,
  hardware-change/continuity evidence, buffer publication and callback ceilings.
  Preserve 01 and all pre-existing tests; register new suites in the runner.

## Completion

Run the execution index's commands and exact-final-commit CI. Update the support
matrix to name the actual fourth datasets, trigger modes and counter limits;
leave other embedded functionality unsupported. Update source ambiguities,
CLI/API guidance, changelog, audit R2 and prompt status with resolution SHA.
Keep physical cadence/trigger/counter-wrap evidence explicitly NOT RUN unless
captured on authorized hardware. Software completion does not close those
physical claims. Commit/push owned work and verify upstream synchronization;
continue to prompt 03.
