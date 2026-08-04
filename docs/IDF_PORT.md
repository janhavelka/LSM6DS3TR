# LSM6DS3TR ESP-IDF Port

The core is a framework-neutral ESP-IDF component. `idf_component.yml`
supports the ESP-IDF 5.4 line, and CI compiles the native example with exactly
ESP-IDF 5.4.4 for ESP32-S2 and ESP32-S3.

The component target compiles `src/LSM6DS3TR.cpp` as C++17 and exports only
the public `include` directory. Package version, public version constants,
component version, and Doxygen project version are synchronized from
`library.json`.

The application owns all ESP-IDF integration:

- entry point: `app_main()`;
- I2C bus/device handles: `i2c_new_master_bus()` and
  `i2c_master_bus_add_device()` from `driver/i2c_master.h`;
- synchronous physical attempts: `i2c_master_transmit()` and
  `i2c_master_transmit_receive()`;
- monotonic operation time: `esp_timer_get_time()` converted to 64-bit ms;
- task scheduling, yields, and logging: FreeRTOS and native ESP-IDF facilities;
- non-blocking console input so CLI traffic cannot stall operation polling;
- pins, bus rate, per-transfer timeout, locking, retries, and recovery;
- fixed CLI buffers and result publication.

The component receives only synchronous, timeout-bounded transport callbacks.
Each callback performs exactly one physical attempt: it neither retries nor
recovers the bus, and it maps `esp_err_t` into the library's framework-neutral
`Status`. The owner may serialize access inside the callback, but the lock wait
must remain within the supplied transfer timeout.

`bind()` does no I2C. The owner starts a tokened operation, calls
`poll(nowMs, budget)`, cancels through the bus-silent cancellation API when
needed, and takes the matching terminal result exactly once. Each operation
also has a hard total callback ceiling reported in its terminal result.

The native example must not include `Arduino.h`, `Wire.h`, `String`, `Serial`,
`TwoWire`, Arduino compatibility facades, or Arduino CLI sources.
The Arduino and native ESP-IDF examples expose the same fixed-memory operator
grammar; repository guards enforce parity without compiling Arduino sources
into the IDF component. Both process at most one completed console command per
owner turn, sample the operation time after input handling, and grant the
driver one callback per `poll()`. This keeps direct diagnostics, cooperative
sessions, and driver jobs from silently sharing a turn's physical-transfer
budget.

## Native CLI Ownership Details

The expanded example is an application owner, not an alternate driver layer:

- `status`/`diag` report bus readiness, selected and bound address, frequency,
  timeout, configuration state/generation/settling, transport totals, complete
  last-error status and time/age, mismatch evidence, active/pending state,
  current poll transactions/wait status, and staged/desired/verified profiles.
  `job [current|last]` adds cooperative-session progress and optional cached
  terminal evidence; `result` is its last-result convenience form. These paths
  perform no I2C.
- `scan` calls `i2c_master_probe()` for `0x6A` and `0x6B`, one address per owner
  turn. An ACK proves presence only; the tokened driver `probe` still checks
  WHO_AM_I before identity-dependent use.
- `addr [0x6a|0x6b]` registers a candidate owner device handle before retiring
  the working handle. Allocation failure therefore preserves the old handle;
  removal failure attempts to discard the candidate, reports cleanup evidence,
  and retains at most one failed-cleanup handle for a bounded retry. The
  owner then rebinds the library with zero sensor I2C. A successful address
  change deliberately clears
  configuration/transport/result provenance and requires a fresh
  probe/configure.
- `freq [100000|400000]` changes `i2c_device_config_t::scl_speed_hz` through the
  same bounded handle-replacement/rollback path. Because this changes only the
  owner transport, successful frequency replacement preserves verified sensor
  configuration.
- `profile set` uses the shared framework-neutral `ProfileCli.h` parser. It
  copies the staged `DeviceProfile`, applies one typed field, validates the
  complete candidate, and commits only on success. All profile fields have an
  explicit CLI representation; unsupported production values such as BDU off,
  FIFO/interrupt enable, gyro HPF enable, slope/high-pass output, and 6D
  filtering are accepted as grammar probes but rejected atomically by the
  production validator. `profile apply` is the ordinary tokened configure job.
- `stress [count] [quantity] [mode]` and `stress_mix [count]` are fixed-memory
  application coordinators over ordinary driver jobs. They keep only counters,
  first/last failure, pending request identity, sequence/generation evidence,
  and transport baselines; they do not add a request queue or retry policy.
  Every terminal result is checked for success, a nonzero transaction count,
  transaction bounds, a
  non-destructive hardware-effect claim, and job-specific provenance before it
  counts as successful.

The complete command/value grammar, including self-test and calibration
arguments, is maintained in the main
[bring-up CLI reference](../README.md#bring-up-cli) and the
[native example README](../examples/idf/basic/README.md#cli-surface).

Run the repository guards after changing the example:

```sh
python tools/check_idf_example_contract.py
python tools/check_cli_contract.py
python tools/check_core_timing_guard.py
python tools/build_docs.py
```

Local compilation requires an ESP-IDF 5.4 installation:

```sh
cd examples/idf/basic
idf.py set-target esp32s3
idf.py build
```

Use `esp32s2` for the other supported target. A static contract check is not a
compiler result; CI's two target jobs provide the actual IDF compile evidence.
