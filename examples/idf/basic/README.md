# Native ESP-IDF Owner-Safe Example

This project demonstrates the version 2 external-owner API with native
ESP-IDF 5.4:

- `app_main()` and fixed C buffers;
- `driver/i2c_master.h` transport owned by the application;
- 64-bit monotonic time from `esp_timer_get_time()`;
- non-blocking console input so owner polling is never held behind `getchar()`;
- tokened start/poll/cancel/take lifecycle;
- one transport callback per owner poll;
- one completed console command per owner turn, with monotonic time sampled
  after input handling;
- no Arduino compatibility facade.

Build locally with ESP-IDF 5.4:

```sh
idf.py set-target esp32s3
idf.py build
```

CI builds both `esp32s2` and `esp32s3` using exactly ESP-IDF 5.4.4.
The example pins GPIO 8/9 and address `0x6A` only as application-owned fixture
defaults; the library owns none of those board choices.

Each transport callback makes one timeout-bounded physical attempt and maps
the native result to the library `Status`; retry and bus-recovery policy remain
with the application owner. See the [ESP-IDF port guide](../../../docs/IDF_PORT.md)
for the complete component and ownership contract.

## CLI Surface

The native CLI intentionally matches the
[Arduino bring-up CLI](../../../README.md#bring-up-cli) while using only native
ESP-IDF facilities:

- `help`/`?`, `version`/`ver`, `status`/`diag`, `job [current|last]`, and
  `result` expose grammar, runtime metadata, bus/binding/configuration state,
  transport counters and complete last-error timing, register mismatch
  evidence, cooperative-session progress, driver poll budgets, and the cached
  last terminal result without performing I2C;
- `bind`, `unbind`, and `cancel` demonstrate the zero-I2C binding lifecycle,
  bus-silent cancellation, and the explicitly destructive hard-teardown path;
- `scan` performs one address-only ACK check per owner turn at `0x6A` and
  `0x6B`; `addr [0x6a|0x6b]` adds a candidate application-owned device handle
  before retiring the working handle, tracks at most one failed-cleanup handle
  for a bounded retry, reports cleanup evidence, resets
  driver provenance on success, and requires a fresh
  probe/configure; `freq [100000|400000]` replaces the handle at a chip-valid
  bus rate while preserving verified sensor configuration on success;
- `probe`, `configure`, `sample [all|accel|gyro|temp] [ready|direct]`, `reset`,
  `boot`, `recover`, `reconcile`, and `powerdown` start bounded tokened jobs;
- `profile [show|validate|defaults|apply]`, `cfg`, and `settings` expose staged,
  desired, and verified profiles. `profile set` atomically covers every
  `DeviceProfile` field, including ranges, ODR/power modes, supported filters,
  gyro sleep, HPF mode bits, hardware offsets, and the explicitly rejected
  production invariants (BDU off, FIFO/interrupt enable, gyro HPF enable,
  slope/high-pass output, and 6D filtering);
- `stress [count] [quantity] [mode]` runs an exact cooperative sample campaign
  (default count 100, quantity `all`, mode `ready`), while
  `stress_mix [count]` rotates probe, reconcile, ready, and direct jobs. Both
  retain fixed memory, one callback per poll, bounded progress, cancellation,
  and terminal/nonzero-transaction/provenance summaries;
- `selftest [5..100]`,
  `calxl [samples [x y z [max_p2p_g]]]`,
  `calg [samples [max_p2p_dps]]`, `purge <1..2048>`, `rreg`, `wreg`, and
  `dump` remain explicit stationary-fixture, destructive, or diagnostic
  maintenance commands.

Profile changes are staged only. `profile apply` starts the normal configure
job, including its own WHO_AM_I proof and complete managed-register readback.
Each individual setter validates the complete candidate and leaves the prior
draft unchanged on parse, arity, cross-field, or unsupported-profile errors.
The exact value tokens and coupled-field ordering are listed in the main
[CLI profile table](../../../README.md#bring-up-cli).
