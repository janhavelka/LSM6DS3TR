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
- `probe`, `configure`, `sample [all|accel|gyro|temp] [ready|direct|unqualified]`, `reset`,
  `boot`, `recover`, `reconcile`, and `powerdown` start bounded tokened jobs;
- `profile [show|validate|defaults|apply]`, `cfg`, and `settings` expose staged,
  desired, and verified profiles. `profile set` atomically covers every
  `DeviceProfile` field, including ranges, ODR/power modes, supported filters,
  gyro sleep, typed LPF/HPF/reference selections, 6D source selection and
  hardware offsets. BDU remains enabled and incompatible filter/dependency
  edits are rejected atomically;
- `fifo <1..2048 words> [1..32 records]` starts typed motion acquisition into
  static caller storage retained through terminal consumption. It prints complete
  vectors only after matching success, plus immutable scale/generation/read-time
  and loss evidence. `purge <1..2048> [stop]` is explicit destructive maintenance;
  `stop` drains bounded content, selects Bypass, and reports possible residual
  loss before later configure/recovery. `profile set fifo continuous 104 1 1 24`
  atomically stages the FIFO mode, ODR, gyro/accel decimation and watermark.
  FIFO interpretation and protocol remain in the core; no raw FIFO recipes are
  present in this native application;
- `stress [count] [quantity] [mode]` runs an exact cooperative sample campaign
  (default count 100, quantity `all`, mode `ready`), while
  `stress_mix [count]` rotates probe, reconcile, ready, and direct jobs. Both
  retain fixed memory, one callback per poll, bounded progress, cancellation,
  and terminal/nonzero-transaction/provenance summaries;
- `selftest [5..100]`,
  `calxl [samples [x y z [max_p2p_g]]]`,
  `calg [samples [max_p2p_dps]]`, `purge <1..2048> [stop]`, `rreg`, `wreg`, and
  `dump` remain explicit stationary-fixture, destructive, or diagnostic
  maintenance commands.

Profile changes are staged only. `profile apply` starts the normal configure
job, including its own WHO_AM_I proof and complete managed-register readback.
Each individual setter validates the complete candidate and leaves the prior
draft unchanged on parse, arity, cross-field, or unsupported-profile errors.
The exact value tokens and coupled-field ordering are listed in the main
[CLI profile table](../../../README.md#bring-up-cli).

## Filter Diagnostics

`profile set xl_filter` changes coupled host output selectors atomically:
`lpf1 2`/`lpf1 4`, `lpf2 50`/`100`/`9`/`400`, `slope 4`,
`highpass 100`/`9`/`400`, or `reference 100`. The reference token is a
canonical register selection, not a high-pass cutoff. Configure consumes
its first XYZ sample with bounded readiness checks before completing.
`xl_input latency|noise`, `xl_hpcf 50|100|9|400`, and
`xl_analog auto|400` select the remaining typed choices under complete-profile
validation. `xl_6d_lpf on` requires a routed orientation profile.

Gyro LPF1 uses `g_lpf1 on` and `g_ftype 0|1|2|3`.
Gyro HPF uses `g_hpf on` followed by `g_hpf_mode 0.016|0.065|0.260|1.040`.
It requires an active awake high-performance gyro and excludes gyro FIFO.
After `profile apply` completes, `sample gyro unqualified` explicitly opts
into a direct read with `filter_unverified` quality and zero freshness;
ordinary `ready` and `direct` requests for that channel reject before I2C.
`filter_unqualified` means verified register settings without a vendor-backed
gyro HPF settling guarantee. It never becomes qualified just by waiting.

Sample, FIFO and event output prints captured filter provenance and per-channel
qualification separately from freshness. Later profile edits do not reinterpret
older results. Calibration rejects DC-removing paths for its selected sensor;
self-test uses the unfiltered vendor recipe and restores the actual verified
filter/interrupt/FIFO profile with separate primary/restoration status.

Both frameworks derive replay and restoration deadlines from the chosen profile:
1.6 Hz LPF2 ODR/400 alone needs 200 seconds, plus quantization, any reference
discard and bus/scheduling allowance. Callback ceilings are multiplied by the
configured transport timeout; sampling reserves all readiness checks and
cadence waits, and calibration reserves all sample waits/callbacks.
Profile output prints that interval and
the configure budget. Hardware filter transients and 6D response for the new
paths remain **NOT RUN**; an HPF cutoff is not evidence of qualified settling.
