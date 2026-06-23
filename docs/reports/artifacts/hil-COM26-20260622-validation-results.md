# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-22T20:26:14.671367+02:00
- Ended: 2026-06-22T20:26:37.260671+02:00
- Suite: validation
- Port: COM26
- Baud: 115200
- Result counts: PASS=120, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=120, min=0.093s, mean=0.121s, max=1.312s
- Transcript: `docs/reports/artifacts/hil-COM26-20260622-validation-transcript.txt`

## Boot Excerpt

```text
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x9 (SPI_FAST_FLASH_BOOT)
Saved PC:0x40379d02
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce2820,len:0x118c
load:0x403c8700,len:0x4
load:0x403c8704,len:0xc20
load:0x403cb700,len:0x30e0
entry 0x403c88b8
[I] === LSM6DS3TR-C Bringup CLI ===
=== Version ===
  Version: 1.1.0
  Full:    1.1.0 (f10e8e4, 2026-06-22 20:17:40, clean)
  Built:   2026-06-22 20:17:40
  Commit:  f10e8e4
  Status:  clean
[I] I2C initialized (SDA=8, SCL=9)
[I] Scanning I2C bus (timeout=50ms)...
[I]      0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- 6A -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         
[I] Scan complete. Found 2 device(s).
[I] Common addresses: 0x3C/0x3D=OLED, 0x6A/0x6B=LSM6DS3TR, 0x76/0x77=BME280
[I] Device initialized successfully
=== Driver Health ===
  State: READY
  Online: yes
  Consecutive failures: 0
  Total success: 0
  Total failures: 0
  Success rate: 0.0%
  Last OK: never
  Last error: never

Type 'help' for commands
>
```

## Results

| Test ID | Feature | Command | Expected | Observed | Elapsed | Result | Notes |
|---|---|---|---|---|---:|---|---|
| BAS-001 | connectivity | `help` | CLI Help | === LSM6DS3TR-C CLI Help === \| Version: 1.1.0 (f10e8e4, 2026-06-22 20:17:40, clean) \| [Common] \| help / ?                         - Show this help \| ... | 0.109s | PASS |  |
| BAS-002 | connectivity | `version` | Version:, Commit: | === Version === \| Version: 1.1.0 \| Full:    1.1.0 (f10e8e4, 2026-06-22 20:17:40, clean) \| Built:   2026-06-22 20:17:40 \| ... | 0.109s | PASS |  |
| BAS-003 | connectivity | `scan` | Scan complete, 6A | [I] Scanning I2C bus (timeout=50ms)... \| [I]      0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F \| 00:                         -- -- -- -- -- -- -- -- \| 10: -- -- -- -- -- -- --... | 0.110s | PASS |  |
| BAS-004 | identity | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| BAS-005 | state | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| BAS-006 | state | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| BAS-007 | state | `drv1` | READY | Health: state=READY online=true consec=0 ok=0 fail=0 rate=0.0% | 0.110s | PASS |  |
| BAS-008 | identity | `whoami` | WHO_AM_I = 0x6A, match= | WHO_AM_I = 0x6A expected=0x6A match=YES | 0.109s | PASS |  |
| BAS-009 | identity | `id` | WHO_AM_I = 0x6A | WHO_AM_I = 0x6A expected=0x6A match=YES | 0.109s | PASS |  |
| DAT-001 | data | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| DAT-002 | data | `raw` | Raw: | Raw: ax=94 ay=-296 az=16697 \| gx=19 gy=-183 gz=-119 \| t=1145 | 0.110s | PASS |  |
| DAT-003 | data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.018 g \| gx=+0.14 gy=-1.59 gz=-1.02 dps \| t=29.44 C | 0.109s | PASS |  |
| DAT-004 | data | `accel` | Accel | Accel: x=0.006 y=-0.018 z=1.018 g (raw: 95 -303 16690) | 0.109s | PASS |  |
| DAT-005 | data | `gyro` | Gyro | Gyro: x=0.11 y=-1.56 z=-1.07 dps (raw: 13 -178 -122) | 0.109s | PASS |  |
| DAT-006 | data | `temp` | Temp | Temp: 29.47 C (raw: 1144) | 0.110s | PASS |  |
| CFG-001 | configuration | `odrxl` | odrxl: | odrxl: 104 Hz | 0.109s | PASS |  |
| CFG-002 | configuration | `odrg` | odrg: | odrg: 104 Hz | 0.109s | PASS |  |
| CFG-003 | configuration | `fsxl` | Accel FS: | Accel FS: +/-2g | 0.109s | PASS |  |
| CFG-004 | configuration | `fsg` | Gyro FS: | Gyro FS: +/-250 dps | 0.110s | PASS |  |
| CFG-005 | configuration | `apm` | apm: | apm: high-performance | 0.109s | PASS |  |
| CFG-006 | configuration | `gpm` | gpm: | gpm: high-performance | 0.109s | PASS |  |
| CFG-007 | configuration | `gsleep` | Gyro sleep: | Gyro sleep: no | 0.109s | PASS |  |
| CFG-008 | configuration | `fsxl 2` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| CFG-009 | configuration | `fsxl 4` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CFG-010 | configuration | `fsxl 8` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CFG-011 | configuration | `fsxl 16` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CFG-012 | configuration | `fsxl 2` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| CFG-013 | configuration | `fsg 125` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CFG-014 | configuration | `fsg 250` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CFG-015 | configuration | `fsg 500` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| CFG-016 | configuration | `fsg 1000` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| CFG-017 | configuration | `fsg 2000` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.093s | PASS |  |
| CFG-018 | configuration | `fsg 250` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| CFG-019 | configuration | `odrxl 12.5` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| CFG-020 | configuration | `odrxl 104` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CFG-021 | configuration | `odrxl 6660` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CFG-022 | configuration | `odrxl 1.6` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| CFG-023 | configuration | `odrxl 104` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| CFG-024 | configuration | `apm hp` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CFG-025 | configuration | `odrg 12.5` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CFG-026 | configuration | `odrg 104` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| CFG-027 | configuration | `odrg 6660` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| CFG-028 | configuration | `odrg 104` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FLT-001 | filters | `alpf2 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FLT-002 | filters | `alpf2 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FLT-003 | filters | `aslope 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FLT-004 | filters | `aslope 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FLT-005 | filters | `a6d 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FLT-006 | filters | `a6d 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FLT-007 | filters | `glpf1 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FLT-008 | filters | `glpf1 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FLT-009 | filters | `ghpf 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FLT-010 | filters | `ghpfmode 3` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FLT-011 | filters | `ghpfmode 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FLT-012 | filters | `ghpf 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FNC-001 | embedded functions | `ts 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FNC-002 | embedded functions | `tsread` | Timestamp | Timestamp: 12810 | 0.110s | PASS |  |
| FNC-003 | embedded functions | `tshr 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FNC-004 | embedded functions | `tsreset` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FNC-005 | embedded functions | `tshr 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FNC-006 | embedded functions | `pedo 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FNC-007 | embedded functions | `steps` | Step | Pedometer: yes \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| FNC-008 | embedded functions | `stepreset` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FNC-009 | embedded functions | `pedo 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FNC-010 | embedded functions | `sigmot 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FNC-011 | embedded functions | `sigmot 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FNC-012 | embedded functions | `tilt 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FNC-013 | embedded functions | `tilt 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| FNC-014 | embedded functions | `wtilt 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| FNC-015 | embedded functions | `wtilt 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| CAL-001 | calibration | `ofswt 16` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CAL-002 | calibration | `offset -4 7 12` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| CAL-003 | calibration | `offset` | Offset: | Offset: x=-4 y=7 z=12 | 0.110s | PASS |  |
| CAL-004 | calibration | `offset 0 0 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| CAL-005 | calibration | `ofswt 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| CAL-006 | calibration | `biasxl` | Accel bias: | Accel bias: x=0.000000 y=0.000000 z=0.000000 g | 0.094s | PASS |  |
| CAL-007 | calibration | `biasg` | Gyro bias: | Gyro bias: x=0.000000 y=0.000000 z=0.000000 dps | 0.094s | PASS |  |
| CAL-008 | calibration | `biasxl 0.001 0.002 -0.003` | Accel bias set: | Accel bias set: x=0.001000 y=0.002000 z=-0.003000 g | 0.094s | PASS |  |
| CAL-009 | calibration | `biasg 0.1 -0.2 0.3` | Gyro bias set: | Gyro bias set: x=0.100000 y=-0.200000 z=0.300000 dps | 0.093s | PASS |  |
| CAL-010 | calibration | `biasreset` | biases cleared | All software biases cleared. | 0.094s | PASS |  |
| FIFO-001 | fifo | `fifo_mode bypass` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FIFO-002 | fifo | `fifo_odr 104` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FIFO-003 | fifo | `fifo_xl 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FIFO-004 | fifo | `fifo_g 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FIFO-005 | fifo | `fifo_th 8` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FIFO-006 | fifo | `fifo_temp 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FIFO-007 | fifo | `fifo_step 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FIFO-008 | fifo | `fifo_stop 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| FIFO-009 | fifo | `fifo_high 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FIFO-010 | fifo | `fifo_mode cont` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| FIFO-011 | fifo | `fifo` | FIFO | === FIFO === \| Mode: continuous \| ODR:  104 Hz \| Threshold: 8 \| ... | 0.109s | PASS |  |
| FIFO-012 | fifo | `fifo_read 8` | FIFO; allowed failures: FIFO_EMPTY | FIFO[0] = 0x0012 \| FIFO[1] = 0xFF47 \| FIFO[2] = 0xFF88 \| FIFO[3] = 0x0056 \| ... | 0.110s | PASS |  |
| FIFO-013 | fifo | `fifo_mode bypass` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| SRC-001 | diagnostics | `wusrc` | 0x | wusrc = 0x00 | 0.109s | PASS |  |
| SRC-002 | diagnostics | `tapsrc` | 0x | tapsrc = 0x00 | 0.109s | PASS |  |
| SRC-003 | diagnostics | `6dsrc` | 0x | 6dsrc = 0x00 | 0.110s | PASS |  |
| SRC-004 | diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SRC-005 | diagnostics | `funcsrc2` | 0x | funcsrc2 = 0x84 \| slave3 nack          yes \| slave2 nack          no \| slave1 nack          no \| ... | 0.110s | PASS |  |
| SRC-006 | diagnostics | `wtstatus` | 0x | wtstatus = 0x00 \| x positive           no \| x negative           no \| y positive           no \| ... | 0.109s | PASS |  |
| SRC-007 | diagnostics | `shub 12` | 2E: | 2E: 00 00 00 00 00 00 00 00 00 00 00 00              \|............\| | 0.109s | PASS |  |
| REG-001 | register access | `rreg 0x0F` | [0x0F] = 0x6A | [0x0F] = 0x6A | 0.110s | PASS |  |
| REG-002 | register access | `dump 0x10 32` | 10: | 10: 40 40 44 00 00 00 00 00 E0 20 00 00 00 00 07 C2  \|@@D...... ......\| \| 20: 7C 04 10 00 48 FF 87 FF 62 00 C1 FE 3D 41 00 00  \|\|...H...b...=A..\| | 0.110s | PASS |  |
| REG-003 | register access | `wreg 0x73 0x00` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| RST-001 | reset/recovery | `reset` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| RST-002 | reset/recovery | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| RST-003 | reset/recovery | `boot` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| RST-004 | reset/recovery | `refresh` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| RST-005 | reset/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 209 -> 226 (+17) \| ... | 0.109s | PASS |  |
| INV-001 | invalid input | `bogus_command` | Unknown command | Unknown command: 'bogus_command' (type 'help') | 0.110s | PASS |  |
| INV-002 | invalid input | `odrxl bad` | Invalid ODR token | Invalid ODR token | 0.110s | PASS |  |
| INV-003 | invalid input | `odrg 1.6` | INVALID_PARAM; allowed failures: INVALID_PARAM | Status: INVALID_PARAM (code=5, detail=0) \| Message: Invalid gyro ODR/power-mode combination | 0.109s | PASS |  |
| INV-004 | invalid input | `fsxl 32` | Expected fsxl | Expected fsxl [2\|4\|8\|16] | 0.109s | PASS |  |
| INV-005 | invalid input | `rreg 0x100` | Invalid register | Invalid register | 0.094s | PASS |  |
| INV-006 | invalid input | `dump 0x10 0` | Invalid dump range | Invalid dump range | 0.094s | PASS |  |
| INV-007 | invalid input | `fifo_th 2048` | Expected fifo_th | Expected fifo_th [0..2047] | 0.110s | PASS |  |
| STRESS-001 | stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.546s | PASS |  |
| STRESS-002 | stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SELF-001 | self-test | `selftest` | Selftest result: | === LSM6DS3TR selftest (safe commands) === \| [PASS] probe responds \| [PASS] probe no-health-side-effects \| [PASS] readWhoAmI \| ... | 1.312s | PASS |  |
| POST-001 | post-check | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| POST-002 | post-check | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
