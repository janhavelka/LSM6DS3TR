# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-23T05:24:32.480150+02:00
- Ended: 2026-06-23T05:24:58.809523+02:00
- Suite: targeted
- Port: COM26
- Baud: 115200
- Result counts: PASS=136, FAIL=8, UNKNOWN=0, NOT_RUN=0
- Timing: count=144, min=0.093s, mean=0.117s, max=1.313s
- Transcript: `docs/reports/artifacts/hil-COM26-20260623-targeted-transcript.txt`

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
  Full:    1.1.0 (f10e8e4, 2026-06-23 05:23:29, dirty)
  Built:   2026-06-23 05:23:29
  Commit:  f10e8e4
  Status:  dirty
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
| TGT-BOOT-001 | baseline | `version` | Version:, Commit: | === Version === \| Version: 1.1.0 \| Full:    1.1.0 (f10e8e4, 2026-06-23 05:23:29, dirty) \| Built:   2026-06-23 05:23:29 \| ... | 0.110s | PASS |  |
| TGT-BOOT-002 | baseline | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| TGT-BOOT-003 | baseline | `whoami` | WHO_AM_I = 0x6A, match= | WHO_AM_I = 0x6A expected=0x6A match=YES | 0.109s | PASS |  |
| TGT-POLL-001 | poll/manual | `job status` | Job status: | Job status: busy=no ready=no hasSample=no manual=no \| Last poll: OK (code=0, detail=0) \| Last poll msg: OK \| Health: state=READY ok=1 fail=0 consecutive=0 | 0.109s | PASS |  |
| TGT-POLL-002 | poll/manual | `job auto 0` | Automatic tick polling: no, manual=yes | Automatic tick polling: no \| Job auto: busy=no ready=no hasSample=no manual=yes \| Last poll: OK (code=0, detail=0) \| Last poll msg: OK \| ... | 0.110s | PASS |  |
| TGT-POLL-003 | poll/sample | `job start sample` | Status:, IN_PROGRESS, Job start:; allowed failures: IN_PROGRESS | Status: IN_PROGRESS (code=10, detail=0) \| Message: Measurement scheduled \| Job start: busy=yes ready=no hasSample=no manual=yes \| Last poll: IN_PROGRESS (code=10, detail=0) \| ... | 0.109s | PASS |  |
| TGT-POLL-004 | poll/sample | `job get` | MEASUREMENT_NOT_READY; allowed failures: IN_PROGRESS, MEASUREMENT_NOT_READY | Status: MEASUREMENT_NOT_READY (code=8, detail=0) \| Message: No sample available \| Job get: busy=yes ready=no hasSample=no manual=yes \| Last poll: IN_PROGRESS (code=10, detail=0)... | 0.109s | PASS |  |
| TGT-POLL-005 | poll/budget-zero | `job poll 0 1` | budget=0 -> IN_PROGRESS, busy=yes; allowed failures: IN_PROGRESS | Poll 1/1 budget=0 -> IN_PROGRESS busy=yes ready=no \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job in progress \| Job poll: busy=yes ready=no hasSample=no manual=ye... | 0.109s | PASS |  |
| TGT-POLL-006 | poll/budget-one | `job poll 1 1` | budget=1 ->, busy=; allowed failures: IN_PROGRESS | Poll 1/1 budget=1 -> TIMEOUT busy=no ready=no \| Status: TIMEOUT (code=4, detail=0) \| Message: Measurement ready timeout \| Job poll: busy=no ready=no hasSample=no manual=no \| ... | 0.110s | FAIL | failures=['TIMEOUT'] |
| TGT-POLL-007 | poll/sample | `job poll 1 20 2` | Status: OK, Job poll:; allowed failures: IN_PROGRESS | Poll 1/20 budget=1 -> TIMEOUT busy=no ready=no \| Status: TIMEOUT (code=4, detail=0) \| Message: Measurement ready timeout \| Job poll: busy=no ready=no hasSample=no manual=no \| ... | 0.109s | FAIL | failures=['TIMEOUT']; missing=['Status: OK'] |
| TGT-POLL-008 | poll/sample | `job getraw` | Cached raw: | Status: MEASUREMENT_NOT_READY (code=8, detail=0) \| Message: No sample available \| Job getraw: busy=no ready=no hasSample=no manual=no \| Last poll: TIMEOUT (code=4, detail=0) \| ... | 0.109s | FAIL | failures=['MEASUREMENT_NOT_READY', 'TIMEOUT']; missing=['Cached raw:'] |
| TGT-POLL-009 | poll/sample | `job get` | Sample:, Status: OK | Status: MEASUREMENT_NOT_READY (code=8, detail=0) \| Message: No sample available \| Job get: busy=no ready=no hasSample=no manual=no \| Last poll: TIMEOUT (code=4, detail=0) \| ... | 0.094s | FAIL | failures=['MEASUREMENT_NOT_READY', 'TIMEOUT']; missing=['Sample:', 'Status: OK'] |
| TGT-POLL-010 | poll/direct | `job run direct 1 3` | Job run: kind=direct, Status: OK; allowed failures: IN_PROGRESS | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Measurement scheduled \| Job run: kind=direct budget=1 polls=1 limit=3 \| ... | 0.094s | PASS |  |
| TGT-POLL-011 | poll/busy | `job start sample` | Status:, IN_PROGRESS; allowed failures: IN_PROGRESS | Status: IN_PROGRESS (code=10, detail=0) \| Message: Measurement scheduled \| Job start: busy=yes ready=no hasSample=yes manual=yes \| Last poll: IN_PROGRESS (code=10, detail=0) \| ... | 0.109s | PASS |  |
| TGT-POLL-012 | poll/busy | `job start refresh` | BUSY; allowed failures: IN_PROGRESS, BUSY | Status: BUSY (code=9, detail=0) \| Message: Poll job in progress \| Job start: busy=yes ready=no hasSample=yes manual=yes \| Last poll: IN_PROGRESS (code=10, detail=0) \| ... | 0.109s | PASS |  |
| TGT-POLL-013 | poll/busy | `job poll 2 20 2` | Status: OK; allowed failures: IN_PROGRESS | Poll 1/20 budget=2 -> TIMEOUT busy=no ready=no \| Status: TIMEOUT (code=4, detail=0) \| Message: Measurement ready timeout \| Job poll: busy=no ready=no hasSample=yes manual=no \| ... | 0.109s | FAIL | failures=['TIMEOUT']; missing=['Status: OK'] |
| TGT-POLL-014 | poll/invalid | `job start fifo 0` | INVALID_PARAM; allowed failures: INVALID_PARAM | Status: INVALID_PARAM (code=5, detail=0) \| Message: maxWords must be > 0 \| Job start: busy=no ready=no hasSample=yes manual=no \| Last poll: TIMEOUT (code=4, detail=0) \| ... | 0.110s | FAIL | failures=['TIMEOUT'] |
| TGT-POLL-015 | poll/invalid | `job start calxl 0` | INVALID_PARAM; allowed failures: INVALID_PARAM | Status: INVALID_PARAM (code=5, detail=0) \| Message: samples must be 1..10000 \| Job start: busy=no ready=no hasSample=yes manual=no \| Last poll: TIMEOUT (code=4, detail=0) \| ... | 0.109s | FAIL | failures=['TIMEOUT'] |
| TGT-POLL-016 | poll/invalid | `job start bogus` | Invalid job kind | Invalid job kind | 0.109s | PASS |  |
| TGT-POLL-017 | poll/invalid | `job poll 999` | Invalid poll budget | Invalid poll budget | 0.109s | PASS |  |
| TGT-POLL-018 | poll/budget-zero | `job run sample 0 3 0` | Job run: kind=sample budget=0, IN_PROGRESS; allowed failures: IN_PROGRESS | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Measurement scheduled \| Job run: kind=sample budget=0 polls=3 limit=3 \| ... | 0.110s | PASS |  |
| TGT-POLL-019 | poll/recover-progress | `job poll 2 20 2` | Status: OK; allowed failures: IN_PROGRESS | Poll 1/20 budget=2 -> TIMEOUT busy=no ready=no \| Status: TIMEOUT (code=4, detail=0) \| Message: Measurement ready timeout \| Job poll: busy=no ready=no hasSample=yes manual=no \| ... | 0.109s | FAIL | failures=['TIMEOUT']; missing=['Status: OK'] |
| TGT-POLL-020 | poll/refresh | `job run refresh 1 30` | Job run: kind=refresh, Status: OK; allowed failures: IN_PROGRESS | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=refresh budget=1 polls=11 limit=30 \| ... | 0.109s | PASS |  |
| TGT-POLL-021 | poll/refresh | `job run refresh 255 3` | Job run: kind=refresh, Status: OK; allowed failures: IN_PROGRESS | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=refresh budget=255 polls=1 limit=3 \| ... | 0.109s | PASS |  |
| TGT-POLL-022 | poll/reset | `job run reset 1 60 2` | Job run: kind=reset, Status: OK; allowed failures: IN_PROGRESS | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=reset budget=1 polls=18 limit=60 \| ... | 0.110s | PASS |  |
| TGT-POLL-023 | poll/boot | `job run boot 1 60 2` | Job run: kind=boot, Status: OK; allowed failures: IN_PROGRESS | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=boot budget=1 polls=13 limit=60 \| ... | 0.109s | PASS |  |
| TGT-POLL-024 | poll/post | `job auto 1` | Automatic tick polling: yes | Automatic tick polling: yes \| Job auto: busy=no ready=no hasSample=no manual=no \| Last poll: OK (code=0, detail=0) \| Last poll msg: OK \| ... | 0.109s | PASS |  |
| TGT-FEAT-001 | data | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| TGT-FEAT-002 | data | `raw` | Raw: | Raw: ax=83 ay=-317 az=16689 \| gx=20 gy=-181 gz=-115 \| t=1587 | 0.110s | PASS |  |
| TGT-FEAT-003 | data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.017 g \| gx=+0.17 gy=-1.63 gz=-1.03 dps \| t=31.22 C | 0.093s | PASS |  |
| TGT-FEAT-004 | data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.019 g (raw: 86 -303 16698) | 0.094s | PASS |  |
| TGT-FEAT-005 | data | `gyro` | Gyro | Gyro: x=0.11 y=-1.61 z=-1.00 dps (raw: 13 -184 -114) | 0.110s | PASS |  |
| TGT-FEAT-006 | data | `temp` | Temp | Temp: 31.22 C (raw: 1592) | 0.109s | PASS |  |
| TGT-FS-001 | full-scale | `fsxl 2` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.093s | PASS |  |
| TGT-FS-002 | full-scale | `fsxl 4` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-FS-003 | full-scale | `fsxl 8` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-FS-004 | full-scale | `fsxl 16` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FS-005 | full-scale | `fsxl 2` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.093s | PASS |  |
| TGT-FS-006 | full-scale | `fsg 125` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-FS-007 | full-scale | `fsg 250` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-FS-008 | full-scale | `fsg 500` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FS-009 | full-scale | `fsg 1000` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FS-010 | full-scale | `fsg 2000` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FS-011 | full-scale | `fsg 250` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-ODR-001 | odr | `odrxl 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-ODR-002 | odr | `odrxl 1.6` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.093s | PASS |  |
| TGT-ODR-003 | odr | `odrxl 12.5` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-004 | odr | `odrxl 26` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-005 | odr | `odrxl 52` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-006 | odr | `odrxl 104` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.093s | PASS |  |
| TGT-ODR-007 | odr | `odrxl 208` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-008 | odr | `odrxl 416` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-009 | odr | `odrxl 833` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-010 | odr | `odrxl 1660` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.093s | PASS |  |
| TGT-ODR-011 | odr | `odrxl 3330` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-012 | odr | `odrxl 6660` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-013 | odr | `odrxl 104` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-014 | odr | `odrg 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.093s | PASS |  |
| TGT-ODR-015 | odr | `odrg 12.5` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-016 | odr | `odrg 26` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-017 | odr | `odrg 52` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-ODR-018 | odr | `odrg 104` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-ODR-019 | odr | `odrg 208` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-ODR-020 | odr | `odrg 416` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-ODR-021 | odr | `odrg 833` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-ODR-022 | odr | `odrg 1660` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-ODR-023 | odr | `odrg 3330` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-ODR-024 | odr | `odrg 6660` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-ODR-025 | odr | `odrg 104` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-PWR-001 | power | `apm lpn` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.093s | PASS |  |
| TGT-PWR-002 | power | `apm hp` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-PWR-003 | power | `gpm lpn` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-PWR-004 | power | `gpm hp` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-PWR-005 | power | `gsleep 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.093s | PASS |  |
| TGT-PWR-006 | power | `gsleep 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.094s | PASS |  |
| TGT-FLT-001 | filters | `alpf2 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FLT-002 | filters | `aslope 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FLT-003 | filters | `a6d 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FLT-004 | filters | `glpf1 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FLT-005 | filters | `ghpf 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FLT-006 | filters | `ghpfmode 3` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FLT-007 | filters | `alpf2 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FLT-008 | filters | `aslope 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FLT-009 | filters | `a6d 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FLT-010 | filters | `glpf1 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FLT-011 | filters | `ghpfmode 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FLT-012 | filters | `ghpf 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FNC-001 | embedded-functions | `ts 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FNC-002 | embedded-functions | `tsread` | Timestamp | Timestamp: 5626 | 0.109s | PASS |  |
| TGT-FNC-003 | embedded-functions | `tshr 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FNC-004 | embedded-functions | `tsreset` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FNC-005 | embedded-functions | `tshr 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FNC-006 | embedded-functions | `pedo 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FNC-007 | embedded-functions | `steps` | Step | Pedometer: yes \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| TGT-FNC-008 | embedded-functions | `stepreset` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FNC-009 | embedded-functions | `pedo 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FNC-010 | embedded-functions | `sigmot 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FNC-011 | embedded-functions | `sigmot 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FNC-012 | embedded-functions | `tilt 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FNC-013 | embedded-functions | `tilt 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FNC-014 | embedded-functions | `wtilt 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FNC-015 | embedded-functions | `wtilt 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-CAL-001 | calibration | `ofswt 16` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-CAL-002 | calibration | `offset -4 7 12` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-CAL-003 | calibration | `offset` | Offset: | Offset: x=-4 y=7 z=12 | 0.109s | PASS |  |
| TGT-CAL-004 | calibration | `offset 0 0 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-CAL-005 | calibration | `job run calxl 1 40 2 1` | Job run: kind=calxl, Status: OK; allowed failures: IN_PROGRESS | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=calxl budget=1 polls=2 limit=40 \| ... | 0.110s | PASS |  |
| TGT-CAL-006 | calibration | `job run calg 1 40 2 1` | Job run: kind=calg, Status: OK; allowed failures: IN_PROGRESS | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=calg budget=1 polls=2 limit=40 \| ... | 0.110s | PASS |  |
| TGT-CAL-007 | calibration | `biasxl` | Accel bias: | Accel bias: x=0.005124 y=-0.018971 z=0.018090 g | 0.109s | PASS |  |
| TGT-CAL-008 | calibration | `biasg` | Gyro bias: | Gyro bias: x=0.166250 y=-1.601250 z=-1.041250 dps | 0.109s | PASS |  |
| TGT-CAL-009 | calibration | `biasreset` | biases cleared | All software biases cleared. | 0.110s | PASS |  |
| TGT-FIFO-001 | fifo | `fifo_mode bypass` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FIFO-002 | fifo | `fifo_odr 104` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FIFO-003 | fifo | `fifo_xl 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FIFO-004 | fifo | `fifo_g 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FIFO-005 | fifo | `fifo_th 8` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FIFO-006 | fifo | `fifo_temp 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FIFO-007 | fifo | `fifo_step 1` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FIFO-008 | fifo | `fifo_stop 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FIFO-009 | fifo | `fifo_high 0` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-FIFO-010 | fifo | `fifo_mode cont` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-FIFO-011 | fifo | `fifo` | FIFO | === FIFO === \| Mode: continuous \| ODR:  104 Hz \| Threshold: 8 \| ... | 0.109s | PASS |  |
| TGT-FIFO-012 | fifo | `job run fifo 1 20 2 8` | Job run: kind=fifo, Status: OK; allowed failures: IN_PROGRESS | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=fifo budget=1 polls=9 limit=20 \| ... | 0.110s | PASS |  |
| TGT-FIFO-013 | fifo | `fifo_read 8` | FIFO; allowed failures: FIFO_EMPTY | FIFO[0] = 0xFF8D \| FIFO[1] = 0x0048 \| FIFO[2] = 0xFEC9 \| FIFO[3] = 0x4127 \| ... | 0.110s | PASS |  |
| TGT-FIFO-014 | fifo | `fifo_mode bypass` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-SRC-001 | sources | `wusrc` | 0x | wusrc = 0x00 | 0.109s | PASS |  |
| TGT-SRC-002 | sources | `tapsrc` | 0x | tapsrc = 0x00 | 0.110s | PASS |  |
| TGT-SRC-003 | sources | `6dsrc` | 0x | 6dsrc = 0x00 | 0.110s | PASS |  |
| TGT-SRC-004 | sources | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| TGT-SRC-005 | sources | `funcsrc2` | 0x | funcsrc2 = 0x84 \| slave3 nack          yes \| slave2 nack          no \| slave1 nack          no \| ... | 0.109s | PASS |  |
| TGT-SRC-006 | sources | `wtstatus` | 0x | wtstatus = 0x00 \| x positive           no \| x negative           no \| y positive           no \| ... | 0.110s | PASS |  |
| TGT-SRC-007 | sources | `shub 12` | 2E: | 2E: 00 00 00 00 00 00 00 00 00 00 00 00              \|............\| | 0.110s | PASS |  |
| TGT-REG-001 | registers | `rreg 0x0F` | [0x0F] = 0x6A | [0x0F] = 0x6A | 0.109s | PASS |  |
| TGT-REG-002 | registers | `dump 0x00 64` | 00:, 30: | 00: 00 00 00 00 00 00 08 88 09 00 20 00 00 00 00 6A  \|.......... ....j\| \| 10: 40 40 44 00 00 08 00 00 E0 20 00 00 00 00 07 C2  \|@@D...... ......\| \| 20: 32 06 0D 00 45 FF 8E FF 4... | 0.109s | PASS |  |
| TGT-REG-003 | registers | `wreg 0x73 0x00` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.110s | PASS |  |
| TGT-INV-001 | invalid-input | `odrg 1.6` | INVALID_PARAM; allowed failures: INVALID_PARAM | Status: INVALID_PARAM (code=5, detail=0) \| Message: Invalid gyro ODR/power-mode combination | 0.110s | PASS |  |
| TGT-INV-002 | invalid-input | `dump 0x10 0` | Invalid dump | Invalid dump range | 0.109s | PASS |  |
| TGT-INV-003 | invalid-input | `rreg 0x100` | Invalid register | Invalid register | 0.109s | PASS |  |
| TGT-STRESS-001 | stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| TGT-STRESS-002 | stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| TGT-SELF-001 | self-test | `selftest` | Selftest result: | === LSM6DS3TR selftest (safe commands) === \| [PASS] probe responds \| [PASS] probe no-health-side-effects \| [PASS] readWhoAmI \| ... | 1.313s | PASS |  |
| TGT-POST-001 | post-check | `refresh` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK | 0.109s | PASS |  |
| TGT-POST-002 | post-check | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| TGT-POST-003 | post-check | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
