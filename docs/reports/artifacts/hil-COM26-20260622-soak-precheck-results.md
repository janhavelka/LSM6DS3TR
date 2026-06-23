# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-22T20:28:18.728519+02:00
- Ended: 2026-06-22T20:28:24.087035+02:00
- Suite: soak
- Port: COM26
- Baud: 115200
- Result counts: PASS=20, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=20, min=0.093s, mean=0.110s, max=0.219s
- Transcript: `docs/reports/artifacts/hil-COM26-20260622-soak-precheck-transcript.txt`

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
| SOAK-RAW-000001 | soak/data | `raw` | Raw: | Raw: ax=66 ay=-314 az=16684 \| gx=15 gy=-182 gz=-120 \| t=1163 | 0.109s | PASS |  |
| SOAK-READ-000002 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.017 g \| gx=+0.14 gy=-1.60 gz=-1.03 dps \| t=29.52 C | 0.109s | PASS |  |
| SOAK-ACCEL-000003 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.018 g (raw: 86 -294 16692) | 0.110s | PASS |  |
| SOAK-GYRO-000004 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.58 z=-1.02 dps (raw: 16 -181 -117) | 0.110s | PASS |  |
| SOAK-TEMP-000005 | soak/data | `temp` | Temp | Temp: 29.52 C (raw: 1158) | 0.109s | PASS |  |
| SOAK-STATUS-000006 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000007 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000008 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000009 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000010 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000011 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000012 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.094s | PASS |  |
| SOAK-STEPS-000013 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000014 | soak/stress | `stress 20` | Stress Summary, Errors: | Progress: 2/20 (10%, ok=2, fail=0) \| Progress: 4/20 (20%, ok=4, fail=0) \| Progress: 6/20 (30%, ok=6, fail=0) \| Progress: 8/20 (40%, ok=8, fail=0) \| ... | 0.219s | PASS |  |
| SOAK-MIX-000015 | soak/stress | `stress_mix 20` | Mixed Stress Summary, Errors: | Progress: 2/20 (10%, ok=2, fail=0) \| Progress: 4/20 (20%, ok=4, fail=0) \| Progress: 6/20 (30%, ok=6, fail=0) \| Progress: 8/20 (40%, ok=8, fail=0) \| ... | 0.093s | PASS |  |
| SOAK-RECOVER-000016 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 258 -> 275 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000017 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-306 az=16679 \| gx=14 gy=-187 gz=-120 \| t=1157 | 0.110s | PASS |  |
| SOAK-READ-000018 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.018 g \| gx=+0.12 gy=-1.60 gz=-1.02 dps \| t=29.54 C | 0.109s | PASS |  |
| SOAK-ACCEL-000019 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 85 -319 16689) | 0.109s | PASS |  |
| SOAK-GYRO-000020 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.58 z=-1.05 dps (raw: 16 -180 -120) | 0.110s | PASS |  |
