# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-22T20:36:43.072785+02:00
- Ended: 2026-06-22T20:37:33.379232+02:00
- Suite: soak
- Port: COM26
- Baud: 115200
- Result counts: PASS=100, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=100, min=0.093s, mean=0.131s, max=0.547s
- Transcript: `docs/reports/artifacts/hil-COM26-20260622-soak-precheck2-transcript.txt`

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
| SOAK-RAW-000001 | soak/data | `raw` | Raw: | Raw: ax=68 ay=-302 az=16683 \| gx=16 gy=-182 gz=-120 \| t=1199 | 0.109s | PASS |  |
| SOAK-READ-000002 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.016 g \| gx=+0.11 gy=-1.63 gz=-1.05 dps \| t=29.67 C | 0.110s | PASS |  |
| SOAK-ACCEL-000003 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 81 -313 16683) | 0.109s | PASS |  |
| SOAK-GYRO-000004 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.61 z=-1.03 dps (raw: 15 -184 -118) | 0.109s | PASS |  |
| SOAK-TEMP-000005 | soak/data | `temp` | Temp | Temp: 29.70 C (raw: 1202) | 0.109s | PASS |  |
| SOAK-STATUS-000006 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000007 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000008 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000009 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000010 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000011 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000012 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.094s | PASS |  |
| SOAK-STEPS-000013 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000014 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.532s | PASS |  |
| SOAK-MIX-000015 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000016 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 648 -> 665 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000017 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-321 az=16663 \| gx=16 gy=-182 gz=-119 \| t=1190 | 0.109s | PASS |  |
| SOAK-READ-000018 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.020 az=+1.018 g \| gx=+0.14 gy=-1.62 gz=-1.07 dps \| t=29.67 C | 0.110s | PASS |  |
| SOAK-ACCEL-000019 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.020 z=1.016 g (raw: 58 -324 16649) | 0.110s | PASS |  |
| SOAK-GYRO-000020 | soak/data | `gyro` | Gyro | Gyro: x=0.11 y=-1.56 z=-1.06 dps (raw: 13 -178 -121) | 0.093s | PASS |  |
| SOAK-TEMP-000021 | soak/data | `temp` | Temp | Temp: 29.65 C (raw: 1190) | 0.094s | PASS |  |
| SOAK-STATUS-000022 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000023 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000024 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000025 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000026 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000027 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000028 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000029 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000030 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.546s | PASS |  |
| SOAK-MIX-000031 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000032 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 1314 -> 1331 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000033 | soak/data | `raw` | Raw: | Raw: ax=78 ay=-308 az=16692 \| gx=15 gy=-180 gz=-121 \| t=1201 | 0.109s | PASS |  |
| SOAK-READ-000034 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.006 ay=-0.019 az=+1.017 g \| gx=+0.12 gy=-1.61 gz=-1.05 dps \| t=29.66 C | 0.109s | PASS |  |
| SOAK-ACCEL-000035 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.017 g (raw: 74 -315 16680) | 0.110s | PASS |  |
| SOAK-GYRO-000036 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.59 z=-1.06 dps (raw: 17 -182 -121) | 0.109s | PASS |  |
| SOAK-TEMP-000037 | soak/data | `temp` | Temp | Temp: 29.66 C (raw: 1194) | 0.094s | PASS |  |
| SOAK-STATUS-000038 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000039 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000040 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000041 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000042 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000043 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000044 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.094s | PASS |  |
| SOAK-STEPS-000045 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000046 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.532s | PASS |  |
| SOAK-MIX-000047 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.094s | PASS |  |
| SOAK-RECOVER-000048 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 1980 -> 1997 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000049 | soak/data | `raw` | Raw: | Raw: ax=95 ay=-295 az=16677 \| gx=17 gy=-184 gz=-117 \| t=1195 | 0.109s | PASS |  |
| SOAK-READ-000050 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.016 g \| gx=+0.16 gy=-1.64 gz=-1.06 dps \| t=29.66 C | 0.110s | PASS |  |
| SOAK-ACCEL-000051 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.017 g (raw: 70 -312 16671) | 0.109s | PASS |  |
| SOAK-GYRO-000052 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.58 z=-1.04 dps (raw: 17 -181 -119) | 0.109s | PASS |  |
| SOAK-TEMP-000053 | soak/data | `temp` | Temp | Temp: 29.62 C (raw: 1182) | 0.109s | PASS |  |
| SOAK-STATUS-000054 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000055 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000056 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000057 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000058 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000059 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000060 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000061 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000062 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000063 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000064 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 2652 -> 2669 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000065 | soak/data | `raw` | Raw: | Raw: ax=90 ay=-314 az=16708 \| gx=17 gy=-184 gz=-122 \| t=1199 | 0.093s | PASS |  |
| SOAK-READ-000066 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.019 g \| gx=+0.15 gy=-1.61 gz=-1.07 dps \| t=29.67 C | 0.094s | PASS |  |
| SOAK-ACCEL-000067 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.018 g (raw: 81 -303 16688) | 0.110s | PASS |  |
| SOAK-GYRO-000068 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.60 z=-1.09 dps (raw: 17 -183 -124) | 0.109s | PASS |  |
| SOAK-TEMP-000069 | soak/data | `temp` | Temp | Temp: 29.66 C (raw: 1193) | 0.109s | PASS |  |
| SOAK-STATUS-000070 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000071 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000072 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000073 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000074 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000075 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000076 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.093s | PASS |  |
| SOAK-STEPS-000077 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000078 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000079 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.094s | PASS |  |
| SOAK-RECOVER-000080 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 3317 -> 3334 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000081 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-309 az=16686 \| gx=10 gy=-183 gz=-119 \| t=1202 | 0.094s | PASS |  |
| SOAK-READ-000082 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.018 g \| gx=+0.12 gy=-1.60 gz=-1.04 dps \| t=29.72 C | 0.093s | PASS |  |
| SOAK-ACCEL-000083 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 86 -317 16686) | 0.094s | PASS |  |
| SOAK-GYRO-000084 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.61 z=-1.06 dps (raw: 17 -184 -121) | 0.094s | PASS |  |
| SOAK-TEMP-000085 | soak/data | `temp` | Temp | Temp: 29.74 C (raw: 1214) | 0.094s | PASS |  |
| SOAK-STATUS-000086 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000087 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000088 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000089 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000090 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000091 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000092 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000093 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000094 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000095 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000096 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 3985 -> 4002 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000097 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-311 az=16697 \| gx=15 gy=-183 gz=-119 \| t=1202 | 0.110s | PASS |  |
| SOAK-READ-000098 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.018 g \| gx=+0.16 gy=-1.60 gz=-1.07 dps \| t=29.69 C | 0.109s | PASS |  |
| SOAK-ACCEL-000099 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.019 z=1.018 g (raw: 94 -317 16690) | 0.109s | PASS |  |
| SOAK-GYRO-000100 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.62 z=-1.04 dps (raw: 17 -185 -119) | 0.109s | PASS |  |
