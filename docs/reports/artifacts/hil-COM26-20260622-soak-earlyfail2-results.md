# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-22T20:38:06.603980+02:00
- Ended: 2026-06-22T20:40:45.279959+02:00
- Suite: soak
- Port: COM26
- Baud: 115200
- Result counts: PASS=554, FAIL=1, UNKNOWN=0, NOT_RUN=0
- Timing: count=555, min=0.093s, mean=0.178s, max=25.109s
- Transcript: `docs/reports/artifacts/hil-COM26-20260622-soak-8h-transcript.txt`

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
| SOAK-RAW-000001 | soak/data | `raw` | Raw: | Raw: ax=65 ay=-307 az=16695 \| gx=18 gy=-182 gz=-117 \| t=1207 | 0.110s | PASS |  |
| SOAK-READ-000002 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.006 ay=-0.019 az=+1.019 g \| gx=+0.17 gy=-1.59 gz=-1.07 dps \| t=29.72 C | 0.109s | PASS |  |
| SOAK-ACCEL-000003 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.019 g (raw: 81 -311 16698) | 0.109s | PASS |  |
| SOAK-GYRO-000004 | soak/data | `gyro` | Gyro | Gyro: x=0.11 y=-1.61 z=-1.05 dps (raw: 13 -184 -120) | 0.109s | PASS |  |
| SOAK-TEMP-000005 | soak/data | `temp` | Temp | Temp: 29.78 C (raw: 1224) | 0.110s | PASS |  |
| SOAK-STATUS-000006 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000007 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000008 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000009 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000010 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000011 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000012 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000013 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000014 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.546s | PASS |  |
| SOAK-MIX-000015 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000016 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 646 -> 663 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000017 | soak/data | `raw` | Raw: | Raw: ax=82 ay=-314 az=16688 \| gx=13 gy=-182 gz=-121 \| t=1214 | 0.109s | PASS |  |
| SOAK-READ-000018 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.017 g \| gx=+0.15 gy=-1.64 gz=-1.06 dps \| t=29.69 C | 0.109s | PASS |  |
| SOAK-ACCEL-000019 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.018 g (raw: 72 -316 16694) | 0.110s | PASS |  |
| SOAK-GYRO-000020 | soak/data | `gyro` | Gyro | Gyro: x=0.12 y=-1.62 z=-1.02 dps (raw: 14 -185 -117) | 0.109s | PASS |  |
| SOAK-TEMP-000021 | soak/data | `temp` | Temp | Temp: 29.71 C (raw: 1206) | 0.109s | PASS |  |
| SOAK-STATUS-000022 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000023 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000024 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000025 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000026 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000027 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000028 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.093s | PASS |  |
| SOAK-STEPS-000029 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000030 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000031 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000032 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 1310 -> 1327 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000033 | soak/data | `raw` | Raw: | Raw: ax=62 ay=-312 az=16706 \| gx=14 gy=-182 gz=-121 \| t=1215 | 0.110s | PASS |  |
| SOAK-READ-000034 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.019 g \| gx=+0.15 gy=-1.60 gz=-1.03 dps \| t=29.71 C | 0.109s | PASS |  |
| SOAK-ACCEL-000035 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.019 g (raw: 87 -325 16703) | 0.109s | PASS |  |
| SOAK-GYRO-000036 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.61 z=-1.07 dps (raw: 18 -184 -122) | 0.109s | PASS |  |
| SOAK-TEMP-000037 | soak/data | `temp` | Temp | Temp: 29.75 C (raw: 1217) | 0.110s | PASS |  |
| SOAK-STATUS-000038 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000039 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000040 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000041 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000042 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000043 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000044 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000045 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000046 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.531s | PASS |  |
| SOAK-MIX-000047 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000048 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 1974 -> 1991 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000049 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-317 az=16686 \| gx=16 gy=-182 gz=-121 \| t=1209 | 0.110s | PASS |  |
| SOAK-READ-000050 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.018 g \| gx=+0.11 gy=-1.60 gz=-1.07 dps \| t=29.76 C | 0.110s | PASS |  |
| SOAK-ACCEL-000051 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 78 -316 16693) | 0.109s | PASS |  |
| SOAK-GYRO-000052 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.66 z=-1.09 dps (raw: 15 -190 -124) | 0.109s | PASS |  |
| SOAK-TEMP-000053 | soak/data | `temp` | Temp | Temp: 29.76 C (raw: 1218) | 0.110s | PASS |  |
| SOAK-STATUS-000054 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000055 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000056 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000057 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000058 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000059 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000060 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.094s | PASS |  |
| SOAK-STEPS-000061 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000062 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.532s | PASS |  |
| SOAK-MIX-000063 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000064 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 2645 -> 2662 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000065 | soak/data | `raw` | Raw: | Raw: ax=78 ay=-313 az=16692 \| gx=16 gy=-182 gz=-118 \| t=1208 | 0.109s | PASS |  |
| SOAK-READ-000066 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.019 g \| gx=+0.13 gy=-1.58 gz=-1.07 dps \| t=29.76 C | 0.109s | PASS |  |
| SOAK-ACCEL-000067 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 77 -312 16696) | 0.110s | PASS |  |
| SOAK-GYRO-000068 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.59 z=-1.02 dps (raw: 17 -182 -117) | 0.110s | PASS |  |
| SOAK-TEMP-000069 | soak/data | `temp` | Temp | Temp: 29.73 C (raw: 1211) | 0.109s | PASS |  |
| SOAK-STATUS-000070 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000071 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000072 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000073 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000074 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000075 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000076 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000077 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000078 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000079 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000080 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 3308 -> 3325 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000081 | soak/data | `raw` | Raw: | Raw: ax=70 ay=-310 az=16703 \| gx=14 gy=-181 gz=-120 \| t=1210 | 0.110s | PASS |  |
| SOAK-READ-000082 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.018 g \| gx=+0.12 gy=-1.60 gz=-1.03 dps \| t=29.75 C | 0.110s | PASS |  |
| SOAK-ACCEL-000083 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.019 g (raw: 84 -314 16700) | 0.109s | PASS |  |
| SOAK-GYRO-000084 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.60 z=-1.08 dps (raw: 15 -183 -123) | 0.109s | PASS |  |
| SOAK-TEMP-000085 | soak/data | `temp` | Temp | Temp: 29.75 C (raw: 1217) | 0.110s | PASS |  |
| SOAK-STATUS-000086 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000087 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000088 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000089 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000090 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000091 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000092 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000093 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000094 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000095 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000096 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 3971 -> 3988 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000097 | soak/data | `raw` | Raw: | Raw: ax=90 ay=-313 az=16687 \| gx=13 gy=-182 gz=-123 \| t=1215 | 0.109s | PASS |  |
| SOAK-READ-000098 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.018 g \| gx=+0.12 gy=-1.62 gz=-1.03 dps \| t=29.74 C | 0.109s | PASS |  |
| SOAK-ACCEL-000099 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.017 g (raw: 86 -302 16680) | 0.110s | PASS |  |
| SOAK-GYRO-000100 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.58 z=-1.05 dps (raw: 17 -180 -120) | 0.110s | PASS |  |
| SOAK-TEMP-000101 | soak/data | `temp` | Temp | Temp: 29.77 C (raw: 1221) | 0.109s | PASS |  |
| SOAK-STATUS-000102 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000103 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000104 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000105 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000106 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000107 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000108 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000109 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000110 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000111 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000112 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 4637 -> 4654 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000113 | soak/data | `raw` | Raw: | Raw: ax=95 ay=-311 az=16684 \| gx=20 gy=-184 gz=-124 \| t=1217 | 0.094s | PASS |  |
| SOAK-READ-000114 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.018 g \| gx=+0.13 gy=-1.58 gz=-1.07 dps \| t=29.72 C | 0.110s | PASS |  |
| SOAK-ACCEL-000115 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.017 g (raw: 83 -314 16677) | 0.125s | PASS |  |
| SOAK-GYRO-000116 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.59 z=-1.08 dps (raw: 19 -182 -123) | 0.109s | PASS |  |
| SOAK-TEMP-000117 | soak/data | `temp` | Temp | Temp: 29.71 C (raw: 1205) | 0.094s | PASS |  |
| SOAK-STATUS-000118 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000119 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000120 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000121 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000122 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000123 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000124 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.093s | PASS |  |
| SOAK-STEPS-000125 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000126 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.531s | PASS |  |
| SOAK-MIX-000127 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.094s | PASS |  |
| SOAK-RECOVER-000128 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 5307 -> 5324 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000129 | soak/data | `raw` | Raw: | Raw: ax=86 ay=-305 az=16677 \| gx=19 gy=-186 gz=-124 \| t=1212 | 0.094s | PASS |  |
| SOAK-READ-000130 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.017 g \| gx=+0.15 gy=-1.58 gz=-1.05 dps \| t=29.71 C | 0.093s | PASS |  |
| SOAK-ACCEL-000131 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.018 g (raw: 89 -321 16688) | 0.094s | PASS |  |
| SOAK-GYRO-000132 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.61 z=-1.02 dps (raw: 18 -184 -116) | 0.109s | PASS |  |
| SOAK-TEMP-000133 | soak/data | `temp` | Temp | Temp: 29.73 C (raw: 1212) | 0.109s | PASS |  |
| SOAK-STATUS-000134 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000135 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000136 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000137 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000138 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000139 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000140 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000141 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000142 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000143 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000144 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 5976 -> 5993 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000145 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-323 az=16648 \| gx=14 gy=-180 gz=-119 \| t=1215 | 0.110s | PASS |  |
| SOAK-READ-000146 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.018 g \| gx=+0.13 gy=-1.60 gz=-1.07 dps \| t=29.75 C | 0.109s | PASS |  |
| SOAK-ACCEL-000147 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 79 -312 16681) | 0.109s | PASS |  |
| SOAK-GYRO-000148 | soak/data | `gyro` | Gyro | Gyro: x=0.09 y=-1.58 z=-1.09 dps (raw: 10 -180 -125) | 0.109s | PASS |  |
| SOAK-TEMP-000149 | soak/data | `temp` | Temp | Temp: 29.79 C (raw: 1225) | 0.110s | PASS |  |
| SOAK-STATUS-000150 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000151 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000152 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000153 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000154 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000155 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000156 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.094s | PASS |  |
| SOAK-STEPS-000157 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000158 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.531s | PASS |  |
| SOAK-MIX-000159 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000160 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 6642 -> 6659 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000161 | soak/data | `raw` | Raw: | Raw: ax=71 ay=-298 az=16677 \| gx=15 gy=-182 gz=-117 \| t=1220 | 0.110s | PASS |  |
| SOAK-READ-000162 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.017 g \| gx=+0.17 gy=-1.58 gz=-1.09 dps \| t=29.71 C | 0.110s | PASS |  |
| SOAK-ACCEL-000163 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.018 g (raw: 75 -297 16688) | 0.109s | PASS |  |
| SOAK-GYRO-000164 | soak/data | `gyro` | Gyro | Gyro: x=0.12 y=-1.61 z=-1.08 dps (raw: 14 -184 -123) | 0.109s | PASS |  |
| SOAK-TEMP-000165 | soak/data | `temp` | Temp | Temp: 29.72 C (raw: 1209) | 0.110s | PASS |  |
| SOAK-STATUS-000166 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000167 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000168 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000169 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000170 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000171 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000172 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000173 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000174 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000175 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000176 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 7310 -> 7327 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000177 | soak/data | `raw` | Raw: | Raw: ax=80 ay=-329 az=16695 \| gx=14 gy=-181 gz=-124 \| t=1207 | 0.109s | PASS |  |
| SOAK-READ-000178 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.021 az=+1.017 g \| gx=+0.18 gy=-1.60 gz=-1.07 dps \| t=29.74 C | 0.109s | PASS |  |
| SOAK-ACCEL-000179 | soak/data | `accel` | Accel | Accel: x=0.003 y=-0.019 z=1.018 g (raw: 49 -307 16681) | 0.110s | PASS |  |
| SOAK-GYRO-000180 | soak/data | `gyro` | Gyro | Gyro: x=0.12 y=-1.62 z=-1.08 dps (raw: 14 -185 -123) | 0.110s | PASS |  |
| SOAK-TEMP-000181 | soak/data | `temp` | Temp | Temp: 29.75 C (raw: 1215) | 0.109s | PASS |  |
| SOAK-STATUS-000182 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000183 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000184 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000185 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000186 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000187 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000188 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000189 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000190 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000191 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000192 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 7975 -> 7992 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000193 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-331 az=16698 \| gx=15 gy=-180 gz=-127 \| t=1214 | 0.110s | PASS |  |
| SOAK-READ-000194 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.020 az=+1.019 g \| gx=+0.17 gy=-1.61 gz=-1.05 dps \| t=29.76 C | 0.110s | PASS |  |
| SOAK-ACCEL-000195 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.019 g (raw: 89 -327 16697) | 0.109s | PASS |  |
| SOAK-GYRO-000196 | soak/data | `gyro` | Gyro | Gyro: x=0.10 y=-1.61 z=-1.09 dps (raw: 11 -184 -124) | 0.109s | PASS |  |
| SOAK-TEMP-000197 | soak/data | `temp` | Temp | Temp: 29.75 C (raw: 1216) | 0.110s | PASS |  |
| SOAK-STATUS-000198 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000199 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000200 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000201 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000202 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000203 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000204 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.093s | PASS |  |
| SOAK-STEPS-000205 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000206 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.531s | PASS |  |
| SOAK-MIX-000207 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000208 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 8647 -> 8664 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000209 | soak/data | `raw` | Raw: | Raw: ax=80 ay=-314 az=16693 \| gx=14 gy=-186 gz=-121 \| t=1226 | 0.109s | PASS |  |
| SOAK-READ-000210 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.018 g \| gx=+0.13 gy=-1.58 gz=-1.03 dps \| t=29.77 C | 0.109s | PASS |  |
| SOAK-ACCEL-000211 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.019 g (raw: 79 -312 16701) | 0.110s | PASS |  |
| SOAK-GYRO-000212 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.59 z=-1.08 dps (raw: 18 -182 -123) | 0.110s | PASS |  |
| SOAK-TEMP-000213 | soak/data | `temp` | Temp | Temp: 29.75 C (raw: 1217) | 0.109s | PASS |  |
| SOAK-STATUS-000214 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000215 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000216 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000217 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000218 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000219 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000220 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000221 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000222 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.531s | PASS |  |
| SOAK-MIX-000223 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000224 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 9308 -> 9325 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000225 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-291 az=16701 \| gx=16 gy=-182 gz=-122 \| t=1232 | 0.110s | PASS |  |
| SOAK-READ-000226 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.018 g \| gx=+0.12 gy=-1.60 gz=-1.05 dps \| t=29.78 C | 0.110s | PASS |  |
| SOAK-ACCEL-000227 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.019 g (raw: 78 -310 16701) | 0.109s | PASS |  |
| SOAK-GYRO-000228 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.60 z=-1.05 dps (raw: 18 -183 -120) | 0.109s | PASS |  |
| SOAK-TEMP-000229 | soak/data | `temp` | Temp | Temp: 29.78 C (raw: 1223) | 0.110s | PASS |  |
| SOAK-STATUS-000230 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000231 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000232 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000233 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000234 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000235 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000236 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000237 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000238 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.531s | PASS |  |
| SOAK-MIX-000239 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000240 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 9976 -> 9993 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000241 | soak/data | `raw` | Raw: | Raw: ax=87 ay=-307 az=16684 \| gx=15 gy=-185 gz=-127 \| t=1221 | 0.109s | PASS |  |
| SOAK-READ-000242 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.018 az=+1.019 g \| gx=+0.18 gy=-1.58 gz=-1.08 dps \| t=29.76 C | 0.109s | PASS |  |
| SOAK-ACCEL-000243 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 78 -314 16691) | 0.110s | PASS |  |
| SOAK-GYRO-000244 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.61 z=-1.10 dps (raw: 16 -184 -126) | 0.110s | PASS |  |
| SOAK-TEMP-000245 | soak/data | `temp` | Temp | Temp: 29.76 C (raw: 1219) | 0.109s | PASS |  |
| SOAK-STATUS-000246 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000247 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000248 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000249 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000250 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000251 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000252 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000253 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000254 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.531s | PASS |  |
| SOAK-MIX-000255 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000256 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 10641 -> 10658 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000257 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-309 az=16711 \| gx=17 gy=-182 gz=-122 \| t=1235 | 0.110s | PASS |  |
| SOAK-READ-000258 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.019 g \| gx=+0.16 gy=-1.60 gz=-1.06 dps \| t=29.84 C | 0.110s | PASS |  |
| SOAK-ACCEL-000259 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 81 -306 16695) | 0.093s | PASS |  |
| SOAK-GYRO-000260 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.58 z=-1.04 dps (raw: 19 -181 -119) | 0.094s | PASS |  |
| SOAK-TEMP-000261 | soak/data | `temp` | Temp | Temp: 29.80 C (raw: 1228) | 0.094s | PASS |  |
| SOAK-STATUS-000262 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000263 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000264 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000265 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000266 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000267 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000268 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000269 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000270 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000271 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000272 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 11309 -> 11326 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000273 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-305 az=16689 \| gx=16 gy=-184 gz=-125 \| t=1229 | 0.093s | PASS |  |
| SOAK-READ-000274 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.019 g \| gx=+0.10 gy=-1.60 gz=-1.05 dps \| t=29.77 C | 0.094s | PASS |  |
| SOAK-ACCEL-000275 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.017 g (raw: 82 -312 16677) | 0.094s | PASS |  |
| SOAK-GYRO-000276 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.62 z=-1.05 dps (raw: 16 -185 -120) | 0.094s | PASS |  |
| SOAK-TEMP-000277 | soak/data | `temp` | Temp | Temp: 29.77 C (raw: 1221) | 0.093s | PASS |  |
| SOAK-STATUS-000278 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000279 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000280 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000281 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000282 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000283 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000284 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000285 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000286 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000287 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000288 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 11976 -> 11993 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000289 | soak/data | `raw` | Raw: | Raw: ax=97 ay=-301 az=16698 \| gx=16 gy=-184 gz=-121 \| t=1230 | 0.094s | PASS |  |
| SOAK-READ-000290 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.018 g \| gx=+0.12 gy=-1.57 gz=-1.05 dps \| t=29.78 C | 0.094s | PASS |  |
| SOAK-ACCEL-000291 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 82 -317 16684) | 0.093s | PASS |  |
| SOAK-GYRO-000292 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.58 z=-1.09 dps (raw: 16 -180 -125) | 0.110s | PASS |  |
| SOAK-TEMP-000293 | soak/data | `temp` | Temp | Temp: 29.81 C (raw: 1232) | 0.109s | PASS |  |
| SOAK-STATUS-000294 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000295 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000296 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000297 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000298 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000299 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000300 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000301 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000302 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000303 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000304 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 12640 -> 12657 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000305 | soak/data | `raw` | Raw: | Raw: ax=179 ay=-162 az=16737 \| gx=20 gy=-193 gz=-129 \| t=1223 | 0.109s | PASS |  |
| SOAK-READ-000306 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.010 ay=-0.021 az=+1.018 g \| gx=+0.18 gy=-1.64 gz=-1.01 dps \| t=29.78 C | 0.094s | PASS |  |
| SOAK-ACCEL-000307 | soak/data | `accel` | Accel | Accel: x=0.003 y=-0.019 z=1.023 g (raw: 56 -310 16773) | 0.109s | PASS |  |
| SOAK-GYRO-000308 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.58 z=-1.02 dps (raw: 18 -181 -116) | 0.109s | PASS |  |
| SOAK-TEMP-000309 | soak/data | `temp` | Temp | Temp: 29.80 C (raw: 1229) | 0.109s | PASS |  |
| SOAK-STATUS-000310 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000311 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000312 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000313 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000314 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000315 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000316 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000317 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000318 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000319 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000320 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 13306 -> 13323 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000321 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-306 az=16700 \| gx=17 gy=-185 gz=-123 \| t=1225 | 0.109s | PASS |  |
| SOAK-READ-000322 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.018 az=+1.018 g \| gx=+0.13 gy=-1.60 gz=-1.07 dps \| t=29.79 C | 0.110s | PASS |  |
| SOAK-ACCEL-000323 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 84 -304 16696) | 0.110s | PASS |  |
| SOAK-GYRO-000324 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.61 z=-1.04 dps (raw: 15 -184 -119) | 0.109s | PASS |  |
| SOAK-TEMP-000325 | soak/data | `temp` | Temp | Temp: 29.80 C (raw: 1228) | 0.109s | PASS |  |
| SOAK-STATUS-000326 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000327 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000328 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000329 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000330 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000331 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000332 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000333 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000334 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000335 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000336 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 13970 -> 13987 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000337 | soak/data | `raw` | Raw: | Raw: ax=87 ay=-307 az=16680 \| gx=15 gy=-185 gz=-121 \| t=1224 | 0.110s | PASS |  |
| SOAK-READ-000338 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.018 g \| gx=+0.13 gy=-1.58 gz=-1.04 dps \| t=29.79 C | 0.109s | PASS |  |
| SOAK-ACCEL-000339 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.020 z=1.019 g (raw: 71 -333 16697) | 0.109s | PASS |  |
| SOAK-GYRO-000340 | soak/data | `gyro` | Gyro | Gyro: x=0.12 y=-1.62 z=-1.06 dps (raw: 14 -185 -121) | 0.110s | PASS |  |
| SOAK-TEMP-000341 | soak/data | `temp` | Temp | Temp: 29.79 C (raw: 1226) | 0.110s | PASS |  |
| SOAK-STATUS-000342 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000343 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000344 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000345 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000346 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000347 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000348 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000349 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000350 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000351 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000352 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 14637 -> 14654 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000353 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-311 az=16725 \| gx=19 gy=-185 gz=-120 \| t=1234 | 0.109s | PASS |  |
| SOAK-READ-000354 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.018 g \| gx=+0.13 gy=-1.59 gz=-1.06 dps \| t=29.79 C | 0.110s | PASS |  |
| SOAK-ACCEL-000355 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.019 g (raw: 81 -315 16701) | 0.110s | PASS |  |
| SOAK-GYRO-000356 | soak/data | `gyro` | Gyro | Gyro: x=0.11 y=-1.62 z=-1.04 dps (raw: 13 -185 -119) | 0.109s | PASS |  |
| SOAK-TEMP-000357 | soak/data | `temp` | Temp | Temp: 29.82 C (raw: 1235) | 0.109s | PASS |  |
| SOAK-STATUS-000358 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000359 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000360 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000361 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000362 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000363 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000364 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000365 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000366 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.531s | PASS |  |
| SOAK-MIX-000367 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.094s | PASS |  |
| SOAK-RECOVER-000368 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 15307 -> 15324 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000369 | soak/data | `raw` | Raw: | Raw: ax=85 ay=-320 az=16698 \| gx=14 gy=-183 gz=-123 \| t=1241 | 0.110s | PASS |  |
| SOAK-READ-000370 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.006 ay=-0.019 az=+1.020 g \| gx=+0.17 gy=-1.59 gz=-1.04 dps \| t=29.81 C | 0.093s | PASS |  |
| SOAK-ACCEL-000371 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.018 g (raw: 70 -308 16696) | 0.094s | PASS |  |
| SOAK-GYRO-000372 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.59 z=-1.03 dps (raw: 19 -182 -118) | 0.094s | PASS |  |
| SOAK-TEMP-000373 | soak/data | `temp` | Temp | Temp: 29.82 C (raw: 1233) | 0.094s | PASS |  |
| SOAK-STATUS-000374 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000375 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000376 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000377 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000378 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000379 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000380 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.094s | PASS |  |
| SOAK-STEPS-000381 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000382 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.531s | PASS |  |
| SOAK-MIX-000383 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.094s | PASS |  |
| SOAK-RECOVER-000384 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 15970 -> 15987 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000385 | soak/data | `raw` | Raw: | Raw: ax=69 ay=-313 az=16695 \| gx=17 gy=-182 gz=-122 \| t=1246 | 0.094s | PASS |  |
| SOAK-READ-000386 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.018 g \| gx=+0.16 gy=-1.63 gz=-1.09 dps \| t=29.81 C | 0.094s | PASS |  |
| SOAK-ACCEL-000387 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.019 g (raw: 75 -318 16707) | 0.094s | PASS |  |
| SOAK-GYRO-000388 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.64 z=-1.05 dps (raw: 15 -187 -120) | 0.093s | PASS |  |
| SOAK-TEMP-000389 | soak/data | `temp` | Temp | Temp: 29.85 C (raw: 1242) | 0.094s | PASS |  |
| SOAK-STATUS-000390 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000391 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000392 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000393 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000394 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000395 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000396 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000397 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000398 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.546s | PASS |  |
| SOAK-MIX-000399 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000400 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 16640 -> 16657 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000401 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-322 az=16701 \| gx=13 gy=-183 gz=-122 \| t=1237 | 0.109s | PASS |  |
| SOAK-READ-000402 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.019 g \| gx=+0.16 gy=-1.60 gz=-1.05 dps \| t=29.83 C | 0.109s | PASS |  |
| SOAK-ACCEL-000403 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.019 g (raw: 79 -313 16709) | 0.110s | PASS |  |
| SOAK-GYRO-000404 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.64 z=-1.04 dps (raw: 18 -187 -119) | 0.109s | PASS |  |
| SOAK-TEMP-000405 | soak/data | `temp` | Temp | Temp: 29.81 C (raw: 1231) | 0.109s | PASS |  |
| SOAK-STATUS-000406 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000407 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000408 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000409 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000410 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000411 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000412 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000413 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000414 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000415 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000416 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 17308 -> 17325 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000417 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-311 az=16689 \| gx=17 gy=-182 gz=-121 \| t=1242 | 0.110s | PASS |  |
| SOAK-READ-000418 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.017 g \| gx=+0.18 gy=-1.59 gz=-1.03 dps \| t=29.84 C | 0.109s | PASS |  |
| SOAK-ACCEL-000419 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 80 -307 16690) | 0.109s | PASS |  |
| SOAK-GYRO-000420 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.62 z=-1.05 dps (raw: 19 -185 -120) | 0.109s | PASS |  |
| SOAK-TEMP-000421 | soak/data | `temp` | Temp | Temp: 29.84 C (raw: 1240) | 0.110s | PASS |  |
| SOAK-STATUS-000422 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000423 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000424 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000425 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000426 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000427 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000428 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000429 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000430 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000431 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000432 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 17974 -> 17991 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000433 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-308 az=16692 \| gx=16 gy=-184 gz=-122 \| t=1233 | 0.110s | PASS |  |
| SOAK-READ-000434 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.017 g \| gx=+0.14 gy=-1.59 gz=-1.05 dps \| t=29.83 C | 0.094s | PASS |  |
| SOAK-ACCEL-000435 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.017 g (raw: 84 -319 16680) | 0.093s | PASS |  |
| SOAK-GYRO-000436 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.60 z=-1.07 dps (raw: 18 -183 -122) | 0.094s | PASS |  |
| SOAK-TEMP-000437 | soak/data | `temp` | Temp | Temp: 29.86 C (raw: 1243) | 0.094s | PASS |  |
| SOAK-STATUS-000438 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000439 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000440 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000441 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000442 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000443 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000444 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.094s | PASS |  |
| SOAK-STEPS-000445 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000446 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.532s | PASS |  |
| SOAK-MIX-000447 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000448 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 18636 -> 18653 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000449 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-309 az=16669 \| gx=18 gy=-187 gz=-121 \| t=1239 | 0.093s | PASS |  |
| SOAK-READ-000450 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.017 g \| gx=+0.13 gy=-1.58 gz=-1.02 dps \| t=29.82 C | 0.094s | PASS |  |
| SOAK-ACCEL-000451 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.018 z=1.017 g (raw: 73 -302 16680) | 0.109s | PASS |  |
| SOAK-GYRO-000452 | soak/data | `gyro` | Gyro | Gyro: x=0.11 y=-1.60 z=-1.10 dps (raw: 13 -183 -126) | 0.109s | PASS |  |
| SOAK-TEMP-000453 | soak/data | `temp` | Temp | Temp: 29.83 C (raw: 1236) | 0.109s | PASS |  |
| SOAK-STATUS-000454 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000455 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000456 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000457 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000458 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000459 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000460 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000461 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000462 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000463 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000464 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 19304 -> 19321 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000465 | soak/data | `raw` | Raw: | Raw: ax=82 ay=-316 az=16676 \| gx=18 gy=-188 gz=-118 \| t=1239 | 0.109s | PASS |  |
| SOAK-READ-000466 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.018 g \| gx=+0.12 gy=-1.62 gz=-1.05 dps \| t=29.80 C | 0.109s | PASS |  |
| SOAK-ACCEL-000467 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.017 g (raw: 89 -311 16680) | 0.109s | PASS |  |
| SOAK-GYRO-000468 | soak/data | `gyro` | Gyro | Gyro: x=0.12 y=-1.61 z=-1.08 dps (raw: 14 -184 -123) | 0.110s | PASS |  |
| SOAK-TEMP-000469 | soak/data | `temp` | Temp | Temp: 29.82 C (raw: 1233) | 0.109s | PASS |  |
| SOAK-STATUS-000470 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000471 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000472 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000473 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000474 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000475 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000476 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.094s | PASS |  |
| SOAK-STEPS-000477 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000478 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.531s | PASS |  |
| SOAK-MIX-000479 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000480 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 19974 -> 19991 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000481 | soak/data | `raw` | Raw: | Raw: ax=99 ay=-325 az=16680 \| gx=17 gy=-185 gz=-123 \| t=1239 | 0.110s | PASS |  |
| SOAK-READ-000482 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.018 g \| gx=+0.12 gy=-1.61 gz=-1.06 dps \| t=29.81 C | 0.109s | PASS |  |
| SOAK-ACCEL-000483 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.021 g (raw: 74 -311 16735) | 0.109s | PASS |  |
| SOAK-GYRO-000484 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.60 z=-1.03 dps (raw: 18 -183 -118) | 0.110s | PASS |  |
| SOAK-TEMP-000485 | soak/data | `temp` | Temp | Temp: 29.83 C (raw: 1237) | 0.110s | PASS |  |
| SOAK-STATUS-000486 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000487 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000488 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000489 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000490 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000491 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000492 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000493 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000494 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000495 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000496 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 20642 -> 20659 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000497 | soak/data | `raw` | Raw: | Raw: ax=84 ay=-306 az=16690 \| gx=19 gy=-188 gz=-117 \| t=1226 | 0.125s | PASS |  |
| SOAK-READ-000498 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.019 g \| gx=+0.12 gy=-1.61 gz=-1.07 dps \| t=29.83 C | 0.110s | PASS |  |
| SOAK-ACCEL-000499 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 75 -314 16691) | 0.110s | PASS |  |
| SOAK-GYRO-000500 | soak/data | `gyro` | Gyro | Gyro: x=0.11 y=-1.61 z=-1.02 dps (raw: 13 -184 -117) | 0.109s | PASS |  |
| SOAK-TEMP-000501 | soak/data | `temp` | Temp | Temp: 29.82 C (raw: 1233) | 0.109s | PASS |  |
| SOAK-STATUS-000502 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000503 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000504 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000505 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000506 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000507 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000508 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000509 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000510 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.546s | PASS |  |
| SOAK-MIX-000511 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000512 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 21309 -> 21326 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000513 | soak/data | `raw` | Raw: | Raw: ax=87 ay=-305 az=16702 \| gx=15 gy=-184 gz=-123 \| t=1246 | 0.109s | PASS |  |
| SOAK-READ-000514 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.019 g \| gx=+0.13 gy=-1.60 gz=-1.07 dps \| t=29.84 C | 0.109s | PASS |  |
| SOAK-ACCEL-000515 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.018 z=1.018 g (raw: 94 -294 16686) | 0.094s | PASS |  |
| SOAK-GYRO-000516 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.65 z=-1.07 dps (raw: 16 -188 -122) | 0.093s | PASS |  |
| SOAK-TEMP-000517 | soak/data | `temp` | Temp | Temp: 29.81 C (raw: 1231) | 0.109s | PASS |  |
| SOAK-STATUS-000518 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000519 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000520 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000521 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000522 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000523 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000524 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.109s | PASS |  |
| SOAK-STEPS-000525 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000526 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000527 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.110s | PASS |  |
| SOAK-RECOVER-000528 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 21980 -> 21997 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000529 | soak/data | `raw` | Raw: | Raw: ax=89 ay=-305 az=16695 \| gx=19 gy=-183 gz=-121 \| t=1233 | 0.109s | PASS |  |
| SOAK-READ-000530 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.019 g \| gx=+0.13 gy=-1.61 gz=-1.06 dps \| t=29.83 C | 0.109s | PASS |  |
| SOAK-ACCEL-000531 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.018 g (raw: 77 -312 16683) | 0.110s | PASS |  |
| SOAK-GYRO-000532 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.58 z=-1.08 dps (raw: 15 -181 -123) | 0.110s | PASS |  |
| SOAK-TEMP-000533 | soak/data | `temp` | Temp | Temp: 29.81 C (raw: 1232) | 0.093s | PASS |  |
| SOAK-STATUS-000534 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000535 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000536 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000537 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000538 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000539 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000540 | soak/functions | `tsread` | Timestamp | Timestamp: 5601 | 0.110s | PASS |  |
| SOAK-STEPS-000541 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000542 | soak/stress | `stress 50` | Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.547s | PASS |  |
| SOAK-MIX-000543 | soak/stress | `stress_mix 50` | Mixed Stress Summary, Errors: | Progress: 5/50 (10%, ok=5, fail=0) \| Progress: 10/50 (20%, ok=10, fail=0) \| Progress: 15/50 (30%, ok=15, fail=0) \| Progress: 20/50 (40%, ok=20, fail=0) \| ... | 0.109s | PASS |  |
| SOAK-RECOVER-000544 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 22647 -> 22664 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000545 | soak/data | `raw` | Raw: | Raw: ax=80 ay=-305 az=16703 \| gx=13 gy=-187 gz=-117 \| t=1237 | 0.110s | PASS |  |
| SOAK-READ-000546 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.019 g \| gx=+0.16 gy=-1.62 gz=-1.02 dps \| t=29.82 C | 0.110s | PASS |  |
| SOAK-ACCEL-000547 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.018 g (raw: 85 -303 16690) | 0.109s | PASS |  |
| SOAK-GYRO-000548 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.59 z=-1.07 dps (raw: 19 -182 -122) | 0.093s | PASS |  |
| SOAK-TEMP-000549 | soak/data | `temp` | Temp | Temp: 29.82 C (raw: 1235) | 0.110s | PASS |  |
| SOAK-STATUS-000550 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000551 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000552 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000553 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000554 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000555 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 25.109s | FAIL | Prompt not seen before timeout |
