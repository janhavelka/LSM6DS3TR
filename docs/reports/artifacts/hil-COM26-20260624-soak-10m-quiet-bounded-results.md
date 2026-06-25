# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-24T23:23:43.383452+02:00
- Ended: 2026-06-24T23:33:46.407080+02:00
- Suite: soak
- Port: COM26
- Baud: 115200
- Result counts: PASS=1102, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=1102, min=0.093s, mean=0.494s, max=36.031s
- Transcript: `docs/reports/artifacts/hil-COM26-20260624-soak-10m-quiet-bounded-transcript.txt`

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
  Version: 1.2.0
  Full:    1.2.0 (4407303, 2026-06-24 22:50:41, dirty)
  Built:   2026-06-24 22:50:41
  Commit:  4407303
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
| SOAK-RAW-000001 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-316 az=16663 \| gx=24 gy=-177 gz=-88 \| t=938 | 0.109s | PASS |  |
| SOAK-READ-000002 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.016 g \| gx=+0.17 gy=-1.51 gz=-0.75 dps \| t=28.67 C | 0.110s | PASS |  |
| SOAK-ACCEL-000003 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 82 -305 16647) | 0.109s | PASS |  |
| SOAK-GYRO-000004 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.51 z=-0.77 dps (raw: 19 -173 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000005 | soak/data | `temp` | Temp | Temp: 28.67 C (raw: 940) | 0.109s | PASS |  |
| SOAK-STATUS-000006 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000007 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000008 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000009 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000010 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000011 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000012 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000013 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000014 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000015 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000016 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 6499 -> 6516 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000017 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-313 az=16640 \| gx=17 gy=-176 gz=-92 \| t=951 | 0.093s | PASS |  |
| SOAK-READ-000018 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.015 g \| gx=+0.18 gy=-1.51 gz=-0.79 dps \| t=28.72 C | 0.110s | PASS |  |
| SOAK-ACCEL-000019 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 85 -307 16629) | 0.109s | PASS |  |
| SOAK-GYRO-000020 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.54 z=-0.80 dps (raw: 19 -176 -91) | 0.109s | PASS |  |
| SOAK-TEMP-000021 | soak/data | `temp` | Temp | Temp: 28.70 C (raw: 948) | 0.109s | PASS |  |
| SOAK-STATUS-000022 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000023 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000024 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000025 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000026 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000027 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000028 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000029 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000030 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000031 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000032 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 13013 -> 13030 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000033 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-306 az=16649 \| gx=17 gy=-174 gz=-86 \| t=950 | 0.110s | PASS |  |
| SOAK-READ-000034 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.016 g \| gx=+0.18 gy=-1.51 gz=-0.78 dps \| t=28.71 C | 0.109s | PASS |  |
| SOAK-ACCEL-000035 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.018 z=1.016 g (raw: 68 -301 16652) | 0.109s | PASS |  |
| SOAK-GYRO-000036 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.51 z=-0.73 dps (raw: 21 -173 -83) | 0.110s | PASS |  |
| SOAK-TEMP-000037 | soak/data | `temp` | Temp | Temp: 28.72 C (raw: 952) | 0.110s | PASS |  |
| SOAK-STATUS-000038 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000039 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000040 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000041 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000042 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000043 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000044 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000045 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000046 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000047 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000048 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 19526 -> 19543 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000049 | soak/data | `raw` | Raw: | Raw: ax=65 ay=-311 az=16642 \| gx=16 gy=-177 gz=-88 \| t=952 | 0.110s | PASS |  |
| SOAK-READ-000050 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.018 az=+1.016 g \| gx=+0.16 gy=-1.52 gz=-0.78 dps \| t=28.75 C | 0.110s | PASS |  |
| SOAK-ACCEL-000051 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 84 -319 16634) | 0.109s | PASS |  |
| SOAK-GYRO-000052 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.54 z=-0.77 dps (raw: 19 -176 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000053 | soak/data | `temp` | Temp | Temp: 28.68 C (raw: 942) | 0.110s | PASS |  |
| SOAK-STATUS-000054 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000055 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000056 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000057 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000058 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000059 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000060 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000061 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000062 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000063 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000064 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 26041 -> 26058 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000065 | soak/data | `raw` | Raw: | Raw: ax=89 ay=-305 az=16648 \| gx=19 gy=-174 gz=-88 \| t=954 | 0.110s | PASS |  |
| SOAK-READ-000066 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.17 gy=-1.51 gz=-0.75 dps \| t=28.71 C | 0.110s | PASS |  |
| SOAK-ACCEL-000067 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.020 z=1.017 g (raw: 61 -327 16680) | 0.109s | PASS |  |
| SOAK-GYRO-000068 | soak/data | `gyro` | Gyro | Gyro: x=0.21 y=-1.54 z=-0.74 dps (raw: 24 -176 -85) | 0.109s | PASS |  |
| SOAK-TEMP-000069 | soak/data | `temp` | Temp | Temp: 28.71 C (raw: 951) | 0.110s | PASS |  |
| SOAK-STATUS-000070 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000071 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000072 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000073 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000074 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000075 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000076 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000077 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000078 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5990 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000079 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000080 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 32560 -> 32577 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000081 | soak/data | `raw` | Raw: | Raw: ax=80 ay=-318 az=16637 \| gx=23 gy=-174 gz=-89 \| t=939 | 0.109s | PASS |  |
| SOAK-READ-000082 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.56 gz=-0.78 dps \| t=28.69 C | 0.109s | PASS |  |
| SOAK-ACCEL-000083 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.018 z=1.015 g (raw: 68 -297 16637) | 0.094s | PASS |  |
| SOAK-GYRO-000084 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.78 dps (raw: 20 -176 -89) | 0.094s | PASS |  |
| SOAK-TEMP-000085 | soak/data | `temp` | Temp | Temp: 28.68 C (raw: 943) | 0.093s | PASS |  |
| SOAK-STATUS-000086 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000087 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000088 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000089 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000090 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000091 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000092 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000093 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000094 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000095 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000096 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 39074 -> 39091 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000097 | soak/data | `raw` | Raw: | Raw: ax=74 ay=-313 az=16630 \| gx=24 gy=-178 gz=-86 \| t=942 | 0.094s | PASS |  |
| SOAK-READ-000098 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.016 g \| gx=+0.16 gy=-1.52 gz=-0.79 dps \| t=28.66 C | 0.094s | PASS |  |
| SOAK-ACCEL-000099 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 76 -313 16630) | 0.094s | PASS |  |
| SOAK-GYRO-000100 | soak/data | `gyro` | Gyro | Gyro: x=0.19 y=-1.54 z=-0.76 dps (raw: 22 -176 -87) | 0.109s | PASS |  |
| SOAK-TEMP-000101 | soak/data | `temp` | Temp | Temp: 28.68 C (raw: 943) | 0.110s | PASS |  |
| SOAK-STATUS-000102 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000103 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000104 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000105 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000106 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000107 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000108 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000109 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000110 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5990 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000111 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY \| === Driver Health === \| State: READY \| Online: yes \| ... | 36.031s | PASS | prompt recovered with health resync |
| SOAK-RECOVER-000112 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 45593 -> 45610 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000113 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-318 az=16649 \| gx=18 gy=-175 gz=-87 \| t=939 | 0.110s | PASS |  |
| SOAK-READ-000114 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.015 g \| gx=+0.17 gy=-1.53 gz=-0.76 dps \| t=28.68 C | 0.109s | PASS |  |
| SOAK-ACCEL-000115 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 77 -312 16645) | 0.109s | PASS |  |
| SOAK-GYRO-000116 | soak/data | `gyro` | Gyro | Gyro: x=0.22 y=-1.56 z=-0.77 dps (raw: 25 -178 -88) | 0.110s | PASS |  |
| SOAK-TEMP-000117 | soak/data | `temp` | Temp | Temp: 28.66 C (raw: 937) | 0.110s | PASS |  |
| SOAK-STATUS-000118 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000119 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000120 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000121 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000122 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000123 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000124 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000125 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000126 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000127 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000128 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 52108 -> 52125 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000129 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-309 az=16635 \| gx=22 gy=-176 gz=-88 \| t=925 | 0.109s | PASS |  |
| SOAK-READ-000130 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.19 gy=-1.58 gz=-0.78 dps \| t=28.64 C | 0.109s | PASS |  |
| SOAK-ACCEL-000131 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 72 -306 16624) | 0.110s | PASS |  |
| SOAK-GYRO-000132 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.77 dps (raw: 20 -174 -88) | 0.093s | PASS |  |
| SOAK-TEMP-000133 | soak/data | `temp` | Temp | Temp: 28.66 C (raw: 937) | 0.094s | PASS |  |
| SOAK-STATUS-000134 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000135 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000136 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000137 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000138 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000139 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 11.031s | PASS | prompt recovered with health resync |
| SOAK-TS-000140 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000141 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000142 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5986 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000143 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000144 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 58623 -> 58640 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000145 | soak/data | `raw` | Raw: | Raw: ax=82 ay=-315 az=16656 \| gx=17 gy=-179 gz=-88 \| t=940 | 0.109s | PASS |  |
| SOAK-READ-000146 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.16 gy=-1.54 gz=-0.79 dps \| t=28.66 C | 0.110s | PASS |  |
| SOAK-ACCEL-000147 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 80 -316 16626) | 0.109s | PASS |  |
| SOAK-GYRO-000148 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.55 z=-0.76 dps (raw: 20 -177 -87) | 0.094s | PASS |  |
| SOAK-TEMP-000149 | soak/data | `temp` | Temp | Temp: 28.64 C (raw: 932) | 0.109s | PASS |  |
| SOAK-STATUS-000150 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000151 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000152 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000153 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000154 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000155 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000156 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000157 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000158 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000159 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000160 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 65139 -> 65156 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000161 | soak/data | `raw` | Raw: | Raw: ax=82 ay=-321 az=16646 \| gx=21 gy=-179 gz=-91 \| t=946 | 0.109s | PASS |  |
| SOAK-READ-000162 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.016 g \| gx=+0.22 gy=-1.51 gz=-0.76 dps \| t=28.66 C | 0.110s | PASS |  |
| SOAK-ACCEL-000163 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.016 g (raw: 69 -316 16648) | 0.109s | PASS |  |
| SOAK-GYRO-000164 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.58 z=-0.75 dps (raw: 16 -180 -86) | 0.109s | PASS |  |
| SOAK-TEMP-000165 | soak/data | `temp` | Temp | Temp: 28.62 C (raw: 928) | 0.109s | PASS |  |
| SOAK-STATUS-000166 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000167 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000168 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000169 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000170 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000171 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000172 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000173 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000174 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5984 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000175 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000176 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 71652 -> 71669 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000177 | soak/data | `raw` | Raw: | Raw: ax=82 ay=-313 az=16637 \| gx=19 gy=-173 gz=-87 \| t=937 | 0.109s | PASS |  |
| SOAK-READ-000178 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.018 az=+1.014 g \| gx=+0.19 gy=-1.53 gz=-0.80 dps \| t=28.68 C | 0.110s | PASS |  |
| SOAK-ACCEL-000179 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.016 g (raw: 81 -295 16651) | 0.093s | PASS |  |
| SOAK-GYRO-000180 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.77 dps (raw: 21 -175 -88) | 0.094s | PASS |  |
| SOAK-TEMP-000181 | soak/data | `temp` | Temp | Temp: 28.68 C (raw: 942) | 0.110s | PASS |  |
| SOAK-STATUS-000182 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000183 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000184 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000185 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000186 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000187 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000188 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000189 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000190 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000191 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000192 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 78169 -> 78186 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000193 | soak/data | `raw` | Raw: | Raw: ax=71 ay=-323 az=16628 \| gx=18 gy=-175 gz=-89 \| t=943 | 0.109s | PASS |  |
| SOAK-READ-000194 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.015 g \| gx=+0.18 gy=-1.54 gz=-0.78 dps \| t=28.69 C | 0.094s | PASS |  |
| SOAK-ACCEL-000195 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.015 g (raw: 83 -324 16634) | 0.093s | PASS |  |
| SOAK-GYRO-000196 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.52 z=-0.76 dps (raw: 15 -174 -87) | 0.094s | PASS |  |
| SOAK-TEMP-000197 | soak/data | `temp` | Temp | Temp: 28.66 C (raw: 936) | 0.094s | PASS |  |
| SOAK-STATUS-000198 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000199 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000200 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000201 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000202 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000203 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000204 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000205 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000206 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5993 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000207 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000208 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 84691 -> 84708 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000209 | soak/data | `raw` | Raw: | Raw: ax=84 ay=-315 az=16614 \| gx=18 gy=-173 gz=-93 \| t=932 | 0.109s | PASS |  |
| SOAK-READ-000210 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.54 gz=-0.76 dps \| t=28.62 C | 0.110s | PASS |  |
| SOAK-ACCEL-000211 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 78 -307 16605) | 0.109s | PASS |  |
| SOAK-GYRO-000212 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.54 z=-0.79 dps (raw: 17 -176 -90) | 0.109s | PASS |  |
| SOAK-TEMP-000213 | soak/data | `temp` | Temp | Temp: 28.63 C (raw: 930) | 0.109s | PASS |  |
| SOAK-STATUS-000214 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000215 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000216 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000217 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000218 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000219 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000220 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000221 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000222 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000223 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000224 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 91206 -> 91223 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000225 | soak/data | `raw` | Raw: | Raw: ax=66 ay=-323 az=16633 \| gx=22 gy=-174 gz=-88 \| t=916 | 0.109s | PASS |  |
| SOAK-READ-000226 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.51 gz=-0.73 dps \| t=28.58 C | 0.110s | PASS |  |
| SOAK-ACCEL-000227 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.019 z=1.014 g (raw: 96 -315 16625) | 0.109s | PASS |  |
| SOAK-GYRO-000228 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.51 z=-0.75 dps (raw: 19 -172 -86) | 0.094s | PASS |  |
| SOAK-TEMP-000229 | soak/data | `temp` | Temp | Temp: 28.61 C (raw: 925) | 0.094s | PASS |  |
| SOAK-STATUS-000230 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000231 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000232 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000233 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000234 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000235 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000236 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000237 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000238 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000239 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000240 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 97721 -> 97738 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000241 | soak/data | `raw` | Raw: | Raw: ax=69 ay=-306 az=16656 \| gx=22 gy=-173 gz=-87 \| t=909 | 0.109s | PASS |  |
| SOAK-READ-000242 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.016 g \| gx=+0.18 gy=-1.53 gz=-0.76 dps \| t=28.64 C | 0.110s | PASS |  |
| SOAK-ACCEL-000243 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.015 g (raw: 71 -315 16633) | 0.109s | PASS |  |
| SOAK-GYRO-000244 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.54 z=-0.75 dps (raw: 19 -176 -86) | 0.109s | PASS |  |
| SOAK-TEMP-000245 | soak/data | `temp` | Temp | Temp: 28.57 C (raw: 913) | 0.094s | PASS |  |
| SOAK-STATUS-000246 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000247 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000248 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000249 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000250 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000251 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000252 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000253 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000254 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000255 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000256 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 104234 -> 104251 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000257 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-313 az=16609 \| gx=17 gy=-176 gz=-87 \| t=890 | 0.094s | PASS |  |
| SOAK-READ-000258 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.55 gz=-0.78 dps \| t=28.53 C | 0.094s | PASS |  |
| SOAK-ACCEL-000259 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.019 z=1.013 g (raw: 93 -317 16599) | 0.093s | PASS |  |
| SOAK-GYRO-000260 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.77 dps (raw: 21 -174 -88) | 0.094s | PASS |  |
| SOAK-TEMP-000261 | soak/data | `temp` | Temp | Temp: 28.50 C (raw: 896) | 0.094s | PASS |  |
| SOAK-STATUS-000262 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000263 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000264 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000265 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000266 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000267 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000268 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000269 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000270 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000271 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000272 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 110752 -> 110769 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000273 | soak/data | `raw` | Raw: | Raw: ax=71 ay=-321 az=16610 \| gx=18 gy=-175 gz=-87 \| t=893 | 0.109s | PASS |  |
| SOAK-READ-000274 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.16 gy=-1.53 gz=-0.74 dps \| t=28.51 C | 0.110s | PASS |  |
| SOAK-ACCEL-000275 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 80 -318 16618) | 0.109s | PASS |  |
| SOAK-GYRO-000276 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.57 z=-0.79 dps (raw: 19 -179 -90) | 0.094s | PASS |  |
| SOAK-TEMP-000277 | soak/data | `temp` | Temp | Temp: 28.49 C (raw: 894) | 0.094s | PASS |  |
| SOAK-STATUS-000278 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000279 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000280 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000281 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000282 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000283 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000284 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000285 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000286 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5990 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000287 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000288 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 117271 -> 117288 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000289 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-321 az=16626 \| gx=18 gy=-174 gz=-87 \| t=895 | 0.109s | PASS |  |
| SOAK-READ-000290 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.54 gz=-0.77 dps \| t=28.46 C | 0.110s | PASS |  |
| SOAK-ACCEL-000291 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 72 -317 16617) | 0.109s | PASS |  |
| SOAK-GYRO-000292 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.73 dps (raw: 20 -175 -83) | 0.094s | PASS |  |
| SOAK-TEMP-000293 | soak/data | `temp` | Temp | Temp: 28.49 C (raw: 893) | 0.094s | PASS |  |
| SOAK-STATUS-000294 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000295 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000296 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000297 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000298 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000299 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000300 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000301 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000302 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5986 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000303 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000304 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 123786 -> 123803 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000305 | soak/data | `raw` | Raw: | Raw: ax=93 ay=-309 az=16622 \| gx=19 gy=-180 gz=-84 \| t=886 | 0.109s | PASS |  |
| SOAK-READ-000306 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.14 gy=-1.56 gz=-0.80 dps \| t=28.48 C | 0.094s | PASS |  |
| SOAK-ACCEL-000307 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 78 -310 16627) | 0.093s | PASS |  |
| SOAK-GYRO-000308 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.76 dps (raw: 20 -176 -87) | 0.094s | PASS |  |
| SOAK-TEMP-000309 | soak/data | `temp` | Temp | Temp: 28.48 C (raw: 891) | 0.094s | PASS |  |
| SOAK-STATUS-000310 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000311 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000312 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000313 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000314 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000315 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000316 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000317 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000318 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000319 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000320 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 130300 -> 130317 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000321 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-320 az=16605 \| gx=18 gy=-174 gz=-86 \| t=878 | 0.094s | PASS |  |
| SOAK-READ-000322 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.15 gy=-1.55 gz=-0.76 dps \| t=28.43 C | 0.093s | PASS |  |
| SOAK-ACCEL-000323 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 71 -305 16605) | 0.094s | PASS |  |
| SOAK-GYRO-000324 | soak/data | `gyro` | Gyro | Gyro: x=0.20 y=-1.58 z=-0.78 dps (raw: 23 -180 -89) | 0.094s | PASS |  |
| SOAK-TEMP-000325 | soak/data | `temp` | Temp | Temp: 28.41 C (raw: 872) | 0.094s | PASS |  |
| SOAK-STATUS-000326 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000327 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000328 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000329 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000330 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000331 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000332 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000333 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000334 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=522 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000335 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000336 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 136815 -> 136832 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000337 | soak/data | `raw` | Raw: | Raw: ax=85 ay=-311 az=16615 \| gx=19 gy=-174 gz=-91 \| t=870 | 0.110s | PASS |  |
| SOAK-READ-000338 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.014 g \| gx=+0.14 gy=-1.53 gz=-0.74 dps \| t=28.42 C | 0.109s | PASS |  |
| SOAK-ACCEL-000339 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 80 -309 16607) | 0.109s | PASS |  |
| SOAK-GYRO-000340 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.58 z=-0.75 dps (raw: 20 -180 -86) | 0.094s | PASS |  |
| SOAK-TEMP-000341 | soak/data | `temp` | Temp | Temp: 28.43 C (raw: 877) | 0.094s | PASS |  |
| SOAK-STATUS-000342 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000343 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000344 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000345 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000346 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000347 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000348 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000349 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000350 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5983 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000351 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000352 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 143327 -> 143344 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000353 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-312 az=16613 \| gx=20 gy=-178 gz=-91 \| t=873 | 0.109s | PASS |  |
| SOAK-READ-000354 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.16 gy=-1.52 gz=-0.77 dps \| t=28.41 C | 0.109s | PASS |  |
| SOAK-ACCEL-000355 | soak/data | `accel` | Accel | Accel: x=0.003 y=-0.019 z=1.013 g (raw: 54 -309 16609) | 0.110s | PASS |  |
| SOAK-GYRO-000356 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.54 z=-0.78 dps (raw: 17 -176 -89) | 0.109s | PASS |  |
| SOAK-TEMP-000357 | soak/data | `temp` | Temp | Temp: 28.41 C (raw: 873) | 0.094s | PASS |  |
| SOAK-STATUS-000358 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000359 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000360 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000361 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000362 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000363 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000364 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000365 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000366 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5992 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000367 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000368 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 149848 -> 149865 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000369 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-313 az=16612 \| gx=22 gy=-171 gz=-84 \| t=867 | 0.094s | PASS |  |
| SOAK-READ-000370 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.54 gz=-0.74 dps \| t=28.38 C | 0.110s | PASS |  |
| SOAK-ACCEL-000371 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 71 -310 16624) | 0.109s | PASS |  |
| SOAK-GYRO-000372 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.56 z=-0.77 dps (raw: 19 -178 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000373 | soak/data | `temp` | Temp | Temp: 28.39 C (raw: 869) | 0.110s | PASS |  |
| SOAK-STATUS-000374 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000375 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000376 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000377 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000378 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000379 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000380 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000381 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000382 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5991 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000383 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000384 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 156368 -> 156385 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000385 | soak/data | `raw` | Raw: | Raw: ax=82 ay=-317 az=16628 \| gx=21 gy=-172 gz=-88 \| t=859 | 0.094s | PASS |  |
| SOAK-READ-000386 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.19 gy=-1.51 gz=-0.71 dps \| t=28.40 C | 0.094s | PASS |  |
| SOAK-ACCEL-000387 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 86 -305 16639) | 0.109s | PASS |  |
| SOAK-GYRO-000388 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.50 z=-0.73 dps (raw: 19 -171 -83) | 0.110s | PASS |  |
| SOAK-TEMP-000389 | soak/data | `temp` | Temp | Temp: 28.39 C (raw: 867) | 0.109s | PASS |  |
| SOAK-STATUS-000390 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000391 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000392 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000393 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000394 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000395 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000396 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000397 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000398 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000399 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000400 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 162886 -> 162903 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000401 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-312 az=16621 \| gx=18 gy=-178 gz=-86 \| t=853 | 0.110s | PASS |  |
| SOAK-READ-000402 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.55 gz=-0.74 dps \| t=28.33 C | 0.110s | PASS |  |
| SOAK-ACCEL-000403 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 86 -324 16627) | 0.109s | PASS |  |
| SOAK-GYRO-000404 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.50 z=-0.79 dps (raw: 19 -171 -90) | 0.109s | PASS |  |
| SOAK-TEMP-000405 | soak/data | `temp` | Temp | Temp: 28.34 C (raw: 854) | 0.094s | PASS |  |
| SOAK-STATUS-000406 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000407 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000408 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000409 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000410 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000411 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000412 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000413 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000414 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000415 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000416 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 169399 -> 169416 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000417 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-329 az=16619 \| gx=20 gy=-177 gz=-88 \| t=842 | 0.094s | PASS |  |
| SOAK-READ-000418 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.013 g \| gx=+0.18 gy=-1.51 gz=-0.78 dps \| t=28.32 C | 0.094s | PASS |  |
| SOAK-ACCEL-000419 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 78 -324 16630) | 0.093s | PASS |  |
| SOAK-GYRO-000420 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.51 z=-0.76 dps (raw: 20 -173 -87) | 0.094s | PASS |  |
| SOAK-TEMP-000421 | soak/data | `temp` | Temp | Temp: 28.32 C (raw: 849) | 0.109s | PASS |  |
| SOAK-STATUS-000422 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000423 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000424 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000425 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000426 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000427 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000428 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000429 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000430 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=519 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000431 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000432 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 175916 -> 175933 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000433 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-312 az=16624 \| gx=20 gy=-174 gz=-82 \| t=857 | 0.109s | PASS |  |
| SOAK-READ-000434 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.53 gz=-0.74 dps \| t=28.35 C | 0.109s | PASS |  |
| SOAK-ACCEL-000435 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 82 -319 16612) | 0.109s | PASS |  |
| SOAK-GYRO-000436 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.77 dps (raw: 20 -175 -88) | 0.110s | PASS |  |
| SOAK-TEMP-000437 | soak/data | `temp` | Temp | Temp: 28.31 C (raw: 848) | 0.109s | PASS |  |
| SOAK-STATUS-000438 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000439 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000440 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000441 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000442 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000443 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000444 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000445 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000446 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000447 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000448 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 182431 -> 182448 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000449 | soak/data | `raw` | Raw: | Raw: ax=71 ay=-310 az=16626 \| gx=18 gy=-174 gz=-86 \| t=846 | 0.093s | PASS |  |
| SOAK-READ-000450 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.55 gz=-0.74 dps \| t=28.31 C | 0.110s | PASS |  |
| SOAK-ACCEL-000451 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 90 -319 16615) | 0.110s | PASS |  |
| SOAK-GYRO-000452 | soak/data | `gyro` | Gyro | Gyro: x=0.19 y=-1.52 z=-0.78 dps (raw: 22 -174 -89) | 0.109s | PASS |  |
| SOAK-TEMP-000453 | soak/data | `temp` | Temp | Temp: 28.32 C (raw: 850) | 0.109s | PASS |  |
| SOAK-STATUS-000454 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000455 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000456 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000457 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000458 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000459 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000460 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000461 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000462 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4731 reads=519 errors=0 accel=500 gyro=500 temp=250 health_success=5981 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000463 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000464 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 188941 -> 188958 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000465 | soak/data | `raw` | Raw: | Raw: ax=87 ay=-301 az=16614 \| gx=21 gy=-176 gz=-90 \| t=838 | 0.110s | PASS |  |
| SOAK-READ-000466 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.15 gy=-1.52 gz=-0.79 dps \| t=28.27 C | 0.109s | PASS |  |
| SOAK-ACCEL-000467 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 76 -307 16625) | 0.109s | PASS |  |
| SOAK-GYRO-000468 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.57 z=-0.75 dps (raw: 19 -179 -86) | 0.094s | PASS |  |
| SOAK-TEMP-000469 | soak/data | `temp` | Temp | Temp: 28.30 C (raw: 845) | 0.094s | PASS |  |
| SOAK-STATUS-000470 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000471 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000472 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000473 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000474 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000475 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000476 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000477 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 11.031s | PASS | prompt recovered with health resync |
| SOAK-STRESS-000478 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=522 errors=0 accel=500 gyro=500 temp=250 health_success=5987 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000479 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000480 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 195457 -> 195474 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000481 | soak/data | `raw` | Raw: | Raw: ax=70 ay=-307 az=16607 \| gx=20 gy=-174 gz=-89 \| t=828 | 0.109s | PASS |  |
| SOAK-READ-000482 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.15 gy=-1.56 gz=-0.75 dps \| t=28.27 C | 0.109s | PASS |  |
| SOAK-ACCEL-000483 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.015 g (raw: 68 -313 16633) | 0.109s | PASS |  |
| SOAK-GYRO-000484 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.54 z=-0.80 dps (raw: 18 -176 -91) | 0.110s | PASS |  |
| SOAK-TEMP-000485 | soak/data | `temp` | Temp | Temp: 28.31 C (raw: 847) | 0.109s | PASS |  |
| SOAK-STATUS-000486 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000487 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000488 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000489 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000490 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000491 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000492 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000493 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000494 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5992 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000495 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000496 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 201978 -> 201995 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000497 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-319 az=16610 \| gx=17 gy=-172 gz=-88 \| t=818 | 0.109s | PASS |  |
| SOAK-READ-000498 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.014 g \| gx=+0.20 gy=-1.53 gz=-0.75 dps \| t=28.24 C | 0.110s | PASS |  |
| SOAK-ACCEL-000499 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 69 -315 16609) | 0.094s | PASS |  |
| SOAK-GYRO-000500 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.74 dps (raw: 20 -176 -85) | 0.109s | PASS |  |
| SOAK-TEMP-000501 | soak/data | `temp` | Temp | Temp: 28.23 C (raw: 828) | 0.094s | PASS |  |
| SOAK-STATUS-000502 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000503 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000504 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000505 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000506 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000507 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000508 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000509 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000510 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5992 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000511 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000512 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 208499 -> 208516 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000513 | soak/data | `raw` | Raw: | Raw: ax=67 ay=-312 az=16607 \| gx=22 gy=-175 gz=-87 \| t=831 | 0.109s | PASS |  |
| SOAK-READ-000514 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.020 az=+1.013 g \| gx=+0.19 gy=-1.53 gz=-0.75 dps \| t=28.25 C | 0.109s | PASS |  |
| SOAK-ACCEL-000515 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.015 g (raw: 70 -313 16638) | 0.109s | PASS |  |
| SOAK-GYRO-000516 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.58 z=-0.72 dps (raw: 17 -181 -82) | 0.110s | PASS |  |
| SOAK-TEMP-000517 | soak/data | `temp` | Temp | Temp: 28.25 C (raw: 833) | 0.093s | PASS |  |
| SOAK-STATUS-000518 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000519 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000520 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000521 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000522 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000523 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000524 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000525 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000526 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5993 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000527 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000528 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 215021 -> 215038 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000529 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-307 az=16601 \| gx=18 gy=-180 gz=-91 \| t=822 | 0.109s | PASS |  |
| SOAK-READ-000530 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.013 g \| gx=+0.17 gy=-1.50 gz=-0.73 dps \| t=28.19 C | 0.125s | PASS |  |
| SOAK-ACCEL-000531 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.019 z=1.013 g (raw: 92 -315 16610) | 0.109s | PASS |  |
| SOAK-GYRO-000532 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.74 dps (raw: 20 -175 -84) | 0.109s | PASS |  |
| SOAK-TEMP-000533 | soak/data | `temp` | Temp | Temp: 28.20 C (raw: 820) | 0.094s | PASS |  |
| SOAK-STATUS-000534 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000535 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000536 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000537 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000538 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000539 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000540 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000541 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000542 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5987 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000543 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000544 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 221537 -> 221554 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000545 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-315 az=16628 \| gx=18 gy=-176 gz=-83 \| t=826 | 0.093s | PASS |  |
| SOAK-READ-000546 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.51 gz=-0.73 dps \| t=28.25 C | 0.094s | PASS |  |
| SOAK-ACCEL-000547 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 73 -309 16615) | 0.094s | PASS |  |
| SOAK-GYRO-000548 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.55 z=-0.76 dps (raw: 18 -177 -87) | 0.109s | PASS |  |
| SOAK-TEMP-000549 | soak/data | `temp` | Temp | Temp: 28.19 C (raw: 817) | 0.093s | PASS |  |
| SOAK-STATUS-000550 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000551 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000552 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000553 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000554 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000555 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000556 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000557 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000558 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000559 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000560 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 228055 -> 228072 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000561 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-315 az=16616 \| gx=17 gy=-175 gz=-85 \| t=815 | 0.094s | PASS |  |
| SOAK-READ-000562 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.011 g \| gx=+0.18 gy=-1.49 gz=-0.79 dps \| t=28.16 C | 0.109s | PASS |  |
| SOAK-ACCEL-000563 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.012 g (raw: 68 -310 16587) | 0.109s | PASS |  |
| SOAK-GYRO-000564 | soak/data | `gyro` | Gyro | Gyro: x=0.22 y=-1.52 z=-0.77 dps (raw: 25 -174 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000565 | soak/data | `temp` | Temp | Temp: 28.21 C (raw: 823) | 0.110s | PASS |  |
| SOAK-STATUS-000566 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000567 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000568 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000569 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000570 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000571 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000572 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000573 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000574 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000575 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000576 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 234573 -> 234590 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000577 | soak/data | `raw` | Raw: | Raw: ax=62 ay=-314 az=16605 \| gx=17 gy=-174 gz=-84 \| t=815 | 0.109s | PASS |  |
| SOAK-READ-000578 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.012 g \| gx=+0.18 gy=-1.52 gz=-0.78 dps \| t=28.21 C | 0.110s | PASS |  |
| SOAK-ACCEL-000579 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.019 z=1.012 g (raw: 96 -314 16596) | 0.110s | PASS |  |
| SOAK-GYRO-000580 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.56 z=-0.74 dps (raw: 19 -178 -85) | 0.109s | PASS |  |
| SOAK-TEMP-000581 | soak/data | `temp` | Temp | Temp: 28.17 C (raw: 811) | 0.094s | PASS |  |
| SOAK-STATUS-000582 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000583 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000584 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000585 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000586 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000587 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000588 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000589 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000590 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5989 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000591 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.204s | PASS |  |
| SOAK-RECOVER-000592 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 241091 -> 241108 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000593 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-301 az=16628 \| gx=21 gy=-176 gz=-90 \| t=822 | 0.094s | PASS |  |
| SOAK-READ-000594 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.17 gy=-1.56 gz=-0.75 dps \| t=28.22 C | 0.093s | PASS |  |
| SOAK-ACCEL-000595 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 75 -316 16618) | 0.110s | PASS |  |
| SOAK-GYRO-000596 | soak/data | `gyro` | Gyro | Gyro: x=0.21 y=-1.52 z=-0.74 dps (raw: 24 -174 -85) | 0.110s | PASS |  |
| SOAK-TEMP-000597 | soak/data | `temp` | Temp | Temp: 28.21 C (raw: 822) | 0.109s | PASS |  |
| SOAK-STATUS-000598 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000599 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000600 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000601 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000602 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000603 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000604 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000605 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 11.031s | PASS | prompt recovered with health resync |
| SOAK-STRESS-000606 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000607 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000608 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 247608 -> 247625 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000609 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-310 az=16617 \| gx=22 gy=-171 gz=-84 \| t=822 | 0.110s | PASS |  |
| SOAK-READ-000610 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.54 gz=-0.75 dps \| t=28.24 C | 0.110s | PASS |  |
| SOAK-ACCEL-000611 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 81 -307 16619) | 0.093s | PASS |  |
| SOAK-GYRO-000612 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.52 z=-0.78 dps (raw: 18 -174 -89) | 0.094s | PASS |  |
| SOAK-TEMP-000613 | soak/data | `temp` | Temp | Temp: 28.14 C (raw: 805) | 0.094s | PASS |  |
| SOAK-STATUS-000614 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000615 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000616 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000617 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000618 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000619 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000620 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000621 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000622 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000623 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000624 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 254126 -> 254143 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000625 | soak/data | `raw` | Raw: | Raw: ax=85 ay=-321 az=16619 \| gx=19 gy=-177 gz=-85 \| t=827 | 0.094s | PASS |  |
| SOAK-READ-000626 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.013 g \| gx=+0.17 gy=-1.59 gz=-0.77 dps \| t=28.14 C | 0.094s | PASS |  |
| SOAK-ACCEL-000627 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 79 -305 16605) | 0.109s | PASS |  |
| SOAK-GYRO-000628 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.57 z=-0.78 dps (raw: 20 -179 -89) | 0.094s | PASS |  |
| SOAK-TEMP-000629 | soak/data | `temp` | Temp | Temp: 28.21 C (raw: 821) | 0.125s | PASS |  |
| SOAK-STATUS-000630 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000631 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000632 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000633 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000634 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000635 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000636 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000637 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000638 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5990 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000639 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000640 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 260645 -> 260662 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000641 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-314 az=16606 \| gx=18 gy=-173 gz=-83 \| t=821 | 0.110s | PASS |  |
| SOAK-READ-000642 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.19 gy=-1.51 gz=-0.76 dps \| t=28.18 C | 0.110s | PASS |  |
| SOAK-ACCEL-000643 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.018 z=1.013 g (raw: 61 -302 16606) | 0.109s | PASS |  |
| SOAK-GYRO-000644 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.51 z=-0.76 dps (raw: 19 -172 -87) | 0.109s | PASS |  |
| SOAK-TEMP-000645 | soak/data | `temp` | Temp | Temp: 28.17 C (raw: 812) | 0.110s | PASS |  |
| SOAK-STATUS-000646 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000647 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000648 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000649 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000650 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000651 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000652 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000653 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000654 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5984 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000655 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000656 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 267158 -> 267175 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000657 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-308 az=16617 \| gx=19 gy=-171 gz=-86 \| t=827 | 0.093s | PASS |  |
| SOAK-READ-000658 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.51 gz=-0.74 dps \| t=28.20 C | 0.094s | PASS |  |
| SOAK-ACCEL-000659 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.012 g (raw: 81 -315 16598) | 0.094s | PASS |  |
| SOAK-GYRO-000660 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.52 z=-0.82 dps (raw: 18 -174 -94) | 0.109s | PASS |  |
| SOAK-TEMP-000661 | soak/data | `temp` | Temp | Temp: 28.20 C (raw: 820) | 0.109s | PASS |  |
| SOAK-STATUS-000662 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000663 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000664 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000665 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000666 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000667 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000668 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000669 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000670 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5984 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000671 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000672 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 273671 -> 273688 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000673 | soak/data | `raw` | Raw: | Raw: ax=69 ay=-316 az=16613 \| gx=21 gy=-180 gz=-87 \| t=820 | 0.110s | PASS |  |
| SOAK-READ-000674 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.013 g \| gx=+0.19 gy=-1.51 gz=-0.75 dps \| t=28.21 C | 0.109s | PASS |  |
| SOAK-ACCEL-000675 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.013 g (raw: 87 -325 16609) | 0.109s | PASS |  |
| SOAK-GYRO-000676 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.76 dps (raw: 21 -176 -87) | 0.109s | PASS |  |
| SOAK-TEMP-000677 | soak/data | `temp` | Temp | Temp: 28.23 C (raw: 827) | 0.094s | PASS |  |
| SOAK-STATUS-000678 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000679 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000680 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000681 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000682 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000683 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000684 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000685 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000686 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5991 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000687 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000688 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 280191 -> 280208 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000689 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-319 az=16621 \| gx=17 gy=-179 gz=-86 \| t=825 | 0.093s | PASS |  |
| SOAK-READ-000690 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.56 gz=-0.74 dps \| t=28.22 C | 0.094s | PASS |  |
| SOAK-ACCEL-000691 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 82 -315 16609) | 0.094s | PASS |  |
| SOAK-GYRO-000692 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.56 z=-0.74 dps (raw: 21 -178 -85) | 0.094s | PASS |  |
| SOAK-TEMP-000693 | soak/data | `temp` | Temp | Temp: 28.23 C (raw: 826) | 0.109s | PASS |  |
| SOAK-STATUS-000694 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000695 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000696 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000697 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000698 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000699 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000700 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000701 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000702 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5993 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000703 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000704 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 286713 -> 286730 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000705 | soak/data | `raw` | Raw: | Raw: ax=90 ay=-300 az=16605 \| gx=22 gy=-173 gz=-86 \| t=814 | 0.109s | PASS |  |
| SOAK-READ-000706 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.014 g \| gx=+0.17 gy=-1.53 gz=-0.74 dps \| t=28.23 C | 0.110s | PASS |  |
| SOAK-ACCEL-000707 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 77 -325 16616) | 0.109s | PASS |  |
| SOAK-GYRO-000708 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.74 dps (raw: 21 -175 -84) | 0.109s | PASS |  |
| SOAK-TEMP-000709 | soak/data | `temp` | Temp | Temp: 28.16 C (raw: 810) | 0.094s | PASS |  |
| SOAK-STATUS-000710 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000711 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000712 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000713 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000714 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000715 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000716 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000717 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000718 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000719 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000720 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 293227 -> 293244 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000721 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-304 az=16630 \| gx=23 gy=-176 gz=-87 \| t=815 | 0.110s | PASS |  |
| SOAK-READ-000722 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.56 gz=-0.69 dps \| t=28.18 C | 0.109s | PASS |  |
| SOAK-ACCEL-000723 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 71 -306 16610) | 0.109s | PASS |  |
| SOAK-GYRO-000724 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.51 z=-0.75 dps (raw: 21 -173 -86) | 0.110s | PASS |  |
| SOAK-TEMP-000725 | soak/data | `temp` | Temp | Temp: 28.20 C (raw: 818) | 0.110s | PASS |  |
| SOAK-STATUS-000726 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000727 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000728 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000729 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000730 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000731 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000732 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000733 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 10.969s | PASS | prompt recovered with health resync |
| SOAK-STRESS-000734 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5983 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000735 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000736 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 299739 -> 299756 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000737 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-326 az=16611 \| gx=24 gy=-180 gz=-85 \| t=799 | 0.109s | PASS |  |
| SOAK-READ-000738 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.012 g \| gx=+0.14 gy=-1.56 gz=-0.74 dps \| t=28.16 C | 0.110s | PASS |  |
| SOAK-ACCEL-000739 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.013 g (raw: 77 -320 16606) | 0.109s | PASS |  |
| SOAK-GYRO-000740 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.51 z=-0.77 dps (raw: 16 -172 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000741 | soak/data | `temp` | Temp | Temp: 28.16 C (raw: 809) | 0.109s | PASS |  |
| SOAK-STATUS-000742 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000743 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000744 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000745 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000746 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000747 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000748 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000749 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000750 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4743 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5994 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000751 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000752 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 306262 -> 306279 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000753 | soak/data | `raw` | Raw: | Raw: ax=89 ay=-307 az=16627 \| gx=20 gy=-178 gz=-82 \| t=819 | 0.109s | PASS |  |
| SOAK-READ-000754 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.020 az=+1.014 g \| gx=+0.16 gy=-1.52 gz=-0.79 dps \| t=28.18 C | 0.109s | PASS |  |
| SOAK-ACCEL-000755 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 72 -319 16607) | 0.110s | PASS |  |
| SOAK-GYRO-000756 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.56 z=-0.74 dps (raw: 19 -178 -85) | 0.094s | PASS |  |
| SOAK-TEMP-000757 | soak/data | `temp` | Temp | Temp: 28.17 C (raw: 812) | 0.093s | PASS |  |
| SOAK-STATUS-000758 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000759 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000760 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000761 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000762 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000763 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000764 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000765 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000766 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000767 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000768 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 312779 -> 312796 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000769 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-315 az=16599 \| gx=21 gy=-175 gz=-86 \| t=805 | 0.093s | PASS |  |
| SOAK-READ-000770 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.012 g \| gx=+0.18 gy=-1.55 gz=-0.73 dps \| t=28.12 C | 0.094s | PASS |  |
| SOAK-ACCEL-000771 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 88 -314 16608) | 0.094s | PASS |  |
| SOAK-GYRO-000772 | soak/data | `gyro` | Gyro | Gyro: x=0.19 y=-1.53 z=-0.74 dps (raw: 22 -175 -85) | 0.094s | PASS |  |
| SOAK-TEMP-000773 | soak/data | `temp` | Temp | Temp: 28.16 C (raw: 809) | 0.093s | PASS |  |
| SOAK-STATUS-000774 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000775 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000776 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000777 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000778 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000779 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000780 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000781 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000782 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5990 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000783 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000784 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 319298 -> 319315 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000785 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-318 az=16609 \| gx=21 gy=-175 gz=-88 \| t=796 | 0.109s | PASS |  |
| SOAK-READ-000786 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.014 g \| gx=+0.17 gy=-1.54 gz=-0.70 dps \| t=28.16 C | 0.109s | PASS |  |
| SOAK-ACCEL-000787 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.019 z=1.013 g (raw: 92 -317 16604) | 0.094s | PASS |  |
| SOAK-GYRO-000788 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.55 z=-0.74 dps (raw: 18 -177 -84) | 0.094s | PASS |  |
| SOAK-TEMP-000789 | soak/data | `temp` | Temp | Temp: 28.11 C (raw: 797) | 0.093s | PASS |  |
| SOAK-STATUS-000790 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000791 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000792 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000793 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000794 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000795 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000796 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000797 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000798 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5991 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000799 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000800 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 325818 -> 325835 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000801 | soak/data | `raw` | Raw: | Raw: ax=85 ay=-313 az=16603 \| gx=17 gy=-175 gz=-85 \| t=809 | 0.109s | PASS |  |
| SOAK-READ-000802 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.54 gz=-0.74 dps \| t=28.15 C | 0.109s | PASS |  |
| SOAK-ACCEL-000803 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.013 g (raw: 74 -323 16613) | 0.094s | PASS |  |
| SOAK-GYRO-000804 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.55 z=-0.73 dps (raw: 21 -177 -83) | 0.094s | PASS |  |
| SOAK-TEMP-000805 | soak/data | `temp` | Temp | Temp: 28.14 C (raw: 805) | 0.093s | PASS |  |
| SOAK-STATUS-000806 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000807 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000808 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000809 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000810 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000811 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000812 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000813 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000814 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000815 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000816 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 332335 -> 332352 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000817 | soak/data | `raw` | Raw: | Raw: ax=66 ay=-313 az=16613 \| gx=18 gy=-176 gz=-88 \| t=797 | 0.093s | PASS |  |
| SOAK-READ-000818 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.56 gz=-0.72 dps \| t=28.11 C | 0.110s | PASS |  |
| SOAK-ACCEL-000819 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.012 g (raw: 75 -322 16593) | 0.110s | PASS |  |
| SOAK-GYRO-000820 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.76 dps (raw: 21 -174 -87) | 0.109s | PASS |  |
| SOAK-TEMP-000821 | soak/data | `temp` | Temp | Temp: 28.09 C (raw: 791) | 0.109s | PASS |  |
| SOAK-STATUS-000822 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000823 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000824 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000825 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000826 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000827 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000828 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000829 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000830 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000831 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000832 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 338852 -> 338869 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000833 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-317 az=16620 \| gx=21 gy=-170 gz=-85 \| t=796 | 0.093s | PASS |  |
| SOAK-READ-000834 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.52 gz=-0.69 dps \| t=28.13 C | 0.094s | PASS |  |
| SOAK-ACCEL-000835 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.015 g (raw: 62 -307 16633) | 0.094s | PASS |  |
| SOAK-GYRO-000836 | soak/data | `gyro` | Gyro | Gyro: x=0.21 y=-1.51 z=-0.72 dps (raw: 24 -172 -82) | 0.094s | PASS |  |
| SOAK-TEMP-000837 | soak/data | `temp` | Temp | Temp: 28.11 C (raw: 797) | 0.093s | PASS |  |
| SOAK-STATUS-000838 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000839 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000840 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000841 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000842 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000843 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000844 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000845 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000846 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5990 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000847 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000848 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 345371 -> 345388 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000849 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-316 az=16642 \| gx=17 gy=-173 gz=-84 \| t=791 | 0.109s | PASS |  |
| SOAK-READ-000850 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.22 gy=-1.54 gz=-0.74 dps \| t=28.09 C | 0.094s | PASS |  |
| SOAK-ACCEL-000851 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 73 -316 16623) | 0.110s | PASS |  |
| SOAK-GYRO-000852 | soak/data | `gyro` | Gyro | Gyro: x=0.21 y=-1.54 z=-0.75 dps (raw: 24 -176 -86) | 0.109s | PASS |  |
| SOAK-TEMP-000853 | soak/data | `temp` | Temp | Temp: 28.04 C (raw: 778) | 0.109s | PASS |  |
| SOAK-STATUS-000854 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000855 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000856 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000857 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000858 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000859 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000860 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000861 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 10.969s | PASS | prompt recovered with health resync |
| SOAK-STRESS-000862 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5986 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000863 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000864 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 351886 -> 351903 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000865 | soak/data | `raw` | Raw: | Raw: ax=74 ay=-318 az=16613 \| gx=21 gy=-175 gz=-87 \| t=779 | 0.109s | PASS |  |
| SOAK-READ-000866 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.006 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.52 gz=-0.75 dps \| t=28.07 C | 0.109s | PASS |  |
| SOAK-ACCEL-000867 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.012 g (raw: 85 -318 16586) | 0.109s | PASS |  |
| SOAK-GYRO-000868 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.77 dps (raw: 21 -176 -88) | 0.094s | PASS |  |
| SOAK-TEMP-000869 | soak/data | `temp` | Temp | Temp: 28.05 C (raw: 781) | 0.093s | PASS |  |
| SOAK-STATUS-000870 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000871 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000872 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000873 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000874 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000875 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000876 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000877 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000878 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000879 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000880 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 358402 -> 358419 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000881 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-303 az=16603 \| gx=18 gy=-176 gz=-91 \| t=795 | 0.094s | PASS |  |
| SOAK-READ-000882 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.20 gy=-1.51 gz=-0.74 dps \| t=28.03 C | 0.094s | PASS |  |
| SOAK-ACCEL-000883 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 74 -308 16626) | 0.093s | PASS |  |
| SOAK-GYRO-000884 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.75 dps (raw: 21 -175 -86) | 0.094s | PASS |  |
| SOAK-TEMP-000885 | soak/data | `temp` | Temp | Temp: 28.07 C (raw: 785) | 0.094s | PASS |  |
| SOAK-STATUS-000886 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000887 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000888 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000889 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000890 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000891 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000892 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000893 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000894 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000895 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000896 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 364915 -> 364932 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000897 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-301 az=16612 \| gx=23 gy=-176 gz=-85 \| t=784 | 0.109s | PASS |  |
| SOAK-READ-000898 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.51 gz=-0.70 dps \| t=28.08 C | 0.109s | PASS |  |
| SOAK-ACCEL-000899 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 71 -308 16617) | 0.110s | PASS |  |
| SOAK-GYRO-000900 | soak/data | `gyro` | Gyro | Gyro: x=0.20 y=-1.55 z=-0.74 dps (raw: 23 -177 -85) | 0.109s | PASS |  |
| SOAK-TEMP-000901 | soak/data | `temp` | Temp | Temp: 28.02 C (raw: 774) | 0.094s | PASS |  |
| SOAK-STATUS-000902 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000903 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000904 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000905 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000906 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000907 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000908 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000909 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000910 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5987 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000911 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000912 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 371431 -> 371448 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000913 | soak/data | `raw` | Raw: | Raw: ax=80 ay=-316 az=16602 \| gx=21 gy=-176 gz=-87 \| t=775 | 0.109s | PASS |  |
| SOAK-READ-000914 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.19 gy=-1.56 gz=-0.79 dps \| t=28.05 C | 0.094s | PASS |  |
| SOAK-ACCEL-000915 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.013 g (raw: 77 -323 16604) | 0.110s | PASS |  |
| SOAK-GYRO-000916 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.55 z=-0.78 dps (raw: 20 -177 -89) | 0.093s | PASS |  |
| SOAK-TEMP-000917 | soak/data | `temp` | Temp | Temp: 28.03 C (raw: 775) | 0.094s | PASS |  |
| SOAK-STATUS-000918 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000919 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000920 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000921 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000922 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000923 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000924 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000925 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000926 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5991 health_fail=0 state=READY | 4.797s | PASS |  |
| SOAK-MIX-000927 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000928 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 377951 -> 377968 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000929 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-320 az=16605 \| gx=22 gy=-172 gz=-83 \| t=783 | 0.109s | PASS |  |
| SOAK-READ-000930 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.54 gz=-0.72 dps \| t=28.01 C | 0.094s | PASS |  |
| SOAK-ACCEL-000931 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 68 -318 16618) | 0.093s | PASS |  |
| SOAK-GYRO-000932 | soak/data | `gyro` | Gyro | Gyro: x=0.23 y=-1.51 z=-0.74 dps (raw: 26 -173 -85) | 0.094s | PASS |  |
| SOAK-TEMP-000933 | soak/data | `temp` | Temp | Temp: 28.04 C (raw: 777) | 0.094s | PASS |  |
| SOAK-STATUS-000934 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000935 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000936 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000937 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000938 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000939 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000940 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000941 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000942 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=519 errors=0 accel=500 gyro=500 temp=250 health_success=5984 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000943 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000944 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 384464 -> 384481 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000945 | soak/data | `raw` | Raw: | Raw: ax=72 ay=-310 az=16609 \| gx=20 gy=-177 gz=-88 \| t=779 | 0.093s | PASS |  |
| SOAK-READ-000946 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.56 gz=-0.67 dps \| t=28.07 C | 0.110s | PASS |  |
| SOAK-ACCEL-000947 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.014 g (raw: 86 -302 16626) | 0.109s | PASS |  |
| SOAK-GYRO-000948 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.52 z=-0.79 dps (raw: 18 -174 -90) | 0.109s | PASS |  |
| SOAK-TEMP-000949 | soak/data | `temp` | Temp | Temp: 28.04 C (raw: 777) | 0.109s | PASS |  |
| SOAK-STATUS-000950 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000951 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000952 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000953 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000954 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000955 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000956 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000957 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000958 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5991 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000959 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000960 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 390984 -> 391001 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000961 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-315 az=16611 \| gx=25 gy=-174 gz=-85 \| t=772 | 0.093s | PASS |  |
| SOAK-READ-000962 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.58 gz=-0.72 dps \| t=28.07 C | 0.110s | PASS |  |
| SOAK-ACCEL-000963 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 88 -309 16613) | 0.109s | PASS |  |
| SOAK-GYRO-000964 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.52 z=-0.77 dps (raw: 18 -174 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000965 | soak/data | `temp` | Temp | Temp: 28.05 C (raw: 781) | 0.109s | PASS |  |
| SOAK-STATUS-000966 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000967 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000968 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000969 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000970 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000971 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000972 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000973 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000974 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5990 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000975 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000976 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 397503 -> 397520 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000977 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-304 az=16619 \| gx=19 gy=-182 gz=-80 \| t=780 | 0.109s | PASS |  |
| SOAK-READ-000978 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.54 gz=-0.76 dps \| t=27.99 C | 0.094s | PASS |  |
| SOAK-ACCEL-000979 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.016 g (raw: 83 -321 16649) | 0.094s | PASS |  |
| SOAK-GYRO-000980 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.54 z=-0.73 dps (raw: 19 -176 -83) | 0.094s | PASS |  |
| SOAK-TEMP-000981 | soak/data | `temp` | Temp | Temp: 28.00 C (raw: 769) | 0.109s | PASS |  |
| SOAK-STATUS-000982 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000983 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000984 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000985 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000986 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000987 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000988 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000989 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 10.875s | PASS | prompt recovered with health resync |
| SOAK-STRESS-000990 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4731 reads=517 errors=0 accel=500 gyro=500 temp=250 health_success=5981 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000991 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000992 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 404013 -> 404030 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000993 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-315 az=16596 \| gx=19 gy=-176 gz=-86 \| t=764 | 0.110s | PASS |  |
| SOAK-READ-000994 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.16 gy=-1.56 gz=-0.74 dps \| t=27.99 C | 0.109s | PASS |  |
| SOAK-ACCEL-000995 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.012 g (raw: 84 -321 16597) | 0.109s | PASS |  |
| SOAK-GYRO-000996 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.76 dps (raw: 21 -175 -87) | 0.109s | PASS |  |
| SOAK-TEMP-000997 | soak/data | `temp` | Temp | Temp: 27.95 C (raw: 756) | 0.094s | PASS |  |
| SOAK-STATUS-000998 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000999 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001000 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001001 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001002 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001003 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001004 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001005 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001006 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5991 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001007 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001008 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 410533 -> 410550 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-001009 | soak/data | `raw` | Raw: | Raw: ax=64 ay=-300 az=16625 \| gx=18 gy=-175 gz=-87 \| t=776 | 0.094s | PASS |  |
| SOAK-READ-001010 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.16 gy=-1.55 gz=-0.74 dps \| t=27.99 C | 0.094s | PASS |  |
| SOAK-ACCEL-001011 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 74 -331 16617) | 0.109s | PASS |  |
| SOAK-GYRO-001012 | soak/data | `gyro` | Gyro | Gyro: x=0.20 y=-1.53 z=-0.74 dps (raw: 23 -175 -84) | 0.109s | PASS |  |
| SOAK-TEMP-001013 | soak/data | `temp` | Temp | Temp: 28.00 C (raw: 768) | 0.110s | PASS |  |
| SOAK-STATUS-001014 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001015 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001016 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001017 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001018 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001019 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001020 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001021 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-001022 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-001023 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001024 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 417051 -> 417068 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001025 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-307 az=16611 \| gx=20 gy=-173 gz=-89 \| t=759 | 0.093s | PASS |  |
| SOAK-READ-001026 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.16 gy=-1.57 gz=-0.73 dps \| t=27.96 C | 0.109s | PASS |  |
| SOAK-ACCEL-001027 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 65 -315 16620) | 0.094s | PASS |  |
| SOAK-GYRO-001028 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.54 z=-0.77 dps (raw: 18 -176 -88) | 0.094s | PASS |  |
| SOAK-TEMP-001029 | soak/data | `temp` | Temp | Temp: 27.99 C (raw: 765) | 0.093s | PASS |  |
| SOAK-STATUS-001030 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001031 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001032 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001033 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001034 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001035 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001036 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001037 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001038 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5984 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001039 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001040 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 423564 -> 423581 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001041 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-316 az=16620 \| gx=21 gy=-173 gz=-83 \| t=778 | 0.109s | PASS |  |
| SOAK-READ-001042 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.15 gy=-1.53 gz=-0.72 dps \| t=28.02 C | 0.109s | PASS |  |
| SOAK-ACCEL-001043 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 74 -317 16619) | 0.109s | PASS |  |
| SOAK-GYRO-001044 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.58 z=-0.77 dps (raw: 16 -181 -88) | 0.094s | PASS |  |
| SOAK-TEMP-001045 | soak/data | `temp` | Temp | Temp: 28.02 C (raw: 772) | 0.093s | PASS |  |
| SOAK-STATUS-001046 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001047 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001048 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001049 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001050 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001051 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001052 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001053 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001054 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=517 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-001055 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001056 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 430081 -> 430098 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001057 | soak/data | `raw` | Raw: | Raw: ax=84 ay=-310 az=16618 \| gx=18 gy=-179 gz=-80 \| t=768 | 0.094s | PASS |  |
| SOAK-READ-001058 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.014 g \| gx=+0.18 gy=-1.56 gz=-0.73 dps \| t=27.98 C | 0.094s | PASS |  |
| SOAK-ACCEL-001059 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 82 -315 16608) | 0.109s | PASS |  |
| SOAK-GYRO-001060 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.51 z=-0.75 dps (raw: 20 -172 -86) | 0.110s | PASS |  |
| SOAK-TEMP-001061 | soak/data | `temp` | Temp | Temp: 27.96 C (raw: 758) | 0.109s | PASS |  |
| SOAK-STATUS-001062 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001063 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001064 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-001065 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001066 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001067 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001068 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001069 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-001070 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001071 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001072 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 436594 -> 436611 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001073 | soak/data | `raw` | Raw: | Raw: ax=66 ay=-313 az=16585 \| gx=19 gy=-177 gz=-85 \| t=764 | 0.110s | PASS |  |
| SOAK-READ-001074 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.53 gz=-0.73 dps \| t=27.99 C | 0.110s | PASS |  |
| SOAK-ACCEL-001075 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.012 g (raw: 82 -306 16589) | 0.109s | PASS |  |
| SOAK-GYRO-001076 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.52 z=-0.73 dps (raw: 17 -174 -83) | 0.109s | PASS |  |
| SOAK-TEMP-001077 | soak/data | `temp` | Temp | Temp: 28.01 C (raw: 770) | 0.110s | PASS |  |
| SOAK-STATUS-001078 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001079 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001080 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001081 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001082 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001083 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-001084 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001085 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001086 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.797s | PASS |  |
| SOAK-MIX-001087 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001088 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 443110 -> 443127 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001089 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-300 az=16627 \| gx=22 gy=-175 gz=-84 \| t=761 | 0.110s | PASS |  |
| SOAK-READ-001090 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.54 gz=-0.78 dps \| t=27.96 C | 0.109s | PASS |  |
| SOAK-ACCEL-001091 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.013 g (raw: 74 -330 16612) | 0.109s | PASS |  |
| SOAK-GYRO-001092 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.53 z=-0.74 dps (raw: 16 -175 -84) | 0.094s | PASS |  |
| SOAK-TEMP-001093 | soak/data | `temp` | Temp | Temp: 27.96 C (raw: 758) | 0.094s | PASS |  |
| SOAK-STATUS-001094 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-001095 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001096 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001097 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001098 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001099 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001100 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001101 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001102 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=522 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.766s | PASS |  |
