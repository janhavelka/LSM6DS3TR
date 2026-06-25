# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-24T22:51:18.782070+02:00
- Ended: 2026-06-24T23:22:09.936627+02:00
- Suite: soak
- Port: COM26
- Baud: 115200
- Result counts: PASS=2095, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=2095, min=0.093s, mean=0.832s, max=126.032s
- Transcript: `docs/reports/artifacts/hil-COM26-20260624-soak-30m-quiet-resync-transcript.txt`

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
| SOAK-RAW-000001 | soak/data | `raw` | Raw: | Raw: ax=71 ay=-294 az=16647 \| gx=22 gy=-173 gz=-91 \| t=968 | 0.109s | PASS |  |
| SOAK-READ-000002 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.016 g \| gx=+0.19 gy=-1.55 gz=-0.77 dps \| t=28.79 C | 0.093s | PASS |  |
| SOAK-ACCEL-000003 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.015 g (raw: 70 -305 16639) | 0.110s | PASS |  |
| SOAK-GYRO-000004 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.52 z=-0.81 dps (raw: 18 -174 -92) | 0.109s | PASS |  |
| SOAK-TEMP-000005 | soak/data | `temp` | Temp | Temp: 28.78 C (raw: 968) | 0.109s | PASS |  |
| SOAK-STATUS-000006 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000007 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000008 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000009 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000010 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000011 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000012 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000013 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000014 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5993 health_fail=0 state=READY | 4.782s | PASS |  |
| SOAK-MIX-000015 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000016 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 6505 -> 6522 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000017 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-310 az=16658 \| gx=19 gy=-173 gz=-89 \| t=967 | 0.094s | PASS |  |
| SOAK-READ-000018 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.015 g \| gx=+0.15 gy=-1.52 gz=-0.79 dps \| t=28.82 C | 0.109s | PASS |  |
| SOAK-ACCEL-000019 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.015 g (raw: 82 -298 16639) | 0.110s | PASS |  |
| SOAK-GYRO-000020 | soak/data | `gyro` | Gyro | Gyro: x=0.19 y=-1.54 z=-0.77 dps (raw: 22 -176 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000021 | soak/data | `temp` | Temp | Temp: 28.79 C (raw: 969) | 0.094s | PASS |  |
| SOAK-STATUS-000022 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000023 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000024 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000025 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000026 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000027 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000028 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000029 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000030 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5984 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000031 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000032 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 13018 -> 13035 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000033 | soak/data | `raw` | Raw: | Raw: ax=74 ay=-317 az=16626 \| gx=14 gy=-176 gz=-88 \| t=956 | 0.094s | PASS |  |
| SOAK-READ-000034 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.20 gy=-1.55 gz=-0.81 dps \| t=28.76 C | 0.109s | PASS |  |
| SOAK-ACCEL-000035 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 74 -320 16628) | 0.110s | PASS |  |
| SOAK-GYRO-000036 | soak/data | `gyro` | Gyro | Gyro: x=0.21 y=-1.51 z=-0.77 dps (raw: 24 -172 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000037 | soak/data | `temp` | Temp | Temp: 28.78 C (raw: 967) | 0.094s | PASS |  |
| SOAK-STATUS-000038 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000039 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000040 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000041 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000042 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000043 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000044 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.125s | PASS |  |
| SOAK-STEPS-000045 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000046 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000047 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000048 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 19534 -> 19551 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000049 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-304 az=16653 \| gx=18 gy=-171 gz=-83 \| t=973 | 0.109s | PASS |  |
| SOAK-READ-000050 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.006 ay=-0.019 az=+1.015 g \| gx=+0.21 gy=-1.53 gz=-0.77 dps \| t=28.84 C | 0.094s | PASS |  |
| SOAK-ACCEL-000051 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.015 g (raw: 66 -316 16632) | 0.093s | PASS |  |
| SOAK-GYRO-000052 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.54 z=-0.79 dps (raw: 18 -176 -90) | 0.110s | PASS |  |
| SOAK-TEMP-000053 | soak/data | `temp` | Temp | Temp: 28.79 C (raw: 971) | 0.110s | PASS |  |
| SOAK-STATUS-000054 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000055 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000056 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000057 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000058 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000059 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000060 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000061 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000062 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000063 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000064 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 26048 -> 26065 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000065 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-310 az=16649 \| gx=23 gy=-176 gz=-82 \| t=976 | 0.110s | PASS |  |
| SOAK-READ-000066 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.016 g \| gx=+0.15 gy=-1.58 gz=-0.79 dps \| t=28.79 C | 0.109s | PASS |  |
| SOAK-ACCEL-000067 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.018 z=1.014 g (raw: 72 -303 16629) | 0.109s | PASS |  |
| SOAK-GYRO-000068 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.56 z=-0.81 dps (raw: 21 -178 -93) | 0.110s | PASS |  |
| SOAK-TEMP-000069 | soak/data | `temp` | Temp | Temp: 28.81 C (raw: 975) | 0.110s | PASS |  |
| SOAK-STATUS-000070 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000071 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000072 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000073 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000074 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000075 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000076 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000077 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000078 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000079 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000080 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 32565 -> 32582 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000081 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-301 az=16642 \| gx=19 gy=-174 gz=-89 \| t=975 | 0.109s | PASS |  |
| SOAK-READ-000082 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.15 gy=-1.54 gz=-0.74 dps \| t=28.82 C | 0.109s | PASS |  |
| SOAK-ACCEL-000083 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.016 g (raw: 77 -314 16649) | 0.110s | PASS |  |
| SOAK-GYRO-000084 | soak/data | `gyro` | Gyro | Gyro: x=0.20 y=-1.55 z=-0.78 dps (raw: 23 -177 -89) | 0.109s | PASS |  |
| SOAK-TEMP-000085 | soak/data | `temp` | Temp | Temp: 28.82 C (raw: 978) | 0.125s | PASS |  |
| SOAK-STATUS-000086 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000087 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000088 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000089 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000090 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000091 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000092 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000093 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000094 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5983 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000095 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000096 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 39077 -> 39094 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000097 | soak/data | `raw` | Raw: | Raw: ax=61 ay=-314 az=16651 \| gx=20 gy=-175 gz=-92 \| t=971 | 0.094s | PASS |  |
| SOAK-READ-000098 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.015 g \| gx=+0.17 gy=-1.51 gz=-0.75 dps \| t=28.80 C | 0.110s | PASS |  |
| SOAK-ACCEL-000099 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 82 -304 16639) | 0.109s | PASS |  |
| SOAK-GYRO-000100 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.58 z=-0.81 dps (raw: 19 -180 -92) | 0.109s | PASS |  |
| SOAK-TEMP-000101 | soak/data | `temp` | Temp | Temp: 28.79 C (raw: 969) | 0.110s | PASS |  |
| SOAK-STATUS-000102 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000103 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000104 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000105 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000106 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000107 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000108 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000109 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000110 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000111 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY \| === Driver Health === \| State: READY \| Online: yes \| ... | 126.032s | PASS | prompt recovered with health resync |
| SOAK-RECOVER-000112 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 45590 -> 45607 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000113 | soak/data | `raw` | Raw: | Raw: ax=72 ay=-312 az=16622 \| gx=18 gy=-174 gz=-82 \| t=958 | 0.093s | PASS |  |
| SOAK-READ-000114 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.52 gz=-0.75 dps \| t=28.75 C | 0.094s | PASS |  |
| SOAK-ACCEL-000115 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 90 -306 16617) | 0.094s | PASS |  |
| SOAK-GYRO-000116 | soak/data | `gyro` | Gyro | Gyro: x=0.21 y=-1.56 z=-0.77 dps (raw: 24 -178 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000117 | soak/data | `temp` | Temp | Temp: 28.76 C (raw: 963) | 0.109s | PASS |  |
| SOAK-STATUS-000118 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000119 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000120 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000121 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000122 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000123 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000124 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000125 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000126 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.735s | PASS |  |
| SOAK-MIX-000127 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000128 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 52106 -> 52123 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000129 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-313 az=16615 \| gx=21 gy=-179 gz=-87 \| t=953 | 0.110s | PASS |  |
| SOAK-READ-000130 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.020 az=+1.015 g \| gx=+0.17 gy=-1.53 gz=-0.81 dps \| t=28.70 C | 0.109s | PASS |  |
| SOAK-ACCEL-000131 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.019 z=1.014 g (raw: 92 -313 16623) | 0.109s | PASS |  |
| SOAK-GYRO-000132 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.51 z=-0.79 dps (raw: 17 -172 -90) | 0.094s | PASS |  |
| SOAK-TEMP-000133 | soak/data | `temp` | Temp | Temp: 28.73 C (raw: 954) | 0.094s | PASS |  |
| SOAK-STATUS-000134 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000135 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000136 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000137 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000138 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000139 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000140 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000141 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000142 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000143 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000144 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 58621 -> 58638 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000145 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-319 az=16645 \| gx=23 gy=-178 gz=-89 \| t=952 | 0.094s | PASS |  |
| SOAK-READ-000146 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.015 g \| gx=+0.17 gy=-1.52 gz=-0.78 dps \| t=28.72 C | 0.094s | PASS |  |
| SOAK-ACCEL-000147 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 87 -313 16630) | 0.109s | PASS |  |
| SOAK-GYRO-000148 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.79 dps (raw: 20 -176 -90) | 0.109s | PASS |  |
| SOAK-TEMP-000149 | soak/data | `temp` | Temp | Temp: 28.70 C (raw: 947) | 0.110s | PASS |  |
| SOAK-STATUS-000150 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000151 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000152 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000153 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000154 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000155 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 21.000s | PASS | prompt recovered with health resync |
| SOAK-TS-000156 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000157 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000158 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5991 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000159 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000160 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 65141 -> 65158 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000161 | soak/data | `raw` | Raw: | Raw: ax=66 ay=-314 az=16637 \| gx=19 gy=-174 gz=-87 \| t=944 | 0.109s | PASS |  |
| SOAK-READ-000162 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.49 gz=-0.74 dps \| t=28.70 C | 0.110s | PASS |  |
| SOAK-ACCEL-000163 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 82 -309 16636) | 0.110s | PASS |  |
| SOAK-GYRO-000164 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.78 dps (raw: 20 -176 -89) | 0.109s | PASS |  |
| SOAK-TEMP-000165 | soak/data | `temp` | Temp | Temp: 28.73 C (raw: 954) | 0.109s | PASS |  |
| SOAK-STATUS-000166 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000167 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000168 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000169 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000170 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000171 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000172 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000173 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000174 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5993 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000175 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000176 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 71663 -> 71680 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000177 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-304 az=16634 \| gx=21 gy=-173 gz=-92 \| t=953 | 0.109s | PASS |  |
| SOAK-READ-000178 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.20 gy=-1.55 gz=-0.78 dps \| t=28.72 C | 0.110s | PASS |  |
| SOAK-ACCEL-000179 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 82 -313 16626) | 0.110s | PASS |  |
| SOAK-GYRO-000180 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.78 dps (raw: 20 -174 -89) | 0.093s | PASS |  |
| SOAK-TEMP-000181 | soak/data | `temp` | Temp | Temp: 28.67 C (raw: 940) | 0.094s | PASS |  |
| SOAK-STATUS-000182 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000183 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000184 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000185 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000186 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000187 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000188 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000189 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000190 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5991 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000191 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000192 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 78183 -> 78200 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000193 | soak/data | `raw` | Raw: | Raw: ax=72 ay=-314 az=16632 \| gx=18 gy=-172 gz=-87 \| t=946 | 0.110s | PASS |  |
| SOAK-READ-000194 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.16 gy=-1.53 gz=-0.77 dps \| t=28.69 C | 0.109s | PASS |  |
| SOAK-ACCEL-000195 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.018 z=1.014 g (raw: 98 -302 16625) | 0.109s | PASS |  |
| SOAK-GYRO-000196 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.77 dps (raw: 21 -175 -88) | 0.094s | PASS |  |
| SOAK-TEMP-000197 | soak/data | `temp` | Temp | Temp: 28.69 C (raw: 945) | 0.094s | PASS |  |
| SOAK-STATUS-000198 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000199 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000200 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000201 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000202 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000203 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000204 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000205 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000206 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5991 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000207 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000208 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 84703 -> 84720 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000209 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-311 az=16627 \| gx=20 gy=-175 gz=-88 \| t=937 | 0.110s | PASS |  |
| SOAK-READ-000210 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.15 gy=-1.57 gz=-0.73 dps \| t=28.65 C | 0.109s | PASS |  |
| SOAK-ACCEL-000211 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.015 g (raw: 83 -303 16633) | 0.109s | PASS |  |
| SOAK-GYRO-000212 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.75 dps (raw: 20 -174 -86) | 0.109s | PASS |  |
| SOAK-TEMP-000213 | soak/data | `temp` | Temp | Temp: 28.67 C (raw: 939) | 0.110s | PASS |  |
| SOAK-STATUS-000214 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000215 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000216 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000217 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000218 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000219 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000220 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000221 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000222 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5986 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000223 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000224 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 91218 -> 91235 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000225 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-297 az=16631 \| gx=18 gy=-172 gz=-87 \| t=942 | 0.110s | PASS |  |
| SOAK-READ-000226 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.53 gz=-0.77 dps \| t=28.66 C | 0.109s | PASS |  |
| SOAK-ACCEL-000227 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.019 z=1.014 g (raw: 91 -308 16621) | 0.109s | PASS |  |
| SOAK-GYRO-000228 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.51 z=-0.76 dps (raw: 15 -172 -87) | 0.109s | PASS |  |
| SOAK-TEMP-000229 | soak/data | `temp` | Temp | Temp: 28.67 C (raw: 939) | 0.094s | PASS |  |
| SOAK-STATUS-000230 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000231 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000232 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000233 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000234 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000235 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000236 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000237 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000238 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000239 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.204s | PASS |  |
| SOAK-RECOVER-000240 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 97735 -> 97752 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000241 | soak/data | `raw` | Raw: | Raw: ax=82 ay=-306 az=16612 \| gx=18 gy=-176 gz=-90 \| t=929 | 0.094s | PASS |  |
| SOAK-READ-000242 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.54 gz=-0.74 dps \| t=28.68 C | 0.109s | PASS |  |
| SOAK-ACCEL-000243 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.018 z=1.013 g (raw: 69 -299 16605) | 0.110s | PASS |  |
| SOAK-GYRO-000244 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.55 z=-0.74 dps (raw: 20 -177 -85) | 0.110s | PASS |  |
| SOAK-TEMP-000245 | soak/data | `temp` | Temp | Temp: 28.66 C (raw: 936) | 0.109s | PASS |  |
| SOAK-STATUS-000246 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000247 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000248 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000249 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000250 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000251 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000252 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000253 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000254 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5990 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000255 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000256 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 104254 -> 104271 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000257 | soak/data | `raw` | Raw: | Raw: ax=90 ay=-301 az=16612 \| gx=19 gy=-173 gz=-84 \| t=920 | 0.109s | PASS |  |
| SOAK-READ-000258 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.012 g \| gx=+0.18 gy=-1.52 gz=-0.74 dps \| t=28.59 C | 0.109s | PASS |  |
| SOAK-ACCEL-000259 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.012 g (raw: 73 -310 16591) | 0.110s | PASS |  |
| SOAK-GYRO-000260 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.56 z=-0.76 dps (raw: 19 -178 -87) | 0.110s | PASS |  |
| SOAK-TEMP-000261 | soak/data | `temp` | Temp | Temp: 28.59 C (raw: 920) | 0.109s | PASS |  |
| SOAK-STATUS-000262 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000263 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000264 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000265 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000266 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000267 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000268 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000269 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000270 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5983 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000271 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.204s | PASS |  |
| SOAK-RECOVER-000272 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 110766 -> 110783 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000273 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-309 az=16586 \| gx=18 gy=-174 gz=-87 \| t=907 | 0.109s | PASS |  |
| SOAK-READ-000274 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.012 g \| gx=+0.17 gy=-1.56 gz=-0.77 dps \| t=28.53 C | 0.109s | PASS |  |
| SOAK-ACCEL-000275 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.018 z=1.013 g (raw: 72 -300 16605) | 0.110s | PASS |  |
| SOAK-GYRO-000276 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.75 dps (raw: 20 -176 -86) | 0.110s | PASS |  |
| SOAK-TEMP-000277 | soak/data | `temp` | Temp | Temp: 28.52 C (raw: 901) | 0.109s | PASS |  |
| SOAK-STATUS-000278 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000279 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000280 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000281 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000282 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000283 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000284 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000285 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000286 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000287 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000288 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 117283 -> 117300 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000289 | soak/data | `raw` | Raw: | Raw: ax=72 ay=-304 az=16606 \| gx=19 gy=-178 gz=-86 \| t=907 | 0.109s | PASS |  |
| SOAK-READ-000290 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.013 g \| gx=+0.21 gy=-1.51 gz=-0.77 dps \| t=28.57 C | 0.110s | PASS |  |
| SOAK-ACCEL-000291 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 74 -324 16619) | 0.109s | PASS |  |
| SOAK-GYRO-000292 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.55 z=-0.74 dps (raw: 21 -177 -85) | 0.109s | PASS |  |
| SOAK-TEMP-000293 | soak/data | `temp` | Temp | Temp: 28.55 C (raw: 908) | 0.094s | PASS |  |
| SOAK-STATUS-000294 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000295 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000296 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000297 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000298 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000299 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000300 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000301 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000302 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5992 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000303 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000304 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 123804 -> 123821 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000305 | soak/data | `raw` | Raw: | Raw: ax=61 ay=-314 az=16624 \| gx=20 gy=-176 gz=-88 \| t=901 | 0.110s | PASS |  |
| SOAK-READ-000306 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.014 g \| gx=+0.17 gy=-1.56 gz=-0.76 dps \| t=28.54 C | 0.109s | PASS |  |
| SOAK-ACCEL-000307 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 87 -305 16642) | 0.109s | PASS |  |
| SOAK-GYRO-000308 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.52 z=-0.76 dps (raw: 18 -174 -87) | 0.094s | PASS |  |
| SOAK-TEMP-000309 | soak/data | `temp` | Temp | Temp: 28.57 C (raw: 915) | 0.094s | PASS |  |
| SOAK-STATUS-000310 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000311 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000312 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000313 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000314 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000315 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000316 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000317 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000318 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=522 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.734s | PASS |  |
| SOAK-MIX-000319 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000320 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 130321 -> 130338 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000321 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-316 az=16611 \| gx=21 gy=-175 gz=-87 \| t=905 | 0.094s | PASS |  |
| SOAK-READ-000322 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.51 gz=-0.74 dps \| t=28.53 C | 0.110s | PASS |  |
| SOAK-ACCEL-000323 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.018 z=1.012 g (raw: 92 -287 16598) | 0.109s | PASS |  |
| SOAK-GYRO-000324 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.53 z=-0.74 dps (raw: 18 -175 -84) | 0.109s | PASS |  |
| SOAK-TEMP-000325 | soak/data | `temp` | Temp | Temp: 28.53 C (raw: 903) | 0.110s | PASS |  |
| SOAK-STATUS-000326 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.125s | PASS |  |
| SOAK-FIFO-000327 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000328 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-000329 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000330 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000331 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000332 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000333 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000334 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5987 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000335 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000336 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 136837 -> 136854 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000337 | soak/data | `raw` | Raw: | Raw: ax=86 ay=-319 az=16612 \| gx=22 gy=-174 gz=-88 \| t=890 | 0.109s | PASS |  |
| SOAK-READ-000338 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.014 g \| gx=+0.18 gy=-1.54 gz=-0.81 dps \| t=28.49 C | 0.094s | PASS |  |
| SOAK-ACCEL-000339 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 83 -320 16631) | 0.094s | PASS |  |
| SOAK-GYRO-000340 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.77 dps (raw: 20 -174 -88) | 0.093s | PASS |  |
| SOAK-TEMP-000341 | soak/data | `temp` | Temp | Temp: 28.48 C (raw: 892) | 0.110s | PASS |  |
| SOAK-STATUS-000342 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000343 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000344 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000345 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000346 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000347 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000348 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000349 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000350 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5986 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000351 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000352 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 143352 -> 143369 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000353 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-313 az=16605 \| gx=17 gy=-177 gz=-92 \| t=898 | 0.109s | PASS |  |
| SOAK-READ-000354 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.15 gy=-1.51 gz=-0.79 dps \| t=28.50 C | 0.109s | PASS |  |
| SOAK-ACCEL-000355 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 73 -311 16620) | 0.109s | PASS |  |
| SOAK-GYRO-000356 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.55 z=-0.80 dps (raw: 19 -177 -91) | 0.094s | PASS |  |
| SOAK-TEMP-000357 | soak/data | `temp` | Temp | Temp: 28.50 C (raw: 895) | 0.109s | PASS |  |
| SOAK-STATUS-000358 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000359 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000360 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000361 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000362 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000363 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000364 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000365 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000366 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5993 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000367 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000368 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 149874 -> 149891 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000369 | soak/data | `raw` | Raw: | Raw: ax=74 ay=-323 az=16626 \| gx=21 gy=-177 gz=-88 \| t=889 | 0.109s | PASS |  |
| SOAK-READ-000370 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.17 gy=-1.55 gz=-0.77 dps \| t=28.53 C | 0.094s | PASS |  |
| SOAK-ACCEL-000371 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 79 -321 16630) | 0.094s | PASS |  |
| SOAK-GYRO-000372 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.51 z=-0.76 dps (raw: 21 -173 -87) | 0.093s | PASS |  |
| SOAK-TEMP-000373 | soak/data | `temp` | Temp | Temp: 28.52 C (raw: 901) | 0.094s | PASS |  |
| SOAK-STATUS-000374 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000375 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000376 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000377 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000378 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000379 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000380 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000381 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000382 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000383 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000384 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 156389 -> 156406 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000385 | soak/data | `raw` | Raw: | Raw: ax=67 ay=-312 az=16625 \| gx=23 gy=-177 gz=-90 \| t=882 | 0.110s | PASS |  |
| SOAK-READ-000386 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.19 gy=-1.51 gz=-0.79 dps \| t=28.46 C | 0.109s | PASS |  |
| SOAK-ACCEL-000387 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.020 z=1.014 g (raw: 70 -322 16623) | 0.109s | PASS |  |
| SOAK-GYRO-000388 | soak/data | `gyro` | Gyro | Gyro: x=0.19 y=-1.57 z=-0.77 dps (raw: 22 -179 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000389 | soak/data | `temp` | Temp | Temp: 28.46 C (raw: 886) | 0.110s | PASS |  |
| SOAK-STATUS-000390 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000391 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000392 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000393 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000394 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000395 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000396 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000397 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000398 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4732 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5982 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000399 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000400 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 162900 -> 162917 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000401 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-301 az=16623 \| gx=19 gy=-181 gz=-93 \| t=886 | 0.094s | PASS |  |
| SOAK-READ-000402 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.53 gz=-0.76 dps \| t=28.49 C | 0.094s | PASS |  |
| SOAK-ACCEL-000403 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.020 z=1.014 g (raw: 71 -325 16618) | 0.094s | PASS |  |
| SOAK-GYRO-000404 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.76 dps (raw: 20 -175 -87) | 0.109s | PASS |  |
| SOAK-TEMP-000405 | soak/data | `temp` | Temp | Temp: 28.45 C (raw: 882) | 0.110s | PASS |  |
| SOAK-STATUS-000406 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000407 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000408 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000409 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000410 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000411 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000412 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000413 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000414 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5984 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000415 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000416 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 169413 -> 169430 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000417 | soak/data | `raw` | Raw: | Raw: ax=65 ay=-319 az=16621 \| gx=20 gy=-177 gz=-91 \| t=887 | 0.094s | PASS |  |
| SOAK-READ-000418 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.55 gz=-0.74 dps \| t=28.46 C | 0.093s | PASS |  |
| SOAK-ACCEL-000419 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 78 -316 16619) | 0.110s | PASS |  |
| SOAK-GYRO-000420 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.79 dps (raw: 21 -174 -90) | 0.110s | PASS |  |
| SOAK-TEMP-000421 | soak/data | `temp` | Temp | Temp: 28.46 C (raw: 886) | 0.109s | PASS |  |
| SOAK-STATUS-000422 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000423 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000424 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000425 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000426 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000427 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000428 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000429 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 21.000s | PASS | prompt recovered with health resync |
| SOAK-STRESS-000430 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000431 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000432 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 175930 -> 175947 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000433 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-309 az=16619 \| gx=20 gy=-177 gz=-85 \| t=870 | 0.110s | PASS |  |
| SOAK-READ-000434 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.52 gz=-0.78 dps \| t=28.44 C | 0.109s | PASS |  |
| SOAK-ACCEL-000435 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 73 -313 16614) | 0.094s | PASS |  |
| SOAK-GYRO-000436 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.55 z=-0.79 dps (raw: 19 -177 -90) | 0.094s | PASS |  |
| SOAK-TEMP-000437 | soak/data | `temp` | Temp | Temp: 28.43 C (raw: 878) | 0.094s | PASS |  |
| SOAK-STATUS-000438 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-000439 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000440 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000441 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000442 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000443 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000444 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000445 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000446 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5983 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000447 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.204s | PASS |  |
| SOAK-RECOVER-000448 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 182442 -> 182459 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000449 | soak/data | `raw` | Raw: | Raw: ax=90 ay=-311 az=16615 \| gx=16 gy=-176 gz=-86 \| t=878 | 0.094s | PASS |  |
| SOAK-READ-000450 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.12 gy=-1.51 gz=-0.78 dps \| t=28.44 C | 0.093s | PASS |  |
| SOAK-ACCEL-000451 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 76 -323 16631) | 0.094s | PASS |  |
| SOAK-GYRO-000452 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.55 z=-0.77 dps (raw: 17 -177 -88) | 0.094s | PASS |  |
| SOAK-TEMP-000453 | soak/data | `temp` | Temp | Temp: 28.45 C (raw: 884) | 0.094s | PASS |  |
| SOAK-STATUS-000454 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000455 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000456 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000457 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000458 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000459 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000460 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000461 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000462 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=519 errors=0 accel=500 gyro=500 temp=250 health_success=5984 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000463 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000464 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 188955 -> 188972 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000465 | soak/data | `raw` | Raw: | Raw: ax=67 ay=-316 az=16626 \| gx=19 gy=-176 gz=-87 \| t=879 | 0.109s | PASS |  |
| SOAK-READ-000466 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.015 g \| gx=+0.15 gy=-1.54 gz=-0.75 dps \| t=28.47 C | 0.109s | PASS |  |
| SOAK-ACCEL-000467 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 69 -316 16626) | 0.110s | PASS |  |
| SOAK-GYRO-000468 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.52 z=-0.75 dps (raw: 16 -174 -86) | 0.110s | PASS |  |
| SOAK-TEMP-000469 | soak/data | `temp` | Temp | Temp: 28.42 C (raw: 876) | 0.109s | PASS |  |
| SOAK-STATUS-000470 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000471 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000472 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000473 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000474 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000475 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000476 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000477 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000478 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4731 reads=519 errors=0 accel=500 gyro=500 temp=250 health_success=5981 health_fail=0 state=READY | 4.782s | PASS |  |
| SOAK-MIX-000479 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000480 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 195465 -> 195482 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000481 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-309 az=16639 \| gx=19 gy=-176 gz=-91 \| t=886 | 0.110s | PASS |  |
| SOAK-READ-000482 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.016 g \| gx=+0.15 gy=-1.55 gz=-0.75 dps \| t=28.43 C | 0.109s | PASS |  |
| SOAK-ACCEL-000483 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 63 -310 16625) | 0.109s | PASS |  |
| SOAK-GYRO-000484 | soak/data | `gyro` | Gyro | Gyro: x=0.19 y=-1.55 z=-0.78 dps (raw: 22 -177 -89) | 0.109s | PASS |  |
| SOAK-TEMP-000485 | soak/data | `temp` | Temp | Temp: 28.43 C (raw: 878) | 0.110s | PASS |  |
| SOAK-STATUS-000486 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000487 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000488 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000489 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000490 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000491 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000492 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000493 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000494 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=519 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000495 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000496 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 201983 -> 202000 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000497 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-306 az=16616 \| gx=15 gy=-175 gz=-88 \| t=864 | 0.110s | PASS |  |
| SOAK-READ-000498 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.013 g \| gx=+0.16 gy=-1.51 gz=-0.76 dps \| t=28.38 C | 0.109s | PASS |  |
| SOAK-ACCEL-000499 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 78 -309 16623) | 0.109s | PASS |  |
| SOAK-GYRO-000500 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.52 z=-0.74 dps (raw: 16 -174 -85) | 0.109s | PASS |  |
| SOAK-TEMP-000501 | soak/data | `temp` | Temp | Temp: 28.35 C (raw: 857) | 0.110s | PASS |  |
| SOAK-STATUS-000502 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000503 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000504 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000505 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000506 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000507 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000508 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000509 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000510 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4743 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5994 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000511 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000512 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 208506 -> 208523 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-000513 | soak/data | `raw` | Raw: | Raw: ax=69 ay=-315 az=16602 \| gx=16 gy=-176 gz=-90 \| t=862 | 0.110s | PASS |  |
| SOAK-READ-000514 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.55 gz=-0.77 dps \| t=28.35 C | 0.109s | PASS |  |
| SOAK-ACCEL-000515 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 75 -313 16603) | 0.109s | PASS |  |
| SOAK-GYRO-000516 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.53 z=-0.72 dps (raw: 19 -175 -82) | 0.109s | PASS |  |
| SOAK-TEMP-000517 | soak/data | `temp` | Temp | Temp: 28.39 C (raw: 869) | 0.110s | PASS |  |
| SOAK-STATUS-000518 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000519 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000520 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000521 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000522 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000523 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000524 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000525 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000526 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.719s | PASS |  |
| SOAK-MIX-000527 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000528 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 215023 -> 215040 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000529 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-314 az=16630 \| gx=18 gy=-175 gz=-84 \| t=866 | 0.110s | PASS |  |
| SOAK-READ-000530 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.58 gz=-0.75 dps \| t=28.34 C | 0.109s | PASS |  |
| SOAK-ACCEL-000531 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 89 -304 16629) | 0.109s | PASS |  |
| SOAK-GYRO-000532 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.75 dps (raw: 21 -175 -86) | 0.110s | PASS |  |
| SOAK-TEMP-000533 | soak/data | `temp` | Temp | Temp: 28.39 C (raw: 867) | 0.110s | PASS |  |
| SOAK-STATUS-000534 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000535 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000536 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000537 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000538 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000539 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000540 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000541 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000542 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4732 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5982 health_fail=0 state=READY | 4.734s | PASS |  |
| SOAK-MIX-000543 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000544 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 221534 -> 221551 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000545 | soak/data | `raw` | Raw: | Raw: ax=66 ay=-309 az=16622 \| gx=18 gy=-179 gz=-87 \| t=855 | 0.110s | PASS |  |
| SOAK-READ-000546 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.018 az=+1.014 g \| gx=+0.18 gy=-1.54 gz=-0.77 dps \| t=28.36 C | 0.094s | PASS |  |
| SOAK-ACCEL-000547 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 74 -310 16627) | 0.093s | PASS |  |
| SOAK-GYRO-000548 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.79 dps (raw: 21 -176 -90) | 0.094s | PASS |  |
| SOAK-TEMP-000549 | soak/data | `temp` | Temp | Temp: 28.34 C (raw: 855) | 0.109s | PASS |  |
| SOAK-STATUS-000550 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000551 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000552 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000553 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000554 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000555 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000556 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000557 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 21.000s | PASS | prompt recovered with health resync |
| SOAK-STRESS-000558 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000559 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000560 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 228049 -> 228066 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000561 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-299 az=16618 \| gx=20 gy=-177 gz=-84 \| t=853 | 0.109s | PASS |  |
| SOAK-READ-000562 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.53 gz=-0.74 dps \| t=28.37 C | 0.094s | PASS |  |
| SOAK-ACCEL-000563 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 89 -305 16631) | 0.110s | PASS |  |
| SOAK-GYRO-000564 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.54 z=-0.75 dps (raw: 19 -176 -86) | 0.093s | PASS |  |
| SOAK-TEMP-000565 | soak/data | `temp` | Temp | Temp: 28.37 C (raw: 862) | 0.094s | PASS |  |
| SOAK-STATUS-000566 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000567 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000568 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000569 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000570 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000571 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000572 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000573 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000574 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5990 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000575 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000576 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 234568 -> 234585 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000577 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-301 az=16608 \| gx=21 gy=-174 gz=-84 \| t=860 | 0.094s | PASS |  |
| SOAK-READ-000578 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.018 az=+1.014 g \| gx=+0.20 gy=-1.54 gz=-0.76 dps \| t=28.32 C | 0.094s | PASS |  |
| SOAK-ACCEL-000579 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 72 -308 16609) | 0.109s | PASS |  |
| SOAK-GYRO-000580 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.51 z=-0.73 dps (raw: 20 -173 -83) | 0.110s | PASS |  |
| SOAK-TEMP-000581 | soak/data | `temp` | Temp | Temp: 28.37 C (raw: 863) | 0.109s | PASS |  |
| SOAK-STATUS-000582 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000583 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000584 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000585 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000586 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000587 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000588 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000589 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000590 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000591 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000592 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 241085 -> 241102 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000593 | soak/data | `raw` | Raw: | Raw: ax=97 ay=-316 az=16629 \| gx=19 gy=-172 gz=-87 \| t=862 | 0.109s | PASS |  |
| SOAK-READ-000594 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.51 gz=-0.76 dps \| t=28.35 C | 0.109s | PASS |  |
| SOAK-ACCEL-000595 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.013 g (raw: 76 -298 16612) | 0.109s | PASS |  |
| SOAK-GYRO-000596 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.74 dps (raw: 20 -174 -85) | 0.110s | PASS |  |
| SOAK-TEMP-000597 | soak/data | `temp` | Temp | Temp: 28.37 C (raw: 863) | 0.109s | PASS |  |
| SOAK-STATUS-000598 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000599 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000600 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000601 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000602 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000603 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000604 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000605 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000606 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5990 health_fail=0 state=READY | 4.735s | PASS |  |
| SOAK-MIX-000607 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000608 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 247604 -> 247621 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000609 | soak/data | `raw` | Raw: | Raw: ax=60 ay=-310 az=16616 \| gx=18 gy=-173 gz=-84 \| t=861 | 0.110s | PASS |  |
| SOAK-READ-000610 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.21 gy=-1.53 gz=-0.78 dps \| t=28.36 C | 0.109s | PASS |  |
| SOAK-ACCEL-000611 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 77 -305 16616) | 0.109s | PASS |  |
| SOAK-GYRO-000612 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.51 z=-0.74 dps (raw: 17 -173 -85) | 0.109s | PASS |  |
| SOAK-TEMP-000613 | soak/data | `temp` | Temp | Temp: 28.39 C (raw: 867) | 0.110s | PASS |  |
| SOAK-STATUS-000614 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000615 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000616 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000617 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000618 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000619 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000620 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000621 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000622 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5992 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000623 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000624 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 254125 -> 254142 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000625 | soak/data | `raw` | Raw: | Raw: ax=71 ay=-308 az=16612 \| gx=21 gy=-178 gz=-85 \| t=857 | 0.109s | PASS |  |
| SOAK-READ-000626 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.54 gz=-0.75 dps \| t=28.29 C | 0.094s | PASS |  |
| SOAK-ACCEL-000627 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 79 -308 16625) | 0.094s | PASS |  |
| SOAK-GYRO-000628 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.74 dps (raw: 21 -175 -85) | 0.094s | PASS |  |
| SOAK-TEMP-000629 | soak/data | `temp` | Temp | Temp: 28.29 C (raw: 842) | 0.093s | PASS |  |
| SOAK-STATUS-000630 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000631 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000632 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000633 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000634 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000635 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000636 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000637 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000638 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000639 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000640 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 260643 -> 260660 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000641 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-307 az=16618 \| gx=22 gy=-172 gz=-84 \| t=855 | 0.093s | PASS |  |
| SOAK-READ-000642 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.020 az=+1.014 g \| gx=+0.18 gy=-1.56 gz=-0.76 dps \| t=28.33 C | 0.110s | PASS |  |
| SOAK-ACCEL-000643 | soak/data | `accel` | Accel | Accel: x=0.003 y=-0.019 z=1.013 g (raw: 56 -307 16611) | 0.109s | PASS |  |
| SOAK-GYRO-000644 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.52 z=-0.77 dps (raw: 19 -174 -88) | 0.109s | PASS |  |
| SOAK-TEMP-000645 | soak/data | `temp` | Temp | Temp: 28.35 C (raw: 857) | 0.109s | PASS |  |
| SOAK-STATUS-000646 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000647 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000648 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000649 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000650 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000651 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000652 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000653 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000654 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5992 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000655 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000656 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 267164 -> 267181 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000657 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-316 az=16625 \| gx=19 gy=-176 gz=-86 \| t=846 | 0.109s | PASS |  |
| SOAK-READ-000658 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.52 gz=-0.75 dps \| t=28.30 C | 0.094s | PASS |  |
| SOAK-ACCEL-000659 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 72 -316 16615) | 0.094s | PASS |  |
| SOAK-GYRO-000660 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.55 z=-0.78 dps (raw: 18 -177 -89) | 0.094s | PASS |  |
| SOAK-TEMP-000661 | soak/data | `temp` | Temp | Temp: 28.30 C (raw: 844) | 0.109s | PASS |  |
| SOAK-STATUS-000662 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000663 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000664 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000665 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000666 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000667 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000668 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000669 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000670 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5991 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000671 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.204s | PASS |  |
| SOAK-RECOVER-000672 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 273684 -> 273701 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000673 | soak/data | `raw` | Raw: | Raw: ax=70 ay=-307 az=16612 \| gx=19 gy=-172 gz=-81 \| t=840 | 0.094s | PASS |  |
| SOAK-READ-000674 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.51 gz=-0.74 dps \| t=28.23 C | 0.093s | PASS |  |
| SOAK-ACCEL-000675 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 73 -319 16620) | 0.110s | PASS |  |
| SOAK-GYRO-000676 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.56 z=-0.75 dps (raw: 19 -178 -86) | 0.110s | PASS |  |
| SOAK-TEMP-000677 | soak/data | `temp` | Temp | Temp: 28.26 C (raw: 835) | 0.109s | PASS |  |
| SOAK-STATUS-000678 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000679 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000680 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000681 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000682 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000683 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000684 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000685 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 21.031s | PASS | prompt recovered with health resync |
| SOAK-STRESS-000686 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5992 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000687 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000688 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 280205 -> 280222 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000689 | soak/data | `raw` | Raw: | Raw: ax=71 ay=-315 az=16613 \| gx=18 gy=-174 gz=-90 \| t=830 | 0.093s | PASS |  |
| SOAK-READ-000690 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.16 gy=-1.54 gz=-0.74 dps \| t=28.23 C | 0.110s | PASS |  |
| SOAK-ACCEL-000691 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 78 -311 16611) | 0.110s | PASS |  |
| SOAK-GYRO-000692 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.78 dps (raw: 20 -176 -89) | 0.109s | PASS |  |
| SOAK-TEMP-000693 | soak/data | `temp` | Temp | Temp: 28.22 C (raw: 825) | 0.109s | PASS |  |
| SOAK-STATUS-000694 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000695 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000696 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000697 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000698 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000699 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000700 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-000701 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-000702 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5989 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000703 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000704 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 286723 -> 286740 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000705 | soak/data | `raw` | Raw: | Raw: ax=71 ay=-315 az=16621 \| gx=19 gy=-172 gz=-90 \| t=834 | 0.110s | PASS |  |
| SOAK-READ-000706 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.19 gy=-1.55 gz=-0.76 dps \| t=28.27 C | 0.109s | PASS |  |
| SOAK-ACCEL-000707 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.013 g (raw: 75 -322 16605) | 0.094s | PASS |  |
| SOAK-GYRO-000708 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.51 z=-0.79 dps (raw: 18 -173 -90) | 0.094s | PASS |  |
| SOAK-TEMP-000709 | soak/data | `temp` | Temp | Temp: 28.23 C (raw: 828) | 0.109s | PASS |  |
| SOAK-STATUS-000710 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000711 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000712 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000713 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000714 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000715 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000716 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000717 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000718 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000719 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000720 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 293241 -> 293258 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000721 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-317 az=16613 \| gx=19 gy=-176 gz=-88 \| t=833 | 0.109s | PASS |  |
| SOAK-READ-000722 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.013 g \| gx=+0.18 gy=-1.51 gz=-0.76 dps \| t=28.27 C | 0.109s | PASS |  |
| SOAK-ACCEL-000723 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 77 -312 16601) | 0.110s | PASS |  |
| SOAK-GYRO-000724 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.77 dps (raw: 21 -176 -88) | 0.094s | PASS |  |
| SOAK-TEMP-000725 | soak/data | `temp` | Temp | Temp: 28.27 C (raw: 836) | 0.093s | PASS |  |
| SOAK-STATUS-000726 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000727 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000728 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000729 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000730 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000731 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000732 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000733 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000734 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000735 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000736 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 299757 -> 299774 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000737 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-306 az=16591 \| gx=21 gy=-174 gz=-87 \| t=830 | 0.109s | PASS |  |
| SOAK-READ-000738 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.51 gz=-0.76 dps \| t=28.26 C | 0.109s | PASS |  |
| SOAK-ACCEL-000739 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 68 -308 16601) | 0.110s | PASS |  |
| SOAK-GYRO-000740 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.53 z=-0.82 dps (raw: 19 -175 -94) | 0.110s | PASS |  |
| SOAK-TEMP-000741 | soak/data | `temp` | Temp | Temp: 28.21 C (raw: 821) | 0.109s | PASS |  |
| SOAK-STATUS-000742 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000743 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000744 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000745 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000746 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000747 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000748 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000749 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000750 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000751 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000752 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 306274 -> 306291 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000753 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-312 az=16621 \| gx=22 gy=-173 gz=-86 \| t=827 | 0.109s | PASS |  |
| SOAK-READ-000754 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.16 gy=-1.53 gz=-0.74 dps \| t=28.22 C | 0.110s | PASS |  |
| SOAK-ACCEL-000755 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 85 -313 16617) | 0.109s | PASS |  |
| SOAK-GYRO-000756 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.56 z=-0.78 dps (raw: 19 -178 -89) | 0.094s | PASS |  |
| SOAK-TEMP-000757 | soak/data | `temp` | Temp | Temp: 28.25 C (raw: 833) | 0.094s | PASS |  |
| SOAK-STATUS-000758 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000759 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000760 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.125s | PASS |  |
| SOAK-HEALTH-000761 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000762 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000763 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000764 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000765 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000766 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5987 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000767 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.204s | PASS |  |
| SOAK-RECOVER-000768 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 312790 -> 312807 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000769 | soak/data | `raw` | Raw: | Raw: ax=80 ay=-312 az=16621 \| gx=25 gy=-179 gz=-86 \| t=825 | 0.109s | PASS |  |
| SOAK-READ-000770 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.58 gz=-0.73 dps \| t=28.25 C | 0.109s | PASS |  |
| SOAK-ACCEL-000771 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 71 -311 16618) | 0.094s | PASS |  |
| SOAK-GYRO-000772 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.78 dps (raw: 20 -174 -89) | 0.094s | PASS |  |
| SOAK-TEMP-000773 | soak/data | `temp` | Temp | Temp: 28.22 C (raw: 825) | 0.093s | PASS |  |
| SOAK-STATUS-000774 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000775 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000776 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000777 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000778 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000779 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000780 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000781 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000782 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5991 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000783 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000784 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 319310 -> 319327 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000785 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-308 az=16606 \| gx=19 gy=-175 gz=-80 \| t=815 | 0.109s | PASS |  |
| SOAK-READ-000786 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.51 gz=-0.73 dps \| t=28.24 C | 0.110s | PASS |  |
| SOAK-ACCEL-000787 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.012 g (raw: 76 -303 16585) | 0.109s | PASS |  |
| SOAK-GYRO-000788 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.50 z=-0.75 dps (raw: 21 -171 -86) | 0.109s | PASS |  |
| SOAK-TEMP-000789 | soak/data | `temp` | Temp | Temp: 28.15 C (raw: 807) | 0.109s | PASS |  |
| SOAK-STATUS-000790 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000791 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000792 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000793 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000794 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000795 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000796 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000797 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000798 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5989 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000799 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000800 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 325828 -> 325845 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000801 | soak/data | `raw` | Raw: | Raw: ax=85 ay=-306 az=16629 \| gx=22 gy=-170 gz=-86 \| t=828 | 0.109s | PASS |  |
| SOAK-READ-000802 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.54 gz=-0.79 dps \| t=28.20 C | 0.110s | PASS |  |
| SOAK-ACCEL-000803 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.013 g (raw: 85 -300 16610) | 0.109s | PASS |  |
| SOAK-GYRO-000804 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.55 z=-0.78 dps (raw: 17 -177 -89) | 0.109s | PASS |  |
| SOAK-TEMP-000805 | soak/data | `temp` | Temp | Temp: 28.17 C (raw: 811) | 0.109s | PASS |  |
| SOAK-STATUS-000806 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000807 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000808 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000809 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000810 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000811 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000812 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000813 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 21.000s | PASS | prompt recovered with health resync |
| SOAK-STRESS-000814 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5989 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000815 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000816 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 332346 -> 332363 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000817 | soak/data | `raw` | Raw: | Raw: ax=74 ay=-311 az=16619 \| gx=24 gy=-175 gz=-86 \| t=826 | 0.109s | PASS |  |
| SOAK-READ-000818 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.15 gy=-1.53 gz=-0.78 dps \| t=28.25 C | 0.109s | PASS |  |
| SOAK-ACCEL-000819 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 90 -319 16628) | 0.110s | PASS |  |
| SOAK-GYRO-000820 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.55 z=-0.76 dps (raw: 21 -177 -87) | 0.110s | PASS |  |
| SOAK-TEMP-000821 | soak/data | `temp` | Temp | Temp: 28.27 C (raw: 838) | 0.109s | PASS |  |
| SOAK-STATUS-000822 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000823 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000824 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000825 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000826 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000827 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000828 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000829 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000830 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000831 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.204s | PASS |  |
| SOAK-RECOVER-000832 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 338864 -> 338881 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000833 | soak/data | `raw` | Raw: | Raw: ax=82 ay=-306 az=16612 \| gx=21 gy=-173 gz=-87 \| t=833 | 0.109s | PASS |  |
| SOAK-READ-000834 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.014 g \| gx=+0.18 gy=-1.53 gz=-0.71 dps \| t=28.25 C | 0.109s | PASS |  |
| SOAK-ACCEL-000835 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 78 -314 16616) | 0.110s | PASS |  |
| SOAK-GYRO-000836 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.56 z=-0.75 dps (raw: 18 -178 -86) | 0.110s | PASS |  |
| SOAK-TEMP-000837 | soak/data | `temp` | Temp | Temp: 28.24 C (raw: 830) | 0.109s | PASS |  |
| SOAK-STATUS-000838 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000839 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000840 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000841 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000842 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000843 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000844 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000845 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000846 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.782s | PASS |  |
| SOAK-MIX-000847 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000848 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 345381 -> 345398 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000849 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-323 az=16614 \| gx=21 gy=-175 gz=-86 \| t=818 | 0.094s | PASS |  |
| SOAK-READ-000850 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.56 gz=-0.71 dps \| t=28.20 C | 0.093s | PASS |  |
| SOAK-ACCEL-000851 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 72 -316 16618) | 0.110s | PASS |  |
| SOAK-GYRO-000852 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.58 z=-0.74 dps (raw: 21 -180 -85) | 0.110s | PASS |  |
| SOAK-TEMP-000853 | soak/data | `temp` | Temp | Temp: 28.29 C (raw: 843) | 0.109s | PASS |  |
| SOAK-STATUS-000854 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000855 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-000856 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000857 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000858 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000859 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000860 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000861 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000862 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=522 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000863 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000864 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 351895 -> 351912 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000865 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-315 az=16621 \| gx=20 gy=-177 gz=-91 \| t=829 | 0.109s | PASS |  |
| SOAK-READ-000866 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.19 gy=-1.54 gz=-0.75 dps \| t=28.26 C | 0.109s | PASS |  |
| SOAK-ACCEL-000867 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.020 z=1.015 g (raw: 66 -323 16647) | 0.110s | PASS |  |
| SOAK-GYRO-000868 | soak/data | `gyro` | Gyro | Gyro: x=0.19 y=-1.55 z=-0.75 dps (raw: 22 -177 -86) | 0.094s | PASS |  |
| SOAK-TEMP-000869 | soak/data | `temp` | Temp | Temp: 28.26 C (raw: 835) | 0.093s | PASS |  |
| SOAK-STATUS-000870 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000871 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000872 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000873 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000874 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000875 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-000876 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000877 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000878 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000879 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000880 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 358412 -> 358429 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000881 | soak/data | `raw` | Raw: | Raw: ax=64 ay=-312 az=16633 \| gx=17 gy=-176 gz=-89 \| t=841 | 0.109s | PASS |  |
| SOAK-READ-000882 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.17 gy=-1.56 gz=-0.76 dps \| t=28.21 C | 0.110s | PASS |  |
| SOAK-ACCEL-000883 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 86 -309 16627) | 0.109s | PASS |  |
| SOAK-GYRO-000884 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.54 z=-0.74 dps (raw: 17 -176 -84) | 0.109s | PASS |  |
| SOAK-TEMP-000885 | soak/data | `temp` | Temp | Temp: 28.22 C (raw: 824) | 0.109s | PASS |  |
| SOAK-STATUS-000886 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-000887 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000888 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000889 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000890 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000891 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000892 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000893 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000894 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=519 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000895 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000896 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 364928 -> 364945 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000897 | soak/data | `raw` | Raw: | Raw: ax=80 ay=-306 az=16630 \| gx=20 gy=-173 gz=-91 \| t=827 | 0.094s | PASS |  |
| SOAK-READ-000898 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.53 gz=-0.74 dps \| t=28.28 C | 0.094s | PASS |  |
| SOAK-ACCEL-000899 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 89 -325 16620) | 0.093s | PASS |  |
| SOAK-GYRO-000900 | soak/data | `gyro` | Gyro | Gyro: x=0.21 y=-1.53 z=-0.73 dps (raw: 24 -175 -83) | 0.110s | PASS |  |
| SOAK-TEMP-000901 | soak/data | `temp` | Temp | Temp: 28.23 C (raw: 826) | 0.110s | PASS |  |
| SOAK-STATUS-000902 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000903 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000904 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000905 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000906 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-000907 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000908 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000909 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000910 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000911 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000912 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 371445 -> 371462 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000913 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-305 az=16609 \| gx=20 gy=-174 gz=-89 \| t=822 | 0.094s | PASS |  |
| SOAK-READ-000914 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.16 gy=-1.51 gz=-0.74 dps \| t=28.22 C | 0.093s | PASS |  |
| SOAK-ACCEL-000915 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 76 -319 16616) | 0.094s | PASS |  |
| SOAK-GYRO-000916 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.74 dps (raw: 20 -175 -85) | 0.110s | PASS |  |
| SOAK-TEMP-000917 | soak/data | `temp` | Temp | Temp: 28.23 C (raw: 826) | 0.094s | PASS |  |
| SOAK-STATUS-000918 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000919 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-000920 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000921 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-000922 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000923 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-000924 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-000925 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-000926 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5989 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-000927 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000928 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 377963 -> 377980 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000929 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-314 az=16626 \| gx=20 gy=-174 gz=-87 \| t=824 | 0.109s | PASS |  |
| SOAK-READ-000930 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.56 gz=-0.74 dps \| t=28.25 C | 0.094s | PASS |  |
| SOAK-ACCEL-000931 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 73 -311 16612) | 0.093s | PASS |  |
| SOAK-GYRO-000932 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.53 z=-0.77 dps (raw: 19 -175 -88) | 0.094s | PASS |  |
| SOAK-TEMP-000933 | soak/data | `temp` | Temp | Temp: 28.23 C (raw: 828) | 0.110s | PASS |  |
| SOAK-STATUS-000934 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000935 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000936 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-000937 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-000938 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-000939 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000940 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000941 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 21.047s | PASS | prompt recovered with health resync |
| SOAK-STRESS-000942 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=519 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-000943 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-000944 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 384481 -> 384498 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-000945 | soak/data | `raw` | Raw: | Raw: ax=84 ay=-295 az=16642 \| gx=24 gy=-176 gz=-88 \| t=812 | 0.094s | PASS |  |
| SOAK-READ-000946 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.50 gz=-0.79 dps \| t=28.18 C | 0.094s | PASS |  |
| SOAK-ACCEL-000947 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 81 -320 16616) | 0.109s | PASS |  |
| SOAK-GYRO-000948 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.53 z=-0.74 dps (raw: 19 -175 -84) | 0.110s | PASS |  |
| SOAK-TEMP-000949 | soak/data | `temp` | Temp | Temp: 28.23 C (raw: 827) | 0.109s | PASS |  |
| SOAK-STATUS-000950 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000951 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000952 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-000953 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-000954 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000955 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000956 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000957 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000958 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5991 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-000959 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000960 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 391001 -> 391018 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000961 | soak/data | `raw` | Raw: | Raw: ax=80 ay=-310 az=16634 \| gx=19 gy=-175 gz=-81 \| t=821 | 0.109s | PASS |  |
| SOAK-READ-000962 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.014 g \| gx=+0.15 gy=-1.55 gz=-0.77 dps \| t=28.21 C | 0.109s | PASS |  |
| SOAK-ACCEL-000963 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 73 -307 16617) | 0.109s | PASS |  |
| SOAK-GYRO-000964 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.56 z=-0.74 dps (raw: 21 -178 -84) | 0.110s | PASS |  |
| SOAK-TEMP-000965 | soak/data | `temp` | Temp | Temp: 28.18 C (raw: 813) | 0.109s | PASS |  |
| SOAK-STATUS-000966 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000967 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-000968 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000969 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000970 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-000971 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-000972 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-000973 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-000974 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000975 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-000976 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 397516 -> 397533 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-000977 | soak/data | `raw` | Raw: | Raw: ax=64 ay=-305 az=16611 \| gx=17 gy=-173 gz=-92 \| t=819 | 0.109s | PASS |  |
| SOAK-READ-000978 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.53 gz=-0.70 dps \| t=28.13 C | 0.109s | PASS |  |
| SOAK-ACCEL-000979 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 75 -317 16599) | 0.109s | PASS |  |
| SOAK-GYRO-000980 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.52 z=-0.77 dps (raw: 19 -174 -88) | 0.110s | PASS |  |
| SOAK-TEMP-000981 | soak/data | `temp` | Temp | Temp: 28.20 C (raw: 818) | 0.109s | PASS |  |
| SOAK-STATUS-000982 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-000983 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-000984 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-000985 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-000986 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-000987 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-000988 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-000989 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-000990 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-000991 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-000992 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 404030 -> 404047 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-000993 | soak/data | `raw` | Raw: | Raw: ax=89 ay=-316 az=16610 \| gx=21 gy=-177 gz=-85 \| t=824 | 0.109s | PASS |  |
| SOAK-READ-000994 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.51 gz=-0.75 dps \| t=28.18 C | 0.094s | PASS |  |
| SOAK-ACCEL-000995 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 85 -319 16621) | 0.094s | PASS |  |
| SOAK-GYRO-000996 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.74 dps (raw: 20 -176 -84) | 0.109s | PASS |  |
| SOAK-TEMP-000997 | soak/data | `temp` | Temp | Temp: 28.17 C (raw: 811) | 0.110s | PASS |  |
| SOAK-STATUS-000998 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-000999 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001000 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001001 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001002 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001003 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001004 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001005 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001006 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=522 errors=0 accel=500 gyro=500 temp=250 health_success=5989 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001007 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.204s | PASS |  |
| SOAK-RECOVER-001008 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 410548 -> 410565 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001009 | soak/data | `raw` | Raw: | Raw: ax=66 ay=-312 az=16578 \| gx=19 gy=-172 gz=-90 \| t=809 | 0.109s | PASS |  |
| SOAK-READ-001010 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.006 ay=-0.019 az=+1.012 g \| gx=+0.16 gy=-1.54 gz=-0.71 dps \| t=28.17 C | 0.109s | PASS |  |
| SOAK-ACCEL-001011 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.012 g (raw: 79 -309 16598) | 0.110s | PASS |  |
| SOAK-GYRO-001012 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.75 dps (raw: 20 -176 -86) | 0.110s | PASS |  |
| SOAK-TEMP-001013 | soak/data | `temp` | Temp | Temp: 28.13 C (raw: 802) | 0.109s | PASS |  |
| SOAK-STATUS-001014 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001015 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001016 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001017 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001018 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001019 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001020 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001021 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001022 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5991 health_fail=0 state=READY | 4.719s | PASS |  |
| SOAK-MIX-001023 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001024 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 417068 -> 417085 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001025 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-310 az=16622 \| gx=21 gy=-175 gz=-83 \| t=804 | 0.109s | PASS |  |
| SOAK-READ-001026 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.014 g \| gx=+0.17 gy=-1.51 gz=-0.74 dps \| t=28.18 C | 0.109s | PASS |  |
| SOAK-ACCEL-001027 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.018 z=1.013 g (raw: 68 -299 16600) | 0.110s | PASS |  |
| SOAK-GYRO-001028 | soak/data | `gyro` | Gyro | Gyro: x=0.20 y=-1.54 z=-0.72 dps (raw: 23 -176 -82) | 0.093s | PASS |  |
| SOAK-TEMP-001029 | soak/data | `temp` | Temp | Temp: 28.20 C (raw: 819) | 0.094s | PASS |  |
| SOAK-STATUS-001030 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001031 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001032 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001033 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001034 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001035 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001036 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001037 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001038 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001039 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001040 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 423581 -> 423598 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001041 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-307 az=16624 \| gx=20 gy=-176 gz=-84 \| t=817 | 0.110s | PASS |  |
| SOAK-READ-001042 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.18 gy=-1.52 gz=-0.72 dps \| t=28.21 C | 0.110s | PASS |  |
| SOAK-ACCEL-001043 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 79 -321 16626) | 0.109s | PASS |  |
| SOAK-GYRO-001044 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.72 dps (raw: 20 -176 -82) | 0.109s | PASS |  |
| SOAK-TEMP-001045 | soak/data | `temp` | Temp | Temp: 28.17 C (raw: 811) | 0.094s | PASS |  |
| SOAK-STATUS-001046 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001047 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-001048 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001049 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001050 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001051 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001052 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001053 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001054 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5985 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001055 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001056 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 430095 -> 430112 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001057 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-314 az=16606 \| gx=25 gy=-175 gz=-83 \| t=802 | 0.094s | PASS |  |
| SOAK-READ-001058 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.56 gz=-0.76 dps \| t=28.18 C | 0.109s | PASS |  |
| SOAK-ACCEL-001059 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.020 z=1.014 g (raw: 69 -328 16622) | 0.109s | PASS |  |
| SOAK-GYRO-001060 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.55 z=-0.69 dps (raw: 17 -177 -79) | 0.110s | PASS |  |
| SOAK-TEMP-001061 | soak/data | `temp` | Temp | Temp: 28.14 C (raw: 804) | 0.093s | PASS |  |
| SOAK-STATUS-001062 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001063 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001064 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001065 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001066 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001067 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001068 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001069 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 20.984s | PASS | prompt recovered with health resync |
| SOAK-STRESS-001070 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.735s | PASS |  |
| SOAK-MIX-001071 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001072 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 436611 -> 436628 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001073 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-312 az=16632 \| gx=18 gy=-174 gz=-86 \| t=809 | 0.110s | PASS |  |
| SOAK-READ-001074 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.006 ay=-0.020 az=+1.014 g \| gx=+0.15 gy=-1.56 gz=-0.74 dps \| t=28.13 C | 0.109s | PASS |  |
| SOAK-ACCEL-001075 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 80 -318 16623) | 0.109s | PASS |  |
| SOAK-GYRO-001076 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.73 dps (raw: 21 -175 -83) | 0.109s | PASS |  |
| SOAK-TEMP-001077 | soak/data | `temp` | Temp | Temp: 28.16 C (raw: 810) | 0.110s | PASS |  |
| SOAK-STATUS-001078 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001079 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001080 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001081 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001082 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001083 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001084 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001085 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001086 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5984 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001087 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001088 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 443124 -> 443141 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001089 | soak/data | `raw` | Raw: | Raw: ax=89 ay=-309 az=16614 \| gx=21 gy=-176 gz=-89 \| t=809 | 0.093s | PASS |  |
| SOAK-READ-001090 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.15 gy=-1.53 gz=-0.76 dps \| t=28.11 C | 0.094s | PASS |  |
| SOAK-ACCEL-001091 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.013 g (raw: 79 -299 16606) | 0.109s | PASS |  |
| SOAK-GYRO-001092 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.55 z=-0.74 dps (raw: 18 -177 -84) | 0.109s | PASS |  |
| SOAK-TEMP-001093 | soak/data | `temp` | Temp | Temp: 28.14 C (raw: 804) | 0.109s | PASS |  |
| SOAK-STATUS-001094 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001095 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001096 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001097 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-001098 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001099 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001100 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001101 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001102 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5986 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001103 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001104 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 449639 -> 449656 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001105 | soak/data | `raw` | Raw: | Raw: ax=72 ay=-311 az=16620 \| gx=22 gy=-176 gz=-84 \| t=819 | 0.109s | PASS |  |
| SOAK-READ-001106 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.17 gy=-1.51 gz=-0.76 dps \| t=28.17 C | 0.110s | PASS |  |
| SOAK-ACCEL-001107 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 84 -306 16623) | 0.109s | PASS |  |
| SOAK-GYRO-001108 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.52 z=-0.74 dps (raw: 19 -174 -85) | 0.109s | PASS |  |
| SOAK-TEMP-001109 | soak/data | `temp` | Temp | Temp: 28.18 C (raw: 815) | 0.109s | PASS |  |
| SOAK-STATUS-001110 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001111 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-001112 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001113 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001114 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001115 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001116 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001117 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-001118 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001119 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001120 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 456156 -> 456173 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001121 | soak/data | `raw` | Raw: | Raw: ax=74 ay=-301 az=16604 \| gx=18 gy=-175 gz=-85 \| t=800 | 0.094s | PASS |  |
| SOAK-READ-001122 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.54 gz=-0.72 dps \| t=28.17 C | 0.109s | PASS |  |
| SOAK-ACCEL-001123 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 81 -307 16614) | 0.109s | PASS |  |
| SOAK-GYRO-001124 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.73 dps (raw: 20 -174 -83) | 0.110s | PASS |  |
| SOAK-TEMP-001125 | soak/data | `temp` | Temp | Temp: 28.14 C (raw: 803) | 0.110s | PASS |  |
| SOAK-STATUS-001126 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-001127 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001128 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001129 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001130 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001131 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001132 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001133 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001134 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001135 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001136 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 462672 -> 462689 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001137 | soak/data | `raw` | Raw: | Raw: ax=94 ay=-314 az=16609 \| gx=15 gy=-174 gz=-86 \| t=798 | 0.125s | PASS |  |
| SOAK-READ-001138 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.17 gy=-1.55 gz=-0.74 dps \| t=28.18 C | 0.093s | PASS |  |
| SOAK-ACCEL-001139 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 89 -313 16624) | 0.094s | PASS |  |
| SOAK-GYRO-001140 | soak/data | `gyro` | Gyro | Gyro: x=0.19 y=-1.51 z=-0.75 dps (raw: 22 -173 -86) | 0.109s | PASS |  |
| SOAK-TEMP-001141 | soak/data | `temp` | Temp | Temp: 28.15 C (raw: 807) | 0.109s | PASS |  |
| SOAK-STATUS-001142 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001143 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001144 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001145 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001146 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001147 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001148 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001149 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001150 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5990 health_fail=0 state=READY | 4.782s | PASS |  |
| SOAK-MIX-001151 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001152 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 469191 -> 469208 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001153 | soak/data | `raw` | Raw: | Raw: ax=80 ay=-309 az=16626 \| gx=20 gy=-172 gz=-87 \| t=811 | 0.110s | PASS |  |
| SOAK-READ-001154 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.14 gy=-1.52 gz=-0.75 dps \| t=28.18 C | 0.109s | PASS |  |
| SOAK-ACCEL-001155 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.014 g (raw: 79 -301 16616) | 0.109s | PASS |  |
| SOAK-GYRO-001156 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.52 z=-0.78 dps (raw: 19 -174 -89) | 0.094s | PASS |  |
| SOAK-TEMP-001157 | soak/data | `temp` | Temp | Temp: 28.17 C (raw: 811) | 0.094s | PASS |  |
| SOAK-STATUS-001158 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-001159 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001160 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001161 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001162 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001163 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001164 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001165 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001166 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001167 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001168 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 475708 -> 475725 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001169 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-300 az=16628 \| gx=22 gy=-173 gz=-87 \| t=804 | 0.109s | PASS |  |
| SOAK-READ-001170 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.58 gz=-0.75 dps \| t=28.12 C | 0.109s | PASS |  |
| SOAK-ACCEL-001171 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.012 g (raw: 73 -316 16596) | 0.110s | PASS |  |
| SOAK-GYRO-001172 | soak/data | `gyro` | Gyro | Gyro: x=0.22 y=-1.52 z=-0.75 dps (raw: 25 -174 -86) | 0.094s | PASS |  |
| SOAK-TEMP-001173 | soak/data | `temp` | Temp | Temp: 28.18 C (raw: 814) | 0.109s | PASS |  |
| SOAK-STATUS-001174 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001175 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001176 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001177 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001178 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001179 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001180 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001181 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-001182 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5985 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001183 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001184 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 482222 -> 482239 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001185 | soak/data | `raw` | Raw: | Raw: ax=84 ay=-301 az=16630 \| gx=21 gy=-178 gz=-90 \| t=808 | 0.094s | PASS |  |
| SOAK-READ-001186 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.19 gy=-1.52 gz=-0.74 dps \| t=28.14 C | 0.094s | PASS |  |
| SOAK-ACCEL-001187 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 86 -312 16630) | 0.093s | PASS |  |
| SOAK-GYRO-001188 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.54 z=-0.76 dps (raw: 17 -176 -87) | 0.094s | PASS |  |
| SOAK-TEMP-001189 | soak/data | `temp` | Temp | Temp: 28.16 C (raw: 810) | 0.109s | PASS |  |
| SOAK-STATUS-001190 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001191 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.125s | PASS |  |
| SOAK-PROBE-001192 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001193 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001194 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001195 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-001196 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001197 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 20.890s | PASS | prompt recovered with health resync |
| SOAK-STRESS-001198 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5986 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001199 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001200 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 488737 -> 488754 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001201 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-311 az=16617 \| gx=18 gy=-176 gz=-86 \| t=807 | 0.110s | PASS |  |
| SOAK-READ-001202 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.19 gy=-1.54 gz=-0.76 dps \| t=28.14 C | 0.109s | PASS |  |
| SOAK-ACCEL-001203 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 86 -309 16630) | 0.109s | PASS |  |
| SOAK-GYRO-001204 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.56 z=-0.73 dps (raw: 21 -178 -83) | 0.110s | PASS |  |
| SOAK-TEMP-001205 | soak/data | `temp` | Temp | Temp: 28.15 C (raw: 807) | 0.110s | PASS |  |
| SOAK-STATUS-001206 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-001207 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001208 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001209 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001210 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001211 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001212 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001213 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001214 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5990 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-001215 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001216 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 495256 -> 495273 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001217 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-311 az=16618 \| gx=20 gy=-173 gz=-86 \| t=820 | 0.109s | PASS |  |
| SOAK-READ-001218 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.016 g \| gx=+0.18 gy=-1.53 gz=-0.77 dps \| t=28.18 C | 0.110s | PASS |  |
| SOAK-ACCEL-001219 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 67 -314 16619) | 0.109s | PASS |  |
| SOAK-GYRO-001220 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.56 z=-0.80 dps (raw: 19 -178 -91) | 0.109s | PASS |  |
| SOAK-TEMP-001221 | soak/data | `temp` | Temp | Temp: 28.17 C (raw: 811) | 0.109s | PASS |  |
| SOAK-STATUS-001222 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001223 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-001224 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001225 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001226 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001227 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-001228 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001229 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001230 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001231 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001232 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 501772 -> 501789 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001233 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-306 az=16627 \| gx=20 gy=-175 gz=-86 \| t=815 | 0.109s | PASS |  |
| SOAK-READ-001234 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.19 gy=-1.49 gz=-0.76 dps \| t=28.21 C | 0.110s | PASS |  |
| SOAK-ACCEL-001235 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 78 -319 16638) | 0.109s | PASS |  |
| SOAK-GYRO-001236 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.54 z=-0.77 dps (raw: 19 -176 -88) | 0.109s | PASS |  |
| SOAK-TEMP-001237 | soak/data | `temp` | Temp | Temp: 28.20 C (raw: 819) | 0.109s | PASS |  |
| SOAK-STATUS-001238 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001239 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001240 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001241 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001242 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001243 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-001244 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001245 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001246 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5993 health_fail=0 state=READY | 4.797s | PASS |  |
| SOAK-MIX-001247 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001248 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 508294 -> 508311 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001249 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-305 az=16619 \| gx=15 gy=-176 gz=-86 \| t=817 | 0.094s | PASS |  |
| SOAK-READ-001250 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.58 gz=-0.74 dps \| t=28.23 C | 0.093s | PASS |  |
| SOAK-ACCEL-001251 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 84 -312 16621) | 0.109s | PASS |  |
| SOAK-GYRO-001252 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.56 z=-0.73 dps (raw: 17 -178 -83) | 0.109s | PASS |  |
| SOAK-TEMP-001253 | soak/data | `temp` | Temp | Temp: 28.19 C (raw: 816) | 0.094s | PASS |  |
| SOAK-STATUS-001254 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-001255 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001256 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001257 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001258 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001259 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001260 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001261 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001262 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5992 health_fail=0 state=READY | 4.797s | PASS |  |
| SOAK-MIX-001263 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001264 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 514815 -> 514832 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001265 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-320 az=16619 \| gx=19 gy=-179 gz=-83 \| t=816 | 0.109s | PASS |  |
| SOAK-READ-001266 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.006 ay=-0.020 az=+1.015 g \| gx=+0.16 gy=-1.53 gz=-0.75 dps \| t=28.18 C | 0.109s | PASS |  |
| SOAK-ACCEL-001267 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.013 g (raw: 79 -302 16611) | 0.094s | PASS |  |
| SOAK-GYRO-001268 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.53 z=-0.76 dps (raw: 19 -175 -87) | 0.094s | PASS |  |
| SOAK-TEMP-001269 | soak/data | `temp` | Temp | Temp: 28.20 C (raw: 819) | 0.093s | PASS |  |
| SOAK-STATUS-001270 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001271 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001272 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001273 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001274 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001275 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001276 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001277 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001278 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5983 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001279 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001280 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 521327 -> 521344 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001281 | soak/data | `raw` | Raw: | Raw: ax=84 ay=-320 az=16616 \| gx=22 gy=-173 gz=-83 \| t=821 | 0.109s | PASS |  |
| SOAK-READ-001282 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.16 gy=-1.57 gz=-0.71 dps \| t=28.19 C | 0.109s | PASS |  |
| SOAK-ACCEL-001283 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.015 g (raw: 68 -311 16640) | 0.109s | PASS |  |
| SOAK-GYRO-001284 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.54 z=-0.74 dps (raw: 18 -176 -84) | 0.110s | PASS |  |
| SOAK-TEMP-001285 | soak/data | `temp` | Temp | Temp: 28.19 C (raw: 816) | 0.109s | PASS |  |
| SOAK-STATUS-001286 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001287 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001288 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-001289 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001290 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001291 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001292 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001293 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-001294 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=519 errors=0 accel=500 gyro=500 temp=250 health_success=5983 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001295 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001296 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 527839 -> 527856 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001297 | soak/data | `raw` | Raw: | Raw: ax=68 ay=-321 az=16622 \| gx=21 gy=-176 gz=-83 \| t=816 | 0.110s | PASS |  |
| SOAK-READ-001298 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.55 gz=-0.77 dps \| t=28.22 C | 0.110s | PASS |  |
| SOAK-ACCEL-001299 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.016 g (raw: 72 -312 16651) | 0.109s | PASS |  |
| SOAK-GYRO-001300 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.57 z=-0.74 dps (raw: 20 -179 -84) | 0.109s | PASS |  |
| SOAK-TEMP-001301 | soak/data | `temp` | Temp | Temp: 28.20 C (raw: 819) | 0.110s | PASS |  |
| SOAK-STATUS-001302 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001303 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-001304 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001305 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001306 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001307 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001308 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001309 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-001310 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001311 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001312 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 534356 -> 534373 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001313 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-312 az=16623 \| gx=21 gy=-180 gz=-86 \| t=821 | 0.109s | PASS |  |
| SOAK-READ-001314 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.013 g \| gx=+0.15 gy=-1.55 gz=-0.73 dps \| t=28.20 C | 0.109s | PASS |  |
| SOAK-ACCEL-001315 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 84 -304 16613) | 0.109s | PASS |  |
| SOAK-GYRO-001316 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.75 dps (raw: 21 -176 -86) | 0.110s | PASS |  |
| SOAK-TEMP-001317 | soak/data | `temp` | Temp | Temp: 28.20 C (raw: 818) | 0.109s | PASS |  |
| SOAK-STATUS-001318 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001319 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001320 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-001321 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001322 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001323 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001324 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001325 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 20.984s | PASS | prompt recovered with health resync |
| SOAK-STRESS-001326 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001327 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001328 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 540872 -> 540889 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001329 | soak/data | `raw` | Raw: | Raw: ax=69 ay=-313 az=16618 \| gx=22 gy=-175 gz=-89 \| t=804 | 0.094s | PASS |  |
| SOAK-READ-001330 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.13 gy=-1.56 gz=-0.78 dps \| t=28.16 C | 0.094s | PASS |  |
| SOAK-ACCEL-001331 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 89 -313 16611) | 0.093s | PASS |  |
| SOAK-GYRO-001332 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.54 z=-0.75 dps (raw: 15 -176 -86) | 0.110s | PASS |  |
| SOAK-TEMP-001333 | soak/data | `temp` | Temp | Temp: 28.19 C (raw: 816) | 0.109s | PASS |  |
| SOAK-STATUS-001334 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001335 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001336 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001337 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001338 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001339 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001340 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001341 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001342 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001343 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001344 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 547388 -> 547405 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001345 | soak/data | `raw` | Raw: | Raw: ax=82 ay=-313 az=16617 \| gx=20 gy=-180 gz=-83 \| t=818 | 0.109s | PASS |  |
| SOAK-READ-001346 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.006 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.52 gz=-0.73 dps \| t=28.13 C | 0.109s | PASS |  |
| SOAK-ACCEL-001347 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 85 -308 16617) | 0.094s | PASS |  |
| SOAK-GYRO-001348 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.52 z=-0.75 dps (raw: 20 -174 -86) | 0.094s | PASS |  |
| SOAK-TEMP-001349 | soak/data | `temp` | Temp | Temp: 28.14 C (raw: 805) | 0.093s | PASS |  |
| SOAK-STATUS-001350 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001351 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001352 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001353 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001354 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001355 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001356 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001357 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001358 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5992 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001359 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001360 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 553909 -> 553926 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001361 | soak/data | `raw` | Raw: | Raw: ax=97 ay=-319 az=16629 \| gx=22 gy=-175 gz=-88 \| t=820 | 0.093s | PASS |  |
| SOAK-READ-001362 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.014 g \| gx=+0.15 gy=-1.52 gz=-0.79 dps \| t=28.18 C | 0.094s | PASS |  |
| SOAK-ACCEL-001363 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 74 -317 16640) | 0.094s | PASS |  |
| SOAK-GYRO-001364 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.58 z=-0.74 dps (raw: 20 -180 -85) | 0.109s | PASS |  |
| SOAK-TEMP-001365 | soak/data | `temp` | Temp | Temp: 28.16 C (raw: 809) | 0.109s | PASS |  |
| SOAK-STATUS-001366 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001367 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001368 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-001369 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001370 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001371 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001372 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001373 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-001374 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001375 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001376 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 560423 -> 560440 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001377 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-304 az=16624 \| gx=19 gy=-174 gz=-88 \| t=819 | 0.093s | PASS |  |
| SOAK-READ-001378 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.14 gy=-1.52 gz=-0.76 dps \| t=28.17 C | 0.110s | PASS |  |
| SOAK-ACCEL-001379 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 73 -311 16607) | 0.110s | PASS |  |
| SOAK-GYRO-001380 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.53 z=-0.76 dps (raw: 19 -175 -87) | 0.109s | PASS |  |
| SOAK-TEMP-001381 | soak/data | `temp` | Temp | Temp: 28.21 C (raw: 821) | 0.109s | PASS |  |
| SOAK-STATUS-001382 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001383 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001384 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-001385 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001386 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001387 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001388 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001389 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-001390 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5993 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001391 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001392 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 566945 -> 566962 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-001393 | soak/data | `raw` | Raw: | Raw: ax=86 ay=-304 az=16616 \| gx=16 gy=-175 gz=-86 \| t=816 | 0.110s | PASS |  |
| SOAK-READ-001394 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.018 az=+1.015 g \| gx=+0.18 gy=-1.52 gz=-0.78 dps \| t=28.21 C | 0.109s | PASS |  |
| SOAK-ACCEL-001395 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.018 z=1.014 g (raw: 94 -298 16619) | 0.109s | PASS |  |
| SOAK-GYRO-001396 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.56 z=-0.73 dps (raw: 17 -178 -83) | 0.109s | PASS |  |
| SOAK-TEMP-001397 | soak/data | `temp` | Temp | Temp: 28.18 C (raw: 815) | 0.110s | PASS |  |
| SOAK-STATUS-001398 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-001399 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001400 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001401 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001402 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001403 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001404 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001405 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-001406 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001407 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001408 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 573461 -> 573478 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001409 | soak/data | `raw` | Raw: | Raw: ax=81 ay=-302 az=16621 \| gx=15 gy=-176 gz=-88 \| t=828 | 0.110s | PASS |  |
| SOAK-READ-001410 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.015 g \| gx=+0.20 gy=-1.54 gz=-0.75 dps \| t=28.23 C | 0.109s | PASS |  |
| SOAK-ACCEL-001411 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.014 g (raw: 85 -301 16623) | 0.094s | PASS |  |
| SOAK-GYRO-001412 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.53 z=-0.79 dps (raw: 15 -175 -90) | 0.093s | PASS |  |
| SOAK-TEMP-001413 | soak/data | `temp` | Temp | Temp: 28.24 C (raw: 829) | 0.110s | PASS |  |
| SOAK-STATUS-001414 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001415 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001416 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001417 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001418 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001419 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001420 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001421 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001422 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5984 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001423 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001424 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 579974 -> 579991 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001425 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-315 az=16611 \| gx=21 gy=-178 gz=-87 \| t=823 | 0.110s | PASS |  |
| SOAK-READ-001426 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.015 g \| gx=+0.20 gy=-1.52 gz=-0.73 dps \| t=28.17 C | 0.109s | PASS |  |
| SOAK-ACCEL-001427 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 84 -314 16626) | 0.109s | PASS |  |
| SOAK-GYRO-001428 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.52 z=-0.75 dps (raw: 19 -174 -86) | 0.109s | PASS |  |
| SOAK-TEMP-001429 | soak/data | `temp` | Temp | Temp: 28.16 C (raw: 810) | 0.110s | PASS |  |
| SOAK-STATUS-001430 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001431 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001432 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001433 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001434 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001435 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001436 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001437 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001438 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5990 health_fail=0 state=READY | 4.781s | PASS |  |
| SOAK-MIX-001439 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001440 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 586493 -> 586510 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001441 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-304 az=16637 \| gx=20 gy=-177 gz=-86 \| t=825 | 0.110s | PASS |  |
| SOAK-READ-001442 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.17 gy=-1.54 gz=-0.77 dps \| t=28.23 C | 0.109s | PASS |  |
| SOAK-ACCEL-001443 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.013 g (raw: 78 -302 16609) | 0.109s | PASS |  |
| SOAK-GYRO-001444 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.56 z=-0.77 dps (raw: 19 -178 -88) | 0.109s | PASS |  |
| SOAK-TEMP-001445 | soak/data | `temp` | Temp | Temp: 28.21 C (raw: 823) | 0.110s | PASS |  |
| SOAK-STATUS-001446 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001447 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001448 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001449 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001450 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001451 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001452 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001453 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 21.031s | PASS | prompt recovered with health resync |
| SOAK-STRESS-001454 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5983 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001455 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001456 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 593005 -> 593022 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001457 | soak/data | `raw` | Raw: | Raw: ax=91 ay=-319 az=16643 \| gx=18 gy=-173 gz=-88 \| t=832 | 0.110s | PASS |  |
| SOAK-READ-001458 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.21 gy=-1.55 gz=-0.74 dps \| t=28.25 C | 0.110s | PASS |  |
| SOAK-ACCEL-001459 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 71 -318 16626) | 0.093s | PASS |  |
| SOAK-GYRO-001460 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.73 dps (raw: 21 -175 -83) | 0.094s | PASS |  |
| SOAK-TEMP-001461 | soak/data | `temp` | Temp | Temp: 28.25 C (raw: 833) | 0.109s | PASS |  |
| SOAK-STATUS-001462 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001463 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001464 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-001465 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001466 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001467 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001468 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001469 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-001470 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5985 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001471 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001472 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 599519 -> 599536 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001473 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-304 az=16621 \| gx=19 gy=-174 gz=-88 \| t=819 | 0.109s | PASS |  |
| SOAK-READ-001474 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.50 gz=-0.74 dps \| t=28.22 C | 0.109s | PASS |  |
| SOAK-ACCEL-001475 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 86 -309 16617) | 0.109s | PASS |  |
| SOAK-GYRO-001476 | soak/data | `gyro` | Gyro | Gyro: x=0.22 y=-1.57 z=-0.78 dps (raw: 25 -179 -89) | 0.094s | PASS |  |
| SOAK-TEMP-001477 | soak/data | `temp` | Temp | Temp: 28.21 C (raw: 823) | 0.093s | PASS |  |
| SOAK-STATUS-001478 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001479 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001480 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001481 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001482 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.125s | PASS |  |
| SOAK-SRC1-001483 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001484 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001485 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-001486 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=522 errors=0 accel=500 gyro=500 temp=250 health_success=5990 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001487 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001488 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 606038 -> 606055 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001489 | soak/data | `raw` | Raw: | Raw: ax=78 ay=-308 az=16621 \| gx=18 gy=-175 gz=-86 \| t=837 | 0.093s | PASS |  |
| SOAK-READ-001490 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.14 gy=-1.54 gz=-0.71 dps \| t=28.23 C | 0.094s | PASS |  |
| SOAK-ACCEL-001491 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.012 g (raw: 76 -313 16595) | 0.094s | PASS |  |
| SOAK-GYRO-001492 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.74 dps (raw: 20 -175 -85) | 0.109s | PASS |  |
| SOAK-TEMP-001493 | soak/data | `temp` | Temp | Temp: 28.22 C (raw: 825) | 0.109s | PASS |  |
| SOAK-STATUS-001494 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001495 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001496 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001497 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001498 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001499 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001500 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001501 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-001502 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5991 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001503 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001504 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 612558 -> 612575 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001505 | soak/data | `raw` | Raw: | Raw: ax=85 ay=-310 az=16637 \| gx=20 gy=-177 gz=-90 \| t=834 | 0.109s | PASS |  |
| SOAK-READ-001506 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.51 gz=-0.71 dps \| t=28.23 C | 0.109s | PASS |  |
| SOAK-ACCEL-001507 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 74 -310 16617) | 0.109s | PASS |  |
| SOAK-GYRO-001508 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.74 dps (raw: 21 -176 -85) | 0.110s | PASS |  |
| SOAK-TEMP-001509 | soak/data | `temp` | Temp | Temp: 28.21 C (raw: 821) | 0.093s | PASS |  |
| SOAK-STATUS-001510 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001511 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001512 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001513 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001514 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001515 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001516 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001517 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001518 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5987 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001519 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001520 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 619074 -> 619091 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001521 | soak/data | `raw` | Raw: | Raw: ax=85 ay=-318 az=16624 \| gx=19 gy=-174 gz=-87 \| t=823 | 0.109s | PASS |  |
| SOAK-READ-001522 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.014 g \| gx=+0.18 gy=-1.56 gz=-0.75 dps \| t=28.24 C | 0.109s | PASS |  |
| SOAK-ACCEL-001523 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 71 -307 16610) | 0.109s | PASS |  |
| SOAK-GYRO-001524 | soak/data | `gyro` | Gyro | Gyro: x=0.22 y=-1.51 z=-0.76 dps (raw: 25 -172 -87) | 0.094s | PASS |  |
| SOAK-TEMP-001525 | soak/data | `temp` | Temp | Temp: 28.24 C (raw: 830) | 0.093s | PASS |  |
| SOAK-STATUS-001526 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001527 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001528 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001529 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001530 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001531 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001532 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001533 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001534 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5992 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001535 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001536 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 625595 -> 625612 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001537 | soak/data | `raw` | Raw: | Raw: ax=71 ay=-317 az=16629 \| gx=17 gy=-174 gz=-85 \| t=835 | 0.109s | PASS |  |
| SOAK-READ-001538 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.53 gz=-0.75 dps \| t=28.25 C | 0.125s | PASS |  |
| SOAK-ACCEL-001539 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.014 g (raw: 77 -299 16624) | 0.094s | PASS |  |
| SOAK-GYRO-001540 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.54 z=-0.77 dps (raw: 18 -176 -88) | 0.094s | PASS |  |
| SOAK-TEMP-001541 | soak/data | `temp` | Temp | Temp: 28.22 C (raw: 824) | 0.093s | PASS |  |
| SOAK-STATUS-001542 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001543 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001544 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001545 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001546 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001547 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001548 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001549 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001550 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5991 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001551 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001552 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 632115 -> 632132 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001553 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-318 az=16629 \| gx=24 gy=-176 gz=-86 \| t=827 | 0.109s | PASS |  |
| SOAK-READ-001554 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.014 g \| gx=+0.15 gy=-1.48 gz=-0.77 dps \| t=28.23 C | 0.109s | PASS |  |
| SOAK-ACCEL-001555 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 81 -310 16630) | 0.109s | PASS |  |
| SOAK-GYRO-001556 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.56 z=-0.73 dps (raw: 18 -178 -83) | 0.110s | PASS |  |
| SOAK-TEMP-001557 | soak/data | `temp` | Temp | Temp: 28.26 C (raw: 834) | 0.109s | PASS |  |
| SOAK-STATUS-001558 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001559 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001560 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001561 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-001562 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001563 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001564 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001565 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001566 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001567 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001568 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 638628 -> 638645 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001569 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-298 az=16612 \| gx=22 gy=-177 gz=-89 \| t=832 | 0.109s | PASS |  |
| SOAK-READ-001570 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.19 gy=-1.51 gz=-0.77 dps \| t=28.22 C | 0.110s | PASS |  |
| SOAK-ACCEL-001571 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.013 g (raw: 78 -321 16613) | 0.110s | PASS |  |
| SOAK-GYRO-001572 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.56 z=-0.75 dps (raw: 20 -178 -86) | 0.109s | PASS |  |
| SOAK-TEMP-001573 | soak/data | `temp` | Temp | Temp: 28.20 C (raw: 820) | 0.109s | PASS |  |
| SOAK-STATUS-001574 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001575 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 20.985s | PASS | prompt recovered with health resync |
| SOAK-PROBE-001576 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001577 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001578 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001579 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001580 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001581 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001582 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5991 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001583 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001584 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 645148 -> 645165 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001585 | soak/data | `raw` | Raw: | Raw: ax=58 ay=-312 az=16611 \| gx=19 gy=-171 gz=-87 \| t=834 | 0.109s | PASS |  |
| SOAK-READ-001586 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.54 gz=-0.76 dps \| t=28.22 C | 0.110s | PASS |  |
| SOAK-ACCEL-001587 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.019 z=1.014 g (raw: 100 -305 16623) | 0.094s | PASS |  |
| SOAK-GYRO-001588 | soak/data | `gyro` | Gyro | Gyro: x=0.13 y=-1.56 z=-0.76 dps (raw: 15 -178 -87) | 0.109s | PASS |  |
| SOAK-TEMP-001589 | soak/data | `temp` | Temp | Temp: 28.28 C (raw: 839) | 0.109s | PASS |  |
| SOAK-STATUS-001590 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001591 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001592 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001593 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001594 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001595 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001596 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001597 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001598 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5990 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001599 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001600 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 651667 -> 651684 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001601 | soak/data | `raw` | Raw: | Raw: ax=88 ay=-305 az=16646 \| gx=16 gy=-173 gz=-87 \| t=831 | 0.109s | PASS |  |
| SOAK-READ-001602 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.020 az=+1.014 g \| gx=+0.18 gy=-1.54 gz=-0.72 dps \| t=28.29 C | 0.109s | PASS |  |
| SOAK-ACCEL-001603 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 90 -305 16632) | 0.109s | PASS |  |
| SOAK-GYRO-001604 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.53 z=-0.76 dps (raw: 19 -175 -87) | 0.094s | PASS |  |
| SOAK-TEMP-001605 | soak/data | `temp` | Temp | Temp: 28.27 C (raw: 836) | 0.093s | PASS |  |
| SOAK-STATUS-001606 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001607 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001608 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001609 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001610 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001611 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001612 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001613 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001614 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001615 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001616 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 658184 -> 658201 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001617 | soak/data | `raw` | Raw: | Raw: ax=87 ay=-304 az=16633 \| gx=19 gy=-171 gz=-86 \| t=837 | 0.093s | PASS |  |
| SOAK-READ-001618 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.16 gy=-1.54 gz=-0.71 dps \| t=28.30 C | 0.094s | PASS |  |
| SOAK-ACCEL-001619 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.014 g (raw: 78 -302 16627) | 0.110s | PASS |  |
| SOAK-GYRO-001620 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.54 z=-0.78 dps (raw: 16 -176 -89) | 0.109s | PASS |  |
| SOAK-TEMP-001621 | soak/data | `temp` | Temp | Temp: 28.30 C (raw: 845) | 0.109s | PASS |  |
| SOAK-STATUS-001622 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001623 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001624 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001625 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001626 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001627 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001628 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001629 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001630 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4734 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5985 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001631 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001632 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 664698 -> 664715 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001633 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-320 az=16635 \| gx=22 gy=-176 gz=-84 \| t=840 | 0.109s | PASS |  |
| SOAK-READ-001634 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.19 gy=-1.52 gz=-0.69 dps \| t=28.25 C | 0.110s | PASS |  |
| SOAK-ACCEL-001635 | soak/data | `accel` | Accel | Accel: x=0.006 y=-0.019 z=1.014 g (raw: 93 -307 16627) | 0.110s | PASS |  |
| SOAK-GYRO-001636 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.52 z=-0.72 dps (raw: 19 -174 -82) | 0.109s | PASS |  |
| SOAK-TEMP-001637 | soak/data | `temp` | Temp | Temp: 28.29 C (raw: 842) | 0.109s | PASS |  |
| SOAK-STATUS-001638 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001639 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001640 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001641 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001642 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001643 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001644 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001645 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-001646 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5988 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001647 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001648 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 671215 -> 671232 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001649 | soak/data | `raw` | Raw: | Raw: ax=69 ay=-310 az=16631 \| gx=21 gy=-176 gz=-85 \| t=838 | 0.094s | PASS |  |
| SOAK-READ-001650 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.18 gy=-1.55 gz=-0.74 dps \| t=28.24 C | 0.094s | PASS |  |
| SOAK-ACCEL-001651 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.013 g (raw: 78 -313 16612) | 0.094s | PASS |  |
| SOAK-GYRO-001652 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.57 z=-0.73 dps (raw: 16 -179 -83) | 0.093s | PASS |  |
| SOAK-TEMP-001653 | soak/data | `temp` | Temp | Temp: 28.27 C (raw: 836) | 0.094s | PASS |  |
| SOAK-STATUS-001654 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001655 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001656 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001657 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001658 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001659 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001660 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001661 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001662 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=522 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001663 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001664 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 677730 -> 677747 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001665 | soak/data | `raw` | Raw: | Raw: ax=64 ay=-310 az=16634 \| gx=20 gy=-178 gz=-86 \| t=834 | 0.094s | PASS |  |
| SOAK-READ-001666 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.51 gz=-0.74 dps \| t=28.26 C | 0.093s | PASS |  |
| SOAK-ACCEL-001667 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 86 -307 16629) | 0.094s | PASS |  |
| SOAK-GYRO-001668 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.54 z=-0.72 dps (raw: 19 -176 -82) | 0.110s | PASS |  |
| SOAK-TEMP-001669 | soak/data | `temp` | Temp | Temp: 28.28 C (raw: 840) | 0.109s | PASS |  |
| SOAK-STATUS-001670 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001671 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001672 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001673 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
| SOAK-CFG-001674 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001675 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001676 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001677 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001678 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4737 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5987 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001679 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001680 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 684246 -> 684263 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001681 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-308 az=16631 \| gx=18 gy=-175 gz=-86 \| t=837 | 0.109s | PASS |  |
| SOAK-READ-001682 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.20 gy=-1.53 gz=-0.75 dps \| t=28.30 C | 0.110s | PASS |  |
| SOAK-ACCEL-001683 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 80 -308 16635) | 0.109s | PASS |  |
| SOAK-GYRO-001684 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.56 z=-0.78 dps (raw: 21 -178 -89) | 0.094s | PASS |  |
| SOAK-TEMP-001685 | soak/data | `temp` | Temp | Temp: 28.27 C (raw: 836) | 0.094s | PASS |  |
| SOAK-STATUS-001686 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001687 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001688 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-001689 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001690 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001691 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001692 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001693 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001694 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001695 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001696 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 690764 -> 690781 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001697 | soak/data | `raw` | Raw: | Raw: ax=84 ay=-320 az=16648 \| gx=19 gy=-177 gz=-88 \| t=846 | 0.094s | PASS |  |
| SOAK-READ-001698 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.16 gy=-1.58 gz=-0.75 dps \| t=28.29 C | 0.093s | PASS |  |
| SOAK-ACCEL-001699 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.015 g (raw: 73 -306 16639) | 0.094s | PASS |  |
| SOAK-GYRO-001700 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.51 z=-0.80 dps (raw: 20 -173 -91) | 0.094s | PASS |  |
| SOAK-TEMP-001701 | soak/data | `temp` | Temp | Temp: 28.30 C (raw: 846) | 0.094s | PASS |  |
| SOAK-STATUS-001702 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-001703 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001704 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001705 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001706 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001707 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001708 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001709 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001710 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5986 health_fail=0 state=READY | 4.734s | PASS |  |
| SOAK-MIX-001711 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001712 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 697279 -> 697296 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001713 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-301 az=16615 \| gx=22 gy=-178 gz=-84 \| t=851 | 0.110s | PASS |  |
| SOAK-READ-001714 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.18 gy=-1.51 gz=-0.77 dps \| t=28.32 C | 0.110s | PASS |  |
| SOAK-ACCEL-001715 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 87 -312 16617) | 0.109s | PASS |  |
| SOAK-GYRO-001716 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.54 z=-0.76 dps (raw: 19 -176 -87) | 0.109s | PASS |  |
| SOAK-TEMP-001717 | soak/data | `temp` | Temp | Temp: 28.29 C (raw: 843) | 0.110s | PASS |  |
| SOAK-STATUS-001718 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001719 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001720 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001721 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001722 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001723 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001724 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001725 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001726 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=522 errors=0 accel=500 gyro=500 temp=250 health_success=5992 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001727 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001728 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 703800 -> 703817 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001729 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-304 az=16621 \| gx=21 gy=-177 gz=-87 \| t=829 | 0.110s | PASS |  |
| SOAK-READ-001730 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.58 gz=-0.73 dps \| t=28.22 C | 0.110s | PASS |  |
| SOAK-ACCEL-001731 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 80 -320 16621) | 0.109s | PASS |  |
| SOAK-GYRO-001732 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.57 z=-0.75 dps (raw: 21 -179 -86) | 0.109s | PASS |  |
| SOAK-TEMP-001733 | soak/data | `temp` | Temp | Temp: 28.27 C (raw: 836) | 0.110s | PASS |  |
| SOAK-STATUS-001734 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001735 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-001736 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-001737 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001738 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001739 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001740 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001741 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-001742 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001743 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001744 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 710313 -> 710330 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001745 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-308 az=16600 \| gx=20 gy=-176 gz=-83 \| t=828 | 0.109s | PASS |  |
| SOAK-READ-001746 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.51 gz=-0.76 dps \| t=28.25 C | 0.109s | PASS |  |
| SOAK-ACCEL-001747 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.014 g (raw: 88 -302 16624) | 0.094s | PASS |  |
| SOAK-GYRO-001748 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.74 dps (raw: 21 -176 -84) | 0.094s | PASS |  |
| SOAK-TEMP-001749 | soak/data | `temp` | Temp | Temp: 28.25 C (raw: 833) | 0.109s | PASS |  |
| SOAK-STATUS-001750 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001751 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001752 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-001753 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001754 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001755 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001756 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001757 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-001758 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.765s | PASS |  |
| SOAK-MIX-001759 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001760 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 716826 -> 716843 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-001761 | soak/data | `raw` | Raw: | Raw: ax=55 ay=-293 az=16630 \| gx=19 gy=-176 gz=-90 \| t=830 | 0.110s | PASS |  |
| SOAK-READ-001762 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.56 gz=-0.74 dps \| t=28.25 C | 0.109s | PASS |  |
| SOAK-ACCEL-001763 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 83 -313 16617) | 0.109s | PASS |  |
| SOAK-GYRO-001764 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.54 z=-0.74 dps (raw: 19 -176 -84) | 0.094s | PASS |  |
| SOAK-TEMP-001765 | soak/data | `temp` | Temp | Temp: 28.24 C (raw: 830) | 0.094s | PASS |  |
| SOAK-STATUS-001766 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001767 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001768 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-001769 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001770 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001771 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001772 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001773 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001774 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=522 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001775 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001776 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 723340 -> 723357 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001777 | soak/data | `raw` | Raw: | Raw: ax=90 ay=-304 az=16621 \| gx=23 gy=-172 gz=-87 \| t=830 | 0.110s | PASS |  |
| SOAK-READ-001778 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.013 g \| gx=+0.18 gy=-1.54 gz=-0.72 dps \| t=28.22 C | 0.109s | PASS |  |
| SOAK-ACCEL-001779 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 73 -319 16621) | 0.109s | PASS |  |
| SOAK-GYRO-001780 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.51 z=-0.79 dps (raw: 18 -173 -90) | 0.109s | PASS |  |
| SOAK-TEMP-001781 | soak/data | `temp` | Temp | Temp: 28.24 C (raw: 829) | 0.110s | PASS |  |
| SOAK-STATUS-001782 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001783 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001784 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001785 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001786 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001787 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001788 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001789 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001790 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001791 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001792 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 729857 -> 729874 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001793 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-310 az=16618 \| gx=19 gy=-178 gz=-88 \| t=824 | 0.110s | PASS |  |
| SOAK-READ-001794 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.54 gz=-0.74 dps \| t=28.22 C | 0.109s | PASS |  |
| SOAK-ACCEL-001795 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.014 g (raw: 76 -327 16625) | 0.125s | PASS |  |
| SOAK-GYRO-001796 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.51 z=-0.80 dps (raw: 21 -172 -91) | 0.093s | PASS |  |
| SOAK-TEMP-001797 | soak/data | `temp` | Temp | Temp: 28.23 C (raw: 827) | 0.094s | PASS |  |
| SOAK-STATUS-001798 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001799 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001800 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001801 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001802 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001803 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001804 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-001805 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001806 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5992 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001807 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001808 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 736378 -> 736395 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001809 | soak/data | `raw` | Raw: | Raw: ax=74 ay=-313 az=16632 \| gx=19 gy=-174 gz=-85 \| t=828 | 0.094s | PASS |  |
| SOAK-READ-001810 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.013 g \| gx=+0.18 gy=-1.53 gz=-0.77 dps \| t=28.20 C | 0.093s | PASS |  |
| SOAK-ACCEL-001811 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 68 -305 16601) | 0.094s | PASS |  |
| SOAK-GYRO-001812 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.54 z=-0.74 dps (raw: 18 -176 -85) | 0.110s | PASS |  |
| SOAK-TEMP-001813 | soak/data | `temp` | Temp | Temp: 28.22 C (raw: 824) | 0.094s | PASS |  |
| SOAK-STATUS-001814 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-001815 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001816 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001817 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001818 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001819 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001820 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001821 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001822 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5993 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001823 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY \| === Driver Health === \| State: READY \| Online: yes \| ... | 125.938s | PASS | prompt recovered with health resync |
| SOAK-RECOVER-001824 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 742900 -> 742917 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001825 | soak/data | `raw` | Raw: | Raw: ax=89 ay=-310 az=16609 \| gx=17 gy=-176 gz=-87 \| t=834 | 0.094s | PASS |  |
| SOAK-READ-001826 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.51 gz=-0.74 dps \| t=28.30 C | 0.094s | PASS |  |
| SOAK-ACCEL-001827 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 87 -310 16619) | 0.109s | PASS |  |
| SOAK-GYRO-001828 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.56 z=-0.71 dps (raw: 20 -178 -81) | 0.110s | PASS |  |
| SOAK-TEMP-001829 | soak/data | `temp` | Temp | Temp: 28.34 C (raw: 854) | 0.110s | PASS |  |
| SOAK-STATUS-001830 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001831 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001832 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001833 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001834 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001835 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001836 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001837 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001838 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5989 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001839 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001840 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 749418 -> 749435 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-001841 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-312 az=16620 \| gx=19 gy=-177 gz=-85 \| t=851 | 0.094s | PASS |  |
| SOAK-READ-001842 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.19 gy=-1.50 gz=-0.78 dps \| t=28.32 C | 0.093s | PASS |  |
| SOAK-ACCEL-001843 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.018 z=1.014 g (raw: 78 -302 16618) | 0.094s | PASS |  |
| SOAK-GYRO-001844 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.54 z=-0.77 dps (raw: 21 -176 -88) | 0.094s | PASS |  |
| SOAK-TEMP-001845 | soak/data | `temp` | Temp | Temp: 28.33 C (raw: 852) | 0.109s | PASS |  |
| SOAK-STATUS-001846 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001847 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001848 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001849 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001850 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001851 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001852 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001853 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001854 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001855 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001856 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 755932 -> 755949 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001857 | soak/data | `raw` | Raw: | Raw: ax=82 ay=-320 az=16611 \| gx=18 gy=-174 gz=-85 \| t=847 | 0.109s | PASS |  |
| SOAK-READ-001858 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.19 gy=-1.56 gz=-0.78 dps \| t=28.30 C | 0.109s | PASS |  |
| SOAK-ACCEL-001859 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 81 -315 16627) | 0.110s | PASS |  |
| SOAK-GYRO-001860 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.55 z=-0.75 dps (raw: 21 -177 -86) | 0.109s | PASS |  |
| SOAK-TEMP-001861 | soak/data | `temp` | Temp | Temp: 28.35 C (raw: 857) | 0.109s | PASS |  |
| SOAK-STATUS-001862 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-001863 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001864 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.093s | PASS |  |
| SOAK-HEALTH-001865 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001866 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001867 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001868 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001869 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-001870 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5983 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001871 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001872 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 762444 -> 762461 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001873 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-302 az=16651 \| gx=22 gy=-176 gz=-84 \| t=853 | 0.109s | PASS |  |
| SOAK-READ-001874 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.015 g \| gx=+0.18 gy=-1.56 gz=-0.75 dps \| t=28.34 C \| === Driver Health === \| State: READY \| Online: yes \| ... | 21.000s | PASS | prompt recovered with health resync |
| SOAK-ACCEL-001875 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 78 -310 16621) | 0.109s | PASS |  |
| SOAK-GYRO-001876 | soak/data | `gyro` | Gyro | Gyro: x=0.17 y=-1.56 z=-0.77 dps (raw: 19 -178 -88) | 0.110s | PASS |  |
| SOAK-TEMP-001877 | soak/data | `temp` | Temp | Temp: 28.29 C (raw: 842) | 0.110s | PASS |  |
| SOAK-STATUS-001878 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-001879 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001880 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001881 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001882 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001883 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001884 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001885 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001886 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5993 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001887 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001888 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 768966 -> 768983 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-001889 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-313 az=16646 \| gx=17 gy=-175 gz=-86 \| t=849 | 0.094s | PASS |  |
| SOAK-READ-001890 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.015 g \| gx=+0.19 gy=-1.55 gz=-0.75 dps \| t=28.38 C | 0.094s | PASS |  |
| SOAK-ACCEL-001891 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 76 -317 16639) | 0.109s | PASS |  |
| SOAK-GYRO-001892 | soak/data | `gyro` | Gyro | Gyro: x=0.19 y=-1.55 z=-0.74 dps (raw: 22 -177 -85) | 0.109s | PASS |  |
| SOAK-TEMP-001893 | soak/data | `temp` | Temp | Temp: 28.38 C (raw: 864) | 0.110s | PASS |  |
| SOAK-STATUS-001894 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001895 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001896 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001897 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001898 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001899 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001900 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001901 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001902 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5989 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001903 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001904 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 775484 -> 775501 (+17) \| ... | 0.093s | PASS |  |
| SOAK-RAW-001905 | soak/data | `raw` | Raw: | Raw: ax=83 ay=-323 az=16624 \| gx=19 gy=-179 gz=-84 \| t=866 | 0.094s | PASS |  |
| SOAK-READ-001906 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.17 gy=-1.51 gz=-0.76 dps \| t=28.33 C | 0.110s | PASS |  |
| SOAK-ACCEL-001907 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.018 z=1.015 g (raw: 70 -299 16633) | 0.109s | PASS |  |
| SOAK-GYRO-001908 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.51 z=-0.74 dps (raw: 20 -173 -84) | 0.109s | PASS |  |
| SOAK-TEMP-001909 | soak/data | `temp` | Temp | Temp: 28.35 C (raw: 857) | 0.110s | PASS |  |
| SOAK-STATUS-001910 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001911 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.093s | PASS |  |
| SOAK-PROBE-001912 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001913 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001914 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001915 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-001916 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001917 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.093s | PASS |  |
| SOAK-STRESS-001918 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5990 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001919 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001920 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 782003 -> 782020 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001921 | soak/data | `raw` | Raw: | Raw: ax=76 ay=-318 az=16608 \| gx=22 gy=-172 gz=-87 \| t=852 | 0.109s | PASS |  |
| SOAK-READ-001922 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.018 az=+1.015 g \| gx=+0.14 gy=-1.53 gz=-0.80 dps \| t=28.35 C | 0.109s | PASS |  |
| SOAK-ACCEL-001923 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.013 g (raw: 70 -317 16602) | 0.109s | PASS |  |
| SOAK-GYRO-001924 | soak/data | `gyro` | Gyro | Gyro: x=0.20 y=-1.52 z=-0.72 dps (raw: 23 -174 -82) | 0.110s | PASS |  |
| SOAK-TEMP-001925 | soak/data | `temp` | Temp | Temp: 28.38 C (raw: 865) | 0.093s | PASS |  |
| SOAK-STATUS-001926 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001927 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001928 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001929 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001930 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-001931 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001932 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001933 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001934 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4733 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5984 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001935 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY \| === Driver Health === \| State: READY \| Online: yes \| ... | 126.015s | PASS | prompt recovered with health resync |
| SOAK-RECOVER-001936 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 788516 -> 788533 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001937 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-315 az=16617 \| gx=19 gy=-175 gz=-88 \| t=896 | 0.110s | PASS |  |
| SOAK-READ-001938 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.014 g \| gx=+0.20 gy=-1.53 gz=-0.77 dps \| t=28.43 C | 0.109s | PASS |  |
| SOAK-ACCEL-001939 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 83 -318 16641) | 0.109s | PASS |  |
| SOAK-GYRO-001940 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.56 z=-0.73 dps (raw: 20 -178 -83) | 0.110s | PASS |  |
| SOAK-TEMP-001941 | soak/data | `temp` | Temp | Temp: 28.45 C (raw: 884) | 0.110s | PASS |  |
| SOAK-STATUS-001942 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-001943 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-001944 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-001945 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001946 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-001947 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-001948 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-001949 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001950 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4739 reads=520 errors=0 accel=500 gyro=500 temp=250 health_success=5989 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001951 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-001952 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 795034 -> 795051 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001953 | soak/data | `raw` | Raw: | Raw: ax=70 ay=-309 az=16642 \| gx=21 gy=-169 gz=-89 \| t=879 | 0.094s | PASS |  |
| SOAK-READ-001954 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.51 gz=-0.71 dps \| t=28.46 C | 0.094s | PASS |  |
| SOAK-ACCEL-001955 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 86 -314 16633) | 0.094s | PASS |  |
| SOAK-GYRO-001956 | soak/data | `gyro` | Gyro | Gyro: x=0.20 y=-1.58 z=-0.74 dps (raw: 23 -180 -85) | 0.109s | PASS |  |
| SOAK-TEMP-001957 | soak/data | `temp` | Temp | Temp: 28.46 C (raw: 885) | 0.110s | PASS |  |
| SOAK-STATUS-001958 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001959 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001960 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001961 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-001962 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001963 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-001964 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001965 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001966 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4738 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5988 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001967 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-001968 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 801551 -> 801568 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-001969 | soak/data | `raw` | Raw: | Raw: ax=71 ay=-303 az=16620 \| gx=21 gy=-175 gz=-89 \| t=885 | 0.109s | PASS |  |
| SOAK-READ-001970 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.019 az=+1.012 g \| gx=+0.17 gy=-1.53 gz=-0.79 dps \| t=28.47 C | 0.109s | PASS |  |
| SOAK-ACCEL-001971 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.020 z=1.013 g (raw: 84 -322 16606) | 0.109s | PASS |  |
| SOAK-GYRO-001972 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.75 dps (raw: 21 -175 -86) | 0.094s | PASS |  |
| SOAK-TEMP-001973 | soak/data | `temp` | Temp | Temp: 28.44 C (raw: 881) | 0.093s | PASS |  |
| SOAK-STATUS-001974 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-001975 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-001976 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001977 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-001978 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-001979 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001980 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-001981 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-001982 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4742 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5992 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-001983 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-001984 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 808072 -> 808089 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-001985 | soak/data | `raw` | Raw: | Raw: ax=73 ay=-306 az=16632 \| gx=23 gy=-175 gz=-87 \| t=883 | 0.094s | PASS |  |
| SOAK-READ-001986 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.014 g \| gx=+0.18 gy=-1.54 gz=-0.79 dps \| t=28.46 C \| === Driver Health === \| State: READY \| Online: yes \| ... | 20.985s | PASS | prompt recovered with health resync |
| SOAK-ACCEL-001987 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 78 -306 16632) | 0.110s | PASS |  |
| SOAK-GYRO-001988 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.53 z=-0.77 dps (raw: 21 -175 -88) | 0.093s | PASS |  |
| SOAK-TEMP-001989 | soak/data | `temp` | Temp | Temp: 28.39 C (raw: 868) | 0.094s | PASS |  |
| SOAK-STATUS-001990 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-001991 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-001992 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-001993 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-001994 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-001995 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-001996 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-001997 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-001998 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4741 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5991 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-001999 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-002000 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 814592 -> 814609 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-002001 | soak/data | `raw` | Raw: | Raw: ax=79 ay=-303 az=16626 \| gx=23 gy=-173 gz=-88 \| t=853 | 0.094s | PASS |  |
| SOAK-READ-002002 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.54 gz=-0.80 dps \| t=28.36 C | 0.093s | PASS |  |
| SOAK-ACCEL-002003 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 82 -314 16637) | 0.094s | PASS |  |
| SOAK-GYRO-002004 | soak/data | `gyro` | Gyro | Gyro: x=0.14 y=-1.53 z=-0.73 dps (raw: 16 -175 -83) | 0.094s | PASS |  |
| SOAK-TEMP-002005 | soak/data | `temp` | Temp | Temp: 28.35 C (raw: 858) | 0.094s | PASS |  |
| SOAK-STATUS-002006 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.093s | PASS |  |
| SOAK-FIFO-002007 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-002008 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.110s | PASS |  |
| SOAK-HEALTH-002009 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-002010 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-002011 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-002012 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.094s | PASS |  |
| SOAK-STEPS-002013 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.109s | PASS |  |
| SOAK-STRESS-002014 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5991 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-002015 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| SOAK-RECOVER-002016 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 821112 -> 821129 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-002017 | soak/data | `raw` | Raw: | Raw: ax=77 ay=-308 az=16646 \| gx=20 gy=-175 gz=-86 \| t=862 | 0.093s | PASS |  |
| SOAK-READ-002018 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.53 gz=-0.79 dps \| t=28.37 C | 0.094s | PASS |  |
| SOAK-ACCEL-002019 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 77 -313 16617) | 0.109s | PASS |  |
| SOAK-GYRO-002020 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.52 z=-0.78 dps (raw: 18 -174 -89) | 0.109s | PASS |  |
| SOAK-TEMP-002021 | soak/data | `temp` | Temp | Temp: 28.34 C (raw: 855) | 0.109s | PASS |  |
| SOAK-STATUS-002022 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SOAK-FIFO-002023 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-002024 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-002025 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-002026 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SOAK-SRC1-002027 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.093s | PASS |  |
| SOAK-TS-002028 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.110s | PASS |  |
| SOAK-STEPS-002029 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-002030 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=521 errors=0 accel=500 gyro=500 temp=251 health_success=5986 health_fail=0 state=READY | 4.750s | PASS |  |
| SOAK-MIX-002031 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-002032 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 827627 -> 827644 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-002033 | soak/data | `raw` | Raw: | Raw: ax=65 ay=-304 az=16612 \| gx=17 gy=-178 gz=-84 \| t=865 | 0.110s | PASS |  |
| SOAK-READ-002034 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.56 gz=-0.76 dps \| t=28.38 C | 0.109s | PASS |  |
| SOAK-ACCEL-002035 | soak/data | `accel` | Accel | Accel: x=0.004 y=-0.019 z=1.014 g (raw: 67 -306 16628) | 0.109s | PASS |  |
| SOAK-GYRO-002036 | soak/data | `gyro` | Gyro | Gyro: x=0.16 y=-1.56 z=-0.74 dps (raw: 18 -178 -85) | 0.110s | PASS |  |
| SOAK-TEMP-002037 | soak/data | `temp` | Temp | Temp: 28.37 C (raw: 862) | 0.110s | PASS |  |
| SOAK-STATUS-002038 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-002039 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SOAK-PROBE-002040 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SOAK-HEALTH-002041 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| SOAK-CFG-002042 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
| SOAK-SRC1-002043 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-002044 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-002045 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-002046 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-002047 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY \| === Driver Health === \| State: READY \| Online: yes \| ... | 125.953s | PASS | prompt recovered with health resync |
| SOAK-RECOVER-002048 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 834141 -> 834158 (+17) \| ... | 0.109s | PASS |  |
| SOAK-RAW-002049 | soak/data | `raw` | Raw: | Raw: ax=105 ay=-308 az=16617 \| gx=21 gy=-176 gz=-85 \| t=895 | 0.109s | PASS |  |
| SOAK-READ-002050 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.018 az=+1.014 g \| gx=+0.12 gy=-1.53 gz=-0.78 dps \| t=28.47 C | 0.109s | PASS |  |
| SOAK-ACCEL-002051 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.014 g (raw: 78 -314 16620) | 0.110s | PASS |  |
| SOAK-GYRO-002052 | soak/data | `gyro` | Gyro | Gyro: x=0.15 y=-1.54 z=-0.75 dps (raw: 17 -176 -86) | 0.109s | PASS |  |
| SOAK-TEMP-002053 | soak/data | `temp` | Temp | Temp: 28.46 C (raw: 887) | 0.094s | PASS |  |
| SOAK-STATUS-002054 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.094s | PASS |  |
| SOAK-FIFO-002055 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.094s | PASS |  |
| SOAK-PROBE-002056 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-002057 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
| SOAK-CFG-002058 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.110s | PASS |  |
| SOAK-SRC1-002059 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.109s | PASS |  |
| SOAK-TS-002060 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.109s | PASS |  |
| SOAK-STEPS-002061 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.110s | PASS |  |
| SOAK-STRESS-002062 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4740 reads=522 errors=0 accel=500 gyro=500 temp=251 health_success=5991 health_fail=0 state=READY | 4.782s | PASS |  |
| SOAK-MIX-002063 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.218s | PASS |  |
| SOAK-RECOVER-002064 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 840661 -> 840678 (+17) \| ... | 0.110s | PASS |  |
| SOAK-RAW-002065 | soak/data | `raw` | Raw: | Raw: ax=75 ay=-310 az=16627 \| gx=23 gy=-173 gz=-90 \| t=911 | 0.110s | PASS |  |
| SOAK-READ-002066 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.004 ay=-0.018 az=+1.015 g \| gx=+0.17 gy=-1.54 gz=-0.79 dps \| t=28.50 C | 0.093s | PASS |  |
| SOAK-ACCEL-002067 | soak/data | `accel` | Accel | Accel: x=0.003 y=-0.018 z=1.014 g (raw: 56 -297 16619) | 0.094s | PASS |  |
| SOAK-GYRO-002068 | soak/data | `gyro` | Gyro | Gyro: x=0.18 y=-1.58 z=-0.74 dps (raw: 21 -180 -85) | 0.094s | PASS |  |
| SOAK-TEMP-002069 | soak/data | `temp` | Temp | Temp: 28.51 C (raw: 899) | 0.094s | PASS |  |
| SOAK-STATUS-002070 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-002071 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-002072 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-002073 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-002074 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-002075 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.110s | PASS |  |
| SOAK-TS-002076 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-002077 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-002078 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4735 reads=521 errors=0 accel=500 gyro=500 temp=250 health_success=5985 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-002079 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.203s | PASS |  |
| SOAK-RECOVER-002080 | soak/recovery | `recover` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| TotalOK: 847175 -> 847192 (+17) \| ... | 0.094s | PASS |  |
| SOAK-RAW-002081 | soak/data | `raw` | Raw: | Raw: ax=80 ay=-311 az=16633 \| gx=22 gy=-176 gz=-87 \| t=906 | 0.094s | PASS |  |
| SOAK-READ-002082 | soak/data | `read` | Sample:, g, dps, C | Sample: ax=+0.005 ay=-0.019 az=+1.014 g \| gx=+0.18 gy=-1.51 gz=-0.76 dps \| t=28.53 C | 0.109s | PASS |  |
| SOAK-ACCEL-002083 | soak/data | `accel` | Accel | Accel: x=0.005 y=-0.019 z=1.015 g (raw: 75 -308 16635) | 0.110s | PASS |  |
| SOAK-GYRO-002084 | soak/data | `gyro` | Gyro | Gyro: x=0.19 y=-1.54 z=-0.78 dps (raw: 22 -176 -89) | 0.109s | PASS |  |
| SOAK-TEMP-002085 | soak/data | `temp` | Temp | Temp: 28.53 C (raw: 903) | 0.109s | PASS |  |
| SOAK-STATUS-002086 | soak/diagnostics | `status` | STATUS_REG | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.109s | PASS |  |
| SOAK-FIFO-002087 | soak/fifo | `fifo` | FIFO | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.110s | PASS |  |
| SOAK-PROBE-002088 | soak/diagnostics | `probe` | Status:, OK | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.109s | PASS |  |
| SOAK-HEALTH-002089 | soak/diagnostics | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SOAK-CFG-002090 | soak/config | `settings` | Driver state:, READY | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.109s | PASS |  |
| SOAK-SRC1-002091 | soak/diagnostics | `funcsrc1` | 0x | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
| SOAK-TS-002092 | soak/functions | `tsread` | Timestamp | Timestamp: 645124 | 0.093s | PASS |  |
| SOAK-STEPS-002093 | soak/functions | `steps` | Step | Pedometer: no \| Accel ODR: 104 Hz \| Steps: 0 \| Step timestamp: 0 \| ... | 0.094s | PASS |  |
| SOAK-STRESS-002094 | soak/stress | `stress 500 quiet` | RESULT stress, errors=0, state=READY | RESULT stress total=500 polls=4736 reads=520 errors=0 accel=500 gyro=500 temp=251 health_success=5987 health_fail=0 state=READY | 4.766s | PASS |  |
| SOAK-MIX-002095 | soak/stress | `stress_mix 500 quiet` | RESULT stress_mix, fail=0, state=READY | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY \| === Driver Health === \| State: READY \| Online: yes \| ... | 125.984s | PASS | prompt recovered with health resync |
