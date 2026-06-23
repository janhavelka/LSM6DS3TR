# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-23T05:27:01.702702+02:00
- Ended: 2026-06-23T05:27:28.076070+02:00
- Suite: smoke
- Port: COM26
- Baud: 115200
- Result counts: PASS=26, FAIL=1, UNKNOWN=0, NOT_RUN=0
- Timing: count=27, min=0.109s, mean=0.848s, max=20.062s
- Transcript: `docs/reports/artifacts/hil-COM26-20260623-poll-edge-retest-transcript.txt`

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
| CMD-001 | custom | `health` | No failure tokens | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| CMD-002 | custom | `job status` | No failure tokens | Job status: busy=no ready=no hasSample=no manual=no \| Last poll: OK (code=0, detail=0) \| Last poll msg: OK \| Health: state=READY ok=0 fail=0 consecutive=0 | 0.110s | PASS |  |
| CMD-003 | custom | `job auto 0` | No failure tokens | Automatic tick polling: no \| Job auto: busy=no ready=no hasSample=no manual=yes \| Last poll: OK (code=0, detail=0) \| Last poll msg: OK \| ... | 0.109s | PASS |  |
| CMD-004 | custom | `job start direct` | No failure tokens | Status: IN_PROGRESS (code=10, detail=0) \| Message: Measurement scheduled \| Job start: busy=yes ready=no hasSample=no manual=yes \| Last poll: IN_PROGRESS (code=10, detail=0) \| ... | 0.109s | PASS |  |
| CMD-005 | custom | `job get` | No failure tokens | Status: MEASUREMENT_NOT_READY (code=8, detail=0) \| Message: No sample available \| Job get: busy=yes ready=no hasSample=no manual=yes \| Last poll: IN_PROGRESS (code=10, detail=0)... | 0.109s | PASS |  |
| CMD-006 | custom | `job poll 0 1` | No failure tokens | Poll 1/1 budget=0 -> IN_PROGRESS busy=yes ready=no \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job in progress \| Job poll: busy=yes ready=no hasSample=no manual=ye... | 0.110s | PASS |  |
| CMD-007 | custom | `job poll 1 1` | No failure tokens | Poll 1/1 budget=1 -> OK busy=no ready=yes \| Status: OK (code=0, detail=0) \| Message: OK \| Job poll: busy=no ready=yes hasSample=yes manual=no \| ... | 0.109s | PASS |  |
| CMD-008 | custom | `job getraw` | No failure tokens | Cached raw: ax=71 ay=-325 az=16701 gx=18 gy=-183 gz=-117 t=1594 ts=2778 \| Job getraw: busy=no ready=yes hasSample=yes manual=no \| Last poll: OK (code=0, detail=0) \| Last poll ms... | 0.109s | PASS |  |
| CMD-009 | custom | `job get` | No failure tokens | Sample: ax=+0.004 ay=-0.020 az=+1.019 g \| gx=+0.16 gy=-1.60 gz=-1.02 dps \| t=31.23 C \| Status: OK (code=0, detail=0) \| Message: OK \| Job get: busy=no ready=no hasSample=yes manu... | 0.109s | PASS |  |
| CMD-010 | custom | `job run sample 1 40 2` | No failure tokens | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Measurement scheduled \| Job run: kind=sample budget=1 polls=2 limit=40 \| ... | 0.110s | PASS |  |
| CMD-011 | custom | `job getraw` | No failure tokens | Cached raw: ax=67 ay=-313 az=16683 gx=14 gy=-181 gz=-115 t=1591 ts=3249 \| Job getraw: busy=no ready=yes hasSample=yes manual=no \| Last poll: OK (code=0, detail=0) \| Last poll ms... | 0.109s | PASS |  |
| CMD-012 | custom | `job run sample 255 3` | No failure tokens | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Measurement scheduled \| Job run: kind=sample budget=255 polls=1 limit=3 \| ... | 0.109s | PASS |  |
| CMD-013 | custom | `job start direct` | No failure tokens | Status: IN_PROGRESS (code=10, detail=0) \| Message: Measurement scheduled \| Job start: busy=yes ready=no hasSample=yes manual=yes \| Last poll: IN_PROGRESS (code=10, detail=0) \| ... | 0.109s | PASS |  |
| CMD-014 | custom | `job start refresh` | No failure tokens | Status: BUSY (code=9, detail=0) \| Message: Poll job in progress \| Job start: busy=yes ready=no hasSample=yes manual=yes \| Last poll: IN_PROGRESS (code=10, detail=0) \| ... | 0.110s | PASS |  |
| CMD-015 | custom | `job poll 1 3` | No failure tokens | Poll 1/3 budget=1 -> OK busy=no ready=yes \| Status: OK (code=0, detail=0) \| Message: OK \| Job poll: busy=no ready=yes hasSample=yes manual=no \| ... | 0.109s | PASS |  |
| CMD-016 | custom | `job start fifo 0` | No failure tokens | Status: INVALID_PARAM (code=5, detail=0) \| Message: maxWords must be > 0 \| Job start: busy=no ready=yes hasSample=yes manual=no \| Last poll: OK (code=0, detail=0) \| ... | 0.109s | PASS |  |
| CMD-017 | custom | `job start calxl 0` | No failure tokens | Status: INVALID_PARAM (code=5, detail=0) \| Message: samples must be 1..10000 \| Job start: busy=no ready=yes hasSample=yes manual=no \| Last poll: OK (code=0, detail=0) \| ... | 0.109s | PASS |  |
| CMD-018 | custom | `job start bogus` | No failure tokens | Invalid job kind | 0.110s | PASS |  |
| CMD-019 | custom | `job poll 999` | No failure tokens | Invalid poll budget | 0.109s | PASS |  |
| CMD-020 | custom | `job run direct 0 3 0` | No failure tokens | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Measurement scheduled \| Job run: kind=direct budget=0 polls=3 limit=3 \| ... | 0.109s | PASS |  |
| CMD-021 | custom | `job poll 1 3` | No failure tokens | Poll 1/3 budget=1 -> OK busy=no ready=yes \| Status: OK (code=0, detail=0) \| Message: OK \| Job poll: busy=no ready=yes hasSample=yes manual=no \| ... | 0.109s | PASS |  |
| CMD-022 | custom | `job run refresh 1 30` | No failure tokens | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=refresh budget=1 polls=11 limit=30 \| ... | 0.110s | PASS |  |
| CMD-023 | custom | `job run refresh 255 3` | No failure tokens | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=refresh budget=255 polls=1 limit=3 \| ... | 0.109s | PASS |  |
| CMD-024 | custom | `job run reset 1 60 2` | No failure tokens | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=reset budget=1 polls=18 limit=60 \| ... | 20.062s | FAIL | Prompt not seen before timeout |
| CMD-025 | custom | `job run boot 1 60 2` | No failure tokens | poll msg: OK \| Health: state=READY ok=46 fail=0 consecutive=0 \| Start: \| Status: IN_PROGRESS (code=10, detail=0) \| ... | 0.110s | PASS |  |
| CMD-026 | custom | `job auto 1` | No failure tokens | Automatic tick polling: yes \| Job auto: busy=no ready=no hasSample=no manual=no \| Last poll: OK (code=0, detail=0) \| Last poll msg: OK \| ... | 0.110s | PASS |  |
| CMD-027 | custom | `health` | No failure tokens | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
