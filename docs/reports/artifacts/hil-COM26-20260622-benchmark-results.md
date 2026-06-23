# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-22T20:26:59.892696+02:00
- Ended: 2026-06-22T20:27:07.509087+02:00
- Suite: smoke
- Port: COM26
- Baud: 115200
- Result counts: PASS=4, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=4, min=0.109s, mean=1.309s, max=4.797s
- Transcript: `docs/reports/artifacts/hil-COM26-20260622-benchmark-transcript.txt`

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
| BENCH-001 | benchmark | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| BENCH-002 | benchmark | `stress 500` | Stress Summary, Errors: | Progress: 50/500 (10%, ok=50, fail=0) \| Progress: 100/500 (20%, ok=100, fail=0) \| Progress: 150/500 (30%, ok=150, fail=0) \| Progress: 200/500 (40%, ok=200, fail=0) \| ... | 4.797s | PASS |  |
| BENCH-003 | benchmark | `stress_mix 500` | Mixed Stress Summary, Errors: | Progress: 50/500 (10%, ok=50, fail=0) \| Progress: 100/500 (20%, ok=100, fail=0) \| Progress: 150/500 (30%, ok=150, fail=0) \| Progress: 200/500 (40%, ok=200, fail=0) \| ... | 0.218s | PASS |  |
| BENCH-004 | benchmark | `health` | State:, READY | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.110s | PASS |  |
