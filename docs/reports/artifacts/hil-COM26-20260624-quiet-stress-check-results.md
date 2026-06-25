# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-24T22:51:03.269553+02:00
- Ended: 2026-06-24T22:51:06.470540+02:00
- Suite: smoke
- Port: COM26
- Baud: 115200
- Result counts: PASS=3, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=3, min=0.093s, mean=0.286s, max=0.547s
- Transcript: `docs/reports/artifacts/hil-COM26-20260624-quiet-stress-check-transcript.txt`

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
| CMD-001 | custom | `stress 50 quiet` | No failure tokens | RESULT stress total=50 polls=458 reads=52 errors=0 accel=50 gyro=50 temp=26 health_success=584 health_fail=0 state=READY | 0.547s | PASS |  |
| CMD-002 | custom | `stress_mix 500 quiet` | No failure tokens | RESULT stress_mix total=500 ok=500 fail=0 health_success=500 health_fail=0 state=READY | 0.219s | PASS |  |
| CMD-003 | custom | `health` | No failure tokens | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.093s | PASS |  |
