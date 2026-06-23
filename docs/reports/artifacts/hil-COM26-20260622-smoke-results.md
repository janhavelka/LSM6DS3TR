# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-22T20:23:49.714498+02:00
- Ended: 2026-06-22T20:23:55.559446+02:00
- Suite: smoke
- Port: COM26
- Baud: 115200
- Result counts: PASS=10, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=10, min=0.093s, mean=0.225s, max=1.312s
- Transcript: `docs/reports/artifacts/hil-COM26-20260622-smoke-transcript.txt`

## Boot Excerpt

```text

```

## Results

| Test ID | Feature | Command | Expected | Observed | Elapsed | Result | Notes |
|---|---|---|---|---|---:|---|---|
| SMK-01 | smoke | `version` | No failure tokens | === Version === \| Version: 1.1.0 \| Full:    1.1.0 (f10e8e4, 2026-06-22 20:17:40, clean) \| Built:   2026-06-22 20:17:40 \| ... | 0.109s | PASS |  |
| SMK-02 | smoke | `scan` | No failure tokens | [I] Scanning I2C bus (timeout=50ms)... \| [I]      0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F \| 00:                         -- -- -- -- -- -- -- -- \| 10: -- -- -- -- -- -- --... | 0.093s | PASS |  |
| SMK-03 | smoke | `probe` | No failure tokens | Status: OK (code=0, detail=0) \| Message: OK \| Health changes: \| (no health changes) | 0.094s | PASS |  |
| SMK-04 | smoke | `settings` | No failure tokens | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.094s | PASS |  |
| SMK-05 | smoke | `health` | No failure tokens | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| SMK-06 | smoke | `whoami` | No failure tokens | WHO_AM_I = 0x6A expected=0x6A match=YES | 0.109s | PASS |  |
| SMK-07 | smoke | `status` | No failure tokens | STATUS_REG = 0x07 (XLDA=1 GDA=1 TDA=1) | 0.110s | PASS |  |
| SMK-08 | smoke | `raw` | No failure tokens | Raw: ax=85 ay=-301 az=16689 \| gx=18 gy=-183 gz=-117 \| t=1121 | 0.110s | PASS |  |
| SMK-09 | smoke | `fifo` | No failure tokens | === FIFO === \| Mode: bypass \| ODR:  POWER_DOWN \| Threshold: 0 \| ... | 0.109s | PASS |  |
| SMK-10 | smoke | `selftest` | Selftest result: | === LSM6DS3TR selftest (safe commands) === \| [PASS] probe responds \| [PASS] probe no-health-side-effects \| [PASS] readWhoAmI \| ... | 1.312s | PASS |  |
