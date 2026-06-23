# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-23T05:08:41.411386+02:00
- Ended: 2026-06-23T05:08:41.759761+02:00
- Suite: smoke
- Port: COM26
- Baud: 115200
- Result counts: PASS=2, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=2, min=0.093s, mean=0.101s, max=0.109s
- Transcript: `docs/reports/artifacts/hil-COM26-20260623-post-soak-health-transcript.txt`

## Boot Excerpt

```text

```

## Results

| Test ID | Feature | Command | Expected | Observed | Elapsed | Result | Notes |
|---|---|---|---|---|---:|---|---|
| CMD-001 | custom | `health` | No failure tokens | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| CMD-002 | custom | `settings` | No failure tokens | === Current Settings === \| Initialized:        yes \| Driver state:       READY \| I2C address:        0x6A \| ... | 0.093s | PASS |  |
