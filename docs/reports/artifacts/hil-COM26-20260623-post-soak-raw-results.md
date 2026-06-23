# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-23T05:08:56.495762+02:00
- Ended: 2026-06-23T05:08:56.843506+02:00
- Suite: smoke
- Port: COM26
- Baud: 115200
- Result counts: PASS=2, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=2, min=0.109s, mean=0.109s, max=0.110s
- Transcript: `docs/reports/artifacts/hil-COM26-20260623-post-soak-raw-transcript.txt`

## Boot Excerpt

```text

```

## Results

| Test ID | Feature | Command | Expected | Observed | Elapsed | Result | Notes |
|---|---|---|---|---|---:|---|---|
| CMD-001 | custom | `raw` | No failure tokens | Raw: ax=75 ay=-313 az=16700 \| gx=19 gy=-186 gz=-113 \| t=1595 | 0.110s | PASS |  |
| CMD-002 | custom | `health` | No failure tokens | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
