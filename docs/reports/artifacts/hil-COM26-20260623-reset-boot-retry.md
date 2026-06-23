# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-23T05:27:44.629515+02:00
- Ended: 2026-06-23T05:27:47.373930+02:00
- Suite: smoke
- Port: COM26
- Baud: 115200
- Result counts: PASS=4, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=4, min=0.094s, mean=0.106s, max=0.110s
- Transcript: `docs/reports/artifacts/hil-COM26-20260623-reset-boot-retry-transcript.txt`

## Boot Excerpt

```text

```

## Results

| Test ID | Feature | Command | Expected | Observed | Elapsed | Result | Notes |
|---|---|---|---|---|---:|---|---|
| CMD-001 | custom | `job run reset 1 60 2` | No failure tokens | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=reset budget=1 polls=18 limit=60 \| ... | 0.109s | PASS |  |
| CMD-002 | custom | `health` | No failure tokens | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
| CMD-003 | custom | `job run boot 1 60 2` | No failure tokens | Start: \| Status: IN_PROGRESS (code=10, detail=0) \| Message: Poll job scheduled \| Job run: kind=boot budget=1 polls=13 limit=60 \| ... | 0.110s | PASS |  |
| CMD-004 | custom | `health` | No failure tokens | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
