# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-22T20:43:49.968214+02:00
- Ended: 2026-06-22T20:43:50.316319+02:00
- Suite: smoke
- Port: COM26
- Baud: 115200
- Result counts: PASS=2, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=2, min=0.094s, mean=0.094s, max=0.094s
- Transcript: `docs/reports/artifacts/hil-COM26-20260622-post-soak-fail2-transcript.txt`

## Boot Excerpt

```text

```

## Results

| Test ID | Feature | Command | Expected | Observed | Elapsed | Result | Notes |
|---|---|---|---|---|---:|---|---|
| CMD-001 | custom | `health` | No failure tokens | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.094s | PASS |  |
| CMD-002 | custom | `funcsrc1` | No failure tokens | funcsrc1 = 0x00 \| step delta           no \| significant motion   no \| tilt                 no \| ... | 0.094s | PASS |  |
