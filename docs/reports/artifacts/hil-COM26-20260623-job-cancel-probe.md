# LSM6DS3TR HIL Runner Output - COM26

- Started: 2026-06-23T05:28:27.396742+02:00
- Ended: 2026-06-23T05:28:30.279010+02:00
- Suite: smoke
- Port: COM26
- Baud: 115200
- Result counts: PASS=5, FAIL=0, UNKNOWN=0, NOT_RUN=0
- Timing: count=5, min=0.093s, mean=0.106s, max=0.110s
- Transcript: `docs/reports/artifacts/hil-COM26-20260623-job-cancel-probe-transcript.txt`

## Boot Excerpt

```text

```

## Results

| Test ID | Feature | Command | Expected | Observed | Elapsed | Result | Notes |
|---|---|---|---|---|---:|---|---|
| CMD-001 | custom | `job auto 0` | No failure tokens | Automatic tick polling: no \| Job auto: busy=no ready=no hasSample=no manual=yes \| Last poll: OK (code=0, detail=0) \| Last poll msg: OK \| ... | 0.109s | PASS |  |
| CMD-002 | custom | `job start direct` | No failure tokens | Status: IN_PROGRESS (code=10, detail=0) \| Message: Measurement scheduled \| Job start: busy=yes ready=no hasSample=no manual=yes \| Last poll: IN_PROGRESS (code=10, detail=0) \| ... | 0.093s | PASS |  |
| CMD-003 | custom | `job cancel` | No failure tokens | Job manual mode cleared. \| Job cancel: busy=yes ready=no hasSample=no manual=no \| Last poll: IN_PROGRESS (code=10, detail=0) \| Last poll msg: Measurement scheduled \| ... | 0.110s | PASS |  |
| CMD-004 | custom | `job status` | No failure tokens | Job status: busy=no ready=yes hasSample=yes manual=no \| Last poll: OK (code=0, detail=0) \| Last poll msg: OK \| Health: state=READY ok=1 fail=0 consecutive=0 | 0.109s | PASS |  |
| CMD-005 | custom | `health` | No failure tokens | === Driver Health === \| State: READY \| Online: yes \| Consecutive failures: 0 \| ... | 0.109s | PASS |  |
