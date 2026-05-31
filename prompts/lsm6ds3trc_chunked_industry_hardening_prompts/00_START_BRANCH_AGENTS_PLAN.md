# Prompt 00 — Start hardening branch, baseline, AGENTS.md, hardening plan

You are working in the LSM6DS3TR-C I2C library repository. The exploration audit has already classified the repo as **engineering-grade with major gaps**, not production-ready. Do not try to fix everything in one pass. This prompt is setup only.

## Start

Run:

```bash
git status --short
git branch --show-current
git remote -v
```

If the tree is dirty, stop and report the dirty files. If clean, create:

```bash
git checkout -b hardening/lsm6ds3trc-industry-readiness
```

If the branch already exists, do not delete it; switch only if clearly intended.

## Spawn subagents

Spawn: `core-contracts-agent`, `device-safety-agent`, `tests-ci-agent`, `docs-agent`. Ask each to read the audit report and return factual findings only.

## AGENTS.md

Create/update `AGENTS.md` with LSM6DS3TR-C-specific rules:

- core under `include/` and `src/` must be framework-neutral: no Arduino/Wire/ESP-IDF/FreeRTOS/String/logging/global bus/pin ownership/tasks/delay/millis;
- core must use injected non-owning I2C transport;
- all fallible APIs return precise `Status`;
- timebase requirements must be explicit; no hidden Arduino fallback;
- driver instances are not thread-safe; public APIs are not ISR-safe unless proven;
- `FUNC_CFG_ACCESS`, `CTRL3_C`, `CTRL4_C`, BDU, IF_INC, BLE, SW_RESET, BOOT, and `I2C_disable` are protected/managed state;
- raw/expert register writes must not leave hidden bank/bus/control state unsafe without dirty-state reporting;
- multi-register writes must surface partial hardware state and provide recover/resync;
- FIFO APIs must document and bound bus latency;
- examples are diagnostic/bring-up unless they implement production bus ownership/locking/error policy;
- hardware-validation claims require logs with commit hash, board, voltage, bus speed, pullups, address, and command sequence.

## Hardening plan

Create `docs/LSM6DS3TRC_HARDENING_PLAN.md` with:

- branch name;
- audit summary;
- chunk sequence 00..08;
- blockers from the audit: Arduino-bound core, unsafe raw register writes, missing partial-apply dirty state, FIFO 4th dataset, reset/boot uncertainty, narrow status vocabulary, FIFO blocking, no hardware evidence;
- rule: each chunk gets committed and synced before the next;
- final report path: `docs/LSM6DS3TRC_HARDENING_FINAL_REPORT.md`.

No source-code fixes in this prompt.

## Validate

```bash
python --version
python -m platformio --version
git status --short
```

Commit message:

```text
docs: start LSM6DS3TR-C industry hardening plan
```


---

## Required finish for this prompt

Run the requested checks, then:

```bash
git diff --check
git status --short
git add <changed files>
git commit -m "<requested commit message>"
git push -u origin HEAD
```

If push/sync fails, report the exact failure and local commit hash. Final response must include branch, commit, files changed, tests/checks run with exact results, and what remains next.
