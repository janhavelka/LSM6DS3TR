# Prompt 08 — Final integration review and hardening final report

This is the final software-integration prompt. Do not add features unless a small fix is needed to make earlier changes coherent. Spawn subagents: `diff-review-agent`, `test-review-agent`, `docs-review-agent`, `release-gate-agent`.

## Full validation

Run:

```bash
git diff --stat
git diff --check
git status --short
python tools/check_core_framework_guard.py
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
python -m platformio pkg pack
idf.py --version
idf.py -C examples/esp_idf/basic set-target esp32s3 build
idf.py -C examples/esp_idf/basic set-target esp32s2 build
```

If `idf.py` is unavailable, record exact reason. Remove generated tarballs after package validation unless intentionally managed.

## Final report

Create `docs/LSM6DS3TRC_HARDENING_FINAL_REPORT.md` with:

- date, branch, commit;
- summary;
- audit findings addressed mapped to H1..H8/M/L findings;
- public API changes;
- core changes;
- status/health/dirty-state changes;
- FIFO changes;
- reset/timing/freshness changes;
- ESP-IDF/Arduino/example changes;
- docs changes;
- tests added;
- CI/build coverage;
- exact local command results;
- checks not run and why;
- hardware validation status, with no invented results;
- remaining work before production claim;
- merge readiness;
- release readiness.

## Final gate

No generated package artifacts should remain unless intentionally ignored. README/docs must not claim industry-grade if hardware/fault validation is still pending.

Commit message:

```text
docs: add LSM6DS3TR-C hardening final report
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
