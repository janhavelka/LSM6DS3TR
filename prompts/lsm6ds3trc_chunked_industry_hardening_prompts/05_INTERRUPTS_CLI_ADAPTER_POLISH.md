# Prompt 05 — Interrupt/DRDY contracts, CLI diagnostics, Arduino adapter polish

Work on typed interrupt/DRDY configuration, CLI diagnostics, and example adapter error details. Spawn subagents: `interrupt-agent`, `cli-agent`, `adapter-agent`, `docs-agent`.

## Context

DRDY can be latched or pulsed; pulsed mode produces 75 us pulses and can be missed under scheduler load. INT1/INT2 can route accel DRDY, gyro DRDY, temperature DRDY, FIFO threshold/full/overrun, boot, and embedded-function events. The examples are diagnostic and should not look like production bus managers.

## Implement

1. Add/verify typed safe interrupt routing for common cases:
   - accel DRDY; gyro DRDY; temp DRDY; FIFO threshold; FIFO full; FIFO overrun; boot if useful.
2. Keep embedded-function interrupt routing unsupported or clearly separate unless fully modeled.
3. Document latched vs pulsed DRDY and 75 us pulse-capture risk.
4. Document polarity/open-drain/push-pull behavior if managed.
5. Improve CLI diagnostics if present:
   - `version`, `scan`, `addr`, `whoami`, `cfg`, `state`, `status`, `read`, `fifo status`, `fifo drain N`, `int cfg`, `selftest` if implemented.
   - Print active address, status code, detail, and message.
6. Improve Arduino example adapter mapping for timeout too large, short read, NACK, timeout, bus error where possible.
7. Label `examples/common/` and CLI as diagnostic/bring-up, not production shared-bus glue.

## Tests

Add tests for interrupt bit encodings, invalid route combinations, CLI contract script updates, and adapter timeout/short-read mapping if testable.

## Validate

```bash
python tools/check_core_framework_guard.py
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python -m platformio test -e native
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
```

Commit message:

```text
examples: improve LSM6DS3TR-C interrupt diagnostics
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
