# Prompt 07 — Documentation hardening and hardware validation matrix

Work only on README/API docs, example labels, hardware validation matrix, and progress docs. Spawn subagents: `docs-contract-agent`, `hardware-matrix-agent`, `release-notes-agent`.

## Documentation requirements

Update docs to state clearly:

- library scope is I2C-only unless SPI was actually implemented;
- device has I2C/SPI capability, but library support is what code implements;
- I2C mode requires CS held high;
- SA0 high means logic high / VDD_IO domain;
- addresses `0x6A` and `0x6B`;
- bus speed: standard and 400 kHz fast mode only unless evidence supports more;
- core is framework-neutral and transport-injected;
- `nowMs` requirements;
- thread-safety and ISR-safety;
- API latency and transaction counts;
- FIFO word-by-word vs bounded drain latency;
- BDU/IF_INC behavior and limitations;
- reset/boot behavior and validation status;
- latest/fresh/stale measurement semantics;
- interrupt/DRDY latched/pulsed behavior;
- raw/expert register hazards and dirty-state behavior;
- examples are diagnostic/bring-up unless production-labeled;
- hardware validation is pending unless logs exist.

Create `docs/LSM6DS3TRC_HARDWARE_VALIDATION_MATRIX.md` with:

- board, commit, sensor marking, VDD, VDD_IO, pullups, bus speed, SA0, CS, INT wiring;
- command matrix: version, scan, whoami, cfg, state, status, accel/gyro/temp reads, ODR changes, all accel FS, all gyro FS, FIFO empty/fill/drain/overrun, INT1/INT2 DRDY, reset/boot/recover, address switch, unplug/replug/fault, long soak;
- evidence requirements: serial logs, register dumps, logic analyzer captures, final verdict.

Update `docs/LSM6DS3TRC_HARDENING_PROGRESS.md` with chunks completed and remaining work.

## Validate

```bash
python tools/check_core_framework_guard.py
python tools/check_idf_example_contract.py
python tools/check_cli_contract.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
```

Commit message:

```text
docs: document LSM6DS3TR-C production contracts and validation matrix
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
