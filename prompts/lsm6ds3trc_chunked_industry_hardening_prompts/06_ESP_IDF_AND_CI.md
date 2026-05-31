# Prompt 06 — Pure ESP-IDF example, CI, and portability checks

Work only on ESP-IDF integration and CI/build coverage. Core must remain framework-neutral. Spawn subagents: `idf-port-agent`, `idf-transport-agent`, `ci-agent`, `contract-agent`.

## Implement

1. Add root component files if missing: `CMakeLists.txt`, `idf_component.yml`.
2. Add minimal pure ESP-IDF example under `examples/esp_idf/basic/`.
3. Example must use native IDF APIs, `app_main`, IDF time source such as `esp_timer_get_time()`, injected I2C callbacks, timeout/error mapping, and mutex/locking if demonstrating shared bus or multitask.
4. No Arduino headers, `Wire`, or compatibility layers in ESP-IDF example.
5. Add `tools/check_idf_example_contract.py` to enforce boundaries and diagnostic/production labeling.
6. Add CI jobs or documented workflows for native tests, Arduino ESP32-S2/S3 builds, guard scripts, and pure ESP-IDF builds for ESP32-S2/S3 if feasible.
7. If local `idf.py` is unavailable, record exact failure. Do not claim local pure-IDF pass.

## Validate

```bash
python tools/check_core_framework_guard.py
python tools/check_idf_example_contract.py
python -m platformio test -e native
python -m platformio run -e esp32s3dev
python -m platformio run -e esp32s2dev
idf.py --version
idf.py -C examples/esp_idf/basic set-target esp32s3 build
idf.py -C examples/esp_idf/basic set-target esp32s2 build
```

Commit message:

```text
idf: add pure ESP-IDF LSM6DS3TR-C example and CI checks
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
