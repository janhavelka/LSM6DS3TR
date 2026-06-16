# Document Inventory

Compact notes in `docs/extracted-md/` summarize LSM6DS3TR-C register, bus, timing, FIFO, interrupt, and scale facts. Raw PDF text lives in `docs/pdf-extracted-md/`.

The compact notes are a curated index, not a replacement for the PDF tables. They must keep stable anchors for library-relevant registers and features so edits do not lose important datasheet coverage; `tools/check_chip_docs_coverage.py` enforces those anchors in CI.

| Source PDF | Raw extract | Role | Revision / date | Pages | Notes |
|---|---|---|---|---:|---|
| `docs/LSM6DS3TR-C_datasheet.pdf` | `docs/pdf-extracted-md/LSM6DS3TR-C_datasheet.md` | Primary datasheet | Rev. 3, May 2017 | 114 | Electrical specs, pinout, I2C/SPI protocol, register map, FIFO, interrupts, embedded functions. |
| `docs/LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.pdf` | `docs/pdf-extracted-md/LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.md` | Supplementary application note | Rev. 1, March 2018 | 109 | Configuration examples and embedded-feature guidance. |
| `docs/design_tips/calibration_1_or_3_point.pdf` | `docs/pdf-extracted-md/calibration_1_or_3_point.md` | MEMS design tip | Rev. 2, January 2022 | 5 | Calibration background for accelerometer/gyroscope workflows; no LSM6DS3TR-C register definitions. |
| `docs/design_tips/computing_tilt.pdf` | `docs/pdf-extracted-md/computing_tilt.md` | General MEMS design tip | Rev. 3, January 2021 | 6 | General tilt computation guidance. |
| `docs/design_tips/dead_reckoning.pdf` | `docs/pdf-extracted-md/dead_reckoning.md` | General MEMS design tip | Rev. 1, August 2018 | 6 | General dead-reckoning background. |
| `docs/design_tips/noise_analysis.pdf` | `docs/pdf-extracted-md/noise_analysis.md` | General MEMS design tip | Rev. 1, July 2016 | 6 | Noise analysis background. |
| `docs/design_tips/updating_tilt_measurement.pdf` | `docs/pdf-extracted-md/updating_tilt_measurement.md` | General MEMS design tip | Rev. 3, January 2021 | 7 | General tilt update guidance. |

## Compact Note Set

| File | Purpose |
|---|---|
| `01_chip_overview.md` | Sensor purpose, ranges, ODRs, FIFO, embedded features. |
| `02_pinout_and_signals.md` | 14-pin LGA pinout, I2C/SPI selection, address strap, interrupt pins. |
| `03_electrical_and_timing.md` | Supply, current, sensitivities, timing, I2C/SPI limits. |
| `04_protocol_commands_and_transactions.md` | I2C/SPI transaction formats, address bytes, auto-increment behavior. |
| `05_register_map.md` | Core user-interface register map and bit-field summaries. |
| `06_modes_interrupts_status_and_faults.md` | ODR/power modes, status, interrupts, FIFO, self-test and embedded functions. |
| `07_initialization_reset_and_operational_notes.md` | Startup, reset, recommended configuration sequence, readout notes. |
| `08_variant_differences_and_open_questions.md` | Naming/revision notes, supplemental document boundaries, open questions. |

## Scope Notes

- The primary datasheet is authoritative for register addresses and bit meanings in these compact notes.
- Design-tip PDFs are not used for LSM6DS3TR-C register addresses, reset values, or bit meanings unless a compact note names the specific source.
- Units use ASCII-friendly forms such as `uA`, `degC`, and `+/-`.
