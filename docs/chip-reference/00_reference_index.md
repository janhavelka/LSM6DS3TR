# LSM6DS3TR-C Maintained Chip Reference

This is the repository's AI- and engineer-facing translation of the
LSM6DS3TR-C silicon documentation. Read it before changing register constants,
configuration validation, conversion math, operation timing, or chip-facing
behavior. It is designed to make routine library work possible without
reinterpreting the vendor PDFs.

This reference describes the chip, not just this library. Every topic therefore
keeps two separate labels where the distinction matters:

- **Source confidence:** `vendor fact`, `vendor ambiguity`, or
  `engineering inference`.
- **Library status:** `supported`, `diagnostic-only`, or `unsupported`.

`unsupported` means the silicon has the capability but the production library
does not expose a typed, replayable contract for it. It does not mean that the
datasheet claim is false.

## Current Official Source Baseline

Verified against ST's live product documentation on **2026-07-31**. ST still
publishes the same revisions; there is no newer LSM6DS3TR-C datasheet revision
to migrate to.

| Authority | Official source | Revision / date | Official pages | Repository evidence |
|---|---|---|---:|---|
| Primary silicon definition | [LSM6DS3TR-C datasheet](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf) | DocID030071 Rev 3, May 2017 | 115 | `docs/LSM6DS3TR-C_datasheet.pdf`, SHA-256 `352f35643f90772781540cf625ee574eb41d8249707440ac200632294b3b773c`; PDFium-normalized 114-page copy omitting only ST's final legal notice page |
| Programming procedures | [AN5130](https://www.st.com/resource/en/application_note/dm00472670-lsm6ds3trc-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf) | AN5130 Rev 1, March 2018 | 109 | `docs/LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.pdf`, SHA-256 `15b90b0ccca09f1cd6f561ecc67c7620714f452ad3f2c07e0f1cbce7e1ebb1c6` |
| Implementation cross-check | [ST standard C driver](https://github.com/STMicroelectronics/lsm6ds3tr-c-pid) | master commit `83f1e3f783a9d4693705f3559afd147fca774062`, 2026-04-09 | n/a | Secondary cross-check only; it does not override the datasheet |

The local datasheet contains all 114 numbered technical pages, including the
revision history. Its only missing official PDF page is page 115, ST's generic
legal notice. Mechanically extracted source text is retained under
`docs/pdf-extracted-md/` for offline audit, but neither the PDFs nor raw extracts
are packaged as the maintained reference.

## Authority And Citation Rules

1. Datasheet Rev 3 register tables and field descriptions define addresses,
   writable bits, encodings, reset values, electrical limits, and sensitivities.
2. AN5130 defines multi-step recipes, filter behavior, settling/discard rules,
   FIFO interpretation, and self-test sequences.
3. [Source ambiguities](12_source_ambiguities.md) records contradictions inside
   ST's own documents and the resolution used by this library.
4. `include/LSM6DS3TR/CommandTable.h` and the implementation must agree with
   this reference. They are not allowed to silently redefine the silicon.
5. Citations use the page number printed by ST, not a zero-based PDF index.
   Prefer a section, table, or register name as well as the page.
6. General MEMS design tips may inform application math, but never define this
   chip's register addresses, bit positions, defaults, or mandatory sequences.

When a maintained note conflicts with an ST source, stop and update the note,
the ambiguity ledger, the affected code/test, and the changelog together. Do
not choose whichever statement is more convenient.

## Reading Order For AI Coders

| Topic | Purpose |
|---|---|
| [Chip overview](01_chip_overview.md) | Identity, sensing blocks, ranges, ODRs, FIFO, and embedded capabilities. |
| [Pinout and signals](02_pinout_and_signals.md) | 14-pin LGA, supplies, host interface selection, address strap, and interrupt pins. |
| [Electrical and timing](03_electrical_and_timing.md) | Operating limits, current, sensitivities, bus limits, and conversion anchors. |
| [Protocols](04_protocol_commands_and_transactions.md) | Exact I2C/SPI transaction model, auto-increment, byte order, and safe output reads. |
| [Register map](05_register_map.md) | User-bank addresses, important bit fields, encodings, and reserved-space rules. |
| [Modes, interrupts, and faults](06_modes_interrupts_status_and_faults.md) | Power modes, ready/status behavior, event routing, FIFO state, and self-test encodings. |
| [Initialization and reset](07_initialization_reset_and_operational_notes.md) | Bring-up, reset/boot, output handling, and configuration dependencies. |
| [Source and variant notes](08_variant_and_source_notes.md) | Device naming, source priority, embedded-bank anchors, and revision scope. |
| [Filters and settling](09_filters_and_settling.md) | Filter ownership, transition gates, sample-discard rules, and data-valid timing. |
| [FIFO and embedded engines](10_fifo_and_embedded_functions.md) | FIFO pattern model, event engines, timestamp, and sensor-hub dependencies. |
| [Self-test and calibration](11_self_test_and_calibration.md) | Vendor self-test procedure and the separate software-calibration contract. |
| [Source ambiguities](12_source_ambiguities.md) | Explicit ST contradictions, naming differences, and conservative resolutions. |
| [Library support matrix](13_library_support_matrix.md) | Which chip capabilities are supported, diagnostic-only, or unsupported here. |

## Maintenance And Distribution

- `tools/check_chip_docs_coverage.py` verifies the source manifest, critical
  semantic facts, high-risk register fields, support labels, and agreement with
  `CommandTable.h`.
- The curated `docs/chip-reference/` tree is included in library packages and
  generated documentation. Vendor PDFs and raw text extracts remain
  repository-only audit artifacts.
- A source update must record the new official revision, date, page count,
  source URL, local SHA-256, changed datasheet revision-history entries, and a
  fresh semantic audit. A matching filename alone is not evidence of alignment.
