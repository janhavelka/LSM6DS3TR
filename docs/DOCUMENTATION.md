# Documentation Map

This directory contains maintained guidance, authoritative vendor sources,
curated chip notes, and locally generated API pages. They do not all have the
same authority.

## Maintained Contracts And Guidance

- [Project README](../README.md): supported behavior, ownership contract,
  installation, examples, and validation.
- [Public API headers](../include/LSM6DS3TR): Doxygen contracts for supported
  types and operations.
- [ESP-IDF port guide](IDF_PORT.md): native component and application-owned
  transport boundary.
- [HIL validation guide](https://github.com/janhavelka/LSM6DS3TR/blob/main/docs/HIL_VALIDATION.md): repeatable targeted and
  owner-soak procedures plus retained validation evidence.
- <a href="chip-reference/README.md">Maintained chip reference</a>:
  source-audited,
  AI-facing translation of LSM6DS3TR-C registers, modes, conversions, timing,
  procedures, vendor ambiguities, and library support boundaries.
Changes to supported behavior must update the applicable public Doxygen,
README, guide, and <a href="../CHANGELOG.md">changelog</a> in the same change.

## Generated Reference

- `docs/doxygen/`: ignored local output produced by
  `python tools/build_docs.py`. Do not edit or commit it.

## Repository-Only Audit Material

- The official [datasheet](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf)
  and [AN5130](https://www.st.com/resource/en/application_note/dm00472670-lsm6ds3trc-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf)
  PDFs, their mechanically extracted text, and local hashes are retained in the
  repository for offline source audit. They are deliberately excluded from the
  library package.
- The [legacy monolithic extraction](https://github.com/janhavelka/LSM6DS3TR/blob/main/docs/archive/LSM6DS3TR_imu_legacy_extraction.md)
  preserves previously collected detail but contains known unsafe statements.
  It is not an implementation source; the maintained chip reference supersedes
  it.
- Vendor design-tip PDFs are optional application-math background. They do not
  define this chip's register map or the production library contract.

Vendor sources define silicon facts but do not automatically define the library
API. When they conflict with a maintained contract, update the owning chip
reference topic, ambiguity ledger, code/tests if affected, and changelog in the
same change.
