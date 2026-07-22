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
- [Implementation manual](../LSM6DS3TR_imu_implementation_manual.md): chip and
  integration guidance used by this repository.

Changes to supported behavior must update the applicable public Doxygen,
README, guide, and <a href="../CHANGELOG.md">changelog</a> in the same change.

## Generated Reference

- `docs/doxygen/`: ignored local output produced by
  `python tools/build_docs.py`. Do not edit or commit it.

## Source Reference Material

- <a href="extracted-md/00_document_inventory.md">Extracted document
  inventory</a>: compact source facts and traceability notes.
- [PDF extracts](pdf-extracted-md/): mechanically extracted LSM6DS3TR-C
  datasheet and application-note text used by the coverage guard.
- [Vendor design tips](design_tips/): optional background PDFs. Their generated
  raw text and derived summaries are intentionally not retained.

Source material supports engineering review but does not define the library
API. When it conflicts with a maintained contract, resolve the conflict in the
owning document and record the change in the changelog.
