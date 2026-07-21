# Documentation Map

This directory contains maintained guidance, audit evidence, source extracts,
and locally generated API pages. They do not all have the same authority.

## Maintained Contracts And Guidance

- [Project README](../README.md): supported behavior, ownership contract,
  installation, examples, and validation.
- [Public API headers](../include/LSM6DS3TR): Doxygen contracts for supported
  types and operations.
- [ESP-IDF port guide](IDF_PORT.md): native component and application-owned
  transport boundary.
- [Implementation manual](../LSM6DS3TR_imu_implementation_manual.md): chip and
  integration guidance used by this repository.
- <a href="TUNNELMONITOR_NODE_SUITABILITY_AUDIT.md">TunnelMonitor suitability
  audit</a> and <a href="tunnelmonitor_fit_report.md">fit report</a>:
  product-specific review of the version 2 contract.

Changes to supported behavior must update the applicable public Doxygen,
README, guide, and [changelog](../CHANGELOG.md) in the same change.

## Validation And Evidence

- <a href="reports/hil-evidence-summary.md">HIL evidence summary</a>: retained
  version 1.x fixture evidence. It is historical and does not validate the
  version 2 operation model.
- `docs/doxygen/`: ignored local output produced by
  `python tools/build_docs.py`. Do not edit or commit it.

## Source Reference Material

- <a href="extracted-md/00_document_inventory.md">Extracted document
  inventory</a>: compact source facts and traceability notes.
- [PDF extracts](pdf-extracted-md/): verbatim or mechanically extracted source
  material.
- [Design tips](design_tips/): derived engineering notes.

Source extracts and derived notes support engineering review but do not define
the library API. When they conflict with a maintained contract, resolve the
conflict in the owning document and record the change in the changelog.
