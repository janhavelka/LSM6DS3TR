#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
COMPACT_DIR = ROOT / "docs" / "extracted-md"
RAW_DIR = ROOT / "docs" / "pdf-extracted-md"

COMPACT_DOCS = [
    "00_document_inventory.md",
    "01_chip_overview.md",
    "02_pinout_and_signals.md",
    "03_electrical_and_timing.md",
    "04_protocol_commands_and_transactions.md",
    "05_register_map.md",
    "06_modes_interrupts_status_and_faults.md",
    "07_initialization_reset_and_operational_notes.md",
    "08_variant_differences_and_open_questions.md",
]

RAW_SOURCE_EXTRACTS = [
    "LSM6DS3TR-C_datasheet.md",
    "LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.md",
]

CRITICAL_COMPACT_ANCHORS = [
    "WHO_AM_I",
    "FUNC_CFG_ACCESS",
    "CTRL1_XL",
    "CTRL2_G",
    "CTRL3_C",
    "BOOT",
    "SW_RESET",
    "CTRL4_C.DRDY_MASK",
    "CTRL10_C",
    "STATUS_REG",
    "OUT_TEMP_L",
    "OUTX_L_G",
    "OUTX_L_XL",
    "FIFO_CTRL1",
    "FIFO_STATUS1",
    "FIFO_DATA_OUT_L",
    "FIFO_PATTERN",
    "SENSOR_SYNC_TIME_FRAME",
    "SENSOR_SYNC_RES_RATIO",
    "MASTER_CONFIG",
    "WAKE_UP_SRC",
    "TAP_SRC",
    "D6D_SRC",
    "TIMESTAMP0_REG",
    "STEP_COUNTER_L",
    "STEP_COUNTER_H",
    "SENSORHUB13_REG",
    "SENSORHUB18_REG",
    "FUNC_SRC1",
    "FUNC_SRC2",
    "WRIST_TILT_IA",
    "TAP_CFG",
    "WAKE_UP_THS",
    "WAKE_UP_DUR",
    "FREE_FALL",
    "MD1_CFG",
    "MD2_CFG",
    "MASTER_CMD_CODE",
    "SENS_SYNC_SPI_ERROR_CODE",
    "OUT_MAG_RAW_X_L",
    "X_OFS_USR",
    "SLV0_ADD",
    "CONFIG_PEDO_THS_MIN",
    "A_WRIST_TILT_LAT",
    "0x80 | addr",
    "Reserved bits",
]

EXACT_REGISTER_FACTS = [
    ("| `0x04` | `SENSOR_SYNC_TIME_FRAME` |", "REG_SENSOR_SYNC_TIME_FRAME = 0x04"),
    ("| `0x05` | `SENSOR_SYNC_RES_RATIO` |", "REG_SENSOR_SYNC_RES_RATIO = 0x05"),
]


def fail(message: str) -> int:
    print(f"Chip documentation coverage FAILED: {message}")
    return 1


def main() -> int:
    missing_paths = []
    for name in COMPACT_DOCS:
        path = COMPACT_DIR / name
        if not path.is_file():
            missing_paths.append(path.relative_to(ROOT).as_posix())

    for name in RAW_SOURCE_EXTRACTS:
        path = RAW_DIR / name
        if not path.is_file():
            missing_paths.append(path.relative_to(ROOT).as_posix())

    if missing_paths:
        print("Chip documentation coverage FAILED: required source files are missing")
        for path in missing_paths:
            print(f"  - {path}")
        return 1

    compact_text = "\n".join(
        (COMPACT_DIR / name).read_text(encoding="utf-8", errors="replace")
        for name in COMPACT_DOCS
    )
    missing_anchors = [
        anchor for anchor in CRITICAL_COMPACT_ANCHORS if anchor not in compact_text
    ]

    if missing_anchors:
        print("Chip documentation coverage FAILED: compact docs lost critical anchors")
        for anchor in missing_anchors:
            print(f"  - {anchor}")
        return 1

    command_table = (ROOT / "include" / "LSM6DS3TR" / "CommandTable.h").read_text(
        encoding="utf-8", errors="replace"
    )
    missing_facts = []
    for compact_fact, header_fact in EXACT_REGISTER_FACTS:
        if compact_fact not in compact_text:
            missing_facts.append(compact_fact)
        if header_fact not in command_table:
            missing_facts.append(header_fact)
    if missing_facts:
        print("Chip documentation coverage FAILED: exact register facts changed")
        for fact in missing_facts:
            print(f"  - {fact}")
        return 1

    print("Chip documentation coverage PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
