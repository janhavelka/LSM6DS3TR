#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import pathlib
import re
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
REFERENCE_DIR = ROOT / "docs" / "chip-reference"
RAW_DIR = ROOT / "docs" / "pdf-extracted-md"

REFERENCE_DOCS = (
    "00_reference_index.md",
    "01_chip_overview.md",
    "02_pinout_and_signals.md",
    "03_electrical_and_timing.md",
    "04_protocol_commands_and_transactions.md",
    "05_register_map.md",
    "06_modes_interrupts_status_and_faults.md",
    "07_initialization_reset_and_operational_notes.md",
    "08_variant_and_source_notes.md",
    "09_filters_and_settling.md",
    "10_fifo_and_embedded_functions.md",
    "11_self_test_and_calibration.md",
    "12_source_ambiguities.md",
    "13_library_support_matrix.md",
)

SOURCE_ARTIFACTS = (
    (
        ROOT / "docs" / "LSM6DS3TR-C_datasheet.pdf",
        RAW_DIR / "LSM6DS3TR-C_datasheet.md",
        "352f35643f90772781540cf625ee574eb41d8249707440ac200632294b3b773c",
        "- Page count: 114",
    ),
    (
        ROOT
        / "docs"
        / "LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.pdf",
        RAW_DIR
        / "LSM6DS3TR-C_Always-On_3D_Accelerometer_3D_Gyroscope_Application_Note_AN5130.md",
        "15b90b0ccca09f1cd6f561ecc67c7620714f452ad3f2c07e0f1cbce7e1ebb1c6",
        "- Page count: 109",
    ),
)

CRITICAL_REFERENCE_ANCHORS = (
    "WHO_AM_I",
    "FUNC_CFG_ACCESS",
    "CTRL1_XL",
    "CTRL2_G",
    "CTRL3_C",
    "CTRL6_C[4]",
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
    "MASTER_CONFIG",
    "WAKE_UP_SRC",
    "TAP_SRC",
    "D6D_SRC",
    "TIMESTAMP0_REG",
    "STEP_COUNTER_L",
    "STEP_COUNTER_H",
    "SENSORHUB18_REG",
    "FUNC_SRC1",
    "FUNC_SRC2",
    "WRIST_TILT_IA",
    "X_OFS_USR",
    "Reserved bits",
    "vendor fact",
    "vendor ambiguity",
    "engineering inference",
    "diagnostic-only",
    "unsupported",
)

# These are independent datasheet-derived expectations for the highest-risk
# constants. The checker compares them with CommandTable.h rather than merely
# checking that a name exists in both documentation and code.
HEADER_FACTS = {
    "REG_FUNC_CFG_ACCESS": 0x01,
    "REG_FIFO_CTRL5": 0x0A,
    "REG_WHO_AM_I": 0x0F,
    "WHO_AM_I_VALUE": 0x6A,
    "REG_CTRL1_XL": 0x10,
    "REG_CTRL2_G": 0x11,
    "REG_CTRL3_C": 0x12,
    "REG_CTRL6_C": 0x15,
    "REG_CTRL9_XL": 0x18,
    "REG_STATUS_REG": 0x1E,
    "REG_OUT_TEMP_L": 0x20,
    "REG_OUTX_L_G": 0x22,
    "REG_OUTX_L_XL": 0x28,
    "REG_FIFO_STATUS1": 0x3A,
    "REG_FIFO_DATA_OUT_L": 0x3E,
    "REG_TIMESTAMP2": 0x42,
    "REG_STEP_COUNTER_L": 0x4B,
    "REG_STEP_COUNTER_H": 0x4C,
    "REG_X_OFS_USR": 0x73,
    "REG_Y_OFS_USR": 0x74,
    "REG_Z_OFS_USR": 0x75,
    "BIT_FUNC_CFG_EN": 7,
    "MASK_FUNC_CFG_EN": 0x80,
    "BIT_FUNC_CFG_EN_B": 5,
    "MASK_FUNC_CFG_EN_B": 0x20,
    "BIT_XL_HM_MODE": 4,
    "MASK_XL_HM_MODE": 0x10,
    "BIT_G_HM_MODE": 7,
    "MASK_G_HM_MODE": 0x80,
    "BIT_BDU": 6,
    "MASK_BDU": 0x40,
    "BIT_IF_INC": 2,
    "MASK_IF_INC": 0x04,
    "BIT_SW_RESET": 0,
    "MASK_SW_RESET": 0x01,
    "BIT_ST_XL": 0,
    "MASK_ST_XL": 0x03,
    "BIT_ST_G": 2,
    "MASK_ST_G": 0x0C,
    "BIT_ODR_FIFO": 3,
    "MASK_ODR_FIFO": 0x78,
    "BIT_FIFO_MODE": 0,
    "MASK_FIFO_MODE": 0x07,
    "BIT_TDA": 2,
    "MASK_TDA": 0x04,
    "BIT_GDA": 1,
    "MASK_GDA": 0x02,
    "BIT_XLDA": 0,
    "MASK_XLDA": 0x01,
    "TIMESTAMP_RESET_VALUE": 0xAA,
}

EXACT_DOCUMENT_FACTS = {
    "00_reference_index.md": (
        "DocID030071 Rev 3, May 2017",
        "AN5130 Rev 1, March 2018",
        "Official pages | Repository evidence",
        "Verified against ST's live product documentation on **2026-07-31**",
    ),
    "04_protocol_commands_and_transactions.md": (
        "IF_INC` (`0x12[2]`)",
        "`0x80 | addr` for reads",
    ),
    "05_register_map.md": (
        "`1011` | 1.6 Hz low-power-only | 12.5 Hz high-performance",
        "`0110` | 416 Hz high-performance | 416 Hz high-performance",
        "silicon cannot honor",
        "Table 19 says `0x00`, while Table 75 field defaults imply `0xE0`",
    ),
    "09_filters_and_settling.md": (
        "138, 131, 121, and 138 Hz",
        "135 / 135 / 135 / 135",
        "BDU=1` protects each individual 16-bit LSB/MSB output pair",
    ),
    "10_fifo_and_embedded_functions.md": (
        "`min(max(ODR_XL, ODR_G), ODR_FIFO)`",
        "require decoded `DIFF_FIFO=0` exactly when `FIFO_EMPTY=1`",
        "continue filling if space remains, then stop when full",
        "`010`/`011`/`100`/`101`/`110`/`111`",
        "`0x3C` for slave-0 write",
    ),
    "11_self_test_and_calibration.md": (
        "`CTRL1_XL=0x38`",
        "`CTRL2_G=0x5C`",
        "Wait 150 ms",
        "Wait 50 ms",
        "90..1700 mg inclusive",
        "150..700 dps inclusive",
        "20 ms at the 52 Hz accelerometer",
        "`CTRL2_G=0x00` to power down the gyroscope",
        "`16 * (samples + 1) + 87`",
    ),
    "12_source_ambiguities.md": (
        "`FUNC_CFG_ACCESS[4]=0`",
        "`A_WRIST_TILT_MASK=0xC0`",
        "requires `STATUS_REG.TDA`",
        "transmit one 8-bit register sub-address",
        "`D4D_EN=1` enables 4D",
    ),
    "13_library_support_matrix.md": (
        "FIFO configuration, acquisition, pattern decoding | unsupported",
        "Raw main-bank register/block reads | diagnostic-only",
        "Built-in accelerometer and gyro self-test | supported",
    ),
}

FORBIDDEN_DOCUMENT_FACTS = {
    "04_protocol_commands_and_transactions.md": ("Sub-address MSB",),
    "11_self_test_and_calibration.md": (
        "`CTRL1_XL=0x60`",
        "`CTRL2_G=0x60`",
        "Wait 800 ms",
        "Wait 60 ms",
    ),
}


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    errors: list[str] = []
    texts: dict[str, str] = {}

    for name in REFERENCE_DOCS:
        path = REFERENCE_DIR / name
        if not path.is_file():
            errors.append(f"missing maintained reference: {path.relative_to(ROOT)}")
            continue
        texts[name] = path.read_text(encoding="utf-8", errors="replace")

    for pdf_path, raw_path, expected_hash, page_anchor in SOURCE_ARTIFACTS:
        if not pdf_path.is_file():
            errors.append(f"missing vendor PDF: {pdf_path.relative_to(ROOT)}")
            continue
        observed_hash = sha256(pdf_path)
        if observed_hash != expected_hash:
            errors.append(
                f"vendor PDF hash changed: {pdf_path.relative_to(ROOT)} "
                f"expected {expected_hash}, observed {observed_hash}"
            )
        if not raw_path.is_file():
            errors.append(f"missing raw source extract: {raw_path.relative_to(ROOT)}")
            continue
        raw_text = raw_path.read_text(encoding="utf-8", errors="replace")
        if f"- SHA256: `{expected_hash}`" not in raw_text:
            errors.append(f"raw extract hash header drifted: {raw_path.relative_to(ROOT)}")
        if page_anchor not in raw_text:
            errors.append(f"raw extract page header drifted: {raw_path.relative_to(ROOT)}")

    combined = "\n".join(texts.values())
    for anchor in CRITICAL_REFERENCE_ANCHORS:
        if anchor not in combined:
            errors.append(f"maintained reference lost critical anchor: {anchor}")

    for name, facts in EXACT_DOCUMENT_FACTS.items():
        text = texts.get(name, "")
        for fact in facts:
            if fact not in text:
                errors.append(f"{name} lost exact audited fact: {fact}")

    for name, facts in FORBIDDEN_DOCUMENT_FACTS.items():
        text = texts.get(name, "")
        for fact in facts:
            if fact in text:
                errors.append(f"{name} regained rejected stale fact: {fact}")

    header_path = ROOT / "include" / "LSM6DS3TR" / "CommandTable.h"
    header = header_path.read_text(encoding="utf-8", errors="replace")
    constants = {
        match.group(1): int(match.group(2), 0)
        for match in re.finditer(
            r"static constexpr uint(?:8|32)_t\s+(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)",
            header,
        )
    }
    for name, expected in HEADER_FACTS.items():
        observed = constants.get(name)
        if observed != expected:
            errors.append(
                f"CommandTable fact mismatch: {name} expected 0x{expected:X}, "
                f"observed {observed!r}"
            )

    if errors:
        print("Chip documentation coverage FAILED")
        for error in errors:
            print(f"  - {error}")
        return 1

    print(
        "Chip documentation coverage PASSED "
        f"({len(REFERENCE_DOCS)} maintained topics, "
        f"{len(HEADER_FACTS)} exact register facts)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
