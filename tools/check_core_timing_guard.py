#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCAN_DIRS = ("src", "include")
VALID_SUFFIXES = {".c", ".cc", ".cpp", ".h", ".hpp"}

FORBIDDEN_CALLS = {
    "delay": re.compile(r"\bdelay\s*\("),
    "millis": re.compile(r"\bmillis\s*\("),
    "micros": re.compile(r"\bmicros\s*\("),
    "delayMicroseconds": re.compile(r"\bdelayMicroseconds\s*\("),
    "yield": re.compile(r"\byield\s*\("),
}

FORBIDDEN_TYPE_OR_EXPR = {
    "String": re.compile(r"\bString\b"),
    "std::vector": re.compile(r"\bstd\s*::\s*vector\b"),
    "new": re.compile(r"\bnew\b"),
}

FORBIDDEN_INCLUDE_PATTERNS = {
    "Arduino.h": re.compile(r'^\s*#\s*include\s*[<\"]Arduino\.h[>\"]', re.MULTILINE),
    "Wire.h": re.compile(r'^\s*#\s*include\s*[<\"]Wire\.h[>\"]', re.MULTILINE),
    "ESP-IDF": re.compile(
        r'^\s*#\s*include\s*[<\"](?:driver/i2c|esp_timer|freertos/)',
        re.MULTILINE,
    ),
}
BLOCK_COMMENT_RE = re.compile(r"/\*.*?\*/", re.DOTALL)
LINE_COMMENT_RE = re.compile(r"//[^\n]*")
STRING_RE = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')


def strip_non_code(text: str) -> str:
    text = BLOCK_COMMENT_RE.sub("", text)
    text = LINE_COMMENT_RE.sub("", text)
    return STRING_RE.sub('""', text)


def collect_sources() -> list[pathlib.Path]:
    files: list[pathlib.Path] = []
    for dirname in SCAN_DIRS:
        root = ROOT / dirname
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.suffix.lower() in VALID_SUFFIXES:
                files.append(path)
    return files


def main() -> int:
    errors: list[str] = []

    for path in collect_sources():
        rel = path.relative_to(ROOT).as_posix()
        raw = path.read_text(encoding="utf-8", errors="replace")
        code = strip_non_code(raw)

        for call_name, pattern in FORBIDDEN_CALLS.items():
            count = len(pattern.findall(code))
            if count > 0:
                errors.append(
                    f"forbidden timing call in {rel}: {call_name} count={count}"
                )

        for token_name, pattern in FORBIDDEN_TYPE_OR_EXPR.items():
            count = len(pattern.findall(code))
            if count > 0:
                errors.append(
                    f"forbidden allocation/framework token in {rel}: "
                    f"{token_name} count={count}"
                )

        for include_name, pattern in FORBIDDEN_INCLUDE_PATTERNS.items():
            count = len(pattern.findall(raw))
            if count > 0:
                errors.append(
                    f"forbidden framework include in {rel}: "
                    f"{include_name} count={count}"
                )

    if errors:
        print("Core timing guard FAILED:")
        for err in errors:
            print(f"- {err}")
        return 1

    print("Core timing guard PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
