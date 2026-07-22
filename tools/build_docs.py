#!/usr/bin/env python3
"""Build a clean, warning-free Doxygen reference tree."""

from __future__ import annotations

import pathlib
import shutil
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[1]
DOXYFILE = ROOT / "Doxyfile"
DOCS_ROOT = (ROOT / "docs").resolve()
OUTPUT = ROOT / "docs" / "doxygen"


def remove_previous_output() -> None:
    resolved = OUTPUT.resolve()
    if resolved.parent != DOCS_ROOT or resolved.name != "doxygen":
        raise RuntimeError(f"refusing to remove unexpected path: {resolved}")
    if OUTPUT.is_symlink():
        OUTPUT.unlink()
    elif OUTPUT.exists():
        shutil.rmtree(OUTPUT)


def main() -> int:
    remove_previous_output()
    try:
        result = subprocess.run(
            ["doxygen", str(DOXYFILE)], cwd=ROOT, check=False
        )
    except FileNotFoundError:
        print("Doxygen executable was not found")
        return 1
    if result.returncode != 0:
        return result.returncode
    print(f"Doxygen documentation built: {OUTPUT / 'html' / 'index.html'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
