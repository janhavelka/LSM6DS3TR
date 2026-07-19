#!/usr/bin/env python3
from __future__ import annotations

import json
import pathlib
import sys
import tarfile


ROOT = pathlib.Path(__file__).resolve().parents[1]

ROOT_FILES = (
    "library.json",
    "README.md",
    "CHANGELOG.md",
    "LICENSE",
    "SECURITY.md",
    "CMakeLists.txt",
    "idf_component.yml",
)

PACKAGE_DIRS = (
    "include",
    "src",
    "examples/01_basic_bringup_cli",
    "examples/common",
    "examples/idf/basic",
)


def fail(message: str) -> int:
    print(f"Package contract FAILED: {message}")
    return 1


def expected_files() -> set[str]:
    expected = set(ROOT_FILES)
    for relative_dir in PACKAGE_DIRS:
        directory = ROOT / relative_dir
        if not directory.is_dir():
            raise FileNotFoundError(f"missing package directory: {relative_dir}")
        expected.update(
            path.relative_to(ROOT).as_posix()
            for path in directory.rglob("*")
            if path.is_file()
        )
    return expected


def default_archive() -> pathlib.Path:
    manifest = json.loads((ROOT / "library.json").read_text(encoding="utf-8"))
    return ROOT / f"{manifest['name']}-{manifest['version']}.tar.gz"


def main() -> int:
    archive = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) == 2 else default_archive()
    if len(sys.argv) > 2:
        return fail("usage: check_package_contract.py [archive.tar.gz]")
    if not archive.is_file():
        return fail(f"archive does not exist: {archive}")

    expected = expected_files()
    observed: set[str] = set()
    with tarfile.open(archive, mode="r:gz") as package:
        for member in package.getmembers():
            name = member.name.removeprefix("./")
            path = pathlib.PurePosixPath(name)
            if path.is_absolute() or ".." in path.parts:
                return fail(f"unsafe archive path: {member.name}")
            if member.isdir():
                continue
            if not member.isfile():
                return fail(f"non-regular archive member: {member.name}")
            if name in observed:
                return fail(f"duplicate archive member: {name}")
            observed.add(name)

    missing = sorted(expected - observed)
    unexpected = sorted(observed - expected)
    if missing or unexpected:
        print("Package contract FAILED: archive content differs from export contract")
        for name in missing:
            print(f"  missing: {name}")
        for name in unexpected:
            print(f"  unexpected: {name}")
        return 1

    print(f"Package contract PASSED ({len(observed)} files)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
