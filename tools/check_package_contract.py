#!/usr/bin/env python3
from __future__ import annotations

import json
import pathlib
import re
import sys
import tarfile
import urllib.parse


ROOT = pathlib.Path(__file__).resolve().parents[1]

ROOT_FILES = (
    "library.json",
    "README.md",
    "CHANGELOG.md",
    "LICENSE",
    "SECURITY.md",
    "docs/DOCUMENTATION.md",
    "docs/IDF_PORT.md",
    "CMakeLists.txt",
    "idf_component.yml",
)

PACKAGE_DIRS = (
    "include",
    "src",
    "examples/01_basic_bringup_cli",
    "examples/common",
    "examples/idf/basic",
    "docs/chip-reference",
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


def local_link_target(document: str, raw_target: str) -> str | None:
    target = raw_target.strip().strip("<>").split("#", 1)[0]
    if not target or urllib.parse.urlparse(target).scheme or target.startswith("//"):
        return None
    candidate = pathlib.PurePosixPath(document).parent / pathlib.PurePosixPath(
        urllib.parse.unquote(target)
    )
    parts: list[str] = []
    for part in candidate.parts:
        if part == "..":
            if parts:
                parts.pop()
        elif part != ".":
            parts.append(part)
    return "/".join(parts)


def main() -> int:
    archive = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) == 2 else default_archive()
    if len(sys.argv) > 2:
        return fail("usage: check_package_contract.py [archive.tar.gz]")
    if not archive.is_file():
        return fail(f"archive does not exist: {archive}")

    expected = expected_files()
    observed: set[str] = set()
    markdown: dict[str, str] = {}
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
            if name.endswith(".md"):
                source = package.extractfile(member)
                if source is None:
                    return fail(f"cannot read Markdown member: {name}")
                markdown[name] = source.read().decode("utf-8", errors="replace")

    missing = sorted(expected - observed)
    unexpected = sorted(observed - expected)
    if missing or unexpected:
        print("Package contract FAILED: archive content differs from export contract")
        for name in missing:
            print(f"  missing: {name}")
        for name in unexpected:
            print(f"  unexpected: {name}")
        return 1

    patterns = (
        re.compile(r"(?<!!)\[[^\]]+\]\(([^)]+)\)"),
        re.compile(r'<a\s+href="([^"]+)"', re.IGNORECASE),
    )
    broken_links: list[str] = []
    for document, contents in markdown.items():
        for pattern in patterns:
            for raw_target in pattern.findall(contents):
                target = local_link_target(document, raw_target)
                if target is None:
                    continue
                if target not in observed and not any(
                    member.startswith(target.rstrip("/") + "/")
                    for member in observed
                ):
                    broken_links.append(f"{document}: {raw_target} -> {target}")
    if broken_links:
        print("Package contract FAILED: packaged Markdown has broken local links")
        for link in broken_links:
            print(f"  - {link}")
        return 1

    print(
        f"Package contract PASSED ({len(observed)} files, "
        f"{len(markdown)} linked Markdown documents)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
