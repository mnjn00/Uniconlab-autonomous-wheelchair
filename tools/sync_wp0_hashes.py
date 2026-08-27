#!/usr/bin/env python3
"""Refresh the strict README -> A15 -> WP0 manifest SHA-256 chain.

This script changes only existing hash fields. It does not suppress validator
errors or generate evidence; it records the exact current bytes that the WP0
validator already requires.
"""

from __future__ import annotations

import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"
A15 = ROOT / "contracts/wp0/A15-evidence-inventory.yaml"
MANIFEST = ROOT / "contracts/wp0/manifest.yaml"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_indented_hash(text: str, heading: str, new_hash: str) -> str:
    lines = text.splitlines(keepends=True)
    heading_line = heading + ":"
    inside = False
    replaced = False
    for index, line in enumerate(lines):
        stripped = line.rstrip("\r\n")
        if stripped == heading_line:
            inside = True
            continue
        if inside and stripped and not line.startswith("    "):
            break
        if inside and line.startswith("    sha256:"):
            ending = "\r\n" if line.endswith("\r\n") else "\n"
            lines[index] = "    sha256: %s%s" % (new_hash, ending)
            replaced = True
            break
    if not replaced:
        raise RuntimeError("hash field under %s was not found" % heading)
    return "".join(lines)


def replace_manifest_hash(text: str, path_name: str, new_hash: str) -> str:
    lines = text.splitlines(keepends=True)
    target = "  - path: %s" % path_name
    for index, line in enumerate(lines):
        if line.rstrip("\r\n") != target:
            continue
        if index + 1 >= len(lines) or not lines[index + 1].startswith("    sha256:"):
            raise RuntimeError("manifest hash field missing after %s" % path_name)
        ending = "\r\n" if lines[index + 1].endswith("\r\n") else "\n"
        lines[index + 1] = "    sha256: %s%s" % (new_hash, ending)
        return "".join(lines)
    raise RuntimeError("manifest entry not found: %s" % path_name)


def main() -> None:
    readme_hash = digest(README)
    original_a15 = A15.read_text(encoding="utf-8")
    updated_a15 = replace_indented_hash(
        original_a15, "  repository_readme", readme_hash)
    if updated_a15 != original_a15:
        A15.write_text(updated_a15, encoding="utf-8", newline="")

    a15_hash = digest(A15)
    original_manifest = MANIFEST.read_text(encoding="utf-8")
    updated_manifest = replace_manifest_hash(
        original_manifest, A15.name, a15_hash)
    if updated_manifest != original_manifest:
        MANIFEST.write_text(updated_manifest, encoding="utf-8", newline="")

    print("README.md", readme_hash)
    print(str(A15.relative_to(ROOT)), a15_hash)
    print(str(MANIFEST.relative_to(ROOT)), digest(MANIFEST))


if __name__ == "__main__":
    main()
