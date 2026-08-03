"""The field recordings are what they claim to be.

These bags are the only copy of two runs that cannot be repeated - the
weather, the parked cars, the people on the path and the state of the code
were all what they were on 2026-07-31. Everything downstream that gets
argued from them (the localization envelope, the cluster-guard defect) is
only as good as the claim that the file on this disk is the file the vehicle
wrote, and a 42 MB binary is exactly the kind of thing that silently
truncates in a transfer and still opens.

So the manifest carries a sha256 per bag and this checks it. It also checks
that the manifest and the folder agree in both directions: a bag with no
entry is a recording nobody can identify later, and an entry with no bag is
a citation to something that is gone.

Skips rather than fails when the bags are unfetched LFS pointers, because a
clone that has not run `git lfs pull` is a normal state and not a corrupted
archive.
"""

import hashlib
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
BLACKBOX = ROOT / "blackbox"
MANIFEST = BLACKBOX / "manifest.json"
LFS_POINTER_MAGIC = b"version https://git-lfs"


def manifests():
    """Every manifest in the folder, not just the first one written.

    Sessions arrived with their own file - manifest.json for 2026-07-31,
    manifest_20260802.json for the next one - and reading only the original
    made three archived bags look unrecorded. Which file a session lands in
    is a filing decision; that every bag is accounted for somewhere is the
    property, so this collects them all.
    """
    return sorted(BLACKBOX.glob("manifest*.json"))


def is_lfs_pointer(path):
    with path.open("rb") as handle:
        return handle.read(len(LFS_POINTER_MAGIC)) == LFS_POINTER_MAGIC


def bag_entries():
    entries = []
    for path in manifests():
        for entry in json.loads(path.read_text(encoding="utf-8"))["bags"]:
            entries.append(dict(entry, _manifest=path.name))
    return entries


def test_at_least_one_manifest_exists_and_parses():
    assert manifests(), "blackbox/ has no manifest"
    assert bag_entries(), "no manifest lists any bag"


@pytest.mark.parametrize("entry", bag_entries(),
                         ids=[e["file"] for e in bag_entries()])
def test_each_bag_matches_its_recorded_checksum(entry):
    path = BLACKBOX / entry["file"]
    assert path.exists(), "%s is in %s but not in the folder" % (
        entry["file"], entry["_manifest"])
    if is_lfs_pointer(path):
        pytest.skip("LFS pointer - run `git lfs pull` to verify the content")
    assert path.stat().st_size == entry["bytes"], (
        "%s is %d bytes, the manifest says %d - a truncated transfer opens "
        "fine and plays short" % (entry["file"], path.stat().st_size,
                                  entry["bytes"]))
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    assert digest == entry["sha256"], "%s does not hash to its manifest entry"


def test_no_bag_sits_in_the_folder_unrecorded():
    """An unlisted recording is one nobody can say anything about later."""
    listed = {e["file"] for e in bag_entries()}
    found = {p.name for p in BLACKBOX.glob("*.bag")}
    assert found <= listed, "unlisted bags in blackbox/: %s" % sorted(found - listed)


def test_no_bag_is_claimed_by_two_manifests():
    """Two entries for one file is two sets of provenance for it, and
    nothing says which one an argument was made from."""
    seen = {}
    for entry in bag_entries():
        seen.setdefault(entry["file"], []).append(entry["_manifest"])
    duplicated = {f: m for f, m in seen.items() if len(m) > 1}
    assert not duplicated, "claimed twice: %s" % duplicated


def test_the_bags_are_stored_through_lfs():
    """Ninety megabytes a session, straight into a history every NUC
    deployment pulls, is how a 68 MB repo becomes a gigabyte one by
    September. LFS keeps the pointer in git and the bytes out of it."""
    attributes = (ROOT / ".gitattributes").read_text(encoding="utf-8")
    assert "blackbox/*.bag" in attributes
    assert "filter=lfs" in attributes


def test_the_runtime_bag_ignore_still_applies_everywhere_else():
    """The exception is for curated evidence, not a general invitation to
    commit recordings - a 14 GB trial bag from a mapping session would not
    survive contact with LFS quota."""
    ignore = (ROOT / ".gitignore").read_text(encoding="utf-8")
    assert "*.bag" in ignore
    assert "!blackbox/*.bag" in ignore
