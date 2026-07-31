#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "numpy==2.4.6",
#   "pillow==12.3.0",
#   "pydantic==2.13.4",
# ]
# ///
"""Generate the 0727 route-band evidence figures.

Run from the repository root:
    uv run decks/unicon-commercial-readiness/visual-report/generate_route_visuals.py
"""

import json
from dataclasses import asdict, dataclass
from hashlib import file_digest
from pathlib import Path

from route_audit import REPO_ROOT, RouteAudit, audit_route_bundle, load_documents
from route_visual_renderer import PlotData, load_xyzi, render_assets


@dataclass(frozen=True, slots=True)
class FileDigest:
    path: str
    bytes: int
    sha256: str


@dataclass(frozen=True, slots=True)
class Provenance:
    generator_source: FileDigest
    band_json: FileDigest
    route_json: FileDigest
    safety_band_source: FileDigest
    canonical_ply: FileDigest
    canonical_ply_points: int
    runtime_pcd: FileDigest
    runtime_pcd_points: int
    overview_png: FileDigest
    hotspot_sheet_png: FileDigest
    hotspot_tiles: tuple[FileDigest, ...]
    web_assets: tuple[FileDigest, ...]


@dataclass(frozen=True, slots=True)
class AuditReceipt:
    audit: RouteAudit
    provenance: Provenance


class PlyFormatError(ValueError):
    pass


def digest_file(path: Path) -> FileDigest:
    try:
        display_path = str(path.resolve().relative_to(REPO_ROOT.resolve()))
    except ValueError:
        display_path = str(path)
    with path.open("rb") as stream:
        sha256 = file_digest(stream, "sha256").hexdigest()
    return FileDigest(display_path, path.stat().st_size, sha256)


def read_ply_vertex_count(path: Path) -> int:
    vertex_count: int | None = None
    header_bytes = 0
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise PlyFormatError("PLY end_header not found")
            header_bytes += len(line)
            if header_bytes > 65_536:
                raise PlyFormatError("PLY header exceeds 64 KiB")
            if line.startswith(b"element vertex "):
                vertex_count = int(line.split(maxsplit=2)[2])
            if line.strip() == b"end_header":
                break
    if vertex_count is None:
        raise PlyFormatError("PLY vertex count not found")
    return vertex_count


def main() -> None:
    report_dir = Path(__file__).resolve().parent
    band_path = REPO_ROOT / "routes/20260727_new_route_safety_band.json"
    route_path = REPO_ROOT / "routes/20260727_new_route_waypoints.json"
    safety_path = REPO_ROOT / "src/static_livox_localization/scripts/safety_band.py"
    ply_path = Path("/Volumes/무제/merged_0707_0725_v1/mergedmap.ply")
    pcd_path = Path("/Volumes/무제/merged_0707_0725_v1/merged_0707_0725_0p20m_xyzi.pcd")
    band_doc, route_doc = load_documents(band_path, route_path)
    audit = audit_route_bundle(band_path, route_path, safety_path)
    data = PlotData(load_xyzi(pcd_path), band_doc.stations, route_doc.waypoints, audit)
    assets = report_dir / "assets"
    render_assets(data, assets)
    hotspot_tiles = tuple(
        digest_file(assets / f"route-band-hotspot-{label}.png")
        for label in ("a", "b", "c", "d", "e")
    )
    web_assets = tuple(
        digest_file(assets / name)
        for name in (
            "route-band-overview.webp",
            "route-band-overview-900.webp",
            "route-band-hotspots.webp",
            *(
                f"route-band-hotspot-{label}-400.webp"
                for label in ("a", "b", "c", "d", "e")
            ),
        )
    )
    provenance = Provenance(
        digest_file(Path(__file__)),
        digest_file(band_path),
        digest_file(route_path),
        digest_file(safety_path),
        digest_file(ply_path),
        read_ply_vertex_count(ply_path),
        digest_file(pcd_path),
        data.points.shape[0],
        digest_file(assets / "route-band-overview.png"),
        digest_file(assets / "route-band-hotspots.png"),
        hotspot_tiles,
        web_assets,
    )
    (assets / "route-band-audit.json").write_text(
        json.dumps(
            asdict(AuditReceipt(audit, provenance)), indent=2, ensure_ascii=False
        )
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
