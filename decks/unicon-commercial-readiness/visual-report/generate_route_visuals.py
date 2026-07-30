#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "numpy>=2.0",
#   "pillow>=11.0",
#   "pydantic>=2.0",
# ]
# ///
"""Generate the 0727 route-band evidence figures.

Run from the repository root:
    uv run decks/unicon-commercial-readiness/visual-report/generate_route_visuals.py
"""

import json
from dataclasses import asdict
from pathlib import Path

from route_audit import REPO_ROOT, audit_route_bundle, load_documents
from route_visual_renderer import PlotData, load_xyzi, render_assets


def main() -> None:
    report_dir = Path(__file__).resolve().parent
    band_path = REPO_ROOT / "routes/20260727_new_route_safety_band.json"
    route_path = REPO_ROOT / "routes/20260727_new_route_waypoints.json"
    safety_path = REPO_ROOT / "src/static_livox_localization/scripts/safety_band.py"
    pcd_path = Path(
        "/Volumes/무제/merged_0707_0725_v1/merged_0707_0725_0p20m_xyzi.pcd"
    )
    band_doc, route_doc = load_documents(band_path, route_path)
    audit = audit_route_bundle(band_path, route_path, safety_path)
    data = PlotData(load_xyzi(pcd_path), band_doc.stations, route_doc.waypoints, audit)
    assets = report_dir / "assets"
    render_assets(data, assets)
    (assets / "route-band-audit.json").write_text(
        json.dumps(asdict(audit), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
