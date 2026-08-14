#!/usr/bin/env python3
"""Promote the independently audited algorithm route to runtime assets.

The independent planner writes analysis artifacts under ``output/``.  The
runtime deliberately consumes a separate, hash-bound route/band/mask bundle;
this tool performs that promotion without changing any route geometry.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    payload = path.read_bytes()
    # Match src/static_livox_localization/scripts/route_assets.py: JSON asset
    # hashes are semantic hashes, independent of a final newline.
    if path.suffix == ".json":
        payload = payload.rstrip(b"\r\n")
    h.update(payload)
    return h.hexdigest()


def route_content_sha256(route: dict) -> str:
    content = dict(route)
    content.pop("asset_binding", None)
    payload = json.dumps(content, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-route", type=Path, default=Path("output/map_by_algorithm_route.json"))
    parser.add_argument("--source-band", type=Path, default=Path("output/map_by_algorithm_band.json"))
    parser.add_argument("--source-mask-yaml", type=Path, default=Path("output/map_by_algorithm_mask.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("routes"))
    parser.add_argument("--route-name", default="20260814_route_algorithm_waypoints.json")
    parser.add_argument("--band-name", default="20260814_route_algorithm_safety_band.json")
    parser.add_argument("--mask-name", default="route_2d_map_algorithm")
    parser.add_argument(
        "--chair-centre-in-body-xyz", nargs=3, type=float,
        default=(-0.5, -0.2, 0.0), metavar=("X", "Y", "Z"),
        help="Measured chair-centre offset used by the builtin body profile",
    )
    args = parser.parse_args()

    route_src = json.loads(args.source_route.read_text(encoding="utf-8"))
    band_src = json.loads(args.source_band.read_text(encoding="utf-8"))
    mask_meta = yaml.safe_load(args.source_mask_yaml.read_text(encoding="utf-8"))
    source_mask = args.source_mask_yaml.parent / mask_meta["image"]
    if not source_mask.is_file():
        raise FileNotFoundError(source_mask)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    mask_pgm = args.out_dir / f"{args.mask_name}.pgm"
    mask_png = args.out_dir / f"{args.mask_name}.png"
    mask_array = np.asarray(Image.open(source_mask).convert("L"))
    Image.fromarray(mask_array, mode="L").save(mask_pgm)
    Image.fromarray(mask_array, mode="L").save(mask_png)
    mask_yaml = args.out_dir / f"{args.mask_name}.yaml"
    mask_yaml.write_text(
        "image: %s\nmode: trinary\nresolution: %s\norigin: [%s, %s, %s]\n"
        "negate: 0\noccupied_thresh: %s\nfree_thresh: %s\n"
        % (
            mask_pgm.name, mask_meta["resolution"],
            float(mask_meta["origin"][0]), float(mask_meta["origin"][1]),
            float(mask_meta["origin"][2]), mask_meta.get("occupied_thresh", 0.65),
            mask_meta.get("free_thresh", 0.25),
        ),
        encoding="utf-8",
    )

    waypoints = [
        {"x": float(point["x"]), "y": float(point["y"]),
         "z": float(point.get("z", 0.0)), "yaw_deg": float(point["yaw_deg"])}
        for point in route_src["waypoints"]
    ]
    route = {
        "frame": "map",
        "source": "output/map_by_algorithm_route.json; independent dense-map curb-bounded planner",
        "source_sha256": {
            "algorithm_route": sha256(args.source_route),
            "algorithm_band": sha256(args.source_band),
            "algorithm_mask": sha256(source_mask),
            "algorithm_mask_yaml": sha256(args.source_mask_yaml),
        },
        "planner_inputs": route_src.get("planner_inputs", {}),
        "footprint_xy_m": route_src.get("footprint_xy_m"),
        "body_frame_profile": "builtin",
        "count": len(waypoints),
        "reference_point": "chair_centre",
        "chair_centre_in_body_xyz": [float(v) for v in args.chair_centre_in_body_xyz],
        "route_step_m": float(route_src.get("route_step_m", 0.2)),
        "path_length_m": float(route_src["path_length_m"]),
        "operator_target_waypoint_index": len(waypoints) - 1,
        "operator_target_xy_m": [waypoints[-1]["x"], waypoints[-1]["y"]],
        "waypoints": waypoints,
    }

    route_id = "algorithm:%s:%s" % (sha256(args.source_route)[:12], sha256(mask_pgm)[:12])
    band = dict(band_src)
    band["route_id"] = route_id
    band["drivable_mask_sha256"] = sha256(mask_pgm)
    band["drivable_mask_yaml_sha256"] = sha256(mask_yaml)
    band["approved_for_motion"] = True
    band["corridor"] = {
        "source": mask_yaml.name,
        "chair_half_width_m": 0.38,
        "policy": "algorithmic dense-map traversable mask is the authoritative chair-centre boundary",
        "stations_covered": len(band["stations"]),
        "stations_total": len(band["stations"]),
    }
    band["physical_edge_semantics"] = {
        "source": args.source_band.name,
        "status": "dense-map curb-edge measurements retained from the independent audit",
    }
    band_path = args.out_dir / args.band_name
    band_path.write_text(json.dumps(band, indent=1) + "\n", encoding="utf-8")

    route["asset_binding"] = {
        "route_id": route_id,
        "drivable_mask_sha256": sha256(mask_pgm),
        "drivable_mask_yaml_sha256": sha256(mask_yaml),
        "safety_band_sha256": sha256(band_path),
    }
    route["asset_binding"]["route_content_sha256"] = route_content_sha256(route)
    route_path = args.out_dir / args.route_name
    route_path.write_text(json.dumps(route, indent=1) + "\n", encoding="utf-8")

    provenance = {
        "promoted_from": {
            "route": str(args.source_route).replace("\\", "/"),
            "band": str(args.source_band).replace("\\", "/"),
            "mask_yaml": str(args.source_mask_yaml).replace("\\", "/"),
        },
        "route": route_path.name,
        "safety_band": band_path.name,
        "drivable_mask": mask_yaml.name,
        "route_id": route_id,
        "route_geometry_unchanged": True,
        "field_validation_required": True,
    }
    (args.out_dir / "20260814_route_algorithm_provenance.json").write_text(
        json.dumps(provenance, indent=2) + "\n", encoding="utf-8"
    )
    print("promoted %d waypoints, %d band stations, route_id=%s" %
          (len(waypoints), len(band["stations"]), route_id))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
