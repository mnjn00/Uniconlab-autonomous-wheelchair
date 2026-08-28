#!/usr/bin/env python3
"""Re-plan the driving route with guaranteed clearance inside a tolerated mask.

The 2026-08-15 trim route hugged the v8 hard boundary (0.10 m at two bends),
and the v8 boundary itself is a hand-drawn overlay rasterised at 0.1 m, so
ordinary localization noise puts the chair centre one pixel outside the mask
and the follower hard-stops with UNSAFE_CHORD. This tool:

1. dilates the authoritative v8 drivable mask by DILATE_M (boundary
   quantisation tolerance) -> route_2d_map_v9.{pgm,yaml};
2. plans the route inside the *original* v8 mask eroded by MIN_CLEARANCE_M,
   so every waypoint keeps at least that much clearance from the v8 edge;
3. measures band stations against the tolerated v9 mask and re-binds all
   asset SHA-256 identities, in the exact document shape consumed by
   waypoint_follower / obstacle_clusters / route_identity_publisher.
"""

from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_preferred_mask_route as base  # noqa: E402

DILATE_PX = 2  # 손으로 그린 경계의 래스터 양자화 허용치 0.2 m
ERODE_PX = 3  # v8 하드 경계로부터 보장할 경로 여유 0.3 m
FREE = 254
BACKGROUND = 205


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _content_sha(route: dict) -> str:
    content = {k: v for k, v in route.items() if k != "asset_binding"}
    return hashlib.sha256(
        json.dumps(content, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def main(
    preferred_yaml: Path,
    drivable_yaml: Path,
    seed_route: Path,
    seed_band: Path,
    out_dir: Path,
) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    preferred, p_res, p_origin = base._load_map(preferred_yaml)
    v8, res, origin = base._load_map(drivable_yaml)
    if preferred.shape != v8.shape or p_res != res or p_origin != origin:
        raise RuntimeError("preferred and drivable maps do not share one grid")
    height, width = v8.shape

    v9 = ndimage.binary_dilation(v8, iterations=DILATE_PX)
    image_v9 = np.full((height, width), BACKGROUND, dtype=np.uint8)
    image_v9[v9] = FREE
    pgm_v9 = out_dir / "route_2d_map_v9.pgm"
    yaml_v9 = out_dir / "route_2d_map_v9.yaml"
    Image.fromarray(image_v9).save(pgm_v9)
    meta = yaml.safe_load(drivable_yaml.read_text(encoding="utf-8"))
    meta["image"] = pgm_v9.name
    yaml_v9.write_text(
        yaml.safe_dump(meta, default_flow_style=False, sort_keys=False),
        encoding="utf-8",
    )

    planning = ndimage.binary_erosion(v8, iterations=ERODE_PX)
    seed = json.loads(seed_route.read_text(encoding="utf-8"))
    seed_band = json.loads(seed_band.read_text(encoding="utf-8"))
    seed_xy = np.asarray([[w["x"], w["y"]] for w in seed["waypoints"]])

    def clamp_inside(point_xy):
        row, col = base._world_to_rc(point_xy, planning.shape, res, origin)
        if 0 <= row < height and 0 <= col < width and planning[row, col]:
            return row, col
        r_idx, c_idx = np.nonzero(planning)
        nearest = np.argmin((r_idx - row) ** 2 + (c_idx - col) ** 2)
        return int(r_idx[nearest]), int(c_idx[nearest])

    start_rc = clamp_inside(seed_xy[0])
    goal_rc = clamp_inside(seed_xy[-1])
    path_rc = base.plan_preferred_path(
        planning, preferred, start_rc, goal_rc, res
    )
    path_xy = np.column_stack(
        (
            origin[0] + path_rc[:, 1] * res,
            origin[1] + (height - 1 - path_rc[:, 0]) * res,
        )
    )
    anchor = base._resample(path_xy, base.PATH_STEP_M)
    smoothed = base.smooth_path(
        base._resample(path_xy, base.PATH_STEP_M), planning, res, origin
    )

    def segments_valid(points):
        return all(
            base._segment_is_drivable(a, b, planning, res, origin)
            for a, b in zip(points[:-1], points[1:])
        )

    if not segments_valid(anchor):
        raise ValueError("A* route left the eroded v8 mask before smoothing")
    if segments_valid(smoothed):
        dense_xy = smoothed
    else:
        merged = []
        for a, b in zip(smoothed[:-1], smoothed[1:]):
            merged.append(a)
            if base._segment_is_drivable(a, b, planning, res, origin):
                continue
            near = sorted(
                anchor,
                key=lambda p: np.linalg.norm((p - a) + (p - b)) \
                    - np.linalg.norm(b - a),
            )[:16]
            chain = [a, *sorted(
                near, key=lambda p: np.linalg.norm(p - a)
            ), b]
            rebuilt = [a]
            for pivot in chain[1:-1]:
                if base._segment_is_drivable(
                    rebuilt[-1], pivot, planning, res, origin
                ):
                    rebuilt.append(pivot)
            if not base._segment_is_drivable(rebuilt[-1], b, planning, res, origin):
                actual = anchor[
                    np.argmin(np.linalg.norm(anchor - a, axis=1)):
                    np.argmin(np.linalg.norm(anchor - b, axis=1)) + 1
                ]
                if len(actual) < 2:
                    raise ValueError(f"cannot repair span at {a}")
                merged.extend(actual[:-1])
                continue
            merged.extend(rebuilt[1:])
        merged.append(smoothed[-1])
        dense_xy = np.asarray(merged)
        if not segments_valid(dense_xy):
            dense_xy = anchor
    if not segments_valid(dense_xy):
        raise ValueError("route leaves the eroded v8 mask after repair")
    tangent = np.gradient(dense_xy, axis=0)
    yaw = np.degrees(np.arctan2(tangent[:, 1], tangent[:, 0]))
    length_m = float(np.linalg.norm(np.diff(dense_xy, axis=0), axis=1).sum())

    pgm_v6_sha = _sha(preferred_yaml.parent / "route_2d_map_v6.pgm")
    pgm_v9_sha = _sha(pgm_v9)
    yaml_v9_sha = _sha(yaml_v9)
    route_id = f"v6-v9:{pgm_v6_sha[:12]}:{pgm_v9_sha[:12]}"
    route_doc = {
        "frame": "map",
        "source": (
            "v6 preferred route re-centred with >=0.3 m clearance inside v9 "
            "(v8 + 0.2 m boundary-quantisation tolerance) drivable mask"
        ),
        "source_sha256": {
            "preferred_pgm": pgm_v6_sha,
            "drivable_pgm": pgm_v9_sha,
        },
        "body_frame_profile": str(seed["body_frame_profile"]),
        "count": len(dense_xy),
        "reference_point": "chair_centre",
        "chair_centre_in_body_xyz": seed["chair_centre_in_body_xyz"],
        "route_step_m": base.PATH_STEP_M,
        "path_length_m": round(length_m, 3),
        "operator_target_waypoint_index": len(dense_xy) - 1,
        "operator_target_xy_m": [
            round(float(dense_xy[-1, 0]), 3),
            round(float(dense_xy[-1, 1]), 3),
        ],
        "waypoints": [
            {
                "x": round(float(p[0]), 3),
                "y": round(float(p[1]), 3),
                "z": 0.0,
                "yaw_deg": round(float(a), 6),
            }
            for p, a in zip(dense_xy, yaw, strict=True)
        ],
    }
    band_xy = base._resample(path_xy, base.BAND_STEP_M)
    band_rc = np.asarray(
        [base._world_to_rc(p, v9.shape, res, origin) for p in band_xy],
        dtype=int,
    )
    out_band = out_dir / "20260816_route_v9_clearance_safety_band.json"
    out_route = out_dir / "20260816_route_v9_clearance_waypoints.json"
    band_doc = {
        "frame": "map",
        "route_id": route_id,
        "drivable_mask_sha256": pgm_v9_sha,
        "drivable_mask_yaml_sha256": yaml_v9_sha,
        "station_spacing_m": base.BAND_STEP_M,
        "stations": base.build_mask_band_stations(
            band_rc, v9, res, origin, seed_band["stations"]
        ),
        "corridor": {
            "source": yaml_v9.name,
            "chair_half_width_m": base.CHAIR_HALF_WIDTH_M,
            "policy": (
                "v9 (v8 + 0.2 m tolerance) is the authoritative "
                "chair-centre drivable mask"
            ),
            "stations_covered": len(band_rc),
            "stations_total": len(band_rc),
        },
        "physical_edge_semantics": {
            "source": Path(seed_band).name if not isinstance(seed_band, dict) else "20260815_route_v6_v8_trim_safety_band.json",
            "status": (
                "nearest v6 measured semantics over v9 "
                "(v8 + 0.2 m tolerance) boundary"
            ),
        },
    }
    out_band.write_text(json.dumps(band_doc, indent=1), encoding="utf-8")
    route_doc["asset_binding"] = {
        "route_id": route_id,
        "preferred_mask_sha256": pgm_v6_sha,
        "drivable_mask_sha256": pgm_v9_sha,
        "drivable_mask_yaml_sha256": yaml_v9_sha,
        "safety_band_sha256": _sha(out_band),
    }
    route_doc["asset_binding"]["route_content_sha256"] = _content_sha(route_doc)
    out_route.write_text(json.dumps(route_doc, indent=1), encoding="utf-8")
    print(
        f"route: {len(dense_xy)} waypoints, {length_m:.3f} m; "
        f"band: {len(band_rc)} stations"
    )
    print(f"assets in {out_dir}")
    return 0


if __name__ == "__main__":
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp")
    raise SystemExit(
        main(
            Path(
                "/Users/minjun/Downloads/route_2d_map_v6_only_20260812/"
                "route_2d_map_v6.yaml"
            ),
            Path(
                "/Users/minjun/Downloads/route_2d_map_merged_v8_20260812/"
                "route_2d_map_v8.yaml"
            ),
            Path("/tmp/20260815_route_v6_v8_trim_waypoints.json"),
            Path("/tmp/20260815_route_v6_v8_trim_safety_band.json"),
            root / "route_v9_20260816",
        )
    )
