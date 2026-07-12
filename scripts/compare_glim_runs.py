#!/usr/bin/env python3
"""Compare three offline GLIM products as consistency evidence, never truth."""

import argparse
import ast
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

POSITION_RMS_LIMIT_M = 0.02
YAW_RMS_LIMIT_DEG = 0.2
OCCUPANCY_IOU_LIMIT = 0.99
LOOP_POSITION_TARGET_M = 0.50
LOOP_YAW_TARGET_DEG = 5.0
TIME_TOLERANCE_S = 1.0e-6


def wrap_angle(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def quaternion_yaw(row):
    qx, qy, qz, qw = (float(row[name]) for name in ("qx", "qy", "qz", "qw"))
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("invalid trajectory quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def load_trajectory(path):
    poses = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        time_name = "timestamp" if "timestamp" in fields else "time"
        required = {time_name, "x", "y"}
        if not required.issubset(fields) or not ({"yaw"}.issubset(fields) or {"qx", "qy", "qz", "qw"}.issubset(fields)):
            raise ValueError("trajectory.csv requires timestamp/time,x,y and yaw or quaternion")
        previous = None
        for row in reader:
            stamp = float(row[time_name])
            x, y = float(row["x"]), float(row["y"])
            yaw = float(row["yaw"]) if "yaw" in fields else quaternion_yaw(row)
            if not all(math.isfinite(value) for value in (stamp, x, y, yaw)):
                raise ValueError("trajectory contains a nonfinite value")
            if previous is not None and stamp <= previous:
                raise ValueError("trajectory timestamps are not strictly increasing")
            previous = stamp
            poses.append((stamp, x, y, wrap_angle(yaw)))
    if len(poses) < 2:
        raise ValueError("trajectory requires at least two poses")
    return poses


def parse_map_metadata(path):
    values = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip().strip("'\"")
    try:
        resolution = float(values["resolution"])
        origin = tuple(float(item) for item in ast.literal_eval(values["origin"]))
        negate = int(values.get("negate", "0"))
        occupied_thresh = float(values.get("occupied_thresh", "0.65"))
    except (KeyError, ValueError, SyntaxError, TypeError) as error:
        raise ValueError("invalid occupancy.yaml: %s" % error)
    if resolution <= 0.0 or len(origin) != 3 or negate not in (0, 1):
        raise ValueError("invalid map geometry")
    return resolution, origin, negate, occupied_thresh


def pgm_tokens(data):
    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and chr(data[index]).isspace():
            index += 1
        if index < len(data) and data[index] == ord("#"):
            while index < len(data) and data[index] not in (10, 13):
                index += 1
            continue
        start = index
        while index < len(data) and not chr(data[index]).isspace():
            index += 1
        if start == index:
            raise ValueError("truncated PGM header")
        tokens.append(data[start:index].decode("ascii"))
    return tokens, index


def load_grid(pgm_path, yaml_path):
    data = pgm_path.read_bytes()
    tokens, offset = pgm_tokens(data)
    magic, width_text, height_text, maximum_text = tokens
    width, height, maximum = int(width_text), int(height_text), int(maximum_text)
    if width <= 0 or height <= 0 or maximum <= 0 or maximum > 255:
        raise ValueError("unsupported PGM geometry")
    if magic == "P5":
        if offset >= len(data) or not chr(data[offset]).isspace():
            raise ValueError("PGM header has no pixel separator")
        offset += 2 if data[offset:offset + 2] == b"\r\n" else 1
        pixels = list(data[offset:])
    elif magic == "P2":
        pixels = [int(token) for token in data[offset:].decode("ascii").split() if not token.startswith("#")]
    else:
        raise ValueError("occupancy grid must be P2 or 8-bit P5 PGM")
    if len(pixels) != width * height:
        raise ValueError("PGM pixel count mismatch")
    resolution, origin, negate, threshold = parse_map_metadata(yaml_path)
    occupied = set()
    for index, pixel in enumerate(pixels):
        probability = (pixel / maximum) if negate else ((maximum - pixel) / maximum)
        if probability >= threshold:
            occupied.add(index)
    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": origin,
        "occupied": occupied,
        "sha256": sha256_file(pgm_path),
    }


def match_poses(first, second, tolerance):
    matches = []
    i = j = 0
    while i < len(first) and j < len(second):
        delta = first[i][0] - second[j][0]
        if abs(delta) <= tolerance:
            matches.append((first[i], second[j], delta))
            i += 1
            j += 1
        elif delta < 0.0:
            i += 1
        else:
            j += 1
    return matches


def path_length(poses):
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(poses, poses[1:]))


def gap_metrics(poses):
    gaps = [b[0] - a[0] for a, b in zip(poses, poses[1:])]
    ordered = sorted(gaps)
    median = ordered[len(ordered) // 2]
    return {
        "duration_s": poses[-1][0] - poses[0][0],
        "samples": len(poses),
        "median_gap_s": median,
        "max_gap_s": max(gaps),
        "gaps_over_2x_median": sum(gap > 2.0 * median for gap in gaps) if median > 0 else 0,
    }


def compare_trajectories(first, second, tolerance):
    matches = match_poses(first, second, tolerance)
    if len(matches) < 2:
        raise ValueError("insufficient timestamp-matched poses")
    ax = [item[0][1] for item in matches]
    ay = [item[0][2] for item in matches]
    bx = [item[1][1] for item in matches]
    by = [item[1][2] for item in matches]
    acx, acy = sum(ax) / len(ax), sum(ay) / len(ay)
    bcx, bcy = sum(bx) / len(bx), sum(by) / len(by)
    dot = sum((x - acx) * (u - bcx) + (y - acy) * (v - bcy) for x, y, u, v in zip(ax, ay, bx, by))
    cross = sum((x - acx) * (v - bcy) - (y - acy) * (u - bcx) for x, y, u, v in zip(ax, ay, bx, by))
    rotation = math.atan2(cross, dot)
    cosine, sine = math.cos(rotation), math.sin(rotation)
    tx = bcx - (cosine * acx - sine * acy)
    ty = bcy - (sine * acx + cosine * acy)
    position_sq = []
    yaw_sq = []
    for a, b, _ in matches:
        aligned_x = cosine * a[1] - sine * a[2] + tx
        aligned_y = sine * a[1] + cosine * a[2] + ty
        position_sq.append((aligned_x - b[1]) ** 2 + (aligned_y - b[2]) ** 2)
        yaw_sq.append(wrap_angle(a[3] + rotation - b[3]) ** 2)
    first_length, second_length = path_length(first), path_length(second)
    coverage = len(matches) / float(max(len(first), len(second)))
    return {
        "alignment": "rigid_SE2_no_scale_no_time_shift",
        "alignment_rotation_deg": math.degrees(rotation),
        "matched_samples": len(matches),
        "coverage_fraction": coverage,
        "max_abs_timestamp_delta_s": max(abs(item[2]) for item in matches),
        "duration_delta_s": abs((first[-1][0] - first[0][0]) - (second[-1][0] - second[0][0])),
        "path_length_ratio": first_length / second_length if second_length else (1.0 if first_length == 0 else None),
        "position_rms_m": math.sqrt(sum(position_sq) / len(position_sq)),
        "yaw_rms_deg": math.degrees(math.sqrt(sum(yaw_sq) / len(yaw_sq))),
        "time_coverage_pass": coverage == 1.0,
    }


def occupancy_iou(first, second):
    geometry_equal = all(first[key] == second[key] for key in ("width", "height", "resolution", "origin"))
    if not geometry_equal:
        return {"geometry_equal": False, "iou": 0.0}
    union = first["occupied"] | second["occupied"]
    intersection = first["occupied"] & second["occupied"]
    return {"geometry_equal": True, "iou": len(intersection) / len(union) if union else 1.0}


def loop_metrics(poses):
    position = math.hypot(poses[-1][1] - poses[0][1], poses[-1][2] - poses[0][2])
    yaw = abs(math.degrees(wrap_angle(poses[-1][3] - poses[0][3])))
    return {
        "position_residual_m": position,
        "yaw_residual_deg": yaw,
        "target_position_m": LOOP_POSITION_TARGET_M,
        "target_yaw_deg": LOOP_YAW_TARGET_DEG,
        "target_met": position <= LOOP_POSITION_TARGET_M and yaw <= LOOP_YAW_TARGET_DEG,
        "interpretation": "loop-closure diagnostic only; not absolute truth",
    }


def write_report(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repro-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--time-tolerance-s", type=float, default=TIME_TOLERANCE_S)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    report = {
        "schema_version": 1,
        "artifact_id": "wheelchair.glim-comparison/v1",
        "claim_label": "REPLAY_CONSISTENCY_NOT_TRUTH",
        "qualification": "candidate",
        "hardware_localization_accuracy_qualified": False,
        "thresholds": {
            "pairwise_position_rms_m": POSITION_RMS_LIMIT_M,
            "pairwise_yaw_rms_deg": YAW_RMS_LIMIT_DEG,
            "occupancy_iou_min": OCCUPANCY_IOU_LIMIT,
            "loop_position_report_target_m": LOOP_POSITION_TARGET_M,
            "loop_yaw_report_target_deg": LOOP_YAW_TARGET_DEG,
        },
        "limitations": [
            "Rigid SE(2) alignment removes only frame origin and heading; it does not fit scale or time shift.",
            "Consistency between repeated estimator runs is not localization or map truth.",
            "This report cannot authorize wheelchair, NUC, route, campus, or passenger operation.",
        ],
        "runs": [],
        "pairs": [],
        "errors": [],
    }
    trajectories = {}
    grids = {}
    try:
        root = args.repro_dir.resolve(strict=True)
        manifest = json.loads((root / "repro_manifest.json").read_text(encoding="utf-8"))
        runs = manifest.get("runs", [])
        if len(runs) != 3:
            raise ValueError("reproduction manifest must contain exactly three runs")
        for expected_id, run in enumerate(runs, 1):
            expected_directory = "run-%02d" % expected_id
            run_dir = root / expected_directory
            entry = {"run_id": expected_id, "status": run.get("status"), "directory": expected_directory}
            if (
                run.get("run_id") != expected_id
                or run.get("directory") != expected_directory
                or run.get("status") != "success"
            ):
                entry["error"] = "run failed or has invalid identity"
                report["errors"].append("run-%02d is not successful and isolated" % expected_id)
            recorded_artifacts = run.get("artifacts", {})
            for name in ("trajectory.csv", "occupancy.pgm", "occupancy.yaml"):
                artifact = recorded_artifacts.get(name, {})
                artifact_path = run_dir / name
                if not artifact_path.is_file() or artifact.get("sha256") != sha256_file(artifact_path):
                    entry["error"] = "missing or hash-mismatched " + name
                    report["errors"].append(
                        "run-%02d artifact hash invalid: %s" % (expected_id, name)
                    )
            try:
                trajectories[expected_id] = load_trajectory(run_dir / "trajectory.csv")
                grids[expected_id] = load_grid(run_dir / "occupancy.pgm", run_dir / "occupancy.yaml")
                entry["trajectory"] = gap_metrics(trajectories[expected_id])
                entry["loop_residual"] = loop_metrics(trajectories[expected_id])
                entry["trajectory_sha256"] = sha256_file(run_dir / "trajectory.csv")
                entry["occupancy_sha256"] = grids[expected_id]["sha256"]
            except (OSError, ValueError, csv.Error) as error:
                entry["error"] = str(error)
                report["errors"].append("run-%02d output invalid: %s" % (expected_id, error))
            report["runs"].append(entry)
        for first_id, second_id in ((1, 2), (1, 3), (2, 3)):
            if first_id not in trajectories or second_id not in trajectories or first_id not in grids or second_id not in grids:
                continue
            trajectory = compare_trajectories(trajectories[first_id], trajectories[second_id], args.time_tolerance_s)
            grid = occupancy_iou(grids[first_id], grids[second_id])
            passed = (
                trajectory["position_rms_m"] <= POSITION_RMS_LIMIT_M
                and trajectory["yaw_rms_deg"] <= YAW_RMS_LIMIT_DEG
                and trajectory["time_coverage_pass"]
                and grid["geometry_equal"]
                and grid["iou"] >= OCCUPANCY_IOU_LIMIT
            )
            report["pairs"].append({
                "runs": [first_id, second_id],
                "trajectory": trajectory,
                "occupancy": grid,
                "ac3_repeatability_pass": passed,
            })
    except (OSError, ValueError, json.JSONDecodeError) as error:
        report["errors"].append(str(error))
    report["loop_target_met_all_runs"] = (
        len(report["runs"]) == 3
        and all(run.get("loop_residual", {}).get("target_met", False) for run in report["runs"])
    )
    if not report["loop_target_met_all_runs"]:
        report["limitations"].append(
            "One or more loop-closure residuals exceed the diagnostic target; this does not change pairwise repeatability."
        )
    valid_successful_runs = (
        len(report["runs"]) == 3
        and all(
            run.get("status") == "success"
            and "error" not in run
            and "trajectory" in run
            and "loop_residual" in run
            for run in report["runs"]
        )
    )
    report["status"] = (
        "pass"
        if (
            not report["errors"]
            and valid_successful_runs
            and len(report["pairs"]) == 3
            and all(pair["ac3_repeatability_pass"] for pair in report["pairs"])
        )
        else "fail"
    )
    write_report(args.output, report)
    print(str(args.output))
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
