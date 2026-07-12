#!/usr/bin/env python3
"""Compare three offline GLIM products as consistency evidence, never truth."""

import argparse
import ast
import csv
import hashlib
import json
import math
import os
import stat
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


ROOT_RECEIPT_FIELDS = {
    "schema_version", "artifact_id", "status", "execution_scope", "nuc_runtime_dependency",
    "claim_label", "qualification", "image", "source_revision", "glim_ros2_revision",
    "seed", "threads", "input_manifest_sha256", "ros2_database_sha256", "config_sha256",
    "config_entrypoint_sha256", "runs",
}
RUN_RECEIPT_FIELDS = {
    "run_id", "directory", "status", "failure", "pseudo_point_time_diagnostics",
    "returncode", "command", "image", "source_revision", "glim_ros2_revision",
    "seed", "threads", "elapsed_s", "artifacts",
}
REQUIRED_ARTIFACTS = {"trajectory.csv", "occupancy.pgm", "occupancy.yaml", "actual_output_evidence.json", "stdout.log", "stderr.log"}
CONFIG_DESTINATIONS = {
    "/opt/glim-config/config.json", "/opt/glim-config/config_preprocess.json",
    "/opt/glim-config/config_odometry_cpu.json", "/opt/glim-config/config_global_mapping_cpu.json",
}


def require_sha256(value, name):
    if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError("%s must be a lowercase SHA-256" % name)


def safe_path(path, name, directory=False):
    path = Path(path).absolute()
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        try:
            mode = os.lstat(current).st_mode
        except OSError as error:
            raise ValueError("%s is missing: %s" % (name, error))
        if stat.S_ISLNK(mode):
            raise ValueError("%s contains a symlink: %s" % (name, current))
    if directory:
        if not stat.S_ISDIR(mode):
            raise ValueError("%s is not a directory" % name)
    elif not stat.S_ISREG(mode):
        raise ValueError("%s is not a regular file" % name)
    return path


def receipt_path(root, relative, name, directory=False):
    relative = Path(relative)
    if relative.is_absolute() or any(part in ("", ".", "..") for part in relative.parts):
        raise ValueError("%s escapes its receipt root" % name)
    return safe_path(root / relative, name, directory)


def validate_artifacts(root, run_dir, artifacts):
    if not isinstance(artifacts, dict) or not REQUIRED_ARTIFACTS.issubset(artifacts):
        raise ValueError("run artifact receipt is skeletal")
    actual = set()
    for directory, dirs, files in os.walk(run_dir, followlinks=False):
        directory = Path(directory)
        if any((directory / child).is_symlink() for child in dirs):
            raise ValueError("run artifact tree contains a symlink")
        for child in files:
            relative = (directory / child).relative_to(run_dir).as_posix()
            if relative != "run_manifest.json":
                actual.add(relative)
    if set(artifacts) != actual:
        raise ValueError("run artifact receipt does not exactly enumerate regular artifacts")
    for relative, receipt in artifacts.items():
        if not isinstance(receipt, dict) or set(receipt) != {"sha256", "size_bytes"}:
            raise ValueError("artifact receipt schema is invalid: " + relative)
        require_sha256(receipt.get("sha256"), "artifact sha256")
        if isinstance(receipt.get("size_bytes"), bool) or not isinstance(receipt.get("size_bytes"), int) or receipt["size_bytes"] < 0:
            raise ValueError("artifact size is invalid: " + relative)
        artifact = receipt_path(root, Path(run_dir.name) / relative, "artifact " + relative)
        if artifact.stat().st_size != receipt["size_bytes"] or sha256_file(artifact) != receipt["sha256"]:
            raise ValueError("artifact hash mismatch: " + relative)


def receipt_bindings(command, image, manifest):
    if not isinstance(command, list) or not all(isinstance(item, str) for item in command):
        raise ValueError("run command receipt is invalid")
    required = {"--network=none", "--read-only", "--security-opt=no-new-privileges", "--cap-drop=ALL",
                "--env=OMP_NUM_THREADS=1", "--env=OPENBLAS_NUM_THREADS=1", "--env=MKL_NUM_THREADS=1",
                "--label=wheelchair.offline-only=true"}
    if not required.issubset(command) or image not in command:
        raise ValueError("run command is not the required offline immutable invocation")
    sources = {}
    for item in command:
        if item.startswith("--mount=type=bind,src=") and ",dst=" in item:
            source, destination = item[len("--mount=type=bind,src="):].split(",dst=", 1)
            destination = destination.split(",", 1)[0]
            if destination in CONFIG_DESTINATIONS | {"/input/rosbag2"}:
                if not source or destination in sources:
                    raise ValueError("input/config mount receipt is invalid")
                sources[destination] = source
    if set(sources) != CONFIG_DESTINATIONS | {"/input/rosbag2"}:
        raise ValueError("run command omits immutable input/config mounts")
    input_root = safe_path(sources["/input/rosbag2"], "input mount", directory=True)
    input_manifest = safe_path(input_root / "glim_rosbag2_manifest.json", "input manifest")
    database = safe_path(input_root / "normalized.db3", "input database")
    if sha256_file(input_manifest) != manifest["input_manifest_sha256"] or sha256_file(database) != manifest["ros2_database_sha256"]:
        raise ValueError("input/database receipt does not bind to command mount")
    config_paths = tuple(sources[name] for name in sorted(CONFIG_DESTINATIONS))
    for source in config_paths:
        safe_path(source, "config mount")
    config_root = Path(config_paths[0]).parent
    if any(Path(source).parent != config_root for source in config_paths):
        raise ValueError("config mounts do not share one immutable bundle")
    if (sha256_file(safe_path(config_root / "manifest.json", "config manifest")) != manifest["config_sha256"]
            or sha256_file(safe_path(sources["/opt/glim-config/config.json"], "config entrypoint")) != manifest["config_entrypoint_sha256"]):
        raise ValueError("config receipt does not bind to command mounts")
    return tuple(config_paths), str(input_root)


def validate_receipts(root):
    manifest = json.loads(receipt_path(root, "repro_manifest.json", "reproduction receipt").read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or set(manifest) != ROOT_RECEIPT_FIELDS:
        raise ValueError("reproduction receipt schema is invalid")
    fixed = {"schema_version": 2, "artifact_id": "wheelchair.glim-reproduction/v2", "status": "success",
             "execution_scope": "OFFLINE_WORKSTATION_ONLY", "nuc_runtime_dependency": False,
             "claim_label": "REPLAY_CONSISTENCY_NOT_TRUTH", "qualification": "candidate"}
    if any(manifest.get(key) != value for key, value in fixed.items()):
        raise ValueError("reproduction receipt immutable contract mismatch")
    for key in ("image", "source_revision", "glim_ros2_revision"):
        if not isinstance(manifest[key], str) or not manifest[key]:
            raise ValueError("reproduction receipt lacks " + key)
    if "@sha256:" not in manifest["image"]:
        raise ValueError("reproduction image is not digest-pinned")
    for key in ("input_manifest_sha256", "ros2_database_sha256", "config_sha256", "config_entrypoint_sha256"):
        require_sha256(manifest[key], key)
    if isinstance(manifest["seed"], bool) or not isinstance(manifest["seed"], int) or isinstance(manifest["threads"], bool) or manifest["threads"] != 1:
        raise ValueError("reproduction seed/thread receipt is invalid")
    if not isinstance(manifest["runs"], list) or len(manifest["runs"]) != 3:
        raise ValueError("reproduction receipt must contain exactly three runs")
    immutable, sources, validated = ("image", "source_revision", "glim_ros2_revision", "seed", "threads"), None, []
    for expected_id, receipt in enumerate(manifest["runs"], 1):
        directory = "run-%02d" % expected_id
        if not isinstance(receipt, dict) or set(receipt) != RUN_RECEIPT_FIELDS:
            raise ValueError("run receipt schema is invalid")
        if (receipt["run_id"] != expected_id or receipt["directory"] != directory or receipt["status"] != "success"
                or receipt["failure"] is not None or receipt["returncode"] != 0 or receipt["pseudo_point_time_diagnostics"] != []):
            raise ValueError("run failed or is not isolated")
        run_dir = receipt_path(root, directory, "run directory", directory=True)
        per_run = json.loads(receipt_path(root, directory + "/run_manifest.json", "per-run receipt").read_text(encoding="utf-8"))
        if per_run != receipt:
            raise ValueError("root and per-run receipts disagree")
        if any(receipt[key] != manifest[key] for key in immutable):
            raise ValueError("cross-run immutable receipt mismatch")
        current_sources = receipt_bindings(receipt["command"], manifest["image"], manifest)
        if sources is None:
            sources = current_sources
        elif sources != current_sources:
            raise ValueError("cross-run immutable input/config receipt mismatch")
        validate_artifacts(root, run_dir, receipt["artifacts"])
        validated.append((expected_id, run_dir, receipt))
    return validated


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repro-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--time-tolerance-s", type=float)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    report = {"schema_version": 1, "artifact_id": "wheelchair.glim-comparison/v1",
              "claim_label": "REPLAY_CONSISTENCY_NOT_TRUTH", "qualification": "candidate",
              "hardware_localization_accuracy_qualified": False,
              "thresholds": {"pairwise_position_rms_m": POSITION_RMS_LIMIT_M, "pairwise_yaw_rms_deg": YAW_RMS_LIMIT_DEG,
                             "occupancy_iou_min": OCCUPANCY_IOU_LIMIT, "loop_position_report_target_m": LOOP_POSITION_TARGET_M,
                             "loop_yaw_report_target_deg": LOOP_YAW_TARGET_DEG, "time_tolerance_s": TIME_TOLERANCE_S},
              "limitations": ["Rigid SE(2) alignment removes only frame origin and heading; it does not fit scale or time shift.",
                              "Consistency between repeated estimator runs is not localization or map truth.",
                              "This report cannot authorize wheelchair, NUC, route, campus, or passenger operation."],
              "runs": [], "pairs": [], "errors": []}
    trajectories, grids = {}, {}
    try:
        root = safe_path(args.repro_dir, "reproduction root", directory=True)
        if args.time_tolerance_s is not None and args.time_tolerance_s != TIME_TOLERANCE_S:
            raise ValueError("time tolerance is frozen at %s" % TIME_TOLERANCE_S)
        for run_id, run_dir, receipt in validate_receipts(root):
            trajectory_path = receipt_path(root, run_dir.name + "/trajectory.csv", "trajectory")
            pgm_path = receipt_path(root, run_dir.name + "/occupancy.pgm", "occupancy map")
            yaml_path = receipt_path(root, run_dir.name + "/occupancy.yaml", "occupancy metadata")
            trajectories[run_id], grids[run_id] = load_trajectory(trajectory_path), load_grid(pgm_path, yaml_path)
            report["runs"].append({"run_id": run_id, "status": receipt["status"], "directory": run_dir.name,
                                   "trajectory": gap_metrics(trajectories[run_id]), "loop_residual": loop_metrics(trajectories[run_id]),
                                   "trajectory_sha256": sha256_file(trajectory_path), "occupancy_sha256": grids[run_id]["sha256"]})
        for first_id, second_id in ((1, 2), (1, 3), (2, 3)):
            trajectory, grid = compare_trajectories(trajectories[first_id], trajectories[second_id], TIME_TOLERANCE_S), occupancy_iou(grids[first_id], grids[second_id])
            passed = (trajectory["position_rms_m"] <= POSITION_RMS_LIMIT_M and trajectory["yaw_rms_deg"] <= YAW_RMS_LIMIT_DEG and trajectory["time_coverage_pass"] and grid["geometry_equal"] and grid["iou"] >= OCCUPANCY_IOU_LIMIT)
            report["pairs"].append({"runs": [first_id, second_id], "trajectory": trajectory, "occupancy": grid, "ac3_repeatability_pass": passed})
    except (OSError, ValueError, json.JSONDecodeError, csv.Error) as error:
        report["errors"].append(str(error))
    report["loop_target_met_all_runs"] = len(report["runs"]) == 3 and all(run.get("loop_residual", {}).get("target_met", False) for run in report["runs"])
    if not report["loop_target_met_all_runs"]:
        report["limitations"].append("One or more loop-closure residuals exceed the diagnostic target; this does not change pairwise repeatability.")
    report["status"] = "pass" if (not report["errors"] and len(report["runs"]) == 3 and len(report["pairs"]) == 3 and all(pair["ac3_repeatability_pass"] for pair in report["pairs"])) else "fail"
    write_report(args.output, report)
    print(str(args.output))
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())