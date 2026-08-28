#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from scipy.ndimage import gaussian_filter1d
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree


def body_to_chair_centre(
    poses,
    chair_centre_in_body_xy=(-0.5, -0.2),
):
    corrected = np.asarray(poses, dtype=float).copy()
    yaw = corrected[:, 3]
    forward, left = chair_centre_in_body_xy
    corrected[:, 0] += np.cos(yaw) * forward - np.sin(yaw) * left
    corrected[:, 1] += np.sin(yaw) * forward + np.cos(yaw) * left
    return corrected


def load_pose_bag(path, topic="/fast_lio_icp/pose"):
    poses = []
    with AnyReader([Path(path)]) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic == topic
        ]
        if not connections:
            raise ValueError("%s has no %s messages" % (path, topic))
        for connection, timestamp, raw in reader.messages(
            connections=connections,
        ):
            message = reader.deserialize(raw, connection.msgtype)
            position = message.pose.pose.position
            quaternion = message.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (
                    quaternion.w * quaternion.z
                    + quaternion.x * quaternion.y
                ),
                1.0 - 2.0 * (
                    quaternion.y * quaternion.y
                    + quaternion.z * quaternion.z
                ),
            )
            poses.append((
                position.x,
                position.y,
                position.z,
                yaw,
                timestamp / 1e9,
            ))
    return np.asarray(poses, dtype=float)


def _resample(poses, spacing_m):
    segment = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(segment)])
    wanted = np.arange(0.0, arc[-1] + spacing_m * 0.5, spacing_m)
    yaw = np.unwrap(poses[:, 3])
    return np.column_stack([
        np.interp(wanted, arc, poses[:, 0]),
        np.interp(wanted, arc, poses[:, 1]),
        np.interp(wanted, arc, poses[:, 2]),
        np.interp(wanted, arc, yaw),
        np.interp(wanted, arc, poses[:, 4]),
    ])


def erase_spatial_loops(
    poses,
    revisit_radius_m=0.3,
    minimum_index_separation=10,
):
    poses = np.asarray(poses, dtype=float)
    stack = []
    cells = {}
    removed = 0

    def cell(point):
        return tuple(np.floor(
            point[:2] / revisit_radius_m).astype(int))

    def neighbors(point):
        col, row = cell(point)
        return (
            (col + dx, row + dy)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
        )

    for pose in poses:
        possible = []
        for key in neighbors(pose):
            possible.extend(cells.get(key, ()))
        revisits = [
            index
            for index in possible
            if index < len(stack) - minimum_index_separation
            and np.linalg.norm(stack[index][:2] - pose[:2])
            <= revisit_radius_m
        ]
        if revisits:
            keep_through = max(revisits)
            removed += len(stack) - keep_through - 1
            for index in range(len(stack) - 1, keep_through, -1):
                key = cell(stack[index])
                cells[key].remove(index)
                if not cells[key]:
                    del cells[key]
            stack = stack[:keep_through + 1]
        stack.append(pose.copy())
        index = len(stack) - 1
        cells.setdefault(cell(pose), set()).add(index)
    return np.asarray(stack), removed


def shortest_driven_path(
    poses,
    revisit_radius_m=0.3,
    minimum_index_separation=10,
    maximum_height_gap_m=0.15,
    maximum_heading_gap_deg=45.0,
):
    poses = np.asarray(poses, dtype=float)
    count = len(poses)
    temporal_weight = np.linalg.norm(
        np.diff(poses[:, :2], axis=0), axis=1)
    rows = list(range(count - 1))
    cols = list(range(1, count))
    weights = list(temporal_weight)
    pairs = np.array(
        list(cKDTree(poses[:, :2]).query_pairs(revisit_radius_m)),
        dtype=np.int64,
    )
    if len(pairs):
        heading_gap = np.abs(np.arctan2(
            np.sin(poses[pairs[:, 1], 3] - poses[pairs[:, 0], 3]),
            np.cos(poses[pairs[:, 1], 3] - poses[pairs[:, 0], 3]),
        ))
        keep = (
            (pairs[:, 1] - pairs[:, 0] > minimum_index_separation)
            & (
                np.abs(poses[pairs[:, 1], 2] - poses[pairs[:, 0], 2])
                <= maximum_height_gap_m
            )
            & (heading_gap <= math.radians(maximum_heading_gap_deg))
        )
        pairs = pairs[keep]
        rows.extend(pairs[:, 0])
        cols.extend(pairs[:, 1])
        weights.extend(np.linalg.norm(
            poses[pairs[:, 1], :2] - poses[pairs[:, 0], :2],
            axis=1,
        ))
    graph = coo_matrix(
        (weights, (rows, cols)),
        shape=(count, count),
    ).tocsr()
    _, predecessor = dijkstra(
        graph,
        directed=True,
        indices=0,
        return_predecessors=True,
    )
    indices = []
    index = count - 1
    while index >= 0:
        indices.append(index)
        if index == 0:
            break
        index = int(predecessor[index])
        if index < 0:
            raise RuntimeError("actual-drive pose graph is disconnected")
    return poses[indices[::-1]], count - len(indices)


def smooth_driven_path(
    poses,
    sample_m=0.1,
    sigma_m=0.5,
    output_spacing_m=0.2,
):
    poses = np.asarray(poses, dtype=float)
    segment = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(segment)])
    wanted = np.arange(0.0, arc[-1], sample_m)
    if not len(wanted) or wanted[-1] != arc[-1]:
        wanted = np.append(wanted, arc[-1])
    source_yaw = np.unwrap(poses[:, 3])
    sampled = np.column_stack([
        np.interp(wanted, arc, poses[:, 0]),
        np.interp(wanted, arc, poses[:, 1]),
        np.interp(wanted, arc, poses[:, 2]),
        np.interp(wanted, arc, source_yaw),
        np.interp(wanted, arc, poses[:, 4]),
    ])
    sigma_samples = sigma_m / sample_m
    smoothed = sampled.copy()
    smoothed[:, 0] = gaussian_filter1d(
        sampled[:, 0], sigma_samples, mode="nearest")
    smoothed[:, 1] = gaussian_filter1d(
        sampled[:, 1], sigma_samples, mode="nearest")
    endpoint_guard = max(2, int(math.ceil(sigma_samples * 2.0)))
    smoothed[:endpoint_guard, :2] = sampled[:endpoint_guard, :2]
    smoothed[-endpoint_guard:, :2] = sampled[-endpoint_guard:, :2]
    shift = np.linalg.norm(
        smoothed[:, :2] - sampled[:, :2], axis=1)
    output = smoothed[::max(1, int(round(output_spacing_m / sample_m)))]
    if not np.array_equal(output[-1], smoothed[-1]):
        output = np.vstack([output, smoothed[-1]])
    output[:, 3] = np.unwrap(np.arctan2(
        np.gradient(output[:, 1]),
        np.gradient(output[:, 0]),
    ))
    return output, {
        "maximum_smoothing_shift_m": float(shift.max()),
        "p95_smoothing_shift_m": float(np.percentile(shift, 95)),
    }


def stitch_driven_poses(
    first,
    second,
    start_xy,
    goal_xy,
    max_position_gap_m=0.2,
    max_heading_gap_deg=5.0,
    resample_m=0.2,
    erase_loops=True,
):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    start_xy = np.asarray(start_xy, dtype=float)
    goal_xy = np.asarray(goal_xy, dtype=float)
    start_index = int(np.argmin(
        np.linalg.norm(first[:, :2] - start_xy, axis=1)))
    goal_index = int(np.argmin(
        np.linalg.norm(second[:, :2] - goal_xy, axis=1)))

    distance, nearest = cKDTree(second[:, :2]).query(first[:, :2])
    heading_gap = np.abs(np.arctan2(
        np.sin(first[:, 3] - second[nearest, 3]),
        np.cos(first[:, 3] - second[nearest, 3]),
    ))
    candidates = np.nonzero(
        (np.arange(len(first)) >= start_index)
        & (nearest < goal_index)
        & (distance <= max_position_gap_m)
        & (heading_gap <= math.radians(max_heading_gap_deg))
    )[0]
    if not len(candidates):
        raise RuntimeError(
            "driven trajectories have no position-and-heading-consistent "
            "monotone overlap")
    first_stitch = int(candidates[-1])
    second_stitch = int(nearest[first_stitch])
    joined = np.vstack([
        first[start_index:first_stitch + 1],
        second[second_stitch + 1:goal_index + 1],
    ])
    removed_loops = 0
    if erase_loops:
        joined, removed_loops = shortest_driven_path(joined)
    joined, smoothing_audit = smooth_driven_path(
        joined,
        output_spacing_m=resample_m,
    )
    source_steps = np.linalg.norm(np.diff(joined[:, :2], axis=0), axis=1)
    dense = joined
    audit = {
        "start_index": start_index,
        "goal_index": goal_index,
        "first_stitch_index": first_stitch,
        "second_stitch_index": second_stitch,
        "start_residual_m": float(np.linalg.norm(
            first[start_index, :2] - start_xy)),
        "goal_residual_m": float(np.linalg.norm(
            second[goal_index, :2] - goal_xy)),
        "stitch_position_gap_m": float(distance[first_stitch]),
        "stitch_heading_gap_deg": float(math.degrees(
            heading_gap[first_stitch])),
        "maximum_source_step_m": float(source_steps.max()),
        "path_length_m": float(source_steps.sum()),
        "source_pose_count": len(joined),
        "route_point_count": len(dense),
        "loop_pose_count_removed": removed_loops,
        **smoothing_audit,
    }
    return dense, audit


def route_document(poses, first_bag, second_bag, audit):
    yaw_deg = np.degrees(poses[:, 3])
    return {
        "frame": "map",
        "source": "stitched physically driven localization trajectories",
        "source_bags": [str(first_bag), str(second_bag)],
        "body_frame_profile": "builtin",
        "reference_point": "chair_centre",
        "chair_centre_in_body_xyz": [-0.5, -0.2, 0.0],
        "route_step_m": 0.2,
        "count": len(poses),
        "path_length_m": round(audit["path_length_m"], 3),
        "stitch_audit": audit,
        "waypoints": [
            {
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "z": round(float(z), 3),
                "yaw_deg": round(float(yaw), 2),
            }
            for x, y, z, yaw in zip(
                poses[:, 0],
                poses[:, 1],
                poses[:, 2],
                yaw_deg,
            )
        ],
    }


def parse_xy(value):
    x, y = value.split(",")
    return float(x), float(y)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--first-bag", required=True)
    parser.add_argument("--second-bag", required=True)
    parser.add_argument("--start", required=True, type=parse_xy)
    parser.add_argument("--goal", required=True, type=parse_xy)
    parser.add_argument("--output-route", required=True)
    parser.add_argument("--output-report", required=True)
    args = parser.parse_args(argv)

    first = body_to_chair_centre(load_pose_bag(args.first_bag))
    second = body_to_chair_centre(load_pose_bag(args.second_bag))
    poses, audit = stitch_driven_poses(
        first,
        second,
        args.start,
        args.goal,
    )
    route = route_document(
        poses,
        args.first_bag,
        args.second_bag,
        audit,
    )
    with open(args.output_route, "w") as handle:
        json.dump(route, handle, indent=1)
    report = {
        "status": "RECONSTRUCTED",
        "frame": "map",
        "route": args.output_route,
        **audit,
    }
    with open(args.output_report, "w") as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
