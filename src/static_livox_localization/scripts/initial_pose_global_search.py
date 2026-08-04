"""ROS-free global fallback for the automatic initial-pose node."""

import math
from pathlib import Path
from typing import List, NamedTuple, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial import cKDTree

from initial_pose_candidates import InitializationCandidate


class BinaryPcdError(Exception):
    """A stable PCD boundary error."""

    def __init__(self, reason: str, path: Path):
        self.reason = reason
        self.path = path
        super().__init__(reason, str(path))

    def __str__(self) -> str:
        return "{}: {}".format(self.reason, self.path)


def load_pcd_xyz(path: Path) -> np.ndarray:
    """Read the exact binary XYZI layout required by the runtime map."""

    try:
        with path.open("rb") as source:
            header_lines = []
            for _ in range(64):
                line = source.readline()
                if not line:
                    raise BinaryPcdError("unsupported_header", path)
                header_lines.append(line)
                if line.startswith(b"DATA "):
                    break
            else:
                raise BinaryPcdError("unsupported_header", path)
            header = b"".join(header_lines)
            if b"FIELDS x y z intensity\n" not in header:
                raise BinaryPcdError("unsupported_fields", path)
            if b"SIZE 4 4 4 4\n" not in header or b"TYPE F F F F\n" not in header:
                raise BinaryPcdError("unsupported_fields", path)
            if not header.endswith(b"DATA binary\n"):
                raise BinaryPcdError("unsupported_encoding", path)
            point_line = next(
                (line for line in header_lines if line.startswith(b"POINTS ")),
                None,
            )
            if point_line is None:
                raise BinaryPcdError("missing_point_count", path)
            try:
                point_count = int(point_line.split()[1])
            except (IndexError, ValueError) as error:
                raise BinaryPcdError("invalid_point_count", path) from error
            payload = source.read()
    except OSError as error:
        raise BinaryPcdError("unreadable_pcd", path) from error

    expected_bytes = point_count * 16
    if len(payload) != expected_bytes:
        raise BinaryPcdError("payload_size_mismatch", path)
    records = np.frombuffer(payload, dtype=np.float32).reshape(point_count, 4)
    points = records[:, :3]
    return points[np.isfinite(points).all(axis=1)]


def load_trajectory_candidates(
    trajectory_path: Path,
    spacing_m: float,
) -> Tuple[Tuple[float, float, float, float], ...]:
    """Sample position and heading hypotheses along the mapping trajectory."""

    rows = np.loadtxt(str(trajectory_path), ndmin=2)
    positions = rows[:, 1:4]
    keep = [0]
    for index in range(1, len(positions)):
        if (
            np.linalg.norm(positions[index, :2] - positions[keep[-1], :2])
            >= spacing_m
        ):
            keep.append(index)
    candidates = []
    for index in keep:
        x, y, z = positions[index]
        nxt = positions[min(index + 5, len(positions) - 1)]
        heading = math.atan2(nxt[1] - y, nxt[0] - x)
        candidates.append((float(x), float(y), float(z), heading))
    return tuple(candidates)


# Ground carries no horizontal information: slide a pavement scan a metre in
# any direction and every ground point still lands on ground. On this route the
# accumulated submap is roughly 70 percent ground, so scoring the raw sample
# spends 70 percent of the metric - and 70 percent of the point budget - on
# points that cannot tell one place from another. Measured on a synthetic
# streetscape, dropping it widened the gap between the true pose and a wrong
# place 22 m away from 0.11 to 0.36 of the inlier fraction.
GROUND_MARGIN_M = 0.4
MIN_STRUCTURAL_POINTS = 150


def structural_sample(
    points: np.ndarray,
    ground_margin_m: float = GROUND_MARGIN_M,
    minimum: int = MIN_STRUCTURAL_POINTS,
) -> Tuple[np.ndarray, bool]:
    """Drop ground-height returns, keeping what actually fixes a position.

    The ground plane is found from a low percentile of z rather than an assumed
    sensor height, so this holds on a slope and needs no calibration. Returns
    the filtered points and whether the filter was applied: somewhere with too
    little vertical structure to score, the full sample is better than nothing,
    and the caller should know the fix is weakly constrained.
    """

    if len(points) == 0:
        return points, False
    ground_z = float(np.percentile(points[:, 2], 5.0))
    structure = points[points[:, 2] > ground_z + ground_margin_m]
    if len(structure) < minimum:
        return points, False
    return structure, True


def voxel_downsample(points: np.ndarray, size_m: float, cap: int) -> np.ndarray:
    """Select one deterministic representative per voxel and cap the sample."""

    keys = np.floor(points / size_m).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    sampled = points[np.sort(unique_idx)]
    if len(sampled) > cap:
        sampled = sampled[
            np.random.RandomState(0).choice(len(sampled), cap, False)
        ]
    return sampled


# The coarse pass only has to BRACKET the answer closely enough that
# refinement can walk onto it. Six offsets (45 deg apart) left a 22.5 deg
# worst case, which brackets nothing: 22.5 deg displaces a point 10 m out by
# 3.9 m, so the correct trajectory sample scored no better than open ground
# and never reached the shortlist. 30 deg steps halve that to 15 deg, which
# is exactly what REFINE_YAW_RADIUS_RAD searches.
COARSE_YAW_STEP_RAD = math.radians(30.0)

# Trajectory candidates sit exactly on the recorded line, so the chair is off
# them by however far it was parked to the side plus half the sample spacing.
# The window has to reach that far, and the step has to land inside the
# localizer's 0.5 m correspondence.
REFINE_POSITION_RADIUS_M = 1.0
REFINE_POSITION_STEP_M = 0.25
REFINE_YAW_RADIUS_RAD = math.radians(15.0)
REFINE_YAW_STEP_RAD = math.radians(3.0)
# A window that lands on its own edge is re-centred and searched again, so an
# offset larger than one window still converges without paying for a grid
# sized to the worst case on every hypothesis.
REFINE_MAX_ROUNDS = 3


def coarse_yaw_offsets() -> Tuple[float, ...]:
    """Yaw hypotheses per trajectory sample, evenly covering the circle."""

    count = int(round(2.0 * math.pi / COARSE_YAW_STEP_RAD))
    return tuple(index * 2.0 * math.pi / count for index in range(count))


def _rotation(yaw: float) -> np.ndarray:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]],
        np.float32,
    )


def _placement(tree, world: np.ndarray, inlier_radius_m: float):
    """Score a placed sample two ways: how much matches, and how closely.

    The inlier fraction is the robust measure for shortlisting - a hypothesis
    in the wrong place simply has few inliers - but it saturates. Every offset
    smaller than the radius keeps a planar surface inside it, so on pavement
    and building walls the fraction reads 1.0 across a band as wide as the
    radius itself and gives refinement nothing to descend. The mean truncated
    distance keeps a gradient inside that band: outliers cost the radius,
    inliers cost what they actually miss by.
    """

    distances, _ = tree.query(world, k=1, distance_upper_bound=inlier_radius_m)
    matched = np.isfinite(distances)
    cost = float(np.where(matched, distances, inlier_radius_m).mean())
    return cost, float(matched.mean())


def _inlier_fraction(tree, world: np.ndarray, inlier_radius_m: float) -> float:
    """Fraction of the placed sample that finds map support within the radius."""

    return _placement(tree, world, inlier_radius_m)[1]


# One query per hypothesis leaves cKDTree walking the tree from Python 9336
# times for a coarse pass, with every core but one idle. Stacking the
# hypotheses and asking once is the same tree and the same query - measured
# bit-identical over the deployed map, max difference 0.0 - and 3.5x faster
# on this hardware. It also puts the work in the shape an accelerator needs:
# one large array of placed points rather than a Python loop, which is what
# array_backend exists to move to the GPU next.
#
# Chunked because the coarse array is (hypotheses x sample x 3) - 200 MB at
# the shipped 1800-point sample - and the NUC has other things to hold.
PLACEMENT_CHUNK = 1024


def _placement_batch(tree, worlds: np.ndarray, inlier_radius_m: float):
    """Cost and inlier fraction for a stack of placed samples.

    worlds is (hypotheses, points, 3). Returns two (hypotheses,) arrays with
    exactly the quantities _placement returns for one of them.
    """

    count = len(worlds)
    costs = np.empty(count, np.float64)
    inliers = np.empty(count, np.float64)
    for start in range(0, count, PLACEMENT_CHUNK):
        block = worlds[start:start + PLACEMENT_CHUNK]
        distances, _ = tree.query(
            block.reshape(-1, 3),
            k=1,
            distance_upper_bound=inlier_radius_m,
            workers=-1,
        )
        distances = distances.reshape(len(block), -1)
        matched = np.isfinite(distances)
        costs[start:start + len(block)] = np.where(
            matched, distances, inlier_radius_m).mean(axis=1)
        inliers[start:start + len(block)] = matched.mean(axis=1)
    return costs, inliers


def _grid(radius: float, step: float) -> Tuple[float, ...]:
    """Symmetric offsets including zero, so refining cannot lose an answer."""

    if radius <= 0.0 or step <= 0.0:
        return (0.0,)
    count = int(round(radius / step))
    return tuple(index * step for index in range(-count, count + 1))


def score_global_candidates(
    sample: np.ndarray,
    map_points: np.ndarray,
    candidates: Sequence[Tuple[float, float, float, float]],
    inlier_radius_m: float,
) -> Tuple[InitializationCandidate, ...]:
    """Rank trajectory/yaw hypotheses by bounded nearest-map inlier fraction."""

    tree = cKDTree(map_points)
    poses = [(x, y, z, heading + offset)
             for x, y, z, heading in candidates
             for offset in coarse_yaw_offsets()]
    if not poses:
        return ()
    worlds = np.empty((len(poses), len(sample), 3), np.float32)
    for index, (x, y, z, yaw) in enumerate(poses):
        worlds[index] = sample @ _rotation(yaw).T + np.array(
            [x, y, z], np.float32)
    _costs, inliers = _placement_batch(tree, worlds, inlier_radius_m)
    scored = [
        InitializationCandidate(x=x, y=y, z=z, yaw_rad=yaw,
                                score=float(inliers[index]),
                                source="global_search")
        for index, (x, y, z, yaw) in enumerate(poses)
    ]
    return _best_first(scored)


# Two hypotheses closer than this, facing nearly the same way, are the same
# answer: the refinement window would walk them onto the same pose. Kept just
# under the trajectory sampling spacing so adjacent samples stay distinct.
SHORTLIST_MIN_SEPARATION_M = 2.5
SHORTLIST_MIN_SEPARATION_YAW_RAD = math.radians(45.0)


def diverse_shortlist(
    candidates: Sequence[InitializationCandidate],
    count: int,
    min_separation_m: float = SHORTLIST_MIN_SEPARATION_M,
    min_separation_yaw_rad: float = SHORTLIST_MIN_SEPARATION_YAW_RAD,
) -> Tuple[InitializationCandidate, ...]:
    """Pick the best hypotheses that are actually different from each other.

    Each attempt downstream costs a localizer reset and up to twenty seconds of
    verification, so the budget has to buy distinct answers. A plain top-N does
    not: the coarse pass scores every yaw at every trajectory sample, and a
    self-similar sidewalk hands back one spot at neighbouring headings. Facing
    the other way from the same spot is kept - a chair parked backwards is the
    likeliest mistake there is.
    """

    selected: List[InitializationCandidate] = []
    for candidate in _best_first(candidates):
        if len(selected) >= count:
            break
        duplicate = False
        for chosen in selected:
            gap = math.hypot(candidate.x - chosen.x, candidate.y - chosen.y)
            heading_gap = abs(
                (candidate.yaw_rad - chosen.yaw_rad + math.pi)
                % (2.0 * math.pi)
                - math.pi
            )
            if gap < min_separation_m and heading_gap < min_separation_yaw_rad:
                duplicate = True
                break
        if not duplicate:
            selected.append(candidate)
    return tuple(selected)


# A wrong initial pose is worse than no initial pose: the localizer verifies
# that a candidate is self-consistent, not that it is the right place, so a
# plausible wrong answer passes verification and the follower then drives a
# route computed from a belief that is metres or a whole heading out. When two
# genuinely different poses explain the same scan this closely, the scan does
# not identify the place and the honest answer is to say so.
AMBIGUITY_MARGIN = 0.08
AMBIGUITY_SEPARATION_M = 5.0
AMBIGUITY_SEPARATION_YAW_RAD = math.radians(45.0)

# A relative margin alone is not enough. On a self-similar stretch the wrong
# answer can win outright with no close rival - measured at 0.65 while the
# correct pose in identifiable surroundings scores 1.0 - so a global fix also
# has to be strongly supported in absolute terms. Refined and scored on
# structure only, a correct pose puts nearly all of its points on mapped
# structure; 0.65 is what a place 21 m away managed.
#
# FIELD CALIBRATION: this number comes from synthetic scenes, not from the
# route. Transient structure the map does not contain - parked cars, people -
# lowers a correct fix, so the first real runs may need it relaxed. It is a
# node parameter for exactly that reason. Err high: refusing costs a
# repositioning, accepting a wrong pose drives the chair off the route.
MIN_REFINED_SCORE = 0.80


def unambiguous_best(
    refined: Sequence[InitializationCandidate],
    margin: float = AMBIGUITY_MARGIN,
    separation_m: float = AMBIGUITY_SEPARATION_M,
    separation_yaw_rad: float = AMBIGUITY_SEPARATION_YAW_RAD,
) -> Tuple[Optional[InitializationCandidate], Optional[InitializationCandidate]]:
    """The winner, or nothing plus the rival that made the fix ambiguous.

    Returns (best, None) when one pose stands clear, and (None, rival) when a
    distinct pose - far away, or facing a different way from the same place -
    explains the scan within the margin. A backwards fix counts as a rival
    precisely because it is the mistake with the worst consequences.
    """

    ordered = _best_first(refined)
    if not ordered:
        return None, None
    best = ordered[0]
    if best.score is None:
        return None, None
    for other in ordered[1:]:
        if other.score is None or other.score < best.score - margin:
            continue
        gap = math.hypot(other.x - best.x, other.y - best.y)
        heading_gap = abs(
            (other.yaw_rad - best.yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
        )
        if gap > separation_m or heading_gap > separation_yaw_rad:
            return None, other
    return best, None


class FixDecision(NamedTuple):
    """Whether the map identified where the chair is, and why not if it did not."""

    candidate: Optional[InitializationCandidate]
    reason: str
    rival: Optional[InitializationCandidate]


def decide_fix(
    refined: Sequence[InitializationCandidate],
    min_refined_score: float = MIN_REFINED_SCORE,
    margin: float = AMBIGUITY_MARGIN,
    separation_m: float = AMBIGUITY_SEPARATION_M,
    separation_yaw_rad: float = AMBIGUITY_SEPARATION_YAW_RAD,
) -> FixDecision:
    """Accept a global fix only when the map actually identifies the place.

    Two independent ways to fail, both of which were observed in measurement
    rather than imagined:

    - ambiguous: a genuinely different pose explains the scan within the
      margin, so the scan does not pick between them.
    - weak_support: one pose wins outright but does not explain enough of what
      the chair can see to be believed.

    Either way the answer is no pose rather than a doubtful one. Downstream
    verification cannot be asked to catch this: it proves a candidate is
    self-consistent, not that it is the right place.
    """

    best, rival = unambiguous_best(
        refined, margin, separation_m, separation_yaw_rad
    )
    if rival is not None:
        return FixDecision(None, "ambiguous", rival)
    if best is None:
        return FixDecision(None, "no_candidates", None)
    if best.score is None or best.score < min_refined_score:
        return FixDecision(None, "weak_support", best)
    return FixDecision(best, "accepted", None)


def refine_candidates(
    sample: np.ndarray,
    map_points: np.ndarray,
    candidates: Sequence[InitializationCandidate],
    inlier_radius_m: float,
    position_radius_m: float = REFINE_POSITION_RADIUS_M,
    position_step_m: float = REFINE_POSITION_STEP_M,
    yaw_radius_rad: float = REFINE_YAW_RADIUS_RAD,
    yaw_step_rad: float = REFINE_YAW_STEP_RAD,
    max_rounds: int = REFINE_MAX_ROUNDS,
) -> Tuple[InitializationCandidate, ...]:
    """Walk each shortlisted hypothesis onto the map with the same metric.

    A bracketing hypothesis is not a usable seed: the localizer matches at
    0.5 m correspondence, so a metre of lateral offset or fifteen degrees of
    heading leaves ICP with nothing to lock onto and verification times out.
    Searching a small grid around the coarse winner costs a few seconds once
    and hands over a pose that is already inside the basin.

    The refined pose keeps its source, so it stays a score-gated fallback
    rather than becoming as trusted as the known-start prior.
    """

    tree = cKDTree(map_points)
    position_offsets = _grid(position_radius_m, position_step_m)
    yaw_offsets = _grid(yaw_radius_rad, yaw_step_rad)
    refined: List[InitializationCandidate] = []
    for candidate in candidates:
        centre = candidate
        settled = None
        for _ in range(max(max_rounds, 1)):
            best = _best_in_window(
                tree,
                sample,
                centre,
                position_offsets,
                yaw_offsets,
                inlier_radius_m,
            )
            if best is None:
                break
            settled = best
            moved = max(abs(best.x - centre.x), abs(best.y - centre.y))
            turned = abs(best.yaw_rad - centre.yaw_rad)
            centre = best
            # A winner sitting on the edge of its own window means the answer
            # is probably outside it. That is the normal case off the recorded
            # line: half the trajectory spacing plus however far the chair was
            # parked to the side can exceed one window. Re-centre and search
            # again rather than inflating every grid to cover the worst case.
            on_boundary = (
                moved >= position_radius_m - 1e-9
                or turned >= yaw_radius_rad - 1e-9
            )
            if not on_boundary:
                break
        if settled is not None:
            refined.append(settled)
    return _best_first(refined)


def _best_in_window(
    tree,
    sample: np.ndarray,
    centre: InitializationCandidate,
    position_offsets: Sequence[float],
    yaw_offsets: Sequence[float],
    inlier_radius_m: float,
) -> Optional[InitializationCandidate]:
    """Lowest-cost pose on one local grid around a hypothesis."""

    placed = []
    worlds = np.empty(
        (len(yaw_offsets) * len(position_offsets) ** 2, len(sample), 3),
        np.float32)
    index = 0
    # Built in the original yaw -> dx -> dy order, because the scan below
    # keeps the FIRST minimum and the loop it replaces kept the first too
    # (it skipped on cost >= best_cost). A different order would silently
    # pick a different pose out of a tie.
    for yaw_offset in yaw_offsets:
        yaw = centre.yaw_rad + yaw_offset
        # One rotation per heading, then translation only: the nearest
        # neighbour queries dominate and this keeps them the only cost that
        # scales with the grid.
        rotated = sample @ _rotation(yaw).T
        for dx in position_offsets:
            for dy in position_offsets:
                worlds[index] = rotated + np.array(
                    [centre.x + dx, centre.y + dy, centre.z], np.float32)
                placed.append((centre.x + dx, centre.y + dy, yaw))
                index += 1
    if not placed:
        return None
    costs, inliers = _placement_batch(tree, worlds, inlier_radius_m)
    winner = int(np.argmin(costs))
    x, y, yaw = placed[winner]
    # The reported score stays the inlier fraction: it is what the
    # minimum-score gate downstream is calibrated against.
    return InitializationCandidate(
        x=x, y=y, z=centre.z, yaw_rad=yaw,
        score=float(inliers[winner]), source=centre.source)


def _best_first(
    candidates: Sequence[InitializationCandidate],
) -> Tuple[InitializationCandidate, ...]:
    return tuple(
        sorted(
            candidates,
            key=lambda item: item.score if item.score is not None else -math.inf,
            reverse=True,
        )
    )
