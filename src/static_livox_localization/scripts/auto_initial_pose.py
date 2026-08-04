#!/usr/bin/env python3
# noqa: SIZE_OK — one ROS initialization transaction owns shared callback state
"""Verified map-based localization with an optional known-start shortcut.

The first waypoint of the selected route is an explicit pose prior when the
wheelchair is placed at its known start. It is sent directly to the localizer,
which still requires ICP consensus (VERIFYING -> TRACKING). Only when that
normal initialization fails does this node build a KD-tree and run the more
expensive global trajectory search.
"""

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool

import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft

from initial_pose_candidates import (
    KnownStartRouteError,
    StationaryStabilityMonitor,
    initialization_attempts,
    load_known_start,
    seed_was_acknowledged,
    tracking_was_verified,
)
from initial_pose_global_search import (
    decide_fix,
    diverse_shortlist,
    load_pcd_xyz,
    load_trajectory_candidates,
    refine_candidates,
    score_global_candidates,
    structural_sample,
    voxel_downsample,
)


class SubmapCollector:
    def __init__(self, window_s):
        self.window_s = window_s
        self.odom = []
        self.clouds = []
        self.odom_sub = rospy.Subscriber(
            "/Odometry", Odometry, self.on_odom, queue_size=100
        )
        self.cloud_sub = rospy.Subscriber(
            "/cloud_registered_body", PointCloud2, self.on_cloud, queue_size=10)

    def on_odom(self, message):
        q = message.pose.pose.orientation
        p = message.pose.pose.position
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = (p.x, p.y, p.z)
        self.odom.append((message.header.stamp.to_sec(), T))
        self.odom = self.odom[-400:]

    def on_cloud(self, message):
        pts = np.array(list(pc2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)
        self.clouds.append((message.header.stamp.to_sec(), pts))
        cutoff = message.header.stamp.to_sec() - self.window_s - 1.0
        self.clouds = [c for c in self.clouds if c[0] >= cutoff]

    def build(self):
        if not self.clouds or not self.odom:
            return None
        newest = self.clouds[-1][0]
        odom_t = np.array([t for t, _ in self.odom])
        ref = None
        merged = []
        for stamp, pts in self.clouds:
            if newest - stamp > self.window_s or len(pts) == 0:
                continue
            k = int(np.argmin(np.abs(odom_t - stamp)))
            if abs(odom_t[k] - stamp) > 0.12:
                continue
            T = self.odom[k][1]
            if ref is None or stamp == newest:
                ref = T
            hom = np.hstack([pts, np.ones((len(pts), 1), np.float32)])
            merged.append((T @ hom.T).T[:, :3])
        if not merged or ref is None:
            return None
        world = np.vstack(merged)
        inv = np.linalg.inv(ref)
        hom = np.hstack([world, np.ones((len(world), 1))])
        return (inv @ hom.T).T[:, :3].astype(np.float32)


def try_candidate(candidate, rank, state, seed_pub, enable, verify_timeout):
    """Publish one seed and let the downstream ICP consensus accept or reject it."""

    score_text = "prior" if candidate.score is None else "{:.3f}".format(
        candidate.score
    )
    rospy.loginfo(
        "trying candidate %d: source=%s score=%s (%.1f, %.1f) yaw=%.0f",
        rank,
        candidate.source,
        score_text,
        candidate.x,
        candidate.y,
        np.degrees(candidate.yaw_rad),
    )
    try:
        rospy.wait_for_service(
            "/fast_lio_icp/enable_auto_correction", timeout=10.0
        )
    except rospy.ROSException as error:
        rospy.logerr("auto-correction service unavailable: %s", error)
        return False
    connection_deadline = rospy.Time.now() + rospy.Duration(5.0)
    while (
        not rospy.is_shutdown()
        and seed_pub.get_num_connections() == 0
        and rospy.Time.now() < connection_deadline
    ):
        rospy.sleep(0.1)
    if seed_pub.get_num_connections() == 0:
        rospy.logerr("initial-pose subscriber unavailable")
        return False

    diagnostic_deadline = rospy.Time.now() + rospy.Duration(5.0)
    while (
        not rospy.is_shutdown()
        and state["reset_count"] is None
        and rospy.Time.now() < diagnostic_deadline
    ):
        rospy.sleep(0.1)
    if state["reset_count"] is None:
        rospy.logerr("localization diagnostics unavailable")
        return False

    try:
        disabled = enable(False)
    except rospy.ServiceException as error:
        rospy.logerr(
            "failed to disable correction before candidate %d: %s",
            rank,
            error,
        )
        return False
    if not disabled.success:
        rospy.logerr(
            "failed to disable correction before candidate %d: %s",
            rank,
            disabled.message,
        )
        return False

    baseline_reset_count = state["reset_count"]
    baseline_sequence = state["sequence"]
    seed = PoseWithCovarianceStamped()
    seed.header.frame_id = "map"
    seed.header.stamp = rospy.Time.now()
    seed.pose.pose.position.x = candidate.x
    seed.pose.pose.position.y = candidate.y
    seed.pose.pose.position.z = candidate.z
    q = tft.quaternion_from_euler(0, 0, candidate.yaw_rad)
    seed.pose.pose.orientation.x = q[0]
    seed.pose.pose.orientation.y = q[1]
    seed.pose.pose.orientation.z = q[2]
    seed.pose.pose.orientation.w = q[3]
    seed_pub.publish(seed)

    acknowledgement_deadline = rospy.Time.now() + rospy.Duration(5.0)
    while not rospy.is_shutdown() and rospy.Time.now() < acknowledgement_deadline:
        if seed_was_acknowledged(
            state, baseline_sequence, baseline_reset_count
        ):
            break
        rospy.sleep(0.1)
    else:
        rospy.logwarn("candidate seed was not acknowledged")
        return False

    candidate_reset_count = state["reset_count"]
    before_enable_sequence = state["sequence"]
    try:
        response = enable(True)
    except rospy.ServiceException as error:
        rospy.logwarn("candidate %d could not be enabled: %s", rank, error)
        disable_correction(enable, rank)
        return False
    if not response.success:
        rospy.logwarn("candidate %d was not enabled: %s", rank, response.message)
        disable_correction(enable, rank)
        return False

    saw_verifying = response.message == "VERIFYING"
    verify_deadline = rospy.Time.now() + rospy.Duration(verify_timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < verify_deadline:
        if (
            state["reset_count"] == candidate_reset_count
            and state["message"] == "VERIFYING"
        ):
            saw_verifying = True
        if tracking_was_verified(
            state,
            before_enable_sequence,
            candidate_reset_count,
            saw_verifying,
        ):
            rospy.loginfo("initialized: candidate %d verified (TRACKING)", rank)
            return True
        rospy.sleep(0.5)
    rospy.logwarn("candidate %d failed verification", rank)
    disable_correction(enable, rank)
    return False


def disable_correction(enable, rank):
    """Fail closed so a timed-out candidate cannot later become active."""

    try:
        response = enable(False)
    except rospy.ServiceException as error:
        rospy.logerr(
            "failed to disable correction after candidate %d: %s", rank, error
        )
        return
    if not response.success:
        rospy.logerr(
            "correction remained enabled after candidate %d: %s",
            rank,
            response.message,
        )


def planar_pose(pose):
    orientation = pose.orientation
    yaw = tft.euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )[2]
    return (pose.position.x, pose.position.y, yaw)


def wait_for_stationary_stability(
    state,
    expected_reset_count,
    duration_s,
):
    monitor = StationaryStabilityMonitor()
    initial_pose_sequence = state["pose_sequence"]
    initial_odom_sequence = state["odom_sequence"]
    observed_sequences = None
    observations = 0
    deadline = rospy.Time.now() + rospy.Duration(duration_s)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if (
            state["message"] != "TRACKING"
            or state["reset_count"] != expected_reset_count
        ):
            rospy.logerr(
                "localization left TRACKING during stationary verification: %s",
                state["message"],
            )
            return False
        sequences = (state["pose_sequence"], state["odom_sequence"])
        if (
            state["localization_pose"] is not None
            and state["odometry_pose"] is not None
            and sequences != observed_sequences
        ):
            refusal = monitor.observe(
                state["localization_pose"], state["odometry_pose"]
            )
            if refusal is not None:
                rospy.logerr("stationary localization verification failed: %s", refusal)
                return False
            observed_sequences = sequences
            observations += 1
        rospy.sleep(0.1)
    fresh_pose = state["pose_sequence"] > initial_pose_sequence
    fresh_odom = state["odom_sequence"] > initial_odom_sequence
    if observations < 10 or not fresh_pose or not fresh_odom:
        rospy.logerr("stationary localization verification had stale pose/odometry")
        return False
    rospy.loginfo("stationary localization stable for %.1f s", duration_s)
    return True


def publish_initialization_receipt(source):
    rospy.set_param("/fast_lio_icp/auto_initialization_source", source)
    rospy.set_param("/fast_lio_icp/auto_initialization_stable", True)
    rospy.set_param("/fast_lio_icp/auto_initialization_verified", True)


def main():
    rospy.init_node("auto_initial_pose")
    rospy.set_param("/fast_lio_icp/auto_initialization_verified", False)
    rospy.set_param("/fast_lio_icp/auto_initialization_stable", False)
    rospy.set_param("/fast_lio_icp/auto_initialization_source", "none")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default=rospy.get_param("~map", ""))
    parser.add_argument("--traj", default=rospy.get_param("~traj", ""))
    parser.add_argument("--route", default=rospy.get_param("~route", ""))
    parser.add_argument(
        "--body-frame-profile",
        default=rospy.get_param("~body_frame_profile", ""),
    )
    parser.add_argument("--spacing", type=float, default=3.0)
    parser.add_argument("--inlier-radius", type=float, default=0.45)
    parser.add_argument("--min-score", type=float, default=0.25)
    parser.add_argument("--top", type=int, default=4)
    parser.add_argument("--refine-top", type=int, default=4)
    parser.add_argument(
        "--min-refined-score",
        type=float,
        default=rospy.get_param("~min_refined_score", 0.80),
    )
    parser.add_argument("--window-s", type=float, default=2.0)
    parser.add_argument("--max-range", type=float, default=25.0)
    parser.add_argument("--verify-timeout", type=float, default=20.0)
    parser.add_argument(
        "--global-only",
        action="store_true",
        default=bool(rospy.get_param("~global_only", False)),
    )
    parser.add_argument(
        "--stability-window-s",
        type=float,
        default=float(rospy.get_param("~stability_window_s", 5.0)),
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])
    required_arguments = ["map", "traj"]
    if not args.global_only:
        required_arguments.extend(("route", "body_frame_profile"))
    for argument in required_arguments:
        if not getattr(args, argument):
            parser.error("--{} or private ROS parameter ~{} is required".format(
                argument.replace("_", "-"), argument
            ))
    route_prior = None
    if not args.global_only:
        try:
            route_prior = load_known_start(
                Path(args.route), "map", args.body_frame_profile
            )
        except KnownStartRouteError as error:
            rospy.logerr("invalid known-start route: %s", error)
            return 5

    collector = SubmapCollector(args.window_s)
    state = {
        "message": "",
        "reset_count": None,
        "sequence": 0,
        "localization_pose": None,
        "odometry_pose": None,
        "pose_sequence": 0,
        "odom_sequence": 0,
    }

    def on_diag(message):
        for status in message.status:
            if status.name == "fast_lio_icp":
                state["message"] = status.message
                values = {item.key: item.value for item in status.values}
                try:
                    state["reset_count"] = int(values["reset_count"])
                except (KeyError, ValueError):
                    state["reset_count"] = None
                state["sequence"] += 1

    def on_pose(message):
        state["localization_pose"] = planar_pose(message.pose.pose)
        state["pose_sequence"] += 1

    def on_stability_odom(message):
        state["odometry_pose"] = planar_pose(message.pose.pose)
        state["odom_sequence"] += 1

    seed_pub = None
    enable = None
    if not args.dry_run:
        rospy.Subscriber(
            "/fast_lio_icp/localization_diagnostics",
            DiagnosticArray,
            on_diag,
            queue_size=5,
        )
        rospy.Subscriber(
            "/fast_lio_icp/pose",
            PoseWithCovarianceStamped,
            on_pose,
            queue_size=20,
        )
        rospy.Subscriber(
            "/Odometry", Odometry, on_stability_odom, queue_size=100
        )
        seed_pub = rospy.Publisher(
            "/fast_lio_icp/initialpose", PoseWithCovarianceStamped, queue_size=1
        )
        enable = rospy.ServiceProxy(
            "/fast_lio_icp/enable_auto_correction", SetBool
        )
        rospy.sleep(0.5)
        if route_prior is not None:
            if try_candidate(
                route_prior, 1, state, seed_pub, enable, args.verify_timeout
            ):
                if wait_for_stationary_stability(
                    state,
                    state["reset_count"],
                    args.stability_window_s,
                ):
                    publish_initialization_receipt(route_prior.source)
                    return 0
                disable_correction(enable, 1)
                return 8
            rospy.logwarn("known start was not verified; starting global fallback")

    deadline = rospy.Time.now() + rospy.Duration(30.0)
    submap = None
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        rospy.sleep(0.5)
        submap = collector.build()
        if submap is not None and len(submap) > 2000 and len(collector.clouds) >= 10:
            break
    if submap is None or len(submap) < 500:
        rospy.logerr("no usable submap from /cloud_registered_body")
        return 2
    ranges = np.linalg.norm(submap[:, :2], axis=1)
    submap = submap[ranges < args.max_range]
    # Filter before downsampling so the whole point budget is spent on geometry
    # that can distinguish one place from another, not on pavement.
    structure, ground_removed = structural_sample(submap)
    sample = voxel_downsample(structure, 0.4, 1800)
    if ground_removed:
        rospy.loginfo(
            "submap sample: %d structural points (of %d, ground removed)",
            len(sample),
            len(submap),
        )
    else:
        rospy.logwarn(
            "submap sample: %d points, too little vertical structure to "
            "separate ground - the fix here is weakly constrained",
            len(sample),
        )

    rospy.loginfo("loading runtime map and mapping trajectory for global fallback")
    map_points = load_pcd_xyz(Path(args.map))
    candidates = load_trajectory_candidates(Path(args.traj), args.spacing)
    rospy.loginfo("%d trajectory candidates", len(candidates))

    scored = score_global_candidates(
        sample,
        map_points,
        candidates,
        args.inlier_radius,
    )

    rospy.loginfo("top coarse candidates:")
    for candidate in scored[:6]:
        rospy.loginfo("  score=%.3f  (%.1f, %.1f, %.1f) yaw=%.0fdeg",
                      candidate.score, candidate.x, candidate.y, candidate.z,
                      np.degrees(candidate.yaw_rad))

    # Coarse hypotheses sit on the recorded trajectory at a fixed spacing and
    # yaw step, so the best of them only brackets the answer. The localizer
    # matches at 0.5 m correspondence and cannot close a bracket that wide, so
    # each shortlisted pose is walked onto the map before it is offered as a
    # seed.
    shortlist = diverse_shortlist(scored, args.refine_top)
    rospy.loginfo(
        "shortlisted %d distinct hypotheses from %d scored",
        len(shortlist),
        len(scored),
    )
    started = rospy.Time.now()
    refined = refine_candidates(
        sample, map_points, shortlist, args.inlier_radius
    )
    rospy.loginfo(
        "refined %d candidates in %.1f s:",
        len(refined),
        (rospy.Time.now() - started).to_sec(),
    )
    for candidate in refined:
        rospy.loginfo("  score=%.3f  (%.2f, %.2f, %.2f) yaw=%.1fdeg",
                      candidate.score, candidate.x, candidate.y, candidate.z,
                      np.degrees(candidate.yaw_rad))

    if args.dry_run:
        return 0

    # Verification downstream proves a candidate is self-consistent, not that
    # it is the right place, so a plausible wrong pose would pass and the
    # follower would drive a route computed from it. Refuse instead.
    decision = decide_fix(refined, args.min_refined_score)
    if decision.reason == "ambiguous":
        rospy.logerr(
            "position is ambiguous: (%.1f, %.1f) yaw=%.0f scores %.3f and "
            "(%.1f, %.1f) yaw=%.0f scores %.3f. This scan does not identify "
            "where the chair is - move it somewhere with more distinct "
            "surroundings, or start from the recorded route start.",
            refined[0].x, refined[0].y, np.degrees(refined[0].yaw_rad),
            refined[0].score,
            decision.rival.x, decision.rival.y,
            np.degrees(decision.rival.yaw_rad), decision.rival.score,
        )
        return 6
    if decision.reason == "weak_support":
        rospy.logerr(
            "best fix (%.1f, %.1f) yaw=%.0f explains only %.3f of what the "
            "chair can see, below %.2f. Too little of the surroundings is in "
            "the map to place the chair here - start from the recorded route "
            "start, or relax ~min_refined_score once it is calibrated on this "
            "route.",
            decision.rival.x, decision.rival.y,
            np.degrees(decision.rival.yaw_rad), decision.rival.score,
            args.min_refined_score,
        )
        return 7
    if decision.candidate is None:
        rospy.logerr("global fallback produced no candidate")
        return 3
    rospy.loginfo(
        "accepted fix (%.2f, %.2f) yaw=%.1f score=%.3f",
        decision.candidate.x, decision.candidate.y,
        np.degrees(decision.candidate.yaw_rad), decision.candidate.score,
    )

    attempts = initialization_attempts(
        None, (decision.candidate,), args.min_score, 1
    )
    if not attempts:
        best = refined[0].score if refined else 0.0
        rospy.logerr(
            "best refined global score %.3f below threshold %.2f - no fallback",
            best,
            args.min_score,
        )
        return 3

    start_rank = 1 if args.global_only else 2
    for rank, candidate in enumerate(attempts, start=start_rank):
        if try_candidate(
            candidate,
            rank,
            state,
            seed_pub,
            enable,
            args.verify_timeout,
        ):
            if wait_for_stationary_stability(
                state,
                state["reset_count"],
                args.stability_window_s,
            ):
                publish_initialization_receipt(candidate.source)
                return 0
            disable_correction(enable, rank)
            return 8
        rospy.logwarn("candidate %d failed verification", rank)
    rospy.logerr("no candidate passed verification")
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
