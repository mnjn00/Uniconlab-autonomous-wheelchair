"""Validate one sensor-only perception/localization shadow snapshot."""

import math

from human_aware_shadow import STATIC_CONFIRM_S

DIAGNOSTIC_KEYS = (
    "tracking_state",
    "raw_scan_points",
    "dynamic_returns_dropped",
    "post_box_points",
    "map_filtered",
    "post_map_points",
    "rolling_submap_points",
)
COMMAND_TOPICS = (
    "/cmd_vel_raw",
    "/cmd_vel_gated",
    "/cmd_vel",
    "/wheel_cmd",
    "/mode_cmd",
)
MOTION_NODE_TOKENS = (
    "wheel",
    "waypoint_follower",
    "dwa_follower",
    "mpc_follower",
    "safety_gate",
    "tip_guard",
)


def _positive_box(box):
    size = box.get("size")
    return (
        isinstance(size, list)
        and len(size) == 3
        and all(
            isinstance(value, (int, float))
            and math.isfinite(value)
            and value > 0.0
            for value in size
        )
    )


def motion_surface_violations(publishers, subscribers):
    """Return connected motion topics/nodes that make shadow QA unsafe."""
    violations = []
    for role, graph in (
        ("publisher", publishers),
        ("subscriber", subscribers),
    ):
        for topic, nodes in graph.items():
            if topic in COMMAND_TOPICS and nodes:
                violations.append(
                    "%s %s: %s" % (role, topic, ", ".join(nodes))
                )
            for node in nodes:
                lowered = node.lower()
                if any(token in lowered for token in MOTION_NODE_TOKENS):
                    violations.append("%s node: %s" % (role, node))
    return sorted(set(violations))


def baseline_graph_violations(graph, baseline):
    """Require cleanup to restore every ROS publisher/subscriber/service edge."""
    violations = []
    labels = {
        "publishers": "publisher",
        "subscribers": "subscriber",
        "services": "service",
    }
    for section, label in labels.items():
        observed = {
            (name, node)
            for name, nodes in graph.get(section, {}).items()
            for node in nodes
        }
        expected = {
            (name, node)
            for name, nodes in baseline.get(section, {}).items()
            for node in nodes
        }
        violations.extend(
            "unexpected %s edge: %s <- %s" % (label, name, node)
            for name, node in sorted(observed - expected)
        )
        violations.extend(
            "missing %s edge: %s <- %s" % (label, name, node)
            for name, node in sorted(expected - observed)
        )
    return violations


def validate_snapshot(summary, dynamic_boxes, diagnostics,
                      diagnostics_stamp=None, boxes_source_stamp=None,
                      max_skew_s=2.0):
    """Return normalized counts, or raise on an unsafe/incomplete snapshot."""
    if summary.get("status") != "OK":
        raise ValueError("objects_summary is not OK")
    objects = summary.get("objects")
    if not isinstance(objects, list):
        raise ValueError("objects_summary has no object list")
    if not isinstance(dynamic_boxes, list):
        raise ValueError("dynamic boxes are not a list")
    if len(objects) != len(dynamic_boxes):
        raise ValueError(
            "object/box count mismatch: %d != %d"
            % (len(objects), len(dynamic_boxes))
        )
    if not all(_positive_box(box) for box in dynamic_boxes):
        raise ValueError("dynamic box sizes must be positive finite triples")
    summary_stamp = float(summary.get("stamp"))
    if not math.isfinite(summary_stamp):
        raise ValueError("objects_summary stamp is not finite")
    if boxes_source_stamp is not None:
        boxes_source_stamp = float(boxes_source_stamp)
        if not math.isfinite(boxes_source_stamp) or abs(
                boxes_source_stamp - summary_stamp) > 0.05:
            raise ValueError("summary and boxes are not from one source cycle")
    summary_ids = sorted(int(item["id"]) for item in objects)
    box_ids = sorted(int(box["id"]) for box in dynamic_boxes)
    if summary_ids != box_ids:
        raise ValueError("object/box IDs do not match")
    for box in dynamic_boxes:
        position = box.get("position")
        if not (
            isinstance(position, list)
            and len(position) == 3
            and all(
                isinstance(value, (int, float))
                and math.isfinite(value)
                for value in position
            )
        ):
            raise ValueError("dynamic box positions must be finite triples")
        stamp = float(box.get("stamp"))
        if not math.isfinite(stamp) or abs(stamp - summary_stamp) > max_skew_s:
            raise ValueError("summary/box snapshot stamps do not agree")
    missing = [key for key in DIAGNOSTIC_KEYS if key not in diagnostics]
    if missing:
        raise ValueError("missing diagnostics: %s" % ", ".join(missing))
    if diagnostics["tracking_state"] != "TRACKING":
        raise ValueError(
            "localization is not TRACKING: %s"
            % diagnostics["tracking_state"]
        )
    if diagnostics_stamp is not None:
        diagnostics_stamp = float(diagnostics_stamp)
        if not math.isfinite(diagnostics_stamp) or \
                abs(diagnostics_stamp - summary_stamp) > max_skew_s:
            raise ValueError(
                "perception/localization snapshot stamps do not agree"
            )
    for key in DIAGNOSTIC_KEYS[1:]:
        value = float(diagnostics[key])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("%s must be finite and non-negative" % key)
    raw = int(diagnostics["raw_scan_points"])
    box_dropped = int(diagnostics["dynamic_returns_dropped"])
    post_box = int(diagnostics["post_box_points"])
    map_filtered = int(diagnostics["map_filtered"])
    post_map = int(diagnostics["post_map_points"])
    rolling = int(diagnostics["rolling_submap_points"])
    if post_box > raw or post_map > post_box:
        raise ValueError("registration filter stage counts are not monotonic")
    if raw - box_dropped != post_box:
        raise ValueError("box-filter accounting is inconsistent")
    if post_box - map_filtered != post_map:
        raise ValueError("map-filter accounting is inconsistent")
    if raw <= 0 or rolling <= 0:
        raise ValueError("localization snapshot is empty")
    return {
        "object_count": len(objects),
        "dynamic_box_count": len(dynamic_boxes),
        "tracking_state": diagnostics["tracking_state"],
        "raw_scan_points": raw,
        "post_box_points": post_box,
        "post_map_points": post_map,
    }


def _nearest_status(statuses, stamp_s, max_skew_s=0.25):
    selected = min(
        statuses,
        key=lambda item: abs(float(item["stamp"]) - stamp_s),
    )
    if abs(float(selected["stamp"]) - stamp_s) > max_skew_s:
        raise ValueError("shadow evidence has no coherent status")
    return selected


def validate_human_aware_replay(replay):
    """Validate one isolated CoHAN/HATEB shadow replay summary."""
    statuses = replay.get("statuses")
    summaries = replay.get("summaries")
    tracked_agents = replay.get("tracked_agents")
    proposals = replay.get("velocity_proposals")
    local_plans = replay.get("local_plans")
    agent_plans = replay.get("agent_plans", [])
    if not all(
            isinstance(value, list) and value
            for value in (
                statuses,
                summaries,
                tracked_agents,
                proposals,
                local_plans,
            )):
        raise ValueError("human-aware replay evidence is incomplete")
    if not isinstance(agent_plans, list):
        raise ValueError("HATEB agent-plan evidence is malformed")
    statuses = sorted(statuses, key=lambda item: float(item["stamp"]))
    decisions = [item.get("decision") for item in statuses]
    allowed = {"OBSERVING", "BYPASS_COMMITTED", "STOP_REQUIRED"}
    if any(decision not in allowed for decision in decisions):
        raise ValueError("human-aware replay has an unknown decision")

    commit_entries = [
        item for index, item in enumerate(statuses)
        if item["decision"] == "BYPASS_COMMITTED"
        and (
            index == 0
            or statuses[index - 1]["decision"] != "BYPASS_COMMITTED"
        )
    ]
    stop_go_reentries = sum(
        float(item.get("evidence_s", 0.0)) < STATIC_CONFIRM_S - 1e-9
        for item in commit_entries[1:]
    )
    if stop_go_reentries:
        raise ValueError("shadow bypass re-entered after a stop")
    committed = [
        item for item in statuses
        if item["decision"] == "BYPASS_COMMITTED"
    ]
    if len(committed) < 2:
        raise ValueError("shadow replay has no sustained commitment")
    committed_ids = [
        int(item["track_id"])
        for item in committed
        if item.get("track_id") is not None
    ]
    if not committed_ids:
        raise ValueError("shadow commitment has no stable identity")
    stable_track_id = max(
        set(committed_ids),
        key=committed_ids.count,
    )
    stable_committed = [
        item for item in committed
        if int(item["track_id"]) == stable_track_id
    ]

    for cycle in tracked_agents:
        if cycle.get("frame_id") != "map":
            raise ValueError("tracked agents are not in map frame")
        status = _nearest_status(statuses, float(cycle["stamp"]))
        if (
                status["decision"] != "BYPASS_COMMITTED"
                or cycle.get("track_ids") != [int(status["track_id"])]):
            raise ValueError("tracked-agent identity is not coherent")

    committed_proposals = []
    for proposal in proposals:
        stamp_s = float(proposal["stamp"])
        status = _nearest_status(statuses, stamp_s)
        linear_x = float(proposal["linear_x"])
        angular_z = float(proposal["angular_z"])
        if not all(math.isfinite(value) for value in (linear_x, angular_z)):
            raise ValueError("velocity proposal is not finite")
        if abs(linear_x) > 0.35 + 1e-9:
            raise ValueError("velocity proposal exceeds shadow speed cap")
        if status["decision"] == "BYPASS_COMMITTED":
            committed_proposals.append(proposal)
    if not committed_proposals:
        raise ValueError("shadow replay has no committed velocity proposal")

    accepted_plans = [
        plan for plan in local_plans
        if plan.get("validation") == "ACCEPTED"
        and int(plan.get("point_count", 0)) >= 2
        and _nearest_status(
            statuses, float(plan["stamp"])
        )["decision"] == "BYPASS_COMMITTED"
    ]
    if not accepted_plans:
        raise ValueError("shadow replay has no accepted HATEB local plan")

    coherent_agent_plans = []
    stable_agent_plan_count = 0
    for cycle in agent_plans:
        status = _nearest_status(statuses, float(cycle["stamp"]))
        if status["decision"] != "BYPASS_COMMITTED":
            continue
        paths = cycle.get("paths")
        if not isinstance(paths, list):
            raise ValueError("HATEB agent plan has no path list")
        usable = [
            path for path in paths
            if int(path.get("point_count", 0)) >= 2
            and int(path.get("track_id", -1)) == int(status["track_id"])
        ]
        if not usable:
            continue
        coherent_agent_plans.append(cycle)
        stable_agent_plan_count += any(
            int(path["track_id"]) == stable_track_id
            for path in usable
        )
    if (
            agent_plans
            and (not coherent_agent_plans or stable_agent_plan_count < 1)):
        raise ValueError("HATEB has no coherent committed agent trajectory")

    unsafe_motion_stop_count = 0
    for summary in summaries:
        people = summary.get("people")
        if not isinstance(people, list):
            raise ValueError("shadow summary has no people list")
        if not any(person.get("motion") == "moving" for person in people):
            continue
        status = _nearest_status(statuses, float(summary["stamp"]))
        unsafe_motion_stop_count += status["decision"] == "STOP_REQUIRED"
    if unsafe_motion_stop_count < 1:
        raise ValueError("moving-person replay never became stop-required")

    return {
        "accepted_plan_count": len(accepted_plans),
        "agent_plan_count": len(coherent_agent_plans),
        "committed_sample_count": len(stable_committed),
        "proposal_count": len(committed_proposals),
        "stable_track_id": stable_track_id,
        "stop_go_reentries": stop_go_reentries,
        "unsafe_motion_stop_count": unsafe_motion_stop_count,
    }
