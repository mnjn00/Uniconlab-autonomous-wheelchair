#!/usr/bin/env python3
"""Pure deterministic evidence for immutable route-safety behavior."""

import ast
import copy
import hashlib
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import types

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src" / "wheelchair_route_safety"
SCRIPT = PACKAGE / "scripts" / "route_safety.py"
CONFIG = PACKAGE / "config" / "route_safety_sim.yaml"
SCHEMA = ROOT / "contracts" / "wp0" / "A06-route-safety-schema.json"
CANDIDATE = ROOT / "contracts" / "wp0" / "route-safety-candidate.yaml"
ROUTE = ROOT / "data" / "hanyang_aegimun_loop" / "hanyang_aegimun_loop.waypoints.yaml"
MAP = ROOT / "data" / "hanyang_aegimun_loop" / "map.pgm"
METADATA = ROOT / "data" / "hanyang_aegimun_loop" / "map.metadata.json"
NAVIGATION_ROUTES = ROOT / "src" / "wheelchair_navigation" / "config" / "hanyang_routes.yaml"
CONFIG_SHA256 = "471bc90f8d52e341d2d6d287992fd26bf4224b776c57a057c82796bb0506eb60"
A03_FUTURE_TOLERANCE_S = 0.05
SPEC = importlib.util.spec_from_file_location("route_safety", SCRIPT)
route_safety = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = route_safety
SPEC.loader.exec_module(route_safety)


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _config():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def _manifest(normal=True):
    value = yaml.safe_load(CANDIDATE.read_text(encoding="utf-8"))
    square = [[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]]
    value["global_allowed_polygon"] = copy.deepcopy(square)
    for route in value["approved_routes"]:
        route["corridor_polygon"] = copy.deepcopy(square)
    value["localization_zones"][0]["polygon"] = copy.deepcopy(square)
    value["localization_zones"][0]["policy"] = "normal" if normal else "manual_only"
    return value


def _load(value):
    temporary = tempfile.NamedTemporaryFile(mode="wb", suffix=".yaml", delete=False)
    raw = yaml.safe_dump(value, sort_keys=False).encode("utf-8")
    temporary.write(raw)
    temporary.close()
    digest = hashlib.sha256(raw).hexdigest()
    hashes = {route["route_id"]: route["route_manifest_sha256"] for route in value["approved_routes"]}
    config = _config()
    return route_safety.load_policy(
        temporary.name, digest, _sha(MAP), hashes,
        config["measured_footprint_length_m"], config["measured_footprint_width_m"],
        config["expected_geometry_sha256"], schema_path=SCHEMA,
    )


def _selection(policy):
    route = policy.routes[0]
    return route_safety.ActiveRouteSelection(
        route.route_id, route.route_manifest_sha256, policy.manifest_sha256,
        policy.map_id, policy.map_sha256, route.segment_ids[0], route.zone_ids[0],
    )


def _pose(x=0.0, y=0.0, yaw=0.0, stamp=9.9, sigma=0.01, state="OK",
          pose_stamp=None, status_stamp=None, transform_stamp=None):
    return route_safety.PoseSample(
        x, y, yaw,
        stamp if pose_stamp is None else pose_stamp,
        stamp if status_stamp is None else status_stamp,
        stamp if transform_stamp is None else transform_stamp,
        sigma, state,
    )


def _point_segment_distance(point, first, second):
    dx, dy = second[0] - first[0], second[1] - first[1]
    fraction = max(0.0, min(1.0, ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / (dx * dx + dy * dy)))
    projection = (first[0] + fraction * dx, first[1] + fraction * dy)
    return math.hypot(point[0] - projection[0], point[1] - projection[1])


def test_sim_config_dynamically_binds_exact_candidate_map_and_route_bytes():
    config = _config()
    candidate = yaml.safe_load(CANDIDATE.read_text(encoding="utf-8"))
    assert config["expected_manifest_sha256"] == _sha(CANDIDATE)
    assert config["expected_map_sha256"] == _sha(MAP)
    assert config["simulation_geometry"]["route_asset_sha256"] == _sha(ROUTE)
    assert set(config["expected_route_hashes"]) == {route["route_id"] for route in candidate["approved_routes"]}
    candidate_hashes = {
        route["route_id"]: route["route_manifest_sha256"]
        for route in candidate["approved_routes"]
    }
    assert config["expected_route_hashes"] == candidate_hashes
    assert len(set(candidate_hashes.values())) == 2
    assert config["active_route_ttl_s"] == route_safety.ACTIVE_ROUTE_TTL_S == 0.75
    assert config["localization_policy_sha256"] == (
        "5d84ea824c98a53639a480ed162a62f015600ca0a0460df7186d5839303d52e8"
    )

def test_simulation_config_bytes_require_exact_sha256_before_policy_creation():
    assert _sha(CONFIG) == CONFIG_SHA256
    policy = route_safety.load_simulation_policy(CONFIG, CONFIG_SHA256)
    assert policy.simulation_only

    with pytest.raises(TypeError):
        route_safety.load_simulation_policy(CONFIG)
    for invalid in ("", "0" * 63, "G" * 64, CONFIG_SHA256.upper()):
        with pytest.raises(route_safety.ManifestError, match="64 lowercase hex"):
            route_safety.load_simulation_policy(CONFIG, invalid)

    raw = b":\n  - ["
    with tempfile.NamedTemporaryFile(mode="wb", suffix=".yaml") as temporary:
        temporary.write(raw)
        temporary.flush()
        with pytest.raises(route_safety.ManifestError, match="simulation config SHA-256 mismatch"):
            route_safety.load_simulation_policy(temporary.name, CONFIG_SHA256)


def test_ros_config_mismatch_fails_before_publishers_are_created(monkeypatch):
    publisher_calls = []
    rospy = types.ModuleType("rospy")
    rospy.init_node = lambda _name: None
    rospy.get_param = lambda name: (
        str(CONFIG) if name == "~config_path" else "0" * 64
    )
    rospy.Publisher = lambda *args, **kwargs: publisher_calls.append((args, kwargs))

    interfaces = types.ModuleType("wheelchair_interfaces")
    interface_messages = types.ModuleType("wheelchair_interfaces.msg")
    for name in (
            "ActiveRoute", "GeofenceStatus", "LocalizationCandidate",
            "LocalizationStatus", "RouteProgress", "SafetySignal"):
        setattr(interface_messages, name, type(name, (), {}))
    interfaces.msg = interface_messages
    std_msgs = types.ModuleType("std_msgs")
    std_messages = types.ModuleType("std_msgs.msg")
    std_messages.Header = type("Header", (), {})
    std_msgs.msg = std_messages
    monkeypatch.setitem(sys.modules, "rospy", rospy)
    monkeypatch.setitem(sys.modules, "wheelchair_interfaces", interfaces)
    monkeypatch.setitem(sys.modules, "wheelchair_interfaces.msg", interface_messages)
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_messages)

    with pytest.raises(route_safety.ManifestError, match="simulation config SHA-256 mismatch"):
        route_safety.run_ros_node()
    assert publisher_calls == []
def _localization_evidence(stamp, reset_count=1, map_id="map", map_sha256="a" * 64,
                           frame_id="map", policy_sha256="p" * 64, state="OK", sequence=1):
    header = types.SimpleNamespace(
        stamp=types.SimpleNamespace(to_sec=lambda: stamp),
        frame_id=frame_id,
    )
    pose = types.SimpleNamespace(header=header)
    candidate = types.SimpleNamespace(
        pose=pose,
        reset_count=reset_count,
        map_id=map_id,
        map_sha256=map_sha256,
    )
    status = types.SimpleNamespace(
        header=header,
        evaluation_stamp=types.SimpleNamespace(to_sec=lambda: stamp),
        transform_age_s=0.0,
        sequence=sequence,
        reset_count=reset_count,
        map_id=map_id,
        map_sha256=map_sha256,
        policy_sha256=policy_sha256,
        state=state,
        reason_mask=0,
        independent_check_passed=True,
    )
    return candidate, status


def test_localization_candidate_then_status_retains_prior_pair_until_new_pair_joins():
    evidence = route_safety.LocalizationEvidenceBuffer()
    candidate_a, status_a = _localization_evidence(10.0, sequence=1)
    candidate_b, status_b = _localization_evidence(10.1, sequence=2)
    first_evidence = (status_a, 10.01, 1)
    in_flight_evidence = (status_b, 10.11, 2)

    evidence.add_candidate(candidate_a)
    evidence.add_status(first_evidence)
    assert evidence.snapshot() == (first_evidence, candidate_a)

    evidence.add_candidate(candidate_b)
    assert evidence.snapshot() == (first_evidence, candidate_a)

    evidence.add_status(in_flight_evidence)
    assert evidence.snapshot() == (in_flight_evidence, candidate_b)


def test_localization_status_then_candidate_retains_prior_pair_until_new_pair_joins():
    evidence = route_safety.LocalizationEvidenceBuffer()
    candidate_a, status_a = _localization_evidence(10.0, sequence=1)
    candidate_b, status_b = _localization_evidence(10.1, sequence=2)
    first_evidence = (status_a, 10.01, 1)
    in_flight_evidence = (status_b, 10.11, 2)

    evidence.add_candidate(candidate_a)
    evidence.add_status(first_evidence)
    assert evidence.snapshot() == (first_evidence, candidate_a)

    evidence.add_status(in_flight_evidence)
    assert evidence.snapshot() == (in_flight_evidence, None)
    assert route_safety.status_allows_pair_hold(
        status_b, 10.11, candidate_a.reset_count, "map", "a" * 64, "map", "p" * 64, "OK",
    )

    evidence.add_candidate(candidate_b)
    assert evidence.snapshot() == (in_flight_evidence, candidate_b)


def test_status_chronology_accepts_newer_status_for_unchanged_candidate_identity():
    prior = (1, 10.0, 10.01, 10.02)
    assert route_safety.status_chronology_is_newer((2, 10.0, 10.03, 10.04), prior)
    assert not route_safety.status_chronology_is_newer((2, 9.9, 10.03, 10.04), prior)
    assert not route_safety.status_chronology_is_newer((2, 10.0, 10.0, 10.04), prior)
    assert not route_safety.status_chronology_is_newer((1, 10.0, 10.03, 10.04), prior)

def test_newer_restrictive_status_revokes_prior_permissive_pair():
    evidence = route_safety.LocalizationEvidenceBuffer()
    candidate_a, status_a = _localization_evidence(10.0, sequence=1)
    candidate_b, restrictive_status = _localization_evidence(10.1, state="NOT_OK", sequence=2)
    first_evidence = (status_a, 10.01, 1)
    restrictive_evidence = (restrictive_status, 10.11, 2)

    evidence.add_candidate(candidate_a)
    evidence.add_status(first_evidence)
    assert evidence.snapshot() == (first_evidence, candidate_a)

    evidence.add_candidate(candidate_b)
    evidence.add_status(restrictive_evidence)
    assert evidence.snapshot() == (restrictive_evidence, candidate_b)
    assert not route_safety.status_allows_pair_hold(
        restrictive_status, 10.11, candidate_a.reset_count, "map", "a" * 64, "map", "p" * 64, "OK",
    )

def test_localization_status_without_exact_candidate_match_fails_closed_then_promotes():
    evidence = route_safety.LocalizationEvidenceBuffer()
    candidate, status = _localization_evidence(10.0)
    status_evidence = (status, 10.01, 1)
    evidence.add_status(status_evidence)
    assert evidence.snapshot() == (status_evidence, None)

    wrong_stamp, _ = _localization_evidence(10.1)
    wrong_reset, _ = _localization_evidence(10.0, reset_count=2)
    wrong_map, _ = _localization_evidence(10.0, map_sha256="b" * 64)
    for invalid_candidate in (wrong_stamp, wrong_reset, wrong_map):
        evidence.add_candidate(invalid_candidate)
    assert evidence.snapshot() == (status_evidence, None)

    evidence.add_candidate(candidate)
    assert evidence.snapshot() == (status_evidence, candidate)


def test_localization_evidence_caches_are_bounded_and_deterministic():
    evidence = route_safety.LocalizationEvidenceBuffer(limit=2)
    candidates = []
    for stamp in (1.0, 2.0, 3.0):
        candidate, status = _localization_evidence(stamp)
        candidates.append(candidate)
        evidence.add_candidate(candidate)
        evidence.add_status((status, stamp, int(stamp)))

    assert evidence.candidates == candidates[-2:]
    assert len(evidence.statuses) == 2
    assert evidence.snapshot()[0][0].header.stamp.to_sec() == 3.0


def test_localization_snapshot_recovers_only_with_newer_matching_evidence():
    evidence = route_safety.LocalizationEvidenceBuffer()
    candidate_a, status_a = _localization_evidence(10.0)
    evidence.add_candidate(candidate_a)
    evidence.add_status((status_a, 10.01, 1))
    assert evidence.snapshot()[1] is candidate_a

    malformed_evidence = (object(), 10.02, 2)
    evidence.add_status(malformed_evidence)
    assert evidence.snapshot() == (malformed_evidence, None)

    candidate_b, status_b = _localization_evidence(10.1)
    recovered_evidence = (status_b, 10.11, 3)
    evidence.add_candidate(candidate_b)
    evidence.add_status(recovered_evidence)
    assert evidence.snapshot() == (recovered_evidence, candidate_b)
def test_permissive_status_hold_rejects_restrictive_identity_and_timing_changes():
    _, status = _localization_evidence(10.0)
    allowed = lambda value: route_safety.status_allows_pair_hold(
        value, 10.01, 1, "map", "a" * 64, "map", "p" * 64, "OK",
    )
    assert allowed(status)
    for field, value in (
            ("reset_count", 2), ("map_id", "other"), ("map_sha256", "b" * 64),
            ("policy_sha256", "q" * 64), ("state", "NOT_OK"),
            ("reason_mask", 1), ("independent_check_passed", False),
            ("transform_age_s", -0.01)):
        changed = types.SimpleNamespace(**vars(status))
        setattr(changed, field, value)
        assert not allowed(changed)

    wrong_frame = types.SimpleNamespace(**vars(status))
    wrong_frame.header = types.SimpleNamespace(
        stamp=status.header.stamp, frame_id="odom",
    )
    assert not allowed(wrong_frame)


def test_status_hold_rejects_malformed_and_stale_prior_evidence():
    _, status = _localization_evidence(10.0)
    assert not route_safety.status_allows_pair_hold(
        object(), 10.01, 1, "map", "a" * 64, "map", "p" * 64, "OK",
    )
    assert not route_safety.status_allows_pair_hold(
        status, 9.0, 1, "map", "a" * 64, "map", "p" * 64, "OK",
    )



def test_ros_pairing_retains_existing_stamp_reset_map_and_policy_checks():
    source = SCRIPT.read_text(encoding="utf-8")
    for condition in (
            "abs(pose_stamp - status_source_stamp) <= 1.0e-9",
            "pose_msg.reset_count == localization_msg.reset_count",
            "pose_msg.map_id == localization_msg.map_id == policy.map_id",
            "pose_msg.map_sha256 == localization_msg.map_sha256 == policy.map_sha256",
            "localization_msg.policy_sha256 == policy.localization_policy_sha256",
            "localization_msg.state == LocalizationStatus.OK",
            "localization_msg.reason_mask == 0",
            "localization_msg.independent_check_passed",
    ):
        assert condition in source


def test_ros_publish_snapshots_evidence_before_sampling_evaluation_clock():
    module = ast.parse(SCRIPT.read_text(encoding="utf-8"))
    run_ros_node = next(
        node for node in module.body
        if isinstance(node, ast.FunctionDef) and node.name == "run_ros_node"
    )
    publish = next(
        node for node in run_ros_node.body
        if isinstance(node, ast.FunctionDef) and node.name == "publish"
    )
    assignments = {
        statement.targets[0].id: index
        for index, statement in enumerate(publish.body)
        if (isinstance(statement, ast.Assign) and len(statement.targets) == 1
            and isinstance(statement.targets[0], ast.Name))
    }
    snapshot_index = next(
        index for index, statement in enumerate(publish.body)
        if (isinstance(statement, ast.Assign) and len(statement.targets) == 1
            and isinstance(statement.targets[0], ast.Tuple)
            and [element.id for element in statement.targets[0].elts] == [
                "status_evidence", "pose_msg"
            ] and isinstance(statement.value, ast.Call)
            and isinstance(statement.value.func, ast.Attribute)
            and statement.value.func.attr == "snapshot"
            and isinstance(statement.value.func.value, ast.Name)
            and statement.value.func.value.id == "evidence")
    )

    assert (
        assignments["route_msg"] < snapshot_index < assignments["now"]
        < assignments["now_s"]
    )


def test_snapshot_evidence_is_not_evaluated_with_a_pre_snapshot_clock():
    policy = _load(_manifest())
    candidate, status = _localization_evidence(10.10)
    evidence = route_safety.LocalizationEvidenceBuffer()
    evidence.add_candidate(candidate)
    evidence.add_status((status, 10.10, 1))

    status_evidence, snapshot_candidate = evidence.snapshot()
    snapshot_status, receipt_stamp, _ = status_evidence
    evaluation_stamp = snapshot_status.evaluation_stamp.to_sec()
    snapshot_sample = _pose(
        pose_stamp=evaluation_stamp,
        status_stamp=evaluation_stamp,
        transform_stamp=evaluation_stamp,
    )

    assert snapshot_candidate is candidate
    assert evaluation_stamp == receipt_stamp == 10.10
    pre_snapshot = route_safety.evaluate(
        policy, snapshot_sample, _selection(policy), 10.00,
    )
    assert pre_snapshot.signal_state == route_safety.STOP
    assert pre_snapshot.reason_mask == (
        route_safety.REASON_SENSOR_STALE | route_safety.REASON_GEOFENCE
    )
    assert route_safety.evaluate(
        policy, snapshot_sample, _selection(policy), receipt_stamp,
    ).clear




@pytest.mark.parametrize("binding", ["manifest", "map", "route"])
def test_any_runtime_hash_mismatch_stops_policy_loading(binding):
    config = _config()
    hashes = dict(config["expected_route_hashes"])
    manifest_hash = config["expected_manifest_sha256"]
    map_hash = config["expected_map_sha256"]
    if binding == "manifest":
        manifest_hash = "0" * 64
    elif binding == "map":
        map_hash = "0" * 64
    else:
        hashes[next(iter(hashes))] = "0" * 64
    with pytest.raises(route_safety.ManifestError):
        route_safety.load_policy(
            CANDIDATE, manifest_hash, map_hash, hashes,
            config["measured_footprint_length_m"], config["measured_footprint_width_m"],
            config["expected_geometry_sha256"], schema_path=SCHEMA,
        )


def test_simulation_corridor_covers_both_recorded_directions_without_widening():
    config = _config()
    source = yaml.safe_load(ROUTE.read_text(encoding="utf-8"))
    geometry = config["simulation_geometry"]
    zone = config["simulation_zones"][0]
    assert geometry["immutable"] is True
    assert geometry["runtime_mutation_allowed"] is False
    assert geometry["widening_allowed"] is False
    assert geometry["recorded_corridor_margin_m"] == 0.2
    assert geometry["outside_corridor_action"] == "STOP"
    assert zone["directions"] == ["outbound", "return"]
    assert zone["policy"] == "normal"
    assert geometry["route_bindings"]["outbound"] == {
        "route_id": "hanyang_aegimun_engineering_outbound",
        "asset_key": "outbound_route",
    }
    assert geometry["route_bindings"]["return"] == {
        "route_id": "hanyang_engineering_aegimun_return",
        "asset_key": "return_route",
    }
    assert geometry["navigation_route_manifest_sha256"] == _sha(NAVIGATION_ROUTES)
    for direction in zone["directions"]:
        points = [(w["x_m"], w["y_m"]) for w in source[direction + "_route"]["waypoints"]]
        assert len(points) in (359, 373)
        for point in points:
            assert min(_point_segment_distance(point, a, b) for a, b in zip(points, points[1:])) <= geometry["recorded_corridor_margin_m"]
        a, b = points[len(points) // 2:len(points) // 2 + 2]
        midpoint = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        normal_length = math.hypot(b[0] - a[0], b[1] - a[1])
        outside = (midpoint[0] - (b[1] - a[1]) / normal_length, midpoint[1] + (b[0] - a[0]) / normal_length)
        assert min(_point_segment_distance(outside, p, q) for p, q in zip(points, points[1:])) > geometry["recorded_corridor_margin_m"]


def test_footprint_uncertainty_and_authority_are_simulation_only():
    config = _config()
    candidate = yaml.safe_load(CANDIDATE.read_text(encoding="utf-8"))
    geometry = config["simulation_geometry"]
    assert config["measured_footprint_length_m"] == 0.97
    assert config["measured_footprint_width_m"] == 0.60
    assert geometry["footprint_clearance_margin_m"] == 0.20
    assert geometry["localization_uncertainty_margin_m"] == candidate["footprint"]["localization_uncertainty_margin_m"] == 0.25
    assert config["simulation_only"] is True
    assert config["hardware_motion_authorized"] is False
    assert config["passenger_operation_authorized"] is False
    assert candidate["authority"]["hardware_authorized"] is False
    assert candidate["authority"]["passenger_authorized"] is False
    for zone in config["simulation_zones"]:
        assert zone["simulation_only"] is True
        assert zone["hardware_authorized"] is False
        assert zone["passenger_authorized"] is False
        assert 0.0 < zone["max_linear_mps"] <= 0.35
        assert 0.0 < zone["max_angular_rps"] <= 0.6


def _simulation_selection(policy, direction, segment_index=0):
    route = next(candidate for candidate in policy.routes if candidate.direction == direction)
    return route_safety.ActiveRouteSelection(
        route.route_id, route.route_manifest_sha256, policy.manifest_sha256,
        policy.map_id, policy.map_sha256, route.segment_ids[segment_index], route.zone_ids[0],
    )


def test_runtime_consumes_centerline_tube_for_route_turn_endpoint_and_off_route():
    policy = route_safety.load_simulation_policy(CONFIG, CONFIG_SHA256)
    source = yaml.safe_load(ROUTE.read_text(encoding="utf-8"))
    assert policy.simulation_only
    assert policy.zone("zone-simulation-candidate").policy == "normal"
    for direction in ("outbound", "return"):
        route = next(candidate for candidate in policy.routes if candidate.direction == direction)
        waypoints = source[direction + "_route"]["waypoints"]
        indexes = (0, len(route.segment_ids) // 2, len(route.segment_ids) - 1)
        for segment_index in indexes:
            selection = _simulation_selection(policy, direction, segment_index)
            waypoint_index = segment_index + (1 if segment_index == len(route.segment_ids) - 1 else 0)
            waypoint = waypoints[waypoint_index]
            pose = _pose(waypoint["x_m"], waypoint["y_m"], waypoint["yaw_rad"])
            assert route_safety.evaluate(policy, pose, selection, 10.0).clear

        segment_index = len(route.segment_ids) // 3
        selection = _simulation_selection(policy, direction, segment_index)
        first, second = waypoints[segment_index:segment_index + 2]
        length = math.hypot(second["x_m"] - first["x_m"], second["y_m"] - first["y_m"])
        off_route = _pose(
            first["x_m"] - 0.5 * (second["y_m"] - first["y_m"]) / length,
            first["y_m"] + 0.5 * (second["x_m"] - first["x_m"]) / length,
            first["yaw_rad"],
        )
        assert route_safety.evaluate(policy, off_route, selection, 10.0).signal_state == route_safety.STOP


def test_simulation_direction_segment_and_hash_ambiguity_stop():
    policy = route_safety.load_simulation_policy(CONFIG, CONFIG_SHA256)
    source = yaml.safe_load(ROUTE.read_text(encoding="utf-8"))
    outbound = _simulation_selection(policy, "outbound", 20)
    waypoint = source["outbound_route"]["waypoints"][20]
    pose = _pose(waypoint["x_m"], waypoint["y_m"], waypoint["yaw_rad"])
    reversed_pose = _pose(
        waypoint["x_m"], waypoint["y_m"], waypoint["yaw_rad"] + math.pi,
    )
    wrong_direction = _simulation_selection(policy, "return", 20)
    wrong_segment = copy.copy(outbound)
    object.__setattr__(wrong_segment, "segment_id", "unknown-segment")
    cross_route_segment = copy.copy(outbound)
    return_route = next(route for route in policy.routes if route.direction == "return")
    object.__setattr__(cross_route_segment, "segment_id", return_route.segment_ids[20])
    wrong_hash = copy.copy(outbound)
    object.__setattr__(wrong_hash, "route_manifest_sha256", "0" * 64)
    assert route_safety.evaluate(policy, pose, wrong_direction, 10.0).signal_state == route_safety.STOP
    assert route_safety.evaluate(policy, reversed_pose, outbound, 10.0).signal_state == route_safety.STOP
    # Segment IDs supplied by navigation are correlation-only and cannot select or widen safety.
    assert route_safety.evaluate(policy, pose, wrong_segment, 10.0).clear
    assert route_safety.evaluate(policy, pose, cross_route_segment, 10.0).clear
    assert route_safety.evaluate(policy, pose, wrong_hash, 10.0).reason_mask & route_safety.REASON_ROUTE_MANIFEST


@pytest.mark.parametrize("mutation", ["nonfinite", "widen", "route_hash", "authority", "active_route_ttl", "localization_policy"])
def test_simulation_geometry_mutation_is_rejected_before_policy_creation(mutation):
    config = _config()
    if mutation == "nonfinite":
        config["simulation_geometry"]["recorded_corridor_margin_m"] = float("nan")
    elif mutation == "widen":
        config["simulation_geometry"]["widening_allowed"] = True
    elif mutation == "route_hash":
        config["simulation_geometry"]["route_asset_sha256"] = "0" * 64
    elif mutation == "authority":
        config["simulation_zones"][0]["hardware_authorized"] = True
    elif mutation == "active_route_ttl":
        config["active_route_ttl_s"] = 0.25
    else:
        config["localization_policy_sha256"] = "0" * 64
    with pytest.raises(route_safety.ManifestError):
        route_safety._load_simulation_policy(config, CONFIG.resolve())


def test_tampered_navigation_manifest_and_pin_are_rejected():
    config = _config()
    config["simulation_geometry"]["navigation_route_manifest_sha256"] = "0" * 64
    with pytest.raises(route_safety.ManifestError):
        route_safety._load_simulation_policy(config, CONFIG.resolve())

    navigation = yaml.safe_load(NAVIGATION_ROUTES.read_text(encoding="utf-8"))
    navigation["outbound_route"]["segments"][0]["hardware_authorized"] = True
    raw = yaml.safe_dump(navigation, sort_keys=False).encode("utf-8")
    with tempfile.NamedTemporaryFile(mode="wb", suffix=".yaml") as temporary:
        temporary.write(raw)
        temporary.flush()
        config = _config()
        config["simulation_geometry"]["navigation_route_manifest_path"] = temporary.name
        config["simulation_geometry"]["navigation_route_manifest_sha256"] = hashlib.sha256(raw).hexdigest()
        with pytest.raises(route_safety.ManifestError):
            route_safety._load_simulation_policy(config, CONFIG.resolve())


def test_inside_clear_boundary_uncertainty_stale_and_manual_only_stop():
    policy = _load(_manifest())
    selection = _selection(policy)
    inside = route_safety.evaluate(policy, _pose(), selection, 10.0, 1)
    assert inside.clear
    assert inside.required_boundary_margin_m == pytest.approx(0.65)
    assert route_safety.evaluate(policy, _pose(1.5), selection, 10.0).signal_state == route_safety.STOP
    assert route_safety.evaluate(policy, _pose(sigma=1.0), selection, 10.0).signal_state == route_safety.STOP
    assert route_safety.evaluate(policy, _pose(stamp=9.0), selection, 10.0).reason_mask & route_safety.REASON_SENSOR_STALE
    manual = _load(_manifest(normal=False))
    assert route_safety.evaluate(manual, _pose(), _selection(manual), 10.0).signal_state == route_safety.STOP


def test_a03_future_tolerance_bounds_each_evidence_age():
    assert route_safety.FUTURE_TOLERANCE_S == A03_FUTURE_TOLERANCE_S
    policy = _load(_manifest())
    selection = _selection(policy)
    cases = (
        (-A03_FUTURE_TOLERANCE_S, True),
        (-A03_FUTURE_TOLERANCE_S - 1e-9, False),
        (0.0, True),
    )
    for field in ("pose_stamp", "status_stamp", "transform_stamp"):
        for age, allowed in cases:
            evaluation = route_safety.evaluate(
                policy, _pose(stamp=0.0, **{field: -age}), selection, 0.0,
            )
            assert evaluation.clear is allowed
            if not allowed:
                assert evaluation.reason_mask == (
                    route_safety.REASON_SENSOR_STALE | route_safety.REASON_GEOFENCE
                )

    for field, ttl_s in (
            ("pose_stamp", policy.pose_ttl_s),
            ("status_stamp", policy.status_ttl_s),
            ("transform_stamp", policy.transform_ttl_s)):
        evaluation = route_safety.evaluate(
            policy, _pose(stamp=0.0, **{field: -ttl_s - 1e-9}), selection, 0.0,
        )
        assert evaluation.signal_state == route_safety.STOP
        assert evaluation.reason_mask == (
            route_safety.REASON_SENSOR_STALE | route_safety.REASON_GEOFENCE
        )


def test_live_shaped_future_transform_evidence_is_accepted_without_age_clamping():
    policy = _load(_manifest())
    evaluation = route_safety.evaluate(
        policy,
        _pose(pose_stamp=9.998, status_stamp=9.997, transform_stamp=10.007),
        _selection(policy),
        10.0,
    )
    assert evaluation.clear
    assert evaluation.transform_age_s == pytest.approx(-0.007)


@pytest.mark.parametrize("field", ("pose_stamp", "status_stamp", "transform_stamp"))
def test_nonfinite_evidence_age_remains_input_unknown(field):
    policy = _load(_manifest())
    evaluation = route_safety.evaluate(
        policy, _pose(**{field: float("nan")}), _selection(policy), 10.0,
    )
    assert evaluation.signal_state == route_safety.STOP
    assert evaluation.reason_mask == (
        route_safety.REASON_INPUT_UNKNOWN | route_safety.REASON_GEOFENCE
    )


def test_untrusted_selection_exclusion_and_mutation_attempts_fail_closed():
    policy = _load(_manifest())
    selection = _selection(policy)
    forged = route_safety.ActiveRouteSelection(
        selection.route_id, "f" * 64, selection.safety_manifest_sha256,
        selection.map_id, selection.map_sha256, selection.segment_id, selection.zone_id,
    )
    assert route_safety.evaluate(policy, _pose(), forged, 10.0).reason_mask & route_safety.REASON_ROUTE_MANIFEST
    assert not hasattr(policy, "reload") and not hasattr(policy, "widen")
    value = _manifest()
    value["global_exclusion_polygons"] = [[[-0.2, -0.2], [-0.2, 0.2], [0.2, 0.2], [0.2, -0.2]]]
    excluded = _load(value)
    assert route_safety.evaluate(excluded, _pose(), _selection(excluded), 10.0).signal_state == route_safety.STOP
