import copy
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
SCRIPT = PACKAGE / "scripts" / "route_manager.py"
CONFIG = PACKAGE / "config" / "hanyang_routes.yaml"
SOURCE = ROOT / "data" / "hanyang_aegimun_loop" / "hanyang_aegimun_loop.waypoints.yaml"
MAP_PGM = ROOT / "data" / "hanyang_aegimun_loop" / "map.pgm"
MAP_YAML = ROOT / "data" / "hanyang_aegimun_loop" / "map.yaml"
SAFETY = ROOT / "contracts" / "wp0" / "route-safety-candidate.yaml"
SPEC = importlib.util.spec_from_file_location("route_manager", SCRIPT)
route_manager = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = route_manager
SPEC.loader.exec_module(route_manager)


def _raw():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _hash(value, omitted):
    payload = {key: item for key, item in value.items() if key != omitted}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()).hexdigest()


def _route_hash(route, map_value):
    payload = {
        "map_id": map_value["map_id"],
        "map_sha256": map_value["sha256"],
        "route": {key: item for key, item in route.items() if key != "route_manifest_sha256"},
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()).hexdigest()


def _seal(value):
    for direction in ("outbound", "return"):
        route = value[direction + "_route"]
        route["route_manifest_sha256"] = _route_hash(route, value["map"])
    value["content_sha256"] = _hash(value, "content_sha256")


def _write(tmp_path, value):
    path = tmp_path / "routes.yaml"
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    return path


def test_committed_manifest_dynamically_binds_map_route_and_safety_bytes():
    raw = _raw()
    manifest = route_manager.load_manifest(str(CONFIG))
    assert manifest.map_sha256 == manifest.map_pgm_sha256 == _sha(MAP_PGM)
    assert raw["map"]["yaml_sha256"] == _sha(MAP_YAML)
    assert manifest.waypoint_asset_sha256 == raw["provenance"]["source_sha256"] == _sha(SOURCE)
    assert manifest.safety_manifest_sha256 == _sha(SAFETY)
    assert raw["provenance"] == {
        "source_path": "data/hanyang_aegimun_loop/hanyang_aegimun_loop.waypoints.yaml",
        "source_sha256": _sha(SOURCE),
        "evidence_level": "simulation_only",
        "surveyed": False,
    }
    for direction in ("outbound", "return"):
        route = raw[direction + "_route"]
        assert manifest.route(direction).route_manifest_sha256 == _route_hash(route, raw["map"])


def test_both_explicit_directions_preserve_all_732_recorded_waypoints():
    source = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))
    manifest = route_manager.load_manifest(str(CONFIG))
    routes = [manifest.route("outbound"), manifest.route("return")]
    source_routes = [source["outbound_route"], source["return_route"]]
    assert [route.direction for route in routes] == ["outbound", "return"]
    assert [len(route.waypoints) for route in routes] == [359, 373]
    assert sum(len(route.waypoints) for route in routes) == 732
    for route, expected in zip(routes, source_routes):
        assert len(route.segments) == len(route.waypoints) - 1
        assert [(w.x_m, w.y_m, w.yaw_rad) for w in route.waypoints] == [
            (w["x_m"], w["y_m"], w["yaw_rad"]) for w in expected["waypoints"]
        ]
        assert route.waypoints[-1].behavior == route.segments[-1].behavior == "terminal_stop"
        for current, following in zip(route.waypoints, route.waypoints[1:]):
            assert current.yaw_rad == pytest.approx(math.atan2(following.y_m - current.y_m, following.x_m - current.x_m), abs=1e-6)
        previous, terminal = route.waypoints[-2:]
        assert terminal.yaw_rad == pytest.approx(math.atan2(terminal.y_m - previous.y_m, terminal.x_m - previous.x_m), abs=1e-6)
    assert routes[0].waypoints[-1].x_m == routes[1].waypoints[0].x_m
    assert routes[0].waypoints[-1].y_m == routes[1].waypoints[0].y_m
    assert [w.waypoint_id for w in routes[1].waypoints] != list(reversed([w.waypoint_id for w in routes[0].waypoints]))


def test_every_segment_is_simulation_only_with_conservative_nonzero_caps():
    raw = _raw()
    assert raw["provenance"]["evidence_level"] == "simulation_only"
    for direction in ("outbound", "return"):
        route = raw[direction + "_route"]
        assert route["qualification"] == "candidate"
        assert route["direction"] == direction
        for segment in route["segments"]:
            assert segment["hardware_authorized"] is False
            assert 0.0 < segment["max_linear_mps"] <= 0.35
            assert 0.0 < segment["max_angular_rps"] <= 0.6
            assert segment["corridor_margin_m"] == 0.2
            assert segment["corridor_width_m"] == 0.4
            assert segment["zone_ids"] == ["candidate-unsurveyed"]


@pytest.mark.parametrize("mutation", [
    lambda value: value.update({"unexpected": True}),
    lambda value: value["outbound_route"].update({"direction": "return"}),
    lambda value: value["outbound_route"]["segments"][0].update({"end_waypoint_index": 2}),
    lambda value: value["outbound_route"]["waypoints"][-1].update({"behavior": "proceed"}),
    lambda value: value["outbound_route"]["waypoints"][0].update({"yaw_rad": float("nan")}),
    lambda value: value["outbound_route"]["segments"][0].update({"hardware_authorized": True}),
    lambda value: value["outbound_route"]["waypoints"][1].update({
        "x_m": value["outbound_route"]["waypoints"][0]["x_m"],
        "y_m": value["outbound_route"]["waypoints"][0]["y_m"],
    }),
])
def test_invalid_or_widened_routes_are_rejected_even_with_resealed_hash(tmp_path, mutation):
    value = copy.deepcopy(_raw())
    mutation(value)
    _seal(value)
    with pytest.raises(route_manager.RouteValidationError):
        route_manager.load_manifest(str(_write(tmp_path, value)), verify_assets=False)

def test_directional_semantic_hashes_are_distinct_and_map_bound(tmp_path):
    value = copy.deepcopy(_raw())
    outbound, returning = value["outbound_route"], value["return_route"]
    assert outbound["route_manifest_sha256"] != returning["route_manifest_sha256"]
    value["map"]["map_id"] = "other-map"
    value["content_sha256"] = _hash(value, "content_sha256")
    with pytest.raises(route_manager.RouteValidationError, match="semantic hash mismatch"):
        route_manager.load_manifest(str(_write(tmp_path, value)), verify_assets=False)


def test_resealed_self_intersection_is_rejected(tmp_path):
    value = copy.deepcopy(_raw())
    points = ((0.0, 0.0, math.pi / 4.0), (2.0, 2.0, math.pi),
              (0.0, 2.0, -math.pi / 4.0), (2.0, 0.0, 0.0))
    for waypoint, (x_m, y_m, yaw_rad) in zip(value["outbound_route"]["waypoints"][:4], points):
        waypoint.update(x_m=x_m, y_m=y_m, yaw_rad=yaw_rad)
    _seal(value)
    with pytest.raises(route_manager.RouteValidationError, match="self-intersects"):
        route_manager.load_manifest(str(_write(tmp_path, value)), verify_assets=False)


@pytest.mark.parametrize("mutation", [
    lambda value: value["outbound_route"]["waypoints"][1].update({"behavior": "terminal_stop"}),
    lambda value: value["outbound_route"]["segments"][0].update({"corridor_width_m": 0.5}),
    lambda value: value["outbound_route"]["segments"][0].update({"max_linear_mps": 0.0}),
    lambda value: value["outbound_route"].update({"qualification": "simulation_qualified"}),
])
def test_resealed_policy_and_terminal_failures_are_rejected(tmp_path, mutation):
    value = copy.deepcopy(_raw())
    mutation(value)
    _seal(value)
    with pytest.raises(route_manager.RouteValidationError):
        route_manager.load_manifest(str(_write(tmp_path, value)), verify_assets=False)


def test_content_hash_mismatch_is_rejected(tmp_path):
    value = copy.deepcopy(_raw())
    value["outbound_route"]["waypoints"][0]["x_m"] += 0.01
    with pytest.raises(route_manager.RouteValidationError, match="content hash mismatch"):
        route_manager.load_manifest(str(_write(tmp_path, value)), verify_assets=False)


def test_bound_asset_hash_mismatch_is_rejected(tmp_path):
    value = copy.deepcopy(_raw())
    asset = tmp_path / "map.yaml"
    asset.write_text("changed", encoding="utf-8")
    value["map"]["yaml_path"] = "map.yaml"
    _seal(value)
    with pytest.raises(route_manager.RouteValidationError, match="map YAML asset hash mismatch"):
        route_manager.load_manifest(str(_write(tmp_path, value)), verify_assets=True)


def test_progress_tracks_dense_outbound_and_fails_closed_when_lost():
    route = route_manager.load_manifest(str(CONFIG)).route("outbound")
    tracker = route_manager.ProgressTracker(route)
    start, next_waypoint = route.waypoints[:2]
    first = tracker.update(start.x_m, start.y_m)
    second = tracker.update(next_waypoint.x_m, next_waypoint.y_m)
    assert first.state == second.state == "ACTIVE"
    assert second.waypoint_index >= 1
    assert second.along_track_m >= first.along_track_m
    assert second.cross_track_error_m == pytest.approx(0.0)
    assert second.distance_remaining_m < first.distance_remaining_m
    lost = tracker.update(start.x_m + 1000.0, start.y_m + 1000.0)
    assert lost == route_manager.Progress("INACTIVE", fault="CROSS_TRACK_LOST")


def _route_manager_node_for_test(manifest):
    node = route_manager.RouteManagerNode.__new__(route_manager.RouteManagerNode)
    node._manifest = manifest
    node._tracker = None
    node._mission_id = ""
    node._activation_sequence = None
    node._activation_identity = None
    node._rospy = SimpleNamespace(logerr=lambda *unused: None)
    published = []
    node._publish = lambda progress, route_id="": published.append((progress, route_id))
    return node, published


def _active_route_message(manifest, sequence=1, mission_id="mission-1"):
    route = manifest.route("outbound")
    return SimpleNamespace(
        DIRECTION_OUTBOUND=1,
        DIRECTION_RETURN=2,
        activation_sequence=sequence,
        direction=1,
        mission_id=mission_id,
        route_id=route.route_id,
        map_id=manifest.map_id,
        map_sha256=manifest.map_sha256,
        route_manifest_sha256=route.route_manifest_sha256,
        safety_manifest_sha256=manifest.safety_manifest_sha256,
    )


def test_active_route_heartbeat_preserves_progress_tracker_state():
    manifest = route_manager.load_manifest(str(CONFIG))
    node, published = _route_manager_node_for_test(manifest)
    message = _active_route_message(manifest)

    node._active_callback(message)
    tracker = node._tracker
    start = manifest.route("outbound").waypoints[0]
    assert tracker.update(start.x_m, start.y_m).state == "ACTIVE"
    assert node._activation_identity[5] == manifest.route("outbound").route_manifest_sha256

    node._active_callback(copy.copy(message))

    assert node._tracker is tracker
    assert len(published) == 1
    assert published[0][0].state == "INACTIVE"


@pytest.mark.parametrize("change", [
    lambda message: setattr(message, "mission_id", "changed"),
    lambda message: setattr(message, "activation_sequence", 0),
])
def test_replayed_or_mutated_route_activation_fails_closed(change):
    manifest = route_manager.load_manifest(str(CONFIG))
    node, published = _route_manager_node_for_test(manifest)
    message = _active_route_message(manifest, sequence=2)
    node._active_callback(message)

    replay = copy.copy(message)
    change(replay)
    node._active_callback(replay)

    assert node._tracker is None
    assert published[-1][0].state == "INVALID"


def test_ros_is_lazy_and_adapter_has_no_motion_or_geofence_authority():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "import rospy" not in "\n".join(text.splitlines()[:40])
    assert 'Publisher("/route/progress"' in text
    assert 'Publisher("/safety/geofence"' not in text
    assert 'Publisher("/cmd_vel' not in text
    assert "runtime_generation_allowed: false" in CONFIG.read_text(encoding="utf-8")
