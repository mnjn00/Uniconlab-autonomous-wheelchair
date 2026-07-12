#!/usr/bin/env python3
"""Pure/static tests for the untrusted localization candidate/TF adapter."""

import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

MODULE_PATH = Path(__file__).parents[1] / "scripts" / "localization_adapter.py"
LAUNCH_PATH = Path(__file__).parents[1] / "launch" / "localization.launch"
CMAKE_PATH = Path(__file__).parents[1] / "CMakeLists.txt"
SPEC = importlib.util.spec_from_file_location("localization_adapter", str(MODULE_PATH))
adapter = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = adapter
SPEC.loader.exec_module(adapter)


def identity(**changes):
    values = dict(
        sequence=7,
        stamp_s=10.0,
        receipt_s=10.01,
        reset_count=3,
        source="amcl",
        map_id="deployed-map",
        map_sha256="map-hash",
        policy_sha256="policy-hash",
    )
    values.update(changes)
    return adapter.CandidateIdentity(**values)


def status_fields(**changes):
    values = dict(
        now_s=10.10,
        status_receipt_s=10.10,
        status_stamp_s=10.0,
        status_frame_id="map",
        evaluation_stamp_s=10.08,
        status_sequence=7,
        status_state=2,
        ok_state=2,
        independent_check_passed=True,
        reset_count=3,
        source="amcl",
        map_id="deployed-map",
        map_sha256="map-hash",
        policy_sha256="policy-hash",
    )
    values.update(changes)
    return values


def test_disabled_source_is_explicitly_uninitialized():
    assert adapter.select_native_source("", ()) is None


@pytest.mark.parametrize("source", adapter.VALID_SOURCES)
def test_exactly_one_configured_native_source(source):
    assert adapter.select_native_source(source, (source,)) == source


def test_simultaneous_or_mismatched_sources_are_rejected():
    with pytest.raises(adapter.ConfigurationError):
        adapter.select_native_source("amcl", ("amcl", "base_model"))
    with pytest.raises(adapter.ConfigurationError):
        adapter.select_native_source("amcl", ("base_model",))
    with pytest.raises(adapter.ConfigurationError):
        adapter.select_native_source("glim_bridge", ("glim_bridge",))


def test_adapter_must_be_only_map_to_odom_authority():
    adapter.validate_tf_authority("/localization_adapter", ("/localization_adapter",))
    with pytest.raises(adapter.ConfigurationError):
        adapter.validate_tf_authority(
            "/localization_adapter", ("/localization_adapter", "/amcl")
        )


def test_map_to_odom_planar_transform_round_trip():
    map_base = adapter.Pose2D(4.0, -1.0, math.radians(35.0))
    odom_base = adapter.Pose2D(1.2, 0.4, math.radians(-10.0))
    map_odom = adapter.map_to_odom(map_base, odom_base)
    recovered = adapter.compose(map_odom, odom_base)
    assert recovered.x == pytest.approx(map_base.x)
    assert recovered.y == pytest.approx(map_base.y)
    assert recovered.yaw == pytest.approx(map_base.yaw)
def test_fresh_odom_extends_bounded_tf_validity_horizon():
    class Stamp:
        def __init__(self, value):
            self.value = value

        def to_sec(self):
            return self.value

        def __add__(self, duration):
            return Stamp(self.value + duration)

    node = adapter.LocalizationAdapterNode.__new__(adapter.LocalizationAdapterNode)
    node.odom = None
    node.map_to_odom_estimate = adapter.Pose2D(1.0, 2.0, 0.25)
    node.map_to_odom_receipt_s = 9.90
    node.external_tf_authority = False
    node.status_max_age_s = 0.25
    node.tf_future_tolerance_s = 0.05
    node.last_tf_stamp_s = None
    node.rospy = SimpleNamespace(
        Time=SimpleNamespace(now=lambda: Stamp(10.0)),
        Duration=lambda seconds: seconds,
    )
    sent = []
    node._send_map_to_odom = lambda transform, stamp: sent.append(
        (transform, stamp.to_sec())
    )

    message = SimpleNamespace(header=SimpleNamespace(stamp=Stamp(9.99)))
    node._odom_callback(message)

    assert node.odom is message
    assert sent == [(node.map_to_odom_estimate, pytest.approx(10.04))]


def test_stale_or_duplicate_authority_does_not_extend_tf_horizon():
    node = adapter.LocalizationAdapterNode.__new__(adapter.LocalizationAdapterNode)
    node.odom = None
    node.map_to_odom_estimate = adapter.Pose2D(1.0, 2.0, 0.25)
    node.map_to_odom_receipt_s = 9.70
    node.external_tf_authority = False
    node.status_max_age_s = 0.25
    node.tf_future_tolerance_s = 0.05
    node.last_tf_stamp_s = None
    node.rospy = SimpleNamespace(
        Time=SimpleNamespace(
            now=lambda: SimpleNamespace(to_sec=lambda: 10.0)
        ),
        Duration=lambda seconds: seconds,
    )
    sent = []
    node._send_map_to_odom = lambda transform, stamp: sent.append((transform, stamp))

    node._odom_callback(SimpleNamespace())
    assert sent == []

    node.map_to_odom_receipt_s = 9.90
    node.external_tf_authority = True
    node._odom_callback(SimpleNamespace())
    assert sent == []


def test_tf_validity_horizon_is_bounded_below_a03_limit():
    assert adapter.TF_VALIDITY_HORIZON_S == pytest.approx(0.04)
    assert adapter.TF_FUTURE_TOLERANCE_S == pytest.approx(0.05)



def test_tracker_rejects_future_backward_clock_and_implicit_relocalization():
    tracker = adapter.EvidenceTracker()
    assert tracker.accept(10.0, 9.9, 0)
    assert not tracker.accept(9.9, 9.9, 0)

    tracker = adapter.EvidenceTracker()
    assert not tracker.accept(10.0, 10.1, 0)
    assert tracker.accept(10.0, 9.9, 0)
    assert not tracker.accept(10.1, 9.8, 0)
    assert not tracker.accept(10.1, 10.0, 1)
    tracker.authorize_relocalization()
    assert tracker.accept(10.2, 10.1, 1)


@pytest.mark.parametrize("value", (math.nan, math.inf, -1, 1.5, 2**32))
def test_invalid_reset_counts_fail_closed(value):
    assert adapter.parse_reset_count(value) is None


def test_exact_uint32_reset_count_is_accepted():
    assert adapter.parse_reset_count(0) == 0
    assert adapter.parse_reset_count(2**32 - 1) == 2**32 - 1


def test_exact_fresh_guard_status_allows_tf():
    assert adapter.guard_status_allows_tf(identity(), **status_fields())


@pytest.mark.parametrize(
    "change",
    [
        {"status_sequence": 6},
        {"status_stamp_s": 10.001},
        {"status_frame_id": "odom"},
        {"status_state": 4},
        {"independent_check_passed": False},
        {"reset_count": 2},
        {"source": "base_model"},
        {"map_id": "other-map"},
        {"map_sha256": "other-map-hash"},
        {"policy_sha256": "other-policy-hash"},
        {"external_tf_authority": True},
    ],
)
def test_guard_status_mismatch_fails_closed(change):
    assert not adapter.guard_status_allows_tf(identity(), **status_fields(**change))


@pytest.mark.parametrize(
    "change",
    [
        {"now_s": 10.251, "status_receipt_s": 10.251},
        {"now_s": 10.40, "status_receipt_s": 10.40, "status_stamp_s": 10.20},
        {"now_s": 10.40, "status_receipt_s": 10.40, "evaluation_stamp_s": 10.10},
        {"now_s": 9.99, "status_receipt_s": 9.99},
        {"evaluation_stamp_s": 10.11},
    ],
)
def test_stale_or_future_guard_status_fails_closed(change):
    assert not adapter.guard_status_allows_tf(identity(), **status_fields(**change))


def test_tf_callback_normalizes_self_name_and_latches_external_authority():
    node = adapter.LocalizationAdapterNode.__new__(adapter.LocalizationAdapterNode)
    node.node_name = "/localization_adapter"
    node.external_tf_authority = False
    discarded = []
    node._discard_pending_candidate = lambda: discarded.append(True)
    transform = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"), child_frame_id="odom"
    )

    own = SimpleNamespace(
        transforms=[transform],
        _connection_header={"callerid": " /localization_adapter "},
    )
    node._tf_callback(own)
    assert not node.external_tf_authority
    assert discarded == []

    external = SimpleNamespace(
        transforms=[transform],
        _connection_header={"callerid": "/rogue_localizer"},
    )
    node._tf_callback(external)
    assert node.external_tf_authority
    assert discarded == [True]


def test_adapter_has_candidate_only_publisher_and_guard_status_subscriber():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert source.count('rospy.Publisher("/localization/candidate"') == 1
    assert 'rospy.Publisher("/localization/status"' not in source
    assert 'rospy.Publisher("/safety/localization"' not in source
    assert source.count('rospy.Subscriber("/localization/status"') == 1
    assert "LocalizationGuardCore" not in source
    assert "localization_guard" not in source
    assert source.count("self.tf_broadcaster.sendTransform(output)") == 1
    assert "candidate.pose.header.seq = self.candidate_sequence" in source
    assert 'candidate.pose.header.frame_id = "map"' in source
    pose_callback = source[source.index("    def _pose_callback"):source.index("    def _status_callback")]
    status_callback = source[source.index("    def _status_callback"):source.index("    def _discard_pending_candidate")]
    assert pose_callback.index("self._broadcast_map_to_odom") < pose_callback.index(
        "self.candidate_pub.publish(candidate)"
    )
    assert "self._broadcast_map_to_odom" not in status_callback


def test_launch_and_install_do_not_own_the_independent_guard():
    launch = LAUNCH_PATH.read_text(encoding="utf-8")
    cmake = CMAKE_PATH.read_text(encoding="utf-8")
    assert 'status_max_age_s" default="0.25"' in launch
    assert 'tf_future_tolerance_s" default="0.04"' in launch
    assert "localization_guard" not in launch
    assert "scripts/localization_guard.py" not in cmake


def test_ros_imports_are_lazy():
    assert "rospy" not in adapter.__dict__
